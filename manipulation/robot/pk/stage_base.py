"""Shared parts of every test stage: server connection, robot model, plate from perception, fingertip force and touch,
parameters, event log, and a path player that checks abort conditions every cycle.
"""
import json
import math
import os
import time

import numpy as np

from . import config
from .fit import external_force
from .model import PINKY_END, PINKY_LOW, Robot
from .stance import pelvis_height


class Abort(RuntimeError):
    pass


def smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def load_params(name, overrides=None):
    path = os.path.join(config.PARAMS_DIR, name + ".json")
    p = json.load(open(path)) if os.path.exists(path) else {}
    p = {k: v for k, v in p.items() if not k.startswith("_")}
    for k, v in (overrides or {}).items():
        p[k] = v
    return p


class Ctx(object):
    def __init__(self, client, out_dir, overrides=None, dry=False):
        self.c = client
        self.rob = Robot()
        self.dry = dry
        try:
            self.gfit = json.load(open(os.path.join(config.MODEL_DIR, "gravity_fit.json")))
        except (OSError, ValueError):
            self.gfit = dict(scale=[1.0] * config.N_ARM, bias_rad=[0.0] * config.N_ARM)
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.events = open(os.path.join(out_dir, "events.jsonl"), "a")
        self.overrides = overrides or {}
        self.results = {}
        self.t0 = time.time()
        self.memory = {}                         # what one stage leaves for the next (push end, plate offset,...)

    # ------------------------------------------------------------------ log
    def log(self, what, **data):
        """events.jsonl + one printed line. A log line never stops a test."""
        t = round(time.time() - self.t0, 3)
        try:
            rec = dict(t=t, what=what, **{k: _js(v) for k, v in data.items()})
            self.events.write(json.dumps(rec, default=str) + "\n")
            self.events.flush()
        except (TypeError, ValueError, OSError) as ex:
            print("(log: could not save '%s': %r)" % (what, ex), flush=True)
        try:
            extra = ", ".join("%s %s" % (k, _short(v)) for k, v in data.items())
        except Exception as ex:                                     # noqa: BLE001
            extra = "(not printable: %r)" % ex
        print("[%6.1f s] %s%s" % (t, what, (": " + extra) if extra else ""), flush=True)

    def params(self, name):
        ov = {k.split(".", 1)[1]: v for k, v in self.overrides.items() if k.startswith(name + ".")}
        return load_params(name, ov)

    # ------------------------------------------------------------------ state
    def st(self):
        return self.c.state()

    def plate(self, st=None, max_age=0.5):
        st = st or self.st()
        p = st.get("plate")
        if not isinstance(p, dict) or st.get("plate_age") is None or st["plate_age"] > max_age:
            return None
        return p

    def wait_plate(self, timeout=20.0, need_located_after=None):
        """The newest good plate estimate (perception tracking). need_located_after: a time.time() that the
        located stack must be newer than (after asking perception to locate again)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            p = self.plate()
            if p and p.get("ok") and p.get("located") and (need_located_after is None or p["located"]["t"] > need_located_after):
                return p
            self.c.hold(0.05)
        raise Abort("no plate from perception within %.0f s (is perception.py running and tracking?)" % timeout)

    def pelvis(self, st=None):
        st = st or self.st()
        h, _ = pelvis_height(st.get("legs", [0] * 12), st.get("rpy", [0, 0, 0]))
        return h

    # ------------------------------------------------------------------ kinematics and forces
    def fk(self, q=None, h=None, st=None):
        if q is None or h is None:
            st = st or self.st()
        q = np.array(st["q"]) if q is None else np.asarray(q)
        h = np.array(st["hands"]) if h is None else np.asarray(h)
        return self.rob.fk(q, h)

    def pinky(self, T, side="left"):
        """(end point, lowest z of the last 12 mm) of a pinky, pelvis frame."""
        M = T["%s_pinky_distal_link" % side]
        e = M[:3, :3] @ PINKY_END[side] + M[:3, 3]
        low = float((PINKY_LOW[side] @ M[:3, :3].T + M[:3, 3])[:, 2].min()) if len(PINKY_LOW[side]) else float(e[2])
        return e, low

    def force(self, st, side="left", link=None, p_local=None):
        """Force of the world on a fingertip point (N, pelvis frame), from the arm's joint lag minus gravity."""
        link = link or "%s_pinky_distal_link" % side
        p_local = PINKY_END[side] if p_local is None else p_local
        F, _ = external_force(self.rob, np.array(st["q"]), np.array(st["hands"]), np.array(st["sent"]), np.array(st["dq"]),
                              side, link, p_local, self.gfit, np.array(st["kp"]), np.array(st["kd"]))
        return F

    @staticmethod
    def touch(st, side="left", finger="Pinky"):
        """Normal-force counts of a fingertip pad (rt/brainco/<side>/touch), or None when the hand server sends none."""
        t = st.get("touch")
        if not t:
            return None
        return float(t[0 if side == "left" else 1][config.TOUCH_FINGERS.index(finger)][0])

    # ------------------------------------------------------------------ motion
    def play(self, q_path, h_path, dt, speed=1.0, check=None, tags=None, hand_rule=None, vmax=None, payload=None):
        """Stream a path (N x 11 arm targets, N x 12 hand targets, sample spacing dt seconds of reference time) at
        `speed` x the reference's pace. check(state, k) -> reason string aborts. hand_rule(state, k, h_target) -> the
        hand command to send (e.g. the closing margin). Returns (reason or "", last index)."""
        n = len(q_path)
        # never ask the server for more than it allows: the playback slows down where the path is fast
        v_allow = 0.9 * (vmax or config.V_MAX)
        v_path = float(np.max(np.abs(np.diff(q_path, axis=0)))) / max(dt, 1e-6) * speed if n > 1 else 0.0
        if v_path > v_allow:
            self.log("play", note="path too fast for the speed limit: playing slower", asked=speed, playing=speed * v_allow / v_path)
            speed = speed * v_allow / v_path
        T_total = (n - 1) * dt / max(speed, 1e-3)
        t0 = time.time()
        self.c.set(vmax=vmax or config.V_MAX)
        self.c._denied = None
        last_tag = None
        k = 0
        while True:
            u = (time.time() - t0) / max(T_total, 1e-6)
            x = min(u, 1.0) * (n - 1)
            k = int(math.floor(x))
            f = x - k
            k2 = min(k + 1, n - 1)
            q = q_path[k] + f * (q_path[k2] - q_path[k])
            h = h_path[k] + f * (h_path[k2] - h_path[k])
            st = self.st()
            if hand_rule is not None:
                h = hand_rule(st, k, h)
            tag = tags[k] if tags is not None else None
            if tag != last_tag:
                last_tag = tag
                self.log("phase", name=tag)
            self.c.set(q=q, hands=h, tag=tag)
            if self.c._denied:
                return "the server refused a target: %s" % self.c._denied, k
            lags = np.abs(np.array(st["des"]) - q)[:10]
            lag = float(np.max(lags))
            if lag > 0.15:
                return "the server is not following the path (%s %.3f rad behind, server mode %s%s, step %d/%d)" % (
                    config.ARM_NAMES[int(np.argmax(lags))], lag, st.get("mode"), (": " + st["freeze"]) if st.get("freeze") else "", k, n), k
            if check is not None:
                why = check(st, k)
                if why:
                    return why, k
            if u >= 1.0:
                return "", n - 1
            time.sleep(0.01)

    def settle(self, seconds, check=None):
        return self.c.hold(seconds, check)


def _js(v):
    if isinstance(v, np.ndarray):
        return [round(float(x), 5) for x in v.ravel()]
    if isinstance(v, (np.floating, float)):
        return round(float(v), 5)
    if isinstance(v, (list, tuple)):
        return [_js(x) for x in v]
    if isinstance(v, dict):
        return {k: _js(x) for k, x in v.items()}
    if isinstance(v, (np.integer,)):
        return int(v)
    return v


def _short(v):
    """One printed log value. Numbers as numbers; anything else as text."""
    if isinstance(v, (np.ndarray, list, tuple)):
        try:
            a = np.asarray(v, dtype=float).ravel()
        except (TypeError, ValueError):
            return "[" + " ".join(str(x) for x in list(v)[:6]) + (" ..]" if len(v) > 6 else "]")
        return "[" + " ".join("%.4g" % x for x in a[:6]) + (" ..]" if len(a) > 6 else "]")
    if isinstance(v, float):
        return "%.4g" % v
    return str(v)
