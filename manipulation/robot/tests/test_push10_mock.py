"""Offline run of push10 against the simulated arm and a simple stack (logic only, spring contacts).

    python3 tests/test_push10_mock.py                     # full push10
    python3 tests/test_push10_mock.py --mode edge --cam-xy-err 0 -12
    python3 tests/test_push10_mock.py --mode air          # the reference path in free air
    python3 tests/test_push10_mock.py --mode shape --set push10.shape_hold_s=2
    python3 tests/test_push10_mock.py --no-touch          # no pad data (the stall path alone)
"""
import argparse
import csv
import json
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from mock_world import C, K, FakePerception, MockStack  # noqa: E402
from pk import config  # noqa: E402
from pk.client import Client, operator  # noqa: E402
from pk.model import Robot  # noqa: E402
from pk.plate import TASK  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.push10 import PAD_POINT, Push10  # noqa: E402

PORT = 9723


class SideStack(MockStack):
    """The top plate for push10: the index pad on its profile, the curled middle against its outer edge."""

    def __init__(self, center, rim_z, slide_N=4.0, knuckle_r=0.003, tip_out=0.009, mu_pad=0.25, ride_over_mm=None, **kw):
        """tip_out: the index stays on the rim until its pad centre is this far past the edge.
        mu_pad: the index pad drags the top plate when mu_pad x its press exceeds slide_N (0 = no drag).
        ride_over_mm: the hook rides over the rim once the plate has slid this far, and catches again outside the rim.
        """
        MockStack.__init__(self, center=center, rim_z=rim_z, slide_N=slide_N, profile=True, finger_r=knuckle_r, **kw)
        self.tip_out = tip_out
        self.peak = {}
        self.mu_pad = mu_pad
        self.ride_over_mm = ride_over_mm
        self.riding = False                     # the hook is passing over the rim
        self.rode = False                       # it did, once
        self.last = {}
        self.drag_mm = 0.0                      # how far the index pad has dragged the plate, total
        self.moves = []                         # the first plate moves: (time, point, p mm, dc mm, how)

    def contact(self, p, v, name="index"):
        F = np.zeros(3)
        R = self.R
        with self.lock:
            d = p[:2] - self.c
            r = float(np.linalg.norm(d))
            u = d / max(r, 1e-9)
            slid = float(np.linalg.norm(self.c - self.c0)) * 1000
            drop = float(np.interp(slid, [0.0, 8.0, 45.0, 80.0], [0.0, 0.0, 4.5, 4.6])) / 1000   # the pushed edge sinks
            top = self.rim_z - drop
            base = top - TASK.H
            if self.ride_over_mm is not None and not self.rode and name == "middle" and slid > self.ride_over_mm:
                self.riding = self.rode = True
            if self.riding and name == "middle" and (r > R + self.fr + 0.002 or p[2] > top + 0.003):
                self.riding = False                                     # out beyond the rim or above it: it catches again
            wall = (name == "middle" and not self.riding and top - TASK.rim_t - 0.001 < p[2] < top + 0.0005
                    and R - 0.004 < r < R + self.fr)
            if wall:                                                    # the hook meets the outer edge: a wall that slides
                pen = R + self.fr - r
                if K * pen > self.slide_N:
                    move = pen - self.slide_N / K
                    self.c -= u * move
                    pen = self.slide_N / K
                F[:2] += u * max(0.0, K * pen - C * float(v[:2] @ u))
            elif r < R or (name == "index" and r - self.tip_out < R and p[2] > top - TASK.rim_t):
                # on the top plate's top; the index's fingertip overhangs its pad by tip_out, but only near the rim's top (a
                # finger that went down beside the rim is not on it)
                zs = base + TASK.z_top(min(r, R))
                if p[2] < zs:
                    F[2] += max(0.0, K * (zs - p[2]) - C * float(v[2]))
                    if name == "index" and self.mu_pad * F[2] > self.slide_N and name in self.last:
                        dp = p[:2] - self.last[name]                    # the pad sticks: the plate goes with it
                        self.c += dp
                        self.drag_mm += 1000 * float(np.linalg.norm(dp))
            else:                                                       # beyond it: the plate below (never moves)
                r0 = float(np.linalg.norm(p[:2] - self.c0))
                if r0 < R:
                    zb = self.rim_z - self.pitch - TASK.H + TASK.z_top(r0)
                    if p[2] < zb:
                        F[2] += max(0.0, K * (zb - p[2]) - C * float(v[2]))
                        self.peak["%s-lower" % name] = max(self.peak.get("%s-lower" % name, 0.0), float(F[2]))
            self.last[name] = p[:2].copy()
            if self.log and np.any(self.c != self.log[-1][4]) and len(self.moves) < 12:
                self.moves.append((time.time(), name, np.round(1000 * p, 1).tolist(), np.round(1000 * (self.c - self.log[-1][4]), 2).tolist(),
                                   "wall" if wall else "top/drag"))
            self.log.append((time.time(), name, p.copy(), F.copy(), self.c.copy()))
        return F


def knuckle_point(rob, hand_cmd, q_plan):
    """The curled middle's lowest mesh point at the push pose, in its link (left_middle_proximal / distal_link)."""
    q = np.zeros(config.N_ARM)
    q[config.LEFT] = q_plan
    h = np.zeros(12)
    h[:6] = hand_cmd
    T = rob.fk(q, h)
    z = np.load(os.path.join(config.MODEL_DIR, "hand_points.npz"))
    best = None
    for ln in ("left_middle_proximal_link", "left_middle_distal_link"):
        if ln not in z.files:
            continue
        loc = z[ln].astype(float)
        w = loc @ T[ln][:3, :3].T + T[ln][:3, 3]
        k = int(np.argmin(w[:, 2]))
        if best is None or w[k, 2] < best[2]:
            best = (ln, loc[k], w[k, 2])
    return best[0], best[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="full", choices=["full", "edge", "air", "shape"])
    ap.add_argument("--table", type=float, default=0.747)
    ap.add_argument("--rim-err", type=float, default=0.0, help="the camera's rim height error, mm (+ = reads too high)")
    ap.add_argument("--cam-xy-err", type=float, nargs=2, default=(0.0, 0.0), help="the camera's plate centre error, mm (x y)")
    ap.add_argument("--stack-xy", type=float, nargs=2, default=(0.36, 0.06), help="true stack centre, m")
    ap.add_argument("--friction", type=float, default=0.3)
    ap.add_argument("--grav-scale", type=float, default=1.2)
    ap.add_argument("--slide-N", type=float, default=4.0, help="push needed to slide the top plate")
    ap.add_argument("--no-touch", action="store_true", help="no fingertip touch data")
    ap.add_argument("--mu-pad", type=float, default=0.25, help="index pad friction on the plate (drags it); 0 = none")
    ap.add_argument("--slip-first", type=float, default=None, help="the hook rides over the rim once the plate slid this many mm")
    ap.add_argument("--set", action="append", default=[], help="push10.key=value (JSON)")
    ap.add_argument("--expect-fail", action="store_true", help="the push must fail cleanly (e.g. --slide-N 80): abort, lift off, home")
    a = ap.parse_args()
    # the top plate's rim top at the edge: the table, 3 plates (pitch 6.8 mm), pelvis 0.71 as the simulation
    rim_z = a.table - config.PELVIS_SIM + 2 * 0.0068 + TASK.H
    stack = SideStack(center=a.stack_xy, rim_z=rim_z, slide_N=a.slide_N, mu_pad=a.mu_pad, ride_over_mm=a.slip_first)
    io = MockBackend(grav_scale=a.grav_scale, friction=a.friction)
    io.touch_alive = not a.no_touch
    P10 = json.load(open(os.path.join(config.PARAMS_DIR, "push10.json")))
    pose = P10["pose_by_table"]["%.3f" % a.table]
    ln, loc = knuckle_point(io.rob, pose["hand_cmd"], pose["q_plan_rad"])
    srv = Server(io, port=PORT, verbose=False, log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
    run = dict(stop=False)

    def loop():
        nxt = time.time()
        while not run["stop"]:
            srv.step()
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))
    threading.Thread(target=loop, daemon=True).start()
    time.sleep(0.3)
    operator("take", port=PORT)
    while srv.mode != "HELD":
        time.sleep(0.05)
    operator("home", port=PORT)
    while srv.mode != "HOME":
        time.sleep(0.05)
    # the stack's contacts from HOME on (the mock arm starts at zero joints; its way to HOME is not a path the R1 takes with
    # plates in front of it: `take` / `home` are done with the table away)
    if a.mode != "air":
        io.contact = stack.contact
        io.contact_pts = [("index", "left_index_touch_link", PAD_POINT, None if a.no_touch else "Index"), ("middle", ln, loc, None)]
        io.proximity = stack.proximity
    per = FakePerception(stack, port=PORT, rim_err_mm=a.rim_err, xy_err_mm=a.cam_xy_err)
    time.sleep(0.5)
    c = Client("test push10 mock", port=PORT)
    out = os.path.join(os.path.dirname(HERE), "logs", "test", "push10_%s_%s" % (a.mode, time.strftime("%H%M%S")))
    overrides = {"push10.table_m": a.table}
    for kv in a.set:
        k, v = kv.split("=", 1)
        try:
            overrides[k.strip()] = json.loads(v)
        except ValueError:
            overrides[k.strip()] = v                         # a bare string, e.g. push10.edge_mode=slide
    with c.busy(20):
        ctx = Ctx(c, out, overrides=overrides)
    ok, res = True, {}
    t_start = time.time()
    try:
        res = Push10(ctx, a.mode).run()
        print("== push10 %s: %s" % (a.mode, {k: v for k, v in res.items() if k != "events"}))
    except Abort as ex:
        ok = False
        res = ctx.results.get("push10", {})
        print("ABORT: %s" % ex)
    c.bye()
    t0 = time.time()
    while srv.mode != "HOME" and time.time() - t0 < 120:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    run["stop"] = True
    per.stop = True
    time.sleep(0.1)
    srv.close()
    moved = 1000 * float(np.linalg.norm(stack.c - stack.c0))
    # the plate must only ever go -y (pushed); +y = dragged out by the edge search or pulled back by a retract
    cyl = np.array([x[4][1] for x in stack.log]) if stack.log else np.array([stack.c0[1]])
    back = 1000 * float(np.max(cyl - np.minimum.accumulate(cyl)))
    out_mm = 1000 * float(np.max(cyl) - stack.c0[1])
    checks = []
    if a.mode in ("full", "edge"):
        checks.append(("plate never dragged out (+y) past its start: %.1f mm" % out_mm, out_mm < 1.0))
        checks.append(("plate never pulled back (+y) after a push: %.1f mm" % back, back < 1.0))
        print("index pad dragged the plate %.1f mm in total (mock friction mu %.2f)" % (stack.drag_mm, a.mu_pad))
        try:                                                    # where the plate moved, by the stage's phase (csv time ~ t_start)
            rows = list(csv.DictReader(open(os.path.join(out, "push10_cmd.csv"))))
            tt = np.array([float(r["t"]) for r in rows])
            phs = [r["phase"] for r in rows]
            per = {}
            prev = stack.c0.copy()
            for x in stack.log:
                k = int(np.clip(np.searchsorted(tt, x[0] - t_start), 0, len(phs) - 1))
                d = x[4] - prev
                prev = x[4].copy()
                if np.any(d != 0):
                    per[phs[k]] = per.get(phs[k], np.zeros(2)) + 1000 * d
            print("plate moved per phase (dx, dy mm): %s" % {k: [round(float(v[0]), 1), round(float(v[1]), 1)] for k, v in per.items()})
            for m in stack.moves:
                k = int(np.clip(np.searchsorted(tt, m[0] - t_start), 0, len(phs) - 1))
                print("  plate move: t %.2f phase %s by %s at %s mm, dc %s mm (%s)" % (m[0] - t_start, phs[k], m[1], m[2], m[3], m[4]))
        except Exception as ex:                                 # noqa: BLE001
            print("(no per-phase motion: %r)" % ex)
    if a.mode == "full" and a.expect_fail:
        checks.append(("the push failed and said so (%s)" % res.get("result"), not ok))
        checks.append(("plate moved only %.1f mm" % moved, moved < P10["stop"]))
        ok = True
    elif a.mode == "full":
        checks.append(("result pushed (%s)" % res.get("result"), res.get("result") == "pushed"))
        if a.slip_first is not None:
            n_slip = len([e for e in res.get("events", []) if e["ev"] == "slip"])
            checks.append(("the hook rode over once: %d slip, %d retries" % (n_slip, res.get("retries", 0)), n_slip >= 1))
            low = [e for e in res.get("events", []) if e["ev"] == "tap" and e.get("result") == "plate below"]
            print("taps on the plate below (rejected): %d" % len(low))
        checks.append(("plate moved %.1f mm (camera stop at %.0f)" % (moved, P10["stop"]), P10["stop"] - 3 <= moved <= P10["stop"] + 25))
        checks.append(("plate moved along -y (dx %.1f, dy %.1f mm)" % tuple(1000 * (stack.c - stack.c0)), stack.c[1] < stack.c0[1]))
    if a.mode == "edge":
        ev = [e for e in res.get("events", []) if e["ev"] == "edge"]
        if ev:
            # the found edge vs the true one (the camera's edge + its error)
            err = res.get("edge_vs_camera_mm", 0.0) + a.cam_xy_err[1]
            checks.append(("edge found %.1f mm from the true edge (camera %+.0f mm off)" % (err, a.cam_xy_err[1]), abs(err) < 10.0))
        else:
            checks.append(("edge found", False))
        checks.append(("plate not pushed by the edge search (%.1f mm)" % moved, moved < 3.0))
    if a.mode in ("air", "shape"):
        checks.append(("ran: %s" % res.get("result"), ok))
    checks.append(("arms back home after the test", home))
    for what, good in checks:
        print(("PASS " if good else "FAIL ") + what)
    if stack.peak:
        print("peak force on the plate below (mock springs): %s" % {k: round(v, 1) for k, v in stack.peak.items()})
    try:
        plot(os.path.join(out, "push10_cmd.csv"), stack, rim_z, os.path.join(os.path.dirname(HERE), "logs", "test_push10_mock_%s.png" % a.mode),
             t_start, a)
    except Exception as ex:                                     # noqa: BLE001
        print("(no figure: %r)" % ex)
    return 0 if ok and all(g for _, g in checks) else 1


def plot(path, stack, rim_z, png, t_start, a):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return
    g = lambda k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows])   # noqa: E731
    t = g("t")
    ph = [r["phase"] for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    ax[0].plot(t, 1000 * g("idx_fk_z"), label="index pad z (FK, measured)")
    ax[0].plot(t, 1000 * g("tgt_z"), "--", lw=0.8, label="target z")
    if a.mode == "air":
        lift = json.load(open(os.path.join(config.PARAMS_DIR, "push10.json")))["air_lift_mm"]
        ax[0].plot(t, 1000 * g("sim_idx_fk_z") + lift, ":", label="the simulated arm's index z (MuJoCo, +%.0f mm)" % lift)
        ax[1].plot(t, 1000 * g("sim_idx_fk_y"), ":", label="the simulated arm's index y (MuJoCo)")
    else:
        ax[0].axhline(1000 * rim_z, color="k", ls=":", lw=0.8, label="true rim top at the edge")
    ax[0].set_ylabel("z mm (pelvis)")
    ax[1].plot(t, 1000 * g("idx_fk_y"), label="index pad y (FK)")
    ax[1].plot(t, 1000 * g("tgt_y"), "--", lw=0.8, label="target y")
    if a.mode != "air":
        tl = np.array([x[0] for x in stack.log]) - t_start
        cl = np.array([x[4] for x in stack.log])
        k = np.argsort(tl)[::20]
        k = k[(tl[k] >= t[0]) & (tl[k] <= t[-1])]
        ax[1].plot(tl[k], 1000 * (cl[k, 1] + stack.R), label="true plate's robot-left edge y")
        ax[1].plot(t, 1000 * (g("cam_y") + stack.R), ":", label="camera's edge y")
    ax[1].set_ylabel("y mm")
    ax[2].plot(t, g("lag_Fx"), label="lag Fx")
    ax[2].plot(t, g("lag_Fy"), label="lag Fy")
    ax[2].plot(t, g("lag_Fz"), label="lag Fz")
    ax[2].plot(t, g("idx_pad_N"), label="index pad N")
    ax[2].set_ylabel("N (above baseline)")
    ax[3].plot(t, np.max(np.abs(np.array([g("q_cmd%d" % i) - g("q_meas%d" % i) for i in range(5)])), axis=0), label="max |q_cmd - q_meas|")
    ax[3].set_ylabel("rad")
    ax[3].set_xlabel("s")
    for x in ax:
        x.legend(fontsize=7, loc="upper right")
        x.grid(alpha=0.3)
        last = None
        for ti, p in zip(t, ph):
            if p != last:
                x.axvline(ti, color="gray", lw=0.5, alpha=0.5)
                if x is ax[0]:
                    x.text(ti, x.get_ylim()[1], p, fontsize=6, rotation=90, va="top")
                last = p
    fig.suptitle("push10 on the SIMULATED ARM (MockBackend + spring contacts, not MuJoCo): mode %s, table %.3f, camera err %s mm" % (
        a.mode, a.table, list(a.cam_xy_err)))
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print("figure: %s" % png)


if __name__ == "__main__":
    sys.exit(main())
