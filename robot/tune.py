#!/usr/bin/env python3
"""Arm accuracy tools: gravity refit, pose checks, joint steps and reference paths in free air.
Needs server.py at the pre-grip pose.

    python3 tune.py fit [LOGS...]          # refit model/gravity_fit.json from logs (default: every logs/server_*.csv
    python3 tune.py check                  # hold 9 poses around the pre-grip (+-0.06 rad, hands stay above the
    python3 tune.py steps [--joint L0]     # 0.05 rad steps out and back on each arm joint at the pre-grip: lag,
    python3 tune.py track push [--lift 40] # play a stage's reference path LIFTED --lift mm into the air (FREE AIR:
    python3 monitor.py fric 0.5            # friction lead, x the fitted joint friction (0 = off)
    python3 monitor.py gains 1.5 1.22      # Unitree's arm kp x1.5, kd x1.22 (1 1 = Unitree's)
"""
import argparse
import glob
import json
import os
import shutil
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk import fit as F  # noqa: E402
from pk import logs as L  # noqa: E402
from pk.model import PINKY_END, Robot  # noqa: E402

FIT = os.path.join(config.MODEL_DIR, "gravity_fit.json")


def tips(rob, q, h):
    """Pinky end (left), middle fingertip (right), pelvis frame."""
    T = rob.fk(q, h)
    return (rob.point(T, "left_pinky_distal_link", PINKY_END["left"]),
            T["right_middle_distal_link"][:3, 3].copy())


# ============================================================================ fit
def cmd_fit(a):
    files = a.logs or (sorted(glob.glob(os.path.join(config.LOG_DIR, "server_*.csv")))
                       + sorted(glob.glob(os.path.join(config.ROOT, "..", "dexpick_deploy", "logs", "play_*.csv"))))
    if not files:
        print("no logs found")
        return 1
    rob = Robot()
    data = []
    for f in files:
        try:
            lg = L.load(f)
        except Exception as ex:                                 # noqa: BLE001
            print("skip %s: %r" % (f, ex))
            continue
        # contact phases would bias a gravity fit: only free motion (dp_play logs: everything; server logs: HOME,
        # GO_HOME, REWIND and the tune.py moves)
        idx = F.samples(lg, every=a.every)
        if "des" in lg and "server_" in os.path.basename(f):
            ok = np.isin(lg["stage"][idx], ("HOME", "GO_HOME", "REWIND", "HELD")) | np.array(["tune" in str(p) for p in lg["phase"][idx]], dtype=bool)
            idx = idx[ok]
        print("%-40s %6d samples" % (os.path.basename(f), len(idx)))
        if len(idx):
            data.append((lg, idx))
    if not data:
        print("no free-motion samples in these logs")
        return 1
    res = F.fit(data, rob)
    for s in res["stats"]:
        print("  %-13s scale %5.2f  bias %+5.2f N m  friction %4.2f N m | residual %.2f N m (URDF alone %.2f)  R2 %.2f" % (
            s["joint"], s["scale"], s["bias_Nm"], s["friction_Nm"], s["rms_Nm"], s["rms_urdf_Nm"], s["r2"]))
    if a.dry:
        return 0
    if os.path.exists(FIT):
        shutil.copy(FIT, FIT.replace(".json", time.strftime("_%Y%m%d_%H%M%S.json")))
    res["sources"] = [os.path.basename(f) for f in files]
    json.dump(res, open(FIT, "w"), indent=1)
    print("wrote %s (old one kept beside it). Restart server.py to use it." % FIT)
    return 0


# ============================================================================ moves through the server
def client(name):
    from pk.client import Client
    c = Client(name)
    why = c.to_pregrip()                     # HOME is the neutral pose; tune was planned from the pre-grip pose
    if why:
        c.bye(then="path")
        raise SystemExit("could not get to the pre-grip pose: %s" % why)
    return c


def cmd_check(a):
    rob = Robot()
    c = client("tune check")
    try:
        st = c.state()
        home = np.array(st["des"])
        h = np.array(st["hands_cmd"])
        rng = np.random.RandomState(0)
        poses = [home] + [home + np.r_[rng.uniform(-a.range, a.range, 10), 0.0] for _ in range(8)]
        rows = []
        for k, q in enumerate(poses):
            q = config.clamp_arm(q)
            why = c.move_to(q, v=0.15, tag="tune check %d" % k)
            if why:
                print("stopped: %s" % why)
                break
            c.hold(1.5)
            samples = []
            for _ in range(30):
                st = c.state()
                samples.append(np.array(st["q"]))
                c.hold(0.02)
            qm = np.median(samples, axis=0)
            e = q - qm
            pl, pr = tips(rob, q, h)
            ml, mr = tips(rob, qm, h)
            rows.append((e, 1000 * np.linalg.norm(pl - ml), 1000 * np.linalg.norm(pr - mr)))
            print("pose %d: planned - measured (rad) %s | fingertip off by L %.1f mm, R %.1f mm" % (
                k, " ".join("%+.3f" % v for v in e[:10]), rows[-1][1], rows[-1][2]))
        c.move_to(home, v=0.15, tag="tune check home")
        if rows:
            E = np.array([r[0] for r in rows])
            print("\nper joint, worst |planned - measured| (rad):")
            for j in range(10):
                print("  %-13s %.3f" % (config.ARM_NAMES[j], np.abs(E[:, j]).max()))
            print("fingertips: left (pushing) worst %.1f mm, right worst %.1f mm" % (max(r[1] for r in rows), max(r[2] for r in rows)))
            print("Over ~5 mm: run `python3 tune.py fit` (the server log of this check is in it), restart the server, check again.")
    finally:
        c.bye()
    return 0


def cmd_steps(a):
    c = client("tune steps")
    try:
        st = c.state()
        home = np.array(st["des"])
        joints = [config.ARM_NAMES.index(a.joint)] if a.joint else list(range(10))
        print("server: kp x%.2f, kd x%.2f of Unitree's" % (st["kp"][0] / config.KP[0], st["kd"][0] / config.KD[0]))
        print("%-13s %8s %8s %8s %8s" % ("joint", "lag ms", "overshoot", "settle s", "error rad"))
        for j in joints:
            for sgn in (+1, -1):
                q1 = home.copy()
                q1[j] += sgn * a.size
                q1 = config.clamp_arm(q1)
                t0 = time.time()
                c.set(q=q1, vmax=0.5, tag="tune step %s" % config.ARM_NAMES[j])
                tr = []
                while time.time() - t0 < 2.5:
                    s2 = c.state()
                    tr.append((time.time() - t0, s2["q"][j], s2["des"][j]))
                    c.hold(0.01)
                t = np.array([x[0] for x in tr])
                qm = np.array([x[1] for x in tr])
                d = qm - home[j]
                target = q1[j] - home[j]
                frac = d / target if abs(target) > 1e-6 else d
                lag = t[np.argmax(frac > 0.5)] if np.any(frac > 0.5) else float("nan")
                final = float(np.median(frac[-20:]))                 # where it settles (gravity error included)
                over = max(0.0, float(np.max(frac) - final))
                inside = np.abs(frac - final) < 0.1
                out_i = np.nonzero(~inside)[0]
                settle = t[out_i[-1] + 1] if len(out_i) and out_i[-1] + 1 < len(t) else (t[0] if not len(out_i) else t[-1])
                err = float(q1[j] - np.median(qm[-20:]))
                print("%-13s %8.0f %8.0f%% %8.2f %+8.3f   (%+.2f rad step)" % (config.ARM_NAMES[j], 1000 * lag, 100 * over, settle, err, sgn * a.size))
                c.move_to(home, v=0.2, tag="tune step back")
                c.hold(0.5)
    finally:
        c.bye()
    print("Overshoot > 20 % or settling > 1 s: the stiffness/damping is off; a steady error > 0.02 rad: refit gravity (tune.py fit).")
    return 0


def cmd_track(a):
    from pk.ik import shift_path
    from pk.refs import STAGES, cached
    from pk.stage_base import Ctx
    stage = {"push": "1push", "grip1": "2grip1", "carry": "3carry", "grip2": "4grip2"}[a.stage]
    ref = cached()
    seg = ref.stage(stage)
    rob = Robot()
    c = client("tune track " + a.stage)
    out = os.path.join(config.LOG_DIR, "tune", time.strftime("%Y%m%d_%H%M%S_track_") + a.stage)
    with c.busy(20):
        ctx = Ctx(c, out)
    try:
        lift = np.array([0.0, 0.0, a.lift / 1000])
        waist = float(c.state()["des"][config.WAIST])      # the server holds it where the robot had it; the stages solve for it
        with c.busy(60):
            ql, _ = shift_path(rob, "left", "left_base_link", [0, 0, 0.08], seg["q"], seg["h"], lift, w_rot=0.05, waist=waist)
            qq, _ = shift_path(rob, "right", "right_base_link", [0, 0, 0.08], ql, seg["h"], lift, w_rot=0.05, waist=waist)
            qq[:, config.WAIST] = waist
        corr = load_correction(a.stage) if a.use_correction else None
        if corr is not None:
            qq = qq + corr["dq"]
            print("playing with the learned correction (%s)" % corr["date"])
        st = c.state()
        # to the start, from the pre-grip pose, then the path (both lifted)
        why = c.move_to(qq[0], seg["h"][0], v=0.15, tag="tune track start")
        if why:
            print("stopped: %s" % why)
            return 1
        rec = []
        srv = dict(kp=st["kp"][0] / config.KP[0], kd=st["kd"][0] / config.KD[0], fric=0.0)

        def check(st_, k):
            if not rec or st_["t"] != rec[-1][3]:          # each server state once (the loop polls faster than it sends)
                rec.append((k, np.array(st_["q"]), np.array(st_["hands"]), st_["t"], np.array(st_["des"])))
                srv["fric"] = max(srv["fric"], float(np.max(np.abs(st_.get("fric", [0.0])))))
            return ""
        dt = float(np.median(np.diff(seg["t"])))
        why, _ = ctx.play(qq, seg["h"], dt, speed=a.speed, check=check, tags=["tune " + p for p in seg["phase"]])
        if why:
            print("stopped: %s" % why)
        # errors: what the plan (before any correction) wanted vs where the fingertips were (both at the robot's waist:
        # until the plan's waist was the simulation's 0, which added ~4 mm of waist turn to every number)
        plan = qq - (corr["dq"] if corr is not None else 0.0)
        el, er, dqs = [], [], np.zeros_like(qq)
        cnt = np.zeros(len(qq))
        for k, qm, hm, _, _ in rec:
            pl, pr = tips(rob, plan[k], seg["h"][k])
            ml, mr = tips(rob, qm, hm)
            el.append(1000 * np.linalg.norm(pl - ml))
            er.append(1000 * np.linalg.norm(pr - mr))
            dqs[k] += plan[k] - qm
            cnt[k] += 1
        ph = [seg["phase"][k] for k, _, _, _, _ in rec]
        stuck = stuck_share(rec, ph)
        print("\nfingertip error along the %s path (lifted %.0f mm, speed x%.2f; server: kp x%.2f, kd x%.2f, friction lead up to %.3f rad):" % (
            a.stage, a.lift, a.speed, srv["kp"], srv["kd"], srv["fric"]))
        print("  (stuck: of the time a joint's plan moved, the share it stood still -- stick-slip from joint friction)")
        for p in dict.fromkeys(ph):
            m = [i for i, x in enumerate(ph) if x == p]
            print("  %-12s left pinky median %4.1f / max %4.1f mm   right middle median %4.1f / max %4.1f mm   stuck L %s R %s" % (
                p, np.median([el[i] for i in m]), max(el[i] for i in m), np.median([er[i] for i in m]), max(er[i] for i in m),
                stuck.get((p, "L"), "  - "), stuck.get((p, "R"), "  - ")))
        if a.learn and rec:
            ok = cnt > 0
            d = np.zeros_like(qq)
            d[ok] = dqs[ok] / cnt[ok, None]
            # fill gaps, smooth over ~0.3 s of reference time, never the waist
            for j in range(config.N_ARM):
                d[:, j] = np.interp(np.arange(len(qq)), np.nonzero(ok)[0], d[ok, j])
                w = max(1, int(0.3 / dt))
                d[:, j] = np.convolve(np.pad(d[:, j], w, mode="edge"), np.ones(2 * w + 1) / (2 * w + 1), mode="same")[w:-w]
            d[:, config.WAIST] = 0.0
            prev = load_correction(a.stage)
            total = d * a.gain + (prev["dq"] if (prev is not None and a.use_correction) else 0.0)
            total = np.clip(total, -0.12, 0.12)
            path = os.path.join(config.REFS_DIR, "corrections", "%s.json" % a.stage)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            json.dump(dict(stage=a.stage, date=time.strftime("%Y-%m-%d %H:%M"), gain=a.gain, speed=a.speed, lift_mm=a.lift,
                           dq=total.round(5).tolist(), note="added to the reference arm targets of this stage (same sample times)"),
                      open(path, "w"))
            print("learned correction -> %s (largest %.3f rad). Run track again with --use-correction to check it." % (path, np.abs(total).max()))
    finally:
        c.bye()
    return 0


def stuck_share(rec, phases, span=0.04, v_plan=0.01, v_still=0.0025):
    """{(phase, 'L'/'R'): 'nn %'}: over spans of ~40 ms, of the joint-spans whose plan (the server's des) moved faster
    than v_plan, the share in which the joint moved slower than v_still (rad/s). rec: (k, q, hands, server t, des);
    phases: the phase of each rec entry."""
    if len(rec) < 3:
        return {}
    t = np.array([r[3] for r in rec])
    Q = np.array([r[1] for r in rec])
    D = np.array([r[4] for r in rec])
    lag = max(1, int(round(span / max(float(np.median(np.diff(t))), 1e-3))))
    if len(rec) <= lag:
        return {}
    dt = np.maximum(t[lag:] - t[:-lag], 1e-3)[:, None]
    moving = np.abs(D[lag:] - D[:-lag]) / dt > v_plan
    still = moving & (np.abs(Q[lag:] - Q[:-lag]) / dt < v_still)
    ph = np.array(phases[lag:])
    out = {}
    for p in dict.fromkeys(phases):
        m = ph == p
        for side, sl in (("L", config.LEFT), ("R", config.RIGHT)):
            n = int(moving[m][:, sl].sum())
            if n >= 10:
                out[(p, side)] = "%3.0f %%" % (100.0 * still[m][:, sl].sum() / n)
    return out


def load_correction(stage):
    path = os.path.join(config.REFS_DIR, "corrections", "%s.json" % stage)
    if not os.path.exists(path):
        return None
    d = json.load(open(path))
    d["dq"] = np.array(d["dq"])
    return d


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("fit")
    p.add_argument("logs", nargs="*")
    p.add_argument("--every", type=int, default=10)
    p.add_argument("--dry", action="store_true", help="print the fit, do not write it")
    p = sub.add_parser("check")
    p.add_argument("--range", type=float, default=0.06, help="pose spread around the pre-grip, rad")
    p = sub.add_parser("steps")
    p.add_argument("--joint", default=None, choices=config.ARM_NAMES[:10])
    p.add_argument("--size", type=float, default=0.05)
    p = sub.add_parser("track")
    p.add_argument("stage", choices=["push", "grip1", "carry", "grip2"])
    p.add_argument("--lift", type=float, default=40.0, help="mm above the reference path (free air: robot back from the table, no plate)")
    p.add_argument("--speed", type=float, default=0.25)
    p.add_argument("--learn", action="store_true")
    p.add_argument("--gain", type=float, default=0.8)
    p.add_argument("--use-correction", action="store_true")
    a = ap.parse_args()
    if a.cmd is None:
        ap.print_help()
        return 1
    return {"fit": cmd_fit, "check": cmd_check, "steps": cmd_steps, "track": cmd_track}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
