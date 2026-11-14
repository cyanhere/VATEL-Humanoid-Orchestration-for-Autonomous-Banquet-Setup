#!/usr/bin/env python3
"""Print and log what the Revo2 Touch fingertips read: normal force, tangential force and proximity, per finger.

    python3 touch_watch.py                       # LISTEN ONLY: reads rt/brainco/left/touch, commands nothing
    python3 touch_watch.py --side right
    python3 touch_watch.py --present             # with server.py at the pre-grip pose: raises the LEFT hand 12 cm,
    python3 touch_watch.py --present --up 0.08 --curl 0.3
"""
import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402

PALM = np.array([0.0, 0.0, 0.08])


def facing(n):
    """A pad's outward normal (pelvis frame) in words."""
    words = []
    for v, pos, neg in ((n[2], "UP", "DOWN"), (n[0], "forward (away from the robot)", "back (toward the robot)"),
                        (n[1], "to the robot's left", "to the robot's right")):
        if abs(v) > 0.35:
            words.append(pos if v > 0 else neg)
    return " and ".join(words) or "sideways"


def present_pose(rob, home_q, home_h, up, curl):
    """The left arm with the palm `up` m above the pre-grip pose, its orientation kept; index and pinky at `curl`."""
    from pk.ik import ArmChain, solve_pose
    T = rob.fk(home_q, home_h)
    M = T["left_base_link"]
    P = M[:3, :3] @ PALM + M[:3, 3] + np.array([0.0, 0.0, up])
    h = home_h.copy()
    h[0:6] = [0.0, 0.0, curl, 0.0, 0.0, curl]                  # left: thumb, thumb aux, index, middle, ring, pinky
    q5, info = solve_pose(ArmChain(rob, "left", "left_base_link", home_q, h), home_q[config.LEFT], P, M[:3, :3], PALM, w_rot=0.05)
    q = home_q.copy()
    q[config.LEFT] = q5
    return q, h, info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--side", choices=["left", "right"], default="left")
    ap.add_argument("--iface", default="eth10")
    ap.add_argument("--present", action="store_true", help="raise the left hand through the server and watch (see above)")
    ap.add_argument("--up", type=float, default=0.12, help="--present: how far the palm goes up from the pre-grip pose (m)")
    ap.add_argument("--curl", type=float, default=0.5, help="--present: index and little finger command (0 open .. 1 closed)")
    a = ap.parse_args()
    os.makedirs(config.LOG_DIR, exist_ok=True)
    path = os.path.join(config.LOG_DIR, time.strftime("touch_%Y%m%d_%H%M%S.csv"))
    log = open(path, "w")
    cols = ["t"] + ["%s_%s" % (w, f) for f in config.TOUCH_FINGERS for w in ("normal", "tangential", "dir", "prox")]
    if a.present:
        return present(a, log, path, cols)
    from pk.robot_io import DDSBackend                 # readers only: its publishers open only in open_arm / open_hands
    io = DDSBackend(a.iface)
    k = 0 if a.side == "left" else 1
    log.write(",".join(cols) + "\n")
    print("listening to rt/brainco/%s/touch (nothing is commanded); log: %s; Ctrl-C stops" % (a.side, path))
    print("columns: N normal force, T tangential force, P proximity (raw counts)")
    t0, t_print, last = time.time(), 0.0, None
    try:
        while True:
            s = io.read()
            now = time.time()
            if s.touch_t[k] and s.touch_t[k] != last:
                last = s.touch_t[k]
                row = s.touch[k]
                log.write(",".join(["%.3f" % (now - t0)] + ["%.0f" % v for f in row for v in f]) + "\n")
                if now - t_print > 0.2:
                    t_print = now
                    show(now - t0, row)
            elif not s.touch_t[k] and now - t0 > 3.0 and t_print == 0.0:
                print("no rt/brainco/%s/touch after 3 s: is the 13 Oct hand server running?" % a.side, flush=True)
                t_print = now
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    log.close()
    print("\nlog: %s" % path)
    return 0


def show(t, row):
    print("%6.1f s  " % t + "  ".join("%s N%5.0f T%5.0f P%7.0f" % (f[:5], v[0], v[1], v[3]) for f, v in zip(config.TOUCH_FINGERS, row)),
          flush=True)


def present(a, log, path, cols):
    from pk.client import Client
    from pk.model import Robot, pad
    rob = Robot()
    home = json.load(open(os.path.join(config.ROOT, "params", "premotion.json")))
    q, h, info = present_pose(rob, np.array(home["q"], float), np.array(home["hands"], float), a.up, a.curl)
    if info["pos_mm"] > 3.0:
        print("cannot raise the hand %.0f mm there (IK %.1f mm off): try a smaller --up" % (1000 * a.up, info["pos_mm"]))
        return 1
    T = rob.fk(q, h)
    log.write(",".join(cols + ["%s_%s" % (f, k) for f in ("index", "pinky") for k in ("x", "y", "z", "nx", "ny", "nz")]) + "\n")
    c = Client("touch present")
    why = c.to_pregrip()                     # HOME is the neutral pose; this tool was planned from the pre-grip pose
    if why:
        print("could not get to the pre-grip pose: %s" % why)
        c.bye(then="path")
        return 1
    try:
        why = c.move_to(q, h, v=0.2, tag="touch present")
        if why:
            print("stopped: %s" % why)
            return 1
        c.hold(1.0)
        st = c.state()
        T = rob.fk(np.array(st["q"]), np.array(st["hands"]))
        pads = {f: pad(T, "left", f) for f in ("index", "pinky")}
        print("the left hand is up; log: %s; Ctrl-C: the arm goes home" % path)
        for f, (p, n) in pads.items():
            print("  %-5s pad at (%.3f %+.3f %.3f) m, facing %s  (normal %+.2f %+.2f %+.2f)" % (f, *p, facing(n), *n))
        print("columns: N normal force, T tangential force, P proximity (raw counts)")
        t0, t_print, t_last = time.time(), 0.0, None
        while True:
            st = c.state()
            if st.get("touch") is None:
                if time.time() - t0 > 3.0 and t_print == 0.0:
                    print("no touch data from the server: is the 13 Oct hand server running?", flush=True)
                    t_print = time.time()
                c.hold(0.05)
                continue
            if st["t"] != t_last:
                t_last = st["t"]
                row = st["touch"][0]
                now = time.time()
                if now - t_print > 0.5:                       # the pads, where the arm measures them (rarely: FK is slow)
                    T = rob.fk(np.array(st["q"]), np.array(st["hands"]))
                    pads = {f: pad(T, "left", f) for f in ("index", "pinky")}
                pp = [v for f in ("index", "pinky") for v in np.r_[pads[f][0], pads[f][1]]]
                log.write(",".join(["%.3f" % (now - t0)] + ["%.0f" % v for fv in row for v in fv] + ["%.4f" % v for v in pp]) + "\n")
                if now - t_print > 0.2:
                    t_print = now
                    show(now - t0, row)
            c.hold(0.01)
    except KeyboardInterrupt:
        print("\nthe arm goes home along its path")
    finally:
        c.bye()
        log.close()
    print("log: %s" % path)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
