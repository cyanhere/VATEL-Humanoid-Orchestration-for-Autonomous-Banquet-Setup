#!/usr/bin/env python3
"""Head camera calibration, capture side: the hands go to known positions and the head camera records them.
Runs instead of perception.py (it needs the camera and the head).

    python3 calib_cam.py --dry                 # the plan only: hand positions, reach, clearance; nothing moves
    python3 calib_cam.py                       # both hands, 8-12 positions x 3 head aims each, ~4 min
    python3 calib_cam.py --sides left          # one hand
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk.ik import ArmChain, solve_pose  # noqa: E402
from pk.model import Robot, hand_points  # noqa: E402
from pk.tracker import project  # noqa: E402

PALM = np.array([0.0, 0.0, 0.08])                     # the point moved: middle of the palm, as tune.py track


# where the palm goes (pelvis frame, m): over the table in front of the robot, in front of the camera, each hand on its
# own side; a snake through it. The hand's orientation is free to follow (only the palm's position is asked).
REGION = {"left": dict(x=(0.34, 0.42), y=(0.18, 0.10), z=(0.22, 0.17, 0.13)),
          "right": dict(x=(0.32, 0.40), y=(-0.18, -0.10), z=(0.29, 0.24))}


def plan(rob, home_q, home_h, side, clear):
    """[(palm target, q11)] in free air in front of the camera, from the pre-grip pose."""
    sl = config.LEFT if side == "left" else config.RIGHT
    link = "%s_base_link" % side
    T = rob.fk(home_q, home_h)
    R0 = T[link][:3, :3]
    out, q_prev = [], home_q.copy()
    g = REGION[side]
    grid = []
    for i, z in enumerate(g["z"]):                     # a snake: each layer's 4 corners in turn, the next layer back
        layer = [(x, y, z) for x in g["x"] for y in g["y"]]
        layer = layer[:2] + layer[2:][::-1]
        grid += layer if i % 2 == 0 else layer[::-1]
    for P in grid:
        P = np.array(P)
        ch = ArmChain(rob, side, link, q_prev, home_h)
        q5, info = solve_pose(ch, q_prev[sl], P, R0, PALM, w_rot=0.01)
        q = q_prev.copy()
        q[sl] = q5
        Tq = rob.fk(q, home_h)
        low = min(float(v[:, 2].min()) for v in hand_points(Tq, side).values())
        ok = info["pos_mm"] < 3.0 and low >= clear
        out.append(dict(P=P, q=q, ok=ok, pos_mm=info["pos_mm"], rot_deg=info["rot_deg"], low=low))
        if ok:
            q_prev = q
    return out


def path_clear(rob, q0, q1, h, side, clear, n=8):
    """The joint-space way from q0 to q1 never takes the hand lower than `clear`, or than where it starts if that is
    lower already (the pre-grip pose itself is)."""
    low = lambda q: min(float(v[:, 2].min()) for v in hand_points(rob.fk(q, h), side).values())   # noqa: E731
    floor = min(clear, low(q0)) - 0.01
    return all(low(q0 + u * (q1 - q0)) >= floor for u in np.linspace(0, 1, n))


def via_up(rob, q0, q1, h, side, rise=0.05):
    """A waypoint straight above where the hand is, as high as the target + rise (free orientation), or None."""
    sl = config.LEFT if side == "left" else config.RIGHT
    link = "%s_base_link" % side
    T0, T1 = rob.fk(q0, h), rob.fk(q1, h)
    p0 = T0[link][:3, :3] @ PALM + T0[link][:3, 3]
    p1 = T1[link][:3, :3] @ PALM + T1[link][:3, 3]
    P = np.array([p0[0], p0[1], max(p0[2], p1[2]) + rise])
    q5, info = solve_pose(ArmChain(rob, side, link, q0, h), q0[sl], P, T0[link][:3, :3], PALM, w_rot=0.01)
    if info["pos_mm"] > 3.0:
        return None
    v = q0.copy()
    v[sl] = q5
    return v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sides", nargs="+", default=["left", "right"], choices=["left", "right"])
    ap.add_argument("--clear", type=float, default=0.08, help="lowest hand point, pelvis frame z (m); the table is ~0.02-0.035")
    ap.add_argument("--frames", type=int, default=2, help="frames saved per pose")
    ap.add_argument("--perception", default=config.PERCEPTION_DIR)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    rob = Robot()
    home = json.load(open(os.path.join(config.ROOT, "params", "premotion.json")))
    home_q, home_h = np.array(home["q"], float), np.array(home["hands"], float)
    plans = {s: plan(rob, home_q, home_h, s, a.clear) for s in a.sides}
    for s, pl in plans.items():
        print("%s hand: %d of %d positions usable" % (s, sum(p["ok"] for p in pl), len(pl)))
        for k, p in enumerate(pl):
            print("  %d palm (%.3f %+.3f %.3f)  IK %.1f mm %.1f deg  lowest hand point %.3f m  %s" % (
                k, p["P"][0], p["P"][1], p["P"][2], p["pos_mm"], p["rot_deg"], p["low"], "ok" if p["ok"] else "SKIPPED"))
    if a.dry:
        return 0

    import perception as per
    from pk.client import Client
    kin = per.Kin(a.perception, correct=False)          # the camera pose as R1_Plate_Perception has it (what is fitted)
    cam = per.Camera()
    out = os.path.join(config.LOG_DIR, "calib", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    c = Client("calib camera")
    why = c.to_pregrip()                     # HOME is the neutral pose; this tool was planned from the pre-grip pose
    if why:
        print("could not get to the pre-grip pose: %s" % why)
        c.bye(then="path")
        return 1
    hc = Client("calib camera head", role="head")
    print("saving to %s" % out)
    n_saved = 0
    try:
        q_now = np.array(c.state()["des"])
        for side, pl in plans.items():
            sl = config.LEFT if side == "left" else config.RIGHT
            for k, p in enumerate(pl):
                if not p["ok"]:
                    continue
                q = q_now.copy()
                q[sl] = p["q"][sl]
                way = [q]
                if not path_clear(rob, q_now, q, home_h, side, a.clear):
                    v = via_up(rob, q_now, q, home_h, side)
                    if v is None or not (path_clear(rob, q_now, v, home_h, side, a.clear) and path_clear(rob, v, q, home_h, side, a.clear)):
                        print("  %s %d: the way there passes too low: skipped" % (side, k))
                        continue
                    way = [v, q]
                for qw in way:
                    why = c.move_to(qw, v=0.2, tag="calib %s %d" % (side, k))
                    if why:
                        raise RuntimeError("move stopped: %s" % why)
                q_now = q
                # the head aimed straight at the hand, and to either side of it (the hand at different places in the
                # picture, the head at different angles: what separates the camera's mount from the head's encoders)
                aims = (np.zeros(3), np.array([0.0, 0.08, 0.04]), np.array([0.03, -0.08, -0.03]))
                for j, shift in enumerate(aims):
                    st = c.state()
                    tgt, clamped = kin.aim(p["P"] + shift, kin.angles(st))
                    if not wait_head(hc, tgt, c):
                        print("  %s %d/%d: the head did not settle: skipped" % (side, k, j))
                        continue
                    c.hold(0.8)                                # the arm settles too
                    for f in range(a.frames):
                        c.state()
                        fs, depth, colour, t = cam.frames()
                        st = c.state()
                        ang = kin.angles(st)
                        T = kin.T_po(ang)
                        pts = np.vstack(list(hand_points(rob.fk(np.array(st["q"]), np.array(st["hands"])), side).values()))
                        P_o = (pts - T[:3, 3]) @ T[:3, :3]
                        uv = project(P_o, cam.intr_c)
                        inside = float(np.mean((P_o[:, 2] > 0.15) & (uv[:, 0] >= 0) & (uv[:, 0] < cam.intr_c["width"])
                                               & (uv[:, 1] >= 0) & (uv[:, 1] < cam.intr_c["height"])))
                        np.savez_compressed(os.path.join(out, "%s_%02d_%d_%d" % (side, k, j, f)), depth=depth,
                                            color=colour if colour is not None else np.zeros(1), intr=json.dumps(cam.intr_d),
                                            intr_c=json.dumps(cam.intr_c), T_cd=cam.T_cd, depth_scale=cam.scale,
                                            state=json.dumps(st), T_po=T, angles=json.dumps(ang), side=side, in_view=inside)
                        n_saved += 1
                        time.sleep(0.15)
                    print("  %s %d/%d: head pitch %.2f yaw %+.2f%s, hand %.0f %% in the picture, saved" % (
                        side, k, j, tgt[0], tgt[1], " (clamped)" if clamped else "", 100 * inside), flush=True)
            # back to the pre-grip pose before the other hand
            why = c.move_to(home_q, v=0.2, tag="calib back")
            q_now = home_q.copy()
    except KeyboardInterrupt:
        print("\nstopped: the arms go home along their path")
    finally:
        try:
            hc.head(hc.head_look)
            time.sleep(1.0)
        except Exception:                                      # noqa: BLE001
            pass
        c.bye()
        hc.close()
    print("%d frames in %s -- pull the logs; the fit is calibration/calib_fit.py on the laptop" % (n_saved, out))
    return 0


def wait_head(hc, target, arms, timeout=8.0):
    """Until the head's command reached the target and the head has stopped (as perception.py). The arms client is
    kept talking meanwhile: silent for 1 s, the server takes the arms back (its hang guard)."""
    t0, still, prev = time.time(), 0, None
    while time.time() - t0 < timeout:
        arms.state()
        hc.head(target)
        st = hc.state()
        hd = np.array(st["head"], float)
        there = np.max(np.abs(np.array(st["head_cmd"], float) - config.clamp_head(np.asarray(target, float)))) < 0.005
        if there and prev is not None and np.max(np.abs(hd - prev)) < 0.002:
            still += 1
            if still > 20:
                return True
        else:
            still = 0
        prev = hd
        time.sleep(0.02)
    return False


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
