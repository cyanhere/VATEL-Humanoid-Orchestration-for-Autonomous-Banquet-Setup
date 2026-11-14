"""Check that the reference paths, moved onto the measured plate, land where they should in the pelvis frame.

    python3 tests/test_ik_offline.py             # laptop or robot NX; ~10 s
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk import config  # noqa: E402
from pk.ik import ArmChain, shift_path  # noqa: E402
from pk.model import Robot  # noqa: E402
from pk.refs import cached  # noqa: E402

OK = []


def check(cond, what):
    OK.append((bool(cond), what))
    print(("PASS " if cond else "FAIL ") + what, flush=True)


def palm(rob, side, q11, h12, p_local):
    ch = ArmChain(rob, side, "%s_base_link" % side, q11, h12)
    sl = config.LEFT if side == "left" else config.RIGHT
    T, _ = ch.fk(q11[sl])
    return T[:3, :3] @ p_local + T[:3, 3], T[:3, :3]


def main():
    import json
    rob = Robot()
    seg = cached().stage("2grip1")
    P = json.load(open(os.path.join(config.ROOT, "params", "grip1.json")))
    every = 6
    q_ref, h_ref = seg["q"][::every], seg["h"][::every]
    waist = 0.0102
    delta = np.array([0.00685, 0.00246, -0.02176])     # grip1's offset in the simulated-arm run of
    p_local = np.array(P["slot_mm"]) / 1000.0           # the grip's slot point, as the stage moves it
    check(np.allclose(q_ref[:, config.WAIST], 0.0), "the simulation's grip1 path has the waist at 0")
    q_old, _ = shift_path(rob, "right", "right_base_link", p_local, q_ref, h_ref, delta, w_rot=P["w_rot"])
    q_new, worst = shift_path(rob, "right", "right_base_link", p_local, q_ref, h_ref, delta, w_rot=P["w_rot"], waist=waist)
    e_old, e_new, r_old, r_new = [], [], [], []
    for k in range(len(q_ref)):
        want, R_want = palm(rob, "right", q_ref[k], h_ref[k], p_local)
        want = want + delta
        for q, e, r in ((q_old, e_old, r_old), (q_new, e_new, r_new)):
            q_run = q[k].copy()
            q_run[config.WAIST] = waist                  # what the robot executes: its own waist
            got, R_got = palm(rob, "right", q_run, h_ref[k], p_local)
            e.append(1000 * np.linalg.norm(got - want))
            r.append(np.degrees(np.arccos(np.clip((np.trace(R_want.T @ R_got) - 1) / 2, -1, 1))))
    print("palm point vs the reference + offset, pelvis frame, waist %.4f rad, %d waypoints:" % (waist, len(q_ref)))
    print("  solved at the path's waist (before): median %.2f mm, max %.2f mm; palm turned median %.2f, max %.2f deg" % (
        np.median(e_old), np.max(e_old), np.median(r_old), np.max(r_old)))
    print("  solved at the robot's waist (now):   median %.2f mm, max %.2f mm; palm turned median %.2f, max %.2f deg" % (
        np.median(e_new), np.max(e_new), np.median(r_new), np.max(r_new)))
    check(np.median(e_old) > 2.5, "before: the hand was off by the waist turn (%.1f mm median)" % np.median(e_old))
    check(np.max(e_new) < 1.0 and worst < 3.0, "now: on the reference + offset within %.2f mm (IK worst %.2f mm)" % (np.max(e_new), worst))
    # the palm may turn a little to keep the slot point exact (w_rot); the waist fix must not make that worse
    check(np.max(r_new) <= np.max(r_old) + 0.6, "now: the palm turns no more than before (max %.2f vs %.2f deg; w_rot %.2f)" % (
        np.max(r_new), np.max(r_old), P["w_rot"]))
    q_zero, _ = shift_path(rob, "right", "right_base_link", p_local, q_ref, h_ref, delta, w_rot=P["w_rot"], waist=0.0)
    check(np.allclose(q_zero, q_old), "waist 0 (the simulation's) gives the old result exactly")
    n_bad = sum(1 for ok, _ in OK if not ok)
    print("\n%d checks, %d failed" % (len(OK), n_bad))
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
