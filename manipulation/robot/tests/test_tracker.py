"""Offline check of pk.tracker against R1_Plate_Perception.

    python3 tests/test_tracker.py [--perception ~/R1_Plate_Perception]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk.tracker import RimTracker  # noqa: E402


def frame_from_plane(n, c):
    """T (z-up frame <- optical) with z along the table's upward normal, origin on the table under the camera."""
    n = np.asarray(n, float)
    n = n / np.linalg.norm(n)
    up = -n if c > 0 else n                       # the camera (optical origin) is above the table
    O = c * n
    ex = np.array([0.0, 0.0, 1.0]) - up * up[2]
    ex /= np.linalg.norm(ex)
    ey = np.cross(up, ex)
    R = np.stack([ex, ey, up])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ O
    return T


def main():
    ap = argparse.ArgumentParser()
    default = os.path.expanduser("~/R1_Plate_Perception")
    if not os.path.isdir(default):
        default = os.path.abspath(os.path.join(HERE, "..", "..", "robot_mirror", "R1_Plate_Perception"))
    ap.add_argument("--perception", default=default)
    a = ap.parse_args()
    sys.path.insert(0, a.perception)
    import plate_detectors as pd
    import plate_stack as ps
    np.seterr(invalid="ignore")
    bad = 0
    for name in ("lab_2plates", "lab_3plates", "lab_4plates"):
        z = np.load(os.path.join(a.perception, "real_frames", name + ".npz"), allow_pickle=False)
        depth, intr, scale = z["depth"], json.loads(str(z["intr"])), float(z["depth_scale"])
        plane = pd.dominant_plane(depth, intr, scale)
        T = frame_from_plane(*plane)
        st = ps.locate_stacks(depth, intr, plane, depth_scale=scale, plate_diameter_m=0.254)[0]
        c_ref = T[:3, :3] @ np.asarray(st["center_m"]) + T[:3, 3]
        trk = RimTracker(c_ref[:2], c_ref[2], 0.0, plate_r=st["diameter_m"] / 2)
        est = trk.init(depth, intr, scale, T)
        t0 = time.time()
        for _ in range(10):
            est = trk.update(depth, intr, scale, T)
        dt = (time.time() - t0) / 10
        d = np.linalg.norm(np.array(est["center"][:2]) - c_ref[:2]) * 1000
        ok = est["ok"] and d < 6.0
        bad += not ok
        print("%s %-12s tracker vs locate_stacks centre %.1f mm, rim z %.1f vs %.1f mm (top height), n %d, coverage %.2f, "
              "rms %.1f mm, band r %.1f mm, %.0f ms/frame" % ("PASS" if ok else "FAIL", name, d, 1000 * est["center"][2],
                                                              1000 * st["top_height_m"], est["n"], est["coverage"], est["rms_mm"] or -1,
                                                              1000 * trk.r_band, 1000 * dt))
    # ---- synthetic slide
    from stack_scene import INTR, render
    import stack_scene
    errs = []
    trk = None
    for k, off in enumerate(np.arange(0.0, 0.046, 0.005)):
        depth, truth = render(n_plates=3, tilt_deg=55.0, dist=0.45, offset=(off, 0.0), noise=0.0015, holes=0.2, seed=k)
        depth = depth.copy()
        # a "hand": 40 x 60 px blob 40 mm closer than the rim, over the rim's left side
        hgt, wid = depth.shape
        st0 = truth["stacks"][0]
        if trk is None:
            plane = truth["plane"]
            T = frame_from_plane(*ps._as_plane(plane))
            sl = ps.locate_stacks(depth, INTR, plane, depth_scale=1000.0, plate_diameter_m=stack_scene.PLATE_D)[0]
            c0 = T[:3, :3] @ np.asarray(sl["center_m"]) + T[:3, 3]
            trk = RimTracker(c0[:2], c0[2], 0.0, plate_r=stack_scene.PLATE_D / 2)
            trk.init(depth, INTR, 1000.0, T)
            start = trk.c.copy()
        u = int(INTR["cx"] - 120)
        v = int(INTR["cy"] - 10)
        blob = depth[v:v + 60, u:u + 40]
        depth[v:v + 60, u:u + 40] = np.where(blob > 0, blob - 40, 0)
        est = trk.update(depth, INTR, 1000.0, T)
        moved = np.linalg.norm(np.array(est["center"][:2]) - start) * 1000
        errs.append(abs(moved - 1000 * off))
        print("synthetic slide %4.1f mm: tracked %5.1f mm  ok %s n %d coverage %.2f %s" % (1000 * off, moved, est["ok"], est["n"], est["coverage"], est.get("why", "")))
    e = max(errs)
    print("%s synthetic slide 0-45 mm with a hand blob: worst travel error %.1f mm" % ("PASS" if e < 2.5 else "FAIL", e))
    bad += e >= 2.5
    bad += finger_at_rim(ps, stack_scene, INTR, render)
    print("%d failed" % bad)
    return 1 if bad else 0


def finger_at_rim(ps, stack_scene, INTR, render):
    """ on the robot: the curled little finger at the rim's height, just outside its edge, pulled the tracked
    circle 5-9 mm toward the hand. Here the hand -- three finger-sized balls along ~5 cm of the rim, 2-26 mm outside the
    edge at the rim's height -- follows the slid plate; the tracker is told roughly where it is (8 mm off, as forward
    kinematics + calibration would). One ball alone (2 of 72 sectors) moves nothing: the circle fit is robust."""
    def ball(depth, T, c_up, r):
        R, t = T[:3, :3], T[:3, 3]
        S = R.T @ (np.asarray(c_up, float) - t)                       # the ball's centre, optical frame
        u0, u1 = int(INTR["fx"] * (S[0] - r) / S[2] + INTR["cx"]) - 2, int(INTR["fx"] * (S[0] + r) / S[2] + INTR["cx"]) + 2
        v0, v1 = int(INTR["fy"] * (S[1] - r) / S[2] + INTR["cy"]) - 2, int(INTR["fy"] * (S[1] + r) / S[2] + INTR["cy"]) + 2
        for v in range(max(v0, 0), min(v1, depth.shape[0])):
            for u in range(max(u0, 0), min(u1, depth.shape[1])):
                d = np.array([(u - INTR["cx"]) / INTR["fx"], (v - INTR["cy"]) / INTR["fy"], 1.0])
                b, cc = -2 * d @ S, S @ S - r * r
                disc = b * b - 4 * (d @ d) * cc
                if disc < 0:
                    continue
                z = (-b - math.sqrt(disc)) / (2 * (d @ d))
                if depth[v, u] == 0 or z * 1000 < depth[v, u]:
                    depth[v, u] = z * 1000
    res = {}
    for handled in (False, True):
        trk, errs = None, []
        for k, off in enumerate(np.arange(0.0, 0.046, 0.005)):
            depth, truth = render(n_plates=3, tilt_deg=55.0, dist=0.45, offset=(off, 0.0), noise=0.0015, holes=0.2, seed=k)
            depth = depth.copy()
            T = frame_from_plane(*ps._as_plane(truth["plane"]))
            sl = ps.locate_stacks(depth, INTR, truth["plane"], depth_scale=1000.0, plate_diameter_m=stack_scene.PLATE_D)[0]
            c_true = T[:3, :3] @ np.asarray(sl["center_m"]) + T[:3, 3]
            if trk is None:
                trk = RimTracker(c_true[:2], c_true[2], 0.0, plate_r=stack_scene.PLATE_D / 2)
                trk.init(depth, INTR, 1000.0, T)
                start, c_start = trk.c.copy(), c_true.copy()
                cam = T[:3, 3] - c_true                               # toward the camera, on the table
                side = np.array([-cam[1], cam[0], 0.0]) / np.linalg.norm(cam[:2])   # the plate's side as the camera sees it
            # the hand beside the edge: three finger-sized balls along the rim (~5 cm), 2-26 mm outside the edge
            r_f = 0.012
            along = np.array([-side[1], side[0], 0.0])
            balls = [np.r_[c_true[:2] + side[:2] * (stack_scene.PLATE_D / 2 + 0.002 + r_f) + along[:2] * k_ * 0.02, c_true[2] - 0.001]
                     for k_ in (-1, 0, 1)]
            for b_ in balls:
                ball(depth, T, b_, r_f)
            # what forward kinematics would say: the fingertip spheres, 8 mm off
            sph = [(b_ + np.array([0.006, -0.005, 0.0]), 0.022) for b_ in balls] if handled else []
            est = trk.update(depth, INTR, 1000.0, T, sph)
            moved = np.linalg.norm(trk.c - start) * 1000
            true_moved = np.linalg.norm(c_true[:2] - c_start[:2]) * 1000
            errs.append(abs(moved - true_moved))
        res[handled] = max(errs)
        print("  finger at the rim's height, %s: worst travel error %.1f mm" % ("hand sectors left out" if handled else "the old tracker (no hand)", max(errs)))
    # this synthetic scene only checks that leaving the hand's sectors out does no harm
    ok = res[True] < 2.5
    print("%s finger at the rim's height beside the slid plate: %.1f mm with the hand left out (the old tracker: %.1f mm, "
          "NOT the robot's 5-9 mm: unreproduced)" % ("PASS" if ok else "FAIL", res[True], res[False]))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
