"""Off-robot test for the mount check.

    python3 test_check_mount_offline.py

The mount check is the first thing that will be run on the assembled helmet, and its whole
value is that it turns a printed part into a falsifiable number. So the arithmetic behind that
number is pinned here: what the CAD predicts, how a measured plane is turned into a tilt and a
roll, and that the verdict thresholds match the error budget they claim to.

The error budget is the reason the thresholds are where they are: plates nest at about 10 mm,
and at a 0.35 m working range 2 deg of tilt error is 12 mm. So 2 deg is not a round number
chosen for neatness, it is one plate pitch.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import check_mount as cm
from stack_scene import INTR, render

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


def plane_frame(up_opt, dist=0.8, intr=INTR):
    """A depth frame of nothing but one flat plane whose upward normal is `up_opt`."""
    h, w = intr["height"], intr["width"]
    yy, xx = np.mgrid[0:h, 0:w]
    dirs = np.stack([(xx - intr["cx"]) / intr["fx"],
                     (yy - intr["cy"]) / intr["fy"],
                     np.ones_like(xx, dtype=float)], axis=2)
    n = np.asarray(up_opt, float)
    n = n / np.linalg.norm(n)
    c = float(n @ (np.array([0.0, 0.0, 1.0]) * dist))
    nd = dirs @ n
    nd = np.where(np.abs(nd) < 1e-9, 1e-9, nd)
    t = c / nd
    return (np.clip(np.where(t > 0, t, 0.0), 0, 6) * 1000.0).astype(np.uint16)


ARGS = argparse.Namespace(head_pitch=0.0, head_yaw=0.0, range=0.35, near=0.20)

# ==========================================================================
print("\n  WHAT THE CAD PREDICTS")
# ==========================================================================
t0, up0 = cm.predicted(0.0)
# Tolerances are 1e-3 deg and 1e-5, not machine epsilon, and deliberately so: R1.urdf
# stores the camera pitch as 0.523599, a rounded pi/6, so the true nominal is 30.0000044
# deg. Asserting 1e-6 here fails on the data's own rounding rather than on anything real.
check("head at zero: the optical axis is 30.0 deg below horizontal",
      abs(t0 - cm.NOMINAL_TILT_DEG) < 1e-3, "%.4f deg" % t0)
check("...and a flat floor's normal reads [0, -0.866, -0.500] in optical coords",
      np.allclose(up0, [0.0, -np.sqrt(3) / 2, -0.5], atol=1e-5),
      np.array2string(up0, precision=4))
check("no roll is predicted, because the helmet is a pure pitch on the head",
      abs(up0[0]) < 1e-5, "x = %.2e" % up0[0])
for hp, want in ((-36.0, -6.0), (0.0, 30.0), (36.0, 66.0)):
    tt, _ = cm.predicted(np.radians(hp))
    check("head_pitch %+5.1f deg predicts %+5.1f deg below horizontal" % (hp, want),
          abs(tt - want) < 1e-4, "%.3f deg" % tt)

# ==========================================================================
print("\n  A MEASURED PLANE BECOMES A TILT AND A ROLL")
# ==========================================================================
for want_tilt in (0.0, 10.0, 30.0, 45.0, 66.0):
    a = np.radians(want_tilt)
    got_t, got_r = cm.readout([0.0, -np.cos(a), -np.sin(a)])
    check("a normal built at %4.1f deg reads back as %4.1f deg" % (want_tilt, want_tilt),
          abs(got_t - want_tilt) < 1e-6 and abs(got_r) < 1e-6,
          "tilt %.4f roll %.4f" % (got_t, got_r))
for want_roll in (-12.0, 0.0, 7.5):
    a, r = np.radians(30.0), np.radians(want_roll)
    up = np.array([np.cos(a) * np.sin(r), -np.cos(a) * np.cos(r), -np.sin(a)])
    got_t, got_r = cm.readout(up)
    check("a helmet rolled %+5.1f deg is reported as rolled" % want_roll,
          abs(got_r - want_roll) < 1e-4 and abs(got_t - 30.0) < 1e-4,
          "tilt %.3f roll %+.3f" % (got_t, got_r))

# ==========================================================================
print("\n  RECOVERED FROM A RENDERED FLOOR")
# ==========================================================================
for want in (30.0, 28.0, 35.0, 50.0):
    a = np.radians(want)
    depth = plane_frame([0.0, -np.cos(a), -np.sin(a)])
    n, c, inl = cm.up_from_floor(depth, INTR, 1000.0)
    got_t, got_r = cm.readout(n)
    check("a floor at %4.1f deg is measured as %4.1f deg" % (want, want),
          abs(got_t - want) < 0.3, "%.2f deg, roll %+.2f, %.0f%% inliers"
          % (got_t, got_r, 100 * (inl or 0)))

depth = plane_frame([0.0, -np.cos(np.radians(30)), -np.sin(np.radians(30))])
n, _, _ = cm.up_from_floor(depth, INTR, 1000.0)
check("the normal is oriented back toward the camera, not away", n[2] < 0,
      "z = %+.4f -- dominant_plane does not promise a sign, so it is fixed" % n[2])

# the ray-marched scene is a harder case: a real stack sits on the floor
depth, tr = render(n_plates=1, tilt_deg=60.0, holes=0.0, noise=0.0, seed=1)
n, _, _ = cm.up_from_floor(depth, INTR, 1000.0)
got_t, _ = cm.readout(n)
check("and it survives a plate standing on the floor", abs(got_t - 30.0) < 0.5,
      "%.2f deg with a plate in frame" % got_t)

# ==========================================================================
print("\n  A WALL BIGGER THAN THE FLOOR IS NOT MISTAKEN FOR IT")
# ==========================================================================
# This is what happened on the first run on the real robot: with the head level the top
# of the frame looks out almost horizontally, a wall 2.2 m away filled more of the picture
# than the floor, and the old code fitted the wall and reported a 40 deg mount error.

def floor_and_wall(tilt_deg=30.0, cam_h=1.07, wall_m=1.5, wall=True, floor=True):
    h, w = INTR["height"], INTR["width"]
    yy, xx = np.mgrid[0:h, 0:w]
    dirs = np.stack([(xx - INTR["cx"]) / INTR["fx"], (yy - INTR["cy"]) / INTR["fy"],
                     np.ones_like(xx, dtype=float)], axis=2)
    a = np.radians(tilt_deg)
    up = np.array([0.0, -np.cos(a), -np.sin(a)])       # floor normal, optical
    back = np.array([0.0, np.sin(a), -np.cos(a)])      # wall normal, toward the camera
    best = np.full((h, w), np.inf)
    for on, n, c in ((floor, up, -cam_h), (wall, back, -wall_m)):
        if not on:
            continue
        nd = dirs @ n
        nd = np.where(np.abs(nd) < 1e-9, 1e-9, nd)
        t = c / nd
        best = np.where((t > 0) & (t < best), t, best)
    best = np.where(np.isfinite(best), best, 0.0)
    return (np.clip(best, 0, 6) * 1000.0).astype(np.uint16)


scene = floor_and_wall()
import plate_detectors as pd_
n_old, _ = pd_.dominant_plane(scene, INTR, 1000.0)
t_old, _ = cm.readout(n_old if n_old[2] < 0 else -n_old)
check("the scene really does fool the old method (it picks the wall)",
      abs(t_old - 30.0) > 20.0, "old method read %.1f deg" % t_old)
n, c, inl = cm.up_from_floor(scene, INTR, 1000.0)
t_new, r_new = cm.readout(n)
check("the floor is found anyway, at the right tilt", abs(t_new - 30.0) < 0.3,
      "%.2f deg, roll %+.2f" % (t_new, r_new))
check("...and the camera height it implies is the real one", abs(-c - 1.07) < 0.01,
      "%.3f m above the floor" % -c)

for tilt in (20.0, 40.0):
    n, c, _ = cm.up_from_floor(floor_and_wall(tilt_deg=tilt), INTR, 1000.0)
    t_m, _ = cm.readout(n)
    check("a mount actually at %.0f deg is still measured as %.0f deg" % (tilt, tilt),
          abs(t_m - tilt) < 0.3, "%.2f deg -- the level-only rule does not bias it" % t_m)

try:
    cm.up_from_floor(floor_and_wall(floor=False), INTR, 1000.0)
    said = ""
except cm.NotFloor as e:
    said = str(e)
check("a view of only a wall is refused, and it says WALL", "WALL" in said,
      said.splitlines()[0] if said else "it returned a number instead")

# ==========================================================================
print("\n  --calibrate TURNS THE FLOOR INTO A SAVED CALIBRATION")
# ==========================================================================
frames = [(floor_and_wall(tilt_deg=34.0, cam_h=1.18, wall=False), INTR, 1000.0)] * 3
cal, spread = cm.calibrate_from(frames, ARGS)
check("it measures the real robot's 34 deg and calls it +4 on the CAD",
      abs(cal["measured_tilt_deg"] - 34.0) < 0.2 and abs(cal["correction_tilt_deg"] - 4.0) < 0.2,
      "%.2f deg, correction %+.2f" % (cal["measured_tilt_deg"], cal["correction_tilt_deg"]))
check("...and the camera's height above the floor", abs(cal["camera_above_floor_m"] - 1.18) < 0.005,
      "%.3f m; robot frame origin %.3f m up" % (cal["camera_above_floor_m"],
                                                cal["pelvis_above_floor_m"]))
moving = [(floor_and_wall(tilt_deg=a, wall=False), INTR, 1000.0) for a in (32.0, 34.0)]
try:
    cm.calibrate_from(moving, ARGS)
    refused = ""
except cm.NotFloor as e:
    refused = str(e)
check("frames that disagree (robot moving) are refused, not averaged", "disagree" in refused,
      refused[:70])

# ==========================================================================
print("\n  THE ERROR BUDGET THE VERDICTS REST ON")
# ==========================================================================
check("2 deg of tilt error costs about one plate pitch at 0.35 m",
      11.0 < cm.cost_mm(2.0, 0.35) < 13.0, "%.1f mm" % cm.cost_mm(2.0, 0.35))
check("the G1's 23 deg error would have cost ~150 mm",
      140 < cm.cost_mm(23.0, 0.35) < 160, "%.0f mm" % cm.cost_mm(23.0, 0.35))
check("under 1 deg is called GOOD", cm.verdict(0.8, 0.35).startswith("GOOD"))
check("1-2 deg is called MARGINAL", cm.verdict(1.5, 0.35).startswith("MARGINAL"))
check("over 2 deg is called TOO BIG", cm.verdict(2.5, 0.35).startswith("TOO BIG"))
check("the sign of the error does not change the verdict",
      cm.verdict(-2.5, 0.35).startswith("TOO BIG"))

# ==========================================================================
print("\n  A PRINTED PART IN THE FRUSTUM IS NOTICED")
# ==========================================================================
clear = plane_frame([0.0, -np.cos(np.radians(30)), -np.sin(np.radians(30))])
frac, _ = cm.helmet_in_view(clear, 1000.0)
check("a clear view reports no occlusion", frac < 1e-4, "%.4f%%" % (100 * frac))

blocked = clear.copy()
blocked[:50, :] = 120          # something 120 mm from the lens, across the top
frac, where = cm.helmet_in_view(blocked, 1000.0)
check("a part across the top of the frame is detected", frac > 0.05,
      "%.1f%% of the frame, strongest at: %s"
      % (100 * frac, max(where.items(), key=lambda kv: kv[1])[0]))
check("...and localised to the top edge rather than the middle",
      where["top"] > where["middle"],
      "top %.2f vs middle %.2f" % (where["top"], where["middle"]))

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
