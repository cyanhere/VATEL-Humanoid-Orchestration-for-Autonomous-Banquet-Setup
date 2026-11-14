"""Off-robot test: from a camera frame to the plate's position in the ROBOT frame.

    python3 test_robot_frame_offline.py

The scene is built in pelvis_link -- floor, table, a plate stack at a known spot -- and seen
by a camera whose mount is WRONG in a known way, like the real one (4 deg more tilt than the
CAD, -2.3 deg of roll). The camera calibrates itself from the level table, the stack is found
in its image, and the plate is carried into the robot frame. Ground truth is exact because the
scene was built there, so the error is the error.

Also checked: that the CAD nominal really would have been wrong by tens of millimetres (or
the calibration is not doing anything), and that a YAW error -- which a level surface cannot
see -- shows up as the sideways error the documentation says it does, and nowhere else.
"""
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import camera_calibration as cc
import plate_stack as ps
import r1_config as cfg
import urdf_fk as fk
from stack_scene import INTR, PITCH, PLATE_D, RIM0, render

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


def rot(axis, deg):
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


J = fk.load_urdf(cfg.URDF_PATH)
T_NOM = fk.T_pelvis_optical(J, {}, allow_nominal=True)

PELVIS_H = 0.71                  # pelvis above the floor (the real robot: 1.18 - 0.47)
TABLE_H = 0.745                  # table above the floor
N_PLATES = 4
BASE = np.array([0.60, -0.02, TABLE_H - PELVIS_H])    # stack base, pelvis_link
TOP = RIM0 + (N_PLATES - 1) * PITCH
TRUTH = BASE + np.array([0.0, 0.0, TOP])              # top rim centre, pelvis_link


def scene(R_true, seed=0):
    """What a camera with orientation R_true (pelvis <- optical) at the CAD position sees."""
    t = T_NOM[:3, 3]
    n_opt = R_true.T @ np.array([0.0, 0.0, 1.0])
    o_opt = R_true.T @ (BASE - t)
    depth, tr = render(n_plates=N_PLATES, normal=n_opt, origin=o_opt, seed=seed)
    return depth, tr, n_opt, PELVIS_H + t[2]


def run(R_true, seed=0):
    depth, tr, up, cam_h = scene(R_true, seed)
    cal = cc.from_level_surface(up, camera_height_m=cam_h, joints=J, surface="floor")
    st = ps.top_plate(depth, INTR, tr["plane"], plate_diameter_m=PLATE_D,
                      plate_pitch_m=PITCH, plate_rim_height_m=RIM0)
    return cal, st, ps.to_robot_frame(st, joints=J, calibration=cal)


def tilt_of(R):
    return cc._tilt_roll(R.T @ np.array([0.0, 0.0, 1.0]))


# ==========================================================================
print("\n  THE CALIBRATION ITSELF")
# ==========================================================================
cal0 = cc.from_level_surface(T_NOM[:3, :3].T @ np.array([0.0, 0.0, 1.0]), joints=J)
T0 = cc.T_pelvis_optical(cal0, J)
check("a surface exactly where the CAD predicts changes nothing",
      np.allclose(T0, T_NOM, atol=1e-9) and abs(cal0["correction_tilt_deg"]) < 1e-6)

for a, b in ((np.array([0.0, -0.8, -0.6]), np.array([0.1, -0.9, -0.42])),
             (np.array([1.0, 0, 0]), np.array([-1.0, 0, 0]))):
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    Q = cc._rotation_between(a, b)
    check("the smallest rotation takes one vector exactly onto the other",
          np.allclose(Q @ a, b, atol=1e-9) and abs(np.linalg.det(Q) - 1) < 1e-9)

# The real robot read 34.0 deg tilt and -2.3 deg roll. The tilt is unambiguous; the ROLL is
# not, and the difference matters:
#   - if it comes from the robot leaning sideways (or the floor sloping), the camera is
#     turned about a HORIZONTAL axis, and a level surface sees all of it
#   - if the helmet is twisted about the camera's OWN viewing axis, which points 34 deg down,
#     then part of that twist is a turn about the vertical: 2.3 x sin 34 = 1.3 deg of yaw,
#     which no level surface can see
# Both give the same reading. The first is modelled here as R_LEAN, the second as R_TWIST.
LEAN = 2.3 * np.cos(np.radians(34.0))          # reads as 2.3 deg of roll at 34 deg tilt
R_LEAN = rot("x", LEAN) @ T_NOM[:3, :3] @ rot("x", -4.0)
R_TWIST = T_NOM[:3, :3] @ rot("x", -4.0) @ rot("z", 2.3)
R_REAL = R_LEAN
t_real, r_real = tilt_of(R_REAL)
check("the test camera really is mounted like the real one",
      abs(t_real - 34.0) < 0.1 and abs(r_real + 2.3) < 0.05,
      "tilt %.2f deg, roll %+.2f deg (real robot: 34.0, -2.3)" % (t_real, r_real))
check("...and the twisted version reads exactly the same",
      np.allclose(tilt_of(R_TWIST), tilt_of(R_LEAN), atol=0.05),
      "tilt %.2f, roll %+.2f -- indistinguishable from a level surface" % tilt_of(R_TWIST))

cal, st, rf = run(R_REAL, seed=1)
check("calibrating from the level table recovers the extra tilt",
      abs(cal["measured_tilt_deg"] - t_real) < 1e-6
      and abs(cal["correction_tilt_deg"] - (t_real - 30.0)) < 1e-3,
      "correction %+.2f deg tilt, %+.2f deg roll"
      % (cal["correction_tilt_deg"], cal["correction_roll_deg"]))

path = os.path.join(tempfile.mkdtemp(), "cal.json")
cc.save(cal, path)
check("it survives being saved and loaded",
      np.allclose(cc.T_pelvis_optical(cc.load(path), J), cc.T_pelvis_optical(cal, J)))

# ==========================================================================
print("\n  THE PLATE, IN THE ROBOT FRAME")
# ==========================================================================
err = rf["center_m"] - TRUTH
check("the top plate's centre is within 5 mm of where it really is",
      np.linalg.norm(err) < 0.005,
      "measured %s, truth %s, error %.1f mm (x %+.1f, y %+.1f, z %+.1f)"
      % (np.array2string(rf["center_m"], precision=3), np.array2string(TRUTH, precision=3),
         1000 * np.linalg.norm(err), 1000 * err[0], 1000 * err[1], 1000 * err[2]))
check("the plate's normal points straight up in the robot frame",
      rf["normal"][2] > np.cos(np.radians(1.0)),
      "%.2f deg off vertical" % np.degrees(np.arccos(min(1.0, rf["normal"][2]))))
check("the table comes out at its real height above the floor",
      abs(rf["table_above_floor_m"] - TABLE_H) < 0.004,
      "%.3f m vs %.3f m" % (rf["table_above_floor_m"], TABLE_H))
check("...and the top plate at table + stack",
      abs(rf["top_above_floor_m"] - (TABLE_H + TOP)) < 0.004,
      "%.3f m" % rf["top_above_floor_m"])
check("the count travels with it", rf["n_plates"] == N_PLATES)
check("and it says which calibration produced it", "level floor" in rf["calibration"])

nom = ps.to_robot_frame(st, joints=J, allow_nominal=True, calibration=None)
e_nom = np.linalg.norm(nom["center_m"] - TRUTH)
check("the CAD nominal alone would have been badly wrong", e_nom > 0.025,
      "%.0f mm off -- %.0f plate pitches" % (1000 * e_nom, e_nom / PITCH))

# ==========================================================================
print("\n  WHAT A LEVEL SURFACE CANNOT SEE: YAW")
# ==========================================================================
reach = np.hypot(*(TRUTH[:2] - T_NOM[:2, 3]))


def sideways(R, yaw_deg, label, seed):
    _, _, rfy = run(R, seed=seed)
    e = rfy["center_m"] - TRUTH
    want = reach * np.tan(np.radians(yaw_deg))
    check(label, abs(abs(e[1]) - want) < 0.004 and abs(e[0]) < 0.006 and abs(e[2]) < 0.004,
          "y %+.1f mm vs %.1f predicted (x %+.1f, z %+.1f)"
          % (1000 * e[1], 1000 * want, 1000 * e[0], 1000 * e[2]))


sideways(rot("z", 1.5) @ R_LEAN, 1.5,
         "a 1.5 deg yaw error is not corrected, and lands sideways as predicted", seed=2)
sideways(R_TWIST, 2.3 * np.sin(np.radians(34.0)),
         "a helmet twisted on its own axis hides 1.3 deg of yaw the same way", seed=3)

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
