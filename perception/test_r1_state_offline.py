"""Off-robot test for r1_state.py, with a fake rt/lowstate message.

    python3 test_r1_state_offline.py

No SDK, no robot. Checks that every joint is read from the slot Unitree's R1 code says it is
in, and that the head encoder correctly splits the floor-measured 4 deg between "the head is
pitched" and "the mount is off" -- the question the floor alone cannot answer.
"""
import contextlib
import io
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import r1_config as cfg
import r1_state as rs

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


def fake_lowstate(angles_deg, rpy_deg=(0.0, 0.0, 0.0)):
    """A stand-in for unitree_hg LowState_: 35 motor slots, an IMU, a mode_machine."""
    ms = [types.SimpleNamespace(q=0.0) for _ in range(cfg.NUM_SLOTS)]
    for name, deg in angles_deg.items():
        ms[cfg.MOTOR_SLOT[name]].q = np.radians(deg)
    # put junk in every slot the R1 does NOT use, so reading a wrong slot shows up
    used = set(cfg.MOTOR_SLOT.values())
    for i in range(cfg.NUM_SLOTS):
        if i not in used:
            ms[i].q = 99.0
    return types.SimpleNamespace(motor_state=ms,
                                 imu_state=types.SimpleNamespace(rpy=np.radians(rpy_deg)),
                                 mode_machine=7)


print("\n  EVERY JOINT FROM ITS OWN SLOT")
want = {n: float(i) for i, n in enumerate(sorted(cfg.MOTOR_SLOT))}   # all different
st = rs.decode(fake_lowstate(want))
check("all 26 joints decoded", len(st["joints"]) == 26)
check("each reads back exactly what was put in its slot",
      all(abs(np.degrees(st["joints"][n]) - v) < 1e-9 for n, v in want.items()))
check("no joint reads an unused slot (they hold 99 rad of junk)",
      all(abs(q) < 30 for q in st["joints"].values()))
check("mode_machine and the IMU come through", st["mode_machine"] == 7 and st["imu_rpy"])

print("\n  THE HEAD ENCODER SPLITS THE 4 DEGREES")
for head, label in ((0.0, "head at 0: the CAD puts the camera at 30 deg"),
                    (4.0, "head pitched 4 deg down: the CAD puts it at 34")):
    t, r = rs.camera_from_joints(rs.decode(fake_lowstate({"head_pitch_joint": head})))
    check(label, abs(t - (30.0 + head)) < 0.05, "%.2f deg, roll %+.2f" % (t, r))

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    rs.report(rs.decode(fake_lowstate({"head_pitch_joint": 4.0})))
out = buf.getvalue()
check("with the head at 4 deg, the report says the head explains the 4 deg",
      "+4.00 deg of the CAD" in out)
check("...and tells you to re-store the calibration at the real head angle",
      "--head-pitch 4.00" in out)

buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    rs.report(rs.decode(fake_lowstate({"left_elbow_joint": 125.0})))
check("a joint past its limit is flagged", "PAST LIMIT" in buf.getvalue())

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
