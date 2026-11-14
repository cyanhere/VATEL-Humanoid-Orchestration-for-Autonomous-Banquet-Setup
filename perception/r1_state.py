"""Read-only: what the R1 reports about itself. Moves nothing, publishes nothing.

    python3 r1_state.py <network_interface>          # one reading
    python3 r1_state.py <network_interface> --watch  # keep printing

Subscribes to rt/lowstate and prints every joint by name (from Unitree's own R1 motor map,
in r1_config.MOTOR_SLOT), the pelvis IMU, and -- the reason this exists before any motion
code -- what the joints say about the CAMERA:

The floor check found the camera 34.0 deg below horizontal where the CAD says 30.0. That 4 deg
is the head joint's actual angle, or the helmet mount, or the body leaning, and the floor alone
cannot tell which. The head's encoder can. If the head reads 4 deg down, the mount is fine and
the calibration should be stored against the real head angle; if it reads 0, the 4 deg is in
the mount. It matters as soon as the head moves -- and to see within the arm's reach, it has to.

Needs unitree_sdk2py and a DDS connection to the robot (Ethernet, 192.168.123.x).
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import r1_config as cfg


def decode(msg):
    """A LowState_ (or anything shaped like one) -> plain dict of joint angles and IMU."""
    joints = {name: float(msg.motor_state[slot].q) for name, slot in cfg.MOTOR_SLOT.items()}
    try:
        rpy = [float(v) for v in msg.imu_state.rpy]
    except (AttributeError, TypeError):
        rpy = None
    return {"joints": joints, "imu_rpy": rpy,
            "mode_machine": int(getattr(msg, "mode_machine", -1))}


def camera_from_joints(state):
    """Camera tilt and roll the CAD predicts at the robot's ACTUAL waist and head angles."""
    import camera_calibration as cc
    import urdf_fk as fk

    J = fk.load_urdf(cfg.URDF_PATH)
    angles = {n: state["joints"][n] for n in cfg.CAMERA_CHAIN_JOINTS}
    T = fk.T_pelvis_optical(J, angles, allow_nominal=True)
    return cc._tilt_roll(T[:3, :3].T @ np.array([0.0, 0.0, 1.0]))


def report(state):
    lim = cfg.joint_limits()
    D = np.degrees
    print("R1 state   (mode_machine %d)" % state["mode_machine"])
    groups = [("waist", cfg.TRUNK_JOINTS), ("head", cfg.HEAD_JOINTS),
              ("left arm", cfg.arm_joints("left")), ("right arm", cfg.arm_joints("right"))]
    for g, names in groups:
        print("  %s" % g)
        for n in names:
            q = state["joints"][n]
            lo, hi = lim[n]
            warn = "   <-- AT/PAST LIMIT" if not (lo + 0.02 <= q <= hi - 0.02) else ""
            print("    %-26s %+7.2f deg   limit [%+6.1f, %+6.1f]%s"
                  % (n.replace("_joint", ""), D(q), D(lo), D(hi), warn))
    if state["imu_rpy"]:
        r, p, y = state["imu_rpy"]
        print("  pelvis IMU   roll %+.2f deg   pitch %+.2f deg   (yaw %+.1f, not used)"
              % (D(r), D(p), D(y)))
    import floor_frame as ff
    print("  " + ff.describe(ff.pelvis_up(state)[1]).replace("\n", "\n  "))
    print("               (legs: only meaningful with both feet flat on a level floor)")

    import camera_calibration as cc
    t_j, r_j = camera_from_joints(state)
    print("\ncamera, from the joints + helmet CAD: %.2f deg below horizontal, roll %+.2f"
          % (t_j, r_j))
    cal = cc.load()
    if cal is None:
        return
    t_f, r_f = cal["measured_tilt_deg"], cal["measured_roll_deg"]
    hp = D(state["joints"]["head_pitch_joint"])
    print("camera, from the floor (calibration): %.2f deg below horizontal, roll %+.2f"
          % (t_f, r_f))
    print("  -> the head itself accounts for %+.2f deg of the CAD's 30;" % (t_j - 30.0))
    print("     what is left, %+.2f deg tilt and %+.2f deg roll, is the mount or a lean."
          % (t_f - t_j, r_f - r_j))
    stored = D(cal.get("angles_at_calibration", {}).get("head_pitch_joint", 0.0))
    if abs(hp - stored) > 0.5:
        print("\n  The calibration was stored assuming head pitch %.1f deg, but the head is at"
              "\n  %.1f deg. Re-run it against the real angle so a later head move is right:"
              "\n    python check_mount.py --calibrate --head-pitch %.2f --head-yaw %.2f"
              % (stored, hp, hp, D(state["joints"]["head_yaw_joint"])))


def read_once(timeout=5.0):
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    got = {}

    def on_state(msg):
        got["msg"] = msg

    sub = ChannelSubscriber(cfg.LOWSTATE_TOPIC, LowState_)
    sub.Init(on_state, 10)
    t0 = time.time()
    while "msg" not in got:
        if time.time() - t0 > timeout:
            raise SystemExit(
                "No %s within %.0f s. Check: the Ethernet cable, that this computer has a "
                "192.168.123.x address, and that the interface name passed is the one on "
                "that network (on Windows it can be the adapter's IP address)."
                % (cfg.LOWSTATE_TOPIC, timeout))
        time.sleep(0.05)
    return decode(got["msg"])


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", nargs="?", help="network interface on the robot's network")
    ap.add_argument("--watch", action="store_true", help="keep printing once a second")
    ap.add_argument("--json", metavar="OUT.json", help="also save the reading")
    args = ap.parse_args(argv)

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    if args.interface:
        ChannelFactoryInitialize(0, args.interface)
    else:
        ChannelFactoryInitialize(0)

    while True:
        state = read_once()
        print("=" * 72)
        report(state)
        if args.json:
            with open(args.json, "w") as f:
                json.dump(state, f, indent=2)
        if not args.watch:
            return 0
        time.sleep(1.0)


if __name__ == "__main__":
    sys.exit(main())
