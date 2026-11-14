"""Everything about the R1's shape that the rest of this package needs to know.

WHY THIS FILE EXISTS
--------------------
The G1 module hard-codes the string "waist_yaw_joint" in about a dozen places across
solve_ik.py, plan_approach.py and compute_wrist_target.py, because on the G1 23-DoF there is
exactly one joint shared above both shoulders. The R1 has TWO (waist_roll then waist_yaw), and
it also has a 2-DoF neck the camera hangs off, which the G1 did not have at all.

Ported code that keeps those names as literals is code that works until someone reads it and
believes it. So the chain is described once, here, and every module asks this file.

Everything below is read from R1.urdf or measured, never assumed. Where a number genuinely is
not known yet, this file RAISES instead of supplying a plausible default -- see TOOL_OFFSET.
"""

import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "R1.urdf")

# --------------------------------------------------------------------------
# the kinematic chain
# --------------------------------------------------------------------------

ROOT_LINK = "pelvis_link"          # NOT "pelvis" -- that is the G1's name

# Shared above both shoulders, in chain order. The G1 had only waist_yaw.
TRUNK_JOINTS = ["waist_roll_joint", "waist_yaw_joint"]

# The neck, in chain order. Branches off waist_yaw_link, parallel to the arms.
# The camera is downstream of BOTH of these, which is the big structural change
# from the G1: there the camera was bolted to the torso and one calibrated
# constant covered it forever. Here, error in a head joint's zero feeds
# straight into where the plate appears to be.
HEAD_JOINTS = ["head_pitch_joint", "head_yaw_joint"]

# Per arm, in chain order. Same five as the G1 23-DoF, which is why the IK and
# FK port at all.
ARM_JOINT_KINDS = ["shoulder_pitch_joint", "shoulder_roll_joint",
                   "shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint"]

ARMS = ("left", "right")


def arm_joints(arm):
    """['left_shoulder_pitch_joint', ...] in chain order."""
    return ["%s_%s" % (arm, k) for k in ARM_JOINT_KINDS]


def arm_tip_link(arm):
    """Where the URDF's arm chain ends.

    R1.urdf has NO hand links -- the chain stops at the wrist roll. The
    dexterous hand extends past this by an amount nobody has measured yet;
    see TOOL_OFFSET.
    """
    return "%s_wrist_roll_link" % arm


# Joints that move the camera, in chain order, root first.
CAMERA_CHAIN_JOINTS = TRUNK_JOINTS + HEAD_JOINTS

# --------------------------------------------------------------------------
# sensor frames (added to R1.urdf from the helmet CAD)
# --------------------------------------------------------------------------

CAMERA_LINK = "d435_link"              # body frame: x along the optical axis, y left, z up
CAMERA_OPTICAL_LINK = "d435_optical_frame"   # x right, y down, z forward
LIDAR_LINK = "jt128_link"

# Nominal, from helmet_geometry.py, cross-checked against this URDF's head joint
# origins to 0.0000 mm. Kept here so a test can assert the URDF was not edited
# by hand into something else.
NOMINAL_CAMERA_IN_HEAD_YAW = np.array([0.095000, 0.0, 0.050000])
NOMINAL_CAMERA_PITCH_RAD = 0.523599     # 30.0 deg down
NOMINAL_LIDAR_IN_HEAD_YAW = np.array([0.003835, 0.0, 0.140896])
NOMINAL_LIDAR_PITCH_RAD = 0.418879      # 24.0 deg back

# --------------------------------------------------------------------------
# what is NOT known yet
# --------------------------------------------------------------------------

_TOOL_OFFSET = None     # a hand measured into code, if ever; hand_frame.json comes first
HAND_FRAME_PATH = os.path.join(HERE, "hand_frame.json")


def _hand_frame(arm):
    """What reach_plate.py measure-hand saved for this arm, or None."""
    import json
    if not os.path.exists(HAND_FRAME_PATH):
        return None
    with open(HAND_FRAME_PATH) as f:
        return json.load(f).get(arm)


def tool_offset(arm):
    """Contact point in the wrist_roll frame: where the hand actually touches things.

    UNSET UNTIL MEASURED. On the G1 this was a measured 0.2492 m, and the bug it fixed
    was not subtle: IK drove the WRIST onto the plate rim, which put the hand ~21 cm
    past the rim, through the plate and into the table. It also made the reach check
    report 0.33 m when the hand could touch 0.53 m.

    The R1's Revo 2 hand is not in R1.urdf. `reach_plate.py <iface> measure-hand <arm>`
    holds the arm in a known pose, takes three tape measurements of the hand's pinch
    point, and saves it to hand_frame.json -- per arm, because the two hands are mirror
    images mounted by hand, not one value reflected.
    """
    hf = _hand_frame(arm)
    if hf is not None:
        return np.asarray(hf["tool_m"], float)
    if _TOOL_OFFSET is None:
        raise NotImplementedError(
            "tool_offset(%r): the R1's hand has not been measured.\n"
            "R1.urdf's arm chain ends at %s, with no hand beyond it. Run\n"
            "    python3 reach_plate.py eth10 measure-hand %s --go\n"
            "See the docstring for why this does not just return a guess."
            % (arm, arm_tip_link(arm), arm))
    if arm == "right":
        return _TOOL_OFFSET
    return _TOOL_OFFSET * np.array([1.0, -1.0, 1.0])


def palm_axis(arm):
    """The palm's outward normal in the wrist_roll frame (where the palm faces), from
    the same measurement. None until measured."""
    hf = _hand_frame(arm)
    return None if hf is None or hf.get("palm_axis") is None else np.asarray(hf["palm_axis"], float)


# The SDK's motor array -- from Unitree's own R1 code, not by analogy with the G1:
#   unitree_sdk2_python  example/r1/low_level/r1_low_level_example.py
#   xr_teleoperate       teleop/robot_control/robot_arm.py, class R1_A5_ArmController
# Both agree. The DDS message is the same unitree_hg LowCmd/LowState as the G1's, with
# 35 slots, of which the R1 uses 26 -- but the NUMBERING differs, and so does how arm_sdk
# is engaged:
#
#   G1: arm_sdk blend weight written as the q of an unused joint, slot 29
#   R1: arm_sdk blend weight is msg.mode_pr, 0..100 percent; slot 29 is HEAD PITCH
#
# so G1 arm code run unchanged on an R1 would command the head pitch motor to 1 rad
# (57 deg) -- past its 36 deg limit -- every time it engaged.
NUM_SLOTS = 35
NUM_MOTORS_FROM_URDF = 26
MOTOR_SLOT = {
    "left_hip_pitch_joint": 0, "left_hip_roll_joint": 1, "left_hip_yaw_joint": 2,
    "left_knee_joint": 3, "left_ankle_pitch_joint": 4, "left_ankle_roll_joint": 5,
    "right_hip_pitch_joint": 6, "right_hip_roll_joint": 7, "right_hip_yaw_joint": 8,
    "right_knee_joint": 9, "right_ankle_pitch_joint": 10, "right_ankle_roll_joint": 11,
    "waist_roll_joint": 12, "waist_yaw_joint": 13,
    "left_shoulder_pitch_joint": 15, "left_shoulder_roll_joint": 16,
    "left_shoulder_yaw_joint": 17, "left_elbow_joint": 18, "left_wrist_roll_joint": 19,
    "right_shoulder_pitch_joint": 22, "right_shoulder_roll_joint": 23,
    "right_shoulder_yaw_joint": 24, "right_elbow_joint": 25, "right_wrist_roll_joint": 26,
    "head_pitch_joint": 29, "head_yaw_joint": 30,
}
LOWSTATE_TOPIC = "rt/lowstate"
ARM_SDK_TOPIC = "rt/arm_sdk"     # the robot's own controller keeps balancing
LOWCMD_TOPIC = "rt/lowcmd"       # releases that controller: robot must be on a stand

# Gains Unitree's teleoperation uses for these joints over arm_sdk.
ARM_SDK_GAINS = {
    "shoulder_pitch_joint": (50.0, 2.0), "shoulder_roll_joint": (50.0, 2.0),
    "shoulder_yaw_joint": (40.0, 2.0), "elbow_joint": (40.0, 2.0),
    "wrist_roll_joint": (30.0, 2.0),
    "head_pitch_joint": (15.0, 1.0), "head_yaw_joint": (15.0, 1.0),
    "waist_roll_joint": (50.0, 3.0), "waist_yaw_joint": (50.0, 3.0),
}


def motor_index():
    """{urdf joint name: SDK slot}, from Unitree's R1 code (see above)."""
    return dict(MOTOR_SLOT)


# --------------------------------------------------------------------------
# limits, straight from the URDF
# --------------------------------------------------------------------------

def joint_limits(joints=None):
    """{joint_name: (lower, upper)} for every limited joint in R1.urdf."""
    if joints is None:
        from urdf_fk import load_urdf
        joints = load_urdf(URDF_PATH)
    return {n: j["limit"] for n, j in joints.items() if j.get("limit")}


if __name__ == "__main__":
    from urdf_fk import load_urdf

    joints = load_urdf(URDF_PATH)
    lim = joint_limits(joints)
    print("R1.urdf: %d joints, %d limited" % (len(joints), len(lim)))
    print()
    for group, names in (("trunk", TRUNK_JOINTS), ("head", HEAD_JOINTS),
                         ("left arm", arm_joints("left"))):
        print("%-9s" % group, end="")
        for n in names:
            lo, hi = lim[n]
            print("  %s [%+.2f %+.2f]" % (n.replace("_joint", ""), lo, hi), end="")
        print()
    print()
    for f in (tool_offset, motor_index):
        try:
            f("left") if f is tool_offset else f()
        except NotImplementedError as e:
            print("%s() correctly refuses:" % f.__name__)
            print("   ", str(e).splitlines()[0])
