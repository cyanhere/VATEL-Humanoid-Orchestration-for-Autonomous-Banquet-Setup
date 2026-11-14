"""Fixed robot and setup constants. Python 3.8, numpy 1.17."""
import os

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))      # ~/project/grasp/pick
MODEL_DIR = os.path.join(ROOT, "model")
URDF = os.path.join(MODEL_DIR, "r1_revo2_upper_both.urdf")
ROBOT_INFO = os.path.join(MODEL_DIR, "robot_info.json")
PARAMS_DIR = os.path.join(ROOT, "params")
REFS_DIR = os.path.join(ROOT, "refs")
LOG_DIR = os.path.join(ROOT, "logs")
PERCEPTION_DIR = os.path.expanduser("~/R1_Plate_Perception")         # used in place, never modified
# Per-arm joint zero offsets (real angle = measured + offset), from calibration/arm_offsets_fit.py --save.
# Applied in pk.model.Robot.joint_values; commands on the wire stay in motor angles.
# Off unless DEXPICK_ARM_OFFSETS=1 (this file) or =<path>.
ARM_OFFSETS = os.path.join(MODEL_DIR, "arm_offsets.json")
ARM_OFFSETS_ON = os.environ.get("DEXPICK_ARM_OFFSETS", "0")
# Head shift (mm, pelvis frame) added to every head-camera location so that the head and the arm agree.
# Off unless DEXPICK_HEAD_SHIFT=1 (this file) or =<path>; read when a Kin is made.
HEAD_SHIFT = os.path.join(MODEL_DIR, "head_shift.json")


def head_shift_mm(which=None):
    """(3,) mm: which None -> the environment (DEXPICK_HEAD_SHIFT: "0"/"" none, "1" HEAD_SHIFT, else a path)."""
    import json
    if which is None:
        on = os.environ.get("DEXPICK_HEAD_SHIFT", "0")
        which = None if on in ("", "0") else (HEAD_SHIFT if on == "1" else on)
    if not which:
        return np.zeros(3)
    return np.array(json.load(open(which))["shift_mm"], float)

# ---------------------------------------------------------------- joints (the order of the dp_play CSVs)
ARM_NAMES = ["L_sh_pitch", "L_sh_roll", "L_sh_yaw", "L_elbow", "L_wrist_roll",
             "R_sh_pitch", "R_sh_roll", "R_sh_yaw", "R_elbow", "R_wrist_roll", "waist_yaw"]
N_ARM = 11                       # 5 left + 5 right + waist yaw
LEFT = slice(0, 5)
RIGHT = slice(5, 10)
WAIST = 10
URDF_ARM = ["shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint"]
# rt/lowstate and rt/arm_sdk slots (unitree dds_wrapper r1/defines.h; checked on the robot)
ARM_SLOT = [15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 13]
HEAD_SLOT = [29, 30]             # head pitch (+ = down), head yaw (+ = left)
WAIST_ROLL_SLOT = 12             # read only (not part of Unitree's arm_sdk example; the robot balances with it)

# URDF limits (rad), as dp_play; every command is kept LIMIT_MARGIN inside them
ARM_LO = np.array([-3.1416, -0.22689, -1.9199, -0.97564, -1.9199,
                   -3.1416, -2.47849, -1.9199, -0.97564, -1.9199, -2.618])
ARM_HI = np.array([2.0944, 2.4784, 1.9199, 2.1852, 1.9199,
                   2.0944, 0.2268, 1.9199, 2.1852, 1.9199, 2.618])
LIMIT_MARGIN = 0.035             # 2 deg
HEAD_LO = np.array([-0.608, -1.987])
HEAD_HI = np.array([0.608, 1.987])

# arm_sdk gains of Unitree's R1 example (dp_play uses the same; every hardware run so far used them)
KP = np.array([50, 50, 40, 40, 30, 50, 50, 40, 40, 30, 50], float)
KD = np.array([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3], float)
HEAD_KP, HEAD_KD = 15.0, 1.0
GAIN_SCALE_KP = (0.5, 2.0)       # what `server.py --kp-scale` / `gains` may set (arm joints only; the waist keeps its gains)
GAIN_SCALE_KD = (0.5, 3.0)


def arm_gains(kp_scale=1.0, kd_scale=1.0):
    """(kp, kd) for the 11 arm_sdk joints: Unitree's arm gains scaled, the waist yaw unchanged."""
    s = np.ones(N_ARM)
    s[WAIST] = 0.0
    return KP * (1.0 + s * (kp_scale - 1.0)), KD * (1.0 + s * (kd_scale - 1.0))


# friction feed-forward (server.py --fric / `fric` command): off unless asked
FRIC_V0 = 0.02                   # rad/s: planned speed at which the lead is 76 % of the full friction (tanh), as the fit
FRIC_TAU = 0.05                  # s: the lead is low-passed, so a reversal of the planned motion is not a step
FRIC_MAX = 1.0                   # never more than the fitted friction

# ---------------------------------------------------------------- hands (BrainCo Revo2 via brainco_hand_server)
FINGERS = ["Thumb", "Thumb_aux", "Index", "Middle", "Ring", "Pinky"]     # service order, 0 open.. 1 closed
TOUCH_FINGERS = ["Thumb", "Index", "Middle", "Ring", "Pinky"]            # rt/brainco/<side>/touch order
# 0..1 command -> actuated joint angle (rad), the simulation's assumption (tools/log_to_csv.py), not measured
HAND_FULL_RAD = np.array([1.0472, 1.55334, 1.41, 1.41, 1.41, 1.41])

# ---------------------------------------------------------------- safety limits enforced by the server
V_MAX = 0.8                      # rad/s, any arm joint, any command
A_MAX = 3.0                      # rad/s^2
V_HOME = 0.25                    # rad/s, the server's first move to HOME after `take` (from wherever the robot had the arms)
V_RETURN = 0.5                   # rad/s, the server's STRAIGHT way back to HOME after a completed test
V_REWIND = 0.25                  # rad/s, back along the path (abort, Ctrl-C, client gone, `home path`): at 0.5 it left the path by 0.048 rad (the simulated arm), 0.25 keeps it < 0.03
HOME_FILE = os.path.join(PARAMS_DIR, "home_neutral.json")      # HOME, the neutral pose (was premotion.json)
PREGRIP_FILE = os.path.join(PARAMS_DIR, "premotion.json")      # the pre-grip pose: the older stages and tools start here
V_HEAD = 0.35                    # rad/s
HEAD_YAW_APPROACH = 0.0698          # rad (4 deg): every head yaw move ENDS turning + (left) by at least this --
                                    # R1_Plate_Perception's calibration was captured that way (r1_head.
                                    # YAW_APPROACH_DEG); the yaw has ~1 deg of play that depends on the side it came from
V_HAND = 1.5                     # 0..1 units per second
HAND_LEAD_MAX = 0.12             # a closing finger is never commanded more than this ahead of where it is
ERR_TRIP = 0.25                  # rad: |planned - measured| this large for ERR_TRIP_S -> freeze where the arm is
ERR_TRIP_S = 0.25
ERR_TRIP_OWN = 0.15              # rad, during the server's own moves (to HOME, returns, rewinds, holding)
TAU_TRIP = 15.0                  # N m, motor torque estimate
LIMP_TAU = 0.25                  # N m: every arm motor below this at full weight = limp, the robot's mode ignores arm_sdk
F_TRIP = 45.0                    # N: force on a hand (from joint lag minus gravity) this large for F_TRIP_S -> freeze
F_TRIP_S = 0.15
LEAN_TRIP = 0.15                 # rad, IMU roll/pitch change since start
STATE_STALE_S = 0.25             # rt/lowstate older than this -> freeze
CLIENT_TIMEOUT_S = 0.6           # no command from the owning client for this long -> it is gone -> rewind home

# ---------------------------------------------------------------- loop and ports (localhost UDP, JSON)
RATE_HZ = 100.0
HAND_EVERY = 2                   # hand commands every 2nd cycle (50 Hz; the service runs at 100 Hz)
SERVER_PORT = 9701               # clients -> server
HOST = "127.0.0.1"

# ---------------------------------------------------------------- the task setup (simulation reference)
PELVIS_SIM = 0.71                # pelvis above the floor in the reference runs (m)
TABLE_SIM = 0.747                # table of the reference runs
STACK_XY_SIM = (0.36, 0.06)      # stack centre in the reference runs, pelvis frame
PLATE_PITCH_SIM = 0.0068         # stacking pitch in the plate model

# head pose used to look at the stack (rad): pitch + = down, yaw + = to the robot's left
HEAD_LOOK = np.array([0.55, 0.30])


def clamp_arm(q):
    return np.minimum(np.maximum(q, ARM_LO + LIMIT_MARGIN), ARM_HI - LIMIT_MARGIN)


def clamp_head(h):
    return np.minimum(np.maximum(h, HEAD_LO + 0.02), HEAD_HI - 0.02)
