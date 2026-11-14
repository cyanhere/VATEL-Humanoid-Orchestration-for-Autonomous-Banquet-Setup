"""Forward kinematics for the Unitree R1 with the sensor helmet.

The engine here (load_urdf / find_chain / fk_chain / ...) is copied byte for byte from the G1
module: it parses any URDF and walks the parent/child tree, so it never needed to change. What
changed is everything that describes the ROBOT, and it changed in three ways that matter:

  1. The root link is "pelvis_link", not the G1's "pelvis".
  2. Two joints sit above both shoulders (waist_roll then waist_yaw), where the G1 had one.
     Functions therefore take an ANGLES DICT, not a single waist_yaw_angle.
  3. The camera hangs off a 2-DoF neck. On the G1 it was bolted to the torso, so one
     calibrated constant held forever and the waist angle was the only thing that moved it.
     Here head_pitch and head_yaw are in the middle of the chain, so their measured zeros are
     part of the camera calibration, and a head-joint zero error is indistinguishable from a
     mount error until you move the head and watch the plate appear to move with it.

The arm chain is the same five joints with the same names as the G1 23-DoF, which is the only
reason the IK ports at all.

Chain names come from r1_config.py. Do not write joint-name literals in here.
"""
import xml.etree.ElementTree as ET
import numpy as np


def load_urdf(path):
    """Parse all <joint> elements into a dict keyed by joint name."""
    tree = ET.parse(path)
    root = tree.getroot()

    joints = {}
    for j in root.findall("joint"):
        name = j.get("name")
        jtype = j.get("type")
        parent = j.find("parent").get("link")
        child = j.find("child").get("link")

        origin_el = j.find("origin")
        if origin_el is not None:
            xyz = np.array([float(v) for v in origin_el.get("xyz", "0 0 0").split()])
            rpy = np.array([float(v) for v in origin_el.get("rpy", "0 0 0").split()])
        else:
            xyz = np.zeros(3)
            rpy = np.zeros(3)

        axis_el = j.find("axis")
        axis = None
        if axis_el is not None:
            axis = np.array([float(v) for v in axis_el.get("xyz", "0 0 0").split()])

        limit_el = j.find("limit")
        limit = None
        if limit_el is not None and limit_el.get("lower") is not None:
            limit = (float(limit_el.get("lower")), float(limit_el.get("upper")))

        joints[name] = {
            "type": jtype, "parent": parent, "child": child,
            "xyz": xyz, "rpy": rpy, "axis": axis, "limit": limit,
        }
    return joints


# find_chain() does a BFS over every joint in the URDF, and the IK solver
# calls FK thousands of times (a finite-difference Jacobian over 11 joints
# re-runs both arms' chains on every iteration). The chain for a given
# (urdf, root, tip) never changes, so cache it -- this is the difference
# between dual-arm IK taking seconds and taking well under a second.
_CHAIN_CACHE = {}


def find_chain(joints, root_link, tip_link):
    """
    BFS over the parent->child joint graph to find the ordered list of
    joint names connecting root_link to tip_link. Raises if no path exists
    (e.g. a typo'd link name, or the two links are on different branches).

    Cached per (joints dict identity, root, tip) -- see _CHAIN_CACHE above.
    """
    from collections import deque

    cache_key = (id(joints), root_link, tip_link)
    if cache_key in _CHAIN_CACHE:
        return _CHAIN_CACHE[cache_key]

    # adjacency: parent_link -> [(joint_name, child_link), ...]
    adj = {}
    for jname, j in joints.items():
        adj.setdefault(j["parent"], []).append((jname, j["child"]))

    queue = deque([(root_link, [])])
    visited = {root_link}
    while queue:
        link, path = queue.popleft()
        if link == tip_link:
            _CHAIN_CACHE[cache_key] = path
            return path
        for jname, child in adj.get(link, []):
            if child not in visited:
                visited.add(child)
                queue.append((child, path + [jname]))

    raise ValueError(f"No joint chain found from '{root_link}' to '{tip_link}' "
                      f"-- check the link names against the URDF.")


def rpy_to_matrix(rpy):
    """URDF convention: fixed-axis roll-pitch-yaw, R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_angle_matrix(axis, angle):
    """Rodrigues' rotation formula for an arbitrary unit axis."""
    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0],
    ])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def homogeneous(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def joint_transform(joint, angle=0.0):
    """4x4 transform for one joint: its fixed origin, plus (for revolute)
    the extra rotation about its axis by the given angle."""
    R_origin = rpy_to_matrix(joint["rpy"])
    T_origin = homogeneous(R_origin, joint["xyz"])

    if joint["type"] == "fixed":
        return T_origin

    if joint["type"] in ("revolute", "continuous"):
        R_joint = axis_angle_matrix(joint["axis"], angle)
        return T_origin @ homogeneous(R_joint, np.zeros(3))

    raise ValueError(f"Unsupported joint type: {joint['type']}")


def fk_chain(joints, chain, joint_angles=None):
    """
    Compose the transform along a chain of joint names.
    joint_angles: dict {joint_name: angle_radians}, defaults to 0.0 for any
    joint not present (fixed joints ignore this entirely).
    """
    joint_angles = joint_angles or {}
    T = np.eye(4)
    for jname in chain:
        angle = joint_angles.get(jname, 0.0)
        T = T @ joint_transform(joints[jname], angle)
    return T


def fk_chain_cumulative(joints, chain, joint_angles=None):
    """
    Like fk_chain, but returns the transform after EVERY joint in the chain,
    as a list of (joint_name, T_root_child). Used by the self-collision check
    in solve_ik.py, which needs the intermediate link positions (shoulder,
    elbow) along each arm, not just the wrist at the end.
    """
    joint_angles = joint_angles or {}
    T = np.eye(4)
    out = []
    for jname in chain:
        T = T @ joint_transform(joints[jname], joint_angles.get(jname, 0.0))
        out.append((jname, T.copy()))
    return out


# --- END OF SHARED URDF ENGINE (byte-identical in G1_Plate_Perception) ---
# ==========================================================================
# R1-specific: everything below knows about THIS robot's shape
# ==========================================================================

import r1_config as cfg


def _angles(angles):
    return dict(angles or {})


def T(joints, root_link, tip_link, angles=None):
    """Generic: transform from root_link to tip_link at the given joint angles."""
    return fk_chain(joints, find_chain(joints, root_link, tip_link), _angles(angles))


def T_pelvis_camera(joints, angles=None):
    """pelvis_link -> d435_link (camera BODY frame: x along the optical axis, y left, z up).

    Straight from the URDF, i.e. the nominal pose from the helmet CAD. Depends on
    waist_roll, waist_yaw, head_pitch and head_yaw -- pass them all in `angles`, or
    the result silently describes a robot standing perfectly straight with its head
    level, which it never is.
    """
    return T(joints, cfg.ROOT_LINK, cfg.CAMERA_LINK, angles)


def T_pelvis_lidar(joints, angles=None):
    """pelvis_link -> jt128_link (z = spin axis, tilted 24 deg back from vertical)."""
    return T(joints, cfg.ROOT_LINK, cfg.LIDAR_LINK, angles)


# Set this once calibrate_camera has been ported and run on the R1. It replaces the
# nominal d435 mount pose AND carries the optical convention, exactly as the G1's
# T_TORSO_CAMERA_CALIBRATED did -- do not stack an axis-convention matrix on top of it.
#
# It is expressed head_yaw_link -> optical, NOT torso -> optical. That differs from the
# G1 on purpose: the frame it hangs off has to be the one the camera is bolted to, and
# on the R1 that is the head.
T_HEADYAW_OPTICAL_CALIBRATED = None


def T_pelvis_optical(joints, angles=None, allow_nominal=False):
    """pelvis_link <- camera OPTICAL frame (x right, y down, z forward).

    This is the transform perception output goes through, so it is the one that is
    worth being paranoid about. Until the R1 has been calibrated it is only the CAD
    nominal, and on the G1 the equivalent nominal was wrong by 23 deg and put the
    plate 23 cm from where it actually was.

    So it refuses by default. Pass allow_nominal=True to say you know you are using
    an uncalibrated chain -- fine for geometry tests, not fine for driving a hand.
    """
    if T_HEADYAW_OPTICAL_CALIBRATED is not None:
        head = T(joints, cfg.ROOT_LINK, "head_yaw_link", angles)
        return head @ T_HEADYAW_OPTICAL_CALIBRATED
    if not allow_nominal:
        raise RuntimeError(
            "T_pelvis_optical: no calibration for the R1 yet, so this would return the\n"
            "CAD nominal. The G1's nominal was 23 deg out and placed the plate 23 cm\n"
            "wrong. Pass allow_nominal=True if you are only testing geometry, or set\n"
            "T_HEADYAW_OPTICAL_CALIBRATED once calibrate_camera has been run on the R1.")
    return T(joints, cfg.ROOT_LINK, cfg.CAMERA_OPTICAL_LINK, angles)


def T_pelvis_wrist(joints, angles=None, arm="left"):
    """pelvis_link -> {arm}_wrist_roll_link.

    NOTE this is the wrist JOINT, not the hand. R1.urdf has no hand. See
    r1_config.tool_offset for why that is not papered over with a default.
    """
    return T(joints, cfg.ROOT_LINK, cfg.arm_tip_link(arm), angles)


def T_pelvis_tool(joints, angles=None, arm="left"):
    """pelvis_link -> the hand's CONTACT POINT. Raises until the hand is measured."""
    Tw = T_pelvis_wrist(joints, angles, arm)
    out = Tw.copy()
    out[:3, 3] = Tw[:3, :3] @ cfg.tool_offset(arm) + Tw[:3, 3]
    return out


ARM_KEYPOINT_JOINTS = ["shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint"]


def arm_keypoints(joints, angles=None, arm="left"):
    """{short_joint_name: xyz in pelvis frame} along one arm, for self-collision checks."""
    chain = find_chain(joints, cfg.ROOT_LINK, cfg.arm_tip_link(arm))
    wanted = {"%s_%s" % (arm, k): k for k in ARM_KEYPOINT_JOINTS}
    out = {}
    for jname, Tj in fk_chain_cumulative(joints, chain, _angles(angles)):
        if jname in wanted:
            out[wanted[jname]] = Tj[:3, 3].copy()
    return out


def shoulder_positions(joints, angles=None):
    """Both shoulder_pitch origins in pelvis frame: {"left": xyz, "right": xyz}."""
    return {arm: T(joints, cfg.ROOT_LINK, "%s_shoulder_pitch_link" % arm,
                   angles)[:3, 3].copy()
            for arm in cfg.ARMS}


# --------------------------------------------------------------------------
# aiming the head
# --------------------------------------------------------------------------

def aim_head_at(joints, target_pelvis, angles=None, iters=50, tol_rad=1e-7):
    """Head angles that point the camera's optical axis at a point in pelvis frame.

    The G1 could not do this at all -- its camera was fixed, and whatever it happened to
    see was what you got. On the R1 the plate can be centred in frame instead.

    Returns {"head_pitch_joint": p, "head_yaw_joint": y, "clamped": bool,
             "residual_deg": float}.

    `clamped` means the target is outside the neck's travel and the angles returned are
    the closest the head can get -- check it, because a clamped aim may not have the
    plate in frame at all.

    Note this changes what is VISIBLE, not how foreshortened it is. The angle between the
    viewing ray and the table normal is fixed by where the robot stands relative to the
    plate; pointing the head down does not unsquash the ellipse. What it buys is being
    able to see a plate that is close in, and standing close is what reduces the angle.

    Solved by iteration because the camera's POSITION moves with the head too, so the
    direction it must point depends on the answer. The camera sits 95 mm forward of the
    yaw axis, so a yaw swings it sideways and the map converges geometrically rather than
    instantly: 8 passes still left 0.08 deg on an off-axis target, 16 reached 0.0002 deg.
    The default is deliberately generous because each pass is only a little matrix algebra.
    """
    lim = cfg.joint_limits(joints)
    p_lo, p_hi = lim["head_pitch_joint"]
    y_lo, y_hi = lim["head_yaw_joint"]

    base = _angles(angles)
    base.pop("head_pitch_joint", None)
    base.pop("head_yaw_joint", None)

    target = np.asarray(target_pelvis, dtype=float)
    pitch = yaw = 0.0
    clamped = False

    # the optical axis in head_yaw_link, which is fixed by the helmet's 30 deg nose tilt
    ct = cfg.NOMINAL_CAMERA_PITCH_RAD
    a_local = np.array([np.cos(ct), 0.0, -np.sin(ct)])
    ax, az = a_local[0], a_local[2]

    for _ in range(max(1, iters)):
        ang = dict(base)
        ang["head_pitch_joint"] = pitch
        ang["head_yaw_joint"] = yaw

        # where the camera is now, and the frame the head joints act in
        cam = T_pelvis_camera(joints, ang)[:3, 3]
        T_wy = T(joints, cfg.ROOT_LINK, "waist_yaw_link", base)

        u = target - cam
        n = np.linalg.norm(u)
        if n < 1e-9:
            break
        u = T_wy[:3, :3].T @ (u / n)      # desired optical axis, in waist_yaw_link

        # optical axis as a function of the joints is Ry(pitch) @ Rz(yaw) @ a_local,
        # since both head joint origins have zero rpy and the axes are +y then +z.
        s = np.clip(u[1] / ax, -1.0, 1.0)
        yaw_new = float(np.arcsin(s))
        c = ax * np.cos(yaw_new)
        pitch_new = float(np.arctan2(az, c) - np.arctan2(u[2], u[0]))
        pitch_new = (pitch_new + np.pi) % (2 * np.pi) - np.pi

        yaw_c = min(max(yaw_new, y_lo), y_hi)
        pitch_c = min(max(pitch_new, p_lo), p_hi)
        clamped = (abs(yaw_c - yaw_new) > 1e-9) or (abs(pitch_c - pitch_new) > 1e-9)

        done = abs(pitch_c - pitch) < tol_rad and abs(yaw_c - yaw) < tol_rad
        pitch, yaw = pitch_c, yaw_c
        if done:
            break

    ang = dict(base)
    ang["head_pitch_joint"] = pitch
    ang["head_yaw_joint"] = yaw
    Tc = T_pelvis_camera(joints, ang)
    axis = Tc[:3, :3] @ np.array([1.0, 0.0, 0.0])
    to_target = target - Tc[:3, 3]
    nt = np.linalg.norm(to_target)
    resid = float(np.degrees(np.arccos(np.clip(
        axis @ (to_target / nt), -1.0, 1.0)))) if nt > 1e-9 else 0.0

    return {"head_pitch_joint": pitch, "head_yaw_joint": yaw,
            "clamped": bool(clamped), "residual_deg": resid}


if __name__ == "__main__":
    joints = load_urdf(cfg.URDF_PATH)
    print("R1 FK self-check")
    print("=" * 60)

    Tc = T_pelvis_camera(joints)
    print("camera in pelvis at zero pose: %s" % np.array2string(Tc[:3, 3], precision=4))
    print("optical axis                 : %s"
          % np.array2string(Tc[:3, :3] @ np.array([1.0, 0, 0]), precision=4))

    for arm in cfg.ARMS:
        w = T_pelvis_wrist(joints, arm=arm)[:3, 3]
        print("%-5s wrist at zero pose      : %s" % (arm, np.array2string(w, precision=4)))

    print()
    for tgt in ([0.35, 0.0, 0.30], [0.25, 0.20, 0.30], [0.9, 0.0, 0.9]):
        r = aim_head_at(joints, tgt)
        print("aim at %-18s pitch %+.3f yaw %+.3f  clamped=%-5s residual %.2f deg"
              % (np.array2string(np.array(tgt), precision=2), r["head_pitch_joint"],
                 r["head_yaw_joint"], r["clamped"], r["residual_deg"]))
