"""The camera's pose on the robot, corrected by what the camera measured -- not just the CAD.

WHAT THE ROBOT FRAME IS
-----------------------
`pelvis_link`, the R1's root: x forward, y left, z up. Its origin sits directly above the point
on the floor midway between the two ankle pivots (within 2 mm, from R1.urdf), about 0.71 m up
when the robot stands. That is the frame the manipulation side works in.

WHERE THE CAMERA'S POSE COMES FROM
----------------------------------
Two sources, each trusted only for what it can actually tell:

  the helmet CAD   -> where the camera is (x, y, z on the head) and which way it faces (yaw)
  a level surface  -> how the camera is tipped (pitch and roll) and how high it is

The first on-robot mount check found the camera 34.0 deg below horizontal where the CAD said
30.0, with -2.3 deg of roll. Four degrees is ~40 mm at the plate's range, several plate
pitches, so the CAD's tilt cannot be used as-is. A level surface -- the floor, or a table --
measures exactly that tilt and roll, in the camera's own frame, at the camera's current head
pose. That correction is applied as a rotation of the camera about its own centre.

WHAT IT CANNOT SEE
------------------
A level surface looks the same however the camera is turned about the vertical, so YAW is not
measured -- it stays at the CAD's (straight ahead, which the helmet's symmetry supports). Each
degree of yaw error moves a plate sideways by ~1 cm at 0.6 m. Check it once with a tape: put
the stack on the robot's centreline and see that y comes out near 0.

THE LEAN (changed 23 Oct)
-------------------------
This used to assume the pelvis is level while calibrating. It is not: standing in balance
the IMU read it pitched 3.0 deg forward (22 Oct). A level-pelvis calibration folds that lean
into the camera -- right in the FLOOR's frame (so a tape check agrees), 3 deg wrong in the
pelvis frame the arms use. Now the floor's up in pelvis_link comes from the robot (the IMU,
cross-checked by the legs: floor_frame.py) and only what is left over is put in the mount.

ONE POSE IS NOT ENOUGH (changed 23 Oct)
---------------------------------------
A level surface seen at ONE head pose cannot tell an error in the mount (above the yaw joint,
turns with the head) from one below it (the yaw axis tipped, a head pitch zero, a lean): both
tip the camera the same way at that pose. Turn the head and they come apart -- a 38-pose
sweep on 22 Oct found the table's height drifting by -27.6 mm x sin(head yaw) with the
one-pose calibration. calibrate_sweep.py fits both, over many head poses; this file holds
the model (`neck_tilt_rad`, below the yaw joint) and applies it wherever a camera pose is
needed. Calibrations without it (one pose) still load and mean exactly what they did.
"""

import datetime
import json
import os

import numpy as np

import r1_config as cfg
import urdf_fk as fk

HERE = os.path.dirname(os.path.abspath(__file__))
CAL_PATH = os.path.join(HERE, "camera_calibration.json")


def _rotation_between(a, b):
    """The smallest rotation taking unit vector a onto unit vector b."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(a, b)
    s = np.linalg.norm(v)
    c = float(a @ b)
    if s < 1e-12:
        if c > 0:
            return np.eye(3)
        # opposite: any perpendicular axis, 180 deg
        p = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        k = np.cross(a, p)
        k /= np.linalg.norm(k)
        return 2.0 * np.outer(k, k) - np.eye(3)
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = np.arctan2(s, c)
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * (K @ K)


def rot(w):
    """Rotation matrix for rotation vector w (rad)."""
    w = np.asarray(w, float)
    a = float(np.linalg.norm(w))
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * (K @ K)


def T_pelvis_headyaw(joints, angles=None, neck=(0.0, 0.0)):
    """pelvis_link <- head_yaw_link, with the yaw axis tipped by `neck` (rad, a rotation
    about head_pitch_link's x and y, at the yaw joint). neck = 0: the URDF exactly."""
    a = dict(angles or {})
    T_hp = fk.T(joints, cfg.ROOT_LINK, "head_pitch_link", a)
    j = joints["head_yaw_joint"]
    R = fk.rpy_to_matrix(j["rpy"]) @ rot([neck[0], neck[1], 0.0]) @         fk.axis_angle_matrix(j["axis"], a.get("head_yaw_joint", 0.0))
    return T_hp @ fk.homogeneous(R, j["xyz"])


def predict_plane(T_po, up_pelvis, height):
    """A level plane `height` m below the pelvis origin (along up), as the camera at pose
    T_po (pelvis <- optical) should see it: (up in optical coords, c) with n.p = c on the
    plane -- the shape check_mount.up_from_floor returns. -c is the camera's height."""
    up = np.asarray(up_pelvis, float)
    return T_po[:3, :3].T @ up, -float(up @ T_po[:3, 3] + height)


def _tilt_roll(up_opt):
    """(tilt below horizontal, roll), degrees, from 'up' in camera optical coords."""
    n = np.asarray(up_opt, float) / np.linalg.norm(up_opt)
    return (float(np.degrees(np.arcsin(-n[2]))),
            float(np.degrees(np.arctan2(n[0], -n[1]))))


def from_level_surface(up_opt, camera_height_m=None, joints=None, angles=None,
                       source="", surface="floor", up_pelvis=None, body=None):
    """Build a calibration from one measured level surface.

    up_opt           the surface's upward unit normal in the camera OPTICAL frame
    camera_height_m  the camera's height above that surface, if it is the floor. With it,
                     outputs also carry heights above the floor.
    angles           the waist/head joint angles at the moment of measuring (default all 0)
    up_pelvis        the floor's up in pelvis_link (floor_frame.pelvis_up). Default: level
    body             the robot's state then ({"joints", "imu_rpy"}), recorded with it
    """
    if joints is None:
        joints = fk.load_urdf(cfg.URDF_PATH)
    angles = dict(angles or {})
    T_nom = fk.T_pelvis_optical(joints, angles, allow_nominal=True)
    R_nom = T_nom[:3, :3]

    u_meas = np.asarray(up_opt, float)
    u_meas = u_meas / np.linalg.norm(u_meas)
    up_p = np.array([0.0, 0.0, 1.0]) if up_pelvis is None else         np.asarray(up_pelvis, float) / np.linalg.norm(up_pelvis)
    u_cad = R_nom.T @ up_p

    # R' = R_nom Q must satisfy R'^T z = u_meas, i.e. Q^T u_cad = u_meas: Q takes u_meas
    # onto u_cad. The smallest such rotation adds no yaw, which a level surface cannot see.
    Q = _rotation_between(u_meas, u_cad)
    T_cal = T_nom.copy()
    T_cal[:3, :3] = R_nom @ Q

    T_pelvis_head = fk.T(joints, cfg.ROOT_LINK, "head_yaw_link", angles)
    T_head_optical = np.linalg.inv(T_pelvis_head) @ T_cal

    t_meas, r_meas = _tilt_roll(u_meas)
    t_cad, r_cad = _tilt_roll(u_cad)
    cal = {
        "method": "level %s: pitch and roll measured, yaw and position from the helmet CAD"
                  % surface + ("" if up_pelvis is None else "; pelvis lean from the robot"),
        "source": source,
        "date": datetime.date.today().isoformat(),
        "angles_at_calibration": {k: float(v) for k, v in angles.items()},
        "measured_tilt_deg": t_meas,
        "measured_roll_deg": r_meas,
        "cad_tilt_deg": t_cad,
        "correction_tilt_deg": t_meas - t_cad,
        "correction_roll_deg": r_meas - r_cad,
        "T_headyaw_optical": T_head_optical.tolist(),
        "pelvis_up": up_p.tolist(),
    }
    if body is not None:
        cal["imu_rpy_at_calibration"] = body.get("imu_rpy")
    if camera_height_m is not None and surface == "floor":
        cal["camera_above_floor_m"] = float(camera_height_m)
        cal["pelvis_above_floor_m"] = float(camera_height_m - up_p @ T_cal[:3, 3])
        if body is not None:
            import floor_frame as ff
            a = ff.ankle_midpoint(joints, body.get("joints", {}))
            if a is not None:      # lets the floor follow a deeper or shallower stance later
                cal["ankle_above_floor_m"] = float(cal["pelvis_above_floor_m"] + up_p @ a)
    return cal


def save(cal, path=CAL_PATH):
    with open(path, "w") as f:
        json.dump(cal, f, indent=2)
    return path


def load(path=CAL_PATH):
    """The saved calibration, or None if there is none."""
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def T_pelvis_optical(cal, joints=None, angles=None):
    """pelvis_link <- camera optical, using a calibration at the given joint angles."""
    if joints is None:
        joints = fk.load_urdf(cfg.URDF_PATH)
    T_head = T_pelvis_headyaw(joints, angles, cal.get("neck_tilt_rad") or (0.0, 0.0))
    return T_head @ np.asarray(cal["T_headyaw_optical"], float)


if __name__ == "__main__":
    cal = load()
    if cal is None:
        print("no calibration saved yet -- run:  python check_mount.py --calibrate")
    else:
        print(json.dumps({k: v for k, v in cal.items() if k != "T_headyaw_optical"},
                         indent=2))
        T = T_pelvis_optical(cal)
        print("camera optical origin in pelvis_link: %s m"
              % np.array2string(T[:3, 3], precision=4))
