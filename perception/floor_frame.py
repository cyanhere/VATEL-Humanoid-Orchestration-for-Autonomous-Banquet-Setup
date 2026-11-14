"""Where the floor is, from the robot's own sensors -- and the tape-measure frame.

Two frames matter, and they are NOT the same when the robot leans:

    pelvis_link   the robot's body: x forward, y left, z up THROUGH THE BODY. The arms'
                  kinematics live here, so a plate position handed to an arm must be here.
    the floor     gravity: what a tape measure, a spirit level and the table agree on.

Standing in balance the R1's pelvis is not level: the IMU read it pitched 3.0 deg forward
on 22 Oct. Three degrees is 3 cm at the camera's height. So "the pelvis is level" cannot be
assumed -- not by the camera calibration (which used to fold the lean into the camera, and
so put plates 3 deg off in the frame the arm uses) and not by anything that turns tape
measurements into robot coordinates.

The lean, two independent ways:

    imu    rt/lowstate imu_state.rpy -- the pelvis IMU, mounted square to pelvis_link
           (R1.urdf, imu_in_pelvis_joint, rpy 0 0 0)
    legs   both feet flat on a level floor: each sole's up IS the floor's up, and leg FK
           from the measured hip/knee/ankle angles gives it in pelvis_link -- no IMU

They should agree to a fraction of a degree. If they do not, one of them is wrong; this
says so rather than picking one quietly.

THE TAPE FRAME
    Origin on the floor directly below the point midway between the ankles; x forward
    (the pelvis' forward, made horizontal), y left, z straight up. "Forward from the ankle
    midpoint, left of centre, height off the floor" -- the three numbers a tape gives --
    are coordinates in it. With a level pelvis whose origin sits over the ankle midpoint,
    it is the old convention exactly: (x, y, z + pelvis height).
"""
import math

import numpy as np

import r1_config as cfg
import urdf_fk as fk

AGREE_DEG = 1.0          # IMU and legs further apart than this: say so
LEG_JOINTS = [n for n in cfg.MOTOR_SLOT if "hip" in n or "knee" in n or "ankle" in n]


def rpy_matrix(roll, pitch, yaw=0.0):
    """world <- body for Unitree's IMU angles (roll about x, pitch about y, yaw about z,
    applied z-y-x: R = Rz(yaw) Ry(pitch) Rx(roll)). Positive pitch: nose down."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def up_from_imu(rpy):
    """The floor's up, in pelvis_link, from the IMU's (roll, pitch, yaw) in radians."""
    return rpy_matrix(rpy[0], rpy[1]).T @ np.array([0.0, 0.0, 1.0])


def up_from_legs(joints, angles):
    """The floor's up in pelvis_link from each sole (both feet flat): (mean up, the two
    feet's disagreement in degrees). None if the leg angles are not in `angles`."""
    if not all(n in angles for n in LEG_JOINTS):
        return None
    ups = [fk.T(joints, cfg.ROOT_LINK, "%s_ankle_roll_link" % s, angles)[:3, 2]
           for s in ("left", "right")]
    up = ups[0] + ups[1]
    return up / np.linalg.norm(up), angle_deg(ups[0], ups[1])


def ankle_midpoint(joints, angles):
    """Midway between the two ankle joints, pelvis_link; None without the leg angles."""
    if not all(n in angles for n in LEG_JOINTS):
        return None
    return 0.5 * sum(fk.T(joints, cfg.ROOT_LINK, "%s_ankle_roll_link" % s, angles)[:3, 3]
                     for s in ("left", "right"))


def angle_deg(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    c = a @ b / (np.linalg.norm(a) * np.linalg.norm(b))
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def lean_deg(up):
    """(forward, left): how far the BODY leans, degrees, from the floor's up in pelvis_link.
    A body leaning forward sees the world's up tipped back (toward -x)."""
    up = np.asarray(up, float)
    return (float(np.degrees(math.atan2(-up[0], up[2]))),
            float(np.degrees(math.atan2(-up[1], up[2]))))


def pelvis_up(state, joints=None, source="imu"):
    """The floor's up in pelvis_link, and how it was found.

    state: {"joints": {name: rad}, "imu_rpy": [r, p, y] or None} (r1_state.decode's shape).
    source: "imu" (default; legs as the fallback), "legs" (IMU as the fallback), or "level"
    (assume no lean -- the old behaviour, for comparison only).
    Returns (up, info) where info = {"source", "imu_lean_deg", "legs_lean_deg",
    "imu_vs_legs_deg", "feet_disagree_deg", "warning"}.
    """
    if joints is None:
        joints = fk.load_urdf(cfg.URDF_PATH)
    ups = {}
    info = {"imu_lean_deg": None, "legs_lean_deg": None, "imu_vs_legs_deg": None,
            "feet_disagree_deg": None, "warning": None}
    rpy = state.get("imu_rpy") if state else None
    if rpy is not None:
        ups["imu"] = up_from_imu(rpy)
        info["imu_lean_deg"] = lean_deg(ups["imu"])
    legs = up_from_legs(joints, state.get("joints", {})) if state else None
    if legs is not None:
        ups["legs"] = legs[0]
        info["legs_lean_deg"] = lean_deg(legs[0])
        info["feet_disagree_deg"] = legs[1]
    if "imu" in ups and "legs" in ups:
        info["imu_vs_legs_deg"] = angle_deg(ups["imu"], ups["legs"])
        if info["imu_vs_legs_deg"] > AGREE_DEG:
            info["warning"] = ("the IMU and the legs disagree about the lean by %.1f deg -- "
                               "one of them is wrong (feet not flat? an IMU offset?)"
                               % info["imu_vs_legs_deg"])
    order = {"imu": ("imu", "legs"), "legs": ("legs", "imu"), "level": ()}[source]
    for s in order:
        if s in ups:
            info["source"] = s
            return ups[s], info
    info["source"] = "level"
    if source != "level":
        info["warning"] = info["warning"] or ("no IMU or leg angles: assuming the pelvis is "
                                              "level")
    return np.array([0.0, 0.0, 1.0]), info


class FloorFrame:
    """The tape frame (see the top of this file), for one standing pose of the robot.

    up             the floor's up in pelvis_link
    pelvis_height  the pelvis origin's height above the floor, along up, metres
    ankle_mid      the ankle midpoint in pelvis_link (None: use the pelvis origin)
    """

    def __init__(self, up, pelvis_height, ankle_mid=None):
        self.up = np.asarray(up, float) / np.linalg.norm(up)
        self.pelvis_height = float(pelvis_height)
        a = np.zeros(3) if ankle_mid is None else np.asarray(ankle_mid, float)
        self.origin = a - (self.up @ a + self.pelvis_height) * self.up   # on the floor
        f = np.array([1.0, 0.0, 0.0]) - self.up[0] * self.up
        self.fwd = f / np.linalg.norm(f)
        self.left = np.cross(self.up, self.fwd)

    def to_tape(self, p):
        """pelvis_link point -> (forward, left, height above floor), metres."""
        d = np.asarray(p, float) - self.origin
        return np.array([d @ self.fwd, d @ self.left, d @ self.up])

    def to_pelvis(self, tape):
        """(forward, left, height above floor), metres -> pelvis_link point."""
        f, l, h = tape
        return self.origin + f * self.fwd + l * self.left + h * self.up

    def direction_to_pelvis(self, word_vec):
        """A direction given in tape terms (e.g. the palm 'down' = (0,0,-1)) -> pelvis_link."""
        v = np.asarray(word_vec, float)
        return v[0] * self.fwd + v[1] * self.left + v[2] * self.up

    @classmethod
    def from_state(cls, state, cal, joints=None, source="imu"):
        """The tape frame now, from the robot's state and the camera calibration's floor.

        The pelvis height comes, best first, from: the legs plus the ankle's height above
        the floor measured at calibration (follows a deeper or shallower stance); the
        calibration's pelvis height; or None if neither is known."""
        if joints is None:
            joints = fk.load_urdf(cfg.URDF_PATH)
        up, info = pelvis_up(state, joints, source)
        a = ankle_midpoint(joints, state.get("joints", {})) if state else None
        cal = cal or {}
        if a is not None and cal.get("ankle_above_floor_m") is not None:
            h = cal["ankle_above_floor_m"] - up @ a
        elif cal.get("pelvis_above_floor_m") is not None:
            h = cal["pelvis_above_floor_m"]
        else:
            return None, info
        return cls(up, h, a), info


def describe(info):
    """One or two lines about the lean, for printing."""
    out = []
    for k in ("imu", "legs"):
        v = info.get("%s_lean_deg" % k)
        if v is not None:
            out.append("%s: leans %.2f deg %s, %.2f deg to its %s"
                       % (k, abs(v[0]), "forward" if v[0] >= 0 else "back",
                          abs(v[1]), "left" if v[1] >= 0 else "right"))
    s = "pelvis   " + ("; ".join(out) if out else "no IMU or leg angles")
    if info.get("imu_vs_legs_deg") is not None:
        s += "  (they agree to %.2f deg)" % info["imu_vs_legs_deg"]
    if info.get("warning"):
        s += "\n         WARNING: " + info["warning"]
    return s
