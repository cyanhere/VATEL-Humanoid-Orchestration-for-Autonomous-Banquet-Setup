"""Kinematics and gravity of the R1 upper body with both Revo2 hands, from the URDF. Numpy only."""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np

from . import config

G = np.array([0.0, 0.0, -9.81])

# The END of the curled pushing fingertip and the lowest part of its last 12 mm, in <side>_pinky_distal_link,
# measured from the simulation's collision mesh (platepick/ik.py PadIK.end; written by calibration/extract_points.py).
PINKY_END = {"left": np.array([0.0, 0.0, 0.0]), "right": np.array([0.0, 0.0, 0.0])}
PINKY_LOW = {"left": np.zeros((0, 3)), "right": np.zeros((0, 3))}
try:
    _pts = json.load(open(config.MODEL_DIR + "/finger_points.json"))
    for _s in ("left", "right"):
        PINKY_END[_s] = np.array(_pts[_s]["pinky_end"])
        PINKY_LOW[_s] = np.array(_pts[_s]["pinky_low"])
except (OSError, KeyError, ValueError):
    pass

# Fingertip pads (<side>_<finger>_touch_link): centre and outward normal in the pad's link frame, and the hands'
# collision mesh points per link (calibration/extract_hand_points.py, from the simulation's meshes).
PADS = {}
try:
    PADS = {s: v for s, v in json.load(open(config.MODEL_DIR + "/pads.json")).items() if s in ("left", "right")}
except (OSError, ValueError):
    pass
_HAND_PTS = None


def pad(T, side, finger):
    """(centre, outward normal) of a fingertip pad in the pelvis frame; finger: thumb index middle ring pinky."""
    p = PADS[side][finger]
    M = T["%s_%s_touch_link" % (side, finger)]
    return M[:3, :3] @ np.array(p["center"]) + M[:3, 3], M[:3, :3] @ np.array(p["normal"])


def hand_points(T, side):
    """{link: N x 3} the hand's collision mesh points (wrist roll link, palm, fingers, pads), pelvis frame."""
    global _HAND_PTS
    if _HAND_PTS is None:
        z = np.load(config.MODEL_DIR + "/hand_points.npz")
        _HAND_PTS = {k: z[k].astype(float) for k in z.files}
    return {ln: v @ T[ln][:3, :3].T + T[ln][:3, 3] for ln, v in _HAND_PTS.items() if ln.startswith(side + "_") and ln in T}


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_angle(axis, a):
    x, y, z = axis
    c, s = math.cos(a), math.sin(a)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def _vec(s, default=(0.0, 0.0, 0.0)):
    return np.array([float(v) for v in s.split()]) if s else np.array(default, float)


def arm_offsets(which="config"):
    """11 joint zero offsets (rad, q11 order; waist 0): real = measured + offset. which: "config" (config.ARM_OFFSETS_ON:
    "0" none, "1" config.ARM_OFFSETS, else a path), None (none), a path, or 11 numbers."""
    d = np.zeros(config.N_ARM)
    if which == "config":
        on = config.ARM_OFFSETS_ON
        which = None if on in ("", "0") else (config.ARM_OFFSETS if on == "1" else on)
    if which is None:
        return d
    if isinstance(which, str):
        o = json.load(open(which))["offsets_rad"]
        d[config.LEFT], d[config.RIGHT] = o["left"], o["right"]
        return d
    d[:] = np.asarray(which, float)
    return d


class Robot:
    def __init__(self, urdf=config.URDF, info=config.ROBOT_INFO, offsets="config"):
        self.d11 = arm_offsets(offsets)              # added to the measured / commanded arm joints in joint_values
        root = ET.parse(urdf).getroot()
        self.joints, self.children, self.mass, self.com = {}, {}, {}, {}
        for l in root.findall("link"):
            i = l.find("inertial")
            m = float(i.find("mass").get("value")) if i is not None and i.find("mass") is not None else 0.0
            o = i.find("origin") if i is not None else None
            self.mass[l.get("name")] = m
            self.com[l.get("name")] = _vec(o.get("xyz") if o is not None else None)
        for j in root.findall("joint"):
            o, a, mim, lim = j.find("origin"), j.find("axis"), j.find("mimic"), j.find("limit")
            T = np.eye(4)
            T[:3, :3] = rpy_matrix(*_vec(o.get("rpy") if o is not None else None))
            T[:3, 3] = _vec(o.get("xyz") if o is not None else None)
            typ = j.get("type")
            name = j.get("name")
            axis = _vec(a.get("xyz") if a is not None else None, (1.0, 0.0, 0.0))
            if name == "waist_yaw_joint":          # fixed in this URDF, turned about z here
                typ, axis = "revolute", np.array([0.0, 0.0, 1.0])
            rec = dict(name=name, type=typ, parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                       T=T, axis=axis / max(np.linalg.norm(axis), 1e-12),
                       lower=float(lim.get("lower", 0)) if lim is not None else 0.0,
                       upper=float(lim.get("upper", 0)) if lim is not None else 0.0,
                       mimic=(mim.get("joint"), float(mim.get("multiplier", 1)), float(mim.get("offset", 0)))
                       if mim is not None else None)
            self.joints[name] = rec
            self.children.setdefault(rec["parent"], []).append(name)
        self.root = "pelvis_link"
        info = json.load(open(info))
        self.hand_act = {s: info["revo2_%s_hand" % s]["actuated_joints"] for s in ("left", "right")}
        # subtree under each arm's shoulder pitch joint, for the gravity sums
        self.arm_joints = {s: ["%s_%s" % (s, j) for j in config.URDF_ARM] for s in ("left", "right")}
        self._below = {}
        for s in ("left", "right"):
            for jn in self.arm_joints[s]:
                self._below[jn] = self._subtree_links(self.joints[jn]["child"])
        # joints reachable from the root that matter (skip the legs: everything fixed there anyway)
        self._order = self._topo(self.root)

    # ------------------------------------------------------------ structure
    def _subtree_links(self, link):
        out, stack = [], [link]
        while stack:
            l = stack.pop()
            out.append(l)
            for jn in self.children.get(l, []):
                stack.append(self.joints[jn]["child"])
        return out

    def _topo(self, link):
        out, stack = [], [link]
        while stack:
            l = stack.pop()
            for jn in self.children.get(l, []):
                if "hip" in jn or "knee" in jn or "ankle" in jn:
                    continue
                out.append(jn)
                stack.append(self.joints[jn]["child"])
        return out

    def joint_values(self, q11, hands12=None):
        """{joint: rad} for the arms, the waist yaw and the actuated finger joints (mimics follow in fk)."""
        v = {"waist_yaw_joint": float(q11[config.WAIST]) + self.d11[config.WAIST]}
        for k, s in ((0, "left"), (5, "right")):
            for i, jn in enumerate(self.arm_joints[s]):
                v[jn] = float(q11[k + i]) + self.d11[k + i]
        if hands12 is not None:
            for k, s in ((0, "left"), (6, "right")):
                act = self.hand_act[s]
                # service order Thumb(bend), Thumb_aux(rotation), Index, Middle, Ring, Pinky
                names = ["%s_thumb_proximal_joint" % s, "%s_thumb_metacarpal_joint" % s, "%s_index_proximal_joint" % s,
                         "%s_middle_proximal_joint" % s, "%s_ring_proximal_joint" % s, "%s_pinky_proximal_joint" % s]
                for i, jn in enumerate(names):
                    if jn in act or jn in self.joints:
                        v[jn] = float(np.clip(hands12[k + i], 0.0, 1.0)) * float(config.HAND_FULL_RAD[i])
        return v

    # ------------------------------------------------------------ kinematics
    def fk(self, q11, hands12=None):
        """{link: 4x4 pelvis_link <- link} for the upper body (legs skipped)."""
        v = self.joint_values(q11, hands12)
        T = {self.root: np.eye(4)}
        for jn in self._order:
            j = self.joints[jn]
            Tp = T[j["parent"]] @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                if j["mimic"] is not None:
                    src, k, off = j["mimic"]
                    a = k * v.get(src, 0.0) + off
                else:
                    a = v.get(jn, 0.0)
                if a:
                    R = np.eye(4)
                    R[:3, :3] = axis_angle(j["axis"], a)
                    Tp = Tp @ R
            T[j["child"]] = Tp
        return T

    @staticmethod
    def point(T, link, p_local):
        M = T[link]
        return M[:3, :3] @ np.asarray(p_local, float) + M[:3, 3]

    def joint_frames(self, T, side):
        """[(origin, axis)] of the 5 arm joints in pelvis_link."""
        out = []
        for jn in self.arm_joints[side]:
            j = self.joints[jn]
            M = T[j["parent"]] @ j["T"]                  # joint frame (before its own rotation)
            out.append((M[:3, 3].copy(), M[:3, :3] @ j["axis"]))
        return out

    def jacobian(self, T, side, p):
        """3 x 5: how the point p (pelvis_link, rigidly carried by the hand) moves per arm joint."""
        J = np.zeros((3, 5))
        for i, (o, a) in enumerate(self.joint_frames(T, side)):
            J[:, i] = np.cross(a, p - o)
        return J

    def rot_jacobian(self, T, side):
        """3 x 5 angular velocity of the hand per arm joint."""
        return np.array([a for _, a in self.joint_frames(T, side)]).T

    # ------------------------------------------------------------ gravity
    def _mass_table(self, side):
        """Links below the shoulder with mass, their masses / local COMs, and which of them each arm joint carries."""
        key = "_mt_" + side
        if not hasattr(self, key):
            links = [l for l in self._below[self.arm_joints[side][0]] if self.mass.get(l, 0.0) > 0.0]
            m = np.array([self.mass[l] for l in links])
            com = np.array([self.com[l] for l in links])
            masks = np.array([[1.0 if l in set(self._below[jn]) else 0.0 for l in links] for jn in self.arm_joints[side]])
            setattr(self, key, (links, m, com, masks))
        return getattr(self, key)

    def gravity(self, T, side, extra=None):
        """Motor torques (N m, arm_sdk sign) that hold the arm still against gravity at pose T.
        extra: [(mass kg, point in pelvis_link)] carried by the hand (e.g. a plate's share).
        Vectorised: joint i carries the links below it, tau_i = -a_i. ((S_i - M_i o_i) x m g), with S_i their summed
        first moment and M_i their summed mass."""
        links, m, com, masks = self._mass_table(side)
        R = np.array([T[l][:3, :3] for l in links])
        t = np.array([T[l][:3, 3] for l in links])
        C = np.einsum("lij,lj->li", R, com) + t                 # link COMs, pelvis frame
        S = masks @ (m[:, None] * C)                            # 5 x 3 first moments
        M = masks @ m                                           # 5 subtree masses
        if extra:
            for mx, cx in extra:
                S = S + mx * np.asarray(cx, float)[None, :]
                M = M + mx
        out = np.zeros(5)
        for i, (o, a) in enumerate(self.joint_frames(T, side)):
            out[i] = -float(a @ np.cross(S[i] - M[i] * o, G))
        return out

    def gravity11(self, q11, hands12=None, T=None):
        T = self.fk(q11, hands12) if T is None else T
        g = np.zeros(config.N_ARM)
        g[config.LEFT] = self.gravity(T, "left")
        g[config.RIGHT] = self.gravity(T, "right")
        return g


class GravityComp:
    """Gravity compensation by position offset: q_sent = q_planned + (scale * G(q_planned) + bias) / kp.

    scale and bias per joint come from model/gravity_fit.json (tune.py fit); without a fit, scale 1 and bias 0.
    The bias is a torque (N m) so that it stays correct when kp is scaled. Also holds the fitted Coulomb friction
    per joint (friction_lead, used only when enabled on the server).
    """

    def __init__(self, robot=None, path=None, enabled=True):
        self.rob = robot or Robot()
        self.scale = np.ones(config.N_ARM)
        self.bias_nm = np.zeros(config.N_ARM)
        self.friction = np.zeros(config.N_ARM)
        self.enabled = enabled
        self.source = "URDF masses, not fitted"
        path = path or (config.MODEL_DIR + "/gravity_fit.json")
        try:
            fit = json.load(open(path))
            self.scale = np.array(fit["scale"], float)
            self.bias_nm = np.array(fit["bias_rad"], float) * config.KP
            self.friction = np.array(fit.get("friction_Nm", [0.0] * config.N_ARM), float)
            self.source = "fit %s (%s)" % (fit.get("date", "?"), path)
        except (OSError, KeyError, ValueError):
            pass
        self.scale[config.WAIST] = 0.0
        self.bias_nm[config.WAIST] = 0.0
        self.friction[config.WAIST] = 0.0

    def offset(self, q11, hands12=None, kp=config.KP, payload=None):
        """payload: dict(side, mass kg, link, p (point in the link frame)) -- e.g. the plate in a hand."""
        if not self.enabled:
            return np.zeros(config.N_ARM)
        T = self.rob.fk(q11, hands12)
        g = self.rob.gravity11(q11, hands12, T=T)
        if payload and payload.get("mass", 0) > 0 and payload.get("link") in T:
            side = payload["side"]
            M = T[payload["link"]]
            c = M[:3, :3] @ np.asarray(payload.get("p", (0, 0, 0)), float) + M[:3, 3]
            extra = self.rob.gravity(T, side, extra=[(float(payload["mass"]), c)]) - self.rob.gravity(T, side)
            sl = config.LEFT if side == "left" else config.RIGHT
            g_off = (self.scale * g + self.bias_nm) / kp
            g_off[sl] += extra / kp[sl]                    # the payload's own torque, not scaled by the arm fit
            return g_off
        return (self.scale * g + self.bias_nm) / kp

    def friction_lead(self, v11, kp=config.KP, alpha=0.0, v0=config.FRIC_V0):
        """alpha * f * tanh(v / v0) / kp: the lead that makes the PD supply the fitted Coulomb friction f along the
        PLANNED joint velocity v (rad/s), so a joint does not first lag f / kp behind before it moves. 0 at rest (a
        holding joint has no direction to help). alpha 0: off."""
        if alpha <= 0.0:
            return np.zeros(config.N_ARM)
        return alpha * self.friction * np.tanh(np.asarray(v11, float) / v0) / kp
