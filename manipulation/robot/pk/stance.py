"""Pelvis height from the leg joints in rt/lowstate."""
import json
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

from . import config
from .model import axis_angle, rpy_matrix

LEG_JOINTS = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint",
              "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
              "right_ankle_pitch_joint", "right_ankle_roll_joint"]

_CACHE = {}


def _load():
    if "j" in _CACHE:
        return _CACHE["j"], _CACHE["soles"]
    root = ET.parse(os.path.join(config.MODEL_DIR, "R1_legs_head.urdf")).getroot()
    J = {}
    for j in root.findall("joint"):
        o, a = j.find("origin"), j.find("axis")
        T = np.eye(4)
        T[:3, :3] = rpy_matrix(*[float(v) for v in (o.get("rpy") or "0 0 0").split()])
        T[:3, 3] = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
        J[j.find("child").get("link")] = dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), T=T,
                                              axis=np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()]))
    soles = json.load(open(os.path.join(config.MODEL_DIR, "sole_points.json")))
    _CACHE["j"], _CACHE["soles"] = J, {k: np.array(v) for k, v in soles.items()}
    return _CACHE["j"], _CACHE["soles"]


def _link_T(J, link, q):
    chain = []
    while link in J:
        chain.append(J[link])
        link = J[link]["parent"]
    T = np.eye(4)
    for j in reversed(chain):
        T = T @ j["T"]
        if j["type"] in ("revolute", "continuous") and j["name"] in q:
            R = np.eye(4)
            R[:3, :3] = axis_angle(j["axis"] / np.linalg.norm(j["axis"]), q[j["name"]])
            T = T @ R
    return T


def pelvis_height(legs12, rpy=(0.0, 0.0, 0.0)):
    J, soles = _load()
    q = dict(zip(LEG_JOINTS, [float(v) for v in legs12]))
    r, p = float(rpy[0]), float(rpy[1])
    cr, sr, cp, sp = math.cos(r), math.sin(r), math.cos(p), math.sin(p)
    W = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    out = {}
    for side in ("left", "right"):
        T = _link_T(J, side + "_ankle_roll_link", q)
        w = (W @ (soles[side] @ T[:3, :3].T + T[:3, 3]).T).T
        out[side] = float(-w[:, 2].min())
    return float(np.mean(list(out.values()))), out
