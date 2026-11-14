"""Write model/hand_points.npz and model/pads.json from the simulation meshes, so the robot can place its hands without MuJoCo.

    python calibration/extract_hand_points.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
SIM = os.path.abspath(os.path.join(PICK, "..", "sim"))
sys.path.insert(0, PICK)
sys.path.insert(0, SIM)

import mujoco  # noqa: E402

from pk.model import Robot  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]


def body_verts(m, name, every=None):
    """Collision mesh vertices of a body, in the body frame."""
    b = m.body(name).id
    out = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] != b or m.geom_contype[g] == 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.geom_quat[g])
        vb = v @ R.reshape(3, 3).T + m.geom_pos[g]
        if every is None:
            vb = vb[:: max(1, len(vb) // 400)]
        out.append(vb)
    return np.vstack(out) if out else np.zeros((0, 3))


def main():
    sc = RobotScene(n=3, table=0.747, xy=(0.36, 0.06), pelvis=0.71)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    rob = Robot()
    T0 = rob.fk(np.zeros(11), np.zeros(12))
    pts, pads = {}, {}
    for side in ("left", "right"):
        links = ["%s_wrist_roll_link" % side, "%s_base_link" % side]
        for f in FINGERS:
            links += [m.body(i).name for i in range(m.nbody) if m.body(i).name.startswith("%s_%s_" % (side, f))]
        for ln in links:
            if ln not in T0:
                continue
            v = body_verts(m, ln)
            if len(v):
                pts[ln] = v.astype(np.float32)
        pads[side] = {}
        for f in FINGERS:
            pad = body_verts(m, "%s_%s_touch_link" % (side, f), every=1)
            c = pad.mean(axis=0)
            w, U = np.linalg.eigh(np.cov((pad - c).T))
            n = U[:, 0]
            # the fingertip's body, in the touch link's frame: the pad's normal points away from it
            dn = "%s_%s_distal_link" % (side, f)
            Td, Tt = T0[dn], T0["%s_%s_touch_link" % (side, f)]
            body = body_verts(m, dn) @ Td[:3, :3].T + Td[:3, 3]
            body_t = (body - Tt[:3, 3]) @ Tt[:3, :3]
            if n @ (c - body_t.mean(axis=0)) < 0:
                n = -n
            pads[side][f] = dict(center=[float(x) for x in c], normal=[float(x) for x in n], thickness_mm=float(2000 * np.sqrt(w[0])),
                                 size_mm=[float(4000 * np.sqrt(w[1])), float(4000 * np.sqrt(w[2]))])
    np.savez_compressed(os.path.join(PICK, "model", "hand_points.npz"), **pts)
    json.dump(dict(pads, note="fingertip pad centre and outward normal in <side>_<finger>_touch_link (calibration/extract_hand_points.py)"),
              open(os.path.join(PICK, "model", "pads.json"), "w"), indent=1)
    print("hand_points.npz: %d links, %d points; pads.json: %s" % (len(pts), sum(len(v) for v in pts.values()),
                                                                  {f: np.round(pads["left"][f]["normal"], 2).tolist() for f in FINGERS}))


if __name__ == "__main__":
    main()
