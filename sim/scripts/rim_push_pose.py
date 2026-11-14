"""Rim push hand pose on the full R1 + Revo2 model (kinematics only).

    python scripts/rim_push_pose.py OUT.png [--hand 0.9,0.6,0.15,0.55,1.3,1.3 --yaw 0 --pitch 10 --roll 0 --di 6]
"""
import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from platepick.ik import PAD_NORMAL, PAD_POINT  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402

FINGERS = ("thumb", "index", "middle", "ring", "pinky")


def place(m, d, meta, spec, top, hand, yaw, pitch, roll, di, dz=0.0, side="left", seed=None):
    """Arm joints that put the index pad point `di` mm inside the +y rim edge, on the rim top (+dz mm), palm down."""
    arm = [m.joint(j).id for j in meta["arm"][side]]
    adr = [m.jnt_qposadr[j] for j in arm]
    lo = np.array([m.jnt_range[j][0] for j in arm])
    hi = np.array([m.jnt_range[j][1] for j in arm])
    C = np.array(meta["coupling"][side])
    for i, j in enumerate(meta["hand"][side]):
        jid = m.joint(j).id
        d.qpos[m.jnt_qposadr[jid]] = min(C[i] @ hand, m.jnt_range[jid][1])
    palm = m.body(f"{side}_base_link").id
    idx = m.body(f"{side}_index_touch_link").id
    pt, Rt = d.xpos[top].copy(), d.xmat[top].reshape(3, 3)
    r = spec.R - di / 1000
    T = pt + Rt @ np.array([0, r, spec.z_top(r)]) + np.array([0, 0, dz / 1000])
    yw, pc, rl = map(math.radians, (yaw, pitch, roll))
    f = np.array([math.cos(yw) * math.cos(pc), -math.sin(yw) * math.cos(pc), -math.sin(pc)])       # fingers
    x = np.array([0, 0, -1.0])                                                                      # pads down
    x = x - (x @ f) * f
    x /= np.linalg.norm(x)
    y = np.cross(f, x)
    x, y = math.cos(rl) * x + math.sin(rl) * y, math.cos(rl) * y - math.sin(rl) * x                  # roll about the fingers
    d2 = mujoco.MjData(m)
    d2.qpos[:] = d.qpos

    def res(q):
        d2.qpos[adr] = q
        mujoco.mj_kinematics(m, d2)
        R = d2.xmat[palm].reshape(3, 3)
        Ri = d2.xmat[idx].reshape(3, 3)
        return np.concatenate([d2.xpos[idx] + Ri @ PAD_POINT - T, 0.1 * (R[:, 0] - x), 0.1 * (R[:, 2] - f)])
    rng = np.random.default_rng(0)
    best = None
    for q0 in ([seed] if seed is not None else []) + [np.array(meta["ready"][side])] + [rng.uniform(lo, hi) for _ in range(30)]:
        s = least_squares(res, np.clip(q0, lo + 1e-4, hi - 1e-4), bounds=(lo, hi), xtol=1e-10)
        e = res(s.x)
        c = float(np.linalg.norm(e[:3]) * 1000 + 20 * np.linalg.norm(e[3:]))
        # a second, flipped arm branch has joint 0 ON its limit (2.094 rad); the controller then cannot reach
        # below it (2-plate pose4) -> a branch with any joint within 0.02 rad of a limit loses
        c += 1000.0 * float(np.min(np.minimum(s.x - lo, hi - s.x)) < 0.02)
        if best is None or c < best[0]:
            best = (c, s.x, float(np.linalg.norm(e[:3]) * 1000), math.degrees(math.asin(min(1.0, np.linalg.norm(e[3:6]) / 0.1 / 2)) * 2),
                    math.degrees(math.asin(min(1.0, np.linalg.norm(e[6:]) / 0.1 / 2)) * 2))
    _, q, pos, ang_x, ang_f = best
    d.qpos[adr] = q
    mujoco.mj_forward(m, d)
    marg = math.degrees(float(np.min(np.minimum(q - lo, hi - q))))
    return q, dict(pos_mm=round(pos, 1), palm_deg=round(ang_x, 1), fingers_deg=round(ang_f, 1), margin_deg=round(marg, 1))


def touches(m, d, n, side="left"):
    """deepest overlap (mm, + = into) of each finger / the palm with the top plate, a lower plate and the table"""
    out = {}
    for i in range(d.ncon):
        c = d.contact[i]
        for a, b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
            na = m.body(m.geom_bodyid[a]).name
            if not na.startswith(side + "_"):
                continue
            nb = m.body(m.geom_bodyid[b]).name
            part = next((f for f in FINGERS if f"_{f}_" in na), "palm/arm")
            what = "top" if nb == f"plate{n - 1}" else ("lower plate" if nb.startswith("plate") else ("table" if m.geom_bodyid[b] == 0 else None))
            if what:
                k = f"{part} on {what}"
                out[k] = round(max(out.get(k, -9.0), -c.dist * 1000), 1)
    return out


def free_cam(eye, target):
    c = mujoco.MjvCamera()
    c.type = mujoco.mjtCamera.mjCAMERA_FREE
    v = np.array(target) - np.array(eye)
    c.lookat[:] = target
    c.distance = float(np.linalg.norm(v))
    c.azimuth = math.degrees(math.atan2(v[1], v[0]))
    c.elevation = math.degrees(math.atan2(v[2], np.linalg.norm(v[:2])))
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--hand", default="0.9,0.6,0.15,0.55,1.3,1.3", help="thumb rot, thumb bend, index, middle, ring, little (0..1.55)")
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--pitch", type=float, default=10.0)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--di", type=float, default=6.0)
    ap.add_argument("--dz", type=float, default=0.0)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--table", type=float, default=0.747)
    ap.add_argument("--pelvis", type=float, default=0.71)
    a = ap.parse_args()
    sc = RobotScene(n=a.n, table=a.table, xy=(0.36, 0.06), pelvis=a.pelvis)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    top = m.body(f"plate{a.n - 1}").id
    hand = np.array([float(v) for v in a.hand.split(",")])
    q, info = place(m, d, meta, sc.spec, top, hand, a.yaw, a.pitch, a.roll, a.di, a.dz)
    info["touch_mm"] = touches(m, d, a.n)
    info["q"] = [round(float(v), 4) for v in q]
    print(json.dumps(info))
    tgt = d.xpos[m.body("left_index_touch_link").id].copy()
    r = mujoco.Renderer(m, 480, 640)
    ims = []
    for eye in ((0.30, 0.10, 0.10), (0.02, 0.28, 0.06), (-0.12, 0.06, 0.25), (0.0, 0.0, 0.32)):
        r.update_scene(d, camera=free_cam(tgt + np.array(eye), tgt))
        ims.append(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
    im = np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])])
    cv2.putText(im, f"hand {a.hand} yaw {a.yaw} pitch {a.pitch} roll {a.roll} di {a.di} | {json.dumps(info['touch_mm'])[:110]}", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(a.out, im)


if __name__ == "__main__":
    main()
