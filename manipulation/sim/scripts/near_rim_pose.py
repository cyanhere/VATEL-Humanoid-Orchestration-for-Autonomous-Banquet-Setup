"""Two-finger near-rim push pose on the full R1 + Revo2 model (kinematics only).

    python scripts/near_rim_pose.py scan OUT.json [--n 3 --table 0.747 --pelvis 0.71 --hand 0,0,0,0,1.3,1.3]
    python scripts/near_rim_pose.py show OUT.png --theta 40 --yaw 30 --pitch 10 [--dz 0]     (4 close-ups + touches)
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from platepick.ik import PAD_POINT  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402
from rim_push_pose import touches  # noqa: E402


def rim_point(spec, theta, inset):
    """plate frame: the rim top `inset` mm inside the edge, `theta` deg from the point nearest the robot (-x) toward +y"""
    r = spec.R - inset / 1000
    th = math.radians(theta)
    return np.array([-r * math.cos(th), r * math.sin(th), spec.z_top(r)])


def place_near(m, d, meta, spec, top, hand, theta, inset, yaw, pitch, roll, dz=0.0, side="left", seed=None, starts=20):
    """Arm joints that put the index pad point on the rim top (theta, inset, +dz mm), palm down, fingers (yaw, pitch, roll)."""
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
    T = pt + Rt @ rim_point(spec, theta, inset) + np.array([0, 0, dz / 1000])
    yw, pc, rl = map(math.radians, (yaw, pitch, roll))
    f = np.array([math.cos(yw) * math.cos(pc), -math.sin(yw) * math.cos(pc), -math.sin(pc)])       # fingers
    x = np.array([0, 0, -1.0])                                                                      # pads down
    x = x - (x @ f) * f
    x /= np.linalg.norm(x)
    y = np.cross(f, x)
    x, y = math.cos(rl) * x + math.sin(rl) * y, math.cos(rl) * y - math.sin(rl) * x
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
    for q0 in ([seed] if seed is not None else []) + [np.array(meta["ready"][side])] + [rng.uniform(lo, hi) for _ in range(starts)]:
        s = least_squares(res, np.clip(q0, lo + 1e-4, hi - 1e-4), bounds=(lo, hi), xtol=1e-10)
        e = res(s.x)
        c = float(np.linalg.norm(e[:3]) * 1000 + 20 * np.linalg.norm(e[3:]))
        c += 1000.0 * float(np.min(np.minimum(s.x - lo, hi - s.x)) < 0.02)       # a joint on its limit (see rim_push_pose)
        if best is None or c < best[0]:
            best = (c, s.x, float(np.linalg.norm(e[:3]) * 1000), math.degrees(math.asin(min(1.0, np.linalg.norm(e[3:6]) / 0.1 / 2)) * 2),
                    math.degrees(math.asin(min(1.0, np.linalg.norm(e[6:]) / 0.1 / 2)) * 2))
    _, q, pos, ang_x, ang_f = best
    d.qpos[adr] = q
    mujoco.mj_forward(m, d)
    marg = math.degrees(float(np.min(np.minimum(q - lo, hi - q))))
    return q, dict(pos_mm=round(pos, 1), palm_deg=round(ang_x, 1), fingers_deg=round(ang_f, 1), margin_deg=round(marg, 1))


def scene(n, table, pelvis):
    sc = RobotScene(n=n, table=table, xy=(0.36, 0.06), pelvis=pelvis)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    return sc, m, d, meta


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["scan", "show"])
    ap.add_argument("out")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--table", type=float, default=0.747)
    ap.add_argument("--pelvis", type=float, default=0.71)
    ap.add_argument("--hand", default="0,0,0,0,1.3,1.3", help="tm,tp,ci,cm,cr,cp")
    ap.add_argument("--inset", type=float, default=18.0)
    ap.add_argument("--theta", type=float, default=40.0)
    ap.add_argument("--yaw", type=float, default=30.0)
    ap.add_argument("--pitch", type=float, default=10.0)
    ap.add_argument("--roll", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.0)
    a = ap.parse_args()
    sc, m, d0, meta = scene(a.n, a.table, a.pelvis)
    top = m.body(f"plate{a.n - 1}").id
    hand = np.array([float(v) for v in a.hand.split(",")])
    if a.mode == "show":
        import cv2
        from rim_push_pose import free_cam
        q, info = place_near(m, d0, meta, sc.spec, top, hand, a.theta, a.inset, a.yaw, a.pitch, a.roll, dz=a.dz)
        info.update(touch_mm=touches(m, d0, a.n), q=[round(float(v), 4) for v in q])
        print(json.dumps(info))
        tgt = d0.xpos[m.body("left_index_touch_link").id].copy()
        r = mujoco.Renderer(m, 480, 640)
        ims = []
        for eye in ((-0.30, 0.0, 0.12), (0.0, 0.30, 0.06), (0.30, -0.10, 0.15), (-0.02, 0.0, 0.34)):
            r.update_scene(d0, camera=free_cam(tgt + np.array(eye), tgt))
            ims.append(cv2.cvtColor(r.render(), cv2.COLOR_RGB2BGR))
        im = np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])])
        cv2.putText(im, f"theta {a.theta} inset {a.inset} yaw {a.yaw} pitch {a.pitch} roll {a.roll} dz {a.dz} hand {a.hand} | "
                    f"{json.dumps(info['touch_mm'])[:90]}", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imwrite(a.out, im)
        return
    rows = []
    for theta, yaw, pitch in itertools.product((15, 30, 45, 60, 75, 90), (-30, -15, 0, 15, 30), (5, 15, 25)):
        d = mujoco.MjData(m)
        d.qpos[:] = d0.qpos
        mujoco.mj_forward(m, d)
        q, info = place_near(m, d, meta, sc.spec, top, hand, theta, a.inset, yaw, pitch, 0.0, dz=3.0, starts=12)
        t = touches(m, d, a.n)
        rows.append(dict(theta=theta, yaw=yaw, pitch=pitch, **info, touch=t, q=[round(float(v), 4) for v in q]))
        print(json.dumps({k: v for k, v in rows[-1].items() if k != "q"}), flush=True)
    json.dump(rows, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
