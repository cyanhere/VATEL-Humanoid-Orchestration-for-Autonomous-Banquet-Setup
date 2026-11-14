"""After the right grip and lift, carry the plate to the robot's centre line and level it for the left hand.

    python scripts/grip_centre.py STATE.npz --save OUT.npz [--video OUT.mp4] [--y 0 --dz 0 --T 2.0]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import mujoco  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from platepick.forces import ForceMeter  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402


def rot_between(a, b):
    """Rotation matrix turning unit vector a onto b (shortest arc)."""
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("state")
    ap.add_argument("--save", default="")
    ap.add_argument("--video", default="")
    ap.add_argument("--y", type=float, default=0.0, help="target plate centre y (m), robot centre line = 0")
    ap.add_argument("--dx", type=float, default=0.0, help="move the plate this much in x (m)")
    ap.add_argument("--x", type=float, default=0.0, help="target plate centre x (m, forward of the pelvis); 0 = keep")
    ap.add_argument("--n_way", type=int, default=20)
    ap.add_argument("--rigid", action="store_true", help="carry only along poses the arm reaches exactly (plate fixed in the hand)")
    ap.add_argument("--w_level", type=float, default=1.0, help="rigid: weight of levelling vs metres off the target")
    ap.add_argument("--dz", type=float, default=0.0, help="raise the plate this much (m)")
    ap.add_argument("--T", type=float, default=2.0)
    ap.add_argument("--w_rot", type=float, default=0.3, help="orientation weight in the IK (per unit axis error vs m)")
    ap.add_argument("--hold", type=float, default=1.0)
    ap.add_argument("--enc_fb", type=int, default=0, help="after the move: N rounds of target += (planned - measured joint angle), from the encoders (soft arm sag)")
    ap.add_argument("--title", default="MuJoCo: the right hand levels the plate and brings it to the robot's centre line")
    a = ap.parse_args()

    st = np.load(a.state, allow_pickle=True)
    pp, gp = json.loads(str(st["par"])), json.loads(str(st["grip"]))
    n = int(pp["n"])
    pick_mu = lambda k, v: gp.get(k, -1) if gp.get(k, -1) > 0 else v  # noqa: E731
    fr = Friction(glaze_glaze=pick_mu("mu_glaze", pp["glaze_glaze"]), foot_glaze=pick_mu("mu_foot", pp["foot_glaze"]),
                  pad_glaze=pick_mu("mu_pad", pp["pad_glaze"]), table=pick_mu("mu_table", pp["table_mu"]))
    sc = RobotScene(n=n, table=pp["table"], xy=(pp["x"], pp["y"]), pelvis=pp["pelvis"], fr=fr, table_right=pp.get("table_right", 0.0))
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:], d.ctrl[:], d.time = st["qpos"], st["qvel"], st["ctrl"], float(st["time"])
    mujoco.mj_forward(m, d)
    top = m.body(f"plate{n - 1}").id
    fm = ForceMeter(m, "right", top, m.body(f"plate{n - 2}").id if n > 1 else None, hands=("right", "left"))
    fm.R_edge = sc.spec.R                 # thumb contact "in from the edge"
    palm = m.body("right_base_link").id
    act = {m.actuator(i).name: i for i in range(m.nu)}
    arm = [m.joint(j).id for j in meta["arm"]["right"]]
    arm_act = [act[f"a_{j}"] for j in meta["arm"]["right"]]
    lo, hi = np.array([m.jnt_range[j][0] for j in arm]), np.array([m.jnt_range[j][1] for j in arm])

    def tilt():
        return math.degrees(math.acos(min(1.0, d.xmat[top][8])))

    P0, R0 = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
    Rl = rot_between(R0[:, 2], np.array([0.0, 0.0, 1.0]))          # levels the plate, keeps its yaw
    Pt = np.array([a.x if a.x > 0 else P0[0] + a.dx, a.y, P0[2] + a.dz])
    Hp, HR = d.xpos[palm].copy(), d.xmat[palm].reshape(3, 3).copy()
    tgt_p = Pt + Rl @ (Hp - P0)                                       # the palm rides with the plate
    tgt_R = Rl @ HR
    q0 = np.array([d.qpos[m.jnt_qposadr[j]] for j in arm])

    dk = mujoco.MjData(m)
    dk.qpos[:] = d.qpos
    # one smooth move: lift, level and carry at the same time. The plate pose is interpolated (position linearly,
    # rotation about one axis), the palm rides with it, and each waypoint is solved from the previous one.
    ang = math.acos(np.clip((np.trace(Rl) - 1) / 2, -1, 1))
    axis = np.array([Rl[2, 1] - Rl[1, 2], Rl[0, 2] - Rl[2, 0], Rl[1, 0] - Rl[0, 1]])
    axis = axis / (np.linalg.norm(axis) + 1e-12)

    def rot(t):
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K

    def solve(tp, tR, q_start):
        def res(q):
            for j, v in zip(arm, q):
                dk.qpos[m.jnt_qposadr[j]] = v
            mujoco.mj_kinematics(m, dk)
            R = dk.xmat[palm].reshape(3, 3)
            return np.concatenate([dk.xpos[palm] - tp, a.w_rot * (R - tR).ravel()])
        r_ = least_squares(res, q_start, bounds=(lo, hi), xtol=1e-10, ftol=1e-12)
        return r_.x, res(r_.x)
    way, q_prev = [q0], q0
    for k in range(1, a.n_way + 1):
        s_ = k / a.n_way
        Rs = rot(s_ * ang)
        q_prev, r = solve(P0 + s_ * (Pt - P0) + Rs @ (Hp - P0), Rs @ HR, q_prev)
        way.append(q_prev)
    if a.rigid:
        # choose arm angles that keep the carried plate as level and as close to the target as reachable, then move in
        # joint space, so the plate never has to slip or twist in the hand
        rel_p, rel_R = HR.T @ (P0 - Hp), HR.T @ R0

        def res_rigid(q):
            for j, v in zip(arm, q):
                dk.qpos[m.jnt_qposadr[j]] = v
            mujoco.mj_kinematics(m, dk)
            Rh = dk.xmat[palm].reshape(3, 3)
            pp_, pR = dk.xpos[palm] + Rh @ rel_p, Rh @ rel_R
            return np.concatenate([a.w_level * pR[:2, 2], pp_ - Pt])
        qr = least_squares(res_rigid, q0, bounds=(lo, hi), xtol=1e-10, ftol=1e-12).x
        way = [q0 + (qr - q0) * u for u in np.linspace(0, 1, a.n_way + 1)]
        rr = res_rigid(qr)
        r = np.concatenate([rr[2:], np.zeros(9)])
        print(json.dumps(dict(rigid_plan=dict(tilt_deg=float(math.degrees(math.asin(min(1.0, np.linalg.norm(rr[:2]) / a.w_level)))),
                                              off_target_mm=(rr[2:] * 1000).round(1).tolist()))))
    sol = type("S", (), {"x": way[-1]})
    ik = dict(pos_err_mm=float(np.linalg.norm(r[:3]) * 1000), rot_err_deg=float(math.degrees(np.linalg.norm(r[3:]) / a.w_rot / 2)))

    vr = vid = None
    if a.video:
        import cv2
        vr = mujoco.Renderer(m, 720, 640)
        vid = cv2.VideoWriter(a.video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))

    def shot():
        import cv2
        ims = []
        for cam, lab in (("front", "from the front"), ("top", "from above (robot at the top)")):
            vr.update_scene(d, camera=cam)
            im = cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im, lab, (10, 708), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            ims.append(im)
        bar = np.full((64, 1280, 3), 20, np.uint8)
        cv2.putText(bar, a.title[:150], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, f"t {d.time:5.2f} s   plate tilt {tilt():4.1f} deg   plate centre y {1000 * d.xpos[top][1]:+5.1f} mm (robot centre line = 0)",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims)]))

    N = int(a.T / m.opt.timestep)
    W = np.array(way)
    for k in range(N):
        u = (k + 1) / N
        u = (3 * u * u - 2 * u ** 3) * (len(W) - 1)
        i0 = min(int(u), len(W) - 2)
        d.ctrl[arm_act] = W[i0] + (W[i0 + 1] - W[i0]) * (u - i0)
        mujoco.mj_step(m, d)
        fm.update(d)
        if vr is not None and k % int(0.04 / m.opt.timestep) == 0:
            shot()
    q_goal = W[-1].copy()
    for it in range(a.enc_fb):
        # a soft (real-gain) arm sags under the plate; the robot reports its joint angles, so the target is
        # shifted by the remaining error until the measured angles are the planned ones
        q_meas = np.array([d.qpos[m.jnt_qposadr[j]] for j in arm])
        tgt0 = np.array(d.ctrl[arm_act])
        tgt1 = np.clip(tgt0 + (q_goal - q_meas), lo, hi)
        for k in range(int(0.3 / m.opt.timestep)):
            u = min(1.0, (k + 1) / int(0.2 / m.opt.timestep))
            d.ctrl[arm_act] = tgt0 + (tgt1 - tgt0) * u
            mujoco.mj_step(m, d)
            fm.update(d)
            if vr is not None and k % int(0.04 / m.opt.timestep) == 0:
                shot()
    for k in range(int(a.hold / m.opt.timestep)):
        mujoco.mj_step(m, d)
        fm.update(d)
        if vr is not None and k % int(0.04 / m.opt.timestep) == 0:
            shot()
    P1 = d.xpos[top]
    fN, w6 = {}, np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        for ga, gb in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
            nm = m.body(m.geom_bodyid[ga]).name
            if nm.startswith("right_") and m.geom_bodyid[gb] == top:
                mujoco.mj_contactForce(m, d, i, w6)
                k_ = nm[6:].replace("_link", "")
                fN[k_] = round(fN.get(k_, 0.0) + float(np.linalg.norm(w6[:3])), 1)
    out = dict(state=os.path.basename(a.state), right_N=fN, ik=ik, tilt_before_deg=math.degrees(math.acos(min(1, R0[2, 2]))), tilt_after_deg=tilt(),
               centre_before_mm=(P0 * 1000).round(1).tolist(), centre_after_mm=(P1 * 1000).round(1).tolist(),
               centre_target_mm=(Pt * 1000).round(1).tolist(), right_arm_q=sol.x.round(3).tolist(),
               right_palm_mm=(d.xpos[palm] * 1000).round(1).tolist(), force=fm.summary())
    print(json.dumps(out))
    if a.save:
        np.savez(a.save, qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, time=d.time, par=str(st["par"]), grip=str(st["grip"]))
    if vid is not None:
        vid.release()


if __name__ == "__main__":
    main()
