"""Write model/finger_points.json and check pk.model's forward kinematics against the MuJoCo scene.

    python calibration/extract_points.py
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

from pk import config  # noqa: E402
from pk.model import Robot  # noqa: E402
from platepick.ik import PadIK  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402


def main():
    sc = RobotScene(n=3, table=0.747, xy=(0.36, 0.06), pelvis=0.71)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    out = {}
    for side in ("left", "right"):
        ik = PadIK(m, meta, side, "pinky", "tip")
        low = ik.end_verts
        # keep the lowest-reaching 60 of the last-12-mm vertices (enough for a min over orientations)
        keep = low[np.argsort(low[:, 2])][-200:]
        keep = keep[:: max(1, len(keep) // 60)]
        out[side] = dict(pinky_end=[float(v) for v in ik.end_pt], pinky_low=[[float(x) for x in v] for v in keep])
    json.dump(dict(out, note="pinky end point and last-12-mm vertices in <side>_pinky_distal_link (platepick/ik.py PadIK)"),
              open(os.path.join(PICK, "model", "finger_points.json"), "w"), indent=1)
    print("pinky end (left):", out["left"]["pinky_end"])

    # ---- FK check: random arm and finger poses, MuJoCo body frames vs pk.model, pelvis frame
    rob = Robot()
    rng = np.random.RandomState(0)
    C = {s: np.array(meta["coupling"][s]) for s in ("left", "right")}
    act = {s: meta["hand_act"][s] for s in ("left", "right")}
    hand = {s: meta["hand"][s] for s in ("left", "right")}
    worst = 0.0
    names = ["left_pinky_distal_link", "left_index_distal_link", "left_thumb_distal_link", "right_middle_distal_link",
             "right_ring_distal_link", "right_base_link", "left_wrist_roll_link"]
    for trial in range(20):
        q = config.clamp_arm(rng.uniform(config.ARM_LO, config.ARM_HI))
        q[config.WAIST] = rng.uniform(-0.3, 0.3)
        h = rng.uniform(0, 1, 12)
        d.qpos[:] = m.qpos0
        for k, s in ((0, "left"), (5, "right")):
            for i, jn in enumerate(meta["arm"][s]):
                d.qpos[m.jnt_qposadr[m.joint(jn).id]] = q[k + i]
        d.qpos[m.jnt_qposadr[m.joint("waist_yaw_joint").id]] = q[config.WAIST]
        for k, s in ((0, "left"), (6, "right")):
            # service order -> actuated joint angles -> coupled physical joints
            svc = {"%s_thumb_proximal_joint" % s: h[k + 0] * config.HAND_FULL_RAD[0],
                   "%s_thumb_metacarpal_joint" % s: h[k + 1] * config.HAND_FULL_RAD[1],
                   "%s_index_proximal_joint" % s: h[k + 2] * config.HAND_FULL_RAD[2],
                   "%s_middle_proximal_joint" % s: h[k + 3] * config.HAND_FULL_RAD[3],
                   "%s_ring_proximal_joint" % s: h[k + 4] * config.HAND_FULL_RAD[4],
                   "%s_pinky_proximal_joint" % s: h[k + 5] * config.HAND_FULL_RAD[5]}
            a = np.array([svc[n] for n in act[s]])
            for i, jn in enumerate(hand[s]):
                d.qpos[m.jnt_qposadr[m.joint(jn).id]] = C[s][i] @ a
        mujoco.mj_kinematics(m, d)
        T = rob.fk(q, h)
        for n in names:
            p_mj = d.xpos[m.body(n).id] - np.array([0, 0, 0.71])
            R_mj = d.xmat[m.body(n).id].reshape(3, 3)
            dp = np.linalg.norm(T[n][:3, 3] - p_mj) * 1000
            dr = np.degrees(np.arccos(np.clip((np.trace(R_mj.T @ T[n][:3, :3]) - 1) / 2, -1, 1)))
            worst = max(worst, dp)
            if dp > 0.5 or dr > 0.5:
                print("MISMATCH %s trial %d: %.2f mm %.2f deg" % (n, trial, dp, dr))
    print("FK check: worst position difference %.4f mm over 20 random poses x %d links" % (worst, len(names)))

    # gravity check: pk.model vs MuJoCo's own bias forces (gravity only, no velocity)
    d.qvel[:] = 0
    q = np.array([-0.56, 1.09, -0.42, 0.0, -0.56, 0.40, 0.01, -1.0, -0.83, 0.23, 0.0])
    d.qpos[:] = m.qpos0
    for k, s in ((0, "left"), (5, "right")):
        for i, jn in enumerate(meta["arm"][s]):
            d.qpos[m.jnt_qposadr[m.joint(jn).id]] = q[k + i]
    m2 = mujoco.MjModel.from_xml_string(xml.replace('gravcomp="1"', 'gravcomp="0"'))
    d2 = mujoco.MjData(m2)
    d2.qpos[:] = d.qpos
    mujoco.mj_forward(m2, d2)
    T = rob.fk(q, np.zeros(12))
    for s, k in (("left", 0), ("right", 5)):
        dofs = [m2.jnt_dofadr[m2.joint(jn).id] for jn in meta["arm"][s]]
        print("%s arm gravity, motor torque to hold (N m): model %s  MuJoCo %s" % (
            s, np.round(rob.gravity(T, s), 3), np.round(d2.qfrc_bias[dofs], 3)))


if __name__ == "__main__":
    main()
