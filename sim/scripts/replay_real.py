"""Replay a logged run's joint commands in MuJoCo, open loop, and record what the simulated robot and plates did.

    python scripts/replay_real.py <push10_cmd.csv> --x 0.378 --y 0.067 --pelvis 0.707 --fric 0.6 -o out.npz
    python scripts/replay_real.py <log>... --no_plate      # the plates far away: the arm alone
    python scripts/replay_real.py <armid_cmd.csv> --no_plate --drive sent --arm_json ../models/sim_arm_R1.json -o out.npz
"""
import argparse
import csv
import json
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402

os.environ.setdefault("DEXPICK_KP_ARM", "50,50,40,40,30")    # the real arm gains, as scripts/push10_reference.py (the scene's default is kp 200)
os.environ.setdefault("DEXPICK_KV_ARM", "2")

from platepick.ik import PAD_POINT  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402

FRIC_ROBOT = np.array([0.90, 0.70, 0.25, 0.39, 0.45])      # N m, left arm (robot/model/gravity_fit.json), as rim_push.py
HAND_FULL = (("thumb_proximal_joint", 1.0472), ("thumb_metacarpal_joint", 1.55334), ("index_proximal_joint", 1.41),
             ("middle_proximal_joint", 1.41), ("ring_proximal_joint", 1.41), ("pinky_proximal_joint", 1.41))


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--x", type=float, default=0.36)
    ap.add_argument("--y", type=float, default=0.06)
    ap.add_argument("--pelvis", type=float, default=0.71)
    ap.add_argument("--table", type=float, default=0.747)
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--fric", type=float, default=0.6)
    ap.add_argument("--pad", type=float, default=0.25)
    ap.add_argument("--foot", type=float, default=0.575)
    ap.add_argument("--drive", default="cmd", choices=("cmd", "meas", "sent"), help="follow the logged q_cmd (tests the arm), sent (what the motors got: needs --arm_json or --arm_model) or q_meas (tests contact and plate)")
    ap.add_argument("--arm_json", default=None, help="sim_arm_R1.json: left arm with gravity on (scale x model + constant) and the measured Coulomb friction")
    ap.add_argument("--arm_model", default=None, help="sim_arm_model_<side>.json from robot/calibration/arm_model_fit.py: segment masses, friction ladder, armature, delay (platepick/r1_arm_model.py)")
    ap.add_argument("--armature", type=float, default=None, help="left arm joints; default the scene's")
    ap.add_argument("--side", default="left", choices=("left", "right"), help="the arm in the log")
    ap.add_argument("--no_plate", action="store_true")
    ap.add_argument("--t_end", type=float, default=1e9)
    ap.add_argument("-o", "--out", required=True)
    a = ap.parse_args()
    SIDE = a.side

    rows = [r for r in csv.DictReader(open(a.log)) if num(r["t"]) <= a.t_end]
    t = np.array([num(r["t"]) for r in rows])
    Q = np.array([[num(r[{"cmd": "q_cmd%d", "meas": "q_meas%d", "sent": "sent%d"}[a.drive] % i]) for i in range(5)] for r in rows])
    Q0 = np.array([num(rows[0]["q_meas%d" % i]) for i in range(5)])
    H = np.array([[num(r["hand%d" % i]) for i in range(6)] for r in rows])

    xy = (a.x + 2.0, a.y) if a.no_plate else (a.x, a.y)
    fr = Friction(glaze_glaze=0.375, foot_glaze=a.foot, pad_glaze=a.pad, table=0.45)
    sc = RobotScene(n=a.n, table=a.table, xy=xy, pelvis=a.pelvis, fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    for sd in ("left", "right"):
        for jn, v in zip(meta["arm"][sd], meta["ready"][sd]):
            d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v
            d.ctrl[m.actuator(f"a_{jn}").id] = v
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = [act[f"a_{j}"] for j in meta["arm"][SIDE]]
    jids = [m.joint(j).id for j in meta["arm"][SIDE]]
    adr = [m.jnt_qposadr[j] for j in jids]
    dof = [m.jnt_dofadr[j] for j in jids]
    HA = [act[f"a_{j}"] for j in meta["hand"][SIDE]]
    C = np.array(meta["coupling"][SIDE])
    hj = list(meta["hand"][SIDE])
    hhi = np.array([m.jnt_range[m.joint(j).id][1] for j in hj])
    svc = [(next(i for i, j in enumerate(hj) if j.endswith(nm)), full) for nm, full in HAND_FULL]
    S = np.array([C[k] / full for k, full in svc])

    def hand_q(h6):
        return np.minimum(C @ np.linalg.lstsq(S, h6, rcond=None)[0], hhi)

    d.qpos[adr] = Q0
    d.ctrl[A] = Q0
    d.ctrl[HA] = hand_q(H[0])
    d.qpos[[m.jnt_qposadr[m.joint(j).id] for j in hj]] = d.ctrl[HA]
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    m.dof_frictionloss[dof] = FRIC_ROBOT * a.fric
    g_scale, g_bias = None, None
    if a.arm_json:
        arm = json.load(open(a.arm_json))["parts"]
        g_scale, g_bias = np.array(arm["grav"]["scale"]), np.array(arm["grav"]["bias_nm"])
        m.dof_frictionloss[dof] = np.array(arm["fric"]["coulomb_nm"])
        for b in range(m.nbody):
            if m.body(b).name.startswith(SIDE + "_"):
                m.body_gravcomp[b] = 0.0
        mujoco.mj_forward(m, d)
    arm = None
    if a.arm_model:
        from platepick.r1_arm_model import R1ArmModel
        arm = R1ArmModel(m, SIDE, json.load(open(a.arm_model)), meta["arm"][SIDE])
        arm.setup(d, A)
        print("arm model %s: gravity %s, torque scale %.2f, delay %.0f ms" % (a.arm_model, arm.chosen, arm.k, 1000 * arm.delay))
    if a.armature is not None:
        m.dof_armature[dof] = a.armature
    pb = [m.body(f"plate{k}").id for k in range(a.n)]
    idx_b = m.body(f"{SIDE}_index_touch_link").id
    hand_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(SIDE + "_")}
    w6 = np.zeros(6)
    off = np.array([0.0, 0.0, a.pelvis])
    out = {k: [] for k in ("q", "idx", "plate_top", "plate_below", "con_top", "con_low")}
    t0 = time.time()
    dt = m.opt.timestep
    now = t[0]
    for k in range(len(t)):
        q_prev = Q[k - 1] if k else (Q[0] if a.drive == "sent" else Q0)
        N = max(1, int(round((t[k] - now) / dt)))
        for i in range(N):
            tg = q_prev + (Q[k] - q_prev) * (i + 1) / N
            d.ctrl[A] = arm.delayed(d.time, tg) if arm is not None else tg
            d.ctrl[HA] = hand_q(H[k]) if i == 0 else d.ctrl[HA]
            if arm is not None:
                arm.apply(d)
            elif g_scale is not None:                               # the motor holds scale x model + constant; MuJoCo's own gravity is the model
                d.qfrc_applied[dof] = -(g_scale - 1.0) * d.qfrc_bias[dof] - g_bias
            mujoco.mj_step(m, d)
        now += N * dt
        ct, cl = 0.0, 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            for g_a, g_b in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if g_a in hand_geoms and m.geom_bodyid[g_b] in pb:
                    mujoco.mj_contactForce(m, d, i, w6)
                    if m.geom_bodyid[g_b] == pb[-1]:
                        ct += float(np.linalg.norm(w6[:3]))
                    else:
                        cl += float(np.linalg.norm(w6[:3]))
        R = d.xmat[idx_b].reshape(3, 3)
        out["q"].append(d.qpos[adr].copy())
        out["idx"].append(d.xpos[idx_b] + R @ PAD_POINT - off)
        out["plate_top"].append(d.xpos[pb[-1]] - off)
        out["plate_below"].append(d.xpos[pb[-2]] - off)
        out["con_top"].append(ct)
        out["con_low"].append(cl)
        if k % 300 == 0:
            print("t %.1f / %.1f, wall %.0f s" % (t[k], t[-1], time.time() - t0), flush=True)
    np.savez_compressed(a.out, t=t, phase=np.array([r["phase"] for r in rows]), **{k: np.array(v) for k, v in out.items()})
    print("wrote", a.out, "mujoco", mujoco.__version__, "wall %.0f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
