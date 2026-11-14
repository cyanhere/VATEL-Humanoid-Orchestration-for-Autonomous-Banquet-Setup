"""Rim push on the full R1 + Revo2 model, closed loop on the signals the robot has (joints, fingertip touch, camera).
Modes for the hooked side push (push10) and the near-rim push.

    python scripts/rim_push.py --pose data/results/rim_push/pose_best.json [--n 3 --table 0.747...] [--video out.mp4 --save P.npz]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402

from platepick.forces import ForceMeter  # noqa: E402
from platepick.ik import PAD_NORMAL, PAD_POINT  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402
from rim_push_pose import place  # noqa: E402

FRIC_ROBOT = np.array([0.90, 0.70, 0.25, 0.39, 0.45])      # N m, left arm, robot/model/gravity_fit.json
DEFAULTS = dict(n=3, table=0.747, pelvis=0.71, x=0.36, y=0.06, foot_glaze=0.575, glaze_glaze=0.375, pad_glaze=0.25, table_mu=0.45,
                cam_x=0.0, cam_y=0.0, cam_z=0.0, cam_noise=1.0, jerr=0.0, jseed=0, fric=0.6, fnoise=2.0,
                tm=0.9, tp=0.6, ci=0.15, cm=0.55, cr=1.3, cp=1.3, yaw=0.0, pitch=10.0, roll=0.0,
                di0=27.0, up=20.0, di=6.0, lead_z=14.0, lead_y=10.0, v=30.0, stop=50.0, slip=12.0, retries=2, stall_lead=6.0, edge_lead=20.0, edge_drop=4.0, press_sink=12.0, open_end=1, edge_tol=30.0, f_press=0.0, f_lo=1.2, f_hi=6.0, dz_rate=0.3, dz_max=20.0, w_rot=0.2, w_rot_edge=0.05, gap_max=40.0, f_engage=3.0, out0=10.0, step=5.0, steps=12, below=6.0, edge_bias=6.5, set_err=0.0, retry_out=0.0, ftrace=0,
                mode="side", theta=40.0, inset=18.0, fwd=10.0, f_target=6.5, f_band=1.5, f_max=12.0, f_step=2.0, steer=1.0, ctrace=0,
                fpush=0, h_max=12.0, h_rate=6.0, fz_rate=4.0, t_push=8.0, cmdlog="", soft_pad="")
# soft_pad="K,B": the pushing hand's fingertips get a soft, damped contact (solref -K, -B; solimp 0.95)
# cmdlog: CSV of every 30 Hz command in the pelvis frame, as the robot's server would send it (logging only)
# mode="near": near-rim two-finger push with its own hand shape and wrist, a deeper force press, slower
NEAR = dict(tm=0.0, tp=0.0, ci=0.0, cm=0.0, cr=1.3, cp=1.3, yaw=30.0, pitch=10.0, dz_max=40.0, v=20.0, below=15.0)
SIDE = "left"


def run(par, video=None, title="", save=None):
    p = {**DEFAULTS, **(NEAR if par.get("mode") == "near" else {}), **par}
    n = int(p["n"])
    fr = Friction(glaze_glaze=p["glaze_glaze"], foot_glaze=p["foot_glaze"], pad_glaze=p["pad_glaze"], table=p["table_mu"])
    sc = RobotScene(n=n, table=p["table"], xy=(p["x"], p["y"]), pelvis=p["pelvis"], fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    if p["soft_pad"]:
        k_, b_ = (float(v_) for v_ in p["soft_pad"].split(","))
        for g in range(m.ngeom):
            bn = m.body(m.geom_bodyid[g]).name
            if bn.startswith(SIDE + "_") and bn.endswith(("_distal_link", "_tip_link", "_touch_link")) and m.geom_contype[g]:
                m.geom_solref[g] = (-k_, -b_)
                m.geom_solimp[g, :3] = (0.95, 0.95, 0.001)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    # both arms at their ready pose (as state and as target) before the plates settle (from zero joints the
    # first move swept the arm through the stack)
    for sd in ("left", "right"):
        for jn, v in zip(meta["arm"][sd], meta["ready"][sd]):
            d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v
            d.ctrl[m.actuator(f"a_{jn}").id] = v
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    pb = [m.body(f"plate{k}").id for k in range(n)]
    top = pb[-1]
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = [act[f"a_{j}"] for j in meta["arm"][SIDE]]
    jids = [m.joint(j).id for j in meta["arm"][SIDE]]
    adr = [m.jnt_qposadr[j] for j in jids]
    dof = [m.jnt_dofadr[j] for j in jids]
    lo = np.array([m.jnt_range[j][0] for j in jids])
    hi = np.array([m.jnt_range[j][1] for j in jids])
    HA = [act[f"a_{j}"] for j in meta["hand"][SIDE]]
    C = np.array(meta["coupling"][SIDE])
    hhi = np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"][SIDE]])
    m.dof_frictionloss[dof] = FRIC_ROBOT * p["fric"]
    kp, kv = m.actuator_gainprm[A, 0].copy(), -m.actuator_biasprm[A, 2].copy()
    rng = np.random.default_rng(int(p["jseed"]))
    jerr = rng.normal(0, 1, 5) * math.radians(p["jerr"])               # encoder zero errors: measured = true + jerr
    idx_b = m.body(f"{SIDE}_index_touch_link").id
    palm = m.body(f"{SIDE}_base_link").id
    hand_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(SIDE + "_")}
    plate_bodies = set(pb)
    fm = ForceMeter(m, SIDE, top, pb[-2] if n > 1 else None, hands=(SIDE,))
    fm.R_edge = spec.R
    hand = np.array([p["tm"], p["tp"], p["ci"], p["cm"], p["cr"], p["cp"]])
    hand_q = np.minimum(C @ hand, hhi)
    # robot=1: the measured arm (R1ArmModel, torque scale 1) and the server's gravity offset (deploy GravityComp);
    # the force the robot can compute is then kp (planned - measured) - kd dq, as the server's lag force minus gravity
    arm_m = comp = None
    des = dict(q=None)
    if p.get("robot"):
        from platepick.r1_arm_model import R1ArmModel
        from platepick.r1_server_sim import ARM_JSON, _deploy
        cfg_d, GC = _deploy()
        comp = GC()
        fit = json.load(open(ARM_JSON % SIDE))
        fit["gravity"]["torque_scale"] = 1.0
        arm_m = R1ArmModel(m, SIDE, fit, meta["arm"][SIDE])
        arm_m.setup(d, A)
        other = "right" if SIDE == "left" else "left"
        adr_o = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][other]]

    # ---- the robot's view
    def q_meas():
        return d.qpos[adr] + jerr

    db = mujoco.MjData(m)                    # belief kinematics (measured joints, the model's arm)

    def belief(q):
        db.qpos[:] = d.qpos
        db.qpos[adr] = q
        mujoco.mj_kinematics(m, db)
        mujoco.mj_comPos(m, db)              # mj_jac needs it
        return db

    def idx_point(dd):
        R = dd.xmat[idx_b].reshape(3, 3)
        return dd.xpos[idx_b] + R @ PAD_POINT

    jac = np.zeros((3, m.nv))
    jacr = np.zeros((3, m.nv))

    def lag_force():
        """world force on the index fingertip from the joint lag (what the server can compute), + noise"""
        tau = kp * ((d.ctrl[A] if arm_m is None or des["q"] is None else des["q"]) - d.qpos[adr]) - kv * d.qvel[dof]
        bb = belief(q_meas())
        mujoco.mj_jac(m, bb, jac, None, idx_point(bb), idx_b)
        F = np.linalg.lstsq(jac[:, dof].T, -tau, rcond=None)[0]
        return F + rng.normal(0, p["fnoise"] / 2, 3)

    gid = np.zeros(1, np.int32)
    w6 = np.zeros(6)

    pad_b = {f_: m.body(f"{SIDE}_{f_}_touch_link").id for f_ in ("thumb", "index", "middle", "ring", "pinky")}

    def pad_sense(b):
        """one fingertip pad: normal force on the pad link (+- 0.25 N) and proximity along the pad normal (<= 10 mm, else 99);
        the ray skips the hand itself"""
        R = d.xmat[b].reshape(3, 3)
        hit = mujoco.mj_ray(m, d, d.xpos[b] + R @ PAD_POINT, R @ PAD_NORMAL, None, 1, b, gid)
        ok_ = hit >= 0 and hit < 0.010 and gid[0] >= 0 and gid[0] not in hand_geoms
        Fn = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            if b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                mujoco.mj_contactForce(m, d, i, w6)
                Fn += abs(w6[0])
        return max(0.0, Fn + rng.normal(0, 0.25)), (hit * 1000 if ok_ else 99.0)

    pad_now = {}

    def pad_sum():
        """near mode: the index + middle pads' forces together, as last sensed"""
        return pad_now["index"][0] + pad_now["middle"][0]

    def index_sense():
        """(index pad force, index proximity, min proximity of any pad, max pad force of any pad)"""
        all_ = {f_: pad_sense(b) for f_, b in pad_b.items()}
        pad_now.update(all_)
        return all_["index"][0], all_["index"][1], min(v[1] for v in all_.values()), max(v[0] for v in all_.values())

    cam_bias = np.array([p["cam_x"], p["cam_y"], p["cam_z"]]) / 1000

    def camera():
        return d.xpos[top] + cam_bias + rng.normal(0, p["cam_noise"] / 1000, 3)

    # ---- truth for the report
    def contacts():
        out = {}
        for i in range(d.ncon):
            c = d.contact[i]
            for a_, b_ in ((c.geom1, c.geom2), (c.geom2, c.geom1)):
                if a_ not in hand_geoms:
                    continue
                nb = m.geom_bodyid[b_]
                what = "top" if nb == top else ("lower" if nb in plate_bodies else ("table" if nb == 0 else None))
                if what is None:
                    continue
                part = next((f for f in ("thumb", "index", "middle", "ring", "pinky") if f"_{f}_" in m.body(m.geom_bodyid[a_]).name), "palm")
                mujoco.mj_contactForce(m, d, i, w6)
                k = f"{part}-{what}"
                out[k] = out.get(k, 0.0) + float(np.linalg.norm(w6[:3]))
        return out

    sol_seen = {}                            # the solref / solimp MuJoCo used in the first index / middle pad contact with the top plate

    def finger_forces():
        """Ground truth per hand part: the
        world force it puts on the TOP plate (N) and the force-weighted contact point (world); per fingertip pad, the normal
        force without noise (what the pad sensor reads)"""
        F, C, W, pads = {}, {}, {}, {f_: 0.0 for f_ in pad_b}
        for i in range(d.ncon):
            c = d.contact[i]
            mujoco.mj_contactForce(m, d, i, w6)
            for f_, b in pad_b.items():                    # as pad_sense
                if b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                    pads[f_] += abs(w6[0])
                    if f_ in ("index", "middle") and f_ not in sol_seen and top in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                        sol_seen[f_] = dict(solref=[round(float(v_), 4) for v_ in c.solref], solimp=[round(float(v_), 4) for v_ in c.solimp[:3]])
            for a_, b_, sg in ((c.geom1, c.geom2, 1.0), (c.geom2, c.geom1, -1.0)):
                if a_ not in hand_geoms or m.geom_bodyid[b_] != top:
                    continue
                part = next((f for f in ("thumb", "index", "middle", "ring", "pinky") if f"_{f}_" in m.body(m.geom_bodyid[a_]).name), "palm")
                f_w = sg * (c.frame.reshape(3, 3).T @ w6[:3])     # the normal points geom1 -> geom2; +f pushes geom2 along it
                w_ = float(np.linalg.norm(f_w))
                F[part] = F.get(part, np.zeros(3)) + f_w
                C[part] = C.get(part, np.zeros(3)) + w_ * c.pos
                W[part] = W.get(part, 0.0) + w_
        return {k: (v, C[k] / max(W[k], 1e-9)) for k, v in F.items()}, pads

    # ---- motion: 30 Hz commands, joint speed limit, belief-frame Cartesian targets for the index pad point
    plan = dict(R=None)
    rec = dict(peak={}, peak_lower_phase={}, lower=0.0, table=0.0, phases=[], trace=[])
    vstate = dict(next=0.0, phase="")
    vid = None
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))
        tgt0 = d.xpos[top] + np.array([0, spec.R, 0.02])
        from rim_push_pose import free_cam
        cams = [free_cam(tgt0 + np.array(e), tgt0) for e in ((0.30, 0.12, 0.10), (0.02, 0.30, 0.05), (0.45, -0.25, 0.25), (-0.05, 0.0, 0.45))]
        if p["mode"] == "near":      # top view with the robot at the bottom , far side, robot's right, robot's left
            tgt0 = d.xpos[top] + np.array([-0.05, 0.0, 0.02])
            cams = [free_cam(tgt0 + np.array(e), tgt0) for e in ((-0.05, 0.0, 0.50), (0.42, 0.10, 0.14), (0.02, -0.40, 0.07), (-0.08, 0.40, 0.12))]

    def grab(sense):
        import cv2
        if d.time < vstate["next"]:
            return
        vstate["next"] += 0.04
        ims = []
        for cm_ in cams:
            vr.update_scene(d, camera=cm_)
            ims.append(cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR))
        bar = np.full((64, 1280, 3), 20, np.uint8)
        cv2.putText(bar, title[:160], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        ct = contacts()
        slide = float(np.linalg.norm((d.xpos[top] - d.xpos[pb[-2]])[:2] - rel0["v"])) * 1000 if n > 1 else 0.0
        pads_txt = f"index+middle pads {pad_sum():4.1f} N" if p["mode"] == "near" else f"index pad {sense[0]:4.1f} N prox {min(sense[1], 9.9):3.1f} mm"
        cv2.putText(bar, f"t {d.time:4.1f}s {vstate['phase']:<8s} slide {slide:5.1f} mm | {pads_txt} | "
                    + " ".join(f"{k} {v:.1f}" for k, v in sorted(ct.items())), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    cmd = dict(q=None)
    # the Revo2 service's six commands (0..1: thumb bend, thumb rotation, index, middle, ring, little) from the hand joints
    # (robot/pk/config.py HAND_FULL_RAD)
    hj = list(meta["hand"][SIDE])
    svc = [(next(i for i, j in enumerate(hj) if j.endswith(nm)), full) for nm, full in
           (("thumb_proximal_joint", 1.0472), ("thumb_metacarpal_joint", 1.55334), ("index_proximal_joint", 1.41),
            ("middle_proximal_joint", 1.41), ("ring_proximal_joint", 1.41), ("pinky_proximal_joint", 1.41))]
    clog = []
    off_p = np.array([0.0, 0.0, p["pelvis"]])

    def clog_row(phase, s, ct):
        # no draws from `rng` here (camera() / lag_force() add noise): the logged run stays the run without the log
        cam = d.xpos[top] + cam_bias
        tau = kp * ((d.ctrl[A] if arm_m is None or des["q"] is None else des["q"]) - d.qpos[adr]) - kv * d.qvel[dof]
        bb = belief(q_meas())
        mujoco.mj_jac(m, bb, jac, None, idx_point(bb), idx_b)
        F_lag = np.linalg.lstsq(jac[:, dof].T, -tau, rcond=None)[0]
        r_ = dict(t=round(d.time, 4), phase=phase)
        r_.update({f"q_cmd{i}": round(float(v), 5) for i, v in enumerate(cmd["q"])})          # encoder frame (what is sent)
        r_.update({f"q_meas{i}": round(float(v), 5) for i, v in enumerate(q_meas())})
        r_.update({f"hand{i}": round(float(hand_q[k] / full), 4) for i, (k, full) in enumerate(svc)})
        r_.update({f"idx_fk_{a}": round(float(v), 4) for a, v in zip("xyz", idx_point(bb) - off_p)})   # index pad by FK of the encoders
        r_.update({f"tgt_{a}": round(float(v), 4) for a, v in zip("xyz", ikres.get("P", off_p) - off_p)})  # Cartesian target of the index pad
        r_.update(ik_mm=round(ikres["pos_mm"], 1), ik_deg=round(ikres["rot_deg"], 1))                    # what the IK could not reach
        r_.update({f"idx_true_{a}": round(float(v), 4) for a, v in zip("xyz", idx_point(d) - off_p)})
        r_.update({f"cam_{a}": round(float(v), 4) for a, v in zip("xyz", cam - off_p)})                 # camera (bias, no noise): top plate centre
        r_.update({f"plate_{a}": round(float(v), 4) for a, v in zip("xyz", d.xpos[top] - off_p)})
        r_.update(idx_pad_N=round(float(s[0]), 2), idx_prox_mm=round(float(min(s[1], 99.0)), 1))
        r_.update({f"pad_{f_}_N": round(float(pad_now[f_][0]), 2) for f_ in ("thumb", "index", "middle", "ring", "pinky")})
        r_.update({f"lag_F{a}": round(float(v), 2) for a, v in zip("xyz", F_lag)})                     # joint-lag force (no noise)
        r_["contacts"] = " ".join(f"{k}={v:.1f}" for k, v in sorted(ct.items()))
        clog.append(r_)
    rel0 = dict(v=(d.xpos[top] - d.xpos[pb[-2]])[:2].copy() if n > 1 else np.zeros(2))

    def grav_off(qd):
        q11 = np.zeros(11)
        sl, so = (slice(0, 5), slice(5, 10)) if SIDE == "left" else (slice(5, 10), slice(0, 5))
        q11[sl], q11[so] = qd, d.qpos[adr_o]
        h12 = np.zeros(12)
        k0 = 0 if SIDE == "left" else 6
        for i, (k, full) in enumerate(svc):
            h12[k0 + i] = hand_q[k] / full
        return comp.offset(q11, h12, cfg_d.KP)[sl]

    def tick(q_target, phase, T=1 / 30):
        """one 30 Hz command: joint target (speed-limited), the server ramps to it; returns sensors after it"""
        q_prev = cmd["q"]
        step = np.clip(q_target - q_prev, -0.8 * T, 0.8 * T)
        q_new = np.clip(q_prev + step, lo, hi)
        N = int(round(T / dt))
        nsrv = max(1, int(round(0.01 / dt)))
        for k in range(N):
            qd = q_prev + (q_new - q_prev) * (k + 1) / N - jerr          # the servo tracks the (biased) encoder
            if arm_m is None:
                d.ctrl[A] = qd
            else:
                if k % nsrv == 0 or des["q"] is None:
                    des["sent"] = np.clip(qd + grav_off(qd), lo + 0.035, hi - 0.035)   # server: 2 deg inside the limits
                des["q"] = qd
                d.ctrl[A] = arm_m.delayed(d.time, des["sent"])
                arm_m.apply(d)
            d.ctrl[HA] = hand_q
            mujoco.mj_step(m, d)
            fm.update(d)
        cmd["q"] = q_new
        s = index_sense()
        ct = contacts()
        if p["cmdlog"]:
            clog_row(phase, s, ct)
        for k_, v_ in ct.items():
            rec["peak"][k_] = max(rec["peak"].get(k_, 0.0), v_)
            if k_.endswith("-lower"):              # WHEN a finger touches the plate below (edge search vs push)
                ph_ = rec["peak_lower_phase"].setdefault(phase, {})
                ph_[k_] = max(ph_.get(k_, 0.0), v_)
        vstate["phase"] = phase
        if vid is not None:
            grab(s)
        return s

    ikres = dict(pos_mm=0.0, rot_deg=0.0)
    wrot = dict(v=p["w_rot"])

    def ik_to(P, q0):
        """belief-frame IK: index pad point to P, palm orientation as planned (small steps, damped least squares)"""
        q = q0.copy()
        best = None
        for _ in range(60):
            bb = belief(q)
            e = P - idx_point(bb)
            R = bb.xmat[palm].reshape(3, 3)
            er = 0.5 * (np.cross(R[:, 0], plan["R"][:, 0]) + np.cross(R[:, 1], plan["R"][:, 1]) + np.cross(R[:, 2], plan["R"][:, 2]))
            c = float(np.linalg.norm(e) ** 2 + (wrot["v"] * np.linalg.norm(er)) ** 2)
            if best is None or c < best[0]:
                best = (c, q.copy(), e, er)
            if np.linalg.norm(e) < 2e-4 and np.linalg.norm(er) < 2e-3:
                break
            mujoco.mj_jac(m, bb, jac, None, idx_point(bb), idx_b)
            mujoco.mj_jacBody(m, bb, None, jacr, palm)
            # low rotation weight (w_rot_edge) on the edge slide only: the 5-DoF arm cannot hold position and full orientation
            J = np.vstack([jac[:, dof], wrot["v"] * jacr[:, dof]])
            err = np.r_[e, wrot["v"] * er]
            # damping 1e-4 (1e-6 diverged near a singular pose), steps <= 0.1 rad, keep the best iterate
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err)
            dq *= min(1.0, 0.1 / max(1e-9, float(np.max(np.abs(dq)))))
            q = np.clip(q + dq, lo, hi)
        _, q, e, er = best
        ikres.update(pos_mm=float(np.linalg.norm(e)) * 1000, rot_deg=math.degrees(float(np.linalg.norm(er))), P=P.copy())
        return q

    def cart_move(P_from, P_to, speed_mm_s, phase, q0, stop=None):
        L = float(np.linalg.norm(P_to - P_from)) * 1000
        K = max(1, int(L / speed_mm_s * 30))
        q = q0
        for k in range(1, K + 1):
            P = P_from + (P_to - P_from) * k / K
            q = ik_to(P, q)
            s = tick(q, phase)
            if k % 10 == 0 or k == K:
                Fl = lag_force()
                rec["trace"].append(dict(t=round(d.time, 2), ph=phase, P_cmd=[round(v * 1000, 1) for v in P], idx_true=[round(v * 1000, 1) for v in idx_point(d)],
                                         idx_belief_cmd=[round(v * 1000, 1) for v in idx_point(belief(q))], pad=round(s[0], 2), prox=round(s[1], 1),
                                         any_prox=round(s[2], 1), any_pad=round(s[3], 2), lag=[round(v, 1) for v in Fl],
                                         idx_normal=[round(v, 2) for v in (d.xmat[idx_b].reshape(3, 3) @ PAD_NORMAL)],
                                         q_cmd=[round(float(v), 3) for v in q], ik_mm=round(ikres["pos_mm"], 1), ik_deg=round(ikres["rot_deg"], 1)))
            if stop is not None:
                r_ = stop(s, P)
                if r_:
                    return q, P, r_, s
        return q, P_to, None, s

    lift_extra = 0.0
    if p["mode"] == "near":
        # near-rim two-finger push: palm down, index and middle tips on the rim on the robot's side, `theta` deg from the
        # nearest point toward the left, `inset` mm inside the edge; ring and little curled. The pads press at
        # f_target +- f_band N and drag the plate toward the robot's right, `fwd` deg forward; the camera steers the hand's x.
        # Slip (hand ahead of the plate by > `slip` mm): lift, press f_step N harder (<= f_max), retry.
        from near_rim_pose import place_near, rim_point
        P0_rel = rel0["v"]
        cam_start = camera()
        p0_true, i0_true = d.xpos[top].copy(), idx_point(d).copy()
        Rt0 = d.xmat[top].reshape(3, 3).copy()
        d_tmp = mujoco.MjData(m)
        d_tmp.qpos[:] = d.qpos
        mujoco.mj_forward(m, d_tmp)
        q_plan, info = place_near(m, d_tmp, meta, spec, top, hand, p["theta"], p["inset"], p["yaw"], p["pitch"], p["roll"], dz=p["up"])
        plan["R"] = d_tmp.xmat[palm].reshape(3, 3).copy()
        out = dict(par=p, pose_ik=info, events=[])
        cmd["q"] = d.qpos[adr] + jerr
        u = np.array([math.sin(math.radians(p["fwd"])), -math.cos(math.radians(p["fwd"])), 0.0])
        # start 60 mm straight above the planned pose with the IK seeded from it, so the arm stays on the planned branch
        P = cam_start + rim_point(spec, p["theta"], p["inset"]) + np.array([0, 0, p["up"] / 1000 + 0.06])
        q = ik_to(P, q_plan + jerr)
        for _ in range(int(3.0 * 30)):
            tick(q, "pre")
        tries, result, slide_cam, dist, dz, f_t = 0, "no push", 0.0, 0.0, 0.0, p["f_target"]
        while tries <= p["retries"]:
            T = camera() + rim_point(spec, p["theta"], p["inset"])
            q, P, _, s = cart_move(P, T + np.array([0, 0, p["up"] / 1000]), 40.0, "pre", q)
            zh, fh = [], []

            def touched_n(s_, P_):
                # the index + middle pads together > 1 N for 3 ticks, or a stall confirmed by the joint-lag force up
                zb = idx_point(belief(q_meas()))[2]
                zh.append(zb)
                fh.append(pad_sum())
                if len(fh) >= 3 and min(fh[-3:]) > 1.0:
                    return "pads"
                if len(zh) > 10 and zh[-10] - zb < 0.001 and zb - P_[2] > p["stall_lead"] / 1000 and lag_force()[2] > 4.0:
                    return "stall"
                return None
            q, P, hit, s = cart_move(P, T - np.array([0, 0, (p["below"] + p["stall_lead"]) / 1000]), 12.0, "touch", q, stop=touched_n)
            if not hit:
                result = "no rim found"
                break
            P = np.array([P[0], P[1], idx_point(belief(q_meas()))[2]])
            if p["fpush"]:
                # force mode: press and push are commanded as a force on the plate through the arm's stiffness.
                # Target = touch point + plate motion (camera) + C @ F, C = J Kq^-1 J^T at the measured joints.
                # F = [H toward `fwd` deg forward of the robot's right, Fz down]; Fz holds the pad sum at f_t, H ramps at h_rate
                # until the plate moves, then holds its speed near `v`. Slip: hand vs plate > `slip` mm.
                Kq_inv = np.diag(1.0 / kp)
                from near_rim_fit4 import Gaps          # true finger-to-plate gaps in the trace (logging)
                gaps_ = Gaps(m, n)
                P_t, cs0, cs_h, q_f = P.copy(), camera(), [], q.copy()
                Fz, H, a_cmd, done, pressed, fh, kk, dz = 0.0, 0.0, p["fwd"], None, False, [], 0, 0.0
                while kk < int(p["t_push"] * 30):
                    kk += 1
                    cs = camera()
                    cs_h.append(cs)
                    dp = cs - cs0
                    f_ = sum(fh[-5:]) / len(fh[-5:]) if fh else 0.0
                    sp = float(np.linalg.norm((cs - cs_h[-6])[:2])) * 6000 if len(cs_h) >= 6 else 0.0      # mm/s over 5 ticks
                    if not pressed:
                        Fz = min(Fz + p["fz_rate"] / 30, 3 * p["f_max"])
                        if len(fh) >= 3 and sum(fh[-3:]) / 3 >= f_t:
                            pressed = True
                            cs_p, i_p, k_p = cs.copy(), idx_point(belief(q_meas())), kk
                            Rt_ = d.xmat[top].reshape(3, 3)
                            ip = Rt_.T @ (idx_point(d) - d.xpos[top])
                            out["events"].append(dict(t=round(d.time, 2), ev="pressed", touch=hit, Fz=round(Fz, 1), pads_N=round(f_, 1), f_target=f_t,
                                                      inset_true_mm=round((spec.R - float(np.hypot(ip[0], ip[1]))) * 1000, 1),
                                                      theta_true_deg=round(math.degrees(math.atan2(ip[1], -ip[0])), 1)))
                    else:
                        Fz = float(np.clip(Fz + 0.05 * (f_t - f_), 0.0, 3 * p["f_max"]))
                        mv = (cs - cs_p)[:2]
                        if float(np.linalg.norm(mv)) > 0.005:
                            head = math.degrees(math.atan2(mv[0], -mv[1]))
                            a_cmd = float(np.clip(p["fwd"] - p["steer"] * (head - p["fwd"]), p["fwd"] - 40, p["fwd"] + 20))
                        H = float(np.clip(H + np.clip(0.004 * (p["v"] - sp), -0.3, p["h_rate"] / 30), 0.0, p["h_max"]))
                    a_ = math.radians(a_cmd)
                    F = np.array([H * math.sin(a_), -H * math.cos(a_), -Fz])
                    # apply the force as joint offsets on top of the plate-following pose: dq = Kq^-1 J^T F
                    bb = belief(q_meas())
                    mujoco.mj_jac(m, bb, jac, None, idx_point(bb), idx_b)
                    dq_f = Kq_inv @ jac[:, dof].T @ F
                    off = jac[:, dof] @ dq_f
                    dz = float(-off[2])
                    P = P_t + dp
                    q_f = ik_to(P, q_f)
                    q = q_f + dq_f
                    tick(q, "push" if pressed else "press")
                    fh.append(pad_sum())
                    if len(rec["trace"]) < 5000 and kk % 3 == 0:
                        Fp, pads_ = finger_forces()
                        Rt_ = d.xmat[top].reshape(3, 3)
                        Rr = Rt_ @ Rt0.T
                        Rp = d.xmat[palm].reshape(3, 3)
                        rec["trace"].append(dict(t=round(d.time, 2), ph="push_n" if pressed else "press_n", dist=round((kk - (k_p if pressed else 0)) * p["v"] / 30, 1),
                                                 f_t=f_t, pads_sensed=round(f_, 2), H=round(H, 2), Fz=round(Fz, 2), a_cmd=round(a_cmd, 1), sp=round(sp, 1),
                                                 off=[round(float(v) * 1000, 1) for v in off],
                                                 fpitch=round(math.degrees(math.asin(max(-1.0, min(1.0, -Rp[2, 2])))), 1),
                                                 palm_tilt=round(math.degrees(math.acos(max(-1.0, min(1.0, -Rp[2, 0])))), 1),
                                                 gap={f_: {k_: v_ for k_, v_ in g_.items() if k_ in ("top", "r", "theta")} for f_, g_ in gaps_(d).items() if f_ in ("index", "middle")},
                                                 lower={k_: round(v, 1) for k_, v in contacts().items() if not k_.endswith("-top")},
                                                 dz=round(dz * 1000, 1), steer=round(a_cmd, 1),
                                                 plate=[round(float(v), 1) for v in (d.xpos[top] - p0_true) * 1000],
                                                 index=[round(float(v), 1) for v in (idx_point(d) - i0_true) * 1000],
                                                 yaw=round(math.degrees(math.atan2(Rr[1, 0], Rr[0, 0])), 2),
                                                 tilt=round(math.degrees(math.acos(min(1.0, Rt_[2, 2]))), 2),
                                                 F={k_: [round(float(-v[0][1]), 2), round(float(-v[0][0]), 2), round(float(-v[0][2]), 2)] for k_, v in Fp.items()},
                                                 pads={k_: round(float(v), 2) for k_, v in pads_.items()}))
                    if p["ctrace"] and pressed and len(rec["trace"]) < 5000:          # every contact, every tick (as the push below)
                        # per tick: each index / middle contact with the top plate (friction, depth, normal and tangential force, point in
                        # the plate frame) and the index pad point in the plate frame
                        Rt_ = d.xmat[top].reshape(3, 3)
                        cl = []
                        for i in range(d.ncon):
                            c = d.contact[i]
                            gs = (c.geom1, c.geom2)
                            if top not in (m.geom_bodyid[gs[0]], m.geom_bodyid[gs[1]]):
                                continue
                            hg = next((g for g in gs if g in hand_geoms), None)
                            if hg is None:
                                continue
                            mujoco.mj_contactForce(m, d, i, w6)
                            pg = gs[1] if hg == gs[0] else gs[0]
                            cl.append(dict(h=m.body(m.geom_bodyid[hg]).name.replace("left_", ""), g=m.geom(pg).name.split("_")[-1], mu=round(float(c.friction[0]), 2),
                                           depth=round(-c.dist * 1000, 2), fn=round(float(w6[0]), 2), ft=round(float(np.hypot(w6[1], w6[2])), 2),
                                           n=[round(float(v), 2) for v in c.frame[:3]], at=[round(float(v), 1) for v in Rt_.T @ (c.pos - d.xpos[top]) * 1000]))
                        rec["trace"].append(dict(t=round(d.time, 3), ph="push_c", dist=round((kk - k_p) * p["v"] / 30, 1),
                                                 tip=[round(float(v), 1) for v in Rt_.T @ (idx_point(d) - d.xpos[top]) * 1000], c=cl))
                    if pressed:
                        if (cam_start[1] - cs[1]) * 1000 >= p["stop"]:
                            done = "pushed"
                            break
                        hand_mv = (idx_point(belief(q_meas())) - i_p)[:2]
                        if float(np.linalg.norm(hand_mv - (cs - cs_p)[:2])) * 1000 > p["slip"]:
                            done = "slip"
                            break
                if not pressed:
                    done = "no press"
            else:
                # press: deeper until the two pads read f_t (3-tick mean)
                dz, fh = 0.0, []
                for _ in range(150):
                    q = ik_to(P - np.array([0, 0, dz]), q)
                    tick(q, "press")
                    fh.append(pad_sum())
                    if sum(fh[-3:]) / len(fh[-3:]) >= f_t:
                        break
                    dz = min(dz + p["dz_rate"] / 1000, p["dz_max"] / 1000)
                Rt_ = d.xmat[top].reshape(3, 3)               # truth, for the log only: where the index pad sits on the plate
                ip = Rt_.T @ (idx_point(d) - d.xpos[top])
                out["events"].append(dict(t=round(d.time, 2), ev="pressed", touch=hit, extra_mm=round(dz * 1000, 1), pads_N=round(fh[-1], 1), f_target=f_t,
                                          inset_true_mm=round((spec.R - float(np.hypot(ip[0], ip[1]))) * 1000, 1),
                                          theta_true_deg=round(math.degrees(math.atan2(ip[1], -ip[0])), 1)))
                # push along u, the press held, the camera steering x
                P_set, cs0, i0 = P.copy(), camera(), idx_point(belief(q_meas()))
                dist, sx, done = 0.0, 0.0, None
                while dist < (p["stop"] + 60) / 1000:
                    dist += p["v"] / 1000 / 30
                    cs = camera()
                    sx = float(np.clip(sx - p["steer"] * (cs[0] - cam_start[0]) / 30, -0.025, 0.025))
                    P = P_set + dist * u + np.array([sx, 0, -dz])
                    q = ik_to(P, q)
                    tick(q, "push")
                    fh.append(pad_sum())
                    f_ = sum(fh[-5:]) / len(fh[-5:])
                    if f_ < f_t - p["f_band"]:
                        dz = min(dz + p["dz_rate"] / 1000, p["dz_max"] / 1000)
                    elif f_ > f_t + p["f_band"]:
                        dz = max(dz - p["dz_rate"] / 1000, -0.005)
                    if len(rec["trace"]) < 5000 and len(fh) % 3 == 0:
                        # truth, 10 Hz: per-part forces on the top plate [along -y, toward the robot (-x), down] N, the pads'
                        # true readings, the plate's and the index pad's paths since the start (world mm), the plate's yaw
                        Fp, pads_ = finger_forces()
                        Rt_ = d.xmat[top].reshape(3, 3)
                        Rr = Rt_ @ Rt0.T
                        Rp = d.xmat[palm].reshape(3, 3)            # fingers' down-tilt, palm normal vs straight down (deg)
                        rec["trace"].append(dict(t=round(d.time, 2), ph="push_n", dist=round(dist * 1000, 1), f_t=f_t, pads_sensed=round(f_, 2),
                                                 fpitch=round(math.degrees(math.asin(max(-1.0, min(1.0, -Rp[2, 2])))), 1),
                                                 palm_tilt=round(math.degrees(math.acos(max(-1.0, min(1.0, -Rp[2, 0])))), 1),
                                                 lower={k: round(v, 1) for k, v in contacts().items() if not k.endswith("-top")},
                                                 dz=round(dz * 1000, 1), steer=round(sx * 1000, 1),
                                                 plate=[round(float(v), 1) for v in (d.xpos[top] - p0_true) * 1000],
                                                 index=[round(float(v), 1) for v in (idx_point(d) - i0_true) * 1000],
                                                 yaw=round(math.degrees(math.atan2(Rr[1, 0], Rr[0, 0])), 2),
                                                 tilt=round(math.degrees(math.acos(min(1.0, Rt_[2, 2]))), 2),
                                                 F={k: [round(float(-v[0][1]), 2), round(float(-v[0][0]), 2), round(float(-v[0][2]), 2)] for k, v in Fp.items()},
                                                 pads={k: round(float(v), 2) for k, v in pads_.items()}))
                    if p["ctrace"] and len(rec["trace"]) < 5000:
                        # per tick: each index / middle contact with the top plate (friction, depth, normal and tangential force, point in
                        # the plate frame) and the index pad point in the plate frame
                        Rt_ = d.xmat[top].reshape(3, 3)
                        cl = []
                        for i in range(d.ncon):
                            c = d.contact[i]
                            gs = (c.geom1, c.geom2)
                            if top not in (m.geom_bodyid[gs[0]], m.geom_bodyid[gs[1]]):
                                continue
                            hg = next((g for g in gs if g in hand_geoms), None)
                            if hg is None:
                                continue
                            mujoco.mj_contactForce(m, d, i, w6)
                            pg = gs[1] if hg == gs[0] else gs[0]
                            cl.append(dict(h=m.body(m.geom_bodyid[hg]).name.replace("left_", ""), g=m.geom(pg).name.split("_")[-1], mu=round(float(c.friction[0]), 2),
                                           depth=round(-c.dist * 1000, 2), fn=round(float(w6[0]), 2), ft=round(float(np.hypot(w6[1], w6[2])), 2),
                                           n=[round(float(v), 2) for v in c.frame[:3]], at=[round(float(v), 1) for v in Rt_.T @ (c.pos - d.xpos[top]) * 1000]))
                        rec["trace"].append(dict(t=round(d.time, 3), ph="push_c", dist=round(dist * 1000, 1),
                                                 tip=[round(float(v), 1) for v in Rt_.T @ (idx_point(d) - d.xpos[top]) * 1000], c=cl))
                    # slip: hand vs plate in 2-D (a plate moving with the finger is not a slip)
                    plate_mv = (cs - cs0)[:2]
                    hand_mv = (idx_point(belief(q_meas())) - i0)[:2]
                    if (cam_start[1] - cs[1]) * 1000 >= p["stop"]:
                        done = "pushed"
                        break
                    if float(np.linalg.norm(hand_mv - plate_mv)) * 1000 > p["slip"]:
                        done = "slip"
                        break
            slide_cam = (cam_start[1] - camera()[1]) * 1000
            out["events"].append(dict(t=round(d.time, 2), ev=done or "push travel limit", camera_slide_mm=round(slide_cam, 1), f_target=f_t,
                                      extra_mm=round(dz * 1000, 1)))
            if done == "pushed":
                result = "pushed"
                break
            tries += 1
            result = done or "push travel limit"
            f_t = min(f_t + p["f_step"], p["f_max"])
            q, P, _, s = cart_move(P, P + np.array([0, 0, 0.020 + max(dz, 0.0)]), 20.0, "retry", q)
            dz = 0.0
        lift_extra = max(dz, 0.0)             # the press is commanded below the rim: lift that much more on release
    else:
        # ---- 1 plan from the camera
        cam0 = camera()
        Rt = d.xmat[top].reshape(3, 3)
        yE = cam0[1] + spec.R                                    # camera's robot-left rim edge (plate frame ~ world)
        zrim = lambda di_: cam0[2] + spec.z_top(spec.R - di_ / 1000)   # noqa: E731
        P_pre = np.array([cam0[0], yE - p["di0"] / 1000, zrim(p["di0"]) + p["up"] / 1000])
        # the planned palm orientation: the searched pose placed at the camera's plate (truth only to build R)
        d_tmp = mujoco.MjData(m)
        d_tmp.qpos[:] = d.qpos
        mujoco.mj_forward(m, d_tmp)
        q_plan, info = place(m, d_tmp, meta, spec, top, hand, p["yaw"], p["pitch"], p["roll"], p["di0"], dz=p["up"])
        plan["R"] = d_tmp.xmat[palm].reshape(3, 3).copy()
        out = dict(par=p, pose_ik=info, events=[])
        # start: arm at its ready pose, hand shaped; joint move to the pre pose (belief IK)
        cmd["q"] = d.qpos[adr] + jerr
        P_pre = np.array([cam0[0], yE + p["out0"] / 1000, zrim(0.0) + p["up"] / 1000])      # v5: outside the camera's edge
        q_high = ik_to(P_pre + np.array([0, 0.03, 0.08]), q_plan + jerr)      # above and outside the rim first
        for _ in range(int(3.0 * 30)):
            tick(q_high, "pre")
        q_pre, _, _, _ = cart_move(P_pre + np.array([0, 0.03, 0.08]), P_pre, 40.0, "pre", q_high)
        P0_rel = rel0["v"]
        q = q_pre
        P = P_pre.copy()
        cam_start = camera()
        tries = 0
        result = "no push"
        slide_cam = 0.0
        while tries <= p["retries"]:
            # 2 touch
            hist = []
            pads = []

            def touched(s, P_):
                # touch as on the robot: a stall (encoders stop going down while the command is >= stall_lead mm ahead) confirmed by
                # a pad's proximity (< 2 mm) or force; a pad force alone also counts
                zb = idx_point(belief(q_meas()))[2]
                hist.append(zb)
                # only the index pad can feel this contact (no pad where the thumb / middle touch); a pad
                # force on any of the five pads, with +-0.5 N noise each, fired in free air -> index pad > 1 N, 3 ticks running
                pads.append(s[0])
                if len(pads) >= 3 and min(pads[-3:]) > 1.0:
                    return "index pad force"
                # the pads' proximity faces down-back and rarely reads in this pose -> a stall confirmed by
                # the joint-lag force up (> 3 N) or a pad (proximity < 3 mm)
                if len(hist) > 10 and hist[-10] - zb < 0.001 and zb - P_[2] > p["stall_lead"] / 1000 and (lag_force()[2] > 4.0 or s[1] < 3.0):
                    return "stall"
                return None
            # step search: start `out0` mm outside the camera's edge, lower to `below` mm under the rim top; nothing -> up,
            # `step` mm inward, again
            thb = m.body(f"{SIDE}_thumb_touch_link").id

            def inner_y(bb):
                Rt_ = bb.xmat[thb].reshape(3, 3)
                return min(idx_point(bb)[1], (bb.xpos[thb] + Rt_ @ PAD_POINT)[1])
            hit = None
            y_try = yE + p["out0"] / 1000
            for k_try in range(int(p["steps"])):
                hist.clear()
                pads.clear()
                P_hi = np.array([P[0], y_try, zrim(0.0) + 0.010])
                q, P, _, s = cart_move(P, P_hi, 25.0, "search", q)
                q, P, hit, s = cart_move(P, np.array([P[0], y_try, zrim(0.0) - (p["below"] + p["stall_lead"]) / 1000]), 15.0, "touch", q, stop=touched)
                if not hit:
                    y_try -= p["step"] / 1000
                    continue
                z_touch = idx_point(belief(q_meas()))[2]                 # where the encoders say the index is
                P = np.array([P[0], P[1], z_touch])
                # 3 edge: press `edge_lead` down and let it settle (the soft fingers compress ~4 mm
                # after the touch); the outward slide's drop is measured from this settled height
                q, P, _, s = cart_move(P, P - np.array([0, 0, p["edge_lead"] / 1000]), 15.0, "edge", q)
                for _ in range(15):
                    s = tick(q, "edge")
                z_set = idx_point(belief(q_meas()))[2]
                # if the press sinks the index > `press_sink` mm below the touch, it is off the rim: up, `step` mm inward, again
                if z_touch - z_set > p["press_sink"] / 1000:
                    out["events"].append(dict(t=round(d.time, 2), ev="touch off the rim", sink_mm=round((z_touch - z_set) * 1000, 1)))
                    hit = None
                    q, P, _, s = cart_move(P, np.array([P[0], P[1], zrim(0.0) + 0.010]), 20.0, "search", q)
                    y_try -= p["step"] / 1000
                    continue
                break
            if not hit:
                result = "no rim found under the index"
                break
            out["events"].append(dict(t=round(d.time, 2), ev="touch", z_vs_camera_mm=round((z_touch - zrim(p["di0"])) * 1000, 1), idx_N=round(s[0], 1)))
            # edge: slide outward pressing until the whole hand drops > edge_drop mm; the innermost fingertip has then just
            # passed the edge

            def off_edge(s, P_):
                zb = idx_point(belief(q_meas()))[2]
                if zb < z_set - p["edge_drop"] / 1000:
                    return "drop"
                return None
            # di0 27 mm: the camera may be 15 mm off sideways either way; the slide covers 70 mm
            wrot["v"] = p["w_rot_edge"]
            q, P, hit, s = cart_move(P, P + np.array([0, 0.070, 0]), 8.0, "edge", q, stop=off_edge)
            wrot["v"] = p["w_rot"]
            if not hit:
                result = "edge not found"
                break
            bb_ = belief(q_meas())
            y_edge = inner_y(bb_) - p["edge_bias"] / 1000      # v5: the drop comes ~7 mm past the edge (tip radius + lag), laptop runs rim15
            q, P, _, s = cart_move(P, P + np.array([0, 0, 0.010]), 20.0, "edge", q)          # off the rim: up again
            out["events"].append(dict(t=round(d.time, 2), ev="edge", y_vs_camera_mm=round((y_edge - yE) * 1000, 1)))
            # the camera is <= ~18 mm off on the robot: an edge more than `edge_tol` mm from the camera's is not believed: use the camera's
            if abs(y_edge - yE) * 1000 > p["edge_tol"]:
                out["events"].append(dict(t=round(d.time, 2), ev="edge implausible, camera used"))
                y_edge = yE
            # set_err mm (+ = outward) offsets the hand on purpose to measure the catch window; retry_out mm outward per slip
            y_edge += (p["set_err"] + tries * p["retry_out"]) / 1000
            # 4 set: back in to di inside the found edge, at the touched height (lift 3 mm while moving in)
            q, P, _, s = cart_move(P, np.array([P[0], P[1], z_touch + 0.008]), 15.0, "set", q)
            q, P, _, s = cart_move(P, np.array([P[0], y_edge - p["di"] / 1000, z_touch + 0.008]), 15.0, "set", q)
            hist.clear()
            pads.clear()
            q, P, _, s = cart_move(P, P - np.array([0, 0, 0.030]), 10.0, "set", q, stop=touched)
            Rt_ = d.xmat[top].reshape(3, 3)                    # truth, for the log only: the index pad's inset from the true edge
            out["events"].append(dict(t=round(d.time, 2), ev="set", inset_true_mm=round((spec.R - float(np.hypot(*(Rt_.T @ (idx_point(d) - d.xpos[top]))[:2]))) * 1000, 1)))
            # 5 push: press (lead below) and move toward the centre with a lead ahead
            P_press = P + np.array([0, -p["lead_y"] / 1000, -p["lead_z"] / 1000])
            q, P, _, s = cart_move(P, P_press, 15.0, "press", q)
            # force-controlled press on the index pad: deeper until it reads f_press N, then held between f_lo and f_hi
            fz = dict(dz=0.0, hist=[])

            def press_adj(s_):
                if p["f_press"] <= 0:
                    return
                fz["hist"].append(s_[0])
                f_ = sum(fz["hist"][-5:]) / len(fz["hist"][-5:])
                if f_ < p["f_lo"]:
                    fz["dz"] = min(fz["dz"] + p["dz_rate"] / 1000, p["dz_max"] / 1000)
                elif f_ > p["f_hi"]:
                    fz["dz"] = max(fz["dz"] - p["dz_rate"] / 1000, -p["lead_z"] / 1000)
            if p["f_press"] > 0:
                for _ in range(90):
                    q = ik_to(P_press - np.array([0, 0, fz["dz"]]), q)
                    s = tick(q, "press")
                    fz["hist"].append(s[0])
                    if sum(fz["hist"][-3:]) / len(fz["hist"][-3:]) >= p["f_press"]:
                        break
                    fz["dz"] = min(fz["dz"] + p["dz_rate"] / 1000, p["dz_max"] / 1000)
                out["events"].append(dict(t=round(d.time, 2), ev="pressed", extra_mm=round(fz["dz"] * 1000, 1), idx_N=round(s[0], 1)))
            cs = camera()
            y_plate0 = cs[1]
            dist = 0.0
            done = None
            # ftrace: per-part forces on the top plate [along the push (-y), toward the robot (-x), down] N, contact points vs
            # the plate centre, pad readings, plate and index pad paths since the engage; 10 Hz
            ft = dict(k=0, p0=d.xpos[top].copy(), i0=idx_point(d).copy(),
                      yaw0=math.degrees(math.atan2(d.xmat[top].reshape(3, 3)[1, 0], d.xmat[top].reshape(3, 3)[0, 0])))

            def ftrace_rec(ph_):
                ft["k"] += 1
                if not p["ftrace"] or ft["k"] % 3 or len(rec["trace"]) >= 5000:
                    return
                Fp, pads_ = finger_forces()
                pt_ = d.xpos[top]
                # where the plate below carries the top plate (centre of pressure, vs the plate centre, mm) and the top plate's
                # yaw (deg): does a finger pressing off the push line pin that side so the plate turns about it?
                cop, wsum = np.zeros(3), 0.0
                for i in range(d.ncon):
                    c = d.contact[i]
                    if n > 1 and {m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]} == {top, pb[-2]}:
                        mujoco.mj_contactForce(m, d, i, w6)
                        cop += abs(w6[0]) * c.pos
                        wsum += abs(w6[0])
                Rt_ = d.xmat[top].reshape(3, 3)
                rec["trace"].append(dict(t=round(d.time, 2), ph=ph_ + "_f", dist=round(dist * 1000, 1),
                                         yaw=round(math.degrees(math.atan2(Rt_[1, 0], Rt_[0, 0]) - ft["yaw0"]), 2),
                                         sup=[round(float(v), 1) for v in ((cop / max(wsum, 1e-9) - pt_) * 1000)[:2]] + [round(wsum, 1)],
                                         plate=[round(float(v), 1) for v in (pt_ - ft["p0"]) * 1000],
                                         index=[round(float(v), 1) for v in (idx_point(d) - ft["i0"]) * 1000],
                                         F={k: [round(float(-v[0][1]), 2), round(float(-v[0][0]), 2), round(float(-v[0][2]), 2)] for k, v in Fp.items()},
                                         at={k: [round(float(x_), 1) for x_ in (v[1] - pt_) * 1000] for k, v in Fp.items()},
                                         pads={k: round(float(v), 2) for k, v in pads_.items()}))
            # engage: in at 15 mm/s until the middle meets the edge (joint-lag force > f_engage N, or the camera sees the plate
            # move > 2 mm), at most gap_max mm; the slip check counts from there
            while dist < p["gap_max"] / 1000:
                dist += 15.0 / 1000 / 30
                q = ik_to(P_press + np.array([0, -dist, -fz["dz"]]), q)
                s = tick(q, "engage")
                ftrace_rec("engage")
                press_adj(s)
                if lag_force()[1] > p["f_engage"] or (y_plate0 - camera()[1]) > 0.002:
                    break
            out["events"].append(dict(t=round(d.time, 2), ev="engaged" if dist < p["gap_max"] / 1000 else "no edge contact", travel_mm=round(dist * 1000, 1)))
            y_hand0, y_plate0 = idx_point(belief(q_meas()))[1], camera()[1]
            mid_b = m.body(f"{SIDE}_middle_touch_link").id
            z_push0 = float(d.xpos[top][2])
            while dist < 0.140:
                dist += p["v"] / 1000 / 30
                q = ik_to(P_press + np.array([0, -dist, -fz["dz"]]), q)
                s = tick(q, "push")
                ftrace_rec("push")
                press_adj(s)
                if len(rec["trace"]) < 5000 and int(round(dist * 30000 / p["v"])) % 5 == 0:
                    # why the hooked middle sinks onto the plate below during the push - heights (truth, mm, vs the
                    # top plate's centre at the start of the push) of the index pad, the middle's tip, the top plate; its tilt
                    Rm_ = d.xmat[mid_b].reshape(3, 3)
                    Rtp = d.xmat[top].reshape(3, 3)
                    ct_ = contacts()
                    rec["trace"].append(dict(t=round(d.time, 2), ph="push_h", dist=round(dist * 1000, 1), idx_z=round((idx_point(d)[2] - z_push0) * 1000, 1),
                                             mid_z=round((d.xpos[mid_b] + Rm_ @ PAD_POINT)[2] * 1000 - z_push0 * 1000, 1),
                                             top_z=round((d.xpos[top][2] - z_push0) * 1000, 1), tilt=round(math.degrees(math.acos(min(1.0, Rtp[2, 2]))), 2),
                                             slide=round((y_plate0 - camera()[1]) * 1000, 1), pad=round(s[0], 2),
                                             lower={k: round(v, 1) for k, v in ct_.items() if k.endswith("-lower")},
                                             top_c={k: round(v, 1) for k, v in ct_.items() if k.endswith("-top")}))
                cs = camera()
                plate_mv = y_plate0 - cs[1]
                hand_mv = y_hand0 - idx_point(belief(q_meas()))[1]
                if (cam_start[1] - cs[1]) * 1000 >= p["stop"]:
                    done = "pushed"
                    break
                if (hand_mv - plate_mv) * 1000 > p["slip"]:
                    done = "slip"
                    break
            slide_cam = (cam_start[1] - camera()[1]) * 1000
            out["events"].append(dict(t=round(d.time, 2), ev=done or "push travel limit", camera_slide_mm=round(slide_cam, 1)))
            P = P_press + np.array([0, -dist, -fz["dz"]])
            if done == "pushed":
                result = "pushed"
                break
            # retry: lift and go back out to find the edge again
            tries += 1
            q, P, _, s = cart_move(P, P + np.array([0, 0.0, 0.020]), 20.0, "retry", q)
            result = done or "push travel limit"
    # 6 release
    q, P, _, s = cart_move(P, P + np.array([0, 0, 0.025 + lift_extra]), 20.0, "release", q)
    q, P, _, s = cart_move(P, P + (np.array([-0.025, 0, 0]) if p["mode"] == "near" else np.array([0, 0.025, 0])), 20.0, "release", q)
    # open the hand once clear (open_end), after rising another 60 mm, so the curled fingers do not sweep the plate
    if p["open_end"]:
        q, P, _, s = cart_move(P, P + np.array([0, 0, 0.060]), 30.0, "release", q)
        hand_q[:] = 0.0
    for _ in range(15 if not p["open_end"] else 30):
        tick(q, "release")
    if vid is not None:
        vid.release()
    rel = (d.xpos[top] - d.xpos[pb[-2]])[:2] - P0_rel if n > 1 else np.zeros(2)
    low_moved = max(float(np.linalg.norm(d.xpos[b][:2] - np.array([p["x"], p["y"]]))) for b in pb[:-1]) * 1000 if n > 1 else 0.0
    tilt = math.degrees(math.acos(min(1.0, d.xmat[top][8])))
    f = fm.summary()
    out["trace"] = rec["trace"]
    out.update(sol_seen=sol_seen, result=result, retries=tries, slide_over_below_mm=round(float(-rel[1]) * 1000, 1), slide_sideways_mm=round(float(abs(rel[0])) * 1000, 1),
               camera_slide_mm=round(slide_cam, 1), lower_moved_mm=round(low_moved, 1), tilt_end_deg=round(tilt, 1),
               peak_contact_N={k: round(v, 1) for k, v in sorted(rec["peak"].items())}, finger_peak50ms_N=f.get(f"{SIDE}_part_max50ms_N"),
               lower_contact_by_phase_N={ph: {k: round(v, 1) for k, v in sorted(dd.items())} for ph, dd in rec["peak_lower_phase"].items()})
    out["ok"] = bool(result == "pushed" and out["slide_over_below_mm"] >= 40 and low_moved < 3 and tilt < 9 and
                     rec["peak"].get("index-lower", 0) + rec["peak"].get("middle-lower", 0) + rec["peak"].get("thumb-lower", 0) < 1.0)
    if p["cmdlog"] and clog:
        import csv
        with open(p["cmdlog"], "w", newline="") as fc:
            w_ = csv.DictWriter(fc, fieldnames=list(clog[0]))
            w_.writeheader()
            w_.writerows(clog)
    if save:
        sp = dict(n=n, table=p["table"], x=p["x"], y=p["y"], pelvis=p["pelvis"], glaze_glaze=p["glaze_glaze"], foot_glaze=p["foot_glaze"],
                  pad_glaze=p["pad_glaze"], table_mu=p["table_mu"], push="rim")
        np.savez(save, qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, time=d.time, par=json.dumps(sp))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="", help="best.json of scripts/rim_pose_search.py (hand + wrist + di)")
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--save", default=None)
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    par = {}
    if a.pose:
        b = json.load(open(a.pose))["best"]["p"]
        par.update(tm=b["tm"], tp=b["tp"], ci=b["ci"], cm=b["cm"], yaw=b["yaw"], pitch=b["pitch"], roll=b["roll"], di=b["di"])
        if "crp" in b:
            par.update(cr=b["crp"], cp=b["crp"])
    par.update({k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None})
    t0 = time.time()
    print(json.dumps(run(par, video=a.video, title=a.title, save=a.save), indent=1, default=float))
    print(f"wall time {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
