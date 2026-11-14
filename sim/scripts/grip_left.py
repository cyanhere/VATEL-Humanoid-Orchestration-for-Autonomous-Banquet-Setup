"""Stage 4: the left hand takes its grip while the right hand holds the plate up.

    python scripts/grip_left.py --state data/results/robot/state_grip_v1best_n3.npz --video out.mp4
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from grip_slot import ArmIK  # noqa: E402
from platepick.forces import ForceMeter  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from reach_push import collisions  # noqa: E402

DEFAULTS = dict(slot_mr=0, arm_relax=0, relax_thumb=-1.0, relax=-1.0, psi=0.0, head=-45.0, din=2.0, sx=1.0, sy=-20.6, sz=100.0, pitch=0.0, flex=0.15, mflex=0.55, others=1.0,
                tm=0.3, tp_open=0.0, tm_close=1.2, tp_close=1.04, back=60.0, appr=30.0, push_in=3.0, approach_T=2.0, in_T=3.0,
                close_T=1.0, up2=20.0, pos_err_x=0.0, pos_err_y=0.0, pos_err_z=0.0, up_T=1.5, w_up=0.05, w_head=0.02, allow=2.0, seeds=40, nway=8,
                lift_ki=0.0, margin_thumb=-1.0, relax_r=0.0, din_alt=0.0, ki_l_scale=1.0, follow_kp=1.0, close_margin=0.0, app_ki=0.0, enc_fb=0, world_psi=0)
# encoder feedback for the soft (real-gain) arms:
# lift_ki  from "both hold" on, every 20 ms: target offset += lift_ki * (planned - measured)
# app_ki   the same for the left arm from the start of the approach
# enc_fb   after the two-hand lift, N rounds of target += (planned - measured)
FINGERS = ("index", "middle", "ring", "pinky", "thumb")


def run(state, par, video=None, title="", record=None):
    p = dict(DEFAULTS, **par)
    st = np.load(state, allow_pickle=True)
    pp = json.loads(str(st["par"]))
    gp = json.loads(str(st["grip"]))
    n = int(pp["n"])
    pick_mu = lambda k, v: gp.get(k, -1) if gp.get(k, -1) > 0 else v  # noqa: E731
    fr = Friction(glaze_glaze=pick_mu("mu_glaze", pp["glaze_glaze"]), foot_glaze=pick_mu("mu_foot", pp["foot_glaze"]),
                  pad_glaze=pick_mu("mu_pad", pp["pad_glaze"]), table=pick_mu("mu_table", pp["table_mu"]))
    sc = RobotScene(n=n, table=pp["table"], xy=(pp["x"], pp["y"]), pelvis=pp["pelvis"], fr=fr, table_right=pp.get("table_right", 0.0))
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    d.qpos[:], d.qvel[:], d.ctrl[:], d.time = st["qpos"], st["qvel"], st["ctrl"], float(st["time"])
    mujoco.mj_forward(m, d)
    pb = [m.body(f"plate{k}").id for k in range(n)]
    top = pb[-1]
    fm = ForceMeter(m, "left", top, pb[-2], hands=("left", "right"))
    fm.R_edge = sc.spec.R                 # thumb contact "in from the edge"
    act = {m.actuator(i).name: i for i in range(m.nu)}
    side = "left"
    arm_act = [act[f"a_{j}"] for j in meta["arm"][side]]
    hand_act = [act[f"a_{j}"] for j in meta["hand"][side]]
    r_arm_act = [act[f"a_{j}"] for j in meta["arm"]["right"]]
    C = np.array(meta["coupling"][side])
    hj = [m.joint(j).id for j in meta["hand"][side]]
    cmd_hi = np.array([min(m.jnt_range[hj[i]][1] / C[i, k] for i in range(C.shape[0]) if C[i, k] > 0) for k in range(6)])
    cmd_open = np.array([p["tm"], p["tp_open"], p["flex"] * cmd_hi[2], p["mflex"] * cmd_hi[3], p["others"] * cmd_hi[4], p["others"] * cmd_hi[5]])
    if p["slot_mr"]:                      # rim between MIDDLE (on top, with the index) and RING (under), as grip_slot.py --slot_mr 1
        cmd_open = np.array([p["tm"], p["tp_open"], p["flex"] * cmd_hi[2], p["flex"] * cmd_hi[3], p["mflex"] * cmd_hi[4], p["others"] * cmd_hi[5]])
    cmd_close = cmd_open.copy()
    cmd_close[0], cmd_close[1] = p["tm_close"], p["tp_close"]
    ik = ArmIK(m, meta, side, cmd_open)
    geoms = {s_: {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(s_ + "_")} for s_ in ("left", "right")}

    # ---- target on the left rim (plate frame), the rim's own surface there
    err = np.array([p["pos_err_x"], p["pos_err_y"], p["pos_err_z"]]) / 1000      # camera error on the plate position
    pt, Rt = d.xpos[top].copy() + err, d.xmat[top].reshape(3, 3).copy()
    if p["world_psi"]:
        # the plate is round and the camera gives no yaw, so plan with the plate's yaw removed
        gam = math.atan2(Rt[1, 0], Rt[0, 0])
        Rt = Rt @ np.array([[math.cos(gam), math.sin(gam), 0], [-math.sin(gam), math.cos(gam), 0], [0, 0, 1.0]])
    psi = math.radians(p["psi"])
    e = np.array([-math.sin(psi), math.cos(psi), 0.0])                 # outward at the grip, plate frame
    z = np.array([0, 0, 1.0])
    r_in = spec.R - p["din"] / 1000
    T = pt + Rt @ (np.array([0, 0, spec.z_under(r_in) + 0.5 * spec.rim_t]) + r_in * e)
    ra = spec.rim_ang - math.radians(p["pitch"])
    up_w = Rt @ (-math.sin(ra) * e + math.cos(ra) * z)
    fwd_w = Rt @ (-math.cos(ra) * e - math.sin(ra) * z)
    e_w = Rt @ e
    hd = math.radians(p["head"])
    fwd_w = math.cos(hd) * fwd_w - math.sin(hd) * np.cross(up_w, fwd_w)          # mirrored sense of "head"
    ap = math.radians(p["appr"])
    dir_in = math.cos(ap) * fwd_w - math.sin(ap) * np.cross(fwd_w, up_w)
    # the left palm is mirrored: its -y axis (index side) is the rim's "up"
    loc = np.array([p["sx"], p["sy"], p["sz"]]) / 1000
    base = d.qpos.copy()                  # the planner's world: the top plate where the camera says it is
    a_top = m.jnt_qposadr[m.joint(f"plate{n - 1}_free").id]
    base[a_top:a_top + 3] += err
    for i, j in enumerate(hj):
        base[m.jnt_qposadr[j]] = min(C[i] @ cmd_open, m.jnt_range[j][1])

    def solve(seeds):
        out = []
        for q0 in seeds:
            d2 = mujoco.MjData(m)
            d2.qpos[:] = base

            def res(q):
                ik.set(d2, q)
                mujoco.mj_kinematics(m, d2)
                P, R = ik.frame(d2, loc)
                return np.concatenate([P - T, p["w_up"] * (-R[:, 1] - up_w), p["w_head"] * (R[:, 2] - fwd_w)])
            r = least_squares(res, np.clip(q0, ik.lo + 1e-4, ik.hi - 1e-4), bounds=(ik.lo, ik.hi), xtol=1e-10, ftol=1e-12)
            ik.set(d2, r.x)
            mujoco.mj_kinematics(m, d2)
            P, R = ik.frame(d2, loc)
            out.append(dict(q=r.x.copy(), pos_mm=float(np.linalg.norm(P - T) * 1000),
                            up_deg=math.degrees(math.acos(np.clip(-R[:, 1] @ up_w, -1, 1))),
                            head_deg=math.degrees(math.acos(np.clip(R[:, 2] @ fwd_w, -1, 1)))))
        return sorted(out, key=lambda s_: s_["pos_mm"] + 0.2 * s_["up_deg"])

    rng = np.random.default_rng(0)
    seeds = [np.array(meta["ready"][side])] + [rng.uniform(ik.lo, ik.hi) for _ in range(int(p["seeds"]))]
    sols = solve(seeds)

    def clear(q, allow_top=0.5):
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, q)
        mujoco.mj_forward(m, d2)
        return {k: v for k, v in collisions(m, d2).items() if k.startswith("left_") and "right_" not in k
                and (not k.endswith(f"plate{n - 1}") or v < -allow_top)}

    def shift(q, s_, w_rot=0.2):
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, q)
        mujoco.mj_kinematics(m, d2)
        P0, R0 = d2.xpos[ik.palm].copy(), d2.xmat[ik.palm].reshape(3, 3).copy()

        def r(qq):
            ik.set(d2, qq)
            mujoco.mj_kinematics(m, d2)
            return np.concatenate([d2.xpos[ik.palm] - P0 - s_, w_rot * (d2.xmat[ik.palm].reshape(3, 3) - R0).ravel()])
        qq = least_squares(r, q, bounds=(ik.lo, ik.hi), xtol=1e-10).x
        return qq, float(np.linalg.norm(r(qq)[:3]) * 1000)

    q_from = np.array([d.ctrl[a_] for a_ in arm_act])
    pick, why = None, ""
    for s_ in sols[:12]:
        if s_["pos_mm"] > 2.0 or s_["up_deg"] > 25:
            continue
        q_in = s_["q"]
        sh = [shift(q_in, -(1 - u) * p["back"] / 1000 * dir_in) for u in np.linspace(0, 1, int(p["nway"]) + 1)[:-1]]
        way = [w[0] for w in sh] + [q_in]
        bad = clear(way[0], 0.0)
        for qw in way[1:-1]:
            bad.update(clear(qw, 1.0))
        bad.update(clear(q_in, 3.0))
        if bad and min(bad.values()) < -p["allow"]:
            k0 = min(bad, key=bad.get)
            why = why or f"{k0} {bad[k0]:.1f} mm"
            continue
        legs = None
        for off in (np.zeros(3), -0.06 * dir_in + 0.04 * z, 0.06 * z, -0.12 * dir_in + 0.06 * z, -0.06 * e_w + 0.08 * z):
            q_far = shift(way[0], off)[0] if off.any() else way[0]
            hit = any(clear(q_from + (q_far - q_from) * u, 0.0) for u in np.linspace(0.1, 1.0, 12)) or \
                (off.any() and any(clear(q_far + (way[0] - q_far) * u, 0.0) for u in np.linspace(0.1, 1.0, 8)))
            if not hit:
                legs = [way[0]] if not off.any() else [q_far, way[0]]
                break
        if legs is None:
            why = why or "path to the start of the slide touches the plate"
            continue
        pick = (s_, q_in, way, legs)
        break
    if pick is None and p["din_alt"] > 0:
        # the deeper grip (din 6) has no clear pose on some stacks (the pinky meets the bottom plate): fall
        # back to the shallower one
        r_alt = run(state, dict(par, din=p["din_alt"], din_alt=0.0), video=video, title=title, record=record)
        r_alt["din_fallback"] = dict(tried=p["din"], why=why)
        return r_alt
    if pick is None:
        return dict(par=p, error=f"no left grip pose: {why or 'not reachable'}", best_pos_mm=sols[0]["pos_mm"], best_up_deg=sols[0]["up_deg"])
    s_, q_in, way, legs = pick
    q_push = shift(q_in, p["push_in"] / 1000 * dir_in)[0]

    def forces():
        out, w = {}, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for hand in ("left", "right"):
                a_, b_ = c.geom1 in geoms[hand], c.geom2 in geoms[hand]
                if a_ == b_:
                    continue
                ob = m.geom_bodyid[c.geom1 if b_ else c.geom2]
                kind = "top" if ob == top else "below" if m.body(ob).name.startswith("plate") else "table" if ob == 0 else None
                if kind is None:
                    continue
                mujoco.mj_contactForce(m, d, i, w)
                part = m.body(m.geom_bodyid[c.geom2 if b_ else c.geom1]).name.replace(hand + "_", "").replace("_link", "")
                if kind == "top":
                    lp = d.xmat[top].reshape(3, 3).T @ (c.pos - d.xpos[top])
                    rr = min(math.hypot(lp[0], lp[1]), spec.R)
                    part += "^" if abs(lp[2] - spec.z_top(rr)) < abs(lp[2] - spec.z_under(rr)) else "v"
                out[(hand, kind, part)] = out.get((hand, kind, part), 0.0) + float(np.linalg.norm(w[:3]))
        return out

    vid = None
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))
        for cam, eye in (("grip", T + 0.2 * e_w - 0.05 * z + 0.12 * np.array([1.0, 0, 0])), ("gripside", T + 0.22 * np.cross(z, e_w) + 0.04 * z + 0.03 * e_w)):
            cid = m.camera(cam).id
            zc = (eye - T) / np.linalg.norm(eye - T)
            xc = np.cross(z, zc)
            xc /= np.linalg.norm(xc)
            q = np.zeros(4)
            mujoco.mju_mat2Quat(q, np.c_[xc, np.cross(zc, xc), zc].ravel())
            m.cam_pos[cid], m.cam_quat[cid] = eye, q
    vs = dict(next=d.time, phase="")

    def grab():
        import cv2
        if d.time < vs["next"]:
            return
        vs["next"] += 0.04
        ims = []
        for cam, lab in zip(("grip", "gripside", "front", "top"), ("close-up of the front-right hand (robot's left), from outside the rim", "close-up, along the rim", "from the front", "from above")):
            vr.update_scene(d, camera=cam)
            im = cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im, lab, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            ims.append(im)
        f = forces()
        g = lambda h, word: sum(v for k, v in f.items() if k[0] == h and k[1] == "top" and k[2].startswith(word))  # noqa: E731
        bar = np.full((64, 1280, 3), 20, np.uint8)
        cv2.putText(bar, title[:150], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        txt = (f"t {d.time:4.1f} s  {vs['phase']:<12s} FRONT-RIGHT hand (robot L): index {g('left', 'index'):4.1f} middle {g('left', 'middle'):4.1f} thumb {g('left', 'thumb'):4.1f} N   "
               f"FRONT-LEFT hand (robot R): index {g('right', 'index'):4.1f} middle {g('right', 'middle'):4.1f} thumb {g('right', 'thumb'):4.1f} N   tilt {math.degrees(math.acos(min(1.0, d.xmat[top][8]))):4.1f} deg")
        cv2.putText(bar, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    log = []
    l_adr = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][side]]
    r_adr = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"]["right"]]
    l_lo, l_hi = np.array([m.jnt_range[m.joint(j).id] for j in meta["arm"][side]]).T
    r_lo, r_hi = np.array([m.jnt_range[m.joint(j).id] for j in meta["arm"]["right"]]).T
    fb = dict(ki_l=p["app_ki"], ki_r=0.0, off_l=np.zeros(5), off_r=np.zeros(5), r_plan=np.array([d.ctrl[a_] for a_ in r_arm_act]))
    every = max(1, int(0.02 / dt))
    # --record: the scripted commands at 30 Hz and a snapshot at the start of each phase
    hinge = [j for j in range(m.njnt) if m.jnt_type[j] in (2, 3)]
    snap_state = lambda: dict(joints={m.joint(j).name: float(d.qpos[m.jnt_qposadr[j]]) for j in hinge},  # noqa: E731
                              plates=[[float(v) for v in d.qpos[m.jnt_qposadr[m.joint(f"plate{k}_free").id]:][:7]] for k in range(n)],
                              ctrl={m.actuator(i).name: float(d.ctrl[i]) for i in range(m.nu)}, time=float(d.time))
    rec = dict(dt=1 / 30, start=snap_state(), steps=[], snaps=[], next=d.time) if record else None

    def move(qa, qb, ca, cb, T_, phase, right=None):
        N = max(1, int(T_ / dt))
        vs["phase"] = phase
        for k in range(N):
            u = (k + 1) / N
            u = 3 * u * u - 2 * u ** 3
            ql_plan = qa + (qb - qa) * u
            if right is not None:
                fb["r_plan"] = right[0] + (right[1] - right[0]) * u
            if k % every == 0:
                fb["off_l"] += fb["ki_l"] * (ql_plan - d.qpos[l_adr])
                fb["off_r"] += fb["ki_r"] * (fb["r_plan"] - d.qpos[r_adr])
            d.ctrl[arm_act] = np.clip(ql_plan + fb["off_l"], l_lo, l_hi)
            c = ca + (cb - ca) * u
            if p["close_margin"] > 0 and phase not in ("approach", "slide in"):
                # force limit on the porcelain: each closing finger is commanded at most close_margin (rad)
                # past its measured angle (the Revo2 reports finger positions), so it presses with ~kp x margin
                for k_, jj in ((0, hj[0]), (1, hj[5]), (2, hj[1]), (3, hj[2])) + (((4, hj[3]),) if p["slot_mr"] else ()):
                    sg = np.sign(cmd_close[k_] - cmd_open[k_])
                    q_ = d.qpos[m.jnt_qposadr[jj]]
                    mg = p["margin_thumb"] if (k_ <= 1 and p["margin_thumb"] >= 0) else p["close_margin"]   # thumb's own margin
                    if sg and sg * (c[k_] - q_) > mg:
                        c[k_] = q_ + sg * mg
            for i, j in enumerate(hj):
                d.ctrl[hand_act[i]] = min(C[i] @ c, m.jnt_range[j][1])
            if right is not None or fb["ki_r"] > 0:
                d.ctrl[r_arm_act] = np.clip(fb["r_plan"] + fb["off_r"], r_lo, r_hi)
            if rec is not None:
                if k == 0 and (not rec["snaps"] or rec["snaps"][-1]["label"] != phase):
                    rec["snaps"].append(dict(snap_state(), label=phase, q_arm=[float(v) for v in d.ctrl[arm_act]], cmd=[float(v) for v in c]))
                if d.time >= rec["next"] - 1e-9:
                    rec["next"] += rec["dt"]
                    rec["steps"].append(dict(phase=phase, q_arm=[round(float(v), 5) for v in d.ctrl[arm_act]], cmd=[round(float(v), 5) for v in c],
                                             q_right=[round(float(v), 5) for v in d.ctrl[r_arm_act]]))
            mujoco.mj_step(m, d)
            fm.update(d)
            if vid is not None:
                grab()
            if k % 20 == 0:
                log.append(dict(phase=phase, f=forces(), tilt=math.degrees(math.acos(min(1.0, d.xmat[top][8])))))

    P0 = d.xpos[top].copy()
    cmd_now = np.array([d.ctrl[a_] for a_ in hand_act])
    cmd0 = np.zeros(6)
    qa = q_from
    for k, ql in enumerate(legs):
        move(qa, ql, cmd0 if k == 0 else cmd_open, cmd_open, p["approach_T"] / len(legs), "approach")
        qa = ql
    for qa_, qb_ in zip(way[:-1], way[1:]):
        move(qa_, qb_, cmd_open, cmd_open, p["in_T"] / (len(way) - 1), "slide in")
    move(q_in, q_push, cmd_open, cmd_open, 0.5, "slide in")
    # where the hand point really is at the end of the slide-in vs the plan, in the rim's frame
    # (up = normal to the rim plane, out = radially outward, along = along the rim); and how far the plate has moved
    P_now, _ = ik.frame(d, loc)
    side_w = np.cross(up_w, e_w)
    slide_err = dict(planned_vs_hand={k_: float((P_now - T) @ v_) * 1000 for k_, v_ in (("up", up_w), ("out", e_w), ("along", side_w))},
                     plate_moved={k_: float((d.xpos[top] - (pt - err)) @ v_) * 1000 for k_, v_ in (("up", up_w), ("out", e_w), ("along", side_w))})
    move(q_push, q_push, cmd_open, cmd_close, p["close_T"], "thumb locks")
    if p["relax"] >= 0:               # as grip_slot.py: index / middle hold where the rim stopped them
        cmd_hold = cmd_close.copy()
        for k_, jj in ((2, hj[1]), (3, hj[2])) + (((4, hj[3]),) if p["slot_mr"] else ()) + (((0, hj[0]), (1, hj[5])) if p["relax_thumb"] >= 0 else ()):
            q_ = d.qpos[m.jnt_qposadr[jj]]
            cmd_hold[k_] = q_ + (p["relax_thumb"] if k_ <= 1 else p["relax"]) * np.sign(cmd_close[k_] - q_)
        move(q_push, q_push, cmd_close, cmd_hold, 0.3, "thumb locks")
        cmd_close = cmd_hold
    rel_lock = d.xmat[top].reshape(3, 3).T @ (d.xpos[m.body("left_base_link").id] - d.xpos[top])
    if p["lift_ki"] > 0:
        fb["ki_l"], fb["ki_r"] = p["lift_ki"] * p["ki_l_scale"], p["lift_ki"]
    if p["follow_kp"] != 1.0:
        # with soft arms both arms fight through the rigid plate: from here the robot-left arm follows (gains x follow_kp)
        # and the robot-right arm carries and lifts
        m.actuator_gainprm[arm_act, 0] *= p["follow_kp"]
        m.actuator_biasprm[arm_act, 1] *= p["follow_kp"]
        m.actuator_biasprm[arm_act, 2] *= p["follow_kp"]
    if p["relax_r"] > 0:
        # once both hands hold, the carrying hand's index, middle and thumb relax to measured angle + relax_r,
        # to keep the finger forces under the 30 N limit
        for jn in meta["hand"]["right"]:
            if any(f"_{f}_" in jn for f in ("index", "middle", "thumb")):
                a_ = act[f"a_{jn}"]
                q_ = d.qpos[m.jnt_qposadr[m.joint(jn).id]]
                d.ctrl[a_] = q_ + p["relax_r"] * np.sign(d.ctrl[a_] - q_)
    move(q_push, q_push, cmd_close, cmd_close, 0.5, "both hold")
    if p["arm_relax"]:
        # two position-controlled arms holding one rigid plate fight over millimetres of target error: once both hold,
        # each arm's target becomes where it actually is (the robot reports its joint angles), then they lift from there
        ql_now = np.array([d.qpos[m.jnt_qposadr[m.joint(j).id]] for j in meta["arm"][side]])
        qr_now = np.array([d.qpos[m.jnt_qposadr[m.joint(j).id]] for j in meta["arm"]["right"]])
        move(q_push, ql_now, cmd_close, cmd_close, 0.3, "both hold", right=(np.array([d.ctrl[a_] for a_ in r_arm_act]), qr_now))
        move(ql_now, ql_now, cmd_close, cmd_close, 0.3, "both hold")
        q_push = ql_now
    f_both = forces()
    force_grip = fm.summary()
    P_b, R_b = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
    palms = {h: m.body(f"{h}_base_link").id for h in ("left", "right")}
    rel0 = {h: R_b.T @ (d.xpos[b] - P_b) for h, b in palms.items()}
    # slip at the fingers (the palm origin is ~10 cm from the rim, so a small turn of the hand about the
    # pinch reads as palm "slip"), and the turn of each hand relative to the plate
    tips = {k_: m.body(f"left_{k_}_link").id for k_ in ("thumb_touch", "middle_proximal", "middle_distal")}
    tip0 = {k_: R_b.T @ (d.xpos[b] - P_b) for k_, b in tips.items()}
    rot0 = {h: R_b.T @ d.xmat[b].reshape(3, 3) for h, b in palms.items()}
    # both hands lift together: each arm moves its palm straight up by up2
    qr0 = np.array([d.ctrl[a_] for a_ in r_arm_act])
    ikr = ArmIK(m, meta, "right", np.zeros(6))
    d2 = mujoco.MjData(m)
    d2.qpos[:] = d.qpos
    mujoco.mj_kinematics(m, d2)
    Pr0, Rr0 = d2.xpos[ikr.palm].copy(), d2.xmat[ikr.palm].reshape(3, 3).copy()

    def res_r(qq):
        for j, v in zip(ikr.arm, qq):
            d2.qpos[m.jnt_qposadr[j]] = v
        mujoco.mj_kinematics(m, d2)
        return np.concatenate([d2.xpos[ikr.palm] - Pr0 - p["up2"] / 1000 * z, 0.05 * (d2.xmat[ikr.palm].reshape(3, 3) - Rr0).ravel()])
    qr0 = fb["r_plan"].copy()                       # planned (not offset) right target
    qr1 = least_squares(res_r, qr0, bounds=(ikr.lo, ikr.hi), xtol=1e-10).x
    ql1 = shift(q_push, p["up2"] / 1000 * z, w_rot=0.05)[0]
    move(q_push, ql1, cmd_close, cmd_close, p["up_T"], "both lift", right=(qr0, qr1))
    sag = {"left": np.abs(ql1 - d.qpos[l_adr]).max(), "right": np.abs(qr1 - d.qpos[r_adr]).max()}
    slip_lift = float(np.linalg.norm(d.xmat[top].reshape(3, 3).T @ (d.xpos[palms["left"]] - d.xpos[top]) - rel0["left"])) * 1000
    for it in range(int(p["enc_fb"])):
        fb["off_l"] += (ql1 - d.qpos[l_adr]) * min(1.0, p["ki_l_scale"])
        fb["off_r"] += qr1 - d.qpos[r_adr]
        move(ql1, ql1, cmd_close, cmd_close, 0.3, "both hold", right=(qr1, qr1))
    move(ql1, ql1, cmd_close, cmd_close, 1.0, "both hold", right=(qr1, qr1))
    if vid is not None:
        vid.release()
    if rec is not None:
        rec.pop("next")
        rec.update(par=json.loads(str(st["par"])), left_par=p, arm_joints=list(meta["arm"][side]), right_arm_joints=list(meta["arm"]["right"]),
                   hand_joints=list(meta["hand"][side]), source=os.path.basename(state), lift_right=dict(q0=[float(v) for v in qr0], q1=[float(v) for v in qr1]))
        json.dump(rec, open(record, "w"))
    Pe, Re = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
    f_end = forces()
    slip = {h: float(np.linalg.norm(Re.T @ (d.xpos[b] - Pe) - rel0[h])) * 1000 for h, b in palms.items()}
    slip_tip = {k_: float(np.linalg.norm(Re.T @ (d.xpos[b] - Pe) - tip0[k_])) * 1000 for k_, b in tips.items()}
    turn = {h: math.degrees(math.acos(np.clip((np.trace(rot0[h].T @ Re.T @ d.xmat[b].reshape(3, 3)) - 1) / 2, -1, 1))) for h, b in palms.items()}
    peak_pre = max([r["tilt"] for r in log if r["phase"] in ("approach", "slide in")] + [0.0])
    left_top = {k[2]: round(v, 1) for k, v in f_end.items() if k[0] == "left" and k[1] == "top"}
    ok = sum(v for k, v in left_top.items() if k.startswith("index") and k.endswith("^")) > 0.3 and \
        sum(v for k, v in left_top.items() if k.startswith("middle") and k.endswith("v")) > 0.3
    # the "pinch" option for the second hand: thumb on top + middle under, index not pressing
    pinch = sum(v for k, v in left_top.items() if k.startswith("thumb") and k.endswith("^")) > 0.3 and         sum(v for k, v in left_top.items() if k.startswith("middle") and k.endswith("v")) > 0.3 and         sum(v for k, v in left_top.items() if k.startswith("index")) < 5.0
    return dict(par=p, left_grip_ok=bool(ok), left_pinch_ok=bool(pinch), left_thumb_on_top=bool(sum(v for k, v in left_top.items() if k.startswith("thumb") and k.endswith("^")) > 0.3),
                left_end_N=left_top, right_end_N={k[2]: round(v, 1) for k, v in f_end.items() if k[0] == "right" and k[1] == "top"},
                both_hold_N={" ".join(k): round(v, 1) for k, v in f_both.items() if k[1] == "top"},
                plate_rose_mm=float(Pe[2] - P0[2]) * 1000, tilt_deg=math.degrees(math.acos(min(1.0, Re[2, 2]))), slip_mm=slip,
                slip_left_hold_mm=float(np.linalg.norm(rel0["left"] - rel_lock)) * 1000, slip_left_lift_mm=slip_lift,
                lift_err_deg={k_: math.degrees(v) for k_, v in sag.items()}, slip_left_tip_mm=slip_tip, hand_turn_deg=turn, slide_in_err_mm=slide_err,
                max_tilt_during_left_approach_deg=peak_pre, left_on_lower_or_table_N=max([v for r in log for k, v in r["f"].items() if k[0] == "left" and k[1] != "top"] + [0.0]),
                force_to_grip=force_grip, force=fm.summary(), pos_mm=s_["pos_mm"], up_deg=s_["up_deg"], head_deg=s_["head_deg"], arm_q=[round(float(v), 3) for v in q_in], _final=dict(qpos=d.qpos.copy(), qvel=d.qvel.copy(), ctrl=d.ctrl.copy(), time=d.time))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--save", default=None)
    ap.add_argument("--record", default=None, help="write the scripted commands (30 Hz) and phase snapshots to this .json")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    over = {k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None}
    t0 = time.time()
    r = run(a.state, over, video=a.video, title=a.title, record=a.record)
    fin = r.pop("_final", None)
    if a.save and fin:
        st = np.load(a.state, allow_pickle=True)
        np.savez(a.save, **fin, par=str(st["par"]), grip=str(st["grip"]), left=json.dumps(r["par"]))
    print(json.dumps(r, indent=1, default=float))
    print(f"wall time {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
