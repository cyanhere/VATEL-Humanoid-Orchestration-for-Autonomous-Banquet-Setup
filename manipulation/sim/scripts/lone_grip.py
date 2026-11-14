"""Two-hand grip of a lone plate, scored on whether the plate stays in the hands, with an optional parameter search.

    python scripts/lone_grip.py --family slot [--set k=v,...] [--cond pad,table,ex,ey] [--video out.mp4]
    python scripts/lone_grip.py --search OUTDIR --family pad --gens 25 --pop 40 --workers 8
    python scripts/lone_grip.py --best OUTDIR/best.json --all (every condition)
"""
import argparse
import json
import math
import os
import sys
import time
import traceback

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault("DEXPICK_KP_ARM", "50,50,40,40,30")
os.environ.setdefault("DEXPICK_KV_ARM", "2")
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402
from scipy.optimize import least_squares  # noqa: E402

from platepick.forces import ForceMeter  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402

SIDES = ("left", "right")
# gene: (low, high)
# v2: wider palm roll / dip, inset up to 30 mm, and a second closing step after the squeeze (tr2, tf2, fingers)
GENES = dict(psi=(-25, 55), h=(0, 90), pitch=(-25, 30), roll=(-80, 60), px=(-5, 35), py=(-45, 45), pz=(40, 160), hz=(2, 35),
             din=(-6, 30), sq=(0, 40), lift_in=(0, 15), tr=(0, 1), tf=(0, 1), ci=(0, 1.4), cm=(0, 1.4), cr=(0, 1.4), cp=(0, 1.4),
             cpre=(0, 0.5), tr2=(0, 1), tf2=(0, 1), ci2=(0, 1.4), cm2=(0, 1.4), cr2=(0, 1.4), cp2=(0, 1.4))
NAMES = list(GENES)
LO = np.array([GENES[k][0] for k in NAMES], float)
HI = np.array([GENES[k][1] for k in NAMES], float)
# starting points of the search, one per grip family
FAMILIES = dict(
    slot=dict(psi=30, h=90, pitch=12.7, roll=0, px=1, py=-20, pz=105, hz=19, din=2, sq=30, lift_in=0, tr=1, tf=0, ci=0.2, cm=0.2,
              cr=0.2, cp=1.0, cpre=0.0),                 # rim between ring and little finger
    pad=dict(psi=15, h=15, pitch=0, roll=0, px=15, py=-32, pz=120, hz=19, din=0, sq=30, lift_in=5, tr=0.5, tf=0, ci=0.3, cm=0.5,
             cr=0.8, cp=0.8, cpre=0.1),                 # palm in, little finger and palm against the rim edge, fingers wrap
    thumb=dict(psi=20, h=45, pitch=10, roll=-20, px=40, py=20, pz=95, hz=19, din=0, sq=25, lift_in=5, tr=1, tf=0.6, ci=0.7, cm=0.7,
               cr=0.3, cp=0.3, cpre=0.1),               # thumb onto the index / middle with the rim between
    wrap=dict(psi=15, h=29.4, pitch=-11, roll=-2, px=15, py=-32, pz=120, hz=19, din=0, sq=30, lift_in=5, tr=0.5, tf=0, ci=0.3, cm=0.5,
              cr=0.8, cp=0.8, cpre=0.1),                # first lift: palm in, fingers tipped up 11 deg, ring + little wrap the rim
    clamp=dict(psi=-8, h=29.4, pitch=-11, roll=-2, px=15, py=-32, pz=120, hz=19, din=0, sq=20, lift_in=5, tr=0.5, tf=0, ci=0.2, cm=0.2,
               cr=0.2, cp=0.2, cpre=0.1),               # hand-found: fingers nearly straight, pads + palm squeeze the rim edge
    slot2=dict(psi=10.23, h=26.29, pitch=-6.34, roll=-36.2, px=14.68, py=-40.0, pz=134.95, hz=12.67, din=3.49, sq=13.03, lift_in=6.68, tr=0.23, tf=0.07, ci=0.33, cm=0.08, cr=0.08, cp=0.0, cpre=0.0),                 # search clamp_s0 gen 4 (6/6 v1): slanted hand, little finger under the lip, ring on top
    shelf=dict(psi=20, h=30, pitch=10, roll=40, px=12, py=-32, pz=110, hz=9, din=4, sq=20, lift_in=5, tr=0, tf=0, ci=0.3, cm=0.4,
               cr=0.6, cp=0.6, cpre=0.0),               # palm turned up, little-finger edge under the lip
)
FIXED = dict(table=0.755, pelvis=0.71, x=0.33, lift=40.0, pre_up=25.0, T=1.2, hold=0.6, hold2=2.0)
# robustness conditions: (fingertip/pad friction, foot-table friction, plate farther + / nearer - (mm), plate sideways (mm))
CONDS_V1 = [(0.25, 0.45, 0, 0), (0.25, 0.45, -10, 0), (0.25, 0.45, 10, 0), (0.25, 0.45, 0, 10), (0.25, 0.3, 0, 0), (0.5, 0.45, 0, 0)]
# v2: camera placement verified to 1-2 mm, so only friction is varied
CONDS = [(pm, tm, 0, 0) for pm in (0.25, 0.5, 0.8) for tm in (0.3, 0.6)]
DIST = [((2, 0, 0), (0, 0, 0)), ((-2, 0, 0), (0, 0, 0)), ((0, 2, 0), (0, 0, 0)), ((0, -2, 0), (0, 0, 0)), ((0, 0, -2.5), (0, 0, 0)),
        ((0, 0, 0), (0.15, 0, 0)), ((0, 0, 0), (-0.15, 0, 0)), ((0, 0, 0), (0, 0.15, 0)), ((0, 0, 0), (0, -0.15, 0))]


def unit(v):
    return v / np.linalg.norm(v)


def frames(p, s, spec, pt, Rt):
    """World target point, finger axis and palm normal for side s (belief: plate at pt, Rt)."""
    z = np.array([0, 0, 1.0])
    psi = math.radians(p["psi"])
    sg = -1.0 if s == "right" else 1.0
    e = np.array([-math.sin(psi), sg * math.cos(psi), 0.0])
    t = np.array([math.cos(psi), sg * math.sin(psi), 0.0])
    h, pi_, ro = (math.radians(p[k]) for k in ("h", "pitch", "roll"))
    fh = math.cos(h) * t - math.sin(h) * e
    f = unit(math.cos(pi_) * fh - math.sin(pi_) * z)
    hp = -math.sin(h) * t - math.cos(h) * e                  # horizontal, normal to f, facing the plate
    v = unit(np.cross(f, hp))
    v = v if v[2] >= 0 else -v
    n = unit(math.cos(ro) * hp + math.sin(ro) * v)
    return e, f, n


def ik(m, meta, d, s, loc, T, f, n, q_seed, rng, nseed):
    arm = [m.joint(j).id for j in meta["arm"][s]]
    adr = [m.jnt_qposadr[j] for j in arm]
    lo = np.array([m.jnt_range[j][0] for j in arm])
    hi = np.array([m.jnt_range[j][1] for j in arm])
    palm = m.body(f"{s}_base_link").id
    d2 = mujoco.MjData(m)
    d2.qpos[:] = d.qpos

    def res(q):
        d2.qpos[adr] = q
        mujoco.mj_kinematics(m, d2)
        R = d2.xmat[palm].reshape(3, 3)
        return np.concatenate([d2.xpos[palm] + R @ loc - T, 0.04 * (R[:, 2] - f), 0.04 * (R[:, 0] - n)])
    best = None
    seeds = [q_seed] + [np.array(meta["ready"][s])] + [rng.uniform(lo, hi) for _ in range(nseed)]
    for q0 in seeds:
        r = least_squares(res, np.clip(q0, lo + 1e-4, hi - 1e-4), bounds=(lo, hi), xtol=1e-9)
        rr = res(r.x)
        pos = float(np.linalg.norm(rr[:3]) * 1000)
        ang = float(np.degrees(np.linalg.norm(rr[3:]) / 0.04 / 1.0))   # rough: chord -> deg (small angles)
        marg = math.degrees(float(np.min(np.minimum(r.x - lo, hi - r.x))))
        cost = pos + 0.3 * ang - 0.1 * min(marg, 10)
        if best is None or cost < best["cost"]:
            best = dict(cost=cost, q=r.x.copy(), pos_mm=round(pos, 1), ang_deg=round(ang, 1), margin_deg=round(marg, 1))
        if pos < 1.0 and ang < 3.0 and marg > 5:
            break
    return best


def run(p, video=None, title="", disturb=False):
    p = dict(FIXED, **p)
    fr = Friction(pad_glaze=p.get("pad_mu", 0.25), table=p.get("table_mu", 0.45))
    ex, ey = p.get("ex", 0.0) / 1000, p.get("ey", 0.0) / 1000
    sc = RobotScene(n=1, table=p["table"], xy=(p["x"] + ex, ey), pelvis=p["pelvis"], fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    spec, dt = sc.spec, m.opt.timestep
    top = m.body("plate0").id
    pt_true = d.xpos[top].copy()
    pt = pt_true - np.array([ex, ey, 0.0])                    # where the robot believes the plate is
    Rt = d.xmat[top].reshape(3, 3).copy()
    z = np.array([0, 0, 1.0])
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = {s: [act[f"a_{j}"] for j in meta["arm"][s]] for s in SIDES}
    adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]] for s in SIDES}
    HA = {s: [act[f"a_{j}"] for j in meta["hand"][s]] for s in SIDES}
    C = {s: np.array(meta["coupling"][s]) for s in SIDES}
    hhi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"][s]]) for s in SIDES}
    palm = {s: m.body(f"{s}_base_link").id for s in SIDES}
    hand_geoms = {s: {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(s + "_")} for s in SIDES}
    plate_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == top}
    fm = ForceMeter(m, "left", top, None, hands=SIDES)
    fm.R_edge = spec.R
    th = np.linspace(0, 2 * math.pi, 72, endpoint=False)
    ring_loc = np.c_[spec.foot_out * np.cos(th), spec.foot_out * np.sin(th), np.zeros_like(th)]
    foot0 = ((Rt @ ring_loc.T)[2] + pt_true[2]).min()
    rng = np.random.default_rng(int(p.get("seed", 0)))

    def contact_force(test):
        F, w = 0.0, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if test(c.geom1, c.geom2) or test(c.geom2, c.geom1):
                mujoco.mj_contactForce(m, d, i, w)
                F += float(np.linalg.norm(w[:3]))
        return F

    hand_table = lambda: contact_force(lambda a, b: m.geom_bodyid[a] == 0 and (b in hand_geoms["left"] or b in hand_geoms["right"]))  # noqa: E731

    def plate_force(s):
        return contact_force(lambda a, b: a in plate_geoms and b in hand_geoms[s])

    def plate_state():
        fz = (d.xmat[top].reshape(3, 3) @ ring_loc.T)[2] + d.xpos[top][2]
        return (fz.min() - foot0) * 1000, math.degrees(math.acos(min(1.0, d.xmat[top][8]))), float(np.linalg.norm(d.xpos[top][:2] - pt_true[:2]) * 1000)

    def in_hand(s):
        R = d.xmat[palm[s]].reshape(3, 3)
        return R.T @ (d.xpos[top] - d.xpos[palm[s]]), R.T @ d.xmat[top].reshape(3, 3)

    def slip(s, lock):
        r, Rr = in_hand(s)
        ang = math.degrees(math.acos(np.clip((np.trace(lock[1].T @ Rr) - 1) / 2, -1, 1)))
        return float(np.linalg.norm(r - lock[0]) * 1000), ang

    # ---- plan
    hand = lambda tr, tf, ci, cm, cr, cp: np.array([tr, tf, ci, cm, cr, cp])  # noqa: E731
    pre_cmd = hand(p["tr"], 0.0, p["cpre"], p["cpre"], p["cpre"], p["cpre"])
    grip_cmd = hand(p["tr"], p["tf"], p["ci"], p["cm"], p["cr"], p["cp"])
    grip2 = hand(*[p.get(k + "2", p[k]) for k in ("tr", "tf", "ci", "cm", "cr", "cp")])   # v2: second closing step
    hcmd = {s: pre_cmd for s in SIDES}
    for s in SIDES:                                          # the IK sees the hand as it is in that phase
        for i, j in enumerate(meta["hand"][s]):
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = min(C[s][i] @ pre_cmd, hhi[s][i])
    mujoco.mj_forward(m, d)
    plan, ikr = {}, {}
    for s in SIDES:
        e, f, n = frames(p, s, spec, pt, Rt)
        sg = 1.0 if s == "right" else -1.0
        loc = np.array([p["px"], sg * p["py"], p["pz"]]) / 1000
        rim = pt + spec.R * e
        rim[2] = foot0 + p["hz"] / 1000                     # height above the table top (foot0 = the table surface)
        ey_ = np.array([0.0, np.sign(e[1]), 0.0])           # squeeze / lift-in straight sideways (radial at psi > 0
        # pushed the plate forward out of both hands, 40 mm in the first video)
        tg = dict(pre=rim + 0.040 * e + p["pre_up"] / 1000 * z, down=rim + 0.040 * e, inn=rim - p["din"] / 1000 * e,
                  sq=rim - p["din"] / 1000 * e - p["sq"] / 1000 * ey_,
                  lift=rim - p["din"] / 1000 * e - (p["sq"] + p["lift_in"]) / 1000 * ey_ + p["lift"] / 1000 * z)
        q = np.array(meta["ready"][s])
        plan[s], ikr[s] = {}, {}
        for k in ("pre", "down", "inn", "sq", "lift"):
            b = ik(m, meta, d, s, loc, tg[k], f, n, q, rng, 12 if k == "pre" else 2)
            q = b["q"]
            plan[s][k] = q
            ikr[s][k] = dict(pos_mm=b["pos_mm"], ang_deg=b["ang_deg"], margin_deg=b["margin_deg"])
    d2 = mujoco.MjData(m)
    d2.qpos[:] = d.qpos
    palm_plan = {s: {} for s in SIDES}
    for ph_, k in (("down", "down"), ("in", "inn"), ("squeeze", "sq"), ("lift", "lift")):
        for s in SIDES:
            d2.qpos[[m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]]] = plan[s][k]
        mujoco.mj_kinematics(m, d2)
        for s in SIDES:
            palm_plan[s][ph_] = d2.xpos[palm[s]].copy()
    reach_err = max(max(v["pos_mm"] for v in ikr[s].values() if True) for s in SIDES)
    out = dict(par={k: (round(v, 3) if isinstance(v, float) else v) for k, v in p.items()}, ik=ikr, reach_mm=reach_err)
    if p.get("plan_only"):                                    # the plan for the robot, pelvis frame
        P0 = np.array([0.0, 0.0, p["pelvis"]])
        FULL = np.array([1.55334, 1.0472, 1.41, 1.41, 1.41, 1.41])
        srvh = lambda c: [round(float(min(1.0, c[i] / FULL[i])), 3) for i in (1, 0, 2, 3, 4, 5)]  # noqa: E731
        out["plan"] = dict(
            plate_centre_pelvis=[round(float(x), 4) for x in pt - P0], rim_top_z_pelvis=round(float(pt[2] - P0[2] + spec.top_z if hasattr(spec, "top_z") else 0.0), 4),
            table_top_z_pelvis=round(float(foot0 - P0[2]), 4),
            hand_service={"pre": srvh(pre_cmd), "close": srvh(grip_cmd), "close2": srvh(grip2)},
            sides={})
        for s in SIDES:
            e, f, n = frames(p, s, spec, pt, Rt)
            sg = 1.0 if s == "right" else -1.0
            rim = pt + spec.R * e
            rim[2] = foot0 + p["hz"] / 1000
            ey_ = np.array([0.0, np.sign(e[1]), 0.0])
            tg = dict(pre=rim + 0.040 * e + p["pre_up"] / 1000 * z, down=rim + 0.040 * e, inn=rim - p["din"] / 1000 * e,
                      sq=rim - p["din"] / 1000 * e - p["sq"] / 1000 * ey_,
                      lift=rim - p["din"] / 1000 * e - (p["sq"] + p["lift_in"]) / 1000 * ey_ + p["lift"] / 1000 * z)
            out["plan"]["sides"][s] = dict(
                palm_point_in_palm_frame_mm=[p["px"], sg * p["py"], p["pz"]], fingers_dir_f=[round(float(x), 4) for x in f],
                palm_normal_n=[round(float(x), 4) for x in n],
                targets_pelvis_m={k: [round(float(x), 4) for x in v - P0] for k, v in tg.items()},
                q_rad={k: [round(float(x), 4) for x in plan[s][k]] for k in plan[s]}, ik=ikr[s])
        return out
    if reach_err > 10 or max(ikr[s]["pre"]["ang_deg"] for s in SIDES) > 25:
        out.update(result="unreachable", score=-3.0 - min(reach_err, 100) / 50, ok=False)
        return out

    # ---- video
    vid = vr = None
    vs = dict(next=0.0, phase="")
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))

    def grab():
        import cv2
        if d.time < vs["next"]:
            return
        vs["next"] += 0.04
        ims = []
        for cam in ("front", "left", "closeright", "top"):
            vr.update_scene(d, camera=cam)
            ims.append(cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR))
        bar = np.full((64, 1280, 3), 20, np.uint8)
        lo_, tl, sl = plate_state()
        cv2.putText(bar, title[:160], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(bar, f"t {d.time:4.1f} s {vs['phase']:<12s} plate lowest point up {lo_:5.1f} mm  tilt {tl:4.1f} deg  moved {sl:4.1f} mm  "
                    f"hand-plate L/R {plate_force('left'):4.1f}/{plate_force('right'):4.1f} N  hand-table {hand_table():4.1f} N",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    cur = {s: plan[s]["pre"].copy() for s in SIDES}
    hnow = {}
    for s in SIDES:
        d.qpos[adr[s]] = cur[s]
        d.ctrl[A[s]] = cur[s]
        hnow[s] = np.minimum(C[s] @ pre_cmd, hhi[s])
        d.ctrl[HA[s]] = hnow[s]
    mujoco.mj_forward(m, d)
    phases = []
    pk = dict(table=0.0)
    hc = {s: pre_cmd.copy() for s in SIDES}                   # hand command, sim order (rad)
    srv = None
    if p.get("robot"):                                        # the R1 server + measured arm / hand (platepick/r1_server_sim.py)
        from platepick.r1_server_sim import ServerSim
        srv = ServerSim(m, d, meta, A, HA, C, hhi, torque_scale=p.get("tscale", 1.0),
                        payload=None)
        for s in SIDES:
            srv.target(s, cur[s], pre_cmd)

    def sstep():
        if srv is None:
            mujoco.mj_step(m, d)
        else:
            srv.step()

    def step_log(ph):
        sstep()
        fm.update(d)
        if int(d.time / dt) % 10 == 0:
            ph["hand_table"] = max(ph["hand_table"], hand_table())
            for s in SIDES:
                ph["F"][s] = max(ph["F"][s], plate_force(s))
        if vid is not None:
            grab()

    def move(qb, T_, phase, cmd=None):
        vs["phase"] = phase
        qa = {s: cur[s].copy() for s in SIDES}
        ha = {s: hnow[s].copy() for s in SIDES}
        ca = {s: hc[s].copy() for s in SIDES}
        cb = cmd if cmd is not None else None
        hb = {s: np.minimum(C[s] @ cmd, hhi[s]) for s in SIDES} if cmd is not None else ha
        N = max(1, int(T_ / dt))
        ph = dict(F={s: 0.0 for s in SIDES}, hand_table=0.0)
        for k in range(N):
            u = (k + 1) / N
            u = 3 * u * u - 2 * u ** 3
            for s in SIDES:
                if srv is None:
                    d.ctrl[A[s]] = qa[s] + (qb.get(s, qa[s]) - qa[s]) * u
                    d.ctrl[HA[s]] = ha[s] + (hb[s] - ha[s]) * u
                else:
                    srv.target(s, qa[s] + (qb.get(s, qa[s]) - qa[s]) * u, ca[s] + ((cb - ca[s]) * u if cb is not None else 0.0))
            step_log(ph)
        lo_, tl, sl = plate_state()
        phases.append(dict(phase=phase, hand_plate_N={s: round(v, 1) for s, v in ph["F"].items()}, hand_table_N=round(ph["hand_table"], 1),
                           low_rise_mm=round(lo_, 1), tilt_deg=round(tl, 1), moved_mm=round(sl, 1)))
        if srv is not None:                                   # where the arm is vs the plan at the end of the phase, mrad
            phases[-1]["q_err_mrad"] = {s: [round(1000 * float(x)) for x in d.qpos[adr[s]] - qb.get(s, qa[s])] for s in SIDES}
            phases[-1]["palm_err_mm"] = {s: round(1000 * float(np.linalg.norm(d.xpos[palm[s]] - palm_plan[s].get(phase, d.xpos[palm[s]]))), 1) for s in SIDES}
        for s in SIDES:
            cur[s] = qb.get(s, cur[s]).copy()
            hnow[s] = hb[s].copy()
            if cb is not None:
                hc[s] = np.array(cb, float).copy()

    T = p["T"]
    move({}, 0.3, "settle")
    move({s: plan[s]["down"] for s in SIDES}, T, "down")
    corr = {s: np.zeros(5) for s in SIDES}
    for it in range(int(p.get("correct", 0))):               # aim correction in free air, 40 mm outside the rim:
        for k in range(int(0.4 / dt)):                        # settle, then command + (plan - measured) in joint space
            sstep()
        for s in SIDES:
            corr[s] += plan[s]["down"] - d.qpos[adr[s]]
        move({s: plan[s]["down"] + corr[s] for s in SIDES}, 0.5, "correct%d" % (it + 1))
        if srv is not None:
            phases[-1]["palm_err_mm"] = {s: round(1000 * float(np.linalg.norm(d.xpos[palm[s]] - palm_plan[s]["down"])), 1) for s in SIDES}
    if p.get("correct", 0):
        for k in range(int(0.4 / dt)):
            sstep()
        phases.append(dict(phase="corrected", palm_err_mm={s: round(1000 * float(np.linalg.norm(d.xpos[palm[s]] - palm_plan[s]["down"])), 1) for s in SIDES},
                           corr_mrad={s: [round(1000 * float(x)) for x in corr[s]] for s in SIDES}))
        for s in SIDES:
            for k in ("inn", "sq", "lift"):
                plan[s][k] = plan[s][k] + corr[s]
            for ph_ in ("in", "squeeze", "lift"):
                pass
    move({s: plan[s]["inn"] for s in SIDES}, T, "in")
    move({}, T, "close", cmd=grip_cmd)
    move({s: plan[s]["sq"] for s in SIDES}, T, "squeeze")
    if any(k + "2" in p for k in ("tr", "tf", "ci", "cm", "cr", "cp")):
        move({}, T, "close2", cmd=grip2)
    move({s: plan[s]["lift"] for s in SIDES}, T * 1.5, "lift")
    vs["phase"] = "hold"
    hold_parts = []
    for k in range(int(p["hold"] / dt)):
        sstep()
        fm.update(d)
        hold_parts.append(max(fm.now.values()))
        if vid is not None:
            grab()
    held_part = float(np.mean(hold_parts[len(hold_parts) // 2:]))
    lo1, tl1, _ = plate_state()
    lock = {s: in_hand(s) for s in SIDES}
    held_N = {s: round(plate_force(s), 1) for s in SIDES}
    sl_max = dict(mm=0.0, deg=0.0)

    def track():
        for s in SIDES:
            a, b = slip(s, lock[s])
            sl_max["mm"], sl_max["deg"] = max(sl_max["mm"], a), max(sl_max["deg"], b)

    vs["phase"] = "hold2"                                     # v2: hold still in the air, slip counted from here
    for k in range(int(p["hold2"] / dt)):
        sstep()
        fm.update(d)
        if k % 20 == 0:
            track()
        if vid is not None:
            grab()
    track()
    static_slip = dict(sl_max)
    if disturb and lo1 > 0.5 * p["lift"]:
        vs["phase"] = "disturb"
        ph = dict(F={s: 0.0 for s in SIDES}, hand_table=0.0)
        for fo, to in DIST:
            N = int(0.4 / dt)
            for k in range(N):
                ramp = min(1.0, k / (0.1 / dt))
                d.xfrc_applied[top] = np.r_[fo, to] * ramp
                step_log(ph)
                if k % 20 == 0:
                    track()
            d.xfrc_applied[top] = 0
            for k in range(int(0.15 / dt)):
                step_log(ph)
        vs["phase"] = "shake"
        N = int(1.0 / dt)
        base = {s: cur[s].copy() for s in SIDES}
        dq = {s: plan[s]["lift"] - plan[s]["sq"] for s in SIDES}   # joint step of the 40 mm lift: +/-10 mm = 1/4 of it
        for k in range(N):
            w = math.sin(2 * math.pi * 4 * k * dt) * 10 / p["lift"]
            for s in SIDES:
                if srv is None:
                    d.ctrl[A[s]] = base[s] + dq[s] * w
                else:
                    srv.target(s, base[s] + dq[s] * w)
            step_log(ph)
            if k % 20 == 0:
                track()
        for s in SIDES:
            if srv is None:
                d.ctrl[A[s]] = base[s]
            else:
                srv.target(s, base[s])
        for k in range(int(0.5 / dt)):
            step_log(ph)
        track()
        phases.append(dict(phase="disturb+shake", hand_plate_N={s: round(v, 1) for s, v in ph["F"].items()}, hand_table_N=round(ph["hand_table"], 1)))
    if vid is not None:
        vid.release()
    lo2, tl2, mv = plate_state()
    fs = fm.summary()
    peak = max(fs[f"{s}_part_max50ms_N"] for s in SIDES)
    ht = max(ph_.get("hand_table_N", 0) for ph_ in phases)
    out.update(phases=phases, lift_rise_mm=round(lo1, 1), lift_tilt_deg=round(tl1, 1), end_rise_mm=round(lo2, 1), end_tilt_deg=round(tl2, 1),
               moved_mm=round(mv, 1), held_N=held_N, slip_mm=round(sl_max["mm"], 1), slip_deg=round(sl_max["deg"], 1),
               static_slip_mm=round(static_slip["mm"], 1), static_slip_deg=round(static_slip["deg"], 1), disturbed=bool(disturb),
               peak50ms_N=round(peak, 1), held_finger_N=round(held_part, 1), hand_table_N=round(ht, 1))
    if srv is not None:
        out["server"] = srv.summary()
    lifted = lo1 > 0.6 * p["lift"] and tl1 < 10
    kept = lo2 > 0.6 * p["lift"] and tl2 < 10
    ok = bool(lifted and kept and sl_max["mm"] < 8 and sl_max["deg"] < 6 and peak <= 60 and held_part <= 30)
    res = "held" if ok else ("dropped/slipped" if lifted else ("not lifted"))
    score = (min(max(lo1, 0) / p["lift"], 1) + 2 * min(max(lo2, 0) / p["lift"], 1) - (tl1 + tl2) / 40
             - (min(sl_max["mm"], 40) / 20 + min(sl_max["deg"], 40) / 20 if lifted else 2.0)
             - min(3.0, max(0.0, peak - 60) / 40) - min(3.0, max(0.0, held_part - 30) / 20) - min(2.0, max(0.0, ht - 15) / 40)
             - max(0.0, reach_err - 3) / 20)
    if not lifted:                                           # shaping: both hands on the plate during the lift, plate not shoved
        fl = next(ph_ for ph_ in phases if ph_["phase"] == "lift")["hand_plate_N"]
        score += 0.6 * min(min(fl.values()), 20.0) / 20 - 0.5 * min(mv, 60.0) / 60
    if peak > 200:                                           # hand inside the plate (kN contact spikes): not a grip
        res, score = "penetration", min(score, -4.0)
    out.update(result=res, ok=ok, score=round(float(score), 3))
    return out


# ---------------------------------------------------------------- search
def decode(u):
    return dict(zip(NAMES, (LO + np.clip(u, 0, 1) * (HI - LO)).tolist()))


def encode(p):
    p = dict(p)
    for k in ("tr", "tf", "ci", "cm", "cr", "cp"):          # no second step given: it repeats the first
        p.setdefault(k + "2", p[k])
    return np.clip((np.array([p[k] for k in NAMES]) - LO) / (HI - LO), 0, 1)


def cond_par(base, c):
    return dict(base, pad_mu=c[0], table_mu=c[1], ex=c[2], ey=c[3])


def _eval(job):
    i, p, c = job
    try:
        r = run(cond_par(p, c))
    except Exception:  # noqa: BLE001
        r = dict(result="error", score=-5.0, ok=False, err=traceback.format_exc()[-400:])
    return i, c, r


def _run_one(p):
    try:
        return run(p)
    except Exception:  # noqa: BLE001
        return dict(par=p, result="error", ok=False, score=-5.0, err=traceback.format_exc()[-400:])


def fitness(rs):
    sc = np.array([r["score"] for r in rs])
    n_ok = int(sum(r["ok"] for r in rs))                    # mean (+ passes), the worst case made the search flat
    return float(sc.mean() + 0.5 * n_ok), n_ok


def search(a):
    from multiprocessing import Pool
    os.makedirs(a.search, exist_ok=True)
    rng = np.random.default_rng(a.seed)
    mean = encode(dict(FAMILIES[a.family], **parse_set(a.set)))
    std = np.full(len(NAMES), a.sigma)
    best = dict(f=-1e9)
    log = open(os.path.join(a.search, "log.jsonl"), "a")
    with Pool(a.workers) as pool:
        for g in range(a.gens):
            U = np.clip(mean + std * rng.standard_normal((a.pop, len(NAMES))), 0, 1)
            if g == 0:
                U[0] = mean
            jobs = [(i, decode(U[i]), c) for i in range(a.pop) for c in CONDS]
            res = {}
            for i, c, r in pool.imap_unordered(_eval, jobs):
                r["cond"] = list(c)
                res.setdefault(i, []).append(r)
            fits = [fitness(res[i]) for i in range(a.pop)]
            order = np.argsort([-f[0] for f in fits])
            elite = U[order[:max(2, a.pop // 5)]]
            w = np.log(len(elite) + 0.5) - np.log(np.arange(1, len(elite) + 1))
            w /= w.sum()
            mean = (w[:, None] * elite).sum(0)
            std = np.clip(0.7 * std + 0.3 * np.sqrt((w[:, None] * (elite - mean) ** 2).sum(0)), 0.02, 0.3)
            ib = order[0]
            if fits[ib][0] > best["f"]:
                best = dict(f=fits[ib][0], n_ok=fits[ib][1], gen=g, p=decode(U[ib]),
                            conds=[{k: r.get(k) for k in ("cond", "result", "score", "lift_rise_mm", "end_rise_mm", "slip_mm", "slip_deg",
                                                          "peak50ms_N", "held_finger_N", "held_N", "hand_table_N", "moved_mm", "reach_mm")}
                                   for r in sorted(res[ib], key=lambda r: CONDS.index(tuple(r["cond"])))])
                json.dump(best, open(os.path.join(a.search, "best.json"), "w"), indent=1, default=float)
            line = dict(gen=g, best_f=round(best["f"], 3), best_ok=best["n_ok"], gen_f=round(fits[ib][0], 3), gen_ok=fits[ib][1],
                        mean_ok=round(float(np.mean([f[1] for f in fits])), 2), std=round(float(std.mean()), 3),
                        results={k: sum(r["result"] == k for i in res for r in res[i]) for k in ("held", "dropped/slipped", "not lifted", "penetration", "unreachable", "error")})
            log.write(json.dumps(line) + "\n")
            log.flush()
            print(json.dumps(line), flush=True)


def parse_set(s):
    return {k: float(v) for k, v in (kv.split("=") for kv in s.split(",") if kv)} if s else {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", default="wrap", choices=list(FAMILIES))
    ap.add_argument("--set", default="")
    ap.add_argument("--cond", default="0.25,0.45,0,0")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--best", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--search", default=None)
    ap.add_argument("--gens", type=int, default=25)
    ap.add_argument("--pop", type=int, default=40)
    ap.add_argument("--sigma", type=float, default=0.15)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=None)
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--disturb", action="store_true", help="also the v1 pushes / twists / shake after the static hold")
    a = ap.parse_args()
    if a.search:
        return search(a)
    if a.grid:                                               # tolerance / friction / height map of one grip
        from multiprocessing import Pool
        p = json.load(open(a.best))["p"]
        jobs = [dict(p, pad_mu=0.25, table_mu=0.45, ex=float(x), ey=float(y), grid="place")
                for x in (-20, -15, -10, -5, 0, 5, 10, 15, 20) for y in (-20, -15, -10, -5, 0, 5, 10, 15, 20)]
        jobs += [dict(p, pad_mu=pm, table_mu=tm, ex=0.0, ey=0.0, grid="friction") for pm in (0.2, 0.25, 0.35, 0.5, 0.8) for tm in (0.3, 0.6)]
        jobs += [dict(p, pad_mu=0.25, table_mu=0.45, ex=0.0, ey=0.0, table=t, pelvis=pv, grid="height")
                 for t in (0.747, 0.755, 0.762) for pv in (0.69, 0.71, 0.73)]
        with Pool(a.workers) as pool, open(a.out, "w") as f:
            for i, r in enumerate(pool.imap_unordered(_run_one, jobs)):
                r.pop("ik", None)
                f.write(json.dumps(r, default=float) + chr(10))
                f.flush()
                print(i + 1, len(jobs), r["result"], flush=True)
        return
    p = json.load(open(a.best))["p"] if a.best else dict(FAMILIES[a.family])
    p.update(parse_set(a.set))
    conds = CONDS if a.all else [tuple(float(x) for x in a.cond.split(","))]
    rows = []
    for c in conds:
        t0 = time.time()
        vid = None
        if a.video:
            vid = a.video if len(conds) == 1 else a.video.replace(".mp4", "_pad%.2f_table%.2f_ex%+.0f_ey%+.0f.mp4" % c)
        title = a.title or f"MuJoCo lone grip ({a.family}), table 0.755, pad {c[0]}, foot-table {c[1]}, plate off {c[2]:+.0f}/{c[3]:+.0f} mm"
        r = run(cond_par(p, c), video=vid, title=title, disturb=a.disturb)
        r["wall_s"] = round(time.time() - t0, 1)
        rows.append(r)
        print(json.dumps({k: r.get(k) for k in ("result", "score", "reach_mm", "lift_rise_mm", "end_rise_mm", "end_tilt_deg", "slip_mm", "slip_deg",
                                                 "held_N", "held_finger_N", "peak50ms_N", "hand_table_N", "moved_mm", "wall_s")}), "cond", c, flush=True)
        if len(conds) == 1:
            for ph in r.get("phases", []):
                print("   ", ph)
    if a.out:
        with open(a.out, "w") as f:
            for r in rows:
                f.write(json.dumps(r, default=float) + "\n")


if __name__ == "__main__":
    main()
