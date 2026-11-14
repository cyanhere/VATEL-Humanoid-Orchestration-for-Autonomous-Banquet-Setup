"""push10_grip's right-hand grip in MuJoCo, starting after the push, with the planner's inputs and fixes as switches.

The top plate is placed `slide` mm from the stack and settled. The left arm starts at the robot's measured joints;
the right hand runs the robot's push10_grip sequence through the simulated server and arm model.

    python scripts/push10_grip_sim.py --set height=plane,tap=1,out=55,hook=3 -o OUT [--frames]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(ROOT)
DEPLOY = os.path.join(PROJECT, "robot")
os.environ.setdefault("DEXPICK_KP_ARM", "50,50,40,40,30")
os.environ.setdefault("DEXPICK_KV_ARM", "2")
sys.path.insert(0, ROOT)
sys.path.insert(0, DEPLOY)
import mujoco  # noqa: E402

from platepick.plate import Friction  # noqa: E402
from platepick.r1_server_sim import FULL_SIM, ServerSim  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402
from pk import config  # noqa: E402
from pk.ik import ArmChain  # noqa: E402
from pk.model import Robot, hand_points  # noqa: E402
from pk.stages.lone_grip import aim_point, palm_ik, targets  # noqa: E402

SIDES = ("left", "right")
SRV2SIM = [1, 0, 2, 3, 4, 5]
# the R1 at the start of its grip, 24 (server_20261106_143244.csv, wall 1511.5): measured joints, hand commands
R1_PLATE_X, R1_PLATE_Y = 0.3545, 0.0388   # MuJoCo's settled top plate for that run (scripts/push10_grip_replay.py)
DEF = dict(stack_x=0.35589, stack_y=0.08132, slide=42.8, slide_x=-1.4, pelvis=0.71, table=0.755, pad=0.25, table_mu=0.45,
           glaze=0.375, foot=0.575, cam_y=0.0, cam_x=0.0, tap=0, tap_err=0.0, height="median", plane_err=0.0, out=40.0, hook=None, hook_gap=1.0,
           sq=None, lift=30.0, hold=2.0, grip="v2_thumb", T=1.2, v=0.3, correct_n=2, correct_pre_n=1, seed=0,
           wrist=0, wrist_at="high", wrist_free="cy+z0+gy", wrist_prior=15.0, wrist_dz=None, wrist_use=1, stop_after=None)


def wrist_modules():
    """Wrist-camera modules, imported only for wrist=1 runs."""
    from platepick.wrist_cam_sim import scene_cams, sim_cams
    from pk import wristcam
    return scene_cams, sim_cams, wristcam


def r1_start():
    """The R1's measured left / right joints and hand commands (service order) at t 31.5 s of the 14:57:24 run
    (server_20261106_143244.csv, wall 1511.45)."""
    return (np.array([-0.38599, 1.11641, -0.64544, -0.11744, 0.57293]), np.array([-0.00801, -0.87741, -0.29047, -0.77631, 0.00795]),
            np.array([0.0, 0.031, 0.0, 0.767, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))


def srv2sim(h_srv):
    return np.asarray(h_srv, float)[SRV2SIM] * FULL_SIM


def run(p, out_dir=None, frames=False):
    p = dict(DEF, **p)
    scene_cams, sim_cams, wristcam = wrist_modules() if p["wrist"] else (None, None, None)
    t_wall = time.time()
    rng = np.random.default_rng(int(p["seed"]))
    L = json.load(open(os.path.join(DEPLOY, "params", "lone_grip.json")))
    G = L["grips"][p["grip"]]
    g = dict(G["genes"])
    if p["sq"] is not None:
        g["sq"] = float(p["sq"])
    LP = dict(L, out_mm=float(p["out"]), lift_mm=0.0)
    fr = Friction(glaze_glaze=p["glaze"], foot_glaze=p["foot"], pad_glaze=p["pad"], table=p["table_mu"])
    sc = RobotScene(n=3, table=p["table"], xy=(p["stack_x"], p["stack_y"]), pelvis=p["pelvis"], fr=fr,
                    wrist_cams=scene_cams() if p["wrist"] else None)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    P0 = np.array([0.0, 0.0, p["pelvis"]])
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    top, low = m.body("plate2").id, m.body("plate1").id
    ja = m.jnt_qposadr[m.body_jntadr[top]]
    d.qpos[ja] += p["slide_x"] / 1000
    d.qpos[ja + 1] -= p["slide"] / 1000
    d.qpos[ja + 2] += 0.012
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    settle_plates(m, d, steps=3000)
    low0 = d.xpos[low].copy()

    def rim_z(a):                                             # pelvis-frame rim-top height at angle a (rad) around the plate
        R = d.xmat[top].reshape(3, 3)
        return float((R @ np.array([spec.R * math.cos(a), spec.R * math.sin(a), spec.H]) + d.xpos[top])[2] - P0[2])

    def tilt():
        return math.degrees(math.acos(min(1.0, d.xmat[top][8])))

    c_true = d.xpos[top] - P0
    rest = dict(tilt_deg=round(tilt(), 2), right_rim=round(1000 * rim_z(-math.pi / 2), 1), left_rim=round(1000 * rim_z(math.pi / 2), 1),
                centre=[round(1000 * float(v), 1) for v in c_true])

    # ---- arms and hands at the R1's grip start, the left moved with the plate
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = {s: [act[f"a_{j}"] for j in meta["arm"][s]] for s in SIDES}
    adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]] for s in SIDES}
    HA = {s: [act[f"a_{j}"] for j in meta["hand"][s]] for s in SIDES}
    C = {s: np.array(meta["coupling"][s]) for s in SIDES}
    hhi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"][s]]) for s in SIDES}
    hadr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["hand"][s]] for s in SIDES}
    qL0, qR0, hLs, hRs = r1_start()
    rob = Robot()
    q11 = np.zeros(11)
    q11[config.LEFT], q11[config.RIGHT] = qL0, qR0
    h12 = np.r_[hLs, hRs]
    chL = ArmChain(rob, "left", "left_base_link", q11, h12)
    T0, _ = chL.fk(qL0)
    shift = np.array([c_true[0] - R1_PLATE_X, c_true[1] - R1_PLATE_Y, 0.0])     # the left hand follows the plate

    def left_ik(dp, q0):
        T, _ = chL.fk(q0)
        q, info = palm_ik(chL, q0, T[:3, 3] + dp, np.zeros(3), T[:3, 2], T[:3, 0], 1.0)
        return q, info
    qL, infoL = left_ik(shift, qL0) if np.linalg.norm(shift) > 1e-4 else (qL0.copy(), dict(pos_mm=0.0))
    hL, hR = srv2sim(hLs), srv2sim(G["hand"]["pre"])
    d.qpos[adr["left"]], d.qpos[adr["right"]] = qL, qR0
    d.qpos[hadr["left"]] = np.minimum(C["left"] @ hL, hhi["left"])
    d.qpos[hadr["right"]] = np.minimum(C["right"] @ srv2sim(hRs), hhi["right"])
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    hand_geoms = {s: {gg for gg in range(m.ngeom) if m.body(m.geom_bodyid[gg]).name.startswith(s + "_")} for s in SIDES}
    top_geoms = {gg for gg in range(m.ngeom) if m.geom_bodyid[gg] == top}
    low_geoms = {gg for gg in range(m.ngeom) if m.body(m.geom_bodyid[gg]).name in ("plate0", "plate1")}

    def set_robot():
        d.qpos[adr["left"]], d.qpos[adr["right"]] = qL, qR0
        d.qpos[hadr["left"]] = np.minimum(C["left"] @ hL, hhi["left"])
        d.qpos[hadr["right"]] = np.minimum(C["right"] @ srv2sim(hRs), hhi["right"])
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)

    def left_pen():
        mujoco.mj_forward(m, d)
        plates = top_geoms | low_geoms
        dd = [d.contact[i].dist for i in range(d.ncon)
              if (d.contact[i].geom1 in plates and d.contact[i].geom2 in hand_geoms["left"])
              or (d.contact[i].geom2 in plates and d.contact[i].geom1 in hand_geoms["left"])]
        return -min(dd) if dd else 0.0

    # at the robot's joints the left hand overlaps the plates by up to 5-6 mm in MuJoCo: raise it (palm translation,
    # IK) until it only touches, and hold it there
    pen_start = left_pen()
    raise_mm = 0.0
    qL_meas = qL.copy()
    while left_pen() > 0.0003 and raise_mm < 15.0:
        raise_mm += 0.5
        qL, _ = left_ik(np.array([0.0, 0.0, raise_mm / 1000]), qL_meas)
        d.qpos[adr["left"]] = qL
    low0 = d.xpos[low].copy()
    c_true = d.xpos[top] - P0
    rest.update(left_overlap_at_R1_joints_mm=round(1000 * pen_start, 1), left_raised_mm=raise_mm, left_overlap_now_mm=round(1000 * left_pen(), 1))
    if p.get("place_only"):
        return dict(rest=rest)

    def force(A_, B_):
        F, w = 0.0, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 in A_ and c.geom2 in B_) or (c.geom2 in A_ and c.geom1 in B_):
                mujoco.mj_contactForce(m, d, i, w)
                F += float(np.linalg.norm(w[:3]))
        return F

    srv = ServerSim(m, d, meta, A, HA, C, hhi, torque_scale=1.0)
    cur = {"left": qL.copy(), "right": qR0.copy()}
    hcur = {"left": hL.copy(), "right": srv2sim(hRs)}
    srv.target("left", cur["left"], hcur["left"])
    srv.target("right", cur["right"], hcur["right"])

    # ---- what the robot believes and its plan (deploy code)
    c_bel = c_true.copy()
    c_bel[0] += p["cam_x"] / 1000
    c_bel[1] += p["cam_y"] / 1000
    if p["tap"]:
        c_bel[1] = c_true[1] + (p["tap_err"] + rng.normal(0, 1.0)) / 1000
    if p["height"] == "plane":
        z_rim = rim_z(-math.pi / 2) + (p["plane_err"] + rng.normal(0, 0.5)) / 1000
    else:
        z_rim = np.mean([rim_z(a) for a in np.linspace(0, 2 * math.pi, 36, endpoint=False)]) - 0.0015
    q11[config.LEFT], q11[config.RIGHT] = qL, qR0
    chR = ArmChain(rob, "right", "right_base_link", q11, h12)
    loc = aim_point(g, "right")
    ref = G["ref_q_rad"]["right"]

    def plan(cxy, zr):
        """deploy's targets + IK for the right hand from a belief (plate centre xy, gripped-rim height)."""
        tg_, f_, n_ = targets(g, "right", (float(cxy[0]), float(cxy[1])), zr - spec.H, LP, 0.0)
        rq_, ik_, prev = {}, {}, None
        for k in ("pre", "down", "inn", "sq", "high"):
            best = None
            for q0 in [np.array(ref["pre" if k == "high" else k], float)] + ([prev] if prev is not None else []):
                q, info = palm_ik(chR, q0, tg_[k], loc, f_, n_, LP["ik_w_rot"])
                cst = info["pos_mm"] + 0.3 * (info["f_deg"] + info["n_deg"])
                if best is None or cst < best[0]:
                    best = (cst, q, info)
            rq_[k], ik_[k] = best[1], {a: round(b, 1) for a, b in best[2].items()}
            if k != "high":
                prev = best[1]
        if p["lift"] > 0:
            q, info = palm_ik(chR, rq_["sq"], tg_["sq"] + np.array([0, 0, p["lift"] / 1000]), loc, f_, n_, LP["ik_w_rot"])
            rq_["lift"], ik_["lift"] = q, {a: round(b, 1) for a, b in info.items()}
        return tg_, f_, n_, rq_, ik_
    tg, f, n, rq, ik = plan(c_bel, z_rim)
    palm_t = {k: tg[k] for k in tg}

    # ---- logging, frames (one renderer for everything: on Windows only the first Renderer of a process draws)
    rec = []
    shots = []
    wshots = []
    wdata = []
    ren = mujoco.Renderer(m, 480, 640) if (frames or p["wrist"]) else None
    st = dict(phase="start", Fr=0.0, Fl=0.0, Flow=0.0)
    wr = m.body("right_base_link").id

    def shot(label):
        if ren is None or not frames:
            return
        import cv2
        ims = []
        for cam in ("closeright", "front", "top", "left"):
            ren.update_scene(d, camera=cam)
            ims.append(cv2.resize(ren.render(), (480, 360), interpolation=cv2.INTER_AREA))
        shots.append((label, np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])])))

    wcam = sim_cams()["right"] if p["wrist"] else None

    def true_plane():
        """The top plate's rim plane, [cx, cy, z0, gx, gy] (pelvis frame), from MuJoCo."""
        R_ = spec.R
        z = {a: rim_z(a) for a in (0.0, math.pi / 2, math.pi, -math.pi / 2)}
        cxy = d.xpos[top][:2] - P0[:2]
        return np.array([cxy[0], cxy[1], np.mean(list(z.values())), (z[0.0] - z[math.pi]) / (2 * R_),
                         (z[math.pi / 2] - z[-math.pi / 2]) / (2 * R_)])

    def wrist_measure(label, hand_srv):
        import cv2
        q11m = np.zeros(11)
        q11m[config.LEFT], q11m[config.RIGHT] = d.qpos[adr["left"]], d.qpos[adr["right"]]
        T = rob.fk(q11m, np.r_[hLs, np.asarray(hand_srv, float)])
        Tc = wcam.pose(T)
        ren.update_scene(d, camera="wrist_right")
        rgb = ren.render()
        occ = np.vstack([np.vstack(list(hand_points(T, s_).values())) for s_ in SIDES])
        over = {} if p["wrist_dz"] is None else {"edge_dz_mm": float(p["wrist_dz"])}
        if p.get("wrist_arc") is not None:
            over["arc_deg"] = [float(v) for v in str(p["wrist_arc"]).split("+")]
        rim = wristcam.RimEdge(wcam, **over)
        p0 = np.array([bel["c"][0], bel["c"][1], bel["z"], 0.0, 0.0])
        free = p["wrist_free"].split("+")
        o = rim.measure(wristcam.to_gray(rgb), Tc, p0, spec.R, occluders=occ, free=free, sigma0_mm=[p["wrist_prior"]] * len(free))
        pt = true_plane()
        ag = -math.pi / 2
        z_true = rim_z(ag)
        rep = dict(at=label, ok=o["ok"], why=o["why"], n_used=o.get("n_used"), n_seen=o.get("n_seen"), rms_px=o.get("rms_px"), sd_mm=o.get("sd_mm"),
                   before=dict(cx_mm=round(1000 * (p0[0] - pt[0]), 1), cy_mm=round(1000 * (p0[1] - pt[1]), 1), rim_z_mm=round(1000 * (bel["z"] - z_true), 1)),
                   after=dict(cx_mm=round(1000 * (o["p"][0] - pt[0]), 1), cy_mm=round(1000 * (o["p"][1] - pt[1]), 1),
                              rim_z_mm=round(1000 * (wristcam.rim_height(o["p"], spec.R, ag) - z_true), 1)),
                   cam_to_rim_mm=round(1000 * float(np.linalg.norm(wristcam.rim_points(pt, spec.R, np.array([ag]))[0] - Tc[:3, 3])), 1))
        txt = ["MuJoCo wrist_right at '%s' (camera mount: params/wrist_cam.json, provisional)" % label,
               "edge points used %s of %s seen, rms %s px, %s" % (o.get("n_used"), o.get("n_seen"), o.get("rms_px"), "OK" if o["ok"] else o["why"]),
               "belief error before: centre y %+.1f, gripped rim z %+.1f mm" % (rep["before"]["cy_mm"], rep["before"]["rim_z_mm"]),
               "after the wrist camera: centre y %+.1f, gripped rim z %+.1f mm" % (rep["after"]["cy_mm"], rep["after"]["rim_z_mm"])]
        fing = np.vstack([v for k_, v in hand_points(T, "right").items() if "touch" in k_])
        wshots.append((label, wristcam.draw(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), wcam, Tc, o, spec.R, p_true=pt, fingers=fing[::25], text=txt)))
        wdata.append((label, dict(rgb=rgb, Tc=Tc, p0=p0, p_true=pt, occ=occ, R=spec.R, z_true_grip=z_true, fing=fing)))
        return o, rep

    def step():
        srv.step()
        if int(round(d.time / dt)) % 10 == 0:
            Fr, Fl = force(top_geoms, hand_geoms["right"]), force(top_geoms, hand_geoms["left"])
            Flow = force(low_geoms, hand_geoms["right"] | hand_geoms["left"])
            st["Fr"], st["Fl"], st["Flow"] = max(st["Fr"], Fr), max(st["Fl"], Fl), max(st["Flow"], Flow)
            rec.append([d.time, *(1000 * (d.xpos[top] - P0)), tilt(), 1000 * rim_z(-math.pi / 2), 1000 * rim_z(math.pi / 2), Fr, Fl, Flow,
                        *(1000 * (d.xpos[low] - low0))])

    def tick(n_steps):
        for _ in range(n_steps):
            step()

    def move(side_q, T, phase, hand=None, left_q=None):
        """Joint smoothstep of the right arm (and left if given) at 30 Hz commands; T stretched to the speed cap."""
        st["phase"] = phase
        shot(phase)
        qa = {s: cur[s].copy() for s in SIDES}
        qb = {"right": side_q if side_q is not None else qa["right"], "left": left_q if left_q is not None else qa["left"]}
        ha = hcur["right"].copy()
        hb = ha if hand is None else hand
        T = max(T, max(float(np.max(np.abs(qb[s] - qa[s]))) for s in SIDES) / p["v"])
        N = max(1, int(round(T * 30)))
        per = int(round(1 / 30 / dt))
        for i in range(1, N + 1):
            u = i / N
            u = 3 * u * u - 2 * u ** 3
            for s in SIDES:
                srv.target(s, qa[s] + u * (qb[s] - qa[s]), hcur[s] if s == "left" else ha + u * (hb - ha))
            tick(per)
        for s in SIDES:
            cur[s] = qb[s].copy()
        hcur["right"] = hb.copy()

    def correct(key, nn, label):
        out = []
        for _ in range(int(nn)):
            st["phase"] = label
            tick(int(0.4 / dt))
            qm = d.qpos[adr["right"]].copy()
            rcorr[:] += rq[key] - qm
            Tm, _ = chR.fk(qm)
            err = 1000 * (Tm[:3, :3] @ loc + Tm[:3, 3] - palm_t[key])
            out.append([round(float(x), 1) for x in err])
            move(rq[key] + rcorr, 0.5, label)
        return out

    def plate_phase():
        return dict(y=round(1000 * float(d.xpos[top][1] - P0[1]), 1), tilt=round(tilt(), 2), right_rim=round(1000 * rim_z(-math.pi / 2), 1),
                    left_rim=round(1000 * rim_z(math.pi / 2), 1))

    res = dict(par=p, rest=rest, ik=ik, belief=dict(centre_y_err_mm=round(1000 * float(c_bel[1] - c_true[1]), 1),
                                                     rim_z_err_mm=round(1000 * (z_rim - rim_z(-math.pi / 2)), 1)),
               left_shift_mm=[round(1000 * float(v), 1) for v in shift], left_ik_mm=round(infoL["pos_mm"], 1))
    worst = max(v["pos_mm"] for v in ik.values())
    res["ik_worst_mm"] = round(worst, 1)
    if worst > LP["ik_max_mm"]:
        res.update(result="unreachable")
        return res
    rcorr = np.zeros(5)
    ph = {}
    tick(int(0.5 / dt))
    ph["start"] = plate_phase()
    move(rq["high"], 2.0, "approach", hand=srv2sim(G["hand"]["pre"]))
    bel = dict(c=c_bel.copy(), z=float(z_rim))
    res["wrist"] = []

    def wrist_step(label):
        """Measure with the wrist camera; with wrist_use, take the measurement as the belief and replan the right hand."""
        nonlocal tg, f, n, rq, ik, palm_t
        tick(int(0.3 / dt))
        o, rep = wrist_measure(label, G["hand"]["pre"])
        if o["ok"] and p["wrist_use"]:
            bel["c"][:2] = o["p"][:2]
            bel["z"] = wristcam.rim_height(o["p"], spec.R, -math.pi / 2)
            tg, f, n, rq, ik = plan(bel["c"], bel["z"])
            palm_t = {k: tg[k] for k in tg}
            rep["replanned"] = True
            rep["ik_worst_mm"] = round(max(v["pos_mm"] for v in ik.values()), 1)
        res["wrist"].append(rep)
        return rep
    if p["wrist"] and "high" in p["wrist_at"]:
        wrist_step("high")
    move(rq["pre"], p["T"], "pre")
    res["err_pre"] = correct("pre", p["correct_pre_n"], "precorrect")
    if p["wrist"] and "pre" in p["wrist_at"]:
        wrist_step("pre")
    if p["stop_after"] == "pre":
        return finish(res, out_dir, rec, shots, wshots, t_wall, wdata)
    move(rq["down"] + rcorr, p["T"], "down")
    res["err_down"] = correct("down", p["correct_n"], "correct")
    if p["wrist"] and "down" in p["wrist_at"]:
        wrist_step("down")
        if p["wrist_use"]:
            move(rq["down"] + rcorr, p["T"], "down (wrist)")
            res["err_down_wrist"] = correct("down", 1, "correct")
    if p["stop_after"] == "down":
        return finish(res, out_dir, rec, shots, wshots, t_wall, wdata)
    tick(int(0.3 / dt))
    Tm, _ = chR.fk(d.qpos[adr["right"]].copy())
    res["palm_err_down_mm"] = [round(float(x), 1) for x in 1000 * (Tm[:3, :3] @ loc + Tm[:3, 3] - palm_t["down"])]
    ph["corrected"] = plate_phase()
    st["Fr"] = 0.0
    move(rq["inn"] + rcorr, p["T"], "in")
    ph["in"] = plate_phase()
    res["F_right_in_N"] = round(st["Fr"], 1)
    move(rq["inn"] + rcorr, p["T"], "close", hand=srv2sim(G["hand"]["close"]))
    ph["close"] = plate_phase()
    if p["hook"] is not None:
        # backstop: move the left hand in (-y) until the hook is `hook_gap` mm outside the believed left edge, then down
        # until its lowest point is `hook` mm below the left rim's top, so the squeeze has a stop on the far side
        def hook_pts(qLx):
            T = rob.fk(np.r_[qLx, cur["right"], 0.0], h12)
            pts = hand_points(T, "left")
            return np.vstack([pts[k] for k in pts if "middle" in k or "ring" in k])
        hk = hook_pts(cur["left"])
        edge_y = float(c_bel[1]) + spec.R
        dy = edge_y + p["hook_gap"] / 1000 - float(hk[:, 1].min())
        qa, _ = left_ik(np.array([0.0, dy, 0.0]), cur["left"])
        hk = hook_pts(qa)
        dz = float(np.clip(rim_z(math.pi / 2) - p["hook"] / 1000 - float(hk[:, 2].min()), -0.012, 0.012))
        qb, _ = left_ik(np.array([0.0, 0.0, dz]), qa)
        res["hook_move_mm"] = [round(1000 * dy, 1), round(1000 * dz, 1)]
        move(None, p["T"], "hook in", left_q=qa)
        move(None, p["T"], "hook down", left_q=qb)
        ph["hook"] = plate_phase()
    move(rq["sq"] + rcorr, p["T"], "squeeze")
    ph["squeeze"] = plate_phase()
    move(rq["sq"] + rcorr, p["T"], "close2", hand=srv2sim(G["hand"]["close2"]))
    ph["close2"] = plate_phase()
    res["F_right_grip_N"] = round(st["Fr"], 1)
    if p["lift"] > 0:
        qLl, _ = left_ik(np.array([0, 0, p["lift"] / 1000]), cur["left"])
        psi = math.radians(g["psi"])
        grip_loc = np.array([-spec.R * math.sin(psi), -spec.R * math.cos(psi), spec.H])   # the gripped rim point, plate frame

        def in_palm():                                          # the gripped rim point in the right palm frame
            Rw = d.xmat[wr].reshape(3, 3)
            return Rw.T @ (d.xmat[top].reshape(3, 3) @ grip_loc + d.xpos[top] - d.xpos[wr])
        palm0 = in_palm()
        move(rq["lift"] + rcorr, p["T"] * 1.5, "lift", left_q=qLl)
        st["phase"] = "hold"
        shot("hold")
        tick(int(0.6 / dt))
        lock = in_palm()
        ph["lifted"] = plate_phase()
        tick(int(p["hold"] / dt))
        now = in_palm()
        ph["held"] = plate_phase()
        res["slip_hold_mm"] = round(1000 * float(np.linalg.norm(now - lock)), 1)
        res["slip_lift_mm"] = round(1000 * float(np.linalg.norm(lock - palm0)), 1)
    shot("end")
    res["phases"] = ph
    res["F_right_max_N"], res["F_left_max_N"], res["F_hands_on_lower_max_N"] = round(st["Fr"], 1), round(st["Fl"], 1), round(st["Flow"], 1)
    res["lower_moved_mm"] = round(1000 * float(np.linalg.norm(d.xpos[low][:2] - low0[:2])), 1)
    res["slide_back_mm"] = round(ph["squeeze"]["y"] - ph["in"]["y"], 1)        # + = toward the left hand
    res["server"] = srv.summary()
    if p["lift"] > 0:
        res["right_rim_rise_mm"] = round(ph["held"]["right_rim"] - ph["close2"]["right_rim"], 1)
        res["left_rim_rise_mm"] = round(ph["held"]["left_rim"] - ph["close2"]["left_rim"], 1)
        lifted = res["right_rim_rise_mm"] > 0.6 * p["lift"]
        res["ok"] = bool(lifted and res["slip_hold_mm"] < 5 and res["slip_lift_mm"] < 8 and abs(res["slide_back_mm"]) < 6 and res["F_hands_on_lower_max_N"] < 20
                         and res["lower_moved_mm"] < 5)
        res["result"] = "held" if res["ok"] else ("lifted, " if lifted else "not lifted, ") + ", ".join(
            w for w, c in (("slipped", res["slip_hold_mm"] >= 5 or res["slip_lift_mm"] >= 8), ("slid back", abs(res["slide_back_mm"]) >= 6),
                           ("pressed lower plates", res["F_hands_on_lower_max_N"] >= 20), ("lower plate moved", res["lower_moved_mm"] >= 5)) if c)
    return finish(res, out_dir, rec, shots, wshots, t_wall, wdata)


def finish(res, out_dir, rec, shots, wshots, t_wall, wdata=()):
    res["wall_s"] = round(time.time() - t_wall)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
        np.save(os.path.join(out_dir, "trace.npy"), np.array(rec))
        json.dump(res, open(os.path.join(out_dir, "result.json"), "w"), indent=1, default=float)
        import cv2
        for i, (label, im) in enumerate(shots):
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            cv2.putText(im, "MuJoCo  " + label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(out_dir, "frame_%02d_%s.jpg" % (i, label)), im)
        for label, im in wshots:
            cv2.imwrite(os.path.join(out_dir, "wrist_right_%s.jpg" % label), im)
        for label, dd in wdata:                     # the measurement's inputs, for offline tuning
            np.savez_compressed(os.path.join(out_dir, "wrist_right_%s.npz" % label), **dd)
    return res


def parse_set(s):
    out = {}
    for kv in filter(None, s.split(",")):
        k, v = kv.split("=", 1)
        try:
            v = json.loads(v)
        except ValueError:
            pass
        out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--set", default="")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--frames", action="store_true")
    a = ap.parse_args()
    r = run(parse_set(a.set), a.out, a.frames)
    r.pop("par", None)
    print(json.dumps(r, indent=1, default=float))


if __name__ == "__main__":
    main()
