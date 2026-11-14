"""Stage 3, slot grip: the plate rests between the right index and middle fingers and the thumb locks it.

    python scripts/grip_slot.py --state data/results/robot/state_push_best_n2_t0.747.npz --sheet out.png
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402

from grip_pinch import sheet  # noqa: E402
from platepick.forces import ForceMeter  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from reach_push import collisions  # noqa: E402

DEFAULTS = dict(world_psi=0, slot_mr=0, pinky_unload=0.0, retreat_radial=0, thumb_seq=0, close_margin=0.0, left_retreat=0.0, left_T=1.0, left_after_up=0, pose_png="", relax_thumb=-1.0, relax=-1.0, back_off=0.0, w_lift_rot=0.02, head=0.0, tm_close=-1.0, pitch=0.0, roll=0.0, psi=60.0, din=12.0, sx=8.0, sy=21.0, sz=118.0, flex=0.15, others=0.9, tm=1.0, tp_open=0.0, tp_close=1.0,
                w_up=0.05, w_head=0.02, back=40.0, push_in=6.0, approach_T=1.5, in_T=1.2, close_T=1.0, up=30.0, up_T=1.5,
                left_away=1, allow=2.0, appr=0.0, mflex=-1.0, nway=8, mu_pad=-1.0, mu_glaze=-1.0, mu_foot=-1.0, mu_table=-1.0, seeds=40, pos_err_x=0.0, pos_err_y=0.0, pos_err_z=0.0)
CAMS = ("grip", "gripside", "front", "top")


class ArmIK:
    """5 arm joints: a point fixed in the palm frame to a target, the palm's y axis (thumb side) along `up`, its
    z axis (toward the fingers) along `fwd` (soft). The hand shape is fixed."""

    def __init__(self, m, meta, side, hand_cmd):
        self.m, self.d = m, mujoco.MjData(m)
        self.arm = [m.joint(j).id for j in meta["arm"][side]]
        self.hj = [m.joint(j).id for j in meta["hand"][side]]
        self.C = np.array(meta["coupling"][side])
        self.palm = m.body(f"{side}_base_link").id
        self.lo = np.array([m.jnt_range[j][0] for j in self.arm])
        self.hi = np.array([m.jnt_range[j][1] for j in self.arm])
        self.cmd = np.asarray(hand_cmd, float)

    def set(self, d, q, cmd=None):
        for j, v in zip(self.arm, q):
            d.qpos[self.m.jnt_qposadr[j]] = v
        c = self.cmd if cmd is None else cmd
        for i, j in enumerate(self.hj):
            d.qpos[self.m.jnt_qposadr[j]] = min(self.C[i] @ c, self.m.jnt_range[j][1])

    def frame(self, d, loc):
        R = d.xmat[self.palm].reshape(3, 3)
        return d.xpos[self.palm] + R @ loc, R

    def solve(self, loc, P, up, fwd, seeds, base, w_up, w_head):
        out = []
        for q0 in seeds:
            self.d.qpos[:] = base

            def res(q):
                self.set(self.d, q)
                mujoco.mj_kinematics(self.m, self.d)
                p, R = self.frame(self.d, loc)
                return np.concatenate([p - P, w_up * (R[:, 1] - up), w_head * (R[:, 2] - fwd)])
            r = least_squares(res, np.clip(q0, self.lo + 1e-4, self.hi - 1e-4), bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-12)
            self.set(self.d, r.x)
            mujoco.mj_kinematics(self.m, self.d)
            p, R = self.frame(self.d, loc)
            out.append(dict(q=r.x.copy(), pos_mm=float(np.linalg.norm(p - P) * 1000),
                            up_deg=math.degrees(math.acos(np.clip(R[:, 1] @ up, -1, 1))),
                            head_deg=math.degrees(math.acos(np.clip(R[:, 2] @ fwd, -1, 1)))))
        return sorted(out, key=lambda s: s["pos_mm"] + 0.2 * s["up_deg"] + 0.05 * s["head_deg"])


def run(state, par, frames=False, video=None, title="", plan_only=False, record=None):
    p = dict(DEFAULTS, **par)
    st = np.load(state, allow_pickle=True)
    pp = json.loads(str(st["par"]))
    pick_mu = lambda k, v: p[k] if p[k] > 0 else v  # noqa: E731   (-1: the push state's own value)
    fr = Friction(glaze_glaze=pick_mu("mu_glaze", pp["glaze_glaze"]), foot_glaze=pick_mu("mu_foot", pp["foot_glaze"]),
                  pad_glaze=pick_mu("mu_pad", pp["pad_glaze"]), table=pick_mu("mu_table", pp["table_mu"]))
    n = int(pp["n"])
    sc = RobotScene(n=n, table=pp["table"], xy=(pp["x"], pp["y"]), pelvis=pp["pelvis"], fr=fr, table_right=pp.get("table_right", 0.0))
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    d.qpos[:], d.qvel[:], d.ctrl[:], d.time = st["qpos"], st["qvel"], st["ctrl"], float(st["time"])
    mujoco.mj_forward(m, d)
    pb = [m.body(f"plate{k}").id for k in range(n)]
    act = {m.actuator(i).name: i for i in range(m.nu)}
    arm_act = [act[f"a_{j}"] for j in meta["arm"]["right"]]
    hand_act = [act[f"a_{j}"] for j in meta["hand"]["right"]]
    right_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("right_")}
    fm = ForceMeter(m, "right", pb[n - 1], pb[n - 2] if n > 1 else None, hands=("right", "left"))
    fm.R_edge = sc.spec.R                 # thumb contact "in from the edge"
    fm.keep_trace = bool(os.environ.get("DEXPICK_FTRACE"))
    phase_t = {}
    left_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")}
    C = np.array(meta["coupling"]["right"])
    cmd_hi = np.array([min(m.jnt_range[m.joint(meta["hand"]["right"][i]).id][1] / C[i, k]
                           for i in range(C.shape[0]) if C[i, k] > 0) for k in range(6)])
    fl = p["flex"] * cmd_hi[2]
    ot = p["others"] * cmd_hi[4]
    mf = fl if p["mflex"] < 0 else p["mflex"] * cmd_hi[3]          # the middle (under the rim) may curl differently
    cmd_open = np.array([p["tm"], p["tp_open"], fl, mf, ot, ot])          # thumb rot, thumb flex, index, middle, ring, pinky
    if p["slot_mr"]:
        # rim between middle (on top) and ring (under it), little finger curled away; use with sy ~0.
        # The hand rides ~20 mm higher than the index-middle slot (for 2-plate stacks).
        cmd_open = np.array([p["tm"], p["tp_open"], fl, fl, p["mflex"] * cmd_hi[4] if p["mflex"] >= 0 else fl, ot])
        if p["slot_mr"] == 2:
            # (lone plate): the rim between RING (on top, index shape) and LITTLE finger (under it, the mflex
            # curl); use with sy ~ -20 (ring -10 / little -29 mm). Only the little finger hangs below the rim.
            cmd_open = np.array([p["tm"], p["tp_open"], fl, fl, fl, p["mflex"] * cmd_hi[5] if p["mflex"] >= 0 else fl])
    cmd_close = cmd_open.copy()
    cmd_close[1] = p["tp_close"]
    if p["tm_close"] >= 0:                # the thumb rotation turns down to clamp
        cmd_close[0] = p["tm_close"]
    ik = ArmIK(m, meta, "right", cmd_open)

    # ---- target from the plate pose (camera estimate; pos_err = an error in it)
    pt, Rt = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()
    pt_est = pt + np.array([p["pos_err_x"], p["pos_err_y"], p["pos_err_z"]]) / 1000
    if p["world_psi"]:
        # plan with the plate's yaw removed: the plate is round and the camera gives no yaw
        gam = math.atan2(Rt[1, 0], Rt[0, 0])
        Rt = Rt @ np.array([[math.cos(gam), math.sin(gam), 0], [-math.sin(gam), math.cos(gam), 0], [0, 0, 1.0]])
    psi = math.radians(p["psi"])
    e = np.array([-math.sin(psi), -math.cos(psi), 0.0])       # outward at the grip, plate frame
    z = np.array([0, 0, 1.0])
    r_in = spec.R - p["din"] / 1000
    mid = np.array([0, 0, spec.z_under(r_in) + 0.5 * spec.rim_t]) + r_in * e     # rim mid-thickness
    T = pt_est + Rt @ mid
    # the rim is a cone, rising 21 deg toward the edge: the hand lines up with the rim's own surface there
    ra = spec.rim_ang - math.radians(p["pitch"])                       # pitch: fingers less steep than the rim
    up_w = Rt @ (-math.sin(ra) * e + math.cos(ra) * z)                  # normal of the rim surface (top side)
    fwd_w = Rt @ (-math.cos(ra) * e - math.sin(ra) * z)                 # inward along the rim, down its slope
    e_w = Rt @ e
    # approach direction, in the rim's own plane: appr 0 = straight in down the rim slope, 90 = along the rim from the
    # robot's side (sliding the rim sideways into the slot), -90 = from the far side
    ap = math.radians(p["appr"])
    dir_in = math.cos(ap) * fwd_w + math.sin(ap) * np.cross(fwd_w, up_w)
    # head: turn the fingers from pointing at the plate centre toward pointing along the rim (degrees)
    hd = math.radians(p["head"])
    fwd_w = math.cos(hd) * fwd_w + math.sin(hd) * np.cross(up_w, fwd_w)
    # roll: turn the hand about its finger direction (positive = thumb side tips toward the robot's left)
    ro = math.radians(p["roll"])
    side_w = np.cross(fwd_w, up_w)
    up_w = math.cos(ro) * up_w + math.sin(ro) * side_w
    loc = np.array([p["sx"], p["sy"], p["sz"]]) / 1000
    base = d.qpos.copy()                  # the planner's world: the top plate where the camera says it is
    a_top = m.jnt_qposadr[m.joint(f"plate{n - 1}_free").id]
    base[a_top:a_top + 3] += np.array([p["pos_err_x"], p["pos_err_y"], p["pos_err_z"]]) / 1000
    for i, j in enumerate(ik.hj):
        base[m.jnt_qposadr[j]] = min(C[i] @ cmd_open, m.jnt_range[j][1])
    rng = np.random.default_rng(0)
    known = [np.array(meta["ready"]["right"]), np.array([-0.2, 0.2, -0.1, 0.27, 0.6]), np.array([-0.5, -0.3, 0.3, 0.5, 1.5])]
    seeds = known + [rng.uniform(ik.lo, ik.hi) for _ in range(int(p["seeds"]))]
    sols = ik.solve(loc, T, up_w, fwd_w, seeds, base, p["w_up"], p["w_head"])

    def clear(q, cmd=None, allow_top=0.5):
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, q, cmd)
        mujoco.mj_forward(m, d2)
        return {k: v for k, v in collisions(m, d2).items() if k.startswith("right_") and (not k.endswith(f"plate{n - 1}") or v < -allow_top)}

    def shift(q, s, w_rot=0.2):
        """Arm joints that move the palm by s (m); w_rot: how strongly its orientation is kept."""
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, q)
        mujoco.mj_kinematics(m, d2)
        P0, R0 = d2.xpos[ik.palm].copy(), d2.xmat[ik.palm].reshape(3, 3).copy()

        def r(qq):
            ik.set(d2, qq)
            mujoco.mj_kinematics(m, d2)
            return np.concatenate([d2.xpos[ik.palm] - P0 - s, w_rot * (d2.xmat[ik.palm].reshape(3, 3) - R0).ravel()])
        qq = least_squares(r, q, bounds=(ik.lo, ik.hi), xtol=1e-10).x
        return qq, float(np.linalg.norm(r(qq)[:3]) * 1000)

    pick, why = None, ""
    for s in sols[:12]:
        if s["pos_mm"] > 2.0 or s["up_deg"] > 25:
            continue
        q_in = s["q"]
        # straight line in the rim plane from `back` mm outside to the grip pose
        sh = [shift(q_in, -(1 - u) * p["back"] / 1000 * dir_in) for u in np.linspace(0, 1, int(p["nway"]) + 1)[:-1]]
        way = [w[0] for w in sh] + [q_in]
        q_pre, e1 = way[0], max(w[1] for w in sh)
        bad = clear(q_pre, allow_top=0.0)
        for qw in way[1:-1]:
            bad.update(clear(qw, allow_top=1.0))
        bad.update(clear(q_in, allow_top=3.0))
        if bad and min(bad.values()) < -p["allow"]:
            k0 = min(bad, key=bad.get)
            why = why or f"{k0} {bad[k0]:.1f} mm"
            continue
        # from where the arm is now to the start of the line: joint interpolation must not touch the top plate;
        # if it does, go via a point further out and higher
        q_from = np.array([d.ctrl[a_] for a_ in arm_act])
        def path_hits(qa, qb, k=12):
            for u in np.linspace(0.1, 1.0, k):
                h = clear(qa + (qb - qa) * u, allow_top=0.0)
                if h:
                    return h
            return {}
        legs, hits = None, path_hits(q_from, q_pre)
        if not hits:
            legs = [q_pre]
        else:
            for off in (-0.06 * dir_in + 0.04 * z, -0.08 * dir_in, 0.06 * z, -0.12 * dir_in + 0.06 * z, -0.06 * e_w + 0.06 * z):
                q_far = shift(q_pre, off)[0]
                h1 = path_hits(q_from, q_far) or path_hits(q_far, q_pre, 8)
                if not h1:
                    legs = [q_far, q_pre]
                    break
                hits = h1
        if legs is None:
            why = why or f"path to the start of the slide touches: {hits}"
            continue
        pick = (s, q_in, q_pre, e1, way, legs)
        break
    best = sols[0]
    plan = dict(best_pos_mm=best["pos_mm"], best_up_deg=best["up_deg"], best_head_deg=best["head_deg"])
    if pick is None:
        return dict(par=p, plan=plan, error=f"no grip pose: {why or 'not reachable'}")
    s, q_in, q_pre, e1, way, legs = pick
    plan.update(pos_mm=s["pos_mm"], up_deg=s["up_deg"], head_deg=s["head_deg"], q_in=[round(float(v), 3) for v in q_in], q_pre=[round(float(v), 4) for v in q_pre], legs=[[round(float(v), 4) for v in ql] for ql in legs])
    # clearance of index (should be above the rim) and middle (below) at the grip pose, before any physics
    d2 = mujoco.MjData(m)
    d2.qpos[:] = base
    ik.set(d2, q_in)
    mujoco.mj_forward(m, d2)
    top_g = [g for g in range(m.ngeom) if m.geom_bodyid[g] == pb[n - 1] and m.geom_contype[g]]
    ft = np.zeros(6)
    for fing in ("index", "middle"):
        gs = [g for g in right_geoms if fing in m.body(m.geom_bodyid[g]).name and m.geom_contype[g]]
        plan[f"{fing}_clear_mm"] = round(min(mujoco.mj_geomDistance(m, d2, g1, g2, 0.03, ft) for g1 in gs for g2 in top_g) * 1000, 1)
    palm_in = d2.xpos[ik.palm].copy()
    q_push, _ = shift(q_in, p["push_in"] / 1000 * dir_in)        # a little further in: the rim wedges in the V
    q_from = np.array([d.ctrl[a_] for a_ in arm_act])
    via = {}
    for u in np.linspace(0.1, 1.0, 10):
        via.update(clear(q_from + (q_pre - q_from) * u))
    plan["way_to_pre"] = {k: round(v, 1) for k, v in via.items() if v < -1.0}
    if p["pose_png"]:
        # picture of the planned "rim in the slot" pose (kinematics only, nothing simulated): along the rim and from
        # outside, with the plate made see-through so a finger inside it shows
        import cv2
        dq = mujoco.MjData(m)
        dq.qpos[:] = d.qpos
        ik.set(dq, q_in, cmd_open)
        mujoco.mj_forward(m, dq)
        rgba0 = m.geom_rgba.copy()
        for g_ in range(m.ngeom):
            if m.geom_bodyid[g_] in pb or m.geom(g_).name == "table":
                m.geom_rgba[g_, 3] = 0.45
        # hand links that penetrate the table / a plate in this pose drawn red; the deepest penetration per link is printed
        hit = {}
        for k_, v_ in collisions(m, dq).items():
            if k_.startswith("right_") and v_ < -0.5:
                hit[k_.split("|")[0]] = min(hit.get(k_.split("|")[0], 0.0), v_)
        for g_ in range(m.ngeom):
            if m.body(m.geom_bodyid[g_]).name in hit:
                m.geom_rgba[g_] = (0.9, 0.1, 0.1, 1.0)
        plan["pose_hits_mm"] = {k_: round(v_, 1) for k_, v_ in hit.items()}
        rr = mujoco.Renderer(m, 600, 800)
        ims = []
        tang_ = np.cross(z, e_w)
        # free cameras aimed at the grip point (the fixed "gripside" camera is not placed by cam_pos edits
        # when it is attached to a body, and showed the other hand): from outside the rim, along the rim, from the front
        for v_, lab_ in ((e_w + 0.15 * z, "from outside the rim"), (tang_ + 0.1 * z, "along the rim"),
                         (np.array([1.0, 0.0, 0.25]), "from the front")):
            v_ = v_ / np.linalg.norm(v_)
            cam_ = mujoco.MjvCamera()
            cam_.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam_.lookat[:] = T
            cam_.distance = 0.32
            cam_.azimuth = math.degrees(math.atan2(-v_[1], -v_[0]))       # MuJoCo: the direction the camera looks
            cam_.elevation = -math.degrees(math.asin(v_[2]))
            rr.update_scene(dq, camera=cam_)
            im_ = cv2.cvtColor(rr.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im_, lab_, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            ims.append(im_)
        cv2.imwrite(p["pose_png"], np.hstack(ims))
        m.geom_rgba[:] = rgba0
    if plan_only:
        return dict(par=p, plan=plan)

    # ---- cameras at the grip
    def aim(cam, eye, at=None):
        at = T if at is None else at
        cid = m.camera(cam).id
        zc = (eye - at) / np.linalg.norm(eye - at)
        xc = np.cross(z, zc)
        xc /= np.linalg.norm(xc)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.c_[xc, np.cross(zc, xc), zc].ravel())
        m.cam_pos[cid], m.cam_quat[cid] = eye, q
    tang = np.cross(z, e_w)
    aim("grip", T + 0.2 * e_w - 0.05 * z + 0.12 * np.array([1.0, 0, 0]))       # from outside, below, the far side
    aim("gripside", T + 0.22 * tang + 0.04 * z + 0.03 * e_w)
    thumbcam = os.environ.get("DEXPICK_THUMBCAM", "0") == "1"
    if thumbcam:                       # close-up of the right thumb on the rim
        # seen from above and a little outside the rim; top plate tinted blue, right thumb orange, the other right-hand
        # parts see-through, so where the thumb lands relative to the rim edge is visible
        at_th = T - 0.010 * e_w - 0.006 * z
        aim("top", at_th + 0.17 * z + 0.08 * e_w, at=at_th)
        for g_ in range(m.ngeom):
            nm_ = m.body(m.geom_bodyid[g_]).name
            if m.geom_bodyid[g_] == pb[n - 1]:
                m.geom_rgba[g_] = (0.45, 0.65, 0.95, 1.0)
            elif nm_.startswith("right_thumb"):
                m.geom_rgba[g_] = (0.95, 0.55, 0.2, 1.0)
            elif nm_.startswith("right_"):
                m.geom_rgba[g_, 3] = 0.3
    renderer, shots = (mujoco.Renderer(m, 720, 1280), []) if frames else (None, [])

    snaps = []

    def shoot(label):
        # a snapshot: every joint by name, every plate, arm target, hand command
        snaps.append(dict(label=label, joints={m.joint(j).name: float(d.qpos[m.jnt_qposadr[j]]) for j in range(m.njnt) if m.jnt_type[j] in (2, 3)},
                          plates=[[float(v) for v in d.xpos[b]] + [float(v) for v in d.xquat[b]] for b in pb],
                          q_arm=[float(d.ctrl[a_]) for a_ in arm_act], cmd=[float(v) for v in vs.get("cmd", np.zeros(6))]))
        if renderer is None:
            return
        ims = []
        for cam in CAMS:
            renderer.update_scene(d, camera=cam)
            ims.append(renderer.render().copy())
        shots.append((label, ims))

    def contact_r():
        """Right hand on the top plate: force-weighted radius (mm, plate frame) of each finger's contacts, top / under."""
        acc, w = {}, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            a_, b_ = c.geom1 in right_geoms, c.geom2 in right_geoms
            if a_ == b_ or m.geom_bodyid[c.geom1 if b_ else c.geom2] != pb[n - 1]:
                continue
            mujoco.mj_contactForce(m, d, i, w)
            f_ = float(np.linalg.norm(w[:3]))
            part = m.body(m.geom_bodyid[c.geom2 if b_ else c.geom1]).name.split("_")[1]
            lp = d.xmat[pb[n - 1]].reshape(3, 3).T @ (c.pos - d.xpos[pb[n - 1]])
            rr = min(math.hypot(lp[0], lp[1]), spec.R)
            face = "^" if abs(lp[2] - spec.z_top(rr)) < abs(lp[2] - spec.z_under(rr)) else "v"
            k_ = part + face
            fsum, rsum = acc.get(k_, (0.0, 0.0))
            acc[k_] = (fsum + f_, rsum + f_ * rr)
        return {k: round(1000 * r_ / f_, 1) for k, (f_, r_) in acc.items() if f_ > 0.3}

    def forces():
        out, w = {}, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            for hand, gs in (("right", right_geoms), ("left", left_geoms)):
                a_, b_ = c.geom1 in gs, c.geom2 in gs
                if a_ == b_:
                    continue
                ob = m.body(m.geom_bodyid[c.geom1 if b_ else c.geom2]).name
                kind = "top" if ob == f"plate{n - 1}" else "below" if ob.startswith("plate") else "table" if ob == "world" else None
                if kind is None:
                    continue
                mujoco.mj_contactForce(m, d, i, w)
                part = m.body(m.geom_bodyid[c.geom2 if b_ else c.geom1]).name.replace(hand + "_", "").replace("_link", "")
                if kind == "top":              # which face of the top plate: ^ = on its top surface, v = under it
                    lp = d.xmat[pb[n - 1]].reshape(3, 3).T @ (c.pos - d.xpos[pb[n - 1]])
                    rr = min(math.hypot(lp[0], lp[1]), spec.R)
                    part += "^" if abs(lp[2] - spec.z_top(rr)) < abs(lp[2] - spec.z_under(rr)) else "v"
                out[(hand, kind, part)] = out.get((hand, kind, part), 0.0) + float(np.linalg.norm(w[:3]))
        return out

    vid, vs = None, dict(next=d.time, phase="")
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))

    def grab():
        import cv2
        if d.time < vs["next"]:
            return
        vs["next"] += 0.01 if (thumbcam and vs["phase"] == "thumb locks") else 0.04      # 4x slow motion while the thumb closes
        ims = []
        for cam, lab in zip(CAMS, ("close-up of the grip, from outside the rim", "close-up of the grip, along the rim", "from the front",
                                   "close-up of the RIGHT THUMB (4x slow while it closes)" if thumbcam else "from above (robot at the top)")):
            vr.update_scene(d, camera=cam)
            im = cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im, lab, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            ims.append(im)
        f = forces()
        g = lambda kind, word="": sum(v for k, v in f.items() if k[0] == "right" and k[1] == kind and word in k[2])   # noqa: E731
        bar = np.full((64, 1280, 3), 20, np.uint8)
        cv2.putText(bar, title[:150], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        low = g("below") + g("table")
        lf = sum(v for k, v in f.items() if k[0] == "left" and k[1] == "top")
        txt = (f"t {d.time:4.1f} s  {vs['phase']:<12s} FRONT-LEFT hand (robot R): index {g('top', 'index'):4.1f} N  middle {g('top', 'middle'):4.1f} N  thumb {g('top', 'thumb'):4.1f} N  "
               + (f"thumb {fm.th['right']['in_end']:.0f} mm in from edge  " if thumbcam and fm.th["right"]["in_end"] is not None and fm.th["right"]["F"] > 0.5 else "")
               + f"on plates below/table {low:4.1f} N  left finger {lf:4.1f} N  tilt {math.degrees(math.acos(min(1.0, d.xmat[pb[n - 1]][8]))):4.1f} deg")
        cv2.putText(bar, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 255) if low > 0.5 else (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    log = []
    # --record: the scripted commands at 30 Hz and a snapshot at the start of each phase (as grip_left.py --record)
    l_all = [act[f"a_{j}"] for j in meta["arm"]["left"]]
    hinge = [j for j in range(m.njnt) if m.jnt_type[j] in (2, 3)]
    snap_state = lambda: dict(joints={m.joint(j).name: float(d.qpos[m.jnt_qposadr[j]]) for j in hinge},  # noqa: E731
                              plates=[[float(v) for v in d.qpos[m.jnt_qposadr[m.joint(f"plate{k}_free").id]:][:7]] for k in range(n)],
                              ctrl={m.actuator(i).name: float(d.ctrl[i]) for i in range(m.nu)}, time=float(d.time))
    rec = dict(dt=1 / 30, start=snap_state(), steps=[], snaps=[], next=d.time) if record else None

    def move(qa, qb, ca, cb, T_, phase, left=None):
        N = max(1, int(T_ / dt))
        vs["phase"] = phase
        phase_t.setdefault(phase, round(float(d.time), 3))
        for k in range(N):
            u = (k + 1) / N
            u = 3 * u * u - 2 * u ** 3
            d.ctrl[arm_act] = qa + (qb - qa) * u
            c = ca + (cb - ca) * u
            if p["close_margin"] > 0 and phase not in ("approach", "slide in"):
                # force limit on the porcelain: each closing finger is commanded at most close_margin (rad)
                # past its measured angle (the Revo2 reports finger positions), so it presses with ~kp x margin
                for k_, jj in ((0, ik.hj[0]), (1, ik.hj[5]), (2, ik.hj[1]), (3, ik.hj[2])) + (((4, ik.hj[3]),) if p["slot_mr"] == 1 else ((5, ik.hj[4]),) if p["slot_mr"] == 2 else ()):
                    sg = np.sign(cmd_close[k_] - cmd_open[k_])
                    q_ = d.qpos[m.jnt_qposadr[jj]]
                    if sg and sg * (c[k_] - q_) > p["close_margin"]:
                        c[k_] = q_ + sg * p["close_margin"]
            for i, j in enumerate(ik.hj):
                d.ctrl[hand_act[i]] = min(C[i] @ c, m.jnt_range[j][1])
            if left is not None:
                d.ctrl[left[0]] = left[1] + (left[2] - left[1]) * u
            if rec is not None:
                if k == 0 and (not rec["snaps"] or rec["snaps"][-1]["label"] != phase):
                    rec["snaps"].append(dict(snap_state(), label=phase, q_arm=[float(d.ctrl[a_]) for a_ in arm_act], cmd=[float(v) for v in c]))
                if d.time >= rec["next"] - 1e-9:
                    rec["next"] += rec["dt"]
                    rec["steps"].append(dict(phase=phase, q_arm=[round(float(d.ctrl[a_]), 5) for a_ in arm_act], cmd=[round(float(v), 5) for v in c],
                                             q_left=[round(float(d.ctrl[a_]), 5) for a_ in l_all]))
            mujoco.mj_step(m, d)
            fm.update(d)
            if vid is not None:
                grab()
            if k % 20 == 0:
                log.append(dict(phase=phase, f=forces()))
        vs["cmd"] = cb

    p_start = [d.xpos[b].copy() for b in pb]
    shoot("after the push")
    cmd0 = np.zeros(6)
    qa = q_from
    for k, ql in enumerate(legs):
        move(qa, ql, cmd0 if k == 0 else cmd_open, cmd_open, p["approach_T"] / len(legs), "approach")
        qa = ql
    shoot("beside the rim")
    for qa, qb in zip(way[:-1], way[1:]):
        move(qa, qb, cmd_open, cmd_open, p["in_T"] / (len(way) - 1), "slide in")
    d3 = mujoco.MjData(m)
    d3.qpos[:] = d.qpos
    ik.set(d3, q_in, cmd_open)
    mujoco.mj_kinematics(m, d3)
    plan["track_err_mm"] = round(float(np.linalg.norm(d.xpos[ik.palm] - d3.xpos[ik.palm])) * 1000, 1)
    plan["plate_moved_by_slide_mm"] = round(float(np.linalg.norm(d.xpos[pb[n - 1]] - p_start[n - 1])) * 1000, 1)
    move(q_in, q_push, cmd_open, cmd_open, 0.5, "slide in")
    shoot("rim between index and middle")
    if p["pinky_unload"] > 0:
        # before the thumb locks, the far-rim pinky (robot left) eases pinky_unload mm straight out from the plate
        # centre: away from the original spot the thumb's clamp pulled the plate against it and wedged it (300-1500 N)
        la_ = [act[f"a_{j}"] for j in meta["arm"]["left"]]
        ldof = [m.jnt_dofadr[m.joint(j).id] for j in meta["arm"]["left"]]
        pk = m.body("left_pinky_distal_link").id
        jp_ = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, jp_, None, d.xpos[pk], pk)
        rv = d.xpos[pk] - d.xpos[pb[n - 1]]
        rv[2] = 0.0
        dx = np.r_[(rv / max(np.linalg.norm(rv), 1e-6))[:2], 0.0] * p["pinky_unload"] / 1000
        ql0 = np.array([d.ctrl[a_] for a_ in la_])
        move(q_push, q_push, cmd_open, cmd_open, 0.3, "slide in", left=(la_, ql0, ql0 + np.linalg.lstsq(jp_[:, ldof], dx, rcond=None)[0]))
    if p["thumb_seq"] in (1, 2):
        # against the thumb sliding: close the thumb's two joints one after the other instead of
        # together. 1 = curl (flex) first in the air, then rotate down onto the rim; 2 = rotate down first, then curl
        mid = cmd_open.copy()
        mid[1 if p["thumb_seq"] == 1 else 0] = cmd_close[1 if p["thumb_seq"] == 1 else 0]
        move(q_push, q_push, cmd_open, mid, p["close_T"] * 0.5, "thumb locks")
        move(q_push, q_push, mid, cmd_close, p["close_T"] * 0.5, "thumb locks")
    else:
        move(q_push, q_push, cmd_open, cmd_close, p["close_T"], "thumb locks")
    move(q_push, q_push, cmd_close, cmd_close, 0.3, "thumb locks")
    if p["relax"] >= 0:
        # index and middle hold where the rim stopped them instead of pushing at the motors' limit: target = measured
        # angle + relax (rad) in the direction they were pushing (the real hand reports its finger positions)
        cmd_hold = cmd_close.copy()
        for k_, jj in ((2, ik.hj[1]), (3, ik.hj[2])) + (((4, ik.hj[3]),) if p["slot_mr"] == 1 else ((5, ik.hj[4]),) if p["slot_mr"] == 2 else ()) + (((0, ik.hj[0]), (1, ik.hj[5])) if p["relax_thumb"] >= 0 else ()):
            q_ = d.qpos[m.jnt_qposadr[jj]]
            cmd_hold[k_] = q_ + (p["relax_thumb"] if k_ <= 1 else p["relax"]) * np.sign(cmd_close[k_] - q_)
        move(q_push, q_push, cmd_close, cmd_hold, 0.3, "thumb locks")
        cmd_close = cmd_hold
    if p["back_off"] > 0:
        # with the thumb clamped, the arm backs out a little along the slide so the rim is not jammed in the V
        q_b, _ = shift(q_push, -p["back_off"] / 1000 * dir_in)
        move(q_push, q_b, cmd_close, cmd_close, 0.5, "thumb locks")
        q_push = q_b
    shoot("thumb locked")
    P_g, R_g = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()

    def rim_left():
        """Lowest point of the top plate's left rim (robot's left, +-40 deg around +y), world z: the room left for
        the left hand."""
        Pp, Rp = d.xpos[pb[n - 1]], d.xmat[pb[n - 1]].reshape(3, 3)
        a = np.radians(np.linspace(50, 130, 9))
        loc = np.c_[spec.R * np.cos(a), spec.R * np.sin(a), np.full(9, spec.H - spec.rim_t)]
        return float((loc @ Rp.T + Pp)[:, 2].min())
    rimL_g = rim_left()
    faces_g = forces()
    palm_g = (R_g.T @ (d.xpos[ik.palm] - P_g)).copy()
    l_act = [act[f"a_{j}"] for j in meta["arm"]["left"]]
    def left_away(q_hold):
        # the left finger leaves the far rim (left_T scales its speed: the plate settles into the right grip as it goes)
        ql0 = np.array([d.ctrl[a_] for a_ in l_act])
        ql1 = ql0.copy()
        ql1[0] -= 0.35
        if p["left_retreat"] > 0:
            # the pinky end backs straight out from the rim (+y) and up by left_retreat mm, on the pinky's Jacobian,
            # so that it does not press into the rim
            ldof = [m.jnt_dofadr[m.joint(j).id] for j in meta["arm"]["left"]]
            pk = m.body("left_pinky_distal_link").id
            jp_ = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jp_, None, d.xpos[pk], pk)
            J = jp_[:, ldof]
            dx = np.array([0.0, 1.0, 0.7]) * p["left_retreat"] / 1000
            if p["retreat_radial"]:
                # straight out from the plate centre (and up), whatever the stack position - the fixed world
                # +y direction (tuned at x 0.36 / y 0.06) pressed the pinky into the rim elsewhere (300-1500 N, stackpos)
                rv = d.xpos[pk] - d.xpos[pb[n - 1]]
                rv[2] = 0.0
                dx = np.r_[(rv / max(np.linalg.norm(rv), 1e-6))[:2], 0.7] * p["left_retreat"] / 1000
            ql1 = ql0 + np.linalg.lstsq(J, dx, rcond=None)[0]
        move(q_hold, q_hold, cmd_close, cmd_close, 0.6 * p["left_T"], "left away", left=(l_act, ql0, ql1))
        move(q_hold, q_hold, cmd_close, cmd_close, 0.8, "left away", left=(l_act, ql1, np.array(meta["ready"]["left"])))
        shoot("left finger away")
    if p["left_away"] and not p["left_after_up"]:
        left_away(q_push)
    q_up, e_up = shift(q_push, p["up"] / 1000 * z, w_rot=p["w_lift_rot"])     # the hand may turn a little while lifting
    if p["left_away"] and p["left_after_up"]:
        # lift a little first (left_after_up mm) with the left finger still on the far rim, then let it go
        q_m, _ = shift(q_push, p["left_after_up"] / 1000 * z, w_rot=p["w_lift_rot"])
        move(q_push, q_m, cmd_close, cmd_close, p["up_T"] * p["left_after_up"] / max(p["up"], 1.0), "lift")
        left_away(q_m)
        q_push = q_m
    move(q_push, q_up, cmd_close, cmd_close, p["up_T"], "lift")
    move(q_up, q_up, cmd_close, cmd_close, 1.0, "hold")
    shoot("lifted")
    if vid is not None:
        vid.release()
    if rec is not None:
        rec.pop("next")
        rec.update(par=pp, right_par=p, arm_joints=list(meta["arm"]["right"]), left_arm_joints=list(meta["arm"]["left"]),
                   hand_joints=list(meta["hand"]["right"]), source=os.path.basename(state), up_mm=p["up"])
        json.dump(rec, open(record, "w"))
    # ---- results
    Pe, Re = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()
    slip = float(np.linalg.norm(Re.T @ (d.xpos[ik.palm] - Pe) - palm_g)) * 1000
    f_end = forces()
    peak, by_phase = {}, {}
    for r_ in log:
        for k, v in r_["f"].items():
            peak[k] = max(peak.get(k, 0.0), v)
            if k[0] == "right" and k[1] == "top":
                ph = by_phase.setdefault(r_["phase"], {})
                ph[k[2]] = round(max(ph.get(k[2], 0.0), v), 1)
    # room for the left hand: height of the top plate's left rim underside above whatever is below it
    gaps = []
    for lat in (-0.03, 0.0, 0.03):
        for rho in np.arange(spec.R - 0.04, spec.R + 0.001, 0.004):
            q = Pe + Re @ np.array([lat, rho, spec.z_under(min(math.hypot(lat, rho), spec.R))])
            below = pp["table"]
            for k in range(n - 1):
                lk = d.xmat[pb[k]].reshape(3, 3).T @ (q - d.xpos[pb[k]])
                if math.hypot(lk[0], lk[1]) <= spec.R:
                    below = max(below, d.xpos[pb[k]][2] + spec.z_top(math.hypot(lk[0], lk[1])))
            gaps.append((q[2] - below) * 1000)
    out = dict(par=p, push_par=pp, plan=plan,
               held=bool(sum(v for k, v in f_end.items() if k[0] == "right" and k[1] == "top") > 1.0),
               plate_rose_mm=float(Pe[2] - P_g[2]) * 1000, tilt_deg=math.degrees(math.acos(min(1.0, Re[2, 2]))),
               slip_in_hand_mm=slip, left_rim_gap_mm=float(min(gaps)),
               plates_below_moved_mm=max([0.0] + [float(np.linalg.norm(d.xpos[b][:2] - p_start[k][:2])) * 1000 for k, b in enumerate(pb[:-1])]),
               right_on_lower_or_table_N=max([v for k, v in peak.items() if k[0] == "right" and k[1] != "top"] + [0.0]),
               right_on_top_by_phase_N=by_phase,
               end_N={" ".join(k): round(v, 1) for k, v in sorted(f_end.items(), key=lambda kv: -kv[1])}, contact_r_mm=contact_r(),
               lift_err_mm=e_up, rimL_rise_mm=(rim_left() - rimL_g) * 1000,
               faces_at_grip={k[2]: round(v, 1) for k, v in faces_g.items() if k[0] == "right" and k[1] == "top"}, force=fm.summary(), phase_start_s=phase_t)
    if fm.keep_trace:
        out["force_trace"] = fm.trace
    if shots:
        out["_shots"] = shots
    out["_snaps"] = snaps
    out["_final"] = dict(qpos=d.qpos.copy(), qvel=d.qvel.copy(), ctrl=d.ctrl.copy(), time=d.time)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--sheet", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--save", default=None)
    ap.add_argument("--plan", action="store_true")
    ap.add_argument("--record", default=None, help="write the scripted commands (30 Hz) and phase snapshots to this .json")
    ap.add_argument("--snaps", default=None, help="write the phase snapshots (RL demo starts) to this .json")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    over = {k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None}
    t0 = time.time()
    res = run(a.state, over, frames=bool(a.sheet), video=a.video, title=a.title, plan_only=a.plan, record=a.record)
    if "error" in res:
        print("ERROR:", res["error"], json.dumps(res.get("plan", {})))
        return
    if a.sheet:
        sheet(res, a.sheet, f"MuJoCo, right hand: rim between index and middle, thumb locks; psi {res['par']['psi']:.0f}: held {res['held']}, "
                            f"plate rose {res['plate_rose_mm']:.0f} mm, tilt {res['tilt_deg']:.0f} deg, slip {res['slip_in_hand_mm']:.1f} mm, "
                            f"gap under the left rim {res['left_rim_gap_mm']:.0f} mm")
    res.pop("_shots", None)
    sn = res.pop("_snaps", None)
    if a.snaps and sn:
        json.dump(dict(source=os.path.basename(a.state), par=json.loads(open(os.path.splitext(a.state)[0] + ".state.json").read())["par"],
                       grip=res["par"], held=res["held"], rimL_rise_mm=res["rimL_rise_mm"], snaps=sn), open(a.snaps, "w"))
    fin = res.pop("_final", None)
    if a.save and fin:
        np.savez(a.save, **fin, par=json.dumps(res["push_par"]), grip=json.dumps(res["par"]))
    res.pop("push_par", None)
    print(json.dumps(res, indent=1))
    print(f"wall time {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
