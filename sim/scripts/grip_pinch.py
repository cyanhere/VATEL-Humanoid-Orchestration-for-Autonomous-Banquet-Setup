"""Stage 3, pinch grip: after the push the right hand pinches the overhanging side (thumb on top of the rim, middle
under it) and lifts slightly.

    python scripts/grip_pinch.py --state data/results/robot/state_push_best_n2_t0.747.npz --sheet out.png
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

from platepick.ik import PAD_NORMAL, PAD_POINT, PadIK  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from push_probe import opening  # noqa: E402
from reach_push import collisions  # noqa: E402

# hand shape of the pinch (found by sampling the hand alone: the middle fingertip 5.6-9 mm straight below the thumb
# pad): thumb rotation, thumb flexion, middle flexion; the solver may move each by +-hand_tol
DEFAULTS = dict(allow_any=0.0, allow_top=2.5, pos_tol=4.0, psi=20.0, dt=8.0, dm=8.0, mbeta=20.0, w_m=0.0, h_tm=1.35, h_tp=0.45, h_mf=0.83, hand_tol=0.08, w_palm=0.05, gap=2.0, others=-1.0, open_t=0.45, open_m=0.0, back=35.0, drop=4.0, squeeze=0.25,
                approach_T=1.5, close_T=1.0, up=30.0, up_T=1.5, tilt_up=0.0, w_n=0.03, seeds=40, left_away=1,
                pos_err_x=0.0, pos_err_y=0.0, pos_err_z=0.0)
CAMS = ("grip", "gripside", "front", "top")       # grip / gripside: aimed at the grip point (set after planning)


class PinchIK:
    """5 arm joints + thumb rotation + thumb flexion + middle flexion: thumb pad and middle pad to two points, their
    pads facing given directions (softly). The other fingers keep a fixed bend."""

    def __init__(self, m, meta, side="right"):
        self.m, self.d = m, mujoco.MjData(m)
        self.arm = [m.joint(j).id for j in meta["arm"][side]]
        hj, C = meta["hand"][side], np.array(meta["coupling"][side])
        self.hj, self.C = [m.joint(j).id for j in hj], C
        self.t_body = m.body(f"{side}_thumb_touch_link").id
        tip = PadIK(m, meta, side, "middle", "tip")          # the END of the middle fingertip 
        self.m_body, self.m_pt, self.m_ax = tip.body, tip.pt, tip.nrm
        self.palm = m.body(f"{side}_base_link").id
        rng = lambda j: m.jnt_range[j]                                              # noqa: E731
        hr = {i: rng(self.hj[i]) for i in range(len(hj))}
        # command ranges of the 6 hand actuators: thumb rot, thumb flex, index, middle, ring, pinky
        cmd_hi = [min(hr[i][1] / C[i, k] for i in range(len(hj)) if C[i, k] > 0) for k in range(6)]
        self.cmd_hi = np.array(cmd_hi)
        self.lo = np.array([rng(j)[0] for j in self.arm] + [0.0, 0.0, 0.0])
        self.hi = np.array([rng(j)[1] for j in self.arm] + [cmd_hi[0], cmd_hi[1], cmd_hi[3]])
        self.others = 0.2

    def cmd(self, x):
        """6 hand commands from the solver vector (thumb rot, thumb flex, middle; index/ring/pinky = others)."""
        o = self.others
        return np.array([x[5], x[6], o, x[7], o, o])

    def set(self, d, x):
        for j, v in zip(self.arm, x[:5]):
            d.qpos[self.m.jnt_qposadr[j]] = v
        c = self.cmd(x)
        for i, j in enumerate(self.hj):
            d.qpos[self.m.jnt_qposadr[j]] = min(self.C[i] @ c, self.m.jnt_range[j][1])

    def pads(self, d):
        """(thumb pad point, its normal), (middle fingertip end, the direction the fingertip points)."""
        R = d.xmat[self.t_body].reshape(3, 3)
        Rm = d.xmat[self.m_body].reshape(3, 3)
        return [(d.xpos[self.t_body] + R @ PAD_POINT, R @ PAD_NORMAL), (d.xpos[self.m_body] + Rm @ self.m_pt, Rm @ self.m_ax)]

    w_m = 0.01
    palm_dir = None
    w_palm = 0.05

    def residual(self, x, Pt, nt, Pm, nm, w_n):
        self.set(self.d, x)
        mujoco.mj_kinematics(self.m, self.d)
        (pt, at), (pm, am) = self.pads(self.d)
        r = [pt - Pt, pm - Pm, w_n * (at - nt), self.w_m * (am - nm)]
        if self.palm_dir is not None:          # the palm faces the plate
            r.append(self.w_palm * (self.d.xmat[self.palm].reshape(3, 3)[:, 0] - self.palm_dir))
        return np.concatenate(r)

    def solve(self, Pt, nt, Pm, nm, seeds, base, w_n=0.03):
        res = []
        for x0 in seeds:
            self.d.qpos[:] = base
            r = least_squares(lambda x: self.residual(x, Pt, nt, Pm, nm, w_n), np.clip(x0, self.lo + 1e-4, self.hi - 1e-4),
                              bounds=(self.lo, self.hi), xtol=1e-10, ftol=1e-12)
            self.set(self.d, r.x)
            mujoco.mj_kinematics(self.m, self.d)
            (pt, at), (pm, am) = self.pads(self.d)
            res.append(dict(x=r.x.copy(), pos_mm=1000 * max(np.linalg.norm(pt - Pt), np.linalg.norm(pm - Pm)),
                            ang_deg=math.degrees(math.acos(np.clip(at @ nt, -1, 1))),          # thumb pad facing
                            mid_ang_deg=math.degrees(math.acos(np.clip(am @ nm, -1, 1)))))     # middle tip direction (loose)
        return sorted(res, key=lambda r: r["pos_mm"] + 0.1 * r["ang_deg"])


def run(state, par, frames=False, video=None, title="", plan_only=False):
    p = dict(DEFAULTS, **par)
    st = np.load(state, allow_pickle=True)
    pp = json.loads(str(st["par"]))
    fr = Friction(glaze_glaze=pp["glaze_glaze"], foot_glaze=pp["foot_glaze"], pad_glaze=pp["pad_glaze"], table=pp["table_mu"])
    n = int(pp["n"])
    sc = RobotScene(n=n, table=pp["table"], xy=(pp["x"], pp["y"]), pelvis=pp["pelvis"], fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    d.qpos[:], d.qvel[:], d.ctrl[:], d.time = st["qpos"], st["qvel"], st["ctrl"], float(st["time"])
    mujoco.mj_forward(m, d)
    pb = [m.body(f"plate{k}").id for k in range(n)]
    act = {m.actuator(i).name: i for i in range(m.nu)}
    arm_act = [act[f"a_{j}"] for j in meta["arm"]["right"]]
    ik = PinchIK(m, meta, "right")
    ik.w_m, ik.w_palm = p["w_m"], p["w_palm"]
    h0 = np.array([p["h_tm"], p["h_tp"], p["h_mf"]])
    ik.lo[5:8] = np.maximum(0.0, h0 - p["hand_tol"])
    ik.hi[5:8] = np.minimum(ik.hi[5:8], h0 + p["hand_tol"])
    if p["others"] >= 0:
        ik.others = p["others"] * ik.cmd_hi[2]
    right_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("right_")}
    left_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")}

    # ---- the grip points, from the plate pose (camera estimate; pos_err = an error in it)
    pt, Rt = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()
    pt_est = pt + np.array([p["pos_err_x"], p["pos_err_y"], p["pos_err_z"]]) / 1000
    psi = math.radians(p["psi"])
    e = np.array([-math.sin(psi), -math.cos(psi), 0.0])       # outward at the grip, plate frame
    z = np.array([0, 0, 1.0])
    rt, rm = spec.R - p["dt"] / 1000, spec.R - p["dm"] / 1000
    a = spec.rim_ang
    n_top = -math.sin(a) * e + math.cos(a) * z                # outward normal of the rim's top surface (parallel to underside)
    n_und = math.sin(a) * e - math.cos(a) * z                 # outward normal of the underside
    # top surface of the rim = underside + rim thickness along the normal
    S_m = np.array([0, 0, spec.z_under(rm)]) + rm * e
    S_t = np.array([0, 0, spec.z_under(rt)]) + rt * e + spec.rim_t * math.cos(a) * n_top

    def targets(off, c=pt_est, R=Rt):
        """Thumb and middle pad points `off` mm off the surfaces (negative = pressed in), and their pad normals."""
        # the middle finger comes down outside the edge and its tip curls in under the rim: the fingertip points
        # inward (toward the plate centre), `mbeta` degrees up
        b = math.radians(p["mbeta"])
        tip_dir = R @ (-math.cos(b) * e + math.sin(b) * z)
        return (c + R @ (S_t + off / 1000 * n_top), -(R @ n_top), c + R @ (S_m + off / 1000 * n_und), tip_dir)

    base = d.qpos.copy()
    rng = np.random.default_rng(0)
    known = [[-0.2, 0.2, -0.1, 0.27, 0.6], [-0.46, -0.11, 0.27, 0.67, 1.77], [0.09, -0.43, 0.44, -0.06, 1.73],
             np.array(meta["ready"]["right"])]
    seeds = [np.r_[k, 1.0, 0.6, 0.8] for k in known] + \
            [np.r_[rng.uniform(ik.lo[:5], ik.hi[:5]), rng.uniform(0.3, 1.5), rng.uniform(0.2, 1.0), rng.uniform(0.3, 1.3)] for _ in range(int(p["seeds"]))]
    Pt, nt, Pm, nm = targets(p["gap"])
    ik.palm_dir = Rt @ (-e)
    seeds = [np.r_[s_[:5], h0] for s_ in seeds]
    sols = ik.solve(Pt, nt, Pm, nm, seeds, base, p["w_n"])

    def pose_clear(x, allow_top=1.0):
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, x)
        mujoco.mj_forward(m, d2)
        bad = {k: v for k, v in collisions(m, d2).items() if k.startswith("right_") and (not k.endswith(f"plate{n - 1}") or v < -allow_top)}
        return bad

    best = None
    for s in sols:
        if s["pos_mm"] > p["pos_tol"] or s["ang_deg"] > 30:
            continue
        x = s["x"]
        # open pose at the grip, then backed out along e: both must be clear, and the way there from the current arm
        xo = x.copy()
        xo[6] = max(0.0, x[6] - p["open_t"])
        xo[7] = max(0.0, x[7] - p["open_m"])
        bad = pose_clear(x, allow_top=p["allow_top"])
        bad.update(pose_clear(xo))
        if bad and min(bad.values()) > -p["allow_any"]:       # small overlaps: let the simulation settle them
            bad = {}
        if bad:
            if best is None or (best[0] is None and min(bad.values()) > best[2]):
                best = (None, s, min(bad.values()), bad)
            continue
        best = (x, s, 0.0, {})
        break
    if best is None:
        return dict(par=p, error=f"pinch not reachable (best {sols[0]['pos_mm']:.1f} mm, thumb {sols[0]['ang_deg']:.0f} deg)")
    if best[0] is None:
        k0 = min(best[3], key=best[3].get)
        return dict(par=p, error=f"pinch pose collides: {k0} {best[3][k0]:.1f} mm", ik=dict(pos_mm=best[1]["pos_mm"], ang_deg=best[1]["ang_deg"]))
    x_grip, sol = best[0], best[1]
    x_open = x_grip.copy()
    x_open[6] = max(0.0, x_grip[6] - p["open_t"])
    x_open[7] = max(0.0, x_grip[7] - p["open_m"])
    # backed-out pre-pose: same hand shape, the arm moved `back` mm outward (re-solved for the arm only)
    arm_ik = PadIK(m, meta, "right", "middle", "pad")

    def arm_shift(x_from, shift):
        """Arm joints that move the hand (rigidly) by `shift` (world, m), hand shape kept."""
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base
        ik.set(d2, x_from)
        mujoco.mj_kinematics(m, d2)
        P0, R0 = d2.xpos[ik.palm].copy(), d2.xmat[ik.palm].reshape(3, 3).copy()

        def r(q):
            xx = x_from.copy()
            xx[:5] = q
            ik.set(d2, xx)
            mujoco.mj_kinematics(m, d2)
            return np.concatenate([d2.xpos[ik.palm] - P0 - shift, 0.2 * (d2.xmat[ik.palm].reshape(3, 3) - R0).ravel()])
        q = least_squares(r, x_from[:5], bounds=(ik.lo[:5], ik.hi[:5]), xtol=1e-10).x
        xx = x_from.copy()
        xx[:5] = q
        return xx, float(np.linalg.norm(r(q)[:3]) * 1000)

    e_w = Rt @ e
    # come in from outside and a little low (the curled middle tip passes under the rim edge), then rise into place
    x_pre, err_pre = arm_shift(x_open, p["back"] / 1000 * e_w - p["drop"] / 1000 * z)
    x_mid, err_mid = arm_shift(x_open, -p["drop"] / 1000 * z)
    plan = dict(ik_pos_mm=sol["pos_mm"], ik_ang_deg=sol["ang_deg"], mid_ang_deg=sol["mid_ang_deg"], pre_err_mm=err_pre, x_grip=[round(float(v), 3) for v in x_grip])
    bad_pre = pose_clear(x_pre)
    bad_pre.update(pose_clear(x_mid))
    q_from = np.array([d.ctrl[a_] for a_ in arm_act])
    for u in np.linspace(0.1, 1.0, 10):
        xx = x_pre.copy()
        xx[:5] = q_from + (x_pre[:5] - q_from) * u
        bad_pre.update(pose_clear(xx))
    plan["approach_collisions"] = {k: round(v, 1) for k, v in bad_pre.items()}
    if plan_only:
        return dict(par=p, plan=plan)

    # ---- simulate. Two close-up cameras aimed at the grip: from outside the rim, and along the rim
    G = 0.5 * (Pt + Pm)

    def aim(cam, eye):
        cid = m.camera(cam).id
        zc = (eye - G) / np.linalg.norm(eye - G)
        xc = np.cross(z, zc)
        xc /= np.linalg.norm(xc)
        yc = np.cross(zc, xc)
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, np.c_[xc, yc, zc].ravel())
        m.cam_pos[cid], m.cam_quat[cid], m.cam_fovy[cid] = eye, q, 32.0
        m.cam_mode[cid] = 0
        m.cam_bodyid[cid] = 0
    tang = np.cross(z, e_w)
    aim("grip", G + 0.22 * e_w + 0.05 * z)
    aim("gripside", G + 0.22 * tang + 0.03 * z - 0.05 * e_w)
    renderer, shots = (mujoco.Renderer(m, 720, 1280), []) if frames else (None, [])

    def shoot(label):
        if renderer is None:
            return
        ims = []
        for cam in CAMS:
            renderer.update_scene(d, camera=cam)
            ims.append(renderer.render().copy())
        shots.append((label, ims))

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
                out[(hand, kind, part)] = out.get((hand, kind, part), 0.0) + float(np.linalg.norm(w[:3]))
        return out

    vid, vstate = None, dict(next=d.time, phase="")
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))

    def grab():
        import cv2
        if d.time < vstate["next"]:
            return
        vstate["next"] += 0.04
        ims = []
        for cam, lab in zip(CAMS, ("close-up from the robot's right", "from the front", "from the left", "from above (robot at the top)")):
            vr.update_scene(d, camera=cam)
            im = cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR)
            cv2.putText(im, lab, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            ims.append(im)
        f = forces()
        g = lambda hand, kind: sum(v for k, v in f.items() if k[0] == hand and k[1] == kind)   # noqa: E731
        bar = np.full((64, 1280, 3), 20, np.uint8)
        cv2.putText(bar, title[:150], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        low = g("right", "below") + g("right", "table")
        th = sum(v for k, v in f.items() if k[0] == "right" and k[1] == "top" and "thumb" in k[2])
        mi = sum(v for k, v in f.items() if k[0] == "right" and k[1] == "top" and "middle" in k[2])
        txt = (f"t {d.time:4.1f} s  {vstate['phase']:<10s} thumb {th:4.1f} N  middle {mi:4.1f} N  right hand on plates below/table {low:4.1f} N  "
               f"left finger {g('left', 'top'):4.1f} N  tilt {math.degrees(math.acos(min(1.0, d.xmat[pb[n - 1]][8]))):4.1f} deg")
        cv2.putText(bar, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60, 60, 255) if low > 0.5 else (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    log = []

    def command(x):
        d.ctrl[arm_act] = x[:5]
        c = ik.cmd(x)
        for i, j in enumerate(ik.hj):
            nm_ = m.joint(j).name
            d.ctrl[act[f"a_{nm_}"]] = min(ik.C[i] @ c, m.jnt_range[j][1])

    def move(xa, xb, T, phase, left=None):
        N = max(1, int(T / dt))
        vstate["phase"] = phase
        for k in range(N):
            u = (k + 1) / N
            u = 3 * u * u - 2 * u ** 3
            command(xa + (xb - xa) * u)
            if left is not None:
                d.ctrl[left[0]] = left[1] + (left[2] - left[1]) * u
            mujoco.mj_step(m, d)
            if vid is not None:
                grab()
            if k % 20 == 0:
                log.append(dict(phase=phase, t=d.time, f=forces(), pos=d.xpos[pb[n - 1]].copy()))

    x_now = np.r_[q_from, 0.0, 0.0, 0.0]
    shoot("after the push")
    move(x_now, x_pre, p["approach_T"], "approach")
    shoot("beside the rim")
    move(x_pre, x_mid, 1.0, "in under the edge")
    move(x_mid, x_open, 0.6, "up to the rim")
    shoot("open at the rim")
    x_sq = x_grip.copy()
    x_sq[6] = min(ik.hi[6], x_grip[6] + p["squeeze"])
    x_sq[7] = min(ik.hi[7], x_grip[7] + p["squeeze"])
    move(x_open, x_sq, p["close_T"], "pinch")
    move(x_sq, x_sq, 0.4, "pinch")
    shoot("pinched")
    P_g, R_g = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()
    # ---- left finger away (it held the plate from being pushed left while the right hand closed)
    l_act = [act[f"a_{j}"] for j in meta["arm"]["left"]]
    if p["left_away"]:
        ql0 = np.array([d.ctrl[a_] for a_ in l_act])
        ql1 = ql0.copy()
        ql1[0] -= 0.35                                         # shoulder pitch: hand up and back
        ql2 = np.array(meta["ready"]["left"])
        move(x_sq, x_sq, 0.6, "left away", left=(l_act, ql0, ql1))
        move(x_sq, x_sq, 0.8, "left away", left=(l_act, ql1, ql2))
        shoot("left finger away")
    # ---- lift a little: the hand rigidly up (and optionally turned so the plate levels)
    x_up, err_up = arm_shift(x_sq, p["up"] / 1000 * z)
    move(x_sq, x_up, p["up_T"], "lift")
    move(x_up, x_up, 1.0, "hold")
    shoot("lifted")
    if vid is not None:
        vid.release()
    # ---- results
    Pe, Re = d.xpos[pb[n - 1]].copy(), d.xmat[pb[n - 1]].reshape(3, 3).copy()
    (tp_, _), (mp_, _) = ik.pads(d)
    grip_local_end = Re.T @ (0.5 * (tp_ + mp_) - Pe)
    d.qpos[:] = d.qpos                                          # (the pads after the pinch were logged below)
    ps = [(d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()) for b in pb]
    # room under the LEFT side: lowest gap between the top plate's left rim underside and what is below it
    pl = ps[n - 2][0] if n > 1 else np.array([pp["x"], pp["y"], pp["table"]])
    left_gap = []
    for lat in (-0.03, 0.0, 0.03):
        for rho in np.arange(spec.R - 0.04, spec.R + 0.001, 0.002):
            loc = np.array([lat, rho, 0.0])
            q = Pe + Re @ np.array([lat, rho, spec.z_under(math.hypot(lat, rho))])
            # what is under q: the lower plates' top surface (approx: lower plate frame, z_top at the same xy)
            below = pp["table"]
            for k in range(n - 1):
                lk = ps[k][1].T @ (q - ps[k][0])
                r_ = math.hypot(lk[0], lk[1])
                if r_ <= spec.R:
                    below = max(below, ps[k][0][2] + spec.z_top(r_))
            left_gap.append((q[2] - below) * 1000)
    f_end = forces()
    peak = {}
    for r_ in log:
        for k, v in r_["f"].items():
            peak[k] = max(peak.get(k, 0.0), v)
    rise_grip = float((Pe + Re @ (0.5 * (S_t + S_m)))[2] - (P_g + R_g @ (0.5 * (S_t + S_m)))[2]) * 1000
    out = dict(par=p, push_par=pp, plan=plan,
               held=bool(sum(v for k, v in f_end.items() if k[0] == "right" and k[1] == "top") > 1.0),
               grip_rose_mm=rise_grip, plate_rose_mm=float(Pe[2] - P_g[2]) * 1000,
               tilt_deg=math.degrees(math.acos(min(1.0, Re[2, 2]))),
               slip_mm=float(np.linalg.norm(grip_local_end - 0.5 * (S_t + S_m))) * 1000,
               left_side_gap_mm=float(min(left_gap)),
               low_shift_mm=max([0.0] + [float(np.linalg.norm(d.xpos[b][:2] - st["p0"][k][:2])) * 1000 for k, b in enumerate(pb[:-1])]),
               right_on_lower_or_table_N=max([v for k, v in peak.items() if k[0] == "right" and k[1] != "top"] + [0.0]),
               peak_N={" ".join(k): round(v, 1) for k, v in sorted(peak.items(), key=lambda kv: -kv[1])[:8]},
               end_N={" ".join(k): round(v, 1) for k, v in sorted(f_end.items(), key=lambda kv: -kv[1])})
    out["lift_err_mm"] = err_up
    if shots:
        out["_shots"] = shots
    out["_final"] = dict(qpos=d.qpos.copy(), qvel=d.qvel.copy(), ctrl=d.ctrl.copy(), time=d.time)
    return out


def sheet(res, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    shots = res["_shots"]
    fig, axs = plt.subplots(4, len(shots), figsize=(4.3 * len(shots), 10.2))
    for j, (label, ims) in enumerate(shots):
        for i in range(4):
            axs[i, j].imshow(ims[i])
            axs[i, j].axis("off")
        axs[0, j].set_title(label, fontsize=11)
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--sheet", default=None)
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--save", default=None)
    ap.add_argument("--plan", action="store_true")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    over = {k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None}
    t0 = time.time()
    res = run(a.state, over, frames=bool(a.sheet), video=a.video, title=a.title, plan_only=a.plan)
    if "error" in res:
        print("ERROR:", res["error"], res.get("ik", ""))
        return
    if a.sheet:
        sheet(res, a.sheet, f"MuJoCo, right-hand pinch (thumb on top, middle under), psi {res['par']['psi']:.0f}: held {res['held']}, "
                            f"grip point rose {res['grip_rose_mm']:.0f} mm, tilt {res['tilt_deg']:.0f} deg, slip {res['slip_mm']:.1f} mm, "
                            f"gap under the left rim {res['left_side_gap_mm']:.0f} mm")
    res.pop("_shots", None)
    fin = res.pop("_final", None)
    if a.save and fin:
        np.savez(a.save, **fin, par=json.dumps(res["push_par"]), grip=json.dumps(res["par"]))
    res.pop("push_par", None)
    print(json.dumps(res, indent=1))
    print(f"wall time {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
