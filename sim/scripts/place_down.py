"""Put-down: from a carried plate, set it flat on the table and clear the hands.

    python scripts/place_down.py --state H.npz [--first left|right --clear 3...] [--video out.mp4]
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

from platepick.forces import ForceMeter  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402

DEFAULTS = dict(first="left", x_place=-1.0, dy=0.0, hover=40.0, carry_T=2.5, enc_fb=3, open_T=0.6, out=35.0, up_out=60.0, out_T=1.0,
                hang_T=0.5, lower_first=0, open_under=1, out_up=0.0, last_back=0.0, first_back=0.0, back_up=0.0, rise_open=0.0, clear=3.0, lower_T=2.0, thumb_open=0.0, w_rot=0.05, settle=1.0,
                tilt=0, tilt_clear=8.0, touch=0.5, w_tilt=0.5, tilt_max=30.0, follow=1, follow_clear=3.0, pre_tilt=0.0, hover_hand=0.0)
SIDES = ("left", "right")
HAND_PARTS = ("base_link", "index", "middle", "ring", "pinky", "thumb")


def run(state, par, video=None, title=""):
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
    # the bare table: every plate but the carried one is hidden, without collisions, parked far away and weightless
    top = m.body(f"plate{n - 1}").id
    for k in range(n - 1):
        b = m.body(f"plate{k}").id
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == b:
                m.geom_contype[g] = m.geom_conaffinity[g] = 0
                m.geom_rgba[g, 3] = 0.0
        j = m.joint(f"plate{k}_free").id
        d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 3] = [5.0 + k, 5.0, 2.0]
        d.qvel[m.jnt_dofadr[j]:m.jnt_dofadr[j] + 6] = 0.0
        m.body_gravcomp[b] = 1.0
    mujoco.mj_forward(m, d)
    t0_sim = d.time
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = {s: [act[f"a_{j}"] for j in meta["arm"][s]] for s in SIDES}
    adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]] for s in SIDES}
    lo = {s: np.array([m.jnt_range[m.joint(j).id][0] for j in meta["arm"][s]]) for s in SIDES}
    hi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in meta["arm"][s]]) for s in SIDES}
    HJ = {s: list(meta["hand"][s]) for s in SIDES}
    HA = {s: [act[f"a_{j}"] for j in HJ[s]] for s in SIDES}
    palm = {s: m.body(f"{s}_base_link").id for s in SIDES}
    hand_geoms = {s: {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(s + "_")} for s in SIDES}
    mesh_geoms = {s: [g for g in hand_geoms[s] if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
                      and any(t in m.body(m.geom_bodyid[g]).name for t in HAND_PARTS)] for s in SIDES}
    plate_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == top}
    fm = ForceMeter(m, "left", top, None, hands=SIDES)
    fm.R_edge = spec.R
    z = np.array([0, 0, 1.0])
    table_top = pp["table"]
    foot_r = spec.foot_out                                             # plate frame: z = 0 at the foot bottom
    th = np.linspace(0, 2 * math.pi, 72, endpoint=False)
    ring_loc = np.c_[foot_r * np.cos(th), foot_r * np.sin(th), np.zeros_like(th)]

    def foot_z(dd=None):
        dd = dd or d
        return (dd.xmat[top].reshape(3, 3) @ ring_loc.T)[2] + dd.xpos[top][2]

    def hand_low(s, dd=None):
        dd = dd or d
        lowz = 9.0
        for g in mesh_geoms[s]:
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            lowz = min(lowz, float((V @ dd.geom_xmat[g].reshape(3, 3).T[:, 2]).min() + dd.geom_xpos[g][2]))
        return lowz

    def ik(s, Pt, Rt, q0, base_q, w_rot=None):
        w_rot = p["w_rot"] if w_rot is None else w_rot
        d2 = mujoco.MjData(m)
        d2.qpos[:] = base_q

        def res(q):
            d2.qpos[adr[s]] = q
            mujoco.mj_kinematics(m, d2)
            return np.concatenate([d2.xpos[palm[s]] - Pt, w_rot * (d2.xmat[palm[s]].reshape(3, 3) - Rt).ravel()])
        r = least_squares(res, np.clip(q0, lo[s] + 1e-4, hi[s] - 1e-4), bounds=(lo[s], hi[s]), xtol=1e-10)
        return r.x, float(np.linalg.norm(res(r.x)[:3]) * 1000)

    def palm_shift(s, q, dv):
        """Arm joints that move the palm by dv from where q puts it (orientation kept, soft); planned on the robot's own
        kinematics (joint angles q), not on the measured plate."""
        d2 = mujoco.MjData(m)
        d2.qpos[:] = d.qpos
        d2.qpos[adr[s]] = q
        mujoco.mj_kinematics(m, d2)
        return ik(s, d2.xpos[palm[s]] + dv, d2.xmat[palm[s]].reshape(3, 3).copy(), q, d2.qpos)

    # ---- plan the carry back over the table (the robot knows its joint angles, the plate pose from the camera)
    P0, R0 = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
    off = {s: d.ctrl[A[s]] - d.qpos[adr[s]] for s in SIDES}            # what each arm's target already carries for sag
    q_meas = {s: d.qpos[adr[s]].copy() for s in SIDES}
    first = p["first"]
    second = "right" if first == "left" else "left"
    target_xy = np.array([pp["x"] if p["x_place"] < 0 else p["x_place"], P0[1] + p["dy"] / 1000])
    ax = R0[:, 2]
    v = np.cross(ax, z)
    Rlev = np.eye(3)
    if np.linalg.norm(v) > 1e-9:                                       # level the plate, keeping its yaw
        ang = math.asin(min(1.0, np.linalg.norm(v)))
        k_ = v / np.linalg.norm(v)
        K = np.array([[0, -k_[2], k_[1]], [k_[2], 0, -k_[0]], [-k_[1], k_[0], 0]])
        Rlev = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    Rc = Rlev @ R0
    if p["pre_tilt"]:
        # one hand alone cannot hold the plate level: both hands carry it tilted, the first hand's side up by pre_tilt deg,
        # so that after that hand lets go the sag brings it near level (--tilt)
        uf = d.xpos[palm[p["first"]]] - P0
        uf[2] = 0.0
        uf /= np.linalg.norm(uf)
        af = np.cross(uf, z)                                           # positive turn about af raises the +uf side
        Kf = np.array([[0, -af[2], af[1]], [af[2], 0, -af[0]], [-af[1], af[0], 0]])
        tf = math.radians(p["pre_tilt"])
        Rc = (np.eye(3) + math.sin(tf) * Kf + (1 - math.cos(tf)) * Kf @ Kf) @ Rc
    Pc = np.r_[target_xy, table_top + p["hover"] / 1000 - (Rc @ ring_loc.T)[2].min()]
    if p["hover_hand"] > 0:
        # also raise the plate until both hands' lowest points are hover_hand mm above the table
        Pc[2] = max(Pc[2], max(table_top + p["hover_hand"] / 1000 - (hand_low(s) - P0[2]) for s in SIDES))
    q_hover, ik_err = {}, {}
    for s in SIDES:
        Pp, Rp = d.xpos[palm[s]].copy(), d.xmat[palm[s]].reshape(3, 3).copy()
        q_hover[s], ik_err[s] = ik(s, Pc + Rc @ R0.T @ (Pp - P0), Rc @ R0.T @ Rp, q_meas[s], d.qpos)

    # ---- video
    vid = vr = None
    vs = dict(next=d.time, phase="")
    if video:
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 784))

    def contact_force(test):
        F, w = 0.0, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if test(c.geom1, c.geom2) or test(c.geom2, c.geom1):
                mujoco.mj_contactForce(m, d, i, w)
                F += float(np.linalg.norm(w[:3]))
        return F

    hand_table = lambda: contact_force(lambda a, b: m.geom_bodyid[a] == 0 and (b in hand_geoms["left"] or b in hand_geoms["right"]))  # noqa: E731
    plate_table = lambda: contact_force(lambda a, b: m.geom_bodyid[a] == 0 and b in plate_geoms)  # noqa: E731
    hand_plate = lambda s: contact_force(lambda a, b: a in plate_geoms and b in hand_geoms[s])  # noqa: E731

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
        cv2.putText(bar, title[:150], (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        fz = foot_z()
        tl = math.degrees(math.acos(min(1.0, d.xmat[top][8])))
        cv2.putText(bar, f"t {d.time - t0_sim:4.1f} s {vs['phase']:<22s} foot above table {1000 * (fz.min() - table_top):5.1f} / {1000 * (fz.max() - table_top):5.1f} mm  "
                    f"tilt {tl:4.1f} deg  plate on table {plate_table():5.1f} N  hand on table {hand_table():4.1f} N",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (120, 220, 120), 1, cv2.LINE_AA)
        vid.write(np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])]))

    log = dict(hand_table=0.0, plate_table=0.0, vdown=0.0, phases=[])
    cur = {s: q_meas[s].copy() for s in SIDES}                       # planned arm targets (without the sag offset)
    hand_now = {s: d.ctrl[HA[s]].copy() for s in SIDES}
    prev = [foot_z().min()]

    def move(qb, T_, phase, hands=None):
        vs["phase"] = phase
        qa = {s: cur[s].copy() for s in SIDES}
        ha = {s: hand_now[s].copy() for s in SIDES}
        N = max(1, int(T_ / dt))
        ph = dict(phase=phase, vdown=0.0, F={s: 0.0 for s in SIDES}, hand_table=0.0)
        for k in range(N):
            u = (k + 1) / N
            u = 3 * u * u - 2 * u ** 3
            for s in SIDES:
                d.ctrl[A[s]] = np.clip(qa[s] + (qb.get(s, qa[s]) - qa[s]) * u + off[s], lo[s], hi[s])
                if hands and s in hands:
                    d.ctrl[HA[s]] = ha[s] + (hands[s] - ha[s]) * u
            mujoco.mj_step(m, d)
            fm.update(d)
            fz = foot_z().min()
            log["vdown"] = max(log["vdown"], (prev[0] - fz) / dt)
            ph["vdown"] = max(ph["vdown"], (prev[0] - fz) / dt)
            prev[0] = fz
            for s in SIDES:
                ph["F"][s] = max(ph["F"][s], max(v for (h_, _), v in fm.now.items() if h_ == s))
            if k % 5 == 0:
                log["hand_table"] = max(log["hand_table"], hand_table())
                ph["hand_table"] = max(ph["hand_table"], hand_table())
                log["plate_table"] = max(log["plate_table"], plate_table())
            if vid is not None:
                grab()
        fz_ = foot_z()
        log["phases"].append(dict(phase=phase, vdown_mm_s=round(ph["vdown"] * 1000), F_max_N={s: round(v, 1) for s, v in ph["F"].items()},
                                  hand_table_N=round(ph["hand_table"], 1), foot_end_mm=[round((fz_.min() - table_top) * 1000, 1), round((fz_.max() - table_top) * 1000, 1)]))
        for s in SIDES:
            if s in qb:
                cur[s] = qb[s].copy()
            if hands and s in hands:
                hand_now[s] = hands[s].copy()

    def enc_fb(qs, phase):
        for _ in range(int(p["enc_fb"])):
            for s in qs:
                off[s] += qs[s] - d.qpos[adr[s]]
            move({}, 0.3, phase)

    # the finger under the rim, per hand: the first grip (robot right) from the grip record, the second from grip_left's
    slots = {"right": int(gp.get("slot_mr", 0)), "left": int(json.loads(str(st["left"])).get("slot_mr", 0)) if "left" in st.files else 0}
    under = {sd: ("middle", "ring", "pinky")[slots[sd]] for sd in SIDES}

    def release(s, tag, lower=False, last=False):
        h = hand_now[s].copy()
        for i, j in enumerate(HJ[s]):
            if "thumb_proximal" in j or "thumb_distal" in j:
                h[i] = p["thumb_open"]
        move({}, p["open_T"], f"{tag}: thumb off", hands={s: h})
        e0 = 0.0
        if lower:
            # thumb off: the rim rests on the finger under it; lower that side until the palm is `clear` mm above the table,
            # then slide out
            q_dn, e0 = palm_shift(s, cur[s], -(hand_low(s) - table_top - p["clear"] / 1000) * z)
            move({s: q_dn}, p["lower_T"], f"{tag}: lower")
            enc_fb({s: q_dn}, f"{tag}: lower")
        if last and p["tilt"] and p["follow"]:
            # tilt put-down: the far side already rests on the table. Lower the palm with the thumb off (the plate pivots on its
            # far foot) until the hand's lowest point is follow_clear mm above the table; the hand never presses the table
            q_f, e0 = palm_shift(s, cur[s], -max(0.0, hand_low(s) - table_top - p["follow_clear"] / 1000) * z)
            move({s: q_f}, p["lower_T"], f"{tag}: follow down")
            enc_fb({s: q_f}, f"{tag}: follow down")
        if p["rise_open"] > 0 and last:
            # the last hand rises rise_open mm, then opens every finger in the air, clear of the table
            q_r, _ = palm_shift(s, cur[s], p["rise_open"] / 1000 * z)
            move({s: q_r}, p["open_T"], f"{tag}: rise")
            move({}, p["open_T"], f"{tag}: all open", hands={s: np.zeros_like(h)})
        elif p["open_under"] and last:
            # the last hand straightens only the finger under the rim before it slides out (opening the curled fingers too
            # pushes them into the table)
            h2 = hand_now[s].copy()
            for i, j in enumerate(HJ[s]):
                if f"_{under[s]}_" in j:
                    h2[i] = 0.0
            move({}, p["open_T"], f"{tag}: {under[s]} straightens", hands={s: h2})
        u = d.xpos[palm[s]] - d.xpos[top]
        u[2] = 0.0
        u /= np.linalg.norm(u)
        e1 = 0.0
        back = p["last_back"] if last else p["first_back"]          # first_back: the same for the first hand
        if back > 0:
            # pull the hand back along its fingers first, so the straightened finger comes out lengthwise
            fd = d.xmat[palm[s]].reshape(3, 3)[:, 2].copy()
            fd[2] = 0.0
            fd /= np.linalg.norm(fd)
            q_b, e1 = palm_shift(s, cur[s], -back / 1000 * fd + p["back_up"] / 1000 * z)    # back_up: rise while pulling back
            move({s: q_b}, p["out_T"], f"{tag}: pull back")
        q_out, e1b = palm_shift(s, cur[s], p["out"] / 1000 * u + p["out_up"] / 1000 * z)    # out_up: rise while sliding out
        e1 = max(e1, e1b)
        move({s: q_out}, p["out_T"], f"{tag}: slide out")
        q_up, e2 = palm_shift(s, cur[s], p["up_out"] / 1000 * z)
        move({s: q_up}, p["out_T"], f"{tag}: away", hands={s: np.zeros_like(h)})
        return max(e0, e1, e2)

    move(q_hover, p["carry_T"], "carry back")
    enc_fb(q_hover, "carry back")
    hover_err = float(np.linalg.norm(d.xpos[top][:2] - target_xy)) * 1000
    e1 = release(first, f"release {first}", lower=bool(p["lower_first"]))
    move({}, p["hang_T"], f"{second} carries alone")
    hang_tilt = math.degrees(math.acos(min(1.0, d.xmat[top][8])))
    tilt_plan = None
    if p["tilt"]:
        # tilt: the carrying hand turns the plate about the horizontal axis through its palm so that the far side goes down,
        # until the far foot touches the table (`touch` mm into it) with the hand's lowest point tilt_clear mm above the
        # table. Planned on the hand's kinematics and the plate pose, the plate rigid in the hand.
        Pp, Rpm = d.xpos[palm[second]].copy(), d.xmat[palm[second]].reshape(3, 3).copy()
        Pt_, Rt_ = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
        u = Pp - Pt_
        u[2] = 0.0
        u /= np.linalg.norm(u)
        a_ = np.cross(u, z)
        a_ /= np.linalg.norm(a_)
        Ka = np.array([[0, -a_[2], a_[1]], [a_[2], 0, -a_[0]], [-a_[1], a_[0], 0]])
        Vh = []
        for g in mesh_geoms[second]:
            mid = m.geom_dataid[g]
            V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            Vh.append(V @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g])
        Vh = np.vstack(Vh)
        ring_w = (Rt_ @ ring_loc.T).T + Pt_
        for th_ in np.arange(0.0, p["tilt_max"] + 1e-6, 0.25):
            Rr = np.eye(3) + math.sin(math.radians(th_)) * Ka + (1 - math.cos(math.radians(th_))) * Ka @ Ka
            dz = table_top - p["touch"] / 1000 - ((ring_w - Pp) @ Rr.T + Pp)[:, 2].min()
            hl = ((Vh - Pp) @ Rr.T + Pp)[:, 2].min() + dz
            if hl >= table_top + p["tilt_clear"] / 1000:
                break
        tilt_plan = dict(deg=round(float(th_), 2), palm_down_mm=round(-dz * 1000, 1), hand_low_mm=round((hl - table_top) * 1000, 1))
        q_t, e2 = ik(second, Pp + dz * z, Rr @ Rpm, cur[second], d.qpos, w_rot=p["w_tilt"])
        move({second: q_t}, p["lower_T"], f"tilt {second}")
        enc_fb({second: q_t}, f"tilt {second}")
    else:
        # the carrying hand straight down until its lowest point is `clear` mm above the table (hand kinematics + table height)
        drop_hand = hand_low(second) - (table_top + p["clear"] / 1000)
        q_low, e2 = palm_shift(second, cur[second], -drop_hand * z)
        move({second: q_low}, p["lower_T"], f"lower {second}")
        enc_fb({second: q_low}, f"lower {second}")
    fz = foot_z()
    before = dict(low_foot_mm=round((fz.min() - table_top) * 1000, 1), high_foot_mm=round((fz.max() - table_top) * 1000, 1),
                  tilt_deg=round(math.degrees(math.acos(min(1.0, d.xmat[top][8]))), 1), hand_low_mm=round((hand_low(second) - table_top) * 1000, 1))
    # the carrying hand's pose at this moment, for the lone-plate pick (the put-down's last step reversed,
    # scripts/lone_slide.py): palm relative to the plate centre (world axes), palm axes, arm joints, hand commands
    pose = dict(side=second, palm_rel=(d.xpos[palm[second]] - d.xpos[top]).tolist(), palm_R=d.xmat[palm[second]].reshape(3, 3).tolist(),
                q_arm=d.qpos[adr[second]].tolist(), hand_ctrl=d.ctrl[HA[second]].tolist(), plate_R=d.xmat[top].reshape(3, 3).tolist(),
                hand_low_above_table_mm=round((hand_low(second) - table_top) * 1000, 2))
    e3 = release(second, f"release {second}", last=True)
    move({}, p["settle"], "settle")
    if vid is not None:
        vid.release()
    Pe, Re = d.xpos[top].copy(), d.xmat[top].reshape(3, 3).copy()
    fz = foot_z()
    f = fm.summary()
    out = dict(par=p, carry_ik_mm={s: round(v, 1) for s, v in ik_err.items()}, move_ik_mm=round(max(e1, e2, e3), 1), hover_xy_err_mm=round(hover_err, 1),
               tilt_carried_alone_deg=round(hang_tilt, 1), tilt_plan=tilt_plan, before_last_release=before, last_hand_pose=pose,
               end=dict(centre_err_mm=round(float(np.linalg.norm(Pe[:2] - target_xy)) * 1000, 1), tilt_deg=round(math.degrees(math.acos(min(1.0, Re[2, 2]))), 2),
                        foot_above_table_mm=round((fz.min() - table_top) * 1000, 1), hand_on_plate_N={s: round(hand_plate(s), 2) for s in SIDES},
                        front_edge_margin_mm=round((Pe[0] - spec.R - (pp["x"] - spec.R - 0.015)) * 1000, 1)),
               foot_descent_max_mm_s=round(log["vdown"] * 1000), plate_on_table_peak_N=round(log["plate_table"], 1), hand_on_table_max_N=round(log["hand_table"], 1),
               peak50ms_N={s: f[f"{s}_part_max50ms_N"] for s in SIDES}, phases=log["phases"], force=f)
    out["placed"] = bool(out["end"]["tilt_deg"] < 1.5 and abs(out["end"]["foot_above_table_mm"]) < 2.0 and max(out["end"]["hand_on_plate_N"].values()) < 0.05
                         and out["end"]["front_edge_margin_mm"] > 0)
    out["within_limits"] = bool(max(out["peak50ms_N"].values()) <= 60)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", required=True)
    ap.add_argument("--video", default=None)
    ap.add_argument("--title", default="")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    over = {k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None}
    t0 = time.time()
    r = run(a.state, over, video=a.video, title=a.title)
    print(json.dumps(r, indent=1, default=float))
    print(f"wall time {time.time() - t0:.0f} s")


if __name__ == "__main__":
    main()
