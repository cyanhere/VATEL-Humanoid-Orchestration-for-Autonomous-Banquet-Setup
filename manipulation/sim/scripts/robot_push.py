"""Stage 2b: the R1's left hand pushes the top plate with a finger pad on the rim edge (full robot, dynamics).

    python scripts/robot_push.py --sheet out.png                       # one push with pictures
    python scripts/robot_push.py --phi 90 --gamma 40 --finger pinky --n 5 --table 0.747
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

from platepick.forces import ForceMeter  # noqa: E402
from platepick.ik import PadIK  # noqa: E402
from platepick.plate import Friction  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from push_probe import opening  # noqa: E402
from reach_push import collisions, push_pose  # noqa: E402

DEFAULTS = dict(retry_find=0, done_mm=0.0, press_g=0.0, cal_r2=15.0, f_slide=1.0, slide_k=0.02, slide_v=5.0, cal2_off_r=0.0, cal2_off_z=0.0, plan_cam=0, sc_nom=0.0, cal_off=0.0, lag_k=0.0, lag0=3.0, fz_k=0.0, fz0=2.0, ctrl=0, cal=1, cam_x=0.0, cam_y=0.0, cam_z=0.0, cam_noise=1.0, f_on=1.5, f_hi=12.0, retry_dz=0.7, retries=3, ride_mm=12.0, cal_r=6.0,
                mode="tip", finger="pinky", phi=0.0, gamma=20.0, beta=0.0, follow=1.0, lift=0.0, roll=0.0, bias=0.0, bias_len=10.0, flex=0.0, along=0, pathcheck=1, fb=0.0, hold=0.0, fb_err=0.0, gate=0.0, fbsrc=0, a0=0.0, fbf=0.015, press=3.0, dist=55.0, speed=20.0, n=3, table=0.762,
                x=0.36, y=0.06, table_right=0.0, pelvis=0.76, foot_glaze=0.575, glaze_glaze=0.375, pad_glaze=0.25, table_mu=0.45)


# how the pushed rim edge sinks as the top plate slides over the one below (mm slid -> mm dropped), measured in
# MuJoCo; the fingertip path follows it
EDGE_S = [0.0, 8.0, 45.0, 80.0]
EDGE_DROP = [0.0, 0.0, 4.5, 4.6]


def run(par, frames=False, video=None, title="", save=None):
    p = dict(DEFAULTS, **par)
    fr = Friction(glaze_glaze=p["glaze_glaze"], foot_glaze=p["foot_glaze"], pad_glaze=p["pad_glaze"], table=p["table_mu"])
    n = int(p["n"])
    sc = RobotScene(n=n, table=p["table"], xy=(p["x"], p["y"]), pelvis=p["pelvis"], fr=fr, table_right=p["table_right"])
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec, dt = sc.spec, m.opt.timestep
    ik = PadIK(m, meta, "left", p["finger"], p["mode"])
    pb = [m.body(f"plate{k}").id for k in range(n)]
    fm = ForceMeter(m, "left", pb[n - 1], pb[n - 2] if n > 1 else None)
    act = {m.actuator(i).name: i for i in range(m.nu)}
    arm_act = [act[f"a_{j}"] for j in meta["arm"]["left"]]
    flex_act = [(act[f"a_{m.joint(j).name}"], g) for j, g in ik.flex]
    left_geoms = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")}
    pad_geoms = {g for g in left_geoms if m.body(m.geom_bodyid[g]).name.startswith("left_" + p["finger"] + "_")}

    where = {}

    def poses():
        return [(d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()) for b in pb]

    # what the real Revo2 Touch pads would sense during the push (one point per fingertip pad: normal force,
    # and proximity to ~1 cm along the pad's normal). The pushing contact itself is the END of the curled finger.
    from platepick.ik import PAD_NORMAL, PAD_POINT
    pads = {f: m.body(f"left_{f}_touch_link").id for f in ("index", "middle", "ring", "pinky", "thumb")}
    plate_bodies = set(pb)

    def pad_sense():
        out, w, gid = {}, np.zeros(6), np.zeros(1, np.int32)
        for f, b in pads.items():
            R = d.xmat[b].reshape(3, 3)
            hit = mujoco.mj_ray(m, d, d.xpos[b] + R @ PAD_POINT, R @ PAD_NORMAL, None, 1, b, gid)
            hb = m.geom_bodyid[gid[0]] if gid[0] >= 0 else -1
            Fn = 0.0
            for i in range(d.ncon):
                c = d.contact[i]
                if b in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]):
                    mujoco.mj_contactForce(m, d, i, w)
                    Fn += abs(w[0])
            out[f] = (float(hit) * 1000 if hb in plate_bodies and hit >= 0 else None, Fn,
                      m.body(hb).name if hb >= 0 else "")
        return out

    def hand_force():
        """Force of the plates on the left hand (world), the part on the pushing pad, and who is touched."""
        F, Fpad, who, w = np.zeros(3), np.zeros(3), {}, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            a, b = c.geom1 in left_geoms, c.geom2 in left_geoms
            if a == b:
                continue
            other_body = m.body(m.geom_bodyid[c.geom1 if b else c.geom2]).name
            if not (other_body.startswith("plate") or other_body == "world"):
                continue                                  # the arm against the robot's own body is checked in the plan
            mujoco.mj_contactForce(m, d, i, w)
            f = c.frame.reshape(3, 3).T @ w[:3]
            f = f if b else -f
            other = c.geom1 if b else c.geom2
            mine = c.geom2 if b else c.geom1
            F += f
            if mine in pad_geoms and m.geom_bodyid[other] == pb[n - 1]:      # the pushing finger on the TOP plate only
                Fpad += f
            key = (m.body(m.geom_bodyid[mine]).name.replace("left_", ""), m.body(m.geom_bodyid[other]).name)
            who[key] = max(who.get(key, 0.0), float(np.linalg.norm(f)))
            ob = m.geom_bodyid[other]
            if other_body.startswith("plate") and ob != pb[n - 1] and np.linalg.norm(f) > where.get("N", 0.0):
                # where the finger meets a lower plate: radius on that plate, height above its rim top, and how far
                # behind the fingertip end (along the finger) the contact is
                rel = d.xmat[ob].reshape(3, 3).T @ (c.pos - d.xpos[ob])
                e, _ = ik.end(d)
                where.update(N=float(np.linalg.norm(f)), r_mm=float(np.hypot(rel[0], rel[1]) * 1000), below_rim_top_mm=float((spec.H - rel[2]) * 1000),
                             behind_tip_mm=float(np.linalg.norm(c.pos - e) * 1000), slid_mm=float(np.linalg.norm((d.xpos[pb[n - 1]] - d.xpos[pb[n - 2]])[:2]) * 1000),
                             part=m.body(m.geom_bodyid[mine]).name.replace("left_", ""))
        return F, Fpad, who

    # ---- plan (kinematics): pre-contact, touch, push
    top_true = np.array([p["x"], p["y"], p["table"] + (n - 1) * spec.floor_top])
    top = top_true + np.array([p["cam_x"], p["cam_y"], p["cam_z"]]) / 1000      # the camera's estimate
    mujoco.mj_forward(m, d)
    base = d.qpos.copy()
    for k in range(n):
        base[m.jnt_qposadr[m.joint(f"plate{k}_free").id] + 2] = p["table"] + k * spec.floor_top
        if p["plan_cam"]:          # the planner's world is the camera's: the stack where the camera says it is
            a_ = m.jnt_qposadr[m.joint(f"plate{k}_free").id]
            base[a_:a_ + 3] += np.array([p["cam_x"], p["cam_y"], p["cam_z"]]) / 1000
    for jn, v in zip(meta["arm"]["right"], meta["ready"]["right"]):
        base[m.jnt_qposadr[m.joint(jn).id]] = v
    P0, nrm = push_pose(spec, top, p["phi"], p["gamma"], standoff=0.0)
    rng = np.random.default_rng(0)
    if p["mode"] == "pad":
        ik.z_keep = top[2] + spec.H + 0.06
    c0 = [-0.55, 0.38, -0.37, 0.42, 0.2, 0.63]                 # typical clear pose in the sampled reach data
    side = p["mode"].startswith("side")
    ax_kw = {}
    if side:                     # palm down, the finger curled downward like a post, its side on the rim
        c0 = [-0.47, 0.39, -0.19, 0.41, 1.27, 0.62]
        ax_kw = dict(axis=np.array([0.0, 0.0, -1.0]), w_axis=0.01)
    seeds = [c0, [-0.6, 0.4, -0.4, 0.4, -0.2, 0.6], [-0.3, 0.2, 0.0, 0.2, 0.8, 0.8], [0, 0, 0, 0, 0.8, 0.6]] + \
            [list(np.clip(np.r_[rng.normal(c0[:5], 0.35), rng.uniform(0.2, 1.3)], ik.lo, ik.hi)) for _ in range(14)]
    if p["along"]:               # the hand lies along the rim (wrist-to-knuckles = the rim's direction, forward)
        ph = math.radians(p["phi"])
        ik.hand_dir = np.array([math.cos(ph), -math.sin(ph), 0.0])
    if p["flex"] > 0:            # finger bent to a set fraction (0.5 = half of its travel)
        f0 = p["flex"] * m.jnt_range[ik.flex[0][0]][1]
        ik.lo[5], ik.hi[5] = f0 - 2e-3, f0 + 2e-3
        seeds = [list(s[:5]) + [f0] for s in seeds]
    res = ik.solve(P0 - 0.002 * nrm + p["lift"] / 1000 * np.array([0, 0, 1.0]), nrm, seeds, base=base, **ax_kw)
    pick, clear = None, -1.0
    for c in res["all"]:
        if c["pos_err_mm"] > 1.0 or c["ang_err_deg"] > 2.0 or (side and c["axis"][2] > -0.6):
            continue
        d.qpos[:] = base
        ik.set(d, c["x"])
        mujoco.mj_forward(m, d)
        if {k: v for k, v in collisions(m, d).items() if v < -1.0}:
            continue
        # among the clear solutions take the one whose wrist and palm are highest above the top plate
        h = min(d.xpos[ik.wrist][2], d.xpos[ik.palm][2]) - (top[2] + spec.H)
        if h > clear:
            pick, clear = c, h
    if pick is None:
        return dict(par=p, error="push pose not reachable without collision")
    ik.z_keep = ik.hand_dir = None
    flex = pick["x"][5]
    ik.lo[5], ik.hi[5] = flex - 2e-3, flex + 2e-3            # the finger keeps its bend along the path
    out = np.array([math.sin(math.radians(p["phi"])), math.cos(math.radians(p["phi"])), 0.0])
    b = math.radians(p["beta"])
    u_push = -math.cos(b) * out - math.sin(b) * np.array([0, 0, 1.0])
    up = np.array([0, 0, 1.0])
    lift = p["lift"] / 1000 * up                              # aim this much above the rim-edge centre
    way = [("pre", P0 - 0.025 * nrm + lift, nrm), ("touch", P0 + p["press"] / 1000 * nrm + lift, nrm)]
    steps = max(2, int(p["dist"] / 5))
    for i in range(1, steps + 1):
        s = p["dist"] * i / steps                             # mm pushed so far
        g = math.radians(p["gamma"] + p["roll"] * i / steps)  # the wrist rolls the fingertip further down by `roll` deg
        nrm_i = -math.cos(g) * out - math.sin(g) * up
        way.append(("push", P0 + p["press"] / 1000 * nrm + lift + u_push * s / 1000
                    - up * p["follow"] * float(np.interp(s, EDGE_S, EDGE_DROP)) / 1000
                    - up * p["bias"] / 1000 * min(1.0, s / p["bias_len"]), nrm_i))
    qs, x = [], pick["x"]
    for name, pt, nrm_w in way:
        r = ik.solve(pt, nrm_w, [x], base=base, **ax_kw)
        if r["pos_err_mm"] > 2.0 or r["ang_err_deg"] > 4.0:
            return dict(par=p, error=f"path leaves the arm's reach at '{name}' ({r['pos_err_mm']:.1f} mm, {r['ang_err_deg']:.1f} deg)")
        x = r["x"]
        qs.append(x.copy())
        d.qpos[:] = base                                  # nothing but the pushing finger may be inside anything
        ik.set(d, x)
        mujoco.mj_forward(m, d)
        bad = {k: v for k, v in collisions(m, d).items() if v < -1.5 and not (k.startswith(f"left_{p['finger']}_") and k.endswith(f"plate{n - 1}"))}
        if bad and name != "pre" and p["pathcheck"]:
            k0 = min(bad, key=bad.get)
            return dict(par=p, error=f"planned path collides: {k0.replace('left_', '')} ({bad[k0]:.0f} mm)")
    # ---- simulate
    d.qpos[:] = m.qpos0
    for jn, v in zip(meta["arm"]["right"], meta["ready"]["right"]):
        d.qpos[m.jnt_qposadr[m.joint(jn).id]] = v
        d.ctrl[act[f"a_{jn}"]] = v
    q_park = qs[0].copy()
    q_park[0] -= 0.25                                         # start a little higher than the pre-contact pose
    ik.set(d, q_park)
    mujoco.mj_forward(m, d)

    def command(q):
        d.ctrl[arm_act] = q[:5]
        for a, g in flex_act:
            d.ctrl[a] = min(g * q[5], m.actuator_ctrlrange[a][1])

    renderer, shots = (mujoco.Renderer(m, 720, 1280), []) if frames else (None, [])

    def shoot(label):
        if renderer is None:
            return
        ims = []
        for cam in ("closefront", "close", "left", "top"):
            renderer.update_scene(d, camera=cam)
            ims.append(renderer.render().copy())
        shots.append((label, ims))

    vid = None
    if video:                    # 2 x 2 views, 25 frames per simulated second (real-time playback)
        import cv2
        vr = mujoco.Renderer(m, 360, 640)
        vid = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"mp4v"), 25, (1280, 760))
        vstate = dict(next=0.0, phase="start")
        top_name_v = f"plate{n - 1}"

        def grab():
            if d.time < vstate["next"]:
                return
            vstate["next"] += 0.04
            ims = []
            for cam, lab in (("closefront", "close-up from the front"), ("close", "close-up from the robot's left"),
                             ("left", "from the left"), ("top", "from above (robot at the top)")):
                vr.update_scene(d, camera=cam)
                im = cv2.cvtColor(vr.render(), cv2.COLOR_RGB2BGR)
                cv2.putText(im, lab, (10, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                ims.append(im)
            F, _, who = hand_force()
            low = max([v for k, v in who.items() if k[1].startswith("plate") and k[1] != top_name_v] + [0.0])
            tab = max([v for k, v in who.items() if k[1] == "world"] + [0.0])
            topf = max([v for k, v in who.items() if k[1] == top_name_v] + [0.0])
            slide_mm = float(np.linalg.norm((d.xpos[pb[n - 1]] - (d.xpos[pb[n - 2]] if n > 1 else np.array([p['x'], p['y'], 0.0])))[:2])) * 1000
            bar = np.full((40, 1280, 3), 30, np.uint8)
            cv2.putText(bar, title[:150], (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            col = (60, 60, 255) if low > 0.5 or tab > 0.5 else (120, 220, 120)
            txt = (f"t {d.time:4.1f} s  {vstate['phase']:<8s} top plate slid {slide_mm:4.0f} mm   finger on top plate {topf:4.1f} N   "
                   f"on plate below {low:4.1f} N   on table {tab:4.1f} N")
            frame = np.vstack([bar, np.hstack(ims[:2]), np.hstack(ims[2:])])
            cv2.rectangle(frame, (0, 40), (1280, 64), (0, 0, 0), -1)
            cv2.putText(frame, txt, (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            vid.write(frame)

    log = []
    edge_local = spec.edge_c[0] * out + np.array([0, 0, spec.edge_c[1]])     # pushed rim-edge centre in the plate frame

    def ref_above_edge():
        """mm by which the finger (its end, or for a side push its lowest point) is above the pushed rim-edge centre."""
        pt, Rt = d.xpos[pb[n - 1]], d.xmat[pb[n - 1]].reshape(3, 3)
        e, low = ik.end(d)
        return ((low if side else e[2]) - (pt + Rt @ edge_local)[2]) * 1000

    def move(qa, qb, T, rec=None, marks=()):
        N = max(1, int(T / dt))
        for k in range(N):
            s = (k + 1) / N
            command(qa + (qb - qa) * s)
            mujoco.mj_step(m, d)
            fm.update(d)
            if vid is not None:
                vstate["phase"] = rec or vstate["phase"]
                grab()
            if rec is not None and k % 10 == 0:
                F, Fpad, who = hand_force()
                ps = poses()
                log.append(dict(phase=rec, t=d.time, F=F, Fpad=Fpad, who=who, top=ps[n - 1][0], sense=pad_sense(),
                                edge=ps[n - 1][0] + ps[n - 1][1] @ (edge_local), tipz=float(ik.pad(d)[0][2]), ref=ref_above_edge(),
                                tilt=math.degrees(math.acos(min(1.0, ps[n - 1][1][2, 2])))))
            if k in marks:
                shoot(f"{rec} {100 * k // N}%")

    # guarded push controller, using only signals the robot has: joint angles and gains -> finger force
    # F = pinv(J^T) (kp (q_cmd - q) - kv dq) minus its free-motion value; camera plate position (cam_noise mm, 30 Hz).
    # 1 touch-down cal_r mm inside the edge until the force rises (f_on N); shift the push path by the height error
    # 2 guarded approach toward the edge until the force rises
    # 3 push along the planned edge-drop path; finger ride_mm ahead of the plate -> back out, retry retry_dz mm lower;
    #   force > f_hi -> retry higher
    arm_dof = [m.jnt_dofadr[m.joint(j).id] for j in meta["arm"]["left"]]
    arm_qadr = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"]["left"]]
    kp_arm, kv_arm = m.actuator_gainprm[arm_act, 0].copy(), -m.actuator_biasprm[arm_act, 2].copy()
    jp_ = np.zeros((3, m.nv))
    rng_c = np.random.default_rng(abs(int(1000 * (p["cam_x"] + 7 * p["cam_y"] + 13 * p["cam_z"]))) % 2 ** 31)

    def f_est():
        tau = kp_arm * (d.ctrl[arm_act] - d.qpos[arm_qadr]) - kv_arm * d.qvel[arm_dof]
        e_, _ = ik.end(d)
        mujoco.mj_jac(m, d, jp_, None, e_, ik.end_body)
        J = jp_[:, arm_dof]                                       # 3 x 5
        return np.linalg.lstsq(J.T, -tau, rcond=None)[0]          # force of the world on the fingertip

    def cam_plate():
        return d.xpos[pb[n - 1]][:2] + rng_c.normal(0, p["cam_noise"] / 1000, 2)

    def st_pt(st):
        e_, _ = ik.end(d)
        return e_.copy()

    def guarded_push():
        info = dict(retries=0, events=[])
        st = dict(x=qs[0].copy())

        def go(pt, nrm_w, T_):
            r_ = ik.solve(pt, nrm_w, [st["x"]], base=base, **ax_kw)
            if r_["pos_err_mm"] < 2.0 and r_["ang_err_deg"] < 4.0:
                move(st["x"], r_["x"], T_, rec="push")
                st["x"] = r_["x"]
                return True
            move(st["x"], st["x"], T_, rec="push")
            return False
        dz = 0.0
        shift = np.zeros(3)

        def find_edge(top_c):
            """cal 2 edge finding around a plate-centre estimate top_c; returns (shift, dz, found). also used
            again after a ride-over, at the plate's new place (camera progress), instead of guessing the edge."""
            shift, dz, found = np.zeros(3), 0.0, False
            # edge finding: touch down cal_r2 mm inside the estimated edge, slide outward pressing lightly (f_slide N) until the
            # finger drops off; the last contact gives the edge position and height
            rc = spec.R - p["cal_r2"] / 1000
            C = top_c + rc * out + np.array([0, 0, spec.z_top(rc) + 0.008])
            ok = go(C, nrm, 0.8)
            move(st["x"], st["x"], 0.2)
            base_f = f_est()
            zc, hit = C[2], False
            while ok and zc > C[2] - 0.020:
                zc -= 0.00005
                ok = go(np.array([C[0], C[1], zc]), nrm, 0.01)
                if (f_est() - base_f)[2] > p["f_on"]:
                    hit = True
                    break
            if not hit:
                info["events"].append("edge finding: no touch-down")
            else:
                ro, off_n, last_c, trace = 0.0, 0, None, []
                dt_s = 0.01
                while ok and ro < p["cal_r2"] + 12 and off_n < 5:
                    ro += p["slide_v"] * dt_s
                    fz = float((f_est() - base_f)[2])
                    zc -= p["slide_k"] * (p["f_slide"] - fz) / 1000          # mm per N per 10 ms
                    ok = go(np.array([C[0], C[1], zc]) + out * ro / 1000, nrm, dt_s)
                    e_, low = ik.end(d)
                    r_e = float((e_[:2] - top_c[:2]) @ out[:2])              # measured fingertip end, along the push direction
                    trace.append((ro, r_e, low, fz))
                    if fz > 0.3:
                        off_n, last_c = 0, True
                    elif ro > 1.0:
                        off_n += 1
                info.setdefault("edge_traces", []).append( [(round(a_, 2), round(1000 * (b_ - spec.R), 2), round(1000 * (c_ - top_c[2]), 2), round(f_, 2)) for a_, b_, c_, f_ in trace[::5]])
                on = [i_ for i_, t_ in enumerate(trace) if t_[3] > 0.3]
                if last_c is None or not on:
                    info["events"].append("edge finding: lost contact at once")
                else:
                    # the rim's highest point under the finger before it falls = the edge top (height); the edge position =
                    # where the finger has first fallen 0.3 mm below it (both from the arm's joint angles)
                    i_max = max(on, key=lambda i_: trace[i_][2])
                    zmax = trace[i_max][2]
                    i_e = next((i_ for i_ in range(i_max, len(trace)) if trace[i_][2] < zmax - 0.0003), len(trace) - 1)
                    r_meas, ro = trace[i_e][1], trace[i_e][0]
                    dr = r_meas - spec.R - p["cal2_off_r"] / 1000             # > 0: the edge is further out than estimated
                    dz = zmax - (top_c[2] + spec.z_top(spec.R)) - p["cal2_off_z"] / 1000
                    shift = dr * np.array([out[0], out[1], 0.0]) / np.linalg.norm(out[:2])
                    info["events"].append(f"edge found {1000 * dr:+.2f} mm along the push direction, height {1000 * dz:+.2f} mm (slid {ro:.1f} mm)")
                    info.setdefault("edge_dr_mm", 1000 * dr)
                    info.setdefault("edge_dz_mm", 1000 * dz)
                    found = True
            info["cal_true_err_mm"] = dict(x=-p["cam_x"], y=-p["cam_y"], z=-p["cam_z"])
            return shift, dz, found

        if p["cal"] == 2:
            shift, dz, _ = find_edge(top)
            go(st_pt(st) + np.array([0, 0, 0.008]), nrm, 0.4)
        elif p["cal"]:
            rc = spec.R - p["cal_r"] / 1000
            C = top + rc * out + np.array([0, 0, spec.z_top(rc) + 0.008])
            ok = go(C, nrm, 0.8)
            move(st["x"], st["x"], 0.2)
            base_f = f_est()
            zc = C[2]
            hit = False
            while ok and zc > C[2] - 0.020:
                zc -= 0.00005                                    # 5 mm/s, 10 ms steps
                ok = go(np.array([C[0], C[1], zc]), nrm, 0.01)
                if (f_est() - base_f)[2] > p["f_on"]:
                    hit = True
                    break
            e_, low = ik.end(d)
            if hit:
                # height error of the plate estimate: where the finger's lowest point touched vs the plate model there
                rr = float(np.hypot(e_[0] - top[0], e_[1] - top[1]))
                dz = low - (top[2] + spec.z_top(min(rr, spec.R))) - p["cal_off"] / 1000   # cal_off: the method's own bias (finger geometry)
                info["events"].append(f"touch-down: plate {1000 * dz:+.2f} mm off the estimate")
            else:
                info["events"].append("touch-down: no contact")
            info["cal_dz_mm"], info["cal_true_err_mm"] = 1000 * dz, -p["cam_z"]
            go(np.array([C[0], C[1], zc + 0.006]), nrm, 0.4)
        h = dz + p["lift"] / 1000
        P_pre = P0 + shift - 0.025 * nrm + np.array([0, 0, h])
        go(P_pre + np.array([0, 0, 0.01]), nrm, 0.6)
        go(P_pre, nrm, 0.4)
        move(st["x"], st["x"], 0.2)
        base_f = f_est()
        s, s_c, s0, contact, start = 0.0, 0.0, 0.0, None, P_pre.copy()
        pl0 = cam_plate()
        dt_c = 0.02
        last = (P_pre, nrm)
        nrm_i = nrm
        corr, prog = 0.0, 0.0
        info["corr_mm"] = []
        while True:
            if contact is None:                                  # guarded approach: in until the force rises
                s_c += p["speed"] * dt_c
                pt = start + (nrm if s == 0 else u_push) * s_c / 1000      # first approach along the pointing direction, as the plan
                go(pt, nrm_i, dt_c)
                last = (pt, nrm_i)
                if -(f_est() - base_f) @ u_push > p["f_on"]:
                    contact = pt.copy()
                    info["events"].append(f"contact after {s_c:.1f} mm (t {d.time:.2f} s)")
                    if p["press_g"] > 0:
                        # preload as the open-loop push (press 3 mm into the edge along the pointing
                        # direction; the arm's stiffness makes the force) before pushing; without it the finger rode over
                        contact = contact + nrm_i * p["press_g"] / 1000
                        go(contact, nrm_i, 0.3)
                    if s == 0 and p["sc_nom"] > 0 and p["cal"]:
                        # where the edge really is (from the approach) moves the touch-down point along the sloped rim top:
                        # correct the measured height by that radial error x the rim slope at the touch-down radius
                        dr = (s_c - p["sc_nom"]) * math.cos(math.radians(p["gamma"])) / 1000     # > 0: edge further in than estimated
                        rc_ = spec.R - p["cal_r"] / 1000
                        slope = (spec.z_top(rc_ + 0.001) - spec.z_top(rc_ - 0.001)) / 0.002
                        dzr = -dr * slope
                        contact = contact + np.array([0, 0, dzr])
                        h += dzr
                        info["events"].append(f"edge {1000 * dr:+.1f} mm from the estimate -> height {1000 * dzr:+.2f} mm")
                elif s_c > 45:
                    info["events"].append("no contact within 45 mm")
                    if p["retry_find"] == 2 and p["cal"] == 2 and not info.get("refound") and s > 0:
                        # the retry approach missed the edge: find it again where the camera now sees the plate
                        info["refound"] = True
                        go(pt + np.array([0, 0, 0.012]), nrm_i, 0.3)
                        top_c = top - np.array([out[0], out[1], 0.0]) / np.linalg.norm(out[:2]) * prog / 1000
                        sh_, dz_, ok_ = find_edge(top_c)
                        go(st_pt(st) + np.array([0, 0, 0.008]), nrm, 0.4)
                        if ok_:
                            start = P0 + (top_c - top) + sh_ + np.array([0, 0, dz_ + h - dz]) + 0.008 * out
                            go(start + np.array([0, 0, 0.010]), nrm_i, 0.3)
                            go(start, nrm_i, 0.3)
                            move(st["x"], st["x"], 0.1)
                            base_f = f_est()
                            s_c = 0.0
                            continue
                    break
                continue
            s += p["speed"] * dt_c
            g_ = math.radians(p["gamma"] + p["roll"] * min(1.0, s / p["dist"]))
            nrm_i = -math.cos(g_) * out - math.sin(g_) * up
            pt = contact + u_push * (s - s0) / 1000 - up * p["follow"] * (
                float(np.interp(s, EDGE_S, EDGE_DROP)) - float(np.interp(s0, EDGE_S, EDGE_DROP))) / 1000 + up * corr
            go(pt, nrm_i, dt_c)
            last = (pt, nrm_i)
            prog = float((cam_plate() - pl0) @ -out[:2]) * 1000
            fv = f_est() - base_f
            F = float(np.linalg.norm(fv))
            # continuous height correction from real signals: the finger running ahead of the plate (arm position vs the
            # camera) or an upward push on the finger (arm deflection) means it is climbing the edge -> lower it
            corr -= (p["lag_k"] * max(0.0, (s - prog) - p["lag0"]) + p["fz_k"] * max(0.0, fv[2] - p["fz0"])) / 1000 * dt_c / 0.02
            corr = float(np.clip(corr, -0.008, 0.003))
            info["corr_mm"].append(round(1000 * corr, 2))
            # done_mm: stop once the camera sees the plate this far (a full slide needs 40 mm); without it the
            # controller kept retrying after 42-48 mm and jammed the pinky (106-305 N spikes in edge10)
            if prog >= (p["done_mm"] or p["dist"] - 5) or s > p["dist"] + 25:
                info["events"].append(f"done: finger {s:.0f} mm, plate {prog:.0f} mm (camera)")
                break
            bad = "rode over" if s - prog > p["ride_mm"] else ("too hard" if F > p["f_hi"] else "")
            if bad:
                info["retries"] += 1
                info["events"].append(f"{bad} at finger {s:.0f} / plate {prog:.0f} mm, F {F:.1f} N")
                if info["retries"] > p["retries"]:
                    break
                h_new = -p["retry_dz"] / 1000 if bad == "rode over" else p["retry_dz"] / 1000
                if p["retry_find"] == 1 and p["cal"] == 2:
                    # lift clear, find the edge again where the camera now sees the plate, restart 8 mm outside it
                    go(pt + np.array([0, 0, 0.012]), nrm_i, 0.3)
                    top_c = top - np.array([out[0], out[1], 0.0]) / np.linalg.norm(out[:2]) * prog / 1000
                    sh_, dz_, ok_ = find_edge(top_c)
                    go(st_pt(st) + np.array([0, 0, 0.008]), nrm, 0.4)
                    if ok_:
                        E_ = P0 + (top_c - top) + sh_ + np.array([0, 0, dz_ + p["lift"] / 1000])
                        start = E_ + 0.008 * out
                        go(start + np.array([0, 0, 0.010]), nrm_i, 0.3)
                        go(start, nrm_i, 0.3)
                        move(st["x"], st["x"], 0.1)
                        base_f = f_est()
                        contact, s_c, s0, s = None, 0.0, prog, prog
                        h = dz_ + p["lift"] / 1000
                        continue
                go(pt + 0.008 * out + np.array([0, 0, 0.004]), nrm_i, 0.3)
                # restart outside the EDGE (where the camera sees the plate), not outside the finger, which ran ahead
                edge_now = contact + u_push * (prog - s0) / 1000 - up * p["follow"] * (
                    float(np.interp(prog, EDGE_S, EDGE_DROP)) - float(np.interp(s0, EDGE_S, EDGE_DROP))) / 1000
                start = edge_now + 0.008 * out + np.array([0, 0, h_new])
                go(start, nrm_i, 0.3)
                move(st["x"], st["x"], 0.1)
                base_f = f_est()
                contact, s_c, s0, s = None, 0.0, prog, prog
                h += h_new
        info["height_offset_mm"] = 1000 * h
        return [st["x"]], last[0], last[1], info

    move(q_park, q_park, 0.6)
    p0 = poses()
    if vid is not None:
        vstate["next"], vstate["phase"] = d.time, "approach"
    move(q_park, qs[0], 0.8)
    shoot("before")
    ctl = {}
    if p["ctrl"]:
        qs, last_pt, last_nrm, ctl = guarded_push()
        fb_miss, corr_log = 0, []
    else:
        move(qs[0], qs[1], 1.0, rec="touch")
        last_pt, last_nrm, fb_miss, corr_log = way[-1][1], way[-1][2], 0, []
    shoot("touch")
    seg_T = p["dist"] / p["speed"] / steps
    if p["ctrl"]:
        pass
    elif p["fb"] > 0:              # height feedback: re-plan the hand height every 20 ms from the measured heights
        K = int(p["dist"] / p["speed"] / 0.02)
        corr, x, s = 0.0, qs[1].copy(), 0.0
        for k in range(3 * K):
            if p["fbsrc"] == 1:
                # from the fingertip's own force: the direction of the top plate's push on the finger (the touch sensor
                # gives normal force and shear direction). Finger above the edge centre -> pushed up -> come down.
                _, Fp, _ = hand_force()
                fh = float(Fp @ out)                     # force of the plate on the finger points outward
                if np.linalg.norm(Fp) > 0.3 and fh > 0.1:
                    a = math.degrees(math.atan2(float(Fp[2]), fh))
                    err = -(a - p["a0"]) * p["fbf"] / p["fb"]
                else:
                    err = 0.0
            else:
                err = p["hold"] - ref_above_edge() + p["fb_err"]
            # --gate G: the hand moves in only as fast as the height is right (stops when it is G mm off)
            s += p["dist"] / K * (max(0.0, 1.0 - abs(err) / p["gate"]) if p["gate"] > 0 else 1.0)
            if s > p["dist"]:
                break
            corr = float(np.clip(corr + p["fb"] * err, -15.0, 15.0))
            g = math.radians(p["gamma"] + p["roll"] * s / p["dist"])
            last_nrm = -math.cos(g) * out - math.sin(g) * up
            last_pt = (P0 + p["press"] / 1000 * nrm + lift + u_push * s / 1000
                       - up * p["follow"] * float(np.interp(s, EDGE_S, EDGE_DROP)) / 1000 + up * corr / 1000)
            r = ik.solve(last_pt, last_nrm, [x], base=base, **ax_kw)
            xn = r["x"] if r["pos_err_mm"] < 2.0 and r["ang_err_deg"] < 4.0 else x
            fb_miss += xn is x
            move(x, xn, 0.02, rec="push", marks=(0,) if k in (K // 3, 2 * K // 3) else ())
            if k >= K and p["gate"] <= 0:
                break
            x = xn
            corr_log.append(corr)
        qs.append(x)
    else:
        for i in range(steps):
            move(qs[1 + i], qs[2 + i], seg_T, rec="push", marks=(0,) if i in (steps // 3, 2 * steps // 3) else ())
    move(qs[-1], qs[-1], 0.4, rec="hold")
    shoot("end of push")
    if save:                     # the state with the finger still on the rim: where the other hand's grip starts
        np.savez(save, qpos=d.qpos, qvel=d.qvel, ctrl=d.ctrl, time=d.time, par=json.dumps(p), p0=np.array([q[0] for q in p0]),
                 x_end=qs[-1], pt_end=last_pt, nrm_end=last_nrm)
    ps = poses()
    res = dict(par=p, q_touch=[float(v) for v in qs[min(1, len(qs) - 1)]], wrist_clearance_mm=clear * 1000, force=fm.summary(),
               arm_gains=dict(kp=os.environ.get("DEXPICK_KP_ARM", "200"), kv=os.environ.get("DEXPICK_KV_ARM", "20")))
    if ctl:
        res["ctrl"] = ctl
    if p["fb"] > 0:
        res["fb"] = dict(push_s=len(corr_log) * 0.02, ik_misses=int(fb_miss), corr_min_mm=min(corr_log), corr_max_mm=max(corr_log), corr_end_mm=corr_log[-1])
    res.update(opening(spec, ps, n))
    pt, Rt = ps[n - 1]
    res["tilt"] = math.degrees(math.acos(min(1.0, Rt[2, 2])))
    res["top_move"] = [float(v) * 1000 for v in (pt - p0[n - 1][0])]
    res["low_shift"] = max([float(np.linalg.norm(ps[k][0][:2] - p0[k][0][:2])) for k in range(n - 1)] + [0.0]) * 1000
    if n == 1:          # lone plate: no plate below - the slide is how far the plate moved on the table
        res["slide"] = float(np.linalg.norm((pt - p0[0][0])[:2])) * 1000
    push = [r for r in log if r["phase"] == "push"]
    res["F_side_mean"] = float(np.mean([-(r["F"] @ out) for r in push]))       # hand pushing toward the plate centre
    res["F_down_mean"] = float(np.mean([r["F"][2] for r in push]))
    res["F_down_peak"] = float(np.max([r["F"][2] for r in push]))
    res["ref_above_edge_mm"] = [float(np.min([r["ref"] for r in push])), float(np.max([r["ref"] for r in push]))]
    res["F_touch"] = float(np.linalg.norm(([r for r in log if r["phase"] == "touch"] or [dict(F=np.zeros(3))])[-1]["F"]))
    res["pad_share"] = float(np.mean([np.linalg.norm(r["Fpad"]) / max(np.linalg.norm(r["F"]), 1e-6) for r in push]))
    # the pads: share of the push in which each one sees a plate within 10 mm along its normal (and the distances),
    # and the largest normal force on it
    res["pad_sense"] = {f: dict(sees_plate_10mm=float(np.mean([(r["sense"][f][0] or 99) < 10 for r in push])),
                                dist_mm=[round(float(np.min([r["sense"][f][0] or 99 for r in push])), 1), round(float(np.max([r["sense"][f][0] or 99 for r in push])), 1)],
                                F_max_N=round(float(np.max([r["sense"][f][1] for r in push])), 2),
                                sees=sorted({r["sense"][f][2] for r in push if r["sense"][f][0] is not None}))
                        for f in pads} if push else {}
    # per log step of the push: time, top plate moved (mm, horizontal), fingertip end above the top plate's
    # pushed edge (mm), force of the hand on the plates below and on the top plate (N)
    p_top0 = p0[n - 1][0]
    res["push_trace"] = [(round(r["t"], 3), round(float(np.linalg.norm((r["top"] - p_top0)[:2])) * 1000, 2), round(float(r["ref"]), 2),
                          round(sum(v for k, v in r["who"].items() if k[1].startswith("plate") and k[1] != f"plate{n - 1}"), 2),
                          round(float(np.linalg.norm(r["Fpad"])), 2)) for r in log if r["phase"] == "push"]
    res["pad_trace"] = [dict(t=round(r["t"], 2), **{f: [None if v[0] is None else round(v[0], 1), round(v[1], 2)] for f, v in r["sense"].items()})
                        for r in log if r["phase"] in ("touch", "push")][::3]
    who = {}
    for r in push:
        for k, v in r["who"].items():
            who[k] = max(who.get(k, 0.0), v)
    top_name = f"plate{n - 1}"
    res["lower_touch_N"] = max([v for k, v in who.items() if k[1].startswith("plate") and k[1] != top_name] + [0.0])
    res["table_touch_N"] = max([v for k, v in who.items() if k[1] == "world"] + [0.0])
    res["lower_touch_where"] = dict(where)
    res["touched"] = {f"{k[0]}>{k[1]}": round(v, 1) for k, v in sorted(who.items(), key=lambda kv: -kv[1])}
    # let go: back out along the pad normal and up
    x = qs[-1]
    r = ik.solve(last_pt - 0.03 * nrm + np.array([0, 0, 0.02]), last_nrm, [x], base=base)
    if vid is not None:
        vstate["phase"] = "let go"
    move(qs[-1], r["x"], 0.8)
    move(r["x"], r["x"], 0.6)
    ps2 = poses()
    res["after_release"] = opening(spec, ps2, n)
    res["after_release"]["tilt"] = math.degrees(math.acos(min(1.0, ps2[n - 1][1][2, 2])))
    shoot("finger released")
    if vid is not None:
        vid.release()
    res["trace"] = [dict(t=round(r["t"], 2), phase=r["phase"], move=[round(float(v) * 1000, 1) for v in (r["top"] - p0[n - 1][0])[:2]],
                         tilt=round(r["tilt"], 1), F=[round(float(v), 1) for v in r["F"]],
                         edge_in=round(float(-(r["edge"] - log[0]["edge"]) @ out) * 1000, 1),
                         edge_drop=round(float(log[0]["edge"][2] - r["edge"][2]) * 1000, 1),
                         tip_above_edge=round((r["tipz"] - float(r["edge"][2])) * 1000, 1), ref=round(float(r["ref"]), 1),
                         on_lower=round(max([v for k, v in r["who"].items() if k[1].startswith("plate") and k[1] != f"plate{n - 1}"] + [0.0]), 1),
                         on_top=round(max([v for k, v in r["who"].items() if k[1] == f"plate{n - 1}"] + [0.0]), 1)) for r in log[::20]]
    if shots:
        res["_shots"] = shots
    return res


def sheet(res, path):
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
    p = res["par"]
    what = {"tip": "fingertip end", "pad": "finger pad"}.get(p["mode"], "finger side")
    fig.suptitle(f"MuJoCo, R1 left hand, {p['finger']} {what} on the rim at {p['phi']:.0f} deg, pointing {p['gamma']:.0f} deg down, "
                 f"{p['n']} plates, table {p['table']} m | slid {res['slide']:.0f} mm, tilt {res['tilt']:.1f} deg, reach under "
                 f"the leading rim {res['grip_depth']:.0f} mm, plates below moved {res['low_shift']:.1f} mm", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


SWEEPS = {
    # which finger, where on the rim, how steeply the fingertip points down, push slope
    "pose": dict(finger=["index", "middle", "pinky"], phi=[0, 20, 40], gamma=[0, 20, 40, 60], beta=[0, 15], n=[3]),
    # around the first working push (pinky fingertip on the leftmost rim point): both tables, short and tall stacks
    # rim-following path: how exactly to follow the edge's drop, where to aim, and wrist roll during the push
    "follow": dict(gamma=[15, 20, 25, 30], roll=[0, 10], follow=[0.7, 1.0, 1.3], lift=[-1, 0, 1, 2], n=[2, 5],
                   table=[0.747, 0.762]),
    # the arm gives way upward under the push; command the fingertip `bias` mm lower over the first 10 mm to make up
    # for it, then follow the edge
    "bias": dict(gamma=[20, 25], bias=[2, 3, 4, 5, 6, 8], follow=[0.6, 1.0], n=[2, 3, 5], table=[0.747, 0.762]),
    # does a grippier fingertip remove the need to press down (and so the contact with the plate below)?
    "mu": dict(pad_glaze=[0.25, 0.4, 0.6], gamma=[20, 30, 40], bias=[0, 2, 4], lift=[0, 2], n=[2, 5]),
    # index (or middle) curled down, its side pushes the rim; gamma = how far the finger's side leans
    "side": dict(mode=["side-"], finger=["index", "middle"], follow=[0.0], gamma=[0, 10, 20, 30], phi=[0, 15], beta=[0, 5],
                 n=[2, 3, 5], table=[0.747, 0.762], x=[0.39], y=[0.06]),
    # fingertip pointing steeply down ONTO the upper corner of the rim edge (so the top plate itself carries it),
    # pressed about bias x 1.7 N, then moved in along the sinking edge. flex 0.5 = half the finger's travel.
    "corner": dict(finger=["index", "pinky"], flex=[0.5], gamma=[35, 45, 55], lift=[2, 4, 6], bias=[2, 3, 4, 5], n=[2, 5],
                   x=[0.39], y=[0.0]),
    # hand along the rim, finger bent 0.5, fingertip pointing at the plate and downward
    "photo5": dict(finger=["index"], along=[1], flex=[0.5], gamma=[30, 45, 60], lift=[0, 2, 4], bias=[0, 2, 4], beta=[0, 13],
                   n=[2, 5], x=[0.33, 0.39], y=[0.0, 0.08]),
    # height feedback: pinky end held at the rim-edge centre...
    "fb_pinky": dict(fb=[0.15], hold=[-3, -2, -1, 0, 1, 2, 3], gamma=[10, 20, 30], press=[1, 3], n=[2, 3, 5], table=[0.747, 0.762]),
    #... and the side of the index held above the rim of the plate below
    "fb_side": dict(mode=["side-"], finger=["index"], fb=[0.15], lift=[7.0], hold=[-4, -3, -2, -1, 0, 1], gamma=[10, 20, 30],
                    press=[1, 3], n=[2, 3, 5], table=[0.747, 0.762], x=[0.39], y=[0.06]),
    # robustness of the clean pinky push found (pinky end pointing 30 deg down, held on the rim-edge centre)
    "fb_robust": dict(fb=[0.15], gamma=[25, 30, 35], hold=[-0.5, 0.0, 0.5], roll=[0, 10], n=[2, 3, 5], table=[0.747, 0.762],
                      table_mu=[0.3, 0.45, 0.6]),
    # height feedback from the fingertip's contact force direction instead of the edge height (no camera precision needed)
    "fb_force": dict(fb=[0.15], fbsrc=[1], a0=[0.0, 10.0, 15.0, 20.0, 25.0, 30.0], fbf=[0.005, 0.01, 0.02], gamma=[25, 30], roll=[10], n=[2, 3, 5],
                     table=[0.747, 0.762], table_mu=[0.6]),
    # the edge-height feedback with a camera error in the edge height
    "fb_camerr": dict(fb=[0.15], gamma=[25, 30], hold=[0.0], roll=[10], fb_err=[-1.0, -0.5, -0.25, 0.25, 0.5, 1.0], n=[3, 5],
                      table=[0.747, 0.762], table_mu=[0.6]),
    # which pointing angle / wrist roll gives the widest window of edge-height error (camera error)?
    "fb_window": dict(fb=[0.15], gamma=[20, 25, 30], hold=[0.0], roll=[10, 20, 30], fb_err=[-0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0],
                      n=[3, 5], table=[0.762], table_mu=[0.6]),
    # open-loop push along the planned edge-drop path; `lift` = error of the estimated edge height (mm)
    "open_window": dict(fb=[0.0], lift=[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0], gamma=[25, 30, 35], press=[1, 3], roll=[10], n=[2, 3, 5],
                        table=[0.762], table_mu=[0.6]),
    #... and with a grippier cap on the pushing finger only (silicone on the pinky: pad friction 0.4-0.8)
    "open_window_mu": dict(fb=[0.0], lift=[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0], gamma=[25, 35], press=[3], roll=[10], pad_glaze=[0.4, 0.6, 0.8],
                           n=[2, 3, 5], table=[0.762], table_mu=[0.6]),
    # guarded push (ctrl=1) with continuous height correction from the finger's lead (lag_k) and/or the upward finger
    # force (fz_k)
    "ctrl": dict(ctrl=[1], lag_k=[0.0, 0.1, 0.3], fz_k=[0.0, 0.1, 0.3], pad_glaze=[0.25, 0.6], cam_z=[-1.5, 0.0, 1.5], cam_x=[2.0], cam_y=[-2.0],
                 gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762], table_mu=[0.6]),
    # the soft (real-gain) arm yields upward: aim lower
    "open_window_low": dict(fb=[0.0], lift=[-6.0, -5.0, -4.0, -3.0, -2.0], gamma=[25, 30], press=[3], roll=[10], pad_glaze=[0.6, 0.8, 1.0],
                            n=[2, 3, 5], table=[0.762], table_mu=[0.6]),
    "open_window_low2": dict(fb=[0.0], pathcheck=[0], lift=[-4.0, -3.0, -2.5, -2.0, -1.5], gamma=[25], press=[3], roll=[10], pad_glaze=[0.8, 1.0],
                             n=[2, 3, 5], table=[0.747, 0.762], table_mu=[0.6]),
    # the guarded controller, refined: calibration bias removed (0.73 mm), aim below the edge centre, lead correction
    "ctrl2": dict(ctrl=[1], cal_off=[0.73], lift=[-2.0, -1.0], lag_k=[0.0, 0.05, 0.1], fz_k=[0.0], pad_glaze=[0.6, 0.8, 1.0], cam_z=[-1.5, 0.0, 1.5],
                  cam_x=[2.0], cam_y=[-2.0], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762], table_mu=[0.6]),
    # chosen recipe, wider conditions: camera error x/y/z, both tables (the planner's collision check off: the controller
    # re-measures the height by touch)
    "ctrl3": dict(ctrl=[1], pathcheck=[0], cal_off=[0.73], sc_nom=[26.6], lift=[-2.0], lag_k=[0.0], pad_glaze=[0.8, 1.0], cam_z=[-1.5, 0.0, 1.5],
                  cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.747, 0.762], table_mu=[0.6]),
    # touch-down on the flat well floor (cal_r mm inside the edge) instead of the sloped rim; planner world
    # shifted by the camera error (plan_cam). First the method's bias (cal_off 0), then the push with it.
    "ctrl4cal": dict(ctrl=[1], plan_cam=[1], pathcheck=[0], cal_off=[0.0], cal_r=[6.0, 40.0, 55.0, 65.0], lift=[-2.0], pad_glaze=[1.0],
                     cam_z=[-1.5, 0.0, 1.5], cam_x=[-3.0, 0.0, 3.0], cam_y=[-3.0, 0.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[3],
                     table=[0.762], table_mu=[0.6], retries=[0]),
    # evening: edge finding (cal 2: touch down cal_r2 mm inside, slide out until the finger falls off the edge).
    # First how well it measures the edge (offsets 0), over camera errors; real arm gains via DEXPICK_KP_ARM.
    "edge5cal": dict(ctrl=[1], cal=[2], plan_cam=[1], pathcheck=[0], lift=[-2.0], pad_glaze=[1.0], cam_z=[-1.5, 0.0, 1.5],
                     cam_x=[-3.0, 0.0, 3.0], cam_y=[-3.0, 0.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762],
                     table_mu=[0.6], retries=[0]),
    # with the measured constant offsets of the method (edge5cal: position -6.03 +-0.14 mm, height -0.60 +-0.01 mm)
    "edge5": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-2.0, -1.0],
                  pad_glaze=[0.25, 1.0], cam_z=[-1.5, 1.5], cam_x=[3.0], cam_y=[-3.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10],
                  n=[2, 3, 5], table=[0.762], table_mu=[0.6], retries=[3]),
    # push height below the MEASURED edge (edge5: -2 / -1 rode over at plate 6-12 mm at every friction)
    "edge5h": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-4.5, -4.0, -3.5, -3.0, -2.5],
                   pad_glaze=[0.25, 0.5, 1.0], cam_y=[-3.0, 0.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[3], table=[0.762],
                   table_mu=[0.6], retries=[0]),
    # with a press after the guarded contact (press_g, as the open-loop push's press 3)
    "edge6": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-2.0], press_g=[2.0, 3.0, 4.0],
                  pad_glaze=[0.25, 1.0], cam_y=[-3.0, 0.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762],
                  table_mu=[0.6], retries=[3]),
    # bare pad (no cap) around the edge6 passes (press 3-4 mm, pad 0.25: 4-5/9)
    "edge7": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-2.0, -1.5], press_g=[4.0, 5.0, 6.0],
                  pad_glaze=[0.2, 0.25, 0.3], cam_y=[-3.0, 3.0], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762],
                  table_mu=[0.6], retries=[3]),
    # bare-pad recipe from edge7 (press 5, lift -1.5: 18/18), wider: camera x/y/z, 2-5 plates, both tables, table friction
    "edge8a": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                   pad_glaze=[0.2, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10],
                   n=[2, 3, 4, 5], table=[0.747], table_mu=[0.3, 0.8], retries=[3]),
    "edge8b": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                   pad_glaze=[0.2, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10],
                   n=[2, 3, 4, 5], table=[0.762], table_mu=[0.3, 0.8], retries=[3]),
    # the fingertip climbs the edge as the plate slides (A in push_bare_fail_explained.png): let the path sink
    # faster than the measured edge-drop profile (follow > 1), and/or lower the finger on an upward force from the arm
    "edge9": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                  follow=[1.0, 1.3, 1.6], fz_k=[0.0, 0.05], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5],
                  f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 5], table=[0.762], table_mu=[0.8], retries=[3]),
    # edge9 best (follow 1.3: 17/18): wide check on a high-friction table (0.8)
    "edge10a": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5], f_hi=[20.0],
                    gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.747], table_mu=[0.8], retries=[3]),
    "edge10b": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5], f_hi=[20.0],
                    gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.762], table_mu=[0.8], retries=[3]),
    # edge10 + stop at 42 mm of plate travel
    "edge11a": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5],
                    f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.747], table_mu=[0.8], retries=[3]),
    "edge11b": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0], cam_z=[-1.5, 1.5],
                    f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.762], table_mu=[0.8], retries=[3]),
    # edge11 + after a ride-over find the edge again at the plate's new place (retry_find)
    "edge12a": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], retry_find=[1], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0],
                    cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.747], table_mu=[0.8], retries=[3]),
    "edge12b": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], retry_find=[1], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0],
                    cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.762], table_mu=[0.8], retries=[3]),
    # edge11 + retry_find 2 (fixed): re-find the edge only when a retry approach misses it
    "edge13a": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], retry_find=[2], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0],
                    cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.747], table_mu=[0.8], retries=[3]),
    "edge13b": dict(ctrl=[1], cal=[2], cal2_off_r=[-6.03], cal2_off_z=[-0.60], plan_cam=[1], pathcheck=[0], lift=[-1.5], press_g=[5.0],
                    follow=[1.3], done_mm=[42.0], retry_find=[2], pad_glaze=[0.2, 0.25, 0.3], cam_x=[-3.0, 3.0], cam_y=[-3.0, 3.0],
                    cam_z=[-1.5, 1.5], f_hi=[20.0], gamma=[25], roll=[10], n=[2, 3, 4, 5], table=[0.762], table_mu=[0.8], retries=[3]),
    "near": dict(finger=["pinky"], phi=[0], follow=[0.0], gamma=[10, 20, 30], beta=[10, 15, 20], press=[1, 3, 5], n=[2, 5], table=[0.747, 0.762]),
}


def _safe(par):
    try:
        r = run(par)
    except Exception as ex:
        r = dict(par=dict(DEFAULTS, **par), error=repr(ex))
    r.pop("_shots", None)
    r.pop("trace", None)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sheet", default=None)
    ap.add_argument("--video", default=None, help="write an mp4 of the run (4 views, real time)")
    ap.add_argument("--title", default="", help="caption for the video")
    ap.add_argument("--save", default=None, help="save the simulation state at the end of the push (.npz)")
    ap.add_argument("--out", default=None)
    ap.add_argument("--sweep", default=None, choices=list(SWEEPS))
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--tag", default="v1")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    over = {k: getattr(a, k) for k in DEFAULTS if getattr(a, k) is not None}
    if a.sweep:
        import itertools
        from multiprocessing import Pool
        grid = SWEEPS[a.sweep]
        pars = [dict(over, **dict(zip(grid, v))) for v in itertools.product(*grid.values())]
        t0 = time.time()
        with Pool(a.workers) as pool:
            out = pool.map(_safe, pars, chunksize=1)
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(root, "data/results/robot", f"push_{a.sweep}_{a.tag}.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        json.dump(dict(simulator="MuJoCo " + mujoco.__version__, keys=list(grid), results=out), open(path, "w"))
        print(f"{len(out)} pushes in {time.time() - t0:.0f} s -> {path}")
        return
    t0 = time.time()
    res = run(over, frames=bool(a.sheet), video=a.video, title=a.title, save=a.save)
    if "error" in res:
        print("ERROR:", res["error"])
        return
    if a.sheet:
        sheet(res, a.sheet)
    res.pop("_shots", None)
    tr = res.pop("trace")
    print(json.dumps(res, indent=1))
    for r in tr:
        print("  ", r)
    print(f"wall time {time.time() - t0:.0f} s")
    if a.out:
        json.dump(dict(res, trace=tr), open(a.out, "w"))


if __name__ == "__main__":
    main()
