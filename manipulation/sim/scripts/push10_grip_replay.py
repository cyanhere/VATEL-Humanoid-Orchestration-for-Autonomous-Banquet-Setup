"""Recreate a robot push10_grip run in MuJoCo, driven by the logged commands, to compare with the robot.

    python scripts/push10_grip_replay.py -o data/results/push10_grip_sim/replay_145724 [--plate_xy 0.3545 0.0385] [--frames]
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT = os.path.dirname(ROOT)
os.environ.setdefault("DEXPICK_KP_ARM", "50,50,40,40,30")
os.environ.setdefault("DEXPICK_KV_ARM", "2")
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402

from platepick.plate import Friction  # noqa: E402
from platepick.r1_server_sim import FULL_SIM, ServerSim  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402
from platepick.settle import settle_plates  # noqa: E402

SIDES = ("left", "right")
LOG = os.path.join(PROJECT, "data", "robot_pick_logs", "logs", "server_20261106_143244.csv")
J = ["sh_pitch", "sh_roll", "sh_yaw", "elbow", "wrist_roll"]
HSRV = ["Thumb", "Thumb_aux", "Index", "Middle", "Ring", "Pinky"]       # service order
SRV2SIM = [1, 0, 2, 3, 4, 5]                                           # sim channel i = service channel SRV2SIM[i]


def sim_hand(row, s):
    srv = np.array([float(row["hcmd_%s_%s" % (s, k)]) for k in HSRV])
    return srv[SRV2SIM] * FULL_SIM


def log_rows(wall0, t_from, t_to):
    out = []
    for r in csv.DictReader(open(LOG)):
        t = float(r["wall"]) - wall0
        if t_from <= t <= t_to:
            r["_t"] = t
            out.append(r)
    return out


def plate_geom(m, d, body, spec, P0):
    """Rim heights (pelvis frame, mm) at the right (-y), left (+y), front, back of the plate, tilt deg and its direction."""
    R = d.xmat[body].reshape(3, 3)
    p = d.xpos[body]
    out = {}
    for name, a in (("right", -math.pi / 2), ("left", math.pi / 2), ("front", 0.0), ("back", math.pi)):
        loc = np.array([spec.R * math.cos(a), spec.R * math.sin(a), spec.H])
        out[name] = round(1000 * float((R @ loc + p)[2] - P0[2]), 1)
    zax = R[:, 2]
    out["tilt_deg"] = round(math.degrees(math.acos(min(1.0, zax[2]))), 2)
    out["tilt_dir_deg"] = round(math.degrees(math.atan2(-zax[1], -zax[0])), 1)   # direction the plate rises toward
    out["centre_mm"] = [round(1000 * float(v), 1) for v in (p - P0)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--wall0", type=float, default=1480.0, help="test start in the server log's wall clock (s)")
    ap.add_argument("--stack_xy", type=float, nargs=2, default=(0.35589, 0.08132), help="located stack, pelvis frame")
    ap.add_argument("--plate_xy", type=float, nargs=2, default=(0.3545, 0.0385), help="top plate after the push (camera)")
    ap.add_argument("--t_grip", type=float, default=31.5, help="s: the R1 state that starts the grip (end of 'left stop')")
    ap.add_argument("--t_end", type=float, default=53.3)
    ap.add_argument("--pelvis", type=float, default=0.71)
    ap.add_argument("--table", type=float, default=0.755)
    ap.add_argument("--pad", type=float, default=0.25)
    ap.add_argument("--table_mu", type=float, default=0.45)
    ap.add_argument("--frames", action="store_true", help="render 4 camera views at every phase change")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    t_wall = time.time()

    fr = Friction(pad_glaze=a.pad, table=a.table_mu)
    sc = RobotScene(n=3, table=a.table, xy=tuple(a.stack_xy), pelvis=a.pelvis, fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    spec = sc.spec
    P0 = np.array([0.0, 0.0, a.pelvis])
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    top = m.body("plate2").id
    before = plate_geom(m, d, top, spec, P0)
    # the push's result: the top plate moved to where the camera saw it, lifted 12 mm, then settled (robot not colliding)
    ja = m.jnt_qposadr[m.body_jntadr[top]]
    d.qpos[ja:ja + 2] = a.plate_xy
    d.qpos[ja + 2] += 0.012
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    settle_plates(m, d, steps=3000)
    after = plate_geom(m, d, top, spec, P0)
    print("before push:", before, flush=True)
    print("after push, MuJoCo rest:", after, flush=True)

    # the R1's arms and hands at the start of the grip
    rows = log_rows(a.wall0, a.t_grip - 0.05, a.t_end)
    r0 = rows[0]
    act = {m.actuator(i).name: i for i in range(m.nu)}
    A = {s: [act[f"a_{j}"] for j in meta["arm"][s]] for s in SIDES}
    adr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"][s]] for s in SIDES}
    HA = {s: [act[f"a_{j}"] for j in meta["hand"][s]] for s in SIDES}
    C = {s: np.array(meta["coupling"][s]) for s in SIDES}
    hhi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"][s]]) for s in SIDES}
    hadr = {s: [m.jnt_qposadr[m.joint(j).id] for j in meta["hand"][s]] for s in SIDES}
    qL = np.array([float(r0["q_L_" + j]) for j in J])
    qR = np.array([float(r0["q_R_" + j]) for j in J])
    hL, hR = sim_hand(r0, "L"), sim_hand(r0, "R")
    d.qpos[adr["left"]], d.qpos[adr["right"]] = qL, qR
    d.qpos[hadr["left"]] = np.minimum(C["left"] @ hL, hhi["left"])
    d.qpos[hadr["right"]] = np.minimum(C["right"] @ hR, hhi["right"])
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    hand_geoms = {s: {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(s + "_")} for s in SIDES}
    plate_geoms = {g for g in range(m.ngeom) if m.geom_bodyid[g] == top}
    pen = min([d.contact[i].dist for i in range(d.ncon)
               if (d.contact[i].geom1 in plate_geoms and d.contact[i].geom2 in hand_geoms["left"])
               or (d.contact[i].geom2 in plate_geoms and d.contact[i].geom1 in hand_geoms["left"])] or [0.0])
    print("left hand placed at the R1's joints: deepest left-hand / top-plate penetration %.1f mm" % (-1000 * pen), flush=True)

    srv = ServerSim(m, d, meta, A, HA, C, hhi, torque_scale=1.0)
    srv.target("left", qL, hL)
    srv.target("right", np.array([float(r0["des_R_" + j]) for j in J]), hR)

    def contact_N(side):
        F, w = 0.0, np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 in plate_geoms and c.geom2 in hand_geoms[side]) or (c.geom2 in plate_geoms and c.geom1 in hand_geoms[side]):
                mujoco.mj_contactForce(m, d, i, w)
                F += float(np.linalg.norm(w[:3]))
        return F

    ren = None
    if a.frames:
        ren = mujoco.Renderer(m, 360, 480)
    shots = []

    def shot(label):
        if ren is None:
            return
        ims = []
        for cam in ("closeright", "front", "top", "left"):
            ren.update_scene(d, camera=cam)
            ims.append(ren.render())
        shots.append((label, np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])])))

    # replay: the R1's right-arm plan (des, before the server's gravity offset) and hand commands, at their times
    rec = {k: [] for k in ("t", "phase", "plate", "tilt", "right_rim", "left_rim", "F_left", "F_right", "wrist_R")}
    wr = m.body("right_wrist_roll_link").id
    t_sim0 = d.time
    k_row = 0
    last_phase = None
    n_rows = len(rows)
    while k_row < n_rows - 1:
        t_rel = a.t_grip + (d.time - t_sim0)
        while k_row < n_rows - 1 and rows[k_row + 1]["_t"] <= t_rel:
            k_row += 1
        r = rows[k_row]
        ph = r["phase"].replace("push10 ", "")
        if ph != last_phase:
            shot("%.1f s  %s" % (t_rel, ph))
            last_phase = ph
        srv.target("right", np.array([float(r["des_R_" + j]) for j in J]), sim_hand(r, "R"))
        srv.step()
        if int(round(d.time / m.opt.timestep)) % 10 == 0:
            g = plate_geom(m, d, top, spec, P0)
            rec["t"].append(t_rel)
            rec["phase"].append(ph)
            rec["plate"].append(g["centre_mm"])
            rec["tilt"].append(g["tilt_deg"])
            rec["right_rim"].append(g["right"])
            rec["left_rim"].append(g["left"])
            rec["F_left"].append(contact_N("left"))
            rec["F_right"].append(contact_N("right"))
            rec["wrist_R"].append(1000 * (d.xpos[wr] - P0))
    shot("%.1f s  end" % (a.t_grip + d.time - t_sim0))
    np.savez_compressed(os.path.join(a.out, "replay.npz"), **{k: np.array(v) for k, v in rec.items()})
    summ = dict(before=before, after_push_rest=after, left_penetration_mm=round(-1000 * pen, 1), server=srv.summary(),
                wall_s=round(time.time() - t_wall), args=vars(a))
    ph = np.array(rec["phase"])
    pl = np.array(rec["plate"])
    for p_ in ("grip in", "grip close", "grip squeeze", "grip close2", "grip hold", "grip up"):
        idx = np.where(ph == p_)[0]
        if len(idx):
            summ[p_] = dict(plate_y_mm=[float(pl[idx[0], 1]), float(pl[idx[-1], 1])], tilt=[rec["tilt"][idx[0]], rec["tilt"][idx[-1]]],
                            F_right_max=round(float(np.max(np.array(rec["F_right"])[idx])), 1),
                            F_left_max=round(float(np.max(np.array(rec["F_left"])[idx])), 1))
    json.dump(summ, open(os.path.join(a.out, "summary.json"), "w"), indent=1, default=float)
    if shots:
        import cv2
        for i, (label, im) in enumerate(shots):
            im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            cv2.putText(im, "MuJoCo  " + label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(a.out, "frame_%02d.jpg" % i), im)
    print(json.dumps(summ, indent=1, default=float))


if __name__ == "__main__":
    main()
