"""Replay a real push10 run in MuJoCo and compare the top plate's motion with the robot's.

    python scripts/friction_replay.py RUN -o OUT --pad 1.0 --fg 0.575 --table_mu 0.9 --dy 0 [--frames]
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
LOGS = os.path.join(PROJECT, "data", "robot_pick_logs", "logs", "tests")
SRV2SIM = [1, 0, 2, 3, 4, 5]
Q_RIGHT = np.array([-0.00801, -0.87741, -0.29047, -0.77631, 0.00795])   # R1 right arm parked (robot log)
FREE_PHASES = ("pre", "search", "release")
PELVIS = 0.71


def load_run(run):
    d = os.path.join(LOGS, run)
    R = list(csv.DictReader(open(os.path.join(d, "push10_cmd.csv"))))
    ev = [json.loads(x) for x in open(os.path.join(d, "events.jsonl"))]
    t = np.array([float(r["t"]) for r in R])
    qd = np.array([[float(r["q_des%d" % i]) for i in range(5)] for r in R])
    qm = np.array([[float(r["q_meas%d" % i]) for i in range(5)] for r in R])
    h = np.array([[float(r["hand%d" % i]) for i in range(6)] for r in R])
    cam = np.array([[float(r["cam_x"]), float(r["cam_y"])] if r.get("cam_x") else [np.nan, np.nan] for r in R])
    ph = [r["phase"] for r in R]
    table = next((e["table"] for e in ev if e.get("what") == "push10 plan"), 0.755)
    touches = [e for e in ev if e.get("what") == "push10 touch"]
    ends = [e for e in ev if e.get("what") in ("push10 slip", "push10 pushed", "push10 done", "ABORT")]
    return dict(t=t, qd=qd, qm=qm, h=h, cam=cam, ph=ph, table=table, touches=touches, ends=ends, ev=ev)


def window(L, t_after=4.0):
    """2 s before the first touch of the top plate.. the first slip / end event + t_after (s, run clock). Starting at
    the touch skips the fast joint moves before it, which the simulated server cannot follow at its speed limits."""
    t = L["t"]
    t_on = next(ts for ts, kind in contact_events(L) if kind == "on")
    t0 = max(t[0], t_on - 2.0)
    t_end = min([e["at_s"] if "at_s" in e else e["t"] for e in L["ends"] if (e.get("at_s") or e["t"]) > t_on] or [t[-1]])
    return t0, min(t[-1], t_end + t_after)


def interp(L, key, tq):
    t = L["t"]
    i = int(np.clip(np.searchsorted(t, tq) - 1, 0, len(t) - 2))
    a = float(np.clip((tq - t[i]) / max(t[i + 1] - t[i], 1e-6), 0, 1))
    return (1 - a) * L[key][i] + a * L[key][i + 1], i


def build(table, xy, fr, n=3):
    sc = RobotScene(n=n, table=table, xy=tuple(xy), pelvis=PELVIS, fr=fr)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    return sc, meta, m


def index_geoms(m):
    return [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_index")
            and m.geom_contype[g] + m.geom_conaffinity[g] > 0]


def body_geoms(m, name):
    b = m.body(name).id
    return [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_contype[g] + m.geom_conaffinity[g] > 0]


def min_dist(m, d, ga, gb, reach=0.05):
    ft = np.zeros(6)
    return min(mujoco.mj_geomDistance(m, d, a, b, reach, ft) for a in ga for b in gb)


def place_arm(m, d, meta, qL, hL_srv):
    adr = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"]["left"]]
    adrR = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"]["right"]]
    C = np.array(meta["coupling"]["left"])
    hadr = [m.jnt_qposadr[m.joint(j).id] for j in meta["hand"]["left"]]
    hhi = np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"]["left"]])
    d.qpos[adr] = qL
    d.qpos[adrR] = Q_RIGHT
    d.qpos[hadr] = np.minimum(C @ (np.asarray(hL_srv)[SRV2SIM] * FULL_SIM), hhi)


def register_height(L, table0, xy, fr):
    """Table height so that the index touches the top plate at the R1's first confirmed touch (measured joints)."""
    tt = next(ts for ts, kind in contact_events(L) if kind == "on")
    q, _ = interp(L, "qm", tt)
    h, _ = interp(L, "h", tt)
    table = table0
    for _ in range(4):
        sc, meta, m = build(table, xy, fr)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        settle_plates(m, d)
        place_arm(m, d, meta, q, h)
        mujoco.mj_forward(m, d)
        gap = min_dist(m, d, index_geoms(m), body_geoms(m, "plate2"))
        if abs(gap) < 2e-4:
            break
        table += gap
    return table, tt, float(gap)


def contact_events(L):
    """(time, kind) from the R1's events: 'on' = the index touched the top plate, 'off' = it did not (tap miss),
    'edge' = the index had just come off the top plate's edge (slide edge)."""
    out = []
    for e in L["ev"]:
        w, ts = e.get("what"), e.get("at_s")
        if ts is None:
            continue
        if w == "push10 touch" and "confirmed" in str(e.get("by", "")):
            out.append((ts, "on"))
        elif w == "push10 tap":
            out.append((ts, "on" if e.get("result") == "top" else "off"))
        elif w == "push10 edge" and "index_y_mm" in e:
            out.append((ts, "edge"))
    return out


def register_dy(L, table, xy0, fr, dys, use_edge=False):
    """Score each stack shift dy (mm, y) by how well the R1's touch / miss / edge events fit the plate's geometry.
    The top plate follows the camera's motion from the window start; the plates below stay. Returns (best dy, table)."""
    # before the first push only (the tracker loses a rising plate later); slide edges are moved by the drag itself
    t_set = next((e["at_s"] for e in L["ev"] if e.get("what") == "push10 set"), 1e9)
    evs = [e for e in contact_events(L) if e[0] < t_set and (use_edge or e[1] != "edge")]
    sc, meta, m = build(table, xy0, fr)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    settle_plates(m, d)
    base = d.qpos.copy()
    ig, pg2 = index_geoms(m), body_geoms(m, "plate2")
    ja = [m.jnt_qposadr[m.body_jntadr[m.body("plate%d" % n).id]] for n in range(3)]
    cam = L["cam"]
    c0 = np.nanmedian(cam[:30], axis=0)
    rows = []
    for dy in dys:
        cost, det = 0.0, []
        for ts, kind in evs:
            d.qpos[:] = base
            c, _ = interp(L, "cam", ts)
            if np.isnan(c).any():
                c = c0
            for n in range(3):
                d.qpos[ja[n] + 1] += dy / 1000
            d.qpos[ja[2]:ja[2] + 2] += (c - c0)
            q, _ = interp(L, "qm", ts)
            h, _ = interp(L, "h", ts)
            place_arm(m, d, meta, q, h)
            mujoco.mj_forward(m, d)
            g = 1000 * min_dist(m, d, ig, pg2, 0.02)
            if kind == "on":
                cst = min(abs(g), 10.0)
            elif kind == "off":
                cst = max(0.0, 1.0 - g) * 3
            else:
                cst = abs(min(max(g, 0.0), 10.0) - 1.5)
            cost += cst
            det.append([round(ts, 2), kind, round(g, 1)])
        rows.append(dict(dy=dy, cost=round(cost, 2), events=det))
    best = min(rows, key=lambda r: r["cost"])
    return best["dy"], rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--pad", type=float, default=1.0, help="fingertip pad on glaze")
    ap.add_argument("--fg", type=float, default=0.575, help="plate foot ring on the glaze of the plate below")
    ap.add_argument("--gg", type=float, default=0.375, help="glaze on glaze (rim on rim)")
    ap.add_argument("--table_mu", type=float, default=0.9, help="plate foot ring on the table")
    ap.add_argument("--dy", type=float, nargs="+", default=[0.0], help="stack shift from the camera's centre, mm (y)")
    ap.add_argument("--dx", type=float, default=0.0)
    ap.add_argument("--tau_bias", type=float, default=0.3, help="s: free-air joint offset learning time constant")
    ap.add_argument("--frames", action="store_true")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    L = load_run(a.run)
    t0, t1 = window(L)
    cam0, i0 = interp(L, "cam", t0)
    k = i0
    while np.isnan(cam0).any() and k < len(L["t"]) - 1:
        k += 1
        cam0 = L["cam"][k]
    fr = Friction(pad_glaze=a.pad, foot_glaze=a.fg, glaze_glaze=a.gg, table=a.table_mu)
    for dy in a.dy:
        tag = "pad%.2f_fg%.3f_tb%.2f_dy%+.0f" % (a.pad, a.fg, a.table_mu, dy)
        f_out = os.path.join(a.out, tag + ".json")
        if os.path.exists(f_out):
            continue
        t_wall = time.time()
        xy = (float(cam0[0]) + a.dx / 1000, float(cam0[1]) + dy / 1000)
        table, t_touch, gap = register_height(L, L["table"], xy, fr)
        sc, meta, m = build(table, xy, fr)
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        settle_plates(m, d)
        q, _ = interp(L, "qm", t0)
        h, _ = interp(L, "h", t0)
        place_arm(m, d, meta, q, h)
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        act = {m.actuator(i).name: i for i in range(m.nu)}
        A = {s: [act[f"a_{j}"] for j in meta["arm"][s]] for s in SIDES}
        HA = {s: [act[f"a_{j}"] for j in meta["hand"][s]] for s in SIDES}
        C = {s: np.array(meta["coupling"][s]) for s in SIDES}
        hhi = {s: np.array([m.jnt_range[m.joint(j).id][1] for j in meta["hand"][s]]) for s in SIDES}
        adr = [m.jnt_qposadr[m.joint(j).id] for j in meta["arm"]["left"]]
        srv = ServerSim(m, d, meta, A, HA, C, hhi, torque_scale=1.0)
        srv.target("right", Q_RIGHT, np.zeros(6))
        hand_g = {g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith("left_")}
        pg = {n: set(body_geoms(m, "plate%d" % n)) for n in range(3)}
        pid = [m.body("plate%d" % n).id for n in range(3)]
        p_start = np.array([d.xpos[b].copy() for b in pid])

        def touching():
            out = [False, False, False]
            for i in range(d.ncon):
                c = d.contact[i]
                for n in range(3):
                    if (c.geom1 in hand_g and c.geom2 in pg[n]) or (c.geom2 in hand_g and c.geom1 in pg[n]):
                        out[n] = True
            return out

        bias = np.zeros(5)
        # settle the arm onto the measured joints before the window (hold the first target, learn the offset)
        qd0, _ = interp(L, "qd", t0)
        for _ in range(2500):
            srv.target("left", qd0 + bias, h[SRV2SIM] * FULL_SIM)
            srv.step()
            bias += (q - d.qpos[adr]) * (m.opt.timestep / 0.1)
        rec = {k_: [] for k_ in ("t", "plate", "below", "bottom", "q_err", "touch", "idx_z", "cam")}
        sim_t0 = d.time
        n_steps = int((t1 - t0) / m.opt.timestep)
        ren = mujoco.Renderer(m, 360, 480) if a.frames else None
        shots, next_shot = [], t0
        for st in range(n_steps):
            tr = t0 + (d.time - sim_t0)
            qd, i = interp(L, "qd", tr)
            qm, _ = interp(L, "qm", tr)
            hh, _ = interp(L, "h", tr)
            srv.target("left", qd + bias, hh[SRV2SIM] * FULL_SIM)
            srv.step()
            if st % 10 == 0:
                tc = touching()
                if L["ph"][i] in FREE_PHASES and not any(tc):
                    bias += (qm - d.qpos[adr]) * (10 * m.opt.timestep / a.tau_bias)
                if st % 33 == 0:
                    rec["t"].append(tr)
                    rec["plate"].append(1000 * (d.xpos[pid[2]] - p_start[2]))
                    rec["below"].append(1000 * (d.xpos[pid[1]] - p_start[1]))
                    rec["bottom"].append(1000 * (d.xpos[pid[0]] - p_start[0]))
                    rec["q_err"].append(1000 * (qm - d.qpos[adr]))
                    rec["touch"].append(tc)
                    c, _ = interp(L, "cam", tr)
                    rec["cam"].append(1000 * (c - cam0))
                if ren is not None and tr >= next_shot:
                    ren.update_scene(d, camera="front")
                    im1 = ren.render()
                    ren.update_scene(d, camera="top")
                    shots.append((tr, L["ph"][i], np.hstack([im1, ren.render()])))
                    next_shot += 2.0
        R = {k_: np.array(v) for k_, v in rec.items()}
        np.savez_compressed(os.path.join(a.out, tag + ".npz"), **R)
        ok = ~np.isnan(R["cam"][:, 0])
        err = R["plate"][:, :2] - R["cam"]
        free = np.array([not any(x) for x in R["touch"]])
        first_touch = next((float(R["t"][j]) for j in range(len(R["t"])) if R["touch"][j][2]), None)
        summ = dict(run=a.run, pad=a.pad, fg=a.fg, gg=a.gg, table_mu=a.table_mu, dy=dy, table_m=round(table, 4),
                    height_gap_mm=round(1000 * gap, 2), window_s=[round(t0, 2), round(t1, 2)],
                    r1_first_touch_s=t_touch, sim_first_touch_s=first_touch,
                    plate_end_mm=[round(float(x), 1) for x in R["plate"][-1][:3]],
                    cam_end_mm=[round(float(x), 1) for x in R["cam"][ok][-1]] if ok.any() else None,
                    below_max_mm=round(float(np.max(np.linalg.norm(R["below"][:, :2], axis=1))), 1),
                    bottom_max_mm=round(float(np.max(np.linalg.norm(R["bottom"][:, :2], axis=1))), 1),
                    err_rms_mm=round(float(np.sqrt(np.mean(np.sum(err[ok] ** 2, 1)))), 2) if ok.any() else None,
                    err_end_mm=round(float(np.linalg.norm(err[ok][-1])), 2) if ok.any() else None,
                    q_err_free_rms_mrad=round(float(np.sqrt(np.mean(R["q_err"][free] ** 2))), 1) if free.any() else None,
                    server=srv.summary(), wall_s=round(time.time() - t_wall))
        json.dump(summ, open(f_out, "w"), indent=1, default=float)
        print(json.dumps(summ, default=float), flush=True)
        if shots:
            import cv2
            for j, (tr, ph, im) in enumerate(shots):
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                cv2.putText(im, "MuJoCo %s  %.1f s  %s  %s" % (mujoco.__version__, tr, ph, tag), (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 255), 1, cv2.LINE_AA)
                cv2.imwrite(os.path.join(a.out, "%s_f%02d.jpg" % (tag, j)), im)


if __name__ == "__main__":
    main()
