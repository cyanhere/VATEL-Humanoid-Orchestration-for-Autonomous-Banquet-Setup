"""Kinematics check: can the left hand place a finger pad on the far rim edge of the top plate, facing the centre?

    python scripts/reach_push.py [--out data/results/robot/reach_push.json] [--sheet picture.png]
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402

from platepick.ik import PadIK  # noqa: E402
from platepick.robot_scene import RobotScene  # noqa: E402

FINGERS = ("index", "middle", "ring", "pinky")
PHIS = (0, 15, 30, 45, 60, 75)


def push_pose(spec, top_pos, phi_deg, gamma_deg, standoff=0.002):
    """Pad point and pad normal for a contact on the rim edge at angle phi (0 = leftmost point, 90 = farthest point)."""
    phi, g = math.radians(phi_deg), math.radians(gamma_deg)
    out = np.array([math.sin(phi), math.cos(phi), 0.0])              # radially outward at the contact
    n = -math.cos(g) * out - math.sin(g) * np.array([0, 0, 1.0])     # the pad faces the plate centre and downward
    ce_r, ce_z = spec.edge_c
    E = top_pos + ce_r * out + np.array([0, 0, ce_z])
    return E - (spec.edge_r + standoff) * n, n


def collisions(m, d):
    """Penetrating contacts between the robot and anything else: {"robot link|other": deepest mm}."""
    out = {}
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name
        r1, r2 = b1.startswith(("left_", "right_")), b2.startswith(("left_", "right_"))
        if not (r1 or r2) or c.dist > -0.0005:
            continue
        key = f"{b1}|{b2}" if r1 else f"{b2}|{b1}"
        out[key] = min(out.get(key, 0.0), c.dist * 1000)
    return out


def setup(table, n, x, pelvis):
    sc = RobotScene(n=n, table=table, xy=(x, 0.0), pelvis=pelvis)
    xml, meta = sc.build()
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    base = d.qpos.copy()
    for k in range(n):                                                # plates at their settled heights
        base[m.jnt_qposadr[m.joint(f"plate{k}_free").id] + 2] = table + k * sc.spec.floor_top
    for jn, v in zip(meta["arm"]["right"], meta["ready"]["right"]):   # right arm parked
        base[m.jnt_qposadr[m.joint(jn).id]] = v
    top = np.array([x, 0.0, table + (n - 1) * sc.spec.floor_top])
    return sc, m, d, meta, base, top


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="data/results/robot/reach_push.json")
    ap.add_argument("--sheet", default=None)
    ap.add_argument("--gamma", type=float, default=45.0)
    ap.add_argument("--pelvis", type=float, default=0.76)
    ap.add_argument("--x", type=float, nargs="*", default=[0.31])
    a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rng = np.random.default_rng(0)
    rows, shots, renderer = [], [], None
    layouts = list(itertools.product((0.747, 0.762), (2, 5), a.x))
    for table, n, x in layouts:
        sc, m, d, meta, base, top = setup(table, n, x, a.pelvis)
        for finger, phi in itertools.product(FINGERS, PHIS):
            ik = PadIK(m, meta, "left", finger)
            p, nrm = push_pose(sc.spec, top, phi, a.gamma)
            seeds = [[0, 0, 0, 0, 0, 0.1], [0, 0, 0.4, 0, 0.8, 0.1]] + \
                    [list(np.r_[rng.normal(0, 0.5, 5), rng.uniform(0, 0.8)]) for _ in range(10)]
            res = ik.solve(p, nrm, seeds, base=base)
            good = [c for c in res["all"] if c["pos_err_mm"] < 1.0 and c["ang_err_deg"] < 2.0]
            pick, col = None, None
            for c in good:
                d.qpos[:] = base
                ik.set(d, c["x"])
                mujoco.mj_forward(m, d)
                cc = collisions(m, d)
                if col is None or len(cc) < len(col):
                    pick, col = c, cc
                if not cc:
                    break
            row = dict(table=table, n=n, x=x, finger=finger, phi=phi, reached=bool(good), free=bool(good) and not col,
                       best_err=[res["pos_err_mm"], res["ang_err_deg"]])
            if pick is not None:
                row.update(q=[float(v) for v in pick["x"]], finger_axis=[float(v) for v in pick["axis"]], collisions=col)
            rows.append(row)
            if a.sheet and pick is not None and table == 0.747 and n == 2 and len(shots) < 4 and row["free"]:
                d.qpos[:] = base
                ik.set(d, pick["x"])
                mujoco.mj_forward(m, d)
                if renderer is not None:
                    renderer.close()
                renderer = mujoco.Renderer(m, 720, 1280)
                ims = []
                for cam in ("front", "left", "top"):
                    renderer.update_scene(d, camera=cam)
                    ims.append(renderer.render().copy())
                shots.append((f"{finger} finger at rim angle {phi} deg (table {table} m, {n} plates): reached, touches nothing", ims))
    os.makedirs(os.path.dirname(os.path.join(root, a.out)), exist_ok=True)
    json.dump(dict(simulator="MuJoCo " + mujoco.__version__ + " (kinematics only)", gamma=a.gamma, pelvis=a.pelvis, rows=rows),
              open(os.path.join(root, a.out), "w"))
    print(f"pad tilted {a.gamma:.0f} deg down, pelvis {a.pelvis} m; {len(layouts)} layouts = tables 0.747/0.762 x 2 or 5 plates x distance {a.x}")
    print("finger  rim angle | reached | reached and touching nothing | what it hits otherwise")
    for finger in FINGERS:
        for phi in PHIS:
            g = [r for r in rows if r["finger"] == finger and r["phi"] == phi]
            hit = sorted({k.split("|")[0].replace("left_", "") + ">" + k.split("|")[1] for r in g if r["reached"] and not r["free"]
                          for k in r["collisions"]})
            print(f"{finger:7s} {phi:3d} | {sum(r['reached'] for r in g)} | {sum(r['free'] for r in g)} | {hit[:5]}")
    if shots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(len(shots), 3, figsize=(17, 3.4 * len(shots)), squeeze=False)
        for i, (label, ims) in enumerate(shots):
            for j in range(3):
                axs[i, j].imshow(ims[j])
                axs[i, j].axis("off")
            axs[i, 0].set_title(label, loc="left", fontsize=11)
        fig.tight_layout()
        fig.savefig(a.sheet, dpi=90)


if __name__ == "__main__":
    main()
