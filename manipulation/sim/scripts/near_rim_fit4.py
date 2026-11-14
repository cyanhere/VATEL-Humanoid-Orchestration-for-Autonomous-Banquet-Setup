"""Hand pose for the near-rim two-finger push: index and middle both on the rim's outer part (kinematics only).

    python scripts/near_rim_fit4.py fit OUT.json [--n 3 --table 0.747 --pelvis 0.71]
    python scripts/near_rim_fit4.py refine OUT.json --start FIT.json     (Nelder-Mead from the fit's best, wider bounds)
    python scripts/near_rim_fit4.py show OUT.png --theta 56 --inset 15.5 --yaw 37 --pitch 29 --roll 0 --ci 0.06 --cm 0
"""
import argparse
import itertools
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
sys.path.insert(0, ROOT)
import mujoco  # noqa: E402

from near_rim_pose import place_near, scene  # noqa: E402

PARTS = ("thumb", "index", "middle", "ring", "pinky")


def part_of(name, side="left"):
    if not name.startswith(side + "_"):
        return None
    for f in PARTS:
        if f"_{f}_" in name:
            return f
    return "palm" if name in (f"{side}_base_link", f"{side}_wrist_roll_link") else None


class Gaps:
    """signed distance (mm, - = into) of each hand part to the top plate (and where on it), to the lower plates, the table"""

    def __init__(self, m, n, side="left"):
        self.m = m
        self.top = m.body(f"plate{n - 1}").id
        self.lower = {m.body(f"plate{k}").id for k in range(n - 1)}
        col = [g for g in range(m.ngeom) if m.geom_contype[g] | m.geom_conaffinity[g]]
        self.hand = {}
        for g in col:
            p = part_of(m.body(m.geom_bodyid[g]).name, side)
            if p:
                self.hand.setdefault(p, []).append(g)
        self.g_top = [g for g in col if m.geom_bodyid[g] == self.top]
        self.g_low = [g for g in col if m.geom_bodyid[g] in self.lower]
        self.g_tab = [g for g in col if m.geom_bodyid[g] == 0]
        self.ft = np.zeros(6)

    def _min(self, d, hg, others, reach=0.04):
        best = (reach * 1000, None)
        for a in hg:
            ca, ra = d.geom_xpos[a], self.m.geom_rbound[a]
            for b in others:
                rb = self.m.geom_rbound[b]
                if rb > 0 and np.linalg.norm(d.geom_xpos[b] - ca) > ra + rb + reach:
                    continue
                dist = mujoco.mj_geomDistance(self.m, d, a, b, reach, self.ft)
                if dist * 1000 < best[0]:
                    best = (dist * 1000, self.ft[3:6].copy())
        return best

    def __call__(self, d):
        pt, Rt = d.xpos[self.top], d.xmat[self.top].reshape(3, 3)
        out = {}
        for p, hg in self.hand.items():
            g, w = self._min(d, hg, self.g_top)
            e = dict(top=round(g, 2))
            if w is not None:
                v = Rt.T @ (w - pt)
                e.update(r=round(float(np.hypot(v[0], v[1])) * 1000, 1), theta=round(math.degrees(math.atan2(v[1], -v[0])), 1))
            e["lower"] = round(self._min(d, hg, self.g_low)[0], 2)
            e["table"] = round(self._min(d, hg, self.g_tab)[0], 2)
            out[p] = e
        return out


def evaluate(m, d0, meta, spec, gaps, x, seed=None, starts=0, r_lo=110.0, r_hi=122.0, clear=8.0):
    th, inset, yaw, pitch, roll, ci, cm = x
    d = mujoco.MjData(m)
    d.qpos[:] = d0.qpos
    mujoco.mj_forward(m, d)
    hand = np.array([0.0, 0.0, ci, cm, 1.55, 1.55])
    q, info = place_near(m, d, meta, spec, gaps.top, hand, th, inset, yaw, pitch, roll, dz=5.0, seed=seed, starts=starts)
    mujoco.mj_forward(m, d)
    G = gaps(d)
    gi, gm = G["index"]["top"], G["middle"]["top"]
    first = min(gi, gm)                             # the hand comes down this far until the first of the two touches
    c = 3.0 * abs(gi - gm)                          # both touch together
    for f in ("index", "middle"):
        r = G[f].get("r", 0.0)
        c += 2.0 * max(0.0, r_lo - r) + 2.0 * max(0.0, r - r_hi)
    for p, e in G.items():
        for k in ("top", "lower", "table"):
            if k == "top" and p in ("index", "middle"):
                continue
            # a tip resting on the top rim is only the rim's thickness + the stacking gap (~6.5 mm) above the
            # plate below - 8 mm there could not be met and pulled the fit off equal touches: 4 mm for the touching two
            cl = 4.0 if (k == "lower" and p in ("index", "middle")) else clear
            c += 4.0 * max(0.0, cl - (e[k] - first))
    c += 0.2 * info["pos_mm"] + 0.5 * info["fingers_deg"] + 0.5 * info["palm_deg"] + 200.0 * (info["margin_deg"] < 5)
    return c, q, dict(x=[round(float(v), 3) for v in x], cost=round(c, 2), first_mm=round(first, 2), gaps=G, ik=info,
                      q=[round(float(v), 4) for v in q])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["fit", "show", "refine"])
    ap.add_argument("--start", default="")
    ap.add_argument("out")
    ap.add_argument("--n", type=int, default=3)
    ap.add_argument("--table", type=float, default=0.747)
    ap.add_argument("--pelvis", type=float, default=0.71)
    for k, v in dict(theta=56.0, inset=15.5, yaw=37.0, pitch=29.0, roll=0.0, ci=0.06, cm=0.0).items():
        ap.add_argument(f"--{k}", type=float, default=v)
    a = ap.parse_args()
    sc, m, d0, meta = scene(a.n, a.table, a.pelvis)
    gaps = Gaps(m, a.n)
    if a.mode == "show":
        show(sc, m, d0, meta, gaps, a)
        return
    if a.mode == "refine":
        refine(sc, m, d0, meta, gaps, a)
        return
    grid = dict(theta=(50, 56, 62, 68), inset=(6, 9, 12, 15), yaw=(37, 47, 57), pitch=(10, 18, 26), roll=(-10, -4, 2, 8),
                ci=(0.0, 0.15, 0.3), cm=(0.0, 0.15, 0.3))
    rows, seed = [], None
    for x in itertools.product(*grid.values()):
        c, q, r = evaluate(m, d0, meta, sc.spec, gaps, x, seed=seed, starts=0 if seed is not None else 20)
        if r["ik"]["pos_mm"] < 3:
            seed = q
        rows.append(r)
        if len(rows) % 200 == 0:
            b = min(rows, key=lambda r_: r_["cost"])
            print(len(rows), b["cost"], b["x"], flush=True)
    rows.sort(key=lambda r_: r_["cost"])
    # refine the best few with fresh starts (the warm-started IK may sit on another branch)
    best = []
    for r in rows[:15]:
        c, q, r2 = evaluate(m, d0, meta, sc.spec, gaps, r["x"], starts=20)
        best.append(r2)
    best.sort(key=lambda r_: r_["cost"])
    json.dump(dict(n=a.n, table=a.table, pelvis=a.pelvis, grid=grid, best=best, top=rows[:60]), open(a.out, "w"), indent=1)
    for r in best[:5]:
        print(json.dumps({k: r[k] for k in ("x", "cost", "first_mm")}), {f: r["gaps"][f] for f in ("index", "middle")})


BOUNDS = np.array([(35, 80), (3, 18), (15, 65), (5, 35), (-20, 25), (0.0, 0.5), (0.0, 0.5)])


def refine(sc, m, d0, meta, gaps, a):
    """: the grid's best sat on its bounds (inset 6, yaw 37, roll 8): Nelder-Mead from the 6 best distinct grid
    poses inside wider bounds (BOUNDS), the IK warm-started from the previous evaluation"""
    from scipy.optimize import minimize
    src = json.load(open(a.start))
    starts, seen = [], set()
    for r in src["best"] + src["top"]:
        k = tuple(r["x"])
        if k not in seen:
            seen.add(k)
            starts.append(r)
        if len(starts) >= 6:
            break
    out = []
    for r0 in starts:
        st = dict(seed=np.array(r0["q"]), best=None)

        def f(x):
            xc = np.clip(x, BOUNDS[:, 0], BOUNDS[:, 1])
            c, q, r = evaluate(m, d0, meta, sc.spec, gaps, xc, seed=st["seed"], starts=0)
            if r["ik"]["pos_mm"] > 3:
                c, q, r = evaluate(m, d0, meta, sc.spec, gaps, xc, seed=st["seed"], starts=8)
            if r["ik"]["pos_mm"] < 3:
                st["seed"] = q
            c += 50.0 * float(np.sum(np.abs(x - xc)))
            if st["best"] is None or c < st["best"]["cost"]:
                st["best"] = dict(r, cost=round(c, 3))
            return c
        x0 = np.array(r0["x"], float)
        simplex = [x0] + [x0 + np.eye(7)[i] * s_ for i, s_ in enumerate((4, 2, 6, 5, 4, 0.1, 0.1))]
        minimize(f, x0, method="Nelder-Mead", options=dict(maxfev=350, xatol=0.1, fatol=0.02, initial_simplex=np.array(simplex)))
        out.append(st["best"])
        g = st["best"]["gaps"]
        print(r0["x"], "->", st["best"]["x"], st["best"]["cost"], "I", g["index"], "M", g["middle"], flush=True)
    out.sort(key=lambda r_: r_["cost"])
    json.dump(dict(n=a.n, table=a.table, pelvis=a.pelvis, bounds=BOUNDS.tolist(), best=out), open(a.out, "w"), indent=1)


def show(sc, m, d0, meta, gaps, a):
    """the pose at the moment the first of index / middle touches: 4 close-ups + the gaps"""
    import cv2
    from rim_push_pose import free_cam
    x = (a.theta, a.inset, a.yaw, a.pitch, a.roll, a.ci, a.cm)
    c, q, r = evaluate(m, d0, meta, sc.spec, gaps, x, starts=20)
    d = mujoco.MjData(m)
    d.qpos[:] = d0.qpos
    mujoco.mj_forward(m, d)
    hand = np.array([0.0, 0.0, a.ci, a.cm, 1.55, 1.55])
    place_near(m, d, meta, sc.spec, gaps.top, hand, a.theta, a.inset, a.yaw, a.pitch, a.roll, dz=5.0 - r["first_mm"], seed=q, starts=0)
    mujoco.mj_forward(m, d)
    r["at_touch"] = gaps(d)
    print(json.dumps({k: r[k] for k in ("x", "cost", "first_mm", "ik")}))
    print(json.dumps(r["at_touch"]))
    tgt = d.xpos[m.body("left_index_touch_link").id].copy()
    rr = mujoco.Renderer(m, 480, 640)
    ims = []
    for eye in ((-0.30, 0.0, 0.12), (0.0, 0.30, 0.06), (0.30, -0.10, 0.15), (-0.02, 0.0, 0.34)):
        rr.update_scene(d, camera=free_cam(tgt + np.array(eye), tgt))
        ims.append(cv2.cvtColor(rr.render(), cv2.COLOR_RGB2BGR))
    im = np.vstack([np.hstack(ims[:2]), np.hstack(ims[2:])])
    t = r["at_touch"]
    cv2.putText(im, f"theta {a.theta} inset {a.inset} yaw {a.yaw} pitch {a.pitch} roll {a.roll} ci {a.ci} cm {a.cm}", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.putText(im, " | ".join(f"{f} {t[f]['top']:+.1f} mm r {t[f].get('r', 0):.0f} th {t[f].get('theta', 0):.0f}" for f in ("index", "middle")),
                (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1, cv2.LINE_AA)
    cv2.imwrite(a.out, im)
    json.dump(r, open(os.path.splitext(a.out)[0] + ".json", "w"), indent=1)


if __name__ == "__main__":
    main()
