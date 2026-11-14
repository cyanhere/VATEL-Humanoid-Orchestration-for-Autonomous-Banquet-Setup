"""Stage 1 study: one free fingertip pushes the top plate of a stack on its rim edge.

    python scripts/push_probe.py --one --alpha 35 --beta 30 --sheet out.png     # one push, with a frame sheet
    python scripts/push_probe.py --sweep contact --tag contact_v1 --workers 12 # a parameter sweep -> JSON
"""
import argparse
import itertools
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import mujoco  # noqa: E402

from platepick.plate import TASK, Friction, PlateSpec  # noqa: E402
from platepick.probe_scene import TIP_R, ProbeScene  # noqa: E402

FINGER_T = 0.0156            # Revo2 finger thickness that has to fit under the rim
FINGER_MARGIN = 0.002
PARK = np.array([0.0, 0.25, 0.15])

DEFAULTS = dict(n=3, alpha=35.0, beta=30.0, offset=0.0, dist=60.0, speed=20.0, foot_glaze=0.575, glaze_glaze=0.375,
                pad_glaze=0.25, table=0.45, floor_r=82.0, fillet=8.0, kink_z=8.75, pitch=6.8, preload=0.0, kz=1500.0, pad=0, land=0.0, frames=0)


def surf_z(spec, pos, R, q, which):
    """World z of a plate's underside / top surface on the vertical line through q = (x, y); None outside the plate."""
    fn = spec.z_under if which == "under" else spec.z_top
    z = pos[2] + (spec.kink_z if which == "under" else spec.floor_top)
    for _ in range(8):
        loc = R.T @ (np.array([q[0], q[1], z]) - pos)
        r = math.hypot(loc[0], loc[1])
        if r > spec.R:
            return None
        z += (fn(r) - loc[2]) / R[2, 2]
    return z


def opening(spec, poses, n):
    """Reach under the leading rim of the top plate for a finger coming from the leading side."""
    pt, Rt = poses[n - 1]
    pl = poses[n - 2][0]
    d = pt[:2] - pl[:2]
    s = float(np.linalg.norm(d))
    d = d / s if s > 1e-4 else np.array([0.0, -1.0])
    perp = np.array([-d[1], d[0]])
    need = FINGER_T + FINGER_MARGIN
    depth, gap_edge = [], None
    for lat in (-0.025, 0.0, 0.025):
        rhos = np.arange(0.06, spec.R + s + 0.012, 0.0005)
        ok_run, seen = 0.0, False
        for rho in rhos[::-1]:                                    # from outside inward
            q = pl[:2] + rho * d + lat * perp
            ceil = surf_z(spec, pt, Rt, q, "under")
            if ceil is None:
                if seen:
                    break
                continue
            seen = True
            floor = 0.0
            for k in range(n - 1):
                z = surf_z(spec, poses[k][0], poses[k][1], q, "top")
                if z is not None:
                    floor = max(floor, z)
            if lat == 0.0 and abs(rho - spec.R) < 0.0003:
                gap_edge = ceil - floor
            if ceil - floor >= need:
                ok_run += 0.0005
            else:
                break
        depth.append(ok_run)
    return dict(slide=s * 1000, grip_depth=min(depth) * 1000, gap_at_lower_edge=None if gap_edge is None else gap_edge * 1000)


def run(par):
    p = dict(DEFAULTS, **par)
    spec = PlateSpec(**{**TASK.__dict__, "floor_r": p["floor_r"] / 1000, "fillet": p["fillet"] / 1000,
                        "kink_z": p["kink_z"] / 1000, "floor_top": p["pitch"] / 1000})
    fr = Friction(glaze_glaze=p["glaze_glaze"], foot_glaze=p["foot_glaze"], pad_glaze=p["pad_glaze"], table=p["table"])
    n = int(p["n"])
    m = mujoco.MjModel.from_xml_string(ProbeScene(spec, n=n, fr=fr, kz=p["kz"], pad_deg=p["alpha"] if p["pad"] else None).xml())
    d = mujoco.MjData(m)
    dt = m.opt.timestep
    tip_g, tip_b = m.geom("tip").id, m.body("tip").id
    pb = [m.body(f"plate{k}").id for k in range(n)]
    qadr = [m.jnt_qposadr[m.joint(f"tip_{a}").id] for a in "xyz"]

    def poses():
        return [(d.xpos[b].copy(), d.xmat[b].reshape(3, 3).copy()) for b in pb]

    def tip_force():
        """World force the plates put on the fingertip, and which plates touch it."""
        F, who = np.zeros(3), {}
        w = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if tip_g not in (c.geom1, c.geom2):
                continue
            mujoco.mj_contactForce(m, d, i, w)
            f = c.frame.reshape(3, 3).T @ w[:3]
            if c.geom2 == tip_g:
                other = c.geom1
            else:
                other, f = c.geom2, -f
            F += f
            name = m.body(m.geom_bodyid[other]).name
            who[name] = who.get(name, 0.0) + float(np.linalg.norm(f))
        return F, who

    def goto(target):
        d.ctrl[:] = np.asarray(target) - PARK

    renderer, shots = None, []
    if p["frames"]:
        renderer = mujoco.Renderer(m, 720, 1280)

    def shoot(label):
        if renderer is None:
            return
        ims = []
        for cam in ("side", "lead"):
            renderer.update_scene(d, camera=cam)
            ims.append(renderer.render().copy())
        shots.append((label, ims))

    goto(PARK)
    for _ in range(int(0.6 / dt)):
        mujoco.mj_step(m, d)
    p0 = poses()
    top0 = p0[n - 1][0]
    a, b = math.radians(p["alpha"]), math.radians(p["beta"])
    e = p["offset"] / 1000
    ce_r, ce_z = spec.edge_c
    E = top0 + np.array([e, math.sqrt(ce_r ** 2 - e ** 2), ce_z])
    out = np.array([0.0, math.cos(a), math.sin(a)])
    T = E + (spec.edge_r + TIP_R) * out
    if p["pad"]:                 # flat pad: the edge meets the pad face `land` mm from its centre line (+ = higher up)
        T = E + spec.edge_r * out - p["land"] / 1000 * np.array([0.0, -math.sin(a), math.cos(a)])
    P0 = T + 0.010 * out
    log = []
    t_log = [0.0]

    def step(target, rec=True):
        goto(target)
        mujoco.mj_step(m, d)
        if rec and d.time - t_log[0] >= 0.01:
            t_log[0] = d.time
            F, who = tip_force()
            ps = poses()
            log.append(dict(t=d.time, tip=d.xpos[tip_b].tolist(), F=F.tolist(), who=who,
                            top=ps[n - 1][0].tolist(), tilt=math.degrees(math.acos(min(1.0, ps[n - 1][1][2, 2]))),
                            low=[x[0].tolist() for x in ps[:-1]]))

    # 1. bring the fingertip to the start point, 10 mm out along the contact direction
    for k in range(int(0.6 / dt)):
        u = (k + 1) / int(0.6 / dt)
        u = u * u * (3 - 2 * u)
        step(PARK + u * (P0 - PARK), rec=False)
    for _ in range(int(0.2 / dt)):
        step(P0, rec=False)
    shoot("before")
    # 2. gentle touch: approach at 10 mm/s until 0.3 N
    tgt = P0.copy()
    touched = False
    for _ in range(int(2.0 / dt)):
        tgt = tgt - 0.010 * dt * out
        step(tgt)
        if np.linalg.norm(tip_force()[0]) > 0.3:
            touched = True
            break
        if np.dot(P0 - tgt, out) > 0.016:
            break
    # 2b. preload: lower the target so the finger rests on the edge with about `preload` newtons
    z_touch = float(d.xpos[tip_b][2])
    n_pre = int(0.3 / dt)
    for k in range(n_pre if p["preload"] > 0 else 0):
        tgt = tgt - np.array([0.0, 0.0, p["preload"] / p["kz"] / n_pre])
        step(tgt)
    t_touch = d.time
    shoot("touch")
    # 3. push
    u_push = np.array([0.0, -math.cos(b), -math.sin(b)])
    n_push = int(p["dist"] / p["speed"] / dt)
    marks = {int(n_push * f) for f in (0.33, 0.66)}
    for k in range(n_push):
        tgt = tgt + p["speed"] / 1000 * dt * u_push
        step(tgt)
        if k in marks:
            shoot(f"push {100 * k // n_push}%")
    for _ in range(int(0.3 / dt)):
        step(tgt)
    shoot("end of push")
    ps = poses()
    res = dict(par=p, touched=touched)
    res.update(opening(spec, ps, n))
    pt, Rt = ps[n - 1]
    res["tilt"] = math.degrees(math.acos(min(1.0, Rt[2, 2])))
    res["top_dy"] = (pt[1] - top0[1]) * 1000
    res["top_dx"] = (pt[0] - top0[0]) * 1000
    res["low_shift"] = max(float(np.linalg.norm(ps[k][0][:2] - p0[k][0][:2])) for k in range(n - 1)) * 1000
    res["bottom_shift"] = float(np.linalg.norm(ps[0][0][:2] - p0[0][0][:2])) * 1000
    push = [r for r in log if r["t"] > t_touch]
    Fn = [float(np.linalg.norm(r["F"])) for r in push] or [0.0]
    res["F_peak"], res["F_mean"] = max(Fn), float(np.mean(Fn))
    res["F_down_mean"] = float(np.mean([r["F"][2] for r in push] or [0.0]))      # finger pressing down on the plates
    res["F_down_peak"] = max([r["F"][2] for r in push] or [0.0])
    res["F_side_mean"] = float(np.mean([r["F"][1] for r in push] or [0.0]))      # finger pushing sideways
    res["tip_drop"] = (z_touch - float(d.xpos[tip_b][2])) * 1000
    top_name = f"plate{n - 1}"
    res["touched_lower"] = max([sum(v for k_, v in r["who"].items() if k_ != top_name) for r in push] or [0.0])
    loc = Rt.T @ (d.xpos[tip_b] - pt)
    res["tip_r"] = math.hypot(loc[0], loc[1]) * 1000          # fingertip centre in the top plate's frame
    res["tip_z"] = loc[2] * 1000
    _, who = tip_force()
    res["on_edge"] = bool(who.get(top_name, 0) > 0.05 and (p["pad"] or res["tip_r"] > (spec.R - 0.002) * 1000))
    # is the plate stable once the finger lets go? (retract, wait)
    for k in range(int(0.4 / dt)):
        step(tgt + min(1.0, k / (0.3 / dt)) * 0.03 * np.array([0.0, 0.6, 0.8]), rec=False)
    for _ in range(int(0.6 / dt)):
        step(tgt + 0.03 * np.array([0.0, 0.6, 0.8]), rec=False)
    ps2 = poses()
    res["after_release"] = opening(spec, ps2, n)
    res["after_release"]["tilt"] = math.degrees(math.acos(min(1.0, ps2[n - 1][1][2, 2])))
    shoot("finger released")
    res["trace"] = [dict(t=round(r["t"] - t_touch, 3), y=round((r["top"][1] - top0[1]) * 1000, 2), tilt=round(r["tilt"], 2),
                         F=[round(x, 2) for x in r["F"]]) for r in push[::5]]
    if shots:
        res["_shots"] = shots
    return res


def sheet(res, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    shots = res["_shots"]
    fig, axs = plt.subplots(2, len(shots), figsize=(3.6 * len(shots), 4.6))
    for j, (label, ims) in enumerate(shots):
        for i in range(2):
            axs[i, j].imshow(ims[i])
            axs[i, j].axis("off")
        axs[0, j].set_title(label, fontsize=11)
    p = res["par"]
    fig.suptitle(f"MuJoCo, free {'flat finger pad' if p['pad'] else 'round fingertip'}, {p['n']} plates: contact {p['alpha']:.0f} deg, push {p['beta']:.0f} deg below "
                 f"horizontal | slid {res['slide']:.0f} mm, tilt {res['tilt']:.1f} deg, reach under leading rim "
                 f"{res['grip_depth']:.0f} mm, push force peak {res['F_peak']:.1f} N", fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


SWEEPS = {
    # where on the edge, and in which direction
    "contact": dict(alpha=[10, 20, 35, 50, 65], beta=[0, 15, 30, 45], n=[3]),
    # friction corners, at a few contact choices
    "friction": dict(alpha=[20, 35, 50], beta=[15, 30], foot_glaze=[0.5, 0.65], glaze_glaze=[0.35, 0.4],
                     pad_glaze=[0.2, 0.25, 0.3], n=[3]),
    # stack height and table friction (matters for the last plates)
    "stack": dict(alpha=[35], beta=[15, 30], n=[2, 3, 4, 5], table=[0.3, 0.45, 0.6], foot_glaze=[0.5, 0.65]),
    # what I assumed about the plate, and a misplaced finger
    "model": dict(pad=[1], alpha=[45, 55], beta=[10, 20], preload=[2, 5], land=[-3, 0], floor_r=[80, 82, 85.5],
                  fillet=[3, 8, 15], n=[3]),
    # push law: rest on the edge with a preload, then push along a shallow slope, with a given vertical stiffness
    "law": dict(alpha=[20, 35, 50], preload=[2, 5, 8], beta=[0, 5, 10, 15, 20], kz=[800, 1500, 3000], n=[3]),
    # flat finger pad instead of a round fingertip; alpha = how far the pad face looks downward
    "pad": dict(pad=[1], alpha=[15, 25, 35, 45, 55], preload=[2, 5], beta=[0, 10, 20], land=[-3, 0, 3], n=[3]),
    # stacking pitch varies between real plates (rim gap = pitch - 5.5 mm)
    "pitch": dict(pad=[1], alpha=[45, 55], beta=[10, 20], preload=[2, 5], land=[-3, 0], pitch=[6.0, 6.4, 6.8, 7.2, 7.5],
                  n=[3]),
    # steeper pad tilts (hand rolled toward palm-down), 2 and 3 plates
    "tilt": dict(pad=[1], alpha=[45, 55, 65, 75], beta=[10, 20, 30, 40], preload=[2, 5], land=[-3, 0], n=[2, 3]),
    # how far to push
    "dist": dict(alpha=[35], beta=[15, 30], dist=[20, 30, 40, 50, 60, 80], n=[3, 5]),
}


def _run_safe(par):
    try:
        r = run(par)
        r.pop("_shots", None)
        return r
    except Exception as ex:                                        # keep the sweep alive, record the failure
        return dict(par=dict(DEFAULTS, **par), error=repr(ex))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--one", action="store_true")
    ap.add_argument("--sweep", default=None, choices=list(SWEEPS))
    ap.add_argument("--tag", default=None)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--sheet", default=None, help="write a frame sheet (needs a GL context)")
    ap.add_argument("--out", default="data/results/push_probe")
    for k, v in DEFAULTS.items():
        if k != "frames":
            ap.add_argument(f"--{k}", type=type(v), default=None)
    a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    over = {k: getattr(a, k) for k in DEFAULTS if k != "frames" and getattr(a, k) is not None}
    if a.one:
        t0 = time.time()
        res = run(dict(over, frames=1 if a.sheet else 0))
        if a.sheet:
            sheet(res, a.sheet)
        res.pop("_shots", None)
        tr = res.pop("trace")
        print(json.dumps(res, indent=1))
        print("trace (t s, plate y mm, tilt deg, force on tip N):")
        for r in tr[::4]:
            print("  ", r)
        print(f"wall time {time.time() - t0:.1f} s")
        return
    grid = SWEEPS[a.sweep]
    keys = list(grid)
    pars = [dict(over, **dict(zip(keys, v))) for v in itertools.product(*[grid[k] for k in keys])]
    from multiprocessing import Pool
    t0 = time.time()
    with Pool(a.workers) as pool:
        out = pool.map(_run_safe, pars, chunksize=1)
    os.makedirs(os.path.join(root, a.out), exist_ok=True)
    path = os.path.join(root, a.out, f"{a.tag or a.sweep}.json")
    json.dump(dict(simulator="MuJoCo " + mujoco.__version__, sweep=a.sweep, keys=keys, results=out), open(path, "w"))
    print(f"{len(out)} pushes in {time.time() - t0:.0f} s -> {path}")


if __name__ == "__main__":
    main()
