"""Arm values for the simulation from the armwide_poses and armwide_motion runs, one arm at a time.

    python calibration/armwide_fit.py <log dir> [<log dir>...] --out../models/sim_arm_wide_left.json
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from pk import config  # noqa: E402
from pk.model import GravityComp, Robot  # noqa: E402

NAMES = ["sh pitch", "sh roll", "sh yaw", "elbow", "wrist"]


def load(d):
    meta = json.load(open(os.path.join(d, "armwide_meta.json")))
    R = list(csv.DictReader(open(os.path.join(d, "armwide_cmd.csv"))))
    D = dict(side=meta["side"], mode=meta["mode"], ph=np.array([r["phase"] for r in R]), t=np.array([float(r["t"]) for r in R]))
    for k, name in (("q", "q_meas"), ("tau", "tau"), ("dq", "dq")):
        D[k] = np.array([[float(r["%s%d" % (name, i)]) for i in range(5)] for r in R])
    D["hand"] = np.array([[float(r["hand%d" % i]) for i in range(6)] for r in R])
    return D


class Grav(object):
    def __init__(self, side, hand):
        self.rob, self.side = Robot(), side
        self.sl = config.LEFT if side == "left" else config.RIGHT
        self.hs = np.zeros(12)
        self.hs[(0 if side == "left" else 6):(6 if side == "left" else 12)] = hand
        self.a = np.zeros(config.N_ARM)

    def __call__(self, q):
        self.a[self.sl] = q
        return self.rob.gravity11(self.a, self.hs)[self.sl]


def fit_poses(runs, g):
    Q, Ta, Tb = [], [], []
    for D in runs:
        tags = sorted({p.split("_")[0] for p in D["ph"] if p[0] == "g" and p[1:].split("_")[0].isdigit()}, key=lambda x: int(x[1:]))
        for tg in tags:
            ka, kb = np.where(D["ph"] == tg + "_a")[0], np.where(D["ph"] == tg + "_b")[0]
            if len(ka) and len(kb):
                ka, kb = ka[len(ka) // 2:], kb[len(kb) // 2:]
                Q.append(0.5 * (np.median(D["q"][ka], axis=0) + np.median(D["q"][kb], axis=0)))
                Ta.append(np.median(D["tau"][ka], axis=0))
                Tb.append(np.median(D["tau"][kb], axis=0))
    Q, Ta, Tb = map(np.array, (Q, Ta, Tb))
    y, fr = 0.5 * (Ta + Tb), 0.5 * np.abs(Ta - Tb)
    M = np.array([g(q) for q in Q])
    comp = None
    p = os.path.join(HERE, "..", "model", "gravity_fit.json")
    if os.path.exists(p):
        comp = GravityComp(g.rob, path=p)
    print("%d poses. Per joint: held torque, friction from the holds, rms N m: the server's fit, this fit leave one pose out; scale, constant" % len(Q))
    out = dict(poses=len(Q), scale=[], bias_nm=[], hold_friction_nm=[], loo_rms_nm=[], server_rms_nm=[])
    for j in range(5):
        now = comp.scale[g.sl][j] * M[:, j] + comp.bias_nm[g.sl][j] if comp is not None else y[:, j] * np.nan
        e = [y[i, j] - np.polyval(np.polyfit(np.delete(M[:, j], i), np.delete(y[:, j], i), 1), M[i, j]) for i in range(len(y))]
        b = np.polyfit(M[:, j], y[:, j], 1)
        out["scale"].append(float(b[0]))
        out["bias_nm"].append(float(b[1]))
        out["hold_friction_nm"].append(float(np.median(fr[:, j])))
        out["loo_rms_nm"].append(float(np.sqrt(np.mean(np.square(e)))))
        out["server_rms_nm"].append(float(np.sqrt(np.mean((y[:, j] - now) ** 2))))
        print("  %-9s %+5.2f..%+5.2f  %.2f  %.2f  %.2f  %.2f, %+.2f" % (NAMES[j], y[:, j].min(), y[:, j].max(), out["hold_friction_nm"][j], out["server_rms_nm"][j], out["loo_rms_nm"][j], b[0], b[1]))
    return out, Q, y, M


def fit_motion(runs, g, grav):
    cou, vis, ine, rms, rms0 = [], [], [], [], []
    X, Y = [[] for _ in range(5)], [[] for _ in range(5)]
    for D in runs:
        k = np.array([p[0] == "m" and p[1:].split("_")[0].isdigit() for p in D["ph"]])
        if not k.any():
            continue
        ddq = np.gradient(D["dq"], D["t"], axis=0)
        for i in np.where(k)[0][::2]:
            G = g(D["q"][i])
            for j in range(5):
                if abs(D["dq"][i, j]) > 0.02:
                    X[j].append([np.sign(D["dq"][i, j]), D["dq"][i, j], ddq[i, j]])
                    Y[j].append(D["tau"][i, j] - (grav["scale"][j] * G[j] + grav["bias_nm"][j]))
    if not X[0]:
        return None
    print("motion: per joint Coulomb N m, viscous N m s/rad, inertia kg m^2 (fit to measured torque), rms N m: gravity only, with these")
    for j in range(5):
        A, b = np.array(X[j]), np.array(Y[j])
        c = np.linalg.lstsq(A, b, rcond=None)[0]
        cou.append(float(c[0]))
        vis.append(float(c[1]))
        ine.append(float(c[2]))
        rms0.append(float(np.sqrt(np.mean(b ** 2))))
        rms.append(float(np.sqrt(np.mean((b - A @ c) ** 2))))
        print("  %-9s %.2f  %+.2f  %+.3f  %.2f  %.2f  (%d rows)" % (NAMES[j], c[0], c[1], c[2], rms0[j], rms[j], len(b)))
    return dict(coulomb_nm=cou, viscous_nm_per_rad_s=vis, inertia_kgm2=ine, rms_gravity_only_nm=rms0, rms_nm=rms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out")
    ap.add_argument("--png")
    a = ap.parse_args()
    runs = [load(d) for d in a.dirs]
    sides = {D["side"] for D in runs}
    if len(sides) != 1:
        raise SystemExit("one arm at a time: %s" % sorted(sides))
    side = sides.pop()
    print("R1 %s arm, %s" % (side, [os.path.basename(os.path.normpath(d)) for d in a.dirs]))
    g = Grav(side, np.nanmedian(np.vstack([D["hand"] for D in runs]), axis=0))
    if not any(D["mode"] == "poses" for D in runs):
        raise SystemExit("needs an armwide_poses run")
    grav, Q, y, M = fit_poses([D for D in runs if D["mode"] == "poses"], g)
    fric = fit_motion([D for D in runs if D["mode"] == "motion"], g, grav)
    if fric is None:
        print("no armwide_motion run: friction from the holds only, which reads low")
        fric = dict(coulomb_nm=grav["hold_friction_nm"])
    if a.out:
        json.dump(dict(robot="R1 %s arm" % side, simulator_note="R1 measurements for the MuJoCo scene; not a MuJoCo result", joints=NAMES, runs=[os.path.basename(os.path.normpath(d)) for d in a.dirs],
                       parts=dict(grav=grav, fric=fric)), open(a.out, "w"), indent=1)
        print(a.out)
    png = a.png or os.path.join(a.dirs[0], "armwide_gravity.png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 5, figsize=(14, 3))
    for j, x in enumerate(ax):
        x.plot(M[:, j], y[:, j], "o", ms=4, color="#2a6fb0")
        s = np.linspace(M[:, j].min(), M[:, j].max(), 2)
        x.plot(s, grav["scale"][j] * s + grav["bias_nm"][j], color="#c2571a", lw=1)
        x.set(title="%s: %.2f x + %.2f" % (NAMES[j], grav["scale"][j], grav["bias_nm"][j]), xlabel="URDF gravity, N m")
        x.grid(alpha=0.3)
    ax[0].set_ylabel("held torque, N m")
    fig.suptitle("R1 %s arm: torque held across the working range" % side, y=1.0)
    fig.tight_layout()
    fig.savefig(png, dpi=130)
    print(png)


if __name__ == "__main__":
    main()
