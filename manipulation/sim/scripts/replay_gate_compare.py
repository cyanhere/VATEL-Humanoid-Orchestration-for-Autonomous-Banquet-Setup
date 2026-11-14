"""Compare five arm models replayed against held-out robot motions: rms per joint per model, and a figure."""
import csv
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))   # sim/
RES = os.path.join(ROOT, "data", "results", "replay_arm")
LOGS = os.path.join(os.path.dirname(ROOT), "data", "robot_pick_logs", "logs", "tests")
RUNS = {"left": "20261105_135848_armwide_motion", "right": "20261105_151004_armwide_motion"}
NAMES = ["sh pitch", "sh roll", "sh yaw", "elbow", "wrist"]
VAR = [("D", "URDF, no friction"), ("C", "fitted gravity, no friction"), ("E", "fitted gravity + friction without memory"),
       ("B", "fitted gravity + friction with memory"), ("A", "as B, torque scale from the bag")]
COL = {"D": "#9a9a9a", "C": "#e0a13a", "E": "#7b52c9", "B": "#2a6fdb", "A": "#2e9e5b"}


def real(side, t_end):
    R = [r for r in csv.DictReader(open(os.path.join(LOGS, RUNS[side], "armwide_cmd.csv"))) if float(r["t"]) <= t_end]
    return np.array([float(r["t"]) for r in R]), np.array([[float(r["q_meas%d" % i]) for i in range(5)] for r in R]), \
        np.array([[float(r["q_des%d" % i]) for i in range(5)] for r in R]), np.array([r["phase"] for r in R])


def main():
    out = {}
    for side in RUNS:
        t, qm, qd, ph = real(side, 95.0)
        moving = np.array([p[0] == "m" and p[1:].split("_")[0].isdigit() for p in ph])
        out[side] = {"plan": 1000 * np.sqrt(np.mean((qd[moving] - qm[moving]) ** 2, axis=0))}
        print("== %s arm, %d s of held-out random motion; rms of simulated - measured joint angle, mrad (MuJoCo %s)" % (side, int(t[-1]), "3.x"))
        print("   %-44s %s" % ("the plan itself (no simulation)", np.round(out[side]["plan"]).astype(int)))
        for k, name in VAR:
            f = os.path.join(RES, "gate_%s_%s.npz" % (side, k))
            if not os.path.exists(f):
                continue
            z = np.load(f)
            n = min(len(z["q"]), len(qm))
            e = z["q"][:n][moving[:n]] - qm[:n][moving[:n]]
            out[side][k] = 1000 * np.sqrt(np.mean(e ** 2, axis=0))
            out[side][k + "_q"] = z["q"][:n]
            print("   %s %-42s %s  mean %.0f" % (k, name, np.round(out[side][k]).astype(int), out[side][k].mean()))
        out[side]["t"], out[side]["qm"] = t, qm
    draw(out, sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "replay_gate_20261106.png"))


def draw(out, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 2, figsize=(15, 8), gridspec_kw=dict(width_ratios=[1.0, 1.6]))
    for r, side in enumerate(RUNS):
        o = out[side]
        a = ax[r, 0]
        ks = [k for k, _ in VAR if k in o]
        w = 0.8 / max(1, len(ks))
        for i, k in enumerate(ks):
            a.bar(np.arange(5) + (i - (len(ks) - 1) / 2) * w, o[k], w * 0.92, color=COL[k], label=dict(VAR)[k])
        a.set_xticks(range(5)); a.set_xticklabels(NAMES)
        a.set_ylabel("rms, simulated - measured (mrad)")
        a.set_title("%s arm: how far MuJoCo's joints are from the R1's" % side, fontsize=10, loc="left")
        a.legend(fontsize=7)
        a.grid(axis="y", alpha=0.3)
        a = ax[r, 1]
        j = int(np.argmax(o.get("D", np.zeros(5))))           # the joint the URDF gets most wrong
        t = o["t"]
        k0 = (t >= 50) & (t <= 80)
        a.plot(t[k0], o["qm"][k0, j], color="black", lw=2.2, label="R1, measured")
        for k in ("D", "E", "B"):
            if k + "_q" in o:
                n = len(o[k + "_q"])
                kk = k0[:n]
                a.plot(t[:n][kk], o[k + "_q"][kk, j], color=COL[k], lw=1.2, label="MuJoCo: " + dict(VAR)[k])
        a.set_title("%s %s, 30 s of the held-out motion at 0.6 of the speed limit" % (side, NAMES[j]), fontsize=10, loc="left")
        a.set_xlabel("s"); a.set_ylabel("rad"); a.legend(fontsize=7); a.grid(alpha=0.3)
    for a in ax.flat:
        a.spines[["top", "right"]].set_visible(False)
    fig.suptitle("MuJoCo replay of the R1's held-out random motions (open loop, the motor commands as logged on 2026-11-05)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=105)
    print(path)


if __name__ == "__main__":
    main()
