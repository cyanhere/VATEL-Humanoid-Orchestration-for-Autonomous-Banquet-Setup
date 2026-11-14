"""Compare a replay_real.py run with the R1 log it replays: joints and index pad, whole run and at the holds.

    python scripts/replay_arm_gate.py <armid_cmd.csv> <out.npz> [more.npz...] [--png fig.png]
"""
import argparse
import csv

import numpy as np

JOINTS = ["sh pitch", "sh roll", "sh yaw", "elbow", "wrist"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--png")
    a = ap.parse_args()
    R = list(csv.DictReader(open(a.log)))
    t = np.array([float(r["t"]) for r in R])
    q = np.array([[float(r["q_meas%d" % i]) for i in range(5)] for r in R])
    idx = np.array([[float(r["idx_fk_" + c]) for c in "xyz"] for r in R])
    plot = []
    for p in a.runs:
        Z = np.load(p)
        n = len(Z["t"])                                     # a replay may stop early (--t_end)
        e = 1000 * (Z["q"] - q[:n])
        pad = 1000 * (Z["idx"] - idx[:n])
        print("%s\n  joints rms mrad %s\n  joints mean mrad %s\n  pad rms %.1f mm (x %.1f y %.1f z %.1f); after one constant %.1f mm" % (
            p, np.round(np.sqrt(np.mean(e ** 2, 0)), 1), np.round(np.mean(e, 0), 1), np.sqrt(np.mean(np.sum(pad ** 2, 1))),
            *np.sqrt(np.mean(pad ** 2, 0)), np.sqrt(np.mean(np.sum((pad - pad.mean(0)) ** 2, 1)))))
        plot.append((p, e))
    if a.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, figsize=(10, 9), sharex=True)
        for i, x in enumerate(ax):
            for p, e in plot:
                x.plot(t[:len(e)], e[:, i], lw=0.8, label=p.replace("\\", "/").split("/")[-1])
            x.set_ylabel(JOINTS[i] + "\nmrad")
            x.grid(alpha=0.3)
        ax[0].legend(fontsize=7)
        ax[0].set_title("MuJoCo replay minus R1 joints, same run")
        ax[-1].set_xlabel("s")
        fig.tight_layout()
        fig.savefig(a.png, dpi=130)


if __name__ == "__main__":
    main()
