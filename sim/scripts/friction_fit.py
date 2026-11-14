"""Find the friction values that reproduce the R1's push10 runs, from friction_batch.py output.

    python scripts/friction_fit.py data/results/friction_replay/grid_v1
"""
import glob
import json
import os
import sys
from collections import defaultdict

import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts"))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


_RUNS = {}


def phase_errors(s):
    """rms error (mm) of the top plate's xy displacement, MuJoCo vs camera, before the push ('pre': up to the first
    engage, which holds the touches and the edge slide) and during it ('push': engage to the window end)."""
    import friction_replay as F
    if s["run"] not in _RUNS:
        L = F.load_run(s["run"])
        t_eng = next((e["at_s"] for e in L["ev"] if e.get("what") == "push10 engaged" and e["at_s"] > s["window_s"][0]), 1e9)
        _RUNS[s["run"]] = t_eng
    t_eng = _RUNS[s["run"]]
    Z = np.load(s["_npz"])
    ok = ~np.isnan(Z["cam"][:, 0])
    e = np.linalg.norm(Z["plate"][:, :2] - Z["cam"], axis=1)
    pre, push = ok & (Z["t"] < t_eng), ok & (Z["t"] >= t_eng)
    rms = lambda m: float(np.sqrt(np.mean(e[m] ** 2))) if m.any() else np.nan
    return rms(pre), rms(push), float(np.linalg.norm(Z["plate"][-1][:2]))


def main():
    out = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "all"    # all | pre | push
    S = []
    for f in glob.glob(os.path.join(out, "*", "pad*.json")):
        s = json.load(open(f))
        s["_npz"] = f[:-5] + ".npz"
        S.append(s)
    runs = sorted({s["run"] for s in S})
    tbs = sorted({s["table_mu"] for s in S})
    pads = sorted({s["pad"] for s in S})
    fgs = sorted({s["fg"] for s in S})
    E = defaultdict(list)       # (run, pad, fg, tb) -> errors over dy
    for s in S:
        if s.get("err_rms_mm") is None or not os.path.exists(s["_npz"]):
            continue
        e_pre, e_push, s["sim_end_disp_mm"] = phase_errors(s)
        v = dict(all=s["err_rms_mm"], pre=e_pre, push=e_push)[which]
        if not np.isnan(v):
            E[(s["run"], s["pad"], s["fg"], s["table_mu"])].append(v)
    res = {}
    for tb in tbs:
        M = np.full((len(fgs), len(pads)), np.nan)
        per = {}
        for i, fg in enumerate(fgs):
            for j, pad in enumerate(pads):
                e = [np.mean(E[(r, pad, fg, tb)]) for r in runs if E.get((r, pad, fg, tb))]
                if e:
                    M[i, j] = np.median(e)
                    per["pad%.2f_fg%.3f" % (pad, fg)] = dict(median=round(float(np.median(e)), 2),
                                                             per_run=[round(float(x), 1) for x in e], n=len(e))
        res["table%.2f" % tb] = dict(pads=pads, fgs=fgs, median_err_mm=np.round(M, 2).tolist(), cells=per)
        fig, ax = plt.subplots(figsize=(6, 3.6))
        im = ax.imshow(M, origin="lower", cmap="viridis_r", aspect="auto")
        ax.set_xticks(range(len(pads)), ["%.2f" % p for p in pads])
        ax.set_yticks(range(len(fgs)), ["%.3f" % f for f in fgs])
        ax.set_xlabel("fingertip pad on glaze")
        ax.set_ylabel("plate foot on glaze")
        for i in range(len(fgs)):
            for j in range(len(pads)):
                if not np.isnan(M[i, j]):
                    ax.text(j, i, "%.1f" % M[i, j], ha="center", va="center", color="w", fontsize=9)
        fig.colorbar(im, label="median rms error, mm")
        ax.set_title("MuJoCo %s vs R1 camera, top plate motion, %s phase (%d runs, table %.2f)" % (S[0].get("mujoco", ""), which, len(runs), tb), fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "fit_map_%s_table%.2f.png" % (which, tb)), dpi=130)
        plt.close(fig)
    json.dump(res, open(os.path.join(out, "fit_%s.json" % which), "w"), indent=1)
    # traces: per run, camera vs MuJoCo for each pad value at the middle plate-on-plate value and the first table value
    fg_mid, tb0 = fgs[len(fgs) // 2], tbs[0]
    n = len(runs)
    fig, axs = plt.subplots(n, 1, figsize=(9, 2.1 * n), squeeze=False)
    cols = plt.cm.plasma(np.linspace(0, 0.85, len(pads)))
    for k, r in enumerate(runs):
        ax = axs[k, 0]
        drawn = False
        for j, pad in enumerate(pads):
            ss = [s for s in S if s["run"] == r and s["pad"] == pad and s["fg"] == fg_mid and s["table_mu"] == tb0]
            for q, s in enumerate(ss):
                if not os.path.exists(s["_npz"]):
                    continue
                Z = np.load(s["_npz"])
                if not drawn:
                    ax.plot(Z["t"], Z["cam"][:, 1], "k.", ms=2, label="R1 head camera")
                    drawn = True
                ax.plot(Z["t"], Z["plate"][:, 1], color=cols[j], lw=1, alpha=0.8,
                        label="MuJoCo pad %.2f" % pad if q == 0 else None)
        ax.set_ylabel("top plate y, mm")
        ax.set_title(r, fontsize=8)
        if k == 0:
            ax.legend(fontsize=6, ncol=3)
    axs[-1, 0].set_xlabel("run time, s")
    fig.suptitle("Top plate displacement along y: R1 camera vs MuJoCo (plate on plate %.3f, table %.2f)" % (fg_mid, tb0), fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "fit_traces.png"), dpi=110)
    print(json.dumps(res, indent=1))


def drag_figure(out, reg_path, tol=1.5):
    """Edge-slide drag (y, mm) vs fingertip friction, one panel per slide run, a line per plate-on-plate value at the
    registered stack shift (band: the other shifts), the camera's value as a black line. Also the fingertip range that
    reproduces the camera within tol mm, per run and plate-on-plate value."""
    R = json.load(open(os.path.join(out, "observables.json")))
    reg = json.load(open(reg_path))
    runs = [r for r in sorted(R) if R[r]["camera"]["drag_y_mm"] is not None]
    fig, axs = plt.subplots(1, len(runs), figsize=(3.3 * len(runs), 3.2), squeeze=False)
    fit = {}
    for k, run in enumerate(runs):
        ax = axs[0, k]
        cam = R[run]["camera"]["drag_y_mm"]
        rows = defaultdict(lambda: defaultdict(dict))
        for key, v in R[run].items():
            if key == "camera" or v["drag_y_mm"] is None:
                continue
            pad, fg, dy = float(key.split("_")[0][3:]), float(key.split("_")[1][2:]), float(key.split("dy")[1])
            rows[fg][pad][dy] = v["drag_y_mm"]
        fit[run] = dict(camera=round(cam, 1))
        for c, fg in zip(("tab:blue", "tab:green", "tab:red"), sorted(rows)):
            pads = sorted(rows[fg])
            best = [rows[fg][p].get(float(reg[run]["dy_best"]), np.nan) for p in pads]
            lo = [min(rows[fg][p].values()) for p in pads]
            hi = [max(rows[fg][p].values()) for p in pads]
            ax.fill_between(pads, lo, hi, color=c, alpha=0.12)
            ax.plot(pads, best, "o-", color=c, ms=3, label="plate on plate %.3f" % fg)
            ok = [p for p, b in zip(pads, best) if not np.isnan(b) and abs(b - cam) <= tol]
            fit[run]["fg%.3f" % fg] = [min(ok), max(ok)] if ok else None
        ax.axhline(cam, color="k", lw=1.5, label="R1 head camera")
        ax.set_title(run.replace("_push10", ""), fontsize=8)
        ax.set_xlabel("fingertip friction")
        if k == 0:
            ax.set_ylabel("plate dragged outward during the edge slide, mm")
            ax.legend(fontsize=6)
    fig.suptitle("Edge-slide drag: R1 camera vs MuJoCo 3.14 replay of the R1's own run (table friction 0.9)", fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "drag_vs_pad.png"), dpi=130)
    json.dump(fit, open(os.path.join(out, "drag_fit.json"), "w"), indent=1)
    return fit


if __name__ == "__main__" and not (len(sys.argv) > 2 and sys.argv[2] == "obs"):
    main()


def observables(out):
    """Per run: the plate's y drag during the edge slide (median over the 'set' hold, slide runs only) and its y
    displacement after the push (median over the last 1.5 s), camera vs MuJoCo per friction set. Writes observables.json
    and observables.png."""
    import friction_replay as F
    S = []
    for f in glob.glob(os.path.join(out, "*", "pad*.json")):
        s = json.load(open(f))
        s["_npz"] = f[:-5] + ".npz"
        if os.path.exists(s["_npz"]):
            S.append(s)
    res = defaultdict(dict)
    for run in sorted({s["run"] for s in S}):
        L = F.load_run(run)
        ss = [s for s in S if s["run"] == run]
        t0 = ss[0]["window_s"][0]
        t_set = next((e["at_s"] for e in L["ev"] if e.get("what") == "push10 set" and e["at_s"] > t0), None)
        t_eng = next((e["at_s"] for e in L["ev"] if e.get("what") == "push10 engaged" and e["at_s"] > t0), None)
        slide = any(e.get("what") == "push10 edge" and "index_y_mm" in e for e in L["ev"])
        Z0 = np.load(ss[0]["_npz"])
        t = Z0["t"]

        def med(Z, key, a, b):
            m = (t >= a) & (t < b)
            v = Z[key][m][:, 1] if key != "cam" else Z["cam"][m][:, 1]
            v = v[~np.isnan(v)]
            return float(np.median(v)) if len(v) >= 3 else None
        real_drag = med(Z0, "cam", t_set, t_eng) if (slide and t_set and t_eng) else None
        real_push = med(Z0, "cam", t[-1] - 1.5, t[-1] + 1)
        res[run]["camera"] = dict(drag_y_mm=real_drag, push_y_mm=real_push, slide_edge=slide)
        for s in ss:
            Z = np.load(s["_npz"])
            k = "pad%.2f_fg%.3f_tb%.2f_dy%+.0f" % (s["pad"], s["fg"], s["table_mu"], s["dy"])
            res[run][k] = dict(drag_y_mm=med(Z, "plate", t_set, t_eng) if real_drag is not None else None,
                               push_y_mm=med(Z, "plate", t[-1] - 1.5, t[-1] + 1),
                               below_max_mm=s["below_max_mm"])
    json.dump(res, open(os.path.join(out, "observables.json"), "w"), indent=1)
    return res


if __name__ == "__main__" and len(sys.argv) > 2 and sys.argv[2] == "obs":
    R = observables(sys.argv[1])
    if len(sys.argv) > 3:
        print(json.dumps(drag_figure(sys.argv[1], sys.argv[3]), indent=1))
    for run, d in R.items():
        print(run, "camera", d["camera"])
        for k in sorted(d):
            if k != "camera":
                print("   ", k, d[k])
