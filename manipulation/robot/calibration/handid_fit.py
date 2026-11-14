"""Fit a model of each Revo2 finger channel from handid_track (and pad offsets from handid_load), for the simulation.

    python calibration/handid_fit.py <handid log dir> [...] --out../models/sim_hand_model.json
"""
import argparse
import csv
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

CH = ["thumb bend", "thumb rot", "index", "middle", "ring", "little"]
PADS = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
PAD_CH = dict(t=(0, 0), i=(1, 2), m=(2, 3), r=(3, 4), p=(4, 5))         # letter -> (pad index, channel)
TOL = 0.03
DT = 0.005


def load(d):
    meta = json.load(open(os.path.join(d, "handid_meta.json")))
    R = list(csv.DictReader(open(os.path.join(d, "handid_cmd.csv"))))
    num = lambda v: float(v) if v not in ("", None) else np.nan          # noqa: E731
    D = dict(dir=d, meta=meta, ph=np.array([r["phase"] for r in R]), t=np.array([float(r["t"]) for r in R]))
    D["u"] = np.array([[float(r["hcmd%d" % i]) for i in range(12)] for r in R])
    D["y"] = np.array([[float(r["hq%d" % i]) for i in range(12)] for r in R])
    D["tn"] = np.array([[[num(r["tn_%s%d" % (s, i)]) for i in range(5)] for s in "LR"] for r in R])
    D["q"] = np.array([[float(r["q_meas%d" % i]) for i in range(10)] for r in R])
    return D


def hands_of(D):
    s = D["meta"]["side"]
    return [("left", 0), ("right", 6)] if s == "both" else [(s, 0 if s == "left" else 6)]


# ====================================================================== finger dynamics, every parameter set at once
def simulate(t, u, y0, model, grid):
    """Reported position at the log's times for every parameter set in grid (dict of equal-length arrays). The command linear between rows,
    sub-steps of DT."""
    d = grid["d"]
    n = len(t)
    P = len(d)
    ud = np.array([np.interp(t - dd, t, u) for dd in np.unique(d)])
    di = np.searchsorted(np.unique(d), d)
    y = np.full(P, y0, float)
    out = np.zeros((n, P))
    out[0] = y
    for k in range(1, n):
        u0, u1 = ud[di, k - 1], ud[di, k]
        h = t[k] - t[k - 1]
        m = max(1, int(np.ceil(h / DT)))
        hs = h / m
        for i in range(m):
            e = u0 + (u1 - u0) * (i + 1) / m - y                    # the command between two log rows: a straight line (the server ramps it)
            if model == "lag":
                y = y + e * (1 - np.exp(-hs / grid["tau"]))
            else:
                if model == "rate+band":
                    e = np.where(np.abs(e) < grid["b"] / 2, 0.0, e - np.sign(e) * grid["b"] / 2)
                y = y + np.clip(e * (1 - np.exp(-hs / 0.03)), -grid["r"] * hs, grid["r"] * hs)
        out[k] = y
    return out


def grids():
    ds = np.arange(0.0, 0.21, 0.02)
    g = {}
    D, T = np.meshgrid(ds, np.logspace(np.log10(0.02), 0.0, 14), indexing="ij")
    g["lag"] = dict(d=D.ravel(), tau=T.ravel())
    D, Rr = np.meshgrid(ds, np.linspace(0.2, 4.0, 20), indexing="ij")
    g["rate"] = dict(d=D.ravel(), r=Rr.ravel())
    D, Rr, B = np.meshgrid(ds, np.linspace(0.2, 4.0, 20), [0.0, 0.005, 0.01, 0.02, 0.04], indexing="ij")
    g["rate+band"] = dict(d=D.ravel(), r=Rr.ravel(), b=B.ravel())
    return g


def fit_channel(D, k, c):
    t, u, y = D["t"], D["u"][:, k + c], D["y"][:, k + c]
    chan = np.array([p.startswith("c%d_" % c) for p in D["ph"]])
    if chan.sum() < 30:
        return None
    # held out: every other single-channel test (step sizes 0.3 and 0.9, ramp speed 0.3,... both ways) and the all-fingers run
    keys = list(dict.fromkeys(p.replace("-", "") for p in D["ph"][chan]))
    out_keys = set(keys[1::2])
    held = (D["ph"] == "ms") | np.array([cc and p.replace("-", "") in out_keys for cc, p in zip(chan, D["ph"])])
    train = chan & ~held
    sel = chan | held
    i0, i1 = np.where(sel)[0][0], np.where(sel)[0][-1] + 1
    t, u, y, train, held, chan = t[i0:i1], u[i0:i1], y[i0:i1], train[i0:i1], held[i0:i1], chan[i0:i1]
    res = {}
    for model, g in grids().items():
        Y = simulate(t, u, y[0], model, g)
        e_in = np.sqrt(np.mean((Y[train] - y[train, None]) ** 2, axis=0))
        b = int(np.argmin(e_in))
        e_out = float(np.sqrt(np.mean((Y[held, b] - y[held]) ** 2)))
        e_all = np.sqrt(np.mean((Y[chan] - y[chan, None]) ** 2, axis=0))     # the numbers kept: refit on every single-channel test
        ba = int(np.argmin(e_all))
        res[model] = dict(params={kk: float(v[ba]) for kk, v in g.items()}, in_rms=float(e_in[b]), heldout_rms=e_out)
    chosen = "lag"
    for model in ("rate", "rate+band"):
        if res[model]["heldout_rms"] < (1 - TOL) * res[chosen]["heldout_rms"]:
            chosen = model
    p = res[chosen]["params"]
    out = dict(chosen=chosen, scored_on="held out: every other single-channel test and the all-fingers run", held_rows=int(held.sum()),
               delay_s=p["d"], rate_per_s=p.get("r"), tau_s=p.get("tau"), dead_band=p.get("b", 0.0), models=res)
    # static map: held levels, up and down
    lv = {}
    for ph in set(D["ph"]):
        m = re.match(r"c%d_lv([0-9.]+)_(up|dn)$" % c, ph)
        if m:
            kk = np.where(D["ph"] == ph)[0]
            kk = kk[len(kk) // 2:]
            lv.setdefault(float(m.group(1)), {})[m.group(2)] = float(np.median(D["y"][kk, k + c]))
    if lv:
        xs = sorted(lv)
        mean = [np.mean(list(lv[x].values())) for x in xs]
        sc, off = np.polyfit(xs, mean, 1) if len(xs) > 1 else (1.0, 0.0)
        hys = [lv[x]["dn"] - lv[x]["up"] for x in xs if "up" in lv[x] and "dn" in lv[x]]
        out.update(scale=float(sc), offset=float(off), hysteresis=float(np.mean(hys)) if hys else None, levels={"%g" % x: lv[x] for x in xs})
    out["_trace"] = (t, u, y, simulate(t, u, y[0], chosen, {kk: np.array([v]) for kk, v in p.items()})[:, 0], held)
    return out


# ====================================================================== pads under hung weights
def fit_pads(D):
    side = D["meta"]["side"]
    sk = 0 if side == "left" else 1
    k0 = 0 if side == "left" else 6
    rows = {}
    for ph in sorted(set(D["ph"])):
        m = re.match(r"load_([timrp])([0-9.]+)_c([0-9.]+)$", ph)
        if not m:
            continue
        kk = np.where(D["ph"] == ph)[0]
        kk = kk[len(kk) // 2:]
        pi, ch = PAD_CH[m.group(1)]
        F = float(m.group(2)) * 9.81e-3
        rows.setdefault((m.group(1), float(m.group(3))), []).append((F, float(np.nanmedian(D["tn"][kk, sk, pi])), float(np.median(D["y"][kk, k0 + ch])),
                                                                    float(np.nanstd(D["tn"][kk, sk, pi]))))
    out = {}
    for (f, curl), r in rows.items():
        r = np.array(sorted(r))
        F, cnt, pos = r[:, 0], r[:, 1], r[:, 2]
        rec = dict(finger=PADS[PAD_CH[f][0]], curl=curl, force_N=F.tolist(), counts=cnt.tolist(), position=pos.tolist())
        if len(F) >= 3 and np.ptp(F) > 0:
            lin = np.polyfit(F, cnt, 1)
            rec["counts_per_N"], rec["counts_at_0"] = float(lin[0]), float(lin[1])
            rec["give_per_N"] = float(np.polyfit(F, pos, 1)[0])
            if len(F) >= 4 and np.all(cnt - cnt[0] >= 0):              # straight line or power law: each load left out
                e_lin, e_pow = [], []
                for i in range(len(F)):
                    m_ = np.ones(len(F), bool)
                    m_[i] = False
                    e_lin.append(cnt[i] - np.polyval(np.polyfit(F[m_], cnt[m_], 1), F[i]))
                    pos_ = (F[m_] > 0) & (cnt[m_] - cnt[0] > 0)
                    if pos_.sum() >= 2 and F[i] > 0:
                        b, a = np.polyfit(np.log(F[m_][pos_]), np.log(cnt[m_][pos_] - cnt[0]), 1)
                        e_pow.append(cnt[i] - (cnt[0] + np.exp(a) * F[i] ** b))
                rec["loo_rms_counts"] = dict(line=float(np.sqrt(np.mean(np.square(e_lin)))), power=float(np.sqrt(np.mean(np.square(e_pow)))) if e_pow else None)
        out["%s_c%g" % (f, curl)] = rec
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--out")
    ap.add_argument("--label", default="R1 measurements; not a MuJoCo result", help="where the logs came from, for the figure and the file")
    a = ap.parse_args()
    res = dict(robot="R1 BrainCo Revo2 hands", simulator_note=a.label, runs=[], hands={}, pads={}, free_air_pads={})
    traces = []
    for d in a.dirs:
        D = load(d)
        res["runs"].append(os.path.basename(os.path.normpath(d)))
        if D["meta"]["mode"] == "track":
            for side, k in hands_of(D):
                sk = 0 if side == "left" else 1
                res["free_air_pads"][side] = dict(median=np.nanmedian(D["tn"][:, sk], axis=0).tolist(), p99=np.nanpercentile(D["tn"][:, sk], 99, axis=0).tolist()) \
                    if np.isfinite(D["tn"][:, sk]).any() else None
                for c in range(6):
                    f = fit_channel(D, k, c)
                    if f is None:
                        continue
                    traces.append(("%s %s" % (side, CH[c]), f.pop("_trace")))
                    res["hands"].setdefault(side, {})[CH[c]] = f
                    print("%-5s %-10s %-9s delay %3.0f ms  speed %s  tau %s  band %.3f  rms %s%s" % (
                        side, CH[c], f["chosen"], 1000 * f["delay_s"], "-" if f["rate_per_s"] is None else "%.2f/s" % f["rate_per_s"],
                        "-" if f["tau_s"] is None else "%.2f s" % f["tau_s"], f["dead_band"], {m: round(v["heldout_rms"] if v["heldout_rms"] is not None else v["in_rms"], 4) for m, v in f["models"].items()},
                        "" if "scale" not in f else "; static x%.3f %+.3f, hysteresis %s" % (f["scale"], f["offset"], None if f["hysteresis"] is None else round(f["hysteresis"], 4))))
        else:
            p = fit_pads(D)
            res["pads"][D["meta"]["side"]] = p
            for kk, r in p.items():
                print("pads %-5s %-6s curl %.1f: %s N -> counts %s; %s counts/N, give %s per N" % (D["meta"]["side"], r["finger"], r["curl"], np.round(r["force_N"], 2), np.round(r["counts"], 0),
                                                                                            r.get("counts_per_N") and round(r["counts_per_N"], 1), r.get("give_per_N") and round(r["give_per_N"], 4)))
    if a.out:
        json.dump(res, open(a.out, "w"), indent=1)
        print(a.out)
        if traces:
            draw(a.out[:-5] + ".png", traces, a.label)


def runs_of(mask):
    out, i = [], 0
    while i < len(mask):
        if mask[i]:
            j = i
            while j < len(mask) and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        i += 1
    return out


def draw(path, traces, label):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(traces)
    fig, ax = plt.subplots(n, 1, figsize=(13, 1.9 * n), squeeze=False)
    for a_, (name, (t, u, y, s, held)) in zip(ax[:, 0], traces):
        a_.plot(t, u, color="#bbbbbb", lw=1, label="command")
        a_.plot(t, y, color="#2a6fb0", lw=1.2, label="reported")
        a_.plot(t, s, color="#c2571a", lw=1, ls="--", label="model")
        for i0, i1 in runs_of(held):
            a_.axvspan(t[i0], t[i1 - 1], color="#f2e6c9", alpha=0.6, lw=0)
        a_.set_ylabel(name, fontsize=8)
        a_.grid(alpha=0.3)
    ax[0, 0].legend(fontsize=7, loc="upper right")
    ax[-1, 0].set_xlabel("s (shaded: held out of the fit that chose the model)")
    fig.suptitle("Revo2 fingers: command (as the server sent it), reported position, model (%s)" % label)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    print(path)


if __name__ == "__main__":
    main()
