"""Analyse nearforce runs: one line per try and a figure per run folder.

    python calibration/nearforce_fit.py logs/tests/<time>_nearforce_tip...      -> one line per try, nearforce_<mode>.png in each folder
    python calibration/nearforce_fit.py --table sheet.csv logs/tests/*_nearforce* # also a table in record_sheet_robot.csv's columns
    python calibration/nearforce_fit.py --edge-gap-mm 6 logs/tests/<time>_nearforce_tip
"""
import argparse
import csv
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLATE_N, FOOT_MM, R_MM = 5.126, 77.5, 127.5
BLUE, ORANGE, GREEN, PURPLE, RED = "#2a78b5", "#d9722b", "#3a9d6a", "#8d62b0", "#b8324f"


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, "nearforce_cmd.csv"))))

    def g(k):
        return np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows])
    return rows, g, np.array([r["phase"] for r in rows])


def events(d):
    p = os.path.join(d, "events.jsonl")
    return [json.loads(x) for x in open(p)] if os.path.exists(p) else []


def plate_path(g):
    """The plate's travel in the plane, mm: the marker's where it has one, else the rim tracker's (each against its own start)."""
    mk = np.c_[g("mk_x"), g("mk_y")] * 1000
    cam = np.c_[g("cam_x"), g("cam_y")] * 1000
    return mk, cam


def first_good(x):
    k = np.where(~np.isnan(x).any(axis=1))[0]
    return x[k[0]] if len(k) else np.array([np.nan, np.nan])


def plot_run(d, mode, g, ph, gain):
    t = g("t")
    fig, ax = plt.subplots(4, 1, figsize=(12, 9.5), sharex=True)
    ax[0].plot(t, g("Fz"), color=ORANGE, label="Fz commanded")
    ax[0].plot(t, g("H"), color=BLUE, label="H commanded")
    ax[0].plot(t, g("lag_Fz"), color="#888", lw=0.7, label="joint-lag Fz, up on the hand")
    ax[0].set_ylabel("N")
    ax[1].plot(t, g("idx_prox"), color=GREEN, lw=0.9, label="index proximity")
    ax[1].plot(t, g("mid_prox"), color=PURPLE, lw=0.9, label="middle proximity")
    ax[1].plot(t, 1000 * g("idx_normal"), color=GREEN, lw=0.6, ls=":", label="index normal x 1000")
    ax[1].plot(t, 1000 * g("mid_normal"), color=PURPLE, lw=0.6, ls=":", label="middle normal x 1000")
    ax[1].set_ylabel("counts")
    mk, cam = plate_path(g)
    m0, c0 = first_good(mk), first_good(cam)
    ax[2].plot(t, mk[:, 0] - m0[0], color=BLUE, label="marker x")
    ax[2].plot(t, mk[:, 1] - m0[1], color=ORANGE, label="marker y")
    ax[2].plot(t, cam[:, 1] - c0[1], color=ORANGE, lw=0.7, ls=":", label="rim tracker y")
    ax[2].plot(t, 1000 * (g("idx_fk_y") - np.nanmedian(g("idx_fk_y")[ph == "press"])), color=GREEN, lw=0.8, label="index y (FK), from the press")
    ax[2].set_ylabel("mm")
    ax[2].set_ylim(-40, 40)
    ax[3].plot(t, 1000 * (g("idx_fk_z") - np.nanmedian(g("idx_fk_z")[ph == "press"])), color=BLUE, label="index z (FK)")
    ax[3].plot(t, 1000 * (g("tgt_z") - np.nanmedian(g("tgt_z")[ph == "press"])), color="#888", lw=0.7, label="touch target z")
    ax[3].plot(t, 1000 * (g("mid_fk_z") - np.nanmedian(g("mid_fk_z")[ph == "press"])), color=PURPLE, lw=0.8, label="middle z (FK)")
    ax[3].set_ylabel("mm, from the press")
    ax[3].set_ylim(-20, 40)
    ax[3].set_xlabel("s")
    last = None
    for ti, p in zip(t, ph):
        if p != last:
            for a in ax:
                a.axvline(ti, color="#bbb", lw=0.5)
            ax[0].annotate(p, (ti, 1.0), xycoords=("data", "axes fraction"), fontsize=7, rotation=30, va="bottom")
            last = p
    for a in ax:
        a.grid(alpha=0.2)
        a.legend(fontsize=7, loc="upper right", ncol=3)
    fig.suptitle("R1 nearforce_%s: %s" % (mode, os.path.basename(os.path.normpath(d))), fontsize=10, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    png = os.path.join(d, "nearforce_%s.png" % mode)
    fig.savefig(png, dpi=95)
    print("  " + png)


def plot_landing(d, L, res):
    """Top view: the plate, its foot ring and where the fingers are (FK + camera), in the plate's frame."""
    fig, ax = plt.subplots(figsize=(5.2, 5.2))
    u = np.linspace(0, 2 * np.pi, 361)
    for r, c, lab in ((R_MM, "#444", "edge"), (R_MM - 5.5, "#aaa", "rim"), (93.0, "#aaa", "first slope ends"), (FOOT_MM, "#ccc", "foot ring")):
        ax.plot(r * np.cos(u), r * np.sin(u), color=c, lw=1, label=lab)
    px, py = L["plan_xy_mm"]
    ax.plot(px, py, "x", color=BLUE, ms=9, label="planned index")
    for name, c in (("index", GREEN), ("middle", PURPLE)):
        x, y = L[name]["xy_mm"]
        ax.plot(x, y, "o", color=c, ms=7, label="%s: %.0f mm in from the edge" % (name, L[name]["edge_gap_mm"]))
    ax.set_aspect("equal")
    ax.set_xlim(-135, 0)
    ax.set_ylim(0, 135)
    ax.set_xlabel("mm, plate frame, robot forward +")
    ax.set_ylabel("mm, robot left +")
    ax.grid(alpha=0.2)
    ax.legend(fontsize=7, loc="lower right")
    ax.set_title("R1, where the fingers landed: %s" % os.path.basename(os.path.normpath(d)), fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "landing.png"), dpi=95)


def plot_ladder(d, lad):
    F = np.array([x["Fz"] for x in lad])
    k = int(np.argmax(F)) + 1
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
    ax[0].plot(F[:k], [x["sink_mm"] for x in lad[:k]], "o-", color=BLUE, label="pressing")
    ax[0].plot(F[k - 1:], [x["sink_mm"] for x in lad[k - 1:]], "s--", color=ORANGE, label="releasing")
    ax[0].set_xlabel("commanded press, N")
    ax[0].set_ylabel("hand down since zero, mm")
    for i, (name, c) in enumerate((("index", GREEN), ("middle", PURPLE))):
        ax[1].plot(F[:k], [x["prox"][i] for x in lad[:k]], "o-", color=c, label=name + " pressing")
        ax[1].plot(F[k - 1:], [x["prox"][i] for x in lad[k - 1:]], "s--", color=c, alpha=0.5, label=name + " releasing")
    ax[1].set_xlabel("commanded press, N")
    ax[1].set_ylabel("proximity, counts")
    for a in ax:
        a.grid(alpha=0.2)
        a.legend(fontsize=7)
    fig.suptitle("R1 press ladder: %s" % os.path.basename(os.path.normpath(d)), fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(d, "ladder.png"), dpi=95)


def analyse(d, a):
    rows, g, ph = load(d)
    s = json.load(open(os.path.join(d, "summary.json"))) if os.path.exists(os.path.join(d, "summary.json")) else {}
    res = next(iter(s.get("results", {}).values()), {})
    mode = res.get("mode", "full")
    print("%s  [%s]  %s" % (os.path.basename(os.path.normpath(d)), mode, s.get("ended")))
    gain = float(res.get("pad_N_per_count", 0.044))
    out = dict(dir=d, mode=mode, ended=s.get("ended"))
    L = res.get("landing") or {}
    if L:
        print("  landing: index %s mm in from the edge, %s mm over the surface at first touch; middle %s, %s" % (
            L["index"]["edge_gap_mm"], L["index"]["above_surface_at_touch_mm"], L["middle"]["edge_gap_mm"], L["middle"]["above_surface_at_touch_mm"]))
        out["landing"] = L
        plot_landing(d, L, res)
    if mode == "tip" and res.get("result") == "marker":
        r = res["lever_r_mm"]
        if a.edge_gap_mm is not None:
            print("  lever from the arm %.1f mm; ruler says %.1f" % (r, R_MM - a.edge_gap_mm))
            r = R_MM - a.edge_gap_mm
        F = PLATE_N * FOOT_MM / (r - FOOT_MM)
        lo, hi = PLATE_N * FOOT_MM / (r + 3 - FOOT_MM), PLATE_N * FOOT_MM / (r - 3 - FOOT_MM)
        print("  tip: lever r %.1f mm (index %s, middle %s) -> true force %.2f N (%.2f-%.2f at +-3 mm); commanded %.2f -> true / commanded %.2f; proximity %s; hand down %s mm" % (
            r, res.get("lever_r_index_mm"), res.get("lever_r_middle_mm"), F, lo, hi, res["Fz_at_tip"], F / max(res["Fz_at_tip"], 0.1), res.get("prox_at_tip"), res.get("sink_mm")))
        out.update(true_force_N=F, true_force_lo_N=lo, true_force_hi_N=hi, commanded_N=res["Fz_at_tip"], true_over_commanded=F / max(res["Fz_at_tip"], 0.1), prox_at_tip=res.get("prox_at_tip"))
    elif mode == "tip":
        print("  tip: no tip (%s): no true force from this run" % res.get("result"))
    if mode == "press" and res.get("ladder"):
        lad = res["ladder"]
        F, z = np.array([x["Fz"] for x in lad]), np.array([x["sink_mm"] for x in lad])
        k = int(np.argmax(F)) + 1
        c = np.polyfit(F[:k], z[:k], 1) if k >= 2 else [np.nan, np.nan]
        same = [z[i] - z[2 * (k - 1) - i] for i in range(k - 1) if 2 * (k - 1) - i < len(z)]
        print("  ladder: %s; the hand gives %.2f mm per N pressing (%.2f N per mm); same force, pressing - releasing: %s mm" % ([(x["Fz"], x["sink_mm"], x["prox"]) for x in lad], c[0], 1 / max(c[0], 1e-6), np.round(same, 2)))
        out.update(ladder=lad, give_mm_per_N=float(c[0]), hysteresis_mm=[float(v) for v in same])
        plot_ladder(d, lad)
    if mode == "slip":
        for t in res.get("tries", []):
            print("  press %.0f N: %s at H %.2f; hand moved 1 mm at H %s, 3 mm at H %s; proximity %s -> %s; hand %.1f mm, plate %.1f mm" % (
                t["press"], t["result"], t.get("H", np.nan), t.get("H_hand_1mm"), t.get("H_hand_3mm"), t.get("prox_press"), t.get("prox_end"), t.get("hand_mm", np.nan), t.get("plate_mm", np.nan)))
        out["tries"] = res.get("tries", [])
        # H at the slide = friction x press + what the arm's joints take (armid_fric: 1-3 N at the pad): the slope is free of the second
        sl = [(t["press"], t.get("H_hand_3mm")) for t in res.get("tries", []) if t.get("result") == "slip" and t.get("H_hand_3mm") is not None]
        if len({p for p, _ in sl}) >= 2:
            mu, H0 = np.polyfit(*zip(*sl), 1)
            print("  commanded H at 3 mm of slide against the commanded press, %d slides: friction %.2f (slope), arm's share %.2f N (at zero press)" % (len(sl), mu, H0))
            out.update(mu_slope=float(mu), arm_share_N=float(H0), slides=sl)
        else:
            print("  too few press levels slid for a slope")
    if mode == "full" and (ph == "push").any():
        k = np.where(ph == "push")[0]
        mk, cam = plate_path(g)
        print("  try: %s at H %.2f (commanded), Fz %.2f; plate %s mm, hand %s mm; proximity %.0f, %.0f -> %.0f, %.0f" % (
            res.get("result"), g("H")[k][-1], g("Fz")[k][-1], res.get("moved_mm"), res.get("hand_mm"), g("idx_prox")[k[0]], g("mid_prox")[k[0]], g("idx_prox")[k[-1]], g("mid_prox")[k[-1]]))
        print("  body during the push: pitch %+.2f mrad, roll %+.2f mrad, waist roll %+.2f mrad; pelvis %+.1f mm" % (
            1e3 * (g("pitch")[k[-1]] - g("pitch")[k[0]]), 1e3 * (g("roll")[k[-1]] - g("roll")[k[0]]), 1e3 * (g("waist_roll")[k[-1]] - g("waist_roll")[k[0]]), 1e3 * (g("pelvis_m")[k[-1]] - g("pelvis_m")[k[0]])))
        out["sheet"] = res.get("sheet")
    plot_run(d, mode, g, ph, gain)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--edge-gap-mm", type=float, default=None, help="tip: the ruler's gap from the fingertips to the plate's outer edge")
    ap.add_argument("--table", default=None, help="write the tries here, record_sheet_robot.csv's columns that the robot knows")
    ap.add_argument("--out", default=None, help="join everything here for the simulation")
    a = ap.parse_args()
    all_ = []
    for d in a.dirs:
        if not os.path.exists(os.path.join(d, "nearforce_cmd.csv")):
            print("%s: no nearforce_cmd.csv" % d)
            continue
        all_.append(analyse(d, a))
    ratios = [o["true_over_commanded"] for o in all_ if o.get("true_over_commanded")]
    if ratios:
        print("true / commanded press from %d tips: median %.2f (%.2f-%.2f)" % (len(ratios), np.median(ratios), min(ratios), max(ratios)))
    if a.out:
        j = join(all_, ratios)
        with open(a.out, "w") as f:
            json.dump(j, f, indent=1)
        print("%s; still missing: %s" % (a.out, j["missing"]))
    tries = [dict(o["sheet"], dir=os.path.basename(os.path.normpath(o["dir"]))) for o in all_ if o.get("sheet")]
    if a.table and tries:
        keys = sorted({k for t in tries for k in t})
        with open(a.table, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(tries)
        print(a.table)


def join(all_, ratios):
    """What the simulation takes from these runs. Commanded forces stay commanded; a value with no run behind it is listed as missing, not guessed."""
    ls = [o["landing"] for o in all_ if o.get("landing")]
    land = {}
    for f in ("index", "middle"):
        for k in ("r_mm", "edge_gap_mm", "above_surface_at_touch_mm"):
            v = [x[f][k] for x in ls if x[f].get(k) is not None]
            if v:
                land["%s_%s" % (f, k)] = dict(median=float(np.median(v)), lo=float(min(v)), hi=float(max(v)), n=len(v))
    ladders = [o for o in all_ if o.get("ladder")]
    mus = [o for o in all_ if o.get("mu_slope") is not None]
    tries = [o["sheet"] for o in all_ if o.get("sheet")]
    q = float(np.median(ratios)) if ratios else None
    j = dict(simulator_note="R1 measurements for the MuJoCo scene; not a MuJoCo result", runs=[os.path.basename(os.path.normpath(o["dir"])) for o in all_], landing=land,
             true_over_commanded=dict(median=q, values=ratios) if ratios else None, tips=[{k: o.get(k) for k in ("true_force_N", "true_force_lo_N", "true_force_hi_N", "commanded_N", "prox_at_tip")} for o in all_ if o.get("true_force_N")],
             hand_give_mm_per_commanded_N=[o["give_mm_per_N"] for o in ladders], hysteresis_mm=[o["hysteresis_mm"] for o in ladders], ladder=[o["ladder"] for o in ladders],
             fingertip_friction_slope=[o["mu_slope"] for o in mus], arm_share_of_push_N=[o["arm_share_N"] for o in mus],
             fingertip_friction_true=[o["mu_slope"] / q for o in mus] if q else None, slides=[o["slides"] for o in mus], pushes=tries)
    miss = []
    if not land:
        miss.append("landing (nearforce_touch)")
    if not ratios:
        miss.append("true force (nearforce_tip tipping one plate)")
    if not ladders:
        miss.append("hand give (nearforce_press)")
    if not mus:
        miss.append("fingertip friction (nearforce_slip, at least 2 press levels sliding)")
    if not tries:
        miss.append("push outcomes (nearforce)")
    miss.append("plate on plate, foot on table, well depth (bench sheet)")
    j["missing"] = miss
    return j


if __name__ == "__main__":
    main()
