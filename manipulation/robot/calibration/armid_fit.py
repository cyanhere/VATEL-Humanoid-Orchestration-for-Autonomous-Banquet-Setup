"""Analyse the armid / armid_load runs: offset, gravity, friction and lag of the arm.

    python calibration/armid_fit.py logs/tests/<time>_armid            -> armid_fit.png in that folder, table on stdout
    python calibration/armid_fit.py logs/tests/<time>_armid_load
"""
import argparse
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

NAMES = ["sh pitch", "sh roll", "sh yaw", "elbow", "wrist roll"]
COL = ["#2a78b5", "#d9722b", "#3a9d6a", "#8d62b0", "#b8324f"]


def load(d):
    rows = list(csv.DictReader(open(os.path.join(d, "armid_cmd.csv"))))
    g = lambda fmt, ks: np.array([[float(r.get(fmt % k) or "nan") for k in ks] for r in rows])     # noqa: E731
    return dict(t=np.array([float(r["t"]) for r in rows]), ph=np.array([r["phase"] for r in rows]), qc=g("q_cmd%d", range(5)),
                qd=g("q_des%d", range(5)), q=g("q_meas%d", range(5)), dq=g("dq%d", range(5)), tau=g("tau%d", range(5)),
                hand=g("hand%d", range(6)), idx=g("idx_fk_%s", "xyz"), tgt=g("tgt_%s", "xyz"), F=g("lag_F%s", "xyz"),
                sent=g("sent%d", range(5)), hc=g("hand_cmd%d", range(6)), pads=g("pad_%s_N", ("thumb", "index", "middle", "ring", "pinky")))


def save_part(d, name, part):
    path = os.path.join(d, "sim_arm.json")
    part = json.loads(json.dumps(part, default=lambda x: np.asarray(x).tolist()))
    json.dump({name: part}, open(path, "w"), indent=1)
    print(path)


def fit_grav(D, d):
    """Torque halfway = gravity + the bias; half the gap = friction (per pose, per joint)."""
    from pk import config
    from pk.model import GravityComp, Robot
    tags = sorted({p.split("_")[0] for p in D["ph"] if p[0] == "g" and p[1:].split("_")[0].isdigit()}, key=lambda x: int(x[1:]))
    Q, Ta, Tb, Ea, Eb = [], [], [], [], []
    for tg in tags:
        ka, kb = np.where(D["ph"] == tg + "_a")[0], np.where(D["ph"] == tg + "_b")[0]
        if not len(ka) or not len(kb):
            continue
        ka, kb = ka[len(ka) // 2:], kb[len(kb) // 2:]
        Q.append(0.5 * (np.median(D["q"][ka], axis=0) + np.median(D["q"][kb], axis=0)))
        Ta.append(np.median(D["tau"][ka], axis=0))
        Tb.append(np.median(D["tau"][kb], axis=0))
        Ea.append(np.median((D["qc"] - D["q"])[ka], axis=0))
        Eb.append(np.median((D["qc"] - D["q"])[kb], axis=0))
    Q, Ta, Tb, Ea, Eb = map(np.array, (Q, Ta, Tb, Ea, Eb))
    tg_mid, fr = 0.5 * (Ta + Tb), 0.5 * np.abs(Ta - Tb)
    rob = Robot()
    hs = np.zeros(12)
    hs[:6] = np.nanmedian(D["hand"], axis=0)
    M = np.zeros_like(Q)
    a = np.zeros(config.N_ARM)
    for i, q in enumerate(Q):
        a[config.LEFT] = q
        M[i] = rob.gravity11(a, hs)[config.LEFT]
    gpath = os.path.join(HERE, "..", "model", "gravity_fit.json")
    now = None
    if os.path.exists(gpath):
        comp = GravityComp(rob, path=gpath)
        now = comp.scale[config.LEFT] * M + comp.bias_nm[config.LEFT]
    print("armid_grav: %d poses. Per joint: gravity torque's range, friction, fits (rms N m, leave one pose out)" % len(Q))
    print("  %-11s %-16s %-14s %-11s %-13s %-13s %-13s %s" % ("joint", "held torque", "friction", "fit now", "scale x URDF", "URDF + const", "scale + const", "fit: scale, const"))
    out = dict(poses=len(Q), scale=[], bias_nm=[], friction_nm=[], loo_rms_nm=[], fit_now_rms_nm=[])
    for j in range(5):
        y, m = tg_mid[:, j], M[:, j]

        def loo(fit):
            e = []
            for i in range(len(y)):
                k = np.arange(len(y)) != i
                e.append(y[i] - fit(m[k], y[k], m[i]))
            return float(np.sqrt(np.mean(np.square(e))))
        e_s = loo(lambda mm, yy, x: float(mm @ yy / max(mm @ mm, 1e-9)) * x)
        e_c = loo(lambda mm, yy, x: x + float(np.mean(yy - mm)))
        e_sc = loo(lambda mm, yy, x: np.polyval(np.polyfit(mm, yy, 1), x))
        b = np.polyfit(m, y, 1)
        e_now = float(np.sqrt(np.mean((y - now[:, j]) ** 2))) if now is not None else np.nan
        print("  %-11s %+5.2f..%+5.2f    %.2f (%.2f-%.2f)  %-11.2f %-13.2f %-13.2f %-13.2f %.2f, %+.2f" % (
            NAMES[j], y.min(), y.max(), np.median(fr[:, j]), fr[:, j].min(), fr[:, j].max(), e_now, e_s, e_c, e_sc, b[0], b[1]))
        out["scale"].append(float(b[0]))
        out["bias_nm"].append(float(b[1]))
        out["friction_nm"].append(float(np.median(fr[:, j])))
        out["loo_rms_nm"].append(e_sc)
        out["fit_now_rms_nm"].append(e_now)
    print("steady error cmd - joint, mrad, mean of the two approaches, across the poses (is a constant enough?)")
    em = 1e3 * 0.5 * (Ea + Eb)
    for j in range(5):
        print("  %-11s median %+6.1f, range %+6.1f..%+6.1f; half the gap from friction %.1f" % (NAMES[j], np.median(em[:, j]), em[:, j].min(), em[:, j].max(), 1e3 * np.median(0.5 * np.abs(Ea - Eb)[:, j])))
    out["steady_error_mrad"] = np.median(em, axis=0).tolist()
    out["steady_error_range_mrad"] = [em.min(axis=0).tolist(), em.max(axis=0).tolist()]
    fig, ax = plt.subplots(1, 5, figsize=(14, 3.3))
    for j in range(5):
        ax[j].plot(M[:, j], tg_mid[:, j], "o", color=COL[j], ms=4, label="R1 held torque")
        x = np.array([M[:, j].min(), M[:, j].max()])
        ax[j].plot(x, np.polyval(np.polyfit(M[:, j], tg_mid[:, j], 1), x), color="#444", lw=1, label="refit")
        if now is not None:
            ax[j].plot(M[:, j], now[:, j], ".", color="#b8324f", ms=3, label="fit running now")
        ax[j].set_title(NAMES[j], fontsize=9)
        ax[j].set_xlabel("URDF gravity torque, N m", fontsize=8)
        ax[j].grid(alpha=0.25)
    ax[0].set_ylabel("torque, N m")
    ax[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(d, "armid_grav.png"), dpi=100)
    print(os.path.join(d, "armid_grav.png"))
    save_part(d, "grav", out)


def fit_fric(D, d):
    """Mean torque while moving at steady speed: tau(v) = c + Fc sign(v) + b v, per joint; small sines: travel against command."""
    out = dict(coulomb_nm=[], viscous_nm_per_rad_s=[], levels=[], follow={})
    print("armid_fric: torque against speed (the mid 60% of each sweep; gravity removed as the mean of up and down)")
    fig, ax = plt.subplots(2, 5, figsize=(14, 5.6))
    for j in range(5):
        V, T = [], []
        for up_dn, sg in (("up", 1), ("dn", -1)):
            for p in sorted({x for x in D["ph"] if x.startswith("f%d_v" % j) and x.endswith(up_dn)}):
                k = np.where(D["ph"] == p)[0]
                k = k[int(0.2 * len(k)):int(0.8 * len(k))]
                if len(k) < 3:
                    continue
                V.append(sg * abs(float(np.median(D["dq"][k, j]))))
                T.append(float(np.median(D["tau"][k, j] - np.polyval(GRAV_DETREND.get(j, [0.0]), D["q"][k, j]))))
        V, T = np.array(V), np.array(T)
        if len(V) < 4:
            print("  %-11s too few sweeps" % NAMES[j])
            continue
        X = np.c_[np.ones(len(V)), np.sign(V), V]
        c, Fc, b = np.linalg.lstsq(X, T, rcond=None)[0]
        print("  %-11s Coulomb %.2f N m, viscous %.2f N m per rad/s, residual %.3f; per speed up %s down %s N m" % (
            NAMES[j], Fc, b, float(np.std(T - X @ [c, Fc, b])), np.round((T - c)[V > 0], 2), np.round((T - c)[V < 0], 2)))
        out["coulomb_nm"].append(float(Fc))
        out["viscous_nm_per_rad_s"].append(float(b))
        ax[0, j].plot(V, T - c, "o", color=COL[j], ms=4)
        x = np.linspace(V.min(), V.max(), 101)
        ax[0, j].plot(x, Fc * np.sign(x) + b * x, color="#444", lw=1)
        ax[0, j].set_title(NAMES[j], fontsize=9)
        ax[0, j].set_xlabel("joint speed, rad/s", fontsize=8)
        ax[0, j].grid(alpha=0.25)
        amp, fol = [], []
        for p in sorted({x for x in D["ph"] if x.startswith("d%d_a" % j)}, key=lambda x: float(x.split("_a")[1])):
            k = D["ph"] == p
            amp.append(float(p.split("_a")[1]))
            fol.append(float(np.ptp(D["q"][k, j]) / max(np.ptp(D["qc"][k, j]), 1e-6)))
        out["follow"][NAMES[j]] = dict(amp_mrad=amp, travel_over_command=fol)
        print("              small sines: amplitude %s mrad -> joint travel / command %s" % (amp, np.round(fol, 2)))
        ax[1, j].plot(amp, fol, "o-", color=COL[j], ms=4)
        ax[1, j].set_xlabel("sine amplitude, mrad", fontsize=8)
        ax[1, j].set_ylim(0, 1.3)
        ax[1, j].grid(alpha=0.25)
    ax[0, 0].set_ylabel("torque about the mean, N m")
    ax[1, 0].set_ylabel("joint travel / command")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "armid_fric.png"), dpi=100)
    print(os.path.join(d, "armid_fric.png"))
    save_part(d, "fric", out)


GRAV_DETREND = {}


def phasor(t, y, hz):
    w = 2 * np.pi * hz
    A = np.c_[np.sin(w * t), np.cos(w * t), np.ones(len(t)), t - t.mean()]
    b = np.linalg.lstsq(A, y, rcond=None)[0]
    return complex(b[0], b[1])


def fit_chirp(D, d):
    """Joint / sent at each frequency. H(s) = e^(-s T) k / (J s^2 + b s + k): the fitted delay T, natural frequency, damping. A joint's
    friction makes small sines read low, so a high frequency (small amplitude) says less than a low one."""
    from scipy.optimize import least_squares
    out = dict(hz=[], gain=[], delay_ms=[], fit={})
    print("armid_chirp: the joint against what was sent to the motor; in brackets the server's filter, des against the command")
    fig, ax = plt.subplots(2, 5, figsize=(14, 5.6), sharex=True)
    for j in range(5):
        H, G, P, A, SRV = [], [], [], [], []
        for p in sorted({x for x in D["ph"] if x.startswith("c%d_h" % j)}, key=lambda x: float(x.split("_h")[1])):
            hz = float(p.split("_h")[1])
            k = D["ph"] == p
            t = D["t"][k]
            H.append(hz)
            r = phasor(t, D["q"][k, j], hz) / phasor(t, D["sent"][k, j], hz)
            f = phasor(t, D["qd"][k, j], hz) / phasor(t, D["qc"][k, j], hz)
            G.append(abs(r))
            P.append(np.angle(r))
            A.append(1e3 * abs(phasor(t, D["sent"][k, j], hz)))
            SRV.append((abs(f), -1e3 * np.angle(f) / (2 * np.pi * hz)))
        H, G, P = np.array(H), np.array(G), np.unwrap(np.array(P))
        if len(H) == 0:
            continue
        w = 2 * np.pi * H

        def model(x):
            T, wn, z = x
            return np.exp(-1j * w * T) * wn ** 2 / (wn ** 2 - w ** 2 + 2j * z * wn * w)
        if len(H) >= 3:
            sol = least_squares(lambda x: np.r_[np.real(model(x)) - G * np.cos(P), np.imag(model(x)) - G * np.sin(P)], [0.05, 20.0, 1.0], bounds=([0, 3, 0.1], [0.5, 200, 5]))
            T, wn, z = sol.x
            print("  %-11s %s Hz, amplitude %s mrad: gain %s, delay %s ms -> fit: dead time %.0f ms, natural %.1f Hz, damping ratio %.2f, residual %.3f (server: gain %s, delay %s ms)" % (
                NAMES[j], H, np.round(A, 0), np.round(G, 2), np.round(-1e3 * P / w, 0), 1e3 * T, wn / (2 * np.pi), z, float(np.sqrt(np.mean(sol.fun ** 2))),
                np.round([x[0] for x in SRV], 2), np.round([x[1] for x in SRV], 0)))
            out["fit"][NAMES[j]] = dict(dead_ms=1e3 * T, natural_hz=wn / (2 * np.pi), damping_ratio=z)
            ax[0, j].plot(H, np.abs(model(sol.x)), color="#444", lw=1)
            ax[1, j].plot(H, 1e3 * -np.unwrap(np.angle(model(sol.x))) / w, color="#444", lw=1)
        else:
            print("  %-11s %s Hz: gain %s, delay %s ms" % (NAMES[j], H, np.round(G, 2), np.round(-1e3 * P / w, 0)))
        out["hz"] = H.tolist()
        out["gain"].append(G.tolist())
        out["delay_ms"].append((-1e3 * P / w).tolist())
        ax[0, j].plot(H, G, "o", color=COL[j], ms=4)
        ax[1, j].plot(H, -1e3 * P / w, "o", color=COL[j], ms=4)
        ax[0, j].set_title(NAMES[j], fontsize=9)
        ax[1, j].set_xlabel("Hz", fontsize=8)
        for a in ax[:, j]:
            a.grid(alpha=0.25)
    ax[0, 0].set_ylabel("gain")
    ax[1, 0].set_ylabel("delay, ms")
    fig.tight_layout()
    fig.savefig(os.path.join(d, "armid_chirp.png"), dpi=100)
    print(os.path.join(d, "armid_chirp.png"))
    save_part(d, "chirp", out)


def fit_hand(D, d):
    print("armid_hand: finger against its command (ramps: fit, gap between going up and going down; steps: time to 90%)")
    labels = ["thumb bend", "thumb rot", "index", "middle", "ring", "little"]
    out = dict(channels=[])
    fig, ax = plt.subplots(1, 6, figsize=(15, 2.9))
    for c in range(6):
        k0, k1 = D["ph"] == "h%d_ramp0" % c, D["ph"] == "h%d_ramp1" % c
        if not k0.any():
            continue
        x, y = np.r_[D["hc"][k0, c], D["hc"][k1, c]], np.r_[D["hand"][k0, c], D["hand"][k1, c]]
        b = np.polyfit(x, y, 1)
        grid = np.linspace(x.min() + 0.05, x.max() - 0.05, 9)
        o0, o1 = np.argsort(D["hc"][k0, c]), np.argsort(D["hc"][k1, c])
        gap = np.interp(grid, D["hc"][k0, c][o0], D["hand"][k0, c][o0]) - np.interp(grid, D["hc"][k1, c][o1], D["hand"][k1, c][o1])
        t90 = []
        for n in range(4):
            k = np.where(D["ph"] == "h%d_step%d" % (c, n))[0]
            if len(k) < 3:
                continue
            a0, a1 = D["hand"][k[0], c], np.median(D["hand"][k[-5:], c])
            if abs(a1 - a0) < 0.05:
                continue
            done = np.where(np.abs(D["hand"][k, c] - a0) >= 0.9 * abs(a1 - a0))[0]
            if len(done):
                t90.append(float(D["t"][k[done[0]]] - D["t"][k[0]]))
        out["channels"].append(dict(name=labels[c], scale=float(b[0]), offset=float(b[1]), hysteresis=float(np.max(np.abs(gap))), t90_s=float(np.median(t90)) if t90 else None))
        print("  %-11s measured = %.3f x command %+.3f; rms %.3f; up - down %.3f; step 90%% in %s s" % (
            labels[c], b[0], b[1], float(np.std(y - np.polyval(b, x))), float(np.max(np.abs(gap))), np.round(t90, 2)))
        ax[c].plot(D["hc"][k0, c], D["hand"][k0, c], color=COL[c % 5], lw=1)
        ax[c].plot(D["hc"][k1, c], D["hand"][k1, c], color=COL[c % 5], lw=1, ls="--")
        ax[c].set_title(labels[c], fontsize=9)
        ax[c].set_xlabel("command", fontsize=8)
        ax[c].grid(alpha=0.25)
    ax[0].set_ylabel("measured")
    pm = [p[0] == "h" and p[1:2].isdigit() for p in D["ph"]]
    drift = D["pads"][pm]
    if len(drift):
        print("  pads, free air (N at 0.044 per count; only the arm and the finger moved): median %s, range %s" % (np.round(np.nanmedian(drift, axis=0), 2), np.round(np.nanmax(drift, axis=0) - np.nanmin(drift, axis=0), 2)))
        out["pad_range_free_air_N"] = (np.nanmax(drift, axis=0) - np.nanmin(drift, axis=0)).tolist()
    fig.tight_layout()
    fig.savefig(os.path.join(d, "armid_hand.png"), dpi=100)
    print(os.path.join(d, "armid_hand.png"))
    save_part(d, "hand", out)


def last_dir(v, eps=0.005):
    """The sign of the last real motion, held while the joint stands still."""
    out = np.zeros(len(v))
    cur = 0.0
    for i, x in enumerate(v):
        if abs(x) > eps:
            cur = np.sign(x)
        out[i] = cur
    return out


def features(D, j, step):
    v = np.gradient(D["qc"][:, j], D["t"])
    q = D["q"]
    X = [np.ones(len(v))]
    if step >= 1:
        X.append(v)
    if step >= 2:
        X.append(last_dir(v))
    if step >= 3:
        for a in (q[:, 0], q[:, 1], q[:, 3], q[:, 0] + q[:, 3], q[:, 2]):
            X += [np.sin(a), np.cos(a)]
    return np.array(X).T


def pad_mm(D, Q):
    """The index pad, pelvis frame, mm, at joint angles Q (the logged hand)."""
    from pk import config
    from pk.model import Robot
    from pk.stages.push10 import PAD_POINT, IDX
    rob = Robot()
    out = np.zeros((len(Q), 3))
    hands = np.zeros(12)
    hands[:6] = D["hand"][0]
    q11 = np.zeros(config.N_ARM)
    for i in range(len(Q)):
        q11[config.LEFT] = Q[i]
        T = rob.fk(q11, hands)[IDX]
        out[i] = 1000 * (T[:3, :3] @ PAD_POINT + T[:3, 3])
    return out


def hand_table(D, e_fit):
    """How far the pad is off: after each correction, mm rms (x, y, z) and the 95th percentile of the distance."""
    k = np.arange(0, len(D["t"]), 4)
    Dk = dict(D, q=D["q"][k])
    base = pad_mm(Dk, D["q"][k])
    fast = np.array([p.endswith("fast") for p in D["ph"]])[k]
    print("the index pad: where the measured arm is, against where the command + each model says, mm")
    print("  %-12s %-28s %-28s" % ("model", "rms x y z (fitted part)", "rms x y z (fast corners, unseen)"))
    for name, e in e_fit:
        P = pad_mm(Dk, D["q"][k] + e[k])
        r = P - base
        print("  %-12s %s  dist95 %4.1f     %s  dist95 %4.1f" % (name, np.round(np.sqrt((r[~fast] ** 2).mean(0)), 1), np.percentile(np.linalg.norm(r[~fast], axis=1), 95),
                                                                np.round(np.sqrt((r[fast] ** 2).mean(0)), 1), np.percentile(np.linalg.norm(r[fast], axis=1), 95)))


def fit_air(D, d):
    e = D["qc"] - D["q"]
    fast = np.array([p.endswith("fast") for p in D["ph"]])
    train = ~fast
    labels = ["constant", "+ delay", "+ direction", "+ pose"]
    print("tracking error, mrad rms, fitted without the fast corners; in brackets: on the fast corners, unseen")
    print("  %-11s %s" % ("joint", "  ".join("%-16s" % s for s in labels)))
    table = np.zeros((5, 4, 2))
    resid = [e.copy()] + [e.copy() for _ in labels]
    for j in range(5):
        cells = []
        for s in range(4):
            X = features(D, j, s)
            b = np.linalg.lstsq(X[train], e[train, j], rcond=None)[0]
            r = e[:, j] - X @ b
            resid[s + 1][:, j] = r
            table[j, s] = 1e3 * np.sqrt((r[train] ** 2).mean()), 1e3 * np.sqrt((r[fast] ** 2).mean()) if fast.any() else np.nan
            cells.append("%5.1f (%5.1f)    " % tuple(table[j, s]))
        print("  %-11s %s" % (NAMES[j], "  ".join(cells)))
    hand_table(D, list(zip(["no model"] + labels, resid)))
    # the joint moves: friction's width and the delay
    print("joint moves: gap between the holds reached from below and from above (backlash + friction / kp); delay")
    width, delay = [], []
    for j in range(5):
        w = []
        for tag in sorted({p.split("_hold")[0] for p in D["ph"] if p.startswith("j%d_v" % j)}):
            up = D["q"][D["ph"] == tag + "_hold_up"]
            dn = D["q"][D["ph"] == tag + "_hold_down"]
            bk = D["q"][D["ph"] == tag + "_hold_back"]
            if len(bk) and len(up):
                n = max(1, len(bk) // 3)
                hold_err = (D["qc"][D["ph"] == tag + "_hold_back"][-n:, j] - bk[-n:, j]).mean()
                w.append(hold_err)
            if len(up) and len(dn):
                e_up = (D["qc"][D["ph"] == tag + "_hold_up"][:, j] - up[:, j]).mean()
                e_dn = (D["qc"][D["ph"] == tag + "_hold_down"][:, j] - dn[:, j]).mean()
                w.append(e_up - e_dn)
        moving = np.array([p.startswith("j%d_v0.2" % j) and "hold" not in p for p in D["ph"]])
        v = np.gradient(D["qc"][:, j], D["t"])
        dl = float(np.median(np.abs(e[moving, j] / np.where(np.abs(v[moving]) > 1e-3, v[moving], np.nan)))) if moving.any() else np.nan
        width.append(np.round(1e3 * np.array(w), 1).tolist())
        delay.append(dl)
        print("  %-11s up - down error %s mrad; lag / speed %.0f ms" % (NAMES[j], width[-1], 1e3 * dl))
    fig, ax = plt.subplots(5, 1, figsize=(13, 11), sharex=True)
    for j in range(5):
        ax[j].plot(D["t"], 1e3 * e[:, j], color=COL[j], lw=0.9)
        ax[j].set_ylabel(NAMES[j] + "\ncmd - meas, mrad", fontsize=8)
        ax[j].grid(alpha=0.25)
        s = 0
        for i in range(1, len(D["ph"]) + 1):
            if i == len(D["ph"]) or D["ph"][i].split("_")[-1] != D["ph"][s].split("_")[-1]:
                if D["ph"][s].endswith("fast"):
                    ax[j].axvspan(D["t"][s], D["t"][min(i, len(D["t"]) - 1)], color="#cccccc", alpha=0.25, lw=0)
                s = i
    ax[-1].set_xlabel("s (grey: fast corners, not fitted)")
    fig.suptitle("armid: R1 left arm tracking error, free air: %s" % os.path.basename(os.path.normpath(d)), fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(os.path.join(d, "armid_fit.png"), dpi=90)
    print(os.path.join(d, "armid_fit.png"))


def fit_load(D, d):
    labs = [p for p in dict.fromkeys(D["ph"]) if p.startswith("load_") and p != "load_none"]
    print("load    pad z sag mm   joint-lag Fz N (hand pad point)   joint error vs no load, mrad")
    ref = None
    W, S, F = [], [], []
    for p in labs:
        k = np.where(D["ph"] == p)[0]
        k = k[len(k) // 2:]                                  # the second half, after the swing
        z, fz, qe = 1e3 * np.median(D["idx"][k, 2]), np.median(D["F"][k, 2]), np.median(D["q"][k], axis=0)
        if ref is None:
            ref = (z, fz, qe)
        print("%-7s %8.1f %16.1f                   %s" % (p[5:], ref[0] - z, fz - ref[1], np.round(1e3 * (qe - ref[2]), 1)))
        try:
            W.append(float(p[5:]))
            S.append(ref[0] - z)
            F.append(fz - ref[1])
        except ValueError:
            pass
    if len(W) >= 2 and np.ptp(S) > 0.5:
        k_ = np.polyfit(S, W, 1)[0] * 1000
        g_ = np.polyfit(W, F, 1)[0]
        print("arm stiffness at this pose, down: %.0f N/m; force estimate / true load: %.2f" % (k_, g_))
        fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
        ax[0].plot(W, S, "o-", color=COL[0])
        ax[0].set_xlabel("hung load, N")
        ax[0].set_ylabel("sag at the pad, mm")
        ax[1].plot(W, F, "o-", color=COL[0])
        ax[1].plot(W, -np.array(W), "--", color="#888", label="exact (force on the hand is down)")
        ax[1].set_xlabel("hung load, N")
        ax[1].set_ylabel("joint-lag Fz change, N")
        ax[1].legend(fontsize=8)
        for a in ax:
            a.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(os.path.join(d, "armid_load.png"), dpi=100)
        print(os.path.join(d, "armid_load.png"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir")
    a = ap.parse_args()
    s = os.path.join(a.dir, "summary.json")
    if os.path.exists(s):
        print("run: %s" % json.load(open(s)).get("ended"))
    D = load(a.dir)
    ph = {p.split("_")[0] for p in D["ph"]}
    has = lambda c: any(p[0] == c and p[1:].isdigit() for p in ph)     # noqa: E731
    if any(p.startswith("load_") for p in D["ph"]):
        fit_load(D, a.dir)
    elif has("g"):
        fit_grav(D, a.dir)
    elif has("f"):
        fit_fric(D, a.dir)
    elif has("c"):
        fit_chirp(D, a.dir)
    elif has("h"):
        fit_hand(D, a.dir)
    else:
        fit_air(D, a.dir)


if __name__ == "__main__":
    main()
