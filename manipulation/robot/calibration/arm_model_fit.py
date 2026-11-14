"""Fit one model of an R1 arm (gravity, friction, inertia, delay) from all its free-air tests, for the simulation.

    python calibration/arm_model_fit.py <log dir> [...] --check <log dir> [...] --out../models/sim_arm_model_left.json
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
sys.path.insert(0, PICK)
from pk import config  # noqa: E402
from pk.model import G, Robot  # noqa: E402

NAMES = ["sh pitch", "sh roll", "sh yaw", "elbow", "wrist"]
TOL = 0.03                                     # a richer model must beat the simpler one by this much on held-out data


# ====================================================================== data
def load(d):
    meta = json.load(open(os.path.join(d, "armwide_meta.json")))
    R = list(csv.DictReader(open(os.path.join(d, "armwide_cmd.csv"))))
    D = dict(dir=d, side=meta["side"], mode=meta["mode"], load_kg=float(meta.get("load_kg", 0.0)), load_link=meta.get("load_link", "%s_wrist_roll_link" % meta["side"]),
             ph=np.array([r["phase"] for r in R]), t=np.array([float(r["t"]) for r in R]))
    for k, name in (("q", "q_meas"), ("tau", "tau"), ("dq", "dq"), ("sent", "sent"), ("cmd", "q_cmd")):
        D[k] = np.array([[float(r["%s%d" % (name, i)]) for i in range(5)] for r in R])
    D["hand"] = np.array([[float(r["hand%d" % i]) for i in range(6)] for r in R])
    D["kp"] = np.array(meta.get("kp", config.KP[:5]), float)
    D["kd"] = np.array(meta.get("kd", config.KD[:5]), float)
    dq = D["dq"].copy()
    k = 5
    sm = np.array([np.convolve(dq[:, j], np.ones(k) / k, mode="same") for j in range(5)]).T
    h = np.gradient(D["t"])                                    # rows can share a time stamp (two states in one client tick)
    h = np.where(h > 1e-3, h, np.median(h[h > 1e-3]) if (h > 1e-3).any() else 1.0 / 30)
    D["ddq"] = np.gradient(sm, axis=0) / h[:, None]
    return D


def segments(mask):
    """[(start, stop)] of the runs of True in mask."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


# ====================================================================== gravity regressor
class Reg(object):
    """Torque at the 5 joints = Y(q) @ pi, pi = per arm segment [mass, mass x centre of mass (3, segment frame)]: linear in pi. The segments are
    the links each arm joint moves (the hand and fingers in the last one, at the logged hand shape)."""

    def __init__(self, side, hand6):
        self.rob, self.side = Robot(), side
        self.sl = config.LEFT if side == "left" else config.RIGHT
        self.hs = np.zeros(12)
        self.hs[(0 if side == "left" else 6):(6 if side == "left" else 12)] = hand6
        self.a = np.zeros(config.N_ARM)
        links, m, com, masks = self.rob._mass_table(side)
        grp = masks.sum(0).astype(int) - 1
        self.ref = [self.rob.joints[jn]["child"] for jn in self.rob.arm_joints[side]]
        T = self.fk(np.zeros(5))
        self.pi0 = np.zeros(20)
        for l, ml, cl, g in zip(links, m, com, grp):
            Rr, tr = T[self.ref[g]][:3, :3], T[self.ref[g]][:3, 3]
            c = Rr.T @ (T[l][:3, :3] @ cl + T[l][:3, 3] - tr)
            self.pi0[4 * g] += ml
            self.pi0[4 * g + 1:4 * g + 4] += ml * c
        self.sd = np.zeros(20)                                 # the prior's width: 30 % of the URDF value, at least 50 g / 10 g m
        for g in range(5):
            self.sd[4 * g] = 0.3 * self.pi0[4 * g] + 0.05
            self.sd[4 * g + 1:4 * g + 4] = 0.3 * np.abs(self.pi0[4 * g + 1:4 * g + 4]) + 0.01

    def fk(self, q5):
        self.a[self.sl] = q5
        return self.rob.fk(self.a, self.hs)

    def Y(self, q5, load_link=None):
        T = self.fk(q5)
        Y = np.zeros((5, 20))
        L = np.zeros(5)
        for i, (o, ax) in enumerate(self.rob.joint_frames(T, self.side)):
            for g in range(i, 5):
                R, p = T[self.ref[g]][:3, :3], T[self.ref[g]][:3, 3]
                Y[i, 4 * g] = -ax @ np.cross(p - o, G)
                for k in range(3):
                    Y[i, 4 * g + 1 + k] = -ax @ np.cross(R[:, k], G)
            if load_link:
                L[i] = -ax @ np.cross(T[load_link][:3, 3] - o, G)
        return Y, L

    def self_check(self):
        rng = np.random.RandomState(0)
        e = 0.0
        for _ in range(5):
            q = rng.uniform(-1, 1, 5)
            Y, _ = self.Y(q)
            e = max(e, float(np.max(np.abs(Y @ self.pi0 - self.rob.gravity(self.fk(q), self.side)))))
        return e


# ====================================================================== gravity
def holds(runs):
    """Every pose held from both sides: the position halfway and the torque halfway (friction cancels), per run."""
    out = []
    for D in runs:
        if D["mode"] != "poses":
            continue
        tags = sorted({p.split("_")[0] for p in D["ph"] if p[0] == "g" and p[1:].split("_")[0].isdigit()}, key=lambda x: int(x[1:]))
        for tg in tags:
            ka, kb = np.where(D["ph"] == tg + "_a")[0], np.where(D["ph"] == tg + "_b")[0]
            if len(ka) > 4 and len(kb) > 4:
                ka, kb = ka[len(ka) // 2:], kb[len(kb) // 2:]
                Ta, Tb = np.median(D["tau"][ka], axis=0), np.median(D["tau"][kb], axis=0)
                out.append(dict(q=0.5 * (np.median(D["q"][ka], axis=0) + np.median(D["q"][kb], axis=0)), y=0.5 * (Ta + Tb), fr=0.5 * np.abs(Ta - Tb),
                                load=D["load_kg"], link=D["load_link"], src="hold", group="%s:%s" % (D["dir"], tg),
                                err=0.5 * (np.median(D["sent"][ka] - D["q"][ka], axis=0) + np.median(D["sent"][kb] - D["q"][kb], axis=0))))
    return out


def sweep_means(runs):
    """armwide_joint: the moving joint's torque at the same angle going up and coming down, averaged: gravity there (friction cancels)."""
    out = []
    for D in runs:
        if D["mode"] != "joint":
            continue
        for j in range(5):
            for v in sorted({p.split("_")[1] for p in D["ph"] if p.startswith("j%d_v" % j)}):
                up, dn = D["ph"] == "j%d_%s_up" % (j, v), D["ph"] == "j%d_%s_dn" % (j, v)
                if not up.any() or not dn.any():
                    continue
                bins = np.arange(D["q"][up | dn, j].min(), D["q"][up | dn, j].max(), 0.04)
                for b0 in bins[1:-1]:
                    ku = np.where(up & (np.abs(D["q"][:, j] - b0) < 0.02) & (np.abs(D["dq"][:, j]) > 0.003))[0]
                    kd = np.where(dn & (np.abs(D["q"][:, j] - b0) < 0.02) & (np.abs(D["dq"][:, j]) > 0.003))[0]
                    if len(ku) > 1 and len(kd) > 1:
                        q = np.median(D["q"][np.r_[ku, kd]], axis=0)
                        y = np.full(5, np.nan)
                        y[j] = 0.5 * (np.median(D["tau"][ku, j]) + np.median(D["tau"][kd, j]))
                        out.append(dict(q=q, y=y, load=0.0, link=None, src="sweep", group=None))
    return out


def design(reg, rows, model):
    """Rows of the linear problem y = A x for one gravity model. x: [pi (20), bias (5), spring (5), k] as the model uses them."""
    A, y, w_src, grp = [], [], [], []
    for r in rows:
        Y, L = reg.Y(r["q"], r["link"])
        for j in range(5):
            if np.isnan(r["y"][j]):
                continue
            a = np.zeros(31)
            if model == "urdf":
                a[30] = r["load"] * L[j]
            else:
                a[:20] = Y[j]
                a[20 + j] = 1.0
                if model == "spring":
                    a[25 + j] = r["q"][j]
                a[30] = r["load"] * L[j]
            A.append(a)
            y.append(r["y"][j] - (Y[j] @ reg.pi0 if model == "urdf" else 0.0))
            w_src.append(r["src"])
            grp.append((r["group"], j))
    return np.array(A), np.array(y), np.array(w_src), grp


def solve(reg, A, y, w, model, has_load):
    """Weighted least squares with a prior: pi near the URDF (30 %), bias and spring near 0 (wide), the torque scale near 1."""
    if model == "urdf":
        x = np.zeros(31)
        x[30] = 1.0
        return x
    P0 = np.r_[reg.pi0, np.zeros(10), 1.0]
    sd = np.r_[reg.sd, 3.0 * np.ones(5), 2.0 * np.ones(5), 0.3 if has_load else 1e-6]
    if model != "spring":
        sd[25:30] = 1e-6
    sw = np.sqrt(w)
    Aa = np.vstack([A * sw[:, None], np.diag(1.0 / sd)])
    ya = np.r_[y * sw, P0 / sd]
    return np.linalg.lstsq(Aa, ya, rcond=None)[0]


def scale_fit(reg, rows, test=None):
    """The server's model: per joint, y = s (URDF + load) + b."""
    X = {j: [] for j in range(5)}
    for r in rows:
        Y, L = reg.Y(r["q"], r["link"])
        for j in range(5):
            if not np.isnan(r["y"][j]):
                X[j].append((Y[j] @ reg.pi0 + r["load"] * L[j], r["y"][j]))
    coef = []
    for j in range(5):
        M = np.array(X[j])
        coef.append(np.polyfit(M[:, 0], M[:, 1], 1) if len(M) > 2 else np.array([1.0, 0.0]))
    return coef


def predict(reg, x, model, r, coef=None):
    Y, L = reg.Y(r["q"], r["link"])
    if model == "scale":
        return np.array([np.polyval(coef[j], Y[j] @ reg.pi0 + r["load"] * L[j]) for j in range(5)])
    if model == "urdf":
        return Y @ reg.pi0 + r["load"] * L
    return Y @ x[:20] + x[20:25] + x[25:30] * r["q"] + x[30] * r["load"] * L


def fit_gravity(reg, H, S):
    has_load = any(r["load"] > 0 for r in H)
    models = ["urdf", "scale", "links", "spring"]
    loo = {}
    w_hold, w_sweep = 1.0, 1.0
    for it in range(2):                                         # weights: 1 / (each source's residual)^2, from the fit before
        A, y, src, _ = design(reg, H + S, "links")
        w = np.where(src == "hold", w_hold, w_sweep)
        x = solve(reg, A, y, w, "links", has_load)
        res = y - A @ x
        w_hold = 1.0 / max(np.mean(res[src == "hold"] ** 2), 1e-4)
        if (src == "sweep").any():
            w_sweep = 1.0 / max(np.mean(res[src == "sweep"] ** 2), 1e-4)
    groups = sorted({r["group"] for r in H})
    for model in models:
        err = []
        for g in groups:
            tr = [r for r in H if r["group"] != g] + S
            te = [r for r in H if r["group"] == g]
            if model == "scale":
                coef = scale_fit(reg, tr)
                p = [predict(reg, None, model, r, coef) for r in te]
            else:
                A, y, src, _ = design(reg, tr, model)
                x = solve(reg, A, y, np.where(src == "hold", w_hold, w_sweep), model, has_load)
                p = [predict(reg, x, model, r) for r in te]
            err += [r["y"] - pp for r, pp in zip(te, p)]
        loo[model] = np.sqrt(np.mean(np.square(err), axis=0))
    chosen = "urdf"
    for model in models[1:]:
        if np.mean(loo[model]) < (1 - TOL) * np.mean(loo[chosen]):
            chosen = model
    A, y, src, _ = design(reg, H + S, chosen if chosen != "scale" else "links")
    x = solve(reg, A, y, np.where(src == "hold", w_hold, w_sweep), chosen if chosen != "scale" else "links", has_load)
    coef = scale_fit(reg, H + S)
    # how well each number is pinned: refit on bootstrap resamples of the poses
    rng = np.random.RandomState(0)
    boots = []
    for _ in range(30):
        pick = set(rng.choice(groups, len(groups)))
        tr = [r for r in H if r["group"] in pick] + S
        A2, y2, src2, _ = design(reg, tr, "links" if chosen in ("urdf", "scale") else chosen)
        boots.append(solve(reg, A2, y2, np.where(src2 == "hold", w_hold, w_sweep), "links" if chosen in ("urdf", "scale") else chosen, has_load))
    boots = np.array(boots)
    k = float(x[30]) if has_load else 1.0
    out = dict(chosen=chosen, loo_rms_nm={m: [float(v) for v in loo[m]] for m in models}, poses=len(groups), sweep_rows=len(S),
               weight_rms_nm=dict(hold=float(w_hold ** -0.5), sweep=float(w_sweep ** -0.5)),
               pi_reading=[float(v) for v in x[:20]], pi_true=[float(v) for v in x[:20] / k], pi_urdf=[float(v) for v in reg.pi0],
               pi_sd=[float(v) for v in np.std(boots[:, :20], axis=0)],
               bias_nm=[float(v) for v in x[20:25]], spring_nm_per_rad=[float(v) for v in x[25:30]],
               bias_sd=[float(v) for v in np.std(boots[:, 20:25], axis=0)],
               scale=[float(c[0]) for c in coef], scale_bias_nm=[float(c[1]) for c in coef],
               torque_scale=k, torque_scale_sd=float(np.std(boots[:, 30])) if has_load else None, has_load=has_load,
               segments=reg.ref, units="pi: per segment [kg, kg m x 3 in the segment's frame]; torque_scale = reading / true N m (1 without a hung load)")
    return out, x, coef


def gravity_now(reg, gf, x, coef, q, load=0.0, link=None):
    r = dict(q=q, load=load, link=link)
    return predict(reg, x, gf["chosen"], r, coef)


# ====================================================================== dynamics (URDF inertia) for the motion rows
class Dyn(object):
    """M(q) ddq + C(q, dq) dq of the arm from the URDF in MuJoCo, gravity off, the other arm at the pre-grip pose. None without MuJoCo."""

    def __init__(self, side):
        try:
            sys.path.insert(0, HERE)
            import mujoco
            import armwide_plan as AP
        except ImportError:
            self.ok = False
            return
        home = np.array(json.load(open(os.path.join(PICK, "params", "premotion.json")))["q"], float)
        sc = AP.Scene(side, home)
        self.mj, self.m, self.d, self.adr = mujoco, sc.m, sc.d, sc.adr
        self.dof = [sc.m.jnt_dofadr[sc.m.joint(j).id] for j in sc.meta["arm"][side]]
        self.m.opt.gravity[:] = 0.0
        self.m.dof_armature[self.dof] = 0.0
        self.res = np.zeros(self.m.nv)
        self.ok = True

    def __call__(self, q, dq, ddq):
        d = self.d
        d.qpos[self.adr], d.qvel[:] = q, 0.0
        d.qvel[self.dof] = dq
        self.mj.mj_forward(self.m, d)                           # positions and velocities; it also overwrites qacc, so that is set after
        d.qacc[:] = 0.0
        d.qacc[self.dof] = ddq
        self.mj.mj_rne(self.m, d, 1, self.res)
        return self.res[self.dof].copy()


# ====================================================================== friction
def fric_rows(runs, reg, gf, x, coef, dyn, k):
    """Per joint: speed, angle, acceleration, torque left after gravity (and after the URDF inertia in motions), and the source."""
    rows = {j: [] for j in range(5)}
    for D in runs:
        for n in range(len(D["t"])):
            p = D["ph"][n]
            joint = p[0] == "j" and p[1].isdigit() and ("_v" in p) and (p.endswith("_up") or p.endswith("_dn"))
            motion = p[0] == "m" and p[1:].split("_")[0].isdigit()
            if not (joint or motion):
                continue
            if motion and n % 2:
                continue
            g = gravity_now(reg, gf, x, coef, D["q"][n])
            inert = dyn(D["q"][n], D["dq"][n], D["ddq"][n]) * k if (motion and dyn.ok) else np.zeros(5)
            js = [int(p[1])] if joint else range(5)
            for j in js:
                v = D["dq"][n, j]
                if joint:
                    vnom = float(p.split("_")[1][1:])
                    if abs(v) < 0.3 * vnom:
                        continue
                elif abs(v) < 0.01:
                    continue
                rows[j].append((v, D["q"][n, j], D["ddq"][n, j] if motion else 0.0, D["tau"][n, j] - g[j] - inert[j], "motion" if motion else "sweep"))
    return rows


def fric_cols(v, q, a, model, vs):
    s = np.sign(v)
    if model == "S":                                            # Coulomb + extra near standstill that fades by ~vs + viscous; all >= 0
        return np.array([s, s * np.exp(-(np.abs(v) / vs) ** 2), v, a]).T
    cols = [s if model == "F1" else 1.0 * (v > 0), v]
    if model != "F1":
        cols.append(-1.0 * (v < 0))
    if model in ("F3", "F4"):
        cols.append(s * np.exp(-(np.abs(v) / vs) ** 2))
    if model == "F4":
        cols += [s * np.cos(q), s * np.sin(q)]
    cols.append(a)                                              # armature
    return np.array(cols).T


def fric_fit(rows, model, vs, w_src):
    v, q, a, y, src = rows
    A = fric_cols(v, q, a, model, vs)
    w = np.array([w_src[s] for s in src])
    has_a = np.any(a != 0)
    if not has_a:
        A = A[:, :-1]
    c = np.zeros(A.shape[1])
    for _ in range(4):                                          # Huber: rows more than 2 sigma off count less
        sw = np.sqrt(w)
        c = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)[0]
        r = y - A @ c
        sig = 1.4826 * np.median(np.abs(r)) + 1e-6
        hub = np.minimum(1.0, 2 * sig / np.maximum(np.abs(r), 1e-9))
        w = np.array([w_src[s] for s in src]) * hub
    if not has_a:
        c = np.r_[c, 0.0]
    return c


def fric_pred(c, v, q, a, model, vs):
    return fric_cols(v, q, a, model, vs) @ c


STEADY_VS = (0.02, 0.03, 0.04, 0.06, 0.08, 0.1, 0.12, 0.15)   # rad/s: how fast the extra friction near standstill fades
LUGRE_S0 = tuple(np.round(np.logspace(1.0, 3.5, 21), 1))         # N m/rad: how stiffly friction builds after a reversal


def steady_points(runs):
    """Per joint: friction at each sweep speed WITHOUT any gravity model. In the steady part of a sweep (measured speed within 25 % of the
    planned), at the same angle, (torque going up - torque going down) / 2 = friction; gravity is the same both ways and cancels. Angle bins
    of 1/8 of the shared range; [(speed, friction, rows)]. the all-rows fit gave a negative viscous term from transients
    and the gravity model's error; these points show a flat curve above 0.15 rad/s."""
    pts = {j: [] for j in range(5)}
    for D in runs:
        if D["mode"] != "joint":
            continue
        phs = set(D["ph"].tolist())
        for j in range(5):
            speeds = sorted({float(p.split("_v")[1].split("_")[0]) for p in phs if p.startswith("j%d_v" % j)})
            for v in speeds:
                su = (D["ph"] == "j%d_v%g_up" % (j, v)) & (np.abs(D["dq"][:, j] - v) < 0.25 * v + 0.003)
                sd = (D["ph"] == "j%d_v%g_dn" % (j, v)) & (np.abs(D["dq"][:, j] + v) < 0.25 * v + 0.003)
                qu, tu, qd, td = D["q"][su, j], D["tau"][su, j], D["q"][sd, j], D["tau"][sd, j]
                if len(qu) < 6 or len(qd) < 6:
                    continue
                lo, hi = max(qu.min(), qd.min()), min(qu.max(), qd.max())
                if hi - lo < 0.01:
                    continue
                edges = np.linspace(lo, hi, 9)
                for a_, b_ in zip(edges[:-1], edges[1:]):
                    mu, md = (qu >= a_) & (qu < b_), (qd >= a_) & (qd < b_)
                    if mu.sum() >= 2 and md.sum() >= 2:
                        pts[j].append((v, 0.5 * (np.median(tu[mu]) - np.median(td[md])), int(mu.sum() + md.sum())))
    return pts


def fit_steady(pts):
    """F(v) = Fc + extra * exp(-(v/vs)^2), both >= 0 (friction only resists), vs from STEADY_VS; on the steady points. Returned as
    [Fc, extra, viscous = 0]."""
    from scipy.optimize import nnls
    if len(pts) < 6:
        return None
    v, f = np.array([p[0] for p in pts]), np.array([p[1] for p in pts])
    best = None
    for vs in STEADY_VS:
        A = np.array([np.ones_like(v), np.exp(-(v / vs) ** 2)]).T
        c, _ = nnls(A, f)
        c = np.r_[c, 0.0]
        A = np.c_[A, v]
        e = float(np.sqrt(np.mean((f - A @ c) ** 2)))
        if best is None or e < best[2] - 1e-9:
            best = (c, vs, e)
    return best


def lugre_run(t, v, s0, g, visc=0.0):
    """Friction along a measured speed trace with a state z (LuGre without the damping of z): dz/dt = v - s0 |v| z / g(v), F = s0 z +
    visc v. After a reversal F builds from -g to +g over about 2 g / s0 of travel; in steady motion F = g(v), the steady-sweep curve.
    Exact update over each step (stiff otherwise at 30 Hz)."""
    z, out = 0.0, np.zeros(len(t))
    for n in range(len(t)):
        if n:
            dt = max(t[n] - t[n - 1], 0.0)
            gv = g(abs(v[n]))
            a_ = s0 * abs(v[n]) / max(gv, 1e-6)
            zs = np.sign(v[n]) * gv / s0
            z = zs + (z - zs) * np.exp(-a_ * dt)
        out[n] = s0 * z + visc * v[n]
    return out


def lugre_traces(runs, reg, gf, x, coef, dyn, k, j):
    """[(t, v, y, mask)]: per run, joint j's whole trace (the state needs every row) with y = torque - gravity - URDF inertia, and the rows
    that score (the joint moving in its own test phases, or every row of a random motion)."""
    out = []
    for D in runs:
        if D["mode"] not in ("joint", "motion"):
            continue
        if "_G" not in D:                                       # once per run, all joints
            D["_G"] = np.array([gravity_now(reg, gf, x, coef, q) for q in D["q"]])
            D["_I"] = (np.array([dyn(D["q"][n], D["dq"][n], D["ddq"][n]) * k for n in range(len(D["t"]))])
                       if D["mode"] == "motion" and dyn.ok else np.zeros_like(D["_G"]))
        G = D["_G"][:, j]
        if D["mode"] == "motion" and dyn.ok:
            I = D["_I"][:, j]
            mask = np.array([p[0] == "m" and p[1:].split("_")[0].isdigit() for p in D["ph"]])
        else:
            I = np.zeros(len(D["t"]))
            mask = np.array([p.startswith("j%d_" % j) for p in D["ph"]])
        mask &= np.abs(D["dq"][:, j]) > 0.002
        out.append((D["t"], D["dq"][:, j], D["tau"][:, j] - G - I, mask))
    return out


def fit_lugre(tr_fit, tr_chk, cs, vs, visc=0.0):
    """s0 from the fit runs (rms over their scoring rows); the held-out rms on the check runs. cs, vs: the steady curve."""
    g = lambda a: cs[0] + cs[1] * np.exp(-(a / vs) ** 2)      # noqa: E731

    def rms(trs, s0):
        e = [y[m] - lugre_run(t, v, s0, g, visc)[m] for t, v, y, m in trs if m.any()]
        e = np.concatenate(e) if e else np.zeros(1)
        return float(np.sqrt(np.mean(e ** 2)))
    best = min(((rms(tr_fit, s0), s0) for s0 in LUGRE_S0))
    return dict(s0=best[1], in_rms=best[0], held_rms=rms(tr_chk, best[1]) if tr_chk else None)


def fric_admissible(c, model):
    """Friction only resists motion: no negative Coulomb either way, no negative viscous, no negative extra near standstill."""
    if model == "S":
        return True
    if model == "F1":
        return c[0] >= 0 and c[1] >= 0
    return c[0] >= 0 and c[1] >= 0 and c[2] >= 0 and (model == "F2" or c[3] >= 0)


def fit_friction(R, R_check, steady=None):
    out = []
    for j in range(5):
        if len(R[j]) < 20:
            out.append(dict(joint=NAMES[j], model=None, rows=len(R[j])))
            continue
        arr = list(map(np.array, zip(*R[j])))
        v, q, a, y, src = arr
        held = None
        if R_check is not None and len(R_check[j]) > 20:
            held = list(map(np.array, zip(*R_check[j])))
        else:                                                   # no check run: the last 30 % of the motion rows, else of all
            n = len(v)
            cut = np.zeros(n, bool)
            mot = np.where(src == "motion")[0]
            idx = mot if len(mot) > 30 else np.arange(n)
            cut[idx[int(0.7 * len(idx)):]] = True
            held = [x[cut] for x in arr]
            arr = [x[~cut] for x in arr]
        w_src = {"sweep": 1.0, "motion": 1.0}
        c = fric_fit(arr, "F1", 0.02, w_src)
        r = arr[3] - fric_pred(c, arr[0], arr[1], arr[2], "F1", 0.02)
        for s in w_src:                                         # each source weighted by its own noise
            if (arr[4] == s).any():
                w_src[s] = 1.0 / max(np.mean(r[arr[4] == s] ** 2), 1e-4)
        ladder = {}
        for model in ("F1", "F2", "F3", "F4"):
            best = None
            for vs in ((0.02,) if model in ("F1", "F2") else (0.005, 0.01, 0.02, 0.04, 0.08)):
                c = fric_fit(arr, model, vs, w_src)
                e = float(np.sqrt(np.mean((held[3] - fric_pred(c, held[0], held[1], held[2], model, vs)) ** 2)))
                ein = float(np.sqrt(np.mean((arr[3] - fric_pred(c, arr[0], arr[1], arr[2], model, vs)) ** 2)))
                if best is None or ein < best[2]:
                    best = (c, vs, ein, e)
            ladder[model] = best
        st = fit_steady(steady[j]) if steady else None
        if st is not None:                                      # the curve from the steady points; armature from the fit set's motion rows
            cs, vs_s, e_st = st
            c4 = np.r_[cs, 0.0]
            if np.any(arr[2] != 0):
                r_ = arr[3] - fric_pred(c4, arr[0], arr[1], arr[2] * 0, "S", vs_s)
                c4[3] = max(0.0, float(np.dot(arr[2], r_) / max(np.dot(arr[2], arr[2]), 1e-9)))
            ein = float(np.sqrt(np.mean((arr[3] - fric_pred(c4, arr[0], arr[1], arr[2], "S", vs_s)) ** 2)))
            e = float(np.sqrt(np.mean((held[3] - fric_pred(c4, held[0], held[1], held[2], "S", vs_s)) ** 2)))
            ladder["S"] = (c4, vs_s, ein, e)
        # S first; a richer model only if it is physical (friction never pushes) and better on the held-out rows by TOL
        chosen = "S" if "S" in ladder else "F1"
        for model in ("F1", "F2", "F3", "F4"):
            if model != chosen and fric_admissible(ladder[model][0], model) and ladder[model][3] < (1 - TOL) * ladder[chosen][3]:
                chosen = model
        full = list(map(np.array, zip(*(R[j] + (R_check[j] if R_check is not None else [])))))
        vs = ladder[chosen][1]
        if chosen == "S":
            c = ladder["S"][0].copy()
            if np.any(full[2] != 0):
                r_ = full[3] - fric_pred(c, full[0], full[1], full[2] * 0, "S", vs)
                c[3] = max(0.0, float(np.dot(full[2], r_) / max(np.dot(full[2], full[2]), 1e-9)))
        else:
            c = fric_fit(full, chosen, vs, w_src)
        rec = dict(joint=NAMES[j], model=chosen, rows=len(R[j]), stribeck_v=vs, heldout_rms_nm={m: ladder[m][3] for m in ladder},
                   in_rms_nm={m: ladder[m][2] for m in ladder}, noise_rms_nm={s: float(w_src[s] ** -0.5) for s in w_src})
        rec["admissible"] = {m: bool(fric_admissible(ladder[m][0], m)) for m in ladder}
        if st is not None:
            rec["steady_points"] = [[float(p[0]), float(p[1])] for p in steady[j]]
            rec["steady_rms_nm"] = e_st
        if chosen == "S":
            rec.update(coulomb_pos_nm=float(c[0]), coulomb_neg_nm=float(c[0]), viscous_nm_s=float(c[2]))
            rest = np.r_[c[1], 0.0, 0.0, c[3]]
        elif chosen == "F1":
            rec.update(coulomb_pos_nm=float(c[0]), coulomb_neg_nm=float(c[0]), viscous_nm_s=float(c[1]))
            rest = c[2:]
        else:
            rec.update(coulomb_pos_nm=float(c[0]), viscous_nm_s=float(c[1]), coulomb_neg_nm=float(c[2]))
            rest = c[3:]
        rec["stribeck_extra_nm"] = float(rest[0]) if chosen in ("F3", "F4", "S") else 0.0
        rec["angle_cos_nm"], rec["angle_sin_nm"] = (float(rest[1]), float(rest[2])) if chosen == "F4" else (0.0, 0.0)
        rec["armature_kgm2"] = float(c[-1])
        rec["coulomb_nm"] = 0.5 * (rec["coulomb_pos_nm"] + rec["coulomb_neg_nm"])
        rec["_curve"] = (np.array(full[0]), np.array(full[3]), c, chosen, vs)
        rec["_ladder_F1"] = ladder["F1"][0]
        out.append(rec)
    return out


# ====================================================================== give before sliding, dead band
def fit_give(runs, reg, gf, x, coef, fr):
    out = {}
    for D in runs:
        if D["mode"] != "joint":
            continue
        for j in range(5):
            if fr[j].get("model") is None:
                continue
            Fc = fr[j]["coulomb_nm"]
            amps = sorted({float(p.split("_r")[1]) for p in D["ph"] if p.startswith("j%d_r" % j)})
            res = {}
            for amp in amps:
                k = np.where(D["ph"] == "j%d_r%g" % (j, amp))[0]
                if len(k) < 20:
                    continue
                qj, y = D["q"][k, j], np.array([D["tau"][n, j] - gravity_now(reg, gf, x, coef, D["q"][n])[j] for n in k])
                best = None
                for s0 in np.logspace(1, 4.5, 36):              # Dahl: friction builds up over the travel since the last reversal
                    F, e = y[0], []
                    for n in range(1, len(k)):
                        dx = qj[n] - qj[n - 1]
                        if dx != 0:
                            sg = np.sign(dx)
                            F = sg * Fc + (F - sg * Fc) * np.exp(-s0 * abs(dx) / max(Fc, 1e-3))
                        e.append(y[n] - F)
                    rms = float(np.sqrt(np.mean(np.square(e))))
                    if best is None or rms < best[1]:
                        best = (float(s0), rms)
                c = D["cmd"][k, j]
                dc = np.sign(np.diff(c))
                rev = np.where(np.diff(dc) != 0)[0] + 1
                lost = []
                for r0 in rev:                                  # command travel after a reversal before the joint moves back 1 mrad
                    sg = dc[r0] if r0 < len(dc) else 0
                    for n in range(r0, len(k)):
                        if sg * (qj[n] - qj[r0]) > 0.001:
                            lost.append(abs(c[n] - c[r0]))
                            break
                coul = float(np.sqrt(np.mean((y - Fc * np.sign(np.gradient(qj))) ** 2)))
                res["%g" % amp] = dict(dahl_nm_per_rad=best[0], dahl_rms_nm=best[1], coulomb_only_rms_nm=coul, lost_motion_rad=float(np.median(lost)) if lost else None,
                                       dahl_kept=bool(best[1] < (1 - TOL) * coul and best[0] < 3e4))
            if res:
                out[NAMES[j]] = res
    return out


# ====================================================================== actuator
def fit_actuator(runs):
    """PD torque asked of the motor, kp (sent - q) - kd dq, with sent taken d seconds earlier, against the torque it reports: d and the gain."""
    best = None
    for dly in np.arange(0.0, 0.155, 0.005):
        num, den, res = np.zeros(5), np.zeros(5), []
        P, Y = [], []
        for D in runs:
            s = np.array([np.interp(D["t"] - dly, D["t"], D["sent"][:, j]) for j in range(5)]).T
            pd = D["kp"] * (s - D["q"]) - D["kd"] * D["dq"]
            P.append(pd)
            Y.append(D["tau"])
        P, Y = np.vstack(P), np.vstack(Y)
        gain = np.sum(P * Y, axis=0) / np.maximum(np.sum(P * P, axis=0), 1e-9)
        rms = np.sqrt(np.mean((Y - P * gain) ** 2, axis=0))
        if best is None or np.mean(rms) < best[2]:
            best = (float(dly), gain, float(np.mean(rms)), rms)
    return dict(delay_ms=1000 * best[0], gain=[float(v) for v in best[1]], rms_nm=[float(v) for v in best[3]],
                note="reported torque = gain x (kp (sent - q) - kd dq) with sent this much earlier; gain 1 and a small delay if the motor does what it is asked")


# ====================================================================== main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--check", nargs="*", default=[], help="runs held out of the fit: scored only")
    ap.add_argument("--out")
    ap.add_argument("--label", default="R1 measurements; not a MuJoCo result", help="where the logs came from, for the figure and the file")
    a = ap.parse_args()
    check = [os.path.normpath(d) for d in a.check]
    runs = [load(d) for d in a.dirs if os.path.normpath(d) not in check]
    runs_c = [load(d) for d in check]
    sides = {D["side"] for D in runs + runs_c}
    if len(sides) != 1:
        raise SystemExit("one arm at a time: %s" % sorted(sides))
    side = sides.pop()
    print("R1 %s arm: fit %s; held out %s" % (side, [os.path.basename(D["dir"]) for D in runs], [os.path.basename(D["dir"]) for D in runs_c]))
    reg = Reg(side, np.nanmedian(np.vstack([D["hand"] for D in runs]), axis=0))
    print("regressor against the URDF gravity: %.1e N m" % reg.self_check())
    H, S = holds(runs), sweep_means(runs)
    if not H:
        raise SystemExit("needs an armwide_poses run")
    gf, x, coef = fit_gravity(reg, H, S)
    print("GRAVITY: %d held poses (%d with a load), %d sweep rows. Held-out rms per joint, N m:" % (gf["poses"], sum(r["load"] > 0 for r in H), gf["sweep_rows"]))
    for m_, v in gf["loo_rms_nm"].items():
        print("  %-7s %s%s" % (m_, np.round(v, 2), "   <- chosen" if m_ == gf["chosen"] else ""))
    if gf["has_load"]:
        print("  torque reading / true torque from the hung load: %.2f +- %.2f" % (gf["torque_scale"], gf["torque_scale_sd"]))
    for g in range(5):
        print("  segment %-24s mass %.2f kg (URDF %.2f, +-%.2f)" % (reg.ref[g], gf["pi_true"][4 * g], reg.pi0[4 * g], gf["pi_sd"][4 * g]))
    dyn = Dyn(side)
    k = gf["torque_scale"]
    R = fric_rows(runs, reg, gf, x, coef, dyn, k)
    Rc = fric_rows(runs_c, reg, gf, x, coef, dyn, k) if runs_c else None
    pts = steady_points(runs)
    fr = fit_friction(R, Rc, pts)
    # friction with memory (LuGre): at low speed after a reversal friction has not built up yet.
    # Every model is scored on the same rows of the check run (joint moving).
    for j, f in enumerate(fr):
        if f.get("model") is None or not pts[j]:
            continue
        st = fit_steady(pts[j])
        if st is None:
            continue
        cs, vs_s, _ = st
        tf = lugre_traces(runs, reg, gf, x, coef, dyn, k, j)
        tc = lugre_traces(runs_c, reg, gf, x, coef, dyn, k, j) if runs_c else []
        lg = fit_lugre(tf, tc, cs, vs_s)
        same = {}
        if tc:
            for m_ in ("S", "F1"):
                if m_ == "S":
                    c_, vv = np.r_[cs, 0.0], vs_s
                    pred = lambda v_: fric_pred(c_, v_, 0 * v_, 0 * v_, "S", vv)     # noqa: E731
                else:
                    c_ = f["_ladder_F1"]
                    pred = lambda v_: fric_pred(c_, v_, 0 * v_, 0 * v_, "F1", 0.02)  # noqa: E731
                e = np.concatenate([y[m] - pred(v)[m] for t, v, y, m in tc if m.any()])
                same[m_] = float(np.sqrt(np.mean(e ** 2)))
            same["L"] = lg["held_rms"]
        f["lugre"] = dict(s0_nm_per_rad=lg["s0"], in_rms_nm=lg["in_rms"], heldout_rms_nm_same_rows=same,
                          build_travel_mrad=1000.0 * 2 * (cs[0] + cs[1]) / lg["s0"])
        ok_static = {m_: v_ for m_, v_ in same.items() if m_ != "L" and (m_ == "S" or f["admissible"].get(m_, False))}
        if tc and ok_static and same["L"] < (1 - TOL) * min(ok_static.values()):
            f.update(model="L", lugre_s0_nm_per_rad=lg["s0"], coulomb_pos_nm=float(cs[0]), coulomb_neg_nm=float(cs[0]), coulomb_nm=float(cs[0]),
                     viscous_nm_s=0.0, stribeck_extra_nm=float(cs[1]), stribeck_v=float(vs_s), angle_cos_nm=0.0, angle_sin_nm=0.0)
            f["_curve"] = (f["_curve"][0], f["_curve"][1], np.r_[cs, 0.0], "S", vs_s)
    print("FRICTION (URDF inertia %s). Per joint: chosen; Coulomb +/-, viscous, extra near standstill, armature; held-out rms N m per model" % ("removed" if dyn.ok else "NOT removed: no MuJoCo"))
    for f in fr:
        if f.get("model") is None:
            print("  %-9s too few moving rows (%d)" % (f["joint"], f["rows"]))
            continue
        print("  %-9s %-2s  %.2f/%.2f  %.2f  %+.2f (fades by %.2f rad/s)  %.3f   %s" % (
            f["joint"], f["model"], f["coulomb_pos_nm"], f["coulomb_neg_nm"], f["viscous_nm_s"], f["stribeck_extra_nm"], f["stribeck_v"], f["armature_kgm2"],
            "  ".join("%s %.3f%s" % (m_, f["heldout_rms_nm"][m_], "" if f["admissible"][m_] else "(pushes)") for m_ in ("S", "F1", "F2", "F3", "F4") if m_ in f["heldout_rms_nm"])))
        if f.get("lugre"):
            lg = f["lugre"]
            print("            with memory (LuGre): builds over %.0f mrad after a reversal (s0 %.0f N m/rad); held-out, every moving row: %s" % (
                lg["build_travel_mrad"], lg["s0_nm_per_rad"], "  ".join("%s %.3f" % kv for kv in lg["heldout_rms_nm_same_rows"].items())))
    give = fit_give(runs, reg, gf, x, coef, fr)
    for f in fr:                                                # one stiffness per joint: the median over the sizes where Dahl beat Coulomb alone
        ks = [r["dahl_nm_per_rad"] for r in give.get(f["joint"], {}).values() if r["dahl_kept"]]
        f["dahl_nm_per_rad"] = float(np.median(ks)) if ks else None
    if give:
        print("GIVE before sliding (back-and-forth runs): per joint and size, Dahl stiffness N m/rad (rms vs Coulomb only), lost motion mrad")
        for jn, rs in give.items():
            print("  %-9s %s" % (jn, "; ".join("%s: %.0f%s (%.2f vs %.2f) %s" % (amp, r["dahl_nm_per_rad"], "" if r["dahl_kept"] else " not kept", r["dahl_rms_nm"], r["coulomb_only_rms_nm"],
                                                                                "-" if r["lost_motion_rad"] is None else "%.1f" % (1000 * r["lost_motion_rad"])) for amp, r in rs.items())))
    act = fit_actuator(runs)
    print("ACTUATOR: delay %.0f ms, gain %s, rms %s N m" % (act["delay_ms"], np.round(act["gain"], 2), np.round(act["rms_nm"], 2)))
    if a.out:
        frj = [{k_: v for k_, v in f.items() if not k_.startswith("_")} for f in fr]
        json.dump(dict(robot="R1 %s arm" % side, simulator_note=a.label, joints=NAMES,
                       fit_runs=[os.path.basename(D["dir"]) for D in runs], check_runs=[os.path.basename(D["dir"]) for D in runs_c],
                       gravity=gf, friction=dict(joints=frj), give=give, actuator=act), open(a.out, "w"), indent=1)
        print(a.out)
        draw(a.out[:-5] + ".png", side, reg, gf, x, coef, H, fr, a.label)


def draw(path, side, reg, gf, x, coef, H, fr, label):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 5, figsize=(16, 6.5))
    Y = np.array([r["y"] for r in H])
    P = np.array([gravity_now(reg, gf, x, coef, r["q"], r["load"], r["link"]) for r in H])
    U = np.array([predict(reg, None, "urdf", r) for r in H])
    ld = np.array([r["load"] > 0 for r in H])
    for j in range(5):
        a_ = ax[0, j]
        a_.plot(Y[:, j], U[:, j], "o", ms=3, color="#bbbbbb", label="URDF")
        a_.plot(Y[~ld, j], P[~ld, j], "o", ms=4, color="#2a6fb0", label="fit (%s)" % gf["chosen"])
        if ld.any():
            a_.plot(Y[ld, j], P[ld, j], "s", ms=5, color="#c2571a", label="fit, hung load")
        lim = [min(Y[:, j].min(), P[:, j].min()) - 0.3, max(Y[:, j].max(), P[:, j].max()) + 0.3]
        a_.plot(lim, lim, color="#555", lw=0.6)
        a_.set(title="%s: held torque" % NAMES[j], xlabel="measured, N m")
        a_.grid(alpha=0.3)
        f = fr[j]
        a_ = ax[1, j]
        if f.get("model"):
            v, y, c, model, vs = f["_curve"]
            a_.plot(v, y, ".", ms=1.5, color="#2a6fb0", alpha=0.4)
            s = np.linspace(v.min(), v.max(), 300)
            a_.plot(s, fric_pred(c, s, np.zeros_like(s), np.zeros_like(s), model, vs), color="#c2571a", lw=1.2)
            if f.get("steady_points"):
                sp = np.array(f["steady_points"])
                a_.plot(np.r_[sp[:, 0], -sp[:, 0]], np.r_[sp[:, 1], -sp[:, 1]], "o", ms=4, mfc="white", mec="black", mew=0.8,
                        label="steady sweeps, gravity cancelled")
                a_.legend(fontsize=6, loc="lower right")
            a_.set(title="%s friction, %s" % (NAMES[j], model), xlabel="joint speed, rad/s")
        a_.grid(alpha=0.3)
    ax[0, 0].set_ylabel("predicted, N m")
    ax[0, 0].legend(fontsize=7)
    ax[1, 0].set_ylabel("torque after gravity, N m")
    fig.suptitle("R1 %s arm model (%s)" % (side, label))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(path)


if __name__ == "__main__":
    main()
