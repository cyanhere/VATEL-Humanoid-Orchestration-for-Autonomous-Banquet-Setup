"""Gravity identification from run logs, and the fingertip force estimated from the arm's joint lag."""
import time

import numpy as np

from . import config
from .model import Robot

STATIC_STAGES = ("wait", "play", "pause", "hold", "goto", "home", "go_home", "held", "client", "rewind", "freeze")


def samples(L, every=5, dq_max=0.3):
    """Usable rows of a log: full weight, slow, not a weight ramp, motors not limp."""
    stage = np.array([str(x).lower() for x in L["stage"]])
    ok = (L["weight"] >= 0.99) & np.isin(stage, STATIC_STAGES) & (np.abs(L["dq"]).max(axis=1) < dq_max)
    ok &= np.abs(L["tau"][:, :10]).max(axis=1) >= config.LIMP_TAU
    idx = np.nonzero(ok)[0][::every]
    return idx


def measured_torque(L, idx):
    """What the PD gave: with the gains it had on each row (logs.load; Unitree's when the log does not say)."""
    kp = L["kp"][idx] if "kp" in L else config.KP
    kd = L["kd"][idx] if "kd" in L else config.KD
    return kp * (L["cmd"][idx] - L["q"][idx]) - kd * L["dq"][idx]


def gravity_rows(rob, L, idx):
    return np.array([rob.gravity11(L["q"][i], L["hq"][i]) for i in idx])


def fit(logs_and_idx, rob=None, friction=True):
    """logs_and_idx: [(log, row indices)]. Returns dict(scale, bias_rad, friction_Nm, stats)."""
    rob = rob or Robot()
    tau, g, dq = [], [], []
    for L, idx in logs_and_idx:
        if len(idx) == 0:
            continue
        tau.append(measured_torque(L, idx))
        g.append(gravity_rows(rob, L, idx))
        dq.append(L["dq"][idx])
    tau, g, dq = np.vstack(tau), np.vstack(g), np.vstack(dq)
    scale, bias, fric = np.ones(config.N_ARM), np.zeros(config.N_ARM), np.zeros(config.N_ARM)
    stats = []
    for j in range(config.N_ARM - 1):                       # the waist yaw carries no arm gravity
        cols = [g[:, j], np.ones(len(g))] + ([np.tanh(dq[:, j] / 0.02)] if friction else [])
        A = np.array(cols).T
        x, *_ = np.linalg.lstsq(A, tau[:, j], rcond=None)
        res = tau[:, j] - A @ x
        res0 = tau[:, j] - g[:, j]                           # URDF only
        scale[j], bias[j] = x[0], x[1] / config.KP[j]
        if friction:
            fric[j] = x[2]
        var = max(float(np.var(tau[:, j])), 1e-12)
        stats.append(dict(joint=config.ARM_NAMES[j], scale=round(float(x[0]), 3), bias_Nm=round(float(x[1]), 3),
                          friction_Nm=round(float(fric[j]), 3), rms_Nm=round(float(np.sqrt(np.mean(res ** 2))), 3),
                          rms_urdf_Nm=round(float(np.sqrt(np.mean(res0 ** 2))), 3),
                          r2=round(1 - float(np.var(res)) / var, 3), range_Nm=[round(float(tau[:, j].min()), 2), round(float(tau[:, j].max()), 2)]))
    return dict(scale=scale.tolist(), bias_rad=bias.tolist(), friction_Nm=fric.tolist(), n=int(len(tau)),
                date=time.strftime("%Y-%m-%d %H:%M"), stats=stats)


def external_force(rob, q11, hands12, sent11, dq11, side, link, p_local, gfit, kp=config.KP, kd=config.KD):
    """Force of the world ON the fingertip point (N, pelvis_link), from the arm's joint lag minus gravity and bias.
    sent11: what the PD was told (incl. any gravity offset)."""
    T = rob.fk(q11, hands12)
    sl = config.LEFT if side == "left" else config.RIGHT
    tau = kp[sl] * (sent11[sl] - q11[sl]) - kd[sl] * dq11[sl]
    tau_g = np.asarray(gfit["scale"])[sl] * rob.gravity(T, side) + np.asarray(gfit["bias_rad"])[sl] * config.KP[sl]   # bias: N m, stored at Unitree's kp
    p = rob.point(T, link, p_local)
    J = rob.jacobian(T, side, p)
    # motor torque = tau_g - J^T F_ext -> F_ext = -pinv(J^T) (tau - tau_g)
    return -force_pinv(J, tau - tau_g), p


def force_pinv(J, tau, lever_min=0.08):
    """Force at a point from joint torques, tau = J^T F, in the directions the joints can feel: a direction whose
    lever arm (singular value of J, m) is below lever_min is left out instead of dividing a model error by ~0
    (an arm hanging straight down cannot feel a vertical push on its hand)."""
    U, S, Vt = np.linalg.svd(J, full_matrices=False)           # J = U S Vt (3x3, 3, 3x5)
    inv = np.where(S > lever_min, 1.0 / np.maximum(S, 1e-9), 0.0)
    return U @ (inv * (Vt @ tau))
