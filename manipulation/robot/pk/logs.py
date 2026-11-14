"""Read run logs (dp_play play_*.csv and the server logs) into numpy arrays."""
import csv

import numpy as np

from . import config

_ARM = config.ARM_NAMES


def load(path):
    rows = []
    with open(path) as f:
        rd = csv.reader(f)
        hdr = next(rd)
        for r in rd:
            if len(r) == len(hdr) and "" not in r:
                rows.append(r)
    col = {h: i for i, h in enumerate(hdr)}

    def num(names):
        idx = [col[n] for n in names]
        return np.array([[float(r[i]) for i in idx] for r in rows]) if rows else np.zeros((0, len(idx)))

    out = dict(path=path, n=len(rows), header=hdr)
    out["wall"] = num(["wall"])[:, 0]
    out["traj_t"] = num(["traj_t"])[:, 0] if "traj_t" in col else np.zeros(len(rows))
    out["stage"] = np.array([r[col["stage"]] for r in rows])
    out["phase"] = np.array([r[col["phase"]] for r in rows]) if "phase" in col else np.array([""] * len(rows))
    out["weight"] = num(["weight"])[:, 0] if "weight" in col else np.ones(len(rows))
    out["cmd"] = num(["cmd_" + j for j in _ARM])          # what was SENT (dp_play: the plan; server: plan + gravity)
    out["q"] = num(["q_" + j for j in _ARM])
    out["tau"] = num(["tau_" + j for j in _ARM])
    if ("des_" + _ARM[0]) in col:                          # server logs: the plan before the gravity offset
        out["des"] = num(["des_" + j for j in _ARM])
    else:
        out["des"] = out["cmd"].copy()
    hk = [("%s_%s_%s" % (w, s, f)) for w in ("hcmd", "hq", "hcur") for s in "LR" for f in config.FINGERS]
    if all(k in col for k in hk):
        out["hcmd"] = num(["hcmd_%s_%s" % (s, f) for s in "LR" for f in config.FINGERS])
        out["hq"] = num(["hq_%s_%s" % (s, f) for s in "LR" for f in config.FINGERS])
        out["hcur"] = num(["hcur_%s_%s" % (s, f) for s in "LR" for f in config.FINGERS])
    else:
        out["hcmd"] = out["hq"] = out["hcur"] = np.zeros((len(rows), 12))
    for k in ("imu_roll", "imu_pitch", "q_head_pitch", "q_head_yaw", "cmd_head_pitch", "cmd_head_yaw"):
        if k in col:
            out[k] = num([k])[:, 0]
    # the gains the PD had (server logs since older logs and dp_play: Unitree's), and the friction lead in cmd
    ks = num(["kp_scale", "kd_scale"]) if "kp_scale" in col else np.ones((len(rows), 2))
    arm = (np.arange(config.N_ARM) != config.WAIST).astype(float)         # as config.arm_gains
    out["kp"] = config.KP * (1.0 + arm * (ks[:, :1] - 1.0))
    out["kd"] = config.KD * (1.0 + arm * (ks[:, 1:2] - 1.0))
    out["fric"] = num(["fric_" + j for j in _ARM]) if ("fric_" + _ARM[0]) in col else np.zeros((len(rows), config.N_ARM))
    # joint velocity from the positions (the logs carry no dq): central difference, 100 Hz logs
    q, t = out["q"], out["wall"]
    dq = np.zeros_like(q)
    if len(t) > 2:
        dt = np.maximum(t[2:] - t[:-2], 1e-3)[:, None]
        dq[1:-1] = (q[2:] - q[:-2]) / dt
    out["dq"] = dq
    return out


def segments(labels):
    """[(label, start index, end index exclusive)] of runs of equal labels."""
    out, start = [], 0
    for i in range(1, len(labels) + 1):
        if i == len(labels) or labels[i] != labels[start]:
            out.append((labels[start], start, i))
            start = i
    return out
