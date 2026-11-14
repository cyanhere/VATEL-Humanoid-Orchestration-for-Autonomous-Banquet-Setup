"""Fit each arm's joint zero offsets from table_check.py grid touches (numpy + scipy).

    python calibration/arm_offsets_fit.py logs/table/<time>/grid.json [more grid.json...]
    python calibration/arm_offsets_fit.py... --save           # also write model/arm_offsets.json (used with DEXPICK_ARM_OFFSETS=1)
"""
import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk import config  # noqa: E402
from pk.model import Robot  # noqa: E402

SIDES = ("left", "right")
NAMES = ["sh_pitch", "sh_roll", "sh_yaw", "elbow", "wrist_roll"]
SIG_D = np.radians(2.0)
SIG_B = 0.002
SIG_Z = 0.001


def load(paths):
    obs = []                                           # (side, spot id, q11, cand, plane)
    for p in paths:
        g = json.load(open(p))
        tab = g["table"]
        for r in g["rows"]:
            if "touched_z" not in r:
                continue
            zt = sorted(r["touches"], key=lambda t_: t_["z"])
            # the two touches that agreed (the closest pair), each one an observation
            i = min(range(len(zt) - 1), key=lambda k: zt[k + 1]["z"] - zt[k]["z"]) if len(zt) > 1 else 0
            for t_ in zt[i:i + 2]:
                if "q" not in t_:
                    continue
                cand = [(ln, np.array(v, float)) for ln, v in r["cand"]]
                obs.append(dict(side=r["side"], spot="%s:%d" % (os.path.basename(os.path.dirname(p)), r["spot"]), q=np.array(t_["q"], float),
                                cand=cand, plane=(tab["a"], tab["b"], tab["c"]), tilt=r["tilt"]))
    return obs


def lowest(rob, q11, cand):
    T = rob.fk(q11, np.zeros(12))
    best = None
    for ln, v in cand:
        w = v @ T[ln][:3, :3].T + T[ln][:3, 3]
        k = int(np.argmin(w[:, 2]))
        if best is None or w[k, 2] < best[2]:
            best = w[k]
    return best


def residuals(x, obs, rob, use):
    d = {"left": x[0:5] * use["left"], "right": x[5:10] * use["right"]}
    b = x[10]
    r = []
    for o in obs:
        q = o["q"].copy()
        s_ = config.LEFT if o["side"] == "left" else config.RIGHT
        q[s_] += d[o["side"]]
        p = lowest(rob, q, o["cand"])
        a_, b_, c_ = o["plane"]
        r.append((p[2] - (a_ * p[0] + b_ * p[1] + c_ + b)) / SIG_Z)
    return np.r_[r, x[:10] / SIG_D, x[10] / SIG_B]


def fit(obs, rob, use):
    from scipy.optimize import least_squares
    sol = least_squares(residuals, np.zeros(11), args=(obs, rob, use), x_scale=np.r_[[SIG_D] * 10, SIG_B])
    J = sol.jac
    try:
        cov = np.linalg.inv(J.T @ J)
    except np.linalg.LinAlgError:
        cov = np.full((11, 11), np.nan)
    return sol.x, np.sqrt(np.clip(np.diag(cov), 0, None))


def per_side(x, obs, rob, use):
    r = residuals(x, obs, rob, use)[:len(obs)] * SIG_Z
    return {s: 1000 * np.array([v for v, o in zip(r, obs) if o["side"] == s]) for s in SIDES}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("grids", nargs="+")
    ap.add_argument("--joints", default="01234", help="which joints to fit, per arm (0 sh_pitch .. 4 wrist_roll)")
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--out", default=config.ARM_OFFSETS, help="where --save writes (default model/arm_offsets.json)")
    a = ap.parse_args()
    rob = Robot(offsets=None)                          # measured q + the d fitted here: the raw model
    obs = load(a.grids)
    n = {s: sum(o["side"] == s for o in obs) for s in SIDES}
    print("%d touch observations (left %d, right %d) from %d spots" % (len(obs), n["left"], n["right"], len(set(o["spot"] for o in obs)) // 1))
    if min(n.values()) < 4:
        print("too few touches")
        return 1
    m = np.array([1.0 if str(j) in a.joints else 0.0 for j in range(5)])
    use = {"left": m, "right": m}
    r0 = per_side(np.zeros(11), obs, rob, use)
    x, sd = fit(obs, rob, use)
    r1 = per_side(x, obs, rob, use)
    for s in SIDES:
        print("%-5s before: the hand's lowest point vs the camera's table, mean %+.1f mm, rms %.1f | after: mean %+.1f, rms %.1f" % (
            s, r0[s].mean(), np.sqrt(np.mean(r0[s] ** 2)), r1[s].mean(), np.sqrt(np.mean(r1[s] ** 2))))
        k = 0 if s == "left" else 5
        print("      offsets (real = measured + d), deg: " + "  ".join("%s %+.2f+-%.2f" % (NAMES[j], np.degrees(x[k + j]), np.degrees(sd[k + j]))
                                                              for j in range(5) if m[j]))
    print("table height offset (camera): %+.1f +- %.1f mm" % (1000 * x[10], 1000 * sd[10]))
    # leave one spot out: predict its touches with the offsets from the rest
    spots = sorted(set(o["spot"] for o in obs))
    loo = {s: [] for s in SIDES}
    for sp in spots:
        train = [o for o in obs if o["spot"] != sp]
        test = [o for o in obs if o["spot"] == sp]
        if not test or min(sum(o["side"] == s for o in train) for s in SIDES) < 4:
            continue
        xt, _ = fit(train, rob, use)
        e = per_side(xt, test, rob, use)
        for s in SIDES:
            loo[s] += list(e[s])
    for s in SIDES:
        if loo[s]:
            print("%-5s leave-one-spot-out: rms %.1f mm (worst %.1f)" % (s, np.sqrt(np.mean(np.square(loo[s]))), np.max(np.abs(loo[s]))))
    if a.save:
        out = dict(offsets_rad={s: [float(v) for v in x[(0 if s == "left" else 5):(5 if s == "left" else 10)]] for s in SIDES},
                   sd_rad={s: [float(v) for v in sd[(0 if s == "left" else 5):(5 if s == "left" else 10)]] for s in SIDES},
                   table_offset_m=float(x[10]), grids=a.grids, joints=a.joints,
                   note="real joint angle = measured + offset (per arm: sh_pitch, sh_roll, sh_yaw, elbow, wrist_roll); applied by "
                        "pk.model.Robot only with DEXPICK_ARM_OFFSETS=1")
        path = a.out
        json.dump(out, open(path, "w"), indent=1)
        print("wrote", path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
