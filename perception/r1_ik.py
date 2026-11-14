"""Inverse kinematics for one R1 arm: put the hand's pinch point somewhere, sensibly.

The R1's arm has five joints (shoulder pitch, roll, yaw, elbow, wrist roll), so it can
place a point (3) and aim the hand (2 of the 3 rotations) -- not a full pose. So the
pinch point's POSITION is the hard requirement, and the rest are wishes, weighted:

    - the forearm pointing where the hand should go in from (toward the plate's centre,
      tilted down a little, so the fingers come over the rim);
    - the palm facing down, once measure-hand has said which way the palm faces;
    - the elbow and wrist kept clear above the table (a penalty that is zero when clear);
    - close to where the arm already is (a small pull, so consecutive waypoints give
      neighbouring solutions and the arm does not flip between them).

Solved with scipy's bounded least squares inside the URDF's joint limits (less 3 deg),
from a few starting points, keeping the best that actually reaches. Where scipy is not
installed (the robot computer may not have it), a small bounded Levenberg-Marquardt in
numpy does the same job, a little slower. The waist and head
are held at their measured angles: they are not part of the solve.

This file moves nothing. r1_arm.py moves; reach_plate.py decides where.
"""
import math

import numpy as np

import r1_config as cfg
import urdf_fk as fk

LIMIT_MARGIN_RAD = math.radians(3.0)
POS_TOL_M = 0.010               # a solve that misses by more than this did not reach
TABLE_CLEAR_M = 0.05            # elbow and wrist at least this far above the table
FORCE_NUMPY = False             # tests set this to try the no-scipy path


def _least_squares():
    """scipy's solver, or the numpy one with the same call shape."""
    if not FORCE_NUMPY:
        try:
            from scipy.optimize import least_squares
            return least_squares
        except ImportError:
            pass
    return _bounded_lm


def _bounded_lm(f, x0, bounds, max_nfev=400, **_):
    """Levenberg-Marquardt, each step clipped into the bounds. Enough for five joints."""
    lo, hi = bounds
    x = np.clip(np.asarray(x0, float), lo, hi)
    r = f(x)
    cost, lam, nfev, h = 0.5 * r @ r, 1e-2, 1, 1e-6
    while nfev < max_nfev:
        Jm = np.empty((r.size, x.size))
        for i in range(x.size):
            xh = x.copy()
            xh[i] += h if xh[i] + h <= hi[i] else -h
            Jm[:, i] = (f(xh) - r) / (xh[i] - x[i])
        nfev += x.size
        A, g = Jm.T @ Jm, Jm.T @ r
        stepped = False
        for _ in range(8):
            dx = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-9), -g)
            xn = np.clip(x + dx, lo, hi)
            rn = f(xn)
            nfev += 1
            cn = 0.5 * rn @ rn
            if cn < cost:
                stepped = cost - cn > 1e-12 * max(cost, 1e-12)
                x, r, cost, lam = xn, rn, cn, max(lam / 3.0, 1e-9)
                break
            lam *= 4.0
        if not stepped:                          # no better step, or a negligible one
            break
    return type("Result", (), {"x": x, "cost": cost})()


def arm_names(arm):
    return cfg.arm_joints(arm)


def tool_point(joints, angles, arm, tool=None):
    """The pinch point, the forearm's direction, and the wrist rotation, pelvis frame."""
    Tw = fk.T_pelvis_wrist(joints, angles, arm)
    t = cfg.tool_offset(arm) if tool is None else np.asarray(tool, float)
    return Tw[:3, :3] @ t + Tw[:3, 3], Tw[:3, 0].copy(), Tw[:3, :3]


def _over_table(p, table_edge_x):
    """Is this point above the table's footprint? (The table starts at x = table_edge_x
    in front of the robot; None: assume everything in front is table.)"""
    return table_edge_x is None or p[0] > table_edge_x - 0.03


def solve(joints, arm, target, start, fixed=None, forearm_dir=None, palm_down=True,
          table_z=None, table_edge_x=None, tool=None, palm=None, w_fore=4.0, w_palm=2.0,
          w_stay=0.05, extra_starts=True):
    """Arm angles that put the pinch point at `target` (pelvis frame, metres).

    start: {joint: rad} the arm's angles now (or the previous waypoint's) -- the solve
    starts there and is pulled gently toward it. fixed: angles held for every other joint
    (waist, head, the other arm). Returns a dict: angles (all joints), arm (the five),
    pos_err_m, forearm_err_deg, palm_err_deg, clearance_m (lowest elbow/wrist height above
    the table), reached (pos_err within POS_TOL_M).
    """
    least_squares = _least_squares()

    names = arm_names(arm)
    lim = cfg.joint_limits(joints)
    lo = np.array([lim[n][0] + LIMIT_MARGIN_RAD for n in names])
    hi = np.array([lim[n][1] - LIMIT_MARGIN_RAD for n in names])
    base = dict(fixed or {})
    x0 = np.clip(np.array([start.get(n, 0.0) for n in names]), lo, hi)
    target = np.asarray(target, float)
    t_off = cfg.tool_offset(arm) if tool is None else np.asarray(tool, float)
    p_ax = cfg.palm_axis(arm) if palm is None else palm
    if forearm_dir is not None:
        forearm_dir = np.asarray(forearm_dir, float) / np.linalg.norm(forearm_dir)

    def full(x):
        a = dict(base)
        a.update(zip(names, x))
        return a

    def residual(x):
        a = full(x)
        Tw = fk.T_pelvis_wrist(joints, a, arm)
        R, w = Tw[:3, :3], Tw[:3, 3]
        r = list((R @ t_off + w - target) / 0.002)            # 2 mm of miss ~ 1
        if forearm_dir is not None:
            r += list(w_fore * (R[:, 0] - forearm_dir))
        if palm_down and p_ax is not None:
            r += list(w_palm * (R @ p_ax - np.array([0.0, 0.0, -1.0])))
        if table_z is not None:
            kp = fk.arm_keypoints(joints, a, arm)
            for k in ("elbow_joint", "wrist_roll_joint"):
                low = max(0.0, table_z + TABLE_CLEAR_M - kp[k][2]) / 0.005
                r.append(low if _over_table(kp[k], table_edge_x) else 0.0)
        r += list(w_stay * (x - x0))
        return np.array(r)

    starts = [x0]
    if extra_starts:
        ie = names.index("%s_elbow_joint" % arm)
        ip = names.index("%s_shoulder_pitch_joint" % arm)
        for d_pitch, d_elbow in ((0.0, 0.6), (0.0, -0.4), (-0.6, -0.6), (-1.0, 0.0), (0.4, -0.8)):
            s = x0.copy()
            s[ip] += d_pitch
            s[ie] += d_elbow
            starts.append(np.clip(s, lo, hi))
        starts.append(np.clip(np.zeros(len(names)), lo, hi))
    best = None
    for s in starts:
        sol = least_squares(residual, s, bounds=(lo, hi), method="trf", x_scale=1.0,
                            max_nfev=400)
        out = _report(joints, arm, full(sol.x), names, target, t_off, forearm_dir, p_ax,
                      table_z, table_edge_x)
        key = (not out["reached"], sol.cost)
        if best is None or key < best[0]:
            best = (key, out)
    return best[1]


def _report(joints, arm, angles, names, target, t_off, forearm_dir, p_ax, table_z,
            table_edge_x=None):
    Tw = fk.T_pelvis_wrist(joints, angles, arm)
    R, w = Tw[:3, :3], Tw[:3, 3]
    p = R @ t_off + w
    out = {"angles": angles, "arm": {n: float(angles[n]) for n in names},
           "tool_m": p, "wrist_m": w.copy(), "pos_err_m": float(np.linalg.norm(p - target))}
    out["reached"] = out["pos_err_m"] <= POS_TOL_M
    out["forearm_err_deg"] = (float(math.degrees(math.acos(np.clip(R[:, 0] @ forearm_dir, -1, 1))))
                              if forearm_dir is not None else None)
    out["palm_err_deg"] = (float(math.degrees(math.acos(np.clip((R @ p_ax)[2] * -1, -1, 1))))
                           if p_ax is not None else None)
    if table_z is not None:
        kp = fk.arm_keypoints(joints, angles, arm)
        over = [kp[k][2] for k in ("elbow_joint", "wrist_roll_joint")
                if _over_table(kp[k], table_edge_x)]
        out["clearance_m"] = float(min(over) - table_z) if over else None
    else:
        out["clearance_m"] = None
    return out


def sweep_clearance(joints, arm, a0, a1, table_z, tool=None, n=12, table_edge_x=None):
    """Lowest heights above the table met while moving in a straight line in joint space
    from a0 to a1, over the table only: (elbow/wrist, pinch point). 9.0 where nothing of
    that kind is over the table at all."""
    names = arm_names(arm)
    t_off = cfg.tool_offset(arm) if tool is None else np.asarray(tool, float)
    low_kp, low_tool = 9.0, 9.0
    for s in np.linspace(0.0, 1.0, n):
        a = dict(a0)
        a.update({k: (1 - s) * a0[k] + s * a1[k] for k in names})
        kp = fk.arm_keypoints(joints, a, arm)
        for k in ("elbow_joint", "wrist_roll_joint"):
            if _over_table(kp[k], table_edge_x):
                low_kp = min(low_kp, kp[k][2] - table_z)
        Tw = fk.T_pelvis_wrist(joints, a, arm)
        pt = Tw[:3, :3] @ t_off + Tw[:3, 3]
        if _over_table(pt, table_edge_x):
            low_tool = min(low_tool, float(pt[2] - table_z))
    return low_kp, low_tool
