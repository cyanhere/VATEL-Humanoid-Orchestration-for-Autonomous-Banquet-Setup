"""Inverse kinematics for one 5-DoF arm: damped least squares with joint limits, numpy only."""
import time

import numpy as np

from . import config
from .model import axis_angle


class ArmChain(object):
    def __init__(self, rob, side, link, q11=None, hands12=None):
        self.rob, self.side, self.link = rob, side, link
        q11 = np.zeros(config.N_ARM) if q11 is None else np.asarray(q11, float)
        vals = rob.joint_values(q11, hands12)
        # mimic joints take their value from their source
        for jn, j in rob.joints.items():
            if j["mimic"] is not None:
                src, k, off = j["mimic"]
                vals[jn] = k * vals.get(src, 0.0) + off
        chain, l = [], link
        child_of = {j["child"]: jn for jn, j in rob.joints.items()}
        while l != rob.root:
            jn = child_of[l]
            chain.append(jn)
            l = rob.joints[jn]["parent"]
        chain.reverse()
        self.chain = [rob.joints[jn] for jn in chain]
        arm = rob.arm_joints[side]
        self.arm_pos = [chain.index(jn) for jn in arm]
        self.fixed = [vals.get(jn, 0.0) for jn in chain]
        sl = config.LEFT if side == "left" else config.RIGHT
        self.lo = config.ARM_LO[sl] + config.LIMIT_MARGIN
        self.hi = config.ARM_HI[sl] - config.LIMIT_MARGIN

    def fk(self, q5):
        """T (pelvis <- link) and [(origin, axis)] of the 5 arm joints."""
        v = list(self.fixed)
        for k, i in enumerate(self.arm_pos):
            v[i] = float(q5[k])
        T = np.eye(4)
        frames = []
        for i, j in enumerate(self.chain):
            T = T @ j["T"]
            if i in self.arm_pos:
                frames.append((T[:3, 3].copy(), T[:3, :3] @ j["axis"]))
            if j["type"] in ("revolute", "continuous") and v[i]:
                R = np.eye(4)
                R[:3, :3] = axis_angle(j["axis"], v[i])
                T = T @ R
        return T, frames

    def point(self, q5, p_local):
        T, _ = self.fk(q5)
        return T[:3, :3] @ np.asarray(p_local, float) + T[:3, 3]


def _lm(residual, q0, lo, hi, iters=50, lam=1e-3, tol=1e-7):
    q = np.clip(np.asarray(q0, float), lo, hi)
    r, J = residual(q)
    cost = float(r @ r)
    for _ in range(iters):
        A = J.T @ J + lam * np.eye(len(q))
        dq = -np.linalg.solve(A, J.T @ r)
        qn = np.clip(q + dq, lo, hi)
        rn, Jn = residual(qn)
        cn = float(rn @ rn)
        if cn < cost:
            q, r, J, cost = qn, rn, Jn, cn
            lam = max(lam * 0.3, 1e-7)
            if np.max(np.abs(dq)) < tol:
                break
        else:
            lam *= 10.0
            if lam > 1e4:
                break
    return q, r


def solve_point_dir(ch, q0, P, p_local, a_local=None, n=None, w=0.05, iters=50):
    """A point fixed in ch.link to P (pelvis frame), and (soft, weight w) the link's axis a_local along direction n.
    Returns (q5, dict(pos_mm, ang_deg))."""
    P = np.asarray(P, float)
    p_local = np.asarray(p_local, float)
    use_dir = a_local is not None and n is not None
    if use_dir:
        a_local = np.asarray(a_local, float)
        n = np.asarray(n, float) / np.linalg.norm(n)

    def residual(q):
        T, fr = ch.fk(q)
        p = T[:3, :3] @ p_local + T[:3, 3]
        Jp = np.array([np.cross(z, p - o) for o, z in fr]).T
        if not use_dir:
            return p - P, Jp
        a = T[:3, :3] @ a_local
        Ja = np.array([np.cross(z, a) for o, z in fr]).T
        return np.r_[p - P, w * (a - n)], np.vstack([Jp, w * Ja])

    q, r = _lm(residual, q0, ch.lo, ch.hi, iters)
    T, _ = ch.fk(q)
    p = T[:3, :3] @ p_local + T[:3, 3]
    info = dict(pos_mm=float(np.linalg.norm(p - P) * 1000))
    if use_dir:
        a = T[:3, :3] @ a_local
        info["ang_deg"] = float(np.degrees(np.arccos(np.clip(a @ n, -1, 1))))
    return q, info


def solve_pose(ch, q0, P, R, p_local=(0.0, 0.0, 0.0), w_rot=0.2, iters=60):
    """A point fixed in ch.link to P, and the link's frame toward R (soft, weight w_rot on the 9 matrix entries)."""
    P, R = np.asarray(P, float), np.asarray(R, float)
    p_local = np.asarray(p_local, float)

    def residual(q):
        T, fr = ch.fk(q)
        Rq = T[:3, :3]
        p = Rq @ p_local + T[:3, 3]
        Jp = np.array([np.cross(z, p - o) for o, z in fr]).T
        # d(R[:, k]) / dq_i = z_i x R[:, k]
        Jr = np.array([np.concatenate([np.cross(z, Rq[:, k]) for k in range(3)]) for o, z in fr]).T
        return np.r_[p - P, w_rot * (Rq - R).ravel(order="F")], np.vstack([Jp, w_rot * Jr])

    q, r = _lm(residual, q0, ch.lo, ch.hi, iters)
    T, _ = ch.fk(q)
    p = T[:3, :3] @ p_local + T[:3, 3]
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R.T @ T[:3, :3]) - 1) / 2, -1, 1))))
    return q, dict(pos_mm=float(np.linalg.norm(p - P) * 1000), rot_deg=ang)


def shift_path(rob, side, link, p_local, path_q11, path_h12, delta, w_rot=0.2, max_mm=3.0, waist=None):
    """Move a recorded arm path so that a point of `link` is displaced by `delta` (pelvis frame, m; one 3-vector or N x 3)
    at every waypoint, orientation kept as a soft weight. The other arm and the waist are unchanged.

    waist: the robot's actual waist yaw. The arm is solved with it so the hand lands at the target in the pelvis frame.
    Returns (new q11 path, worst position error in mm).
    """
    sl = config.LEFT if side == "left" else config.RIGHT
    out = np.array(path_q11, float).copy()
    worst = 0.0
    q_prev = out[0, sl].copy()
    for k in range(len(out)):
        if k % 20 == 0:
            time.sleep(0.001)                           # let other threads (the client's heartbeat) run
        h_k = path_h12[k] if path_h12 is not None else None
        ch = ArmChain(rob, side, link, out[k], h_k)
        T, _ = ch.fk(out[k, sl])
        d_k = np.asarray(delta, float)
        if d_k.ndim == 2:                              # one displacement per waypoint (ramped shifts)
            d_k = d_k[k]
        P = T[:3, :3] @ np.asarray(p_local, float) + T[:3, 3] + d_k
        if waist is not None:
            q_w = out[k].copy()
            q_w[config.WAIST] = waist
            ch = ArmChain(rob, side, link, q_w, h_k)
        seed = out[k, sl] + (q_prev - path_q11[max(k - 1, 0)][sl] if k else 0.0)
        q5, info = solve_pose(ch, seed, P, T[:3, :3], p_local, w_rot)
        worst = max(worst, info["pos_mm"])
        out[k, sl] = q5
        q_prev = q5
    return out, worst
