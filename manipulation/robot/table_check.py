#!/usr/bin/env python3
"""Check that the head camera sees the table where the hands touch it. Runs instead of perception.py.

    python3 table_check.py --dry                  # the plan only: both hands' IK and their mirror symmetry; nothing moves
    python3 table_check.py pose                   # 1. both hands to the same pose, mirrored, by IK (palm down, elbows out,
    python3 table_check.py touch                  # 1-3. the pose; the camera measures the table; both hands go down to
    python3 table_check.py touch --head 0.45 -0.2 # the camera measures with the head at pitch, yaw (rad)
    python3 table_check.py sweep                  # 4. camera only, after a touch run: the head through pitch x yaw, each
    python3 table_check.py touch sweep            # both in one go
"""
import argparse
import glob
import json
import os
import sys
import threading
import time
from collections import deque

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk.fit import force_pinv  # noqa: E402
from pk.ik import ArmChain, _lm  # noqa: E402
from pk.model import PADS, Robot  # noqa: E402
from pk.tracker import deproject  # noqa: E402

SIDES = ("left", "right")
MIRROR = np.array([1.0, -1.0, -1.0, 1.0, -1.0])        # right arm joints = MIRROR x left: the URDF's mirror image
TILT_DEG = 15.0                                         # fingers this much below horizontal (--tilt): the fingertips touch first
                                                        # (5 deg until the left hand's palm camera, not in the model, hit the table)
W_ROT = 0.3
SEED = np.array([-0.2, 1.1, -0.4, -0.2, 0.47])         # left arm: elbow out, palm down (found offline)
TOUCH_IDX = {f: i for i, f in enumerate(config.TOUCH_FINGERS)}


def sl(side):
    return config.LEFT if side == "left" else config.RIGHT


# ============================================================================ the pose
class Flat(object):
    """One arm's IK for the flat hand: the little finger's pad centre to a point, the hand palm down with its fingers
    TILT_DEG below horizontal and no roll; the heading is what the arm gives."""

    def __init__(self, rob, side, q11, hands):
        self.side = side
        self.link = "%s_base_link" % side
        self.ch = ArmChain(rob, side, self.link, q11, hands)
        T = rob.fk(q11, hands)
        M, P = T[self.link], T["%s_pinky_touch_link" % side]
        c = P[:3, :3] @ np.array(PADS[side]["pinky"]["center"]) + P[:3, 3]
        self.p_local = M[:3, :3].T @ (c - M[:3, 3])                             # the pad centre, in the hand's frame

    def solve(self, q0, P, iters=60):
        s, co = np.sin(np.radians(TILT_DEG)), np.cos(np.radians(TILT_DEG))
        p_local, ch = self.p_local, self.ch

        def residual(q):
            T, fr = ch.fk(q)
            R = T[:3, :3]
            p = R @ p_local + T[:3, 3]
            Jp = np.array([np.cross(z, p - o) for o, z in fr]).T
            J = [Jp]
            r = [p - P]
            # the hand's frame: x the palm's normal (the pads face +x), y across the hand, z along the fingers
            for k, want in ((0, -co), (1, 0.0), (2, -s)):
                J.append(W_ROT * np.array([np.cross(z, R[:, k]) for o, z in fr]).T[2:3])
                r.append([W_ROT * (R[2, k] - want)])
            return np.concatenate(r), np.vstack(J)
        q, r = _lm(residual, q0, ch.lo, ch.hi, iters)
        T, _ = ch.fk(q)
        R = T[:3, :3]
        return q, dict(pos_mm=1000 * float(np.linalg.norm(r[:3])), tilt_deg=float(np.degrees(np.arcsin(-R[2, 2]))),
                       roll_deg=float(np.degrees(np.arcsin(R[2, 1]))), palm_down=bool(R[2, 0] < -0.9),
                       heading_deg=float(np.degrees(np.arctan2(R[1, 2], R[0, 2]))))

    def solve_best(self, q0, P):
        seeds = [q0, SEED if self.side == "left" else MIRROR * SEED]
        rs = np.random.RandomState(0)
        seeds += [rs.uniform(self.ch.lo, self.ch.hi) for _ in range(30)]
        best = None
        for sd in seeds:
            q, info = self.solve(sd, P, 150)
            score = info["pos_mm"] + (0.0 if info["palm_down"] else 1e3) + abs(info["roll_deg"]) + abs(info["tilt_deg"] - TILT_DEG)
            if best is None or score < best[2] - 1e-6:
                best = (q, info, score)
            if score < 0.5:
                break
        return best[0], best[1]


class HandPoints(object):
    """The hand's collision points near its lowest one (at the planned pose), for a fast lowest point every cycle."""

    def __init__(self, rob, q11, hands, side, band=0.006):
        z = np.load(os.path.join(config.MODEL_DIR, "hand_points.npz"))
        T = rob.fk(q11, hands)
        pts = {ln: z[ln].astype(float) for ln in z.files if ln.startswith(side + "_") and ln in T}
        zz = {ln: (v @ T[ln][:3, :3].T + T[ln][:3, 3])[:, 2] for ln, v in pts.items()}
        low = min(float(v.min()) for v in zz.values())
        self.cand = [(ln, pts[ln][zz[ln] < low + band]) for ln in pts if np.any(zz[ln] < low + band)]

    def lowest(self, T):
        best = None
        for ln, v in self.cand:
            w = v @ T[ln][:3, :3].T + T[ln][:3, 3]
            k = int(np.argmin(w[:, 2]))
            if best is None or w[k, 2] < best[0]:
                best = (float(w[k, 2]), w[k], ln)
        return best                                     # (z, point, link)


def plan(rob, home_q, hands, x, y, z_low):
    """Both arms, mirrored targets: the hand's lowest point at z_low, the little finger's pad at (x, +-y). Returns
    (q11, {side: Flat}, {side: info})."""
    q11 = home_q.copy()
    flats, infos = {}, {}
    for side in SIDES:
        f = Flat(rob, side, home_q, hands)
        P = np.array([x, y if side == "left" else -y, z_low + 0.007])
        q5 = q11[sl(side)]
        for _ in range(3):                              # the pad 7 mm above the lowest point, roughly: measure, correct
            q5, info = f.solve_best(q5, P)
            q11[sl(side)] = q5
            z, _, ln = HandPoints(rob, q11, hands, side).lowest(rob.fk(q11, hands))
            P[2] += z_low - z
        info.update(P=P, lowest_link=ln)
        flats[side], infos[side] = f, info
    return q11, flats, infos


def symmetry(q11):
    """Right minus mirrored left, per joint (deg)."""
    return np.degrees(q11[config.RIGHT] - MIRROR * q11[config.LEFT])


def report_pose(rob, st, hands_pts, head=""):
    q, h = np.array(st["q"]), np.array(st["hands"])
    T = rob.fk(q, h)
    des = np.array(st["des"])
    print("  joints (deg)        %s" % "  ".join("%9s" % n[2:] for n in config.ARM_NAMES[:5]))
    print("  left  commanded     %s" % "  ".join("%9.2f" % v for v in np.degrees(des[config.LEFT])))
    print("  right commanded     %s" % "  ".join("%9.2f" % v for v in np.degrees(des[config.RIGHT])))
    print("  command, R - mirror(L)   %s   <- both IKs alike: ~0" % "  ".join("%7.2f" % v for v in symmetry(des)))
    print("  measured, R - mirror(L)  %s   <- the arms' own joints" % "  ".join("%7.2f" % v for v in symmetry(q)))
    for side in SIDES:
        M = T["%s_pinky_touch_link" % side]
        c = M[:3, :3] @ np.array(PADS[side]["pinky"]["center"]) + M[:3, 3]
        z, p, ln = hands_pts[side].lowest(T)
        print("  %-5s little-finger pad at (%.3f %+.3f %.4f), lowest hand point %.4f m (%s)  [arm's joints]" % (side, c[0], c[1], c[2], z, ln))
    zl = {s: hands_pts[s].lowest(T)[0] for s in SIDES}
    print("  lowest points, left - right: %+.1f mm by the arms' joints -- compare with what you see (a ruler from the table)%s" % (
        1000 * (zl["left"] - zl["right"]), head))


# ============================================================================ the camera's table
def fit_plane(P, rs, n_iter=300, tol=0.004, max_slope=0.27):
    """The dominant near-level plane z = a x + b y + c (RANSAC, then least squares on its points). (a, b, c, n points)."""
    if len(P) < 200:
        return None
    best, best_n = None, 0
    A = np.c_[P[:, :2], np.ones(len(P))]
    sub = rs.choice(len(P), min(len(P), 4000), replace=False)               # the search on a sample, the fit on all
    As, Ps = A[sub], P[sub]
    for _ in range(n_iter):
        i = rs.choice(len(Ps), 3, replace=False)
        try:
            m = np.linalg.solve(As[i], Ps[i, 2])
        except np.linalg.LinAlgError:
            continue
        if abs(m[0]) > max_slope or abs(m[1]) > max_slope:
            continue
        n = int(np.sum(np.abs(As @ m - Ps[:, 2]) < tol))
        if n > best_n:
            best, best_n = m, n
    if best is None:
        return None
    for t in (tol, 0.0025):
        ok = np.abs(A @ best - P[:, 2]) < t
        best = np.linalg.lstsq(A[ok], P[ok, 2], rcond=None)[0]
    return float(best[0]), float(best[1]), float(best[2]), int(ok.sum())


def camera_table(cam, kin, c, n_frames, out_dir=None, tag="table", seed=0):
    """The table as the camera sees it (pelvis frame, R1_Plate_Perception's calibration, the robot's own arms cut out),
    median of n_frames. dict(a, b, c, head, n, spread_mm) or None."""
    rs = np.random.RandomState(seed)
    fits, heads = [], []
    for f in range(n_frames):
        c.state()
        with c.busy(10):                                # the arms hold meanwhile (the NX takes its time over a frame)
            fs, depth, colour, t = cam.frames()
            st = c.state()
            ang = kin.angles(st)
            T = kin.T_po(ang)
            H, W = depth.shape
            Po = deproject(depth, cam.intr_d, cam.scale, (0, W, 0, H), 4)
            Po = Po @ cam.T_cd[:3, :3].T + cam.T_cd[:3, 3]                    # depth optical -> colour optical
            P = Po @ T[:3, :3].T + T[:3, 3]
            keep = (P[:, 0] > 0.15) & (P[:, 0] < 0.85) & (np.abs(P[:, 1]) < 0.45) & (P[:, 2] > -0.15) & (P[:, 2] < 0.20)
            for ctr, r in kin.spheres(st):
                keep &= np.sum((P - ctr) ** 2, axis=1) > r * r
            m = fit_plane(P[keep], rs)
            if m is not None:
                fits.append(m)
                heads.append([st["head"][0], st["head"][1]])
            if out_dir and f == 0:
                np.savez_compressed(os.path.join(out_dir, tag), depth=depth, color=colour if colour is not None else np.zeros(1),
                                    intr=json.dumps(cam.intr_d), intr_c=json.dumps(cam.intr_c), T_cd=cam.T_cd, depth_scale=cam.scale,
                                    state=json.dumps(st), T_po=T, angles=json.dumps(ang))
        c.hold(0.05)
    if not fits:
        return None
    F = np.array(fits)
    a, b, c0 = np.median(F[:, :3], axis=0)
    zc = F[:, 2] + F[:, 0] * 0.4                                              # height at x 0.4, y 0 per frame: the spread
    return dict(a=float(a), b=float(b), c=float(c0), n=int(np.median(F[:, 3])), frames=len(F),
                spread_mm=float(1000 * (zc.max() - zc.min())), head=np.median(np.array(heads), axis=0).tolist(),
                tilt_left_down_deg=float(np.degrees(np.arctan(-b))), tilt_front_down_deg=float(np.degrees(np.arctan(-a))))


def z_at(tab, xy):
    return tab["a"] * xy[0] + tab["b"] * xy[1] + tab["c"]


def wait_head(hc, target, arms, timeout=8.0):
    from calib_cam import wait_head as wh
    return wh(hc, target, arms, timeout)


# ============================================================================ going down
class Hand(object):
    def __init__(self, side, flat, q5, P, pts):
        self.side, self.flat, self.q5, self.pts = side, flat, q5.copy(), pts
        self.xy, self.z_cmd, self.z0 = np.array(P[:2], float), float(P[2]), float(P[2])
        self.state = "idle"
        self.touches, self.reached = [], None
        self.base, self.base_rows, self.t_down = None, [], None
        self.hist = deque()
        self.n_on, self.why = 0, ""
        self.up_until = self.z_up = None
        self.px = deque(maxlen=3)                       # the fingers' highest proximity, last readings
        self.free_stops = 0


class Descent(object):
    """Both hands down at once, each on its own: until it touches (confirmed) or its lowest point reaches z_stop."""

    def __init__(self, c, rob, gfit, hands, q11, h, a, log):
        self.c, self.rob, self.gfit, self.hands, self.q11, self.h, self.a, self.log = c, rob, gfit, hands, q11, h, a, log
        self.t0 = time.time()

    def force(self, st, T, side, p):
        """Force of the world on point p of a hand (N, pelvis), from the arm's joint lag minus gravity (pk.fit)."""
        s = sl(side)
        q, sent, dq = np.array(st["q"]), np.array(st["sent"]), np.array(st["dq"])
        kp, kd = np.array(st["kp"]), np.array(st["kd"])
        tau = kp[s] * (sent[s] - q[s]) - kd[s] * dq[s]
        tau_g = np.asarray(self.gfit["scale"])[s] * self.rob.gravity(T, side) + np.asarray(self.gfit["bias_rad"])[s] * config.KP[s]
        return -force_pinv(self.rob.jacobian(T, side, p), tau - tau_g)

    @staticmethod
    def pads(st, side):
        t = st.get("touch")
        if not t:
            return None
        return np.array(t[0 if side == "left" else 1], float)                 # 5 fingers x (normal, tangential, dir, prox)

    def step_ik(self, hd):
        q5, info = hd.flat.solve(hd.q5, np.r_[hd.xy, hd.z_cmd], iters=15)
        if info["pos_mm"] > 2.0:
            return False
        hd.q5 = q5
        self.q11[sl(hd.side)] = q5
        return True

    def run(self, z_stop, label):
        """z_stop: {side: lowest-point height at which a hand that has not touched stops}. Each hand ends 'touched',
        'reached' (z_stop in free air), or 'stopped' (why)."""
        a = self.a
        for hd in self.hands.values():
            if hd.state in ("idle", "reached"):
                hd.state, hd.t_down = "down", time.time()
                hd.hist.clear()
        t_prev = time.time()
        while any(hd.state in ("down", "up") for hd in self.hands.values()):
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = self.c.state()
            T = self.rob.fk(np.array(st["q"]), np.array(st["hands"]))
            row = [round(now - self.t0, 3), label]
            for side in SIDES:
                hd = self.hands[side]
                z_low, p_low, ln = hd.pts.lowest(T)
                M = T[hd.flat.link]
                z_pt = float((M[:3, :3] @ hd.flat.p_local + M[:3, 3])[2])
                lead = z_pt - hd.z_cmd                                         # the command this far below the hand
                F = self.force(st, T, side, p_low)
                pd = self.pads(st, side)
                row += [hd.state, round(hd.z_cmd, 5), round(z_pt, 5), round(z_low, 5), round(lead, 5)] + [round(v, 2) for v in F]
                row += ([int(v) for v in pd[:, 0]] + [int(v) for v in pd[:, 3]]) if pd is not None else [""] * 10
                if hd.state == "up":
                    if now >= hd.up_until:
                        hd.state, hd.t_down = "down", now
                        hd.hist.clear()
                    continue
                if hd.state != "down":
                    continue
                if hd.base is None:                    # baseline WHILE moving (joint friction flips with the motion)
                    hd.base_rows.append((lead, F[2], pd[:, 0].copy() if pd is not None else None))
                    if now - hd.t_down > a.baseline_s:
                        rows = hd.base_rows[len(hd.base_rows) // 3:]
                        pads = [r[2] for r in rows if r[2] is not None]
                        hd.base = dict(lead=float(np.mean([r[0] for r in rows])), fz=float(np.mean([r[1] for r in rows])),
                                       pads=np.median(np.array(pads), axis=0) if pads else None)
                        self.log("baseline", side=side, lead_mm=1000 * hd.base["lead"], fz=hd.base["fz"])
                hd.z_cmd -= a.speed / 1000 * dt
                lead0 = hd.base["lead"] if hd.base else 0.0
                # proximity (when the hand server sends it): nothing near -> the command may run --lead-free ahead to
                # break the joints' friction; a surface near -> only --lead
                if pd is not None:
                    hd.px.append(float(np.max(pd[:, 3])))
                prox = float(np.median(hd.px)) if hd.px else None
                near = prox is None or prox >= a.prox_near
                free = min(a.lead_free + 4.0 * hd.free_stops, a.lead_free_max)        # more after each stop in free air
                hd.z_cmd = max(hd.z_cmd, z_pt - lead0 - (a.lead if near else free) / 1000)
                if not self.step_ik(hd):
                    hd.state, hd.why = "stopped", "out of reach at %.0f mm" % (1000 * hd.z_cmd)
                    continue
                if float(np.linalg.norm(F)) - abs(hd.base["fz"] if hd.base else 0.0) > a.f_abort:
                    hd.state, hd.why = "stopped", "force %.1f N > %.0f N" % (float(np.linalg.norm(F)), a.f_abort)
                    hd.z_cmd = z_pt + 0.01
                    self.step_ik(hd)
                    continue
                if hd.base is None:
                    continue
                if z_low <= z_stop[side]:
                    hd.state, hd.reached = "reached", z_low
                    hd.z_cmd = z_pt - lead0                                      # hold where it is
                    self.step_ik(hd)
                    continue
                lead_rel, fz = lead - hd.base["lead"], F[2] - hd.base["fz"]
                if now < getattr(hd, "quiet_until", 0.0):
                    continue
                hd.hist.append((now, z_low, lead_rel))
                while hd.hist and hd.hist[0][0] < now - a.stall_s:
                    hd.hist.popleft()
                by = []
                if fz > a.f_on and lead_rel > a.lead_min / 1000:
                    by.append("force")
                if pd is not None and hd.base["pads"] is not None and np.max(pd[:, 0] - hd.base["pads"]) > a.touch_on:
                    by.append("pad")
                hd.n_on = hd.n_on + 1 if by else 0
                zs = [v[1] for v in hd.hist]
                stall = (len(hd.hist) > 5 and now - hd.hist[0][0] > 0.9 * a.stall_s and max(zs) - min(zs) < a.stall_mm / 1000
                         and min(v[2] for v in hd.hist) > a.lead_min / 1000)
                if (stall or hd.n_on >= 3) and prox is not None and prox < a.prox_touch:
                    # the hand stopped, but no finger is at the table: friction in free air, not a touch -- go on
                    hd.free_stops += 1
                    hd.free_z = (getattr(hd, "free_z", []) + [z_low])[-3:]
                    self.log("stop in free air", side=side, lowest_z=z_low, proximity=prox, force_up_N=fz, n=hd.free_stops,
                             note="not a touch (no finger near the table): going on down")
                    hd.hist.clear()
                    hd.n_on = 0
                    hd.z_cmd = z_pt - lead0 - min(a.lead_free + 4.0 * hd.free_stops, a.lead_free_max) / 1000    # press harder now
                    self.step_ik(hd)
                    hd.quiet_until = now + 0.8                   # let the arm take the new command before judging again
                    if (len(hd.free_z) == 3 and max(hd.free_z) - min(hd.free_z) < 0.003
                            and a.lead_free + 4.0 * (hd.free_stops - 1) >= a.lead_free_max):
                        hd.state, hd.why = "stopped", ("stopped 3x at %.0f mm with no finger near the table (proximity %.0f): something "
                                                       "else touches (palm, wrist, cable?) or the arm is stuck" % (1000 * z_low, prox))
                        hd.z_cmd = z_pt - lead0 + 0.01
                        self.step_ik(hd)
                elif stall or hd.n_on >= 3:
                    how = "+".join(by + (["stall"] if stall else []))
                    hd.touches.append(dict(z=float(z_low), point=p_low.tolist(), link=ln, by=how, fz=float(fz), proximity=prox,
                                           q=[float(v) for v in st["q"]],
                                           pads=None if pd is None else pd[:, 0].tolist(),
                                           prox=None if pd is None else pd[:, 3].tolist()))
                    self.log("touch", side=side, n=len(hd.touches), lowest_z=z_low, by=how, force_up_N=fz, at=p_low, link=ln)
                    zt = [t_["z"] for t_ in hd.touches]
                    if len(zt) >= 2 and min(abs(zt[-1] - v) for v in zt[:-1]) <= a.agree_mm / 1000:
                        hd.state = "touched"
                        hd.z_cmd = z_pt - lead0 + 0.004                             # off the table
                        self.step_ik(hd)
                        continue
                    if len(zt) >= a.tries:
                        hd.state, hd.why = "stopped", "the touches did not repeat: %s mm" % [round(1000 * v, 1) for v in zt]
                        hd.z_cmd = z_pt - lead0 + 0.01
                        self.step_ik(hd)
                        continue
                    hd.state, hd.up_until = "up", now + 0.9                        # up 5 mm and down again
                    hd.z_cmd = z_pt - lead0 + 0.005
                    hd.n_on = 0
                    self.step_ik(hd)
            self.c.set(q=self.q11)
            if self.trace is not None:
                self.trace.write(",".join(str(v) for v in row) + "\n")
            self.c.hold(0.02)

    trace = None

    def move_z(self, targets, T_s):
        """Both hands straight up or down to targets {side: little-finger pad z} over T_s (free air)."""
        z0 = {s: self.hands[s].z_cmd for s in SIDES}
        t0 = time.time()
        while True:
            u = min(1.0, (time.time() - t0) / max(T_s, 1e-3))
            u_s = u * u * (3 - 2 * u)
            for s in SIDES:
                hd = self.hands[s]
                hd.z_cmd = z0[s] + u_s * (targets[s] - z0[s])
                if not self.step_ik(hd):
                    raise RuntimeError("%s hand: the IK cannot follow to %.3f" % (s, hd.z_cmd))
            self.c.set(q=self.q11)
            self.c.hold(0.02)
            if u >= 1.0:
                return


# ============================================================================ main
def main(argv=None, cam=None, kin=None, open_camera=None):
    global TILT_DEG
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", nargs="*", default=["touch"], help="pose | touch | sweep (touch sweep: both) | grid")
    ap.add_argument("--dry", action="store_true", help="the plan only, nothing moves")
    ap.add_argument("--x", type=float, default=0.38, help="little-finger pad, pelvis x (m)")
    ap.add_argument("--y", type=float, default=0.22, help="... +-y (m): left hand +y, right hand -y")
    ap.add_argument("--tilt", type=float, default=TILT_DEG, help="fingers this much below horizontal, deg (the palm >= 25 mm above the tips at 15)")
    ap.add_argument("--hover", type=float, default=0.12, help="the pose: the hands' lowest point at this pelvis z (m); the table is ~0.03")
    ap.add_argument("--head", type=float, nargs=2, default=None, metavar=("PITCH", "YAW"),
                    help="the head for the camera's measurement (rad; default: aimed between the hands)")
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--speed", type=float, default=4.0, help="down, mm/s")
    ap.add_argument("--above", type=float, default=40.0, help="the descent starts this far above the camera's table, mm")
    ap.add_argument("--below", type=float, default=40.0, help="a hand goes at most this far below the camera's table, mm")
    ap.add_argument("--lead", type=float, default=8.0, help="the command never more than this below the hand, mm")
    ap.add_argument("--lead-min", type=float, default=3.0, help="a stall counts only with the command this far below, mm")
    ap.add_argument("--lead-free", type=float, default=14.0, help="... while no finger's proximity says a surface is near, mm")
    ap.add_argument("--lead-free-max", type=float, default=18.0, help="... raised by 4 mm after each stop in free air, up to this, mm")
    ap.add_argument("--prox-near", type=float, default=20000, help="proximity (raw counts, the fingers' highest): a surface within a few mm")
    ap.add_argument("--prox-touch", type=float, default=150000, help="proximity a touch needs (when the hand server sends it); "
                    "the table on 2026-10-22: 2.5-4.6e5 at contact, ~1e4 at ~5 mm")
    ap.add_argument("--stall-s", type=float, default=0.6)
    ap.add_argument("--stall-mm", type=float, default=1.0)
    ap.add_argument("--f-on", type=float, default=3.0, help="contact: upward force from the joint lag, N")
    ap.add_argument("--touch-on", type=float, default=120.0, help="contact: a fingertip pad's normal force, counts")
    ap.add_argument("--f-abort", type=float, default=15.0)
    ap.add_argument("--agree-mm", type=float, default=2.0)
    ap.add_argument("--tries", type=int, default=3)
    ap.add_argument("--baseline-s", type=float, default=0.6)
    ap.add_argument("--stop-at-camera", action="store_true", help="test 2 only: do not go on below the camera's table")
    ap.add_argument("--ref", default=None, help="sweep: the touch run's result.json (default: the newest)")
    ap.add_argument("--perception", default=config.PERCEPTION_DIR)
    ap.add_argument("--port", type=int, default=config.SERVER_PORT)
    a = ap.parse_args(argv)
    for w in a.what:
        if w not in ("pose", "touch", "sweep", "grid"):
            ap.error("unknown: %s" % w)
    TILT_DEG = a.tilt
    rob = Robot()
    print("arm joint offsets (%s): %s" % (
        "DEXPICK_ARM_OFFSETS=" + config.ARM_OFFSETS_ON if rob.d11.any() else "OFF, raw URDF",
        "  ".join("%s %+.2f" % (n, v) for n, v in zip(config.ARM_NAMES, np.degrees(rob.d11)) if v) or "-"))
    home = json.load(open(os.path.join(config.ROOT, "params", "premotion.json")))
    home_q, h = np.array(home["q"], float), np.zeros(12)               # fingers straight
    q_pose, flats, infos = plan(rob, home_q, h, a.x, a.y, a.hover)
    pts = {s: HandPoints(rob, q_pose, h, s) for s in SIDES}
    print("the pose (IK, mirrored targets): little-finger pads at (%.3f, +-%.3f), lowest hand points at z %.3f" % (a.x, a.y, a.hover))
    for s in SIDES:
        i = infos[s]
        print("  %-5s IK %.1f mm, fingers %.1f deg down, roll %.1f deg, heading %+.1f deg, palm %s; lowest: %s; joints (deg) %s" % (
            s, i["pos_mm"], i["tilt_deg"], i["roll_deg"], i["heading_deg"], "down" if i["palm_down"] else "NOT DOWN", i["lowest_link"],
            np.round(np.degrees(q_pose[sl(s)]), 1)))
    print("  right - mirror(left), deg: %s" % np.round(symmetry(q_pose), 2))
    if max(infos[s]["pos_mm"] for s in SIDES) > 2.0 or not all(infos[s]["palm_down"] for s in SIDES):
        print("the pose is out of reach: try other --x --y --hover")
        return 1
    from calib_cam import path_clear, via_up
    for s in SIDES:
        q = home_q.copy()
        q[sl(s)] = q_pose[sl(s)]
        print("  %-5s from the pre-grip pose: %s" % (s, "straight" if path_clear(rob, home_q, q, h, s, 0.08) else "via a point above"))
    if a.dry:
        return 0

    import perception as per
    from pk.client import Client
    out = os.path.join(config.LOG_DIR, "table", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    events = open(os.path.join(out, "events.jsonl"), "a")
    t_start = time.time()

    def log(what, **kw):
        kw = {k: (np.round(np.asarray(v, float), 5).tolist() if isinstance(v, (np.ndarray, list, tuple)) and k != "by" else v) for k, v in kw.items()}
        events.write(json.dumps(dict(t=round(time.time() - t_start, 3), what=what, **kw), default=str) + "\n")
        events.flush()
        print("[%6.1f s] %s: %s" % (time.time() - t_start, what, ", ".join("%s %s" % (k, v) for k, v in kw.items())), flush=True)

    try:
        gfit = json.load(open(os.path.join(config.MODEL_DIR, "gravity_fit.json")))
    except (OSError, ValueError):
        gfit = dict(scale=[1.0] * config.N_ARM, bias_rad=[0.0] * config.N_ARM)
    kin = kin or per.Kin(a.perception, correct=False)               # R1_Plate_Perception's calibration: what is checked
    if cam is None and ("touch" in a.what or "sweep" in a.what or "grid" in a.what):
        # BEFORE taking the arms: the RealSense takes seconds to start, and a client silent for 1 s loses them
        cam = (open_camera or per.Camera)()
    ref = load_ref(a.ref) if "sweep" in a.what and "touch" not in a.what else None
    c = Client("table check", port=a.port)
    why = c.to_pregrip()                     # HOME is the neutral pose; this tool was planned from the pre-grip pose
    if why:
        print("could not get to the pre-grip pose: %s" % why)
        c.bye(then="path")
        return 1
    hc = Client("table check head", role="head", port=a.port)
    result = dict(args=vars(a), started=time.strftime("%Y-%m-%d %H:%M:%S"), pose=dict(q=q_pose.tolist(), symmetry_deg=symmetry(q_pose).tolist(),
                  info={s: {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in infos[s].items()} for s in SIDES}))
    print("logs: %s" % out)
    try:
        # 1. the pose: straight there, or each arm first to a point above where its hand is
        q_now = np.array(c.state()["des"])
        q_mid = q_now.copy()
        for s in SIDES:
            q = q_now.copy()
            q[sl(s)] = q_pose[sl(s)]
            if not path_clear(rob, q_now, q, h, s, 0.08):
                v = via_up(rob, q_now, q, h, s, rise=0.06)
                if v is None:
                    raise RuntimeError("no safe way to the pose for the %s arm" % s)
                q_mid[sl(s)] = v[sl(s)]
            else:
                q_mid[sl(s)] = q_pose[sl(s)]
        q_pose[config.WAIST] = q_now[config.WAIST]
        q_mid[config.WAIST] = q_now[config.WAIST]
        for qw in ([q_mid] if np.any(q_mid != q_pose) else []) + [q_pose]:
            why = c.move_to(qw, h, v=0.25, tag="table pose")
            if why:
                raise RuntimeError("move stopped: %s" % why)
        c.hold(1.5)
        c.mark()                                    # the rewind home will start from here (back_to_pose at the end)
        print("\n1. THE POSE, both hands by IK from mirrored targets:")
        report_pose(rob, c.state(), pts)
        log("pose", state_q=c.state()["q"])
        hands = {s: Hand(s, flats[s], q_pose[sl(s)], infos[s]["P"], pts[s]) for s in SIDES}
        q11 = q_pose.copy()
        if "pose" in a.what:
            done = threading.Event()
            threading.Thread(target=lambda: (sys.stdin.readline(), done.set()), daemon=True).start()
            print("holding: look at the two little fingers (same height?). Enter: home", flush=True)
            while not done.is_set():
                c.hold(0.1)
        if "touch" in a.what:
            touch(a, c, hc, rob, kin, cam, gfit, hands, q11, h, out, log, result)
            json.dump(result, open(os.path.join(out, "result.json"), "w"), indent=1, default=str)
        if "sweep" in a.what:
            ref = result if "touch" in a.what else ref
            sweep(a, c, hc, kin, cam, ref, out, log, result)
            json.dump(result.get("sweep"), open(os.path.join(out, "sweep.json"), "w"), indent=1, default=str)
        if "grid" in a.what:
            grid(a, c, hc, rob, kin, cam, gfit, h, out, log, result)
        back_to_pose(c, rob, h, q_pose, log)
    except KeyboardInterrupt:
        print("\nstopped: the arms go home along their path")
    finally:
        try:
            hc.head(hc.head_look)
            time.sleep(0.5)
        except Exception:                                   # noqa: BLE001
            pass
        c.bye()
        hc.close()
        events.close()
    print("logs: %s" % out)
    return 0


def back_to_pose(c, rob, h, q_pose, log):
    """Up and back to the marked pose, then trim the server's path there: the rewind home at bye() retraces only the
    way out, not every touch."""
    from calib_cam import via_up
    q_now = np.array(c.state()["des"])
    if np.max(np.abs(q_now - q_pose)[:10]) < 0.004:
        c.trim()
        return
    q_up = q_now.copy()
    for s in SIDES:
        v = via_up(rob, q_now, q_pose, h, s, rise=0.03)
        if v is not None:
            q_up[sl(s)] = v[sl(s)]
    for qw in (q_up, q_pose):
        why = c.move_to(qw, h, v=0.2, tag="table back")
        if why:
            log("note", what="could not get back to the pose (%s): the rewind retraces everything" % why)
            return
    c.hold(0.3)
    c.trim()
    time.sleep(0.1)


def trace_cols():
    cols = ["t", "phase"]
    for s in SIDES:
        cols += ["%s_%s" % (s, k) for k in ("state", "z_cmd", "z_pad", "z_low", "lead", "fx", "fy", "fz")]
        cols += ["%s_normal_%s" % (s, f) for f in config.TOUCH_FINGERS] + ["%s_prox_%s" % (s, f) for f in config.TOUCH_FINGERS]
    return ",".join(cols) + "\n"


# the touch grid: little-finger pad targets (x, +-y, m) and finger tilts (deg), a snake through them; the last repeats the
# middle (a check); joints vary: shoulder pitch -48..+5, roll 47..76, yaw -68..-11, elbow -20..+21, wrist 12..38 deg
SPOTS = [(0.34, 0.16, 10.0), (0.34, 0.28, 10.0), (0.43, 0.28, 10.0), (0.43, 0.16, 10.0),
         (0.43, 0.16, 20.0), (0.43, 0.28, 20.0), (0.34, 0.28, 20.0), (0.34, 0.16, 20.0), (0.38, 0.22, 15.0)]


def grid(a, c, hc, rob, kin, cam, gfit, h, out, log, result):
    """Both hands touch the table at mirrored spots and tilts (SPOTS), each touch confirmed and seen by the fingertips'
    proximity; the joints at every touch and the camera's table there go to grid.json for calibration/arm_offsets_fit.py."""
    global TILT_DEG
    from calib_cam import path_clear
    st = c.state()
    tgt, _ = kin.aim(np.array([0.40, 0.0, 0.03]), kin.angles(st))
    if not wait_head(hc, tgt, c):
        raise RuntimeError("the head did not settle")
    c.hold(0.5)
    tab = camera_table(cam, kin, c, a.frames, out, "grid_table")
    if tab is None:
        raise RuntimeError("the camera found no table")
    log("grid camera table", head_deg=np.degrees(tab["head"]), z_at_middle=z_at(tab, [0.40, 0.0]),
        tilt_left_down_deg=tab["tilt_left_down_deg"], tilt_front_down_deg=tab["tilt_front_down_deg"])
    trace = open(os.path.join(out, "grid_trace.csv"), "w")
    trace.write(trace_cols())
    rows = []
    q_now = np.array(c.state()["des"])
    print("\nGRID: %d spots, both hands at once (mirrored)" % len(SPOTS))
    for k, (x, y, tilt) in enumerate(SPOTS):
        TILT_DEG = tilt
        z_hi = z_at(tab, [x, y]) + 0.07
        q_hi, flats, infos = plan(rob, q_now, h, x, y, z_hi)
        q_hi[config.WAIST] = q_now[config.WAIST]
        if max(infos[s]["pos_mm"] for s in SIDES) > 2.0 or not all(infos[s]["palm_down"] for s in SIDES):
            log("grid", spot=k, note="out of reach: skipped")
            continue
        if not all(path_clear(rob, q_now, q_hi, h, s, z_at(tab, [x, y]) + 0.035) for s in SIDES):
            log("grid", spot=k, note="the way there passes low: skipped")
            continue
        why = c.move_to(q_hi, h, v=0.2, tag="grid %d" % k)
        if why:
            raise RuntimeError("move stopped: %s" % why)
        q_now = q_hi.copy()
        pts = {s: HandPoints(rob, q_hi, h, s) for s in SIDES}
        hands = {s: Hand(s, flats[s], q_hi[sl(s)], infos[s]["P"], pts[s]) for s in SIDES}
        dsc = Descent(c, rob, gfit, hands, q_hi.copy(), h, a, log)
        dsc.trace = trace
        st = c.state()
        T = rob.fk(np.array(st["q"]), np.array(st["hands"]))
        lows = {s: pts[s].lowest(T) for s in SIDES}
        zc = {s: z_at(tab, lows[s][1][:2]) for s in SIDES}
        dsc.move_z({s: hands[s].z_cmd + (zc[s] + 0.025) - lows[s][0] for s in SIDES}, 2.0)
        c.hold(0.4)
        log("grid", spot=k, x=x, y=y, tilt=tilt, note="down")
        dsc.run({s: zc[s] - a.below / 1000 for s in SIDES}, "grid %d" % k)
        for s in SIDES:
            hd = hands[s]
            row = dict(spot=k, x=x, y=y if s == "left" else -y, tilt=tilt, side=s, state=hd.state, why=hd.why, touches=hd.touches,
                       cand=[(ln, v.tolist()) for ln, v in pts[s].cand])
            if hd.state == "touched":
                z = touched_z(hd)
                zc_t = z_at(tab, hd.touches[-1]["point"][:2])
                row.update(touched_z=z, camera_z=zc_t, camera_minus_touched_mm=1000 * (zc_t - z))
                log("grid touch", spot=k, side=s, touched_z=z, camera_z=zc_t, camera_minus_touched_mm=1000 * (zc_t - z))
            else:
                log("grid touch", spot=k, side=s, note="no touch: %s" % (hd.why or hd.state))
            rows.append(row)
        dsc.move_z({s: hands[s].z_cmd + 0.03 for s in SIDES}, 1.5)
        q_now = dsc.q11.copy()
        result["grid"] = dict(table=tab, rows=rows, arm_offsets_rad=rob.d11.tolist())
        json.dump(result["grid"], open(os.path.join(out, "grid.json"), "w"), indent=1, default=str)
    trace.close()
    for s in SIDES:
        e = [r["camera_minus_touched_mm"] for r in rows if r["side"] == s and "camera_minus_touched_mm" in r]
        if e:
            log("GRID", side=s, touches=len(e), camera_minus_touched_mm=np.round(e, 1),
                note="median %+.1f mm, spread %.1f mm (0 everywhere: the arm's joints are right)" % (np.median(e), np.std(e)))


def touch(a, c, hc, rob, kin, cam, gfit, hands, q11, h, out, log, result):
    # 2. the camera's table, the head aimed between the hands (or where --head says)
    st = c.state()
    if a.head is not None:
        tgt = np.array(a.head, float)
    else:
        tgt, _ = kin.aim(np.array([a.x + 0.05, 0.0, 0.03]), kin.angles(st))
    if not wait_head(hc, tgt, c):
        raise RuntimeError("the head did not settle")
    c.hold(0.5)
    tab = camera_table(cam, kin, c, a.frames, out, "camera_table")
    if tab is None:
        raise RuntimeError("the camera found no table under the hands")
    st = c.state()
    T = rob.fk(np.array(st["q"]), np.array(st["hands"]))
    low_xy = {s: hands[s].pts.lowest(T)[1][:2] for s in SIDES}                # where each hand will meet the table
    zc = {s: z_at(tab, low_xy[s]) for s in SIDES}
    log("camera table", head_deg=np.degrees(tab["head"]), z_left=zc["left"], z_right=zc["right"],
        tilt_left_down_deg=tab["tilt_left_down_deg"], tilt_front_down_deg=tab["tilt_front_down_deg"], points=tab["n"], spread_mm=tab["spread_mm"])
    result["camera"] = dict(tab, z_left=zc["left"], z_right=zc["right"], at_left=low_xy["left"].tolist(), at_right=low_xy["right"].tolist())
    if not all(-0.05 < z < 0.10 for z in zc.values()):
        raise RuntimeError("the camera's table is at %.3f / %.3f m: not where a table can be" % (zc["left"], zc["right"]))
    dsc = Descent(c, rob, gfit, hands, q11, h, a, log)
    dsc.trace = open(os.path.join(out, "trace.csv"), "w")
    dsc.trace.write(trace_cols())
    # down at speed to --above over the camera's table, then slowly
    T = rob.fk(np.array(c.state()["q"]), np.array(c.state()["hands"]))
    tgt_z = {}
    for s in SIDES:
        hd = hands[s]
        z_low = hd.pts.lowest(T)[0]
        tgt_z[s] = hd.z_cmd + (zc[s] + a.above / 1000) - z_low
    if any(tgt_z[s] > hands[s].z_cmd + 0.002 for s in SIDES):
        log("note", what="the camera's table is high: the descent starts from the pose")
    else:
        dsc.move_z(tgt_z, 2.5)
        c.hold(0.6)
    print("\n2. DOWN TO THE CAMERA'S TABLE (each hand stops on contact), %.0f mm/s" % a.speed)
    dsc.run(zc, "to camera")
    res = {}
    for s in SIDES:
        hd = hands[s]
        if hd.state == "touched":
            z = touched_z(hd)
            res[s] = dict(phase2="touched %.1f mm ABOVE the camera's table" % (1000 * (z - zc[s])))
        elif hd.state == "reached":
            res[s] = dict(phase2="reached the camera's table height (%.4f) without touching" % hd.reached)
        else:
            res[s] = dict(phase2="stopped: %s" % hd.why)
        log("test 2", side=s, result=res[s]["phase2"])
    if not a.stop_at_camera and any(hands[s].state == "reached" for s in SIDES):
        c.hold(1.5)
        print("\n3. ON DOWN until each hand finds the table (at most %.0f mm below the camera's)" % a.below)
        dsc.run({s: zc[s] - a.below / 1000 for s in SIDES}, "below camera")
    for s in SIDES:
        hd = hands[s]
        r = res[s]
        r.update(camera_z=zc[s], state=hd.state, why=hd.why, touches=hd.touches)
        if hd.state == "touched":
            z = touched_z(hd)
            zc_t = z_at(tab, hd.touches[-1]["point"][:2])                       # the camera's table right where it touched
            r.update(touched_z=z, camera_z=zc_t, camera_minus_touched_mm=1000 * (zc_t - z))
            log("RESULT", side=s, camera_table_z=zc_t, touched_z=z, at=hd.touches[-1]["point"],
                note="the camera puts the table %.1f mm %s than the %s arm finds it" % (abs(1000 * (zc_t - z)), "HIGHER" if zc_t > z else "LOWER", s))
        else:
            log("RESULT", side=s, note="no table found: %s" % (hd.why or hd.state))
    if all("touched_z" in res[s] for s in SIDES):
        dz_arm = res["left"]["touched_z"] - res["right"]["touched_z"]
        dz_cam = res["left"]["camera_z"] - res["right"]["camera_z"]
        log("left - right", arms_mm=1000 * dz_arm, camera_mm=1000 * dz_cam,
            note="the table's level across, by the arms vs by the camera (a camera rolled about its view shows up here)")
    result["touch"] = res
    # up off the table, back to the pose's height
    dsc.move_z({s: hands[s].z_cmd + 0.03 for s in SIDES}, 2.0)
    dsc.move_z({s: max(hands[s].z0, hands[s].z_cmd) for s in SIDES}, 2.0)
    dsc.trace.close()


def touched_z(hd):
    zt = sorted(t_["z"] for t_ in hd.touches)
    best = min(range(len(zt) - 1), key=lambda i: zt[i + 1] - zt[i])
    return 0.5 * (zt[best] + zt[best + 1])


def load_ref(path):
    if path is None:
        cands = sorted(glob.glob(os.path.join(config.LOG_DIR, "table", "*", "result.json")))
        cands = [p for p in cands if "touched_z" in json.dumps(json.load(open(p)).get("touch", {}))]
        if not cands:
            raise RuntimeError("no touch run with both hands' touches in logs/table: run `table_check.py touch` first")
        path = cands[-1]
    print("the table as the hands touched it: %s" % path)
    return json.load(open(path))


def sweep(a, c, hc, kin, cam, ref, out, log, result):
    """4. The head through pitch x yaw (each pose reached from below and from above), the camera's table at the two
    places the hands touched vs where they touched."""
    T_ = ref["touch"]
    pts = {s: (np.array(T_[s]["touches"][-1]["point"][:2], float), T_[s]["touched_z"]) for s in SIDES if "touched_z" in T_[s]}
    if not pts:
        raise RuntimeError("the reference has no touches")
    st = c.state()
    mid = np.mean([p for p, _ in pts.values()], axis=0)
    (p0, y0), _ = kin.aim(np.array([mid[0] + 0.05, 0.0, 0.03]), kin.angles(st))
    p_hi = config.HEAD_HI[0] - 0.03
    pitches = sorted(set(round(float(min(p0 + d, p_hi)), 3) for d in (-0.2, -0.1, 0.0, 0.1)))
    yaws = [round(float(y0 + d), 3) for d in (-0.35, -0.175, 0.0, 0.175, 0.35)]
    poses = []
    for p in pitches[1:]:
        poses += [(p, y, "yaw up") for y in [yaws[0] - 0.1] + yaws] + [(p, y, "yaw down") for y in [yaws[-1] + 0.1] + yaws[::-1]]
    poses += [(p, y0, "pitch up") for p in [pitches[0] - 0.08] + pitches] + [(p, y0, "pitch down") for p in [min(pitches[-1] + 0.08, p_hi)] + pitches[::-1]]
    print("\n4. HEAD SWEEP: %d head poses (pitch %s, yaw %s rad); the table where the hands touched: %s" % (
        len(poses), pitches, yaws, {s: round(1000 * z, 1) for s, (_, z) in pts.items()}))
    rows = []
    for k, (p, y, how) in enumerate(poses):
        first = (k == 0) or (poses[k - 1][2] != how) or (how.startswith("yaw") and poses[k - 1][0] != p)
        if not wait_head(hc, np.array([p, y]), c):
            print("  head did not settle at %.2f %.2f: skipped" % (p, y))
            continue
        if first:                                       # the run-up pose: only sets the direction of approach
            continue
        c.hold(0.4)
        tab = camera_table(cam, kin, c, 2, out, "sweep_%02d" % k, seed=k)
        if tab is None:
            print("  pitch %.2f yaw %+.2f: no table" % (p, y))
            continue
        err = {s: 1000 * (z_at(tab, xy) - z) for s, (xy, z) in pts.items()}
        rows.append(dict(pitch=tab["head"][0], yaw=tab["head"][1], cmd=[p, y], approach=how, err_mm=err,
                         tilt_left_down_deg=tab["tilt_left_down_deg"], tilt_front_down_deg=tab["tilt_front_down_deg"]))
        print("  %-10s head pitch %5.1f yaw %+6.1f deg: camera - touched  left %+6.1f  right %+6.1f mm   table tilt left-down %+.2f front-down %+.2f deg" % (
            how, np.degrees(tab["head"][0]), np.degrees(tab["head"][1]), err.get("left", np.nan), err.get("right", np.nan),
            tab["tilt_left_down_deg"], tab["tilt_front_down_deg"]), flush=True)
    result["sweep"] = dict(rows=rows, summary=sweep_summary(rows))
    for line in result["sweep"]["summary"]:
        log("sweep", note=line)


def sweep_summary(rows):
    out = []
    for s in SIDES:
        R = [r for r in rows if s in r["err_mm"]]
        if len(R) < 4:
            continue
        e = np.array([r["err_mm"][s] for r in R])
        P = np.c_[np.degrees([r["pitch"] for r in R]), np.degrees([r["yaw"] for r in R]), np.ones(len(R))]
        m, *_ = np.linalg.lstsq(P, e, rcond=None)
        out.append("%s: camera - touched %.1f mm on average (spread %.1f); %+.2f mm per degree of head pitch, %+.2f mm per degree of yaw" % (
            s, e.mean(), e.std(), m[0], m[1]))
        hyst = []
        for how in ("yaw", "pitch"):
            up = {tuple(r["cmd"]): r["err_mm"][s] for r in R if r["approach"] == how + " up"}
            dn = {tuple(r["cmd"]): r["err_mm"][s] for r in R if r["approach"] == how + " down"}
            d = [up[k] - dn[k] for k in up if k in dn]
            if d:
                hyst.append("%s: reached going up - going down %+.1f mm (%d poses)" % (how, float(np.mean(d)), len(d)))
        if hyst:
            out.append("%s, the same head pose from either side: %s" % (s, "; ".join(hyst)))
    if len(rows) >= 4:
        P = np.c_[np.degrees([r["pitch"] for r in rows]), np.degrees([r["yaw"] for r in rows]), np.ones(len(rows))]
        for key, what in (("tilt_left_down_deg", "left-down"), ("tilt_front_down_deg", "front-down")):
            v = np.array([r[key] for r in rows])
            m, *_ = np.linalg.lstsq(P, v, rcond=None)
            out.append("the table's %s tilt as the camera sees it: %+.2f deg on average (spread %.2f), %+.3f deg per degree of head pitch, "
                       "%+.3f per degree of yaw" % (what, v.mean(), v.std(), m[0], m[1]))
        out.append("(a level table should read 0 everywhere; a tilt that changes with yaw = a tilt below the yaw joint that the "
                   "calibration took out in the head's frame: it turns into the other tilt as the head turns)")
    return out


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
