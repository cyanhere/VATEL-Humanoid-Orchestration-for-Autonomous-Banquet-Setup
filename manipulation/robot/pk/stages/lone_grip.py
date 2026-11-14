"""Two-hand grip of a lone plate: both hands at once, mirrored, on the plate's two sides."""
import csv
import math
import os
import time

import numpy as np

from .. import config
from ..ik import ArmChain, _lm
from ..model import hand_points
from ..plate import TASK
from ..stage_base import Abort, smooth

SIDES = ("left", "right")
SL = {"left": config.LEFT, "right": config.RIGHT}
PLAN = ("high", "pre", "down", "inn", "sq", "lift")
GRIPPING = ("close", "squeeze", "close2", "lift", "hold")


def unit(v):
    return v / np.linalg.norm(v)


def frames(g, side):
    """Outward radial e, finger direction f and palm normal n of one hand (scripts/lone_grip.py frames())."""
    z = np.array([0.0, 0.0, 1.0])
    psi = math.radians(g["psi"])
    sg = -1.0 if side == "right" else 1.0
    e = np.array([-math.sin(psi), sg * math.cos(psi), 0.0])
    t = np.array([math.cos(psi), sg * math.sin(psi), 0.0])
    h, pi_, ro = (math.radians(g[k]) for k in ("h", "pitch", "roll"))
    fh = math.cos(h) * t - math.sin(h) * e
    f = unit(math.cos(pi_) * fh - math.sin(pi_) * z)
    hp = -math.sin(h) * t - math.cos(h) * e
    v = unit(np.cross(f, hp))
    v = v if v[2] >= 0 else -v
    n = unit(math.cos(ro) * hp + math.sin(ro) * v)
    return e, f, n


def aim_point(g, side):
    """The aim point in the palm frame (m): the genes are the RIGHT hand's; the left is mirrored (py -> -py)."""
    sg = 1.0 if side == "right" else -1.0
    return np.array([g["px"], sg * g["py"], g["pz"]]) / 1000.0


def targets(g, side, cxy, table_z, P, up=0.0):
    """Pelvis-frame targets of the aim point (scripts/lone_grip.py run()), plus `high` above `pre`; all raised by `up` m."""
    e, f, n = frames(g, side)
    z = np.array([0.0, 0.0, 1.0])
    rim = np.array([cxy[0], cxy[1], 0.0]) + TASK.R * e
    rim[2] = table_z + g["hz"] / 1000
    ey_ = np.array([0.0, np.sign(e[1]), 0.0])                 # the squeeze is straight sideways
    out, din, sq = P["out_mm"] / 1000, g["din"] / 1000, g["sq"] / 1000
    tg = dict(pre=rim + out * e + P["pre_up_mm"] / 1000 * z, down=rim + out * e, inn=rim - din * e, sq=rim - din * e - sq * ey_,
              lift=rim - din * e - (sq + g["lift_in"] / 1000) * ey_ + P["lift_mm"] / 1000 * z)
    tg["high"] = tg["pre"] + P["high_up_mm"] / 1000 * z
    return {k: v + up * z for k, v in tg.items()}, f, n


def palm_ik(ch, q0, P, loc, f, n, w=0.04):
    """Aim point (palm frame loc) to P; palm z axis toward f and x axis toward n, soft (weight w), as the simulation.
    Returns (q5 real joints, dict(pos_mm, f_deg, n_deg, margin_deg))."""
    P, loc = np.asarray(P, float), np.asarray(loc, float)

    def residual(q):
        T, fr = ch.fk(q)
        R = T[:3, :3]
        p = R @ loc + T[:3, 3]
        Jp = np.array([np.cross(z, p - o) for o, z in fr]).T
        Jz = np.array([np.cross(z, R[:, 2]) for o, z in fr]).T
        Jx = np.array([np.cross(z, R[:, 0]) for o, z in fr]).T
        return np.r_[p - P, w * (R[:, 2] - f), w * (R[:, 0] - n)], np.vstack([Jp, w * Jz, w * Jx])

    q, _ = _lm(residual, q0, ch.lo, ch.hi, iters=150)
    T, _ = ch.fk(q)
    R = T[:3, :3]
    ang = lambda a, b: math.degrees(math.acos(max(-1.0, min(1.0, float(a @ b)))))     # noqa: E731
    marg = math.degrees(float(np.min(np.minimum(q - ch.lo, ch.hi - q))))
    return q, dict(pos_mm=1000 * float(np.linalg.norm(R @ loc + T[:3, 3] - P)), f_deg=ang(R[:, 2], f), n_deg=ang(R[:, 0], n),
                   margin_deg=marg)


class LoneGrip(object):
    name = "lone_grip"

    def __init__(self, ctx, mode="full"):
        self.ctx = ctx
        self.mode = mode
        self.P = ctx.params("lone_grip")
        gname = self.P["grip"]
        if gname not in self.P["grips"]:
            raise Abort("lone_grip: no grip '%s' (have %s)" % (gname, sorted(self.P["grips"])))
        self.gname = gname
        self.G = self.P["grips"][gname]
        self.g = self.G["genes"]
        self.H = {k: np.array(v, float) for k, v in self.G["hand"].items()}
        self.d11 = np.asarray(ctx.rob.d11, float)
        self.csv = None
        self.phase = "start"
        self.q_mark = None
        self.corr = {s: np.zeros(5) for s in SIDES}
        self.reached = "start"
        self._lag_t = None
        self._log_t = 0.0
        self.notes = {}
        self.plan_now = None                     # the planned pose (no aim correction) at this instant, for the log

    # ================================================================== plan
    def plate_and_table(self, st, need_camera):
        """(cx, cy, table_z, source) in the pelvis frame. The camera's plate when need_camera, else params plate_xy and
        table_m minus the pelvis height from the legs."""
        ctx, P = self.ctx, self.P
        pelvis = ctx.pelvis(st)
        table_legs = P["table_m"] - pelvis
        if not need_camera:
            cx, cy = P["plate_xy"]
            return float(cx), float(cy), float(table_legs), "params plate_xy, table_m %.3f - pelvis %.3f" % (P["table_m"], pelvis)
        t_req = time.time()
        ctx.c.request("locate")
        ctx.log("perception", note="asked for a fresh locate")
        pl = ctx.wait_plate(timeout=20.0, need_located_after=t_req - 0.5)
        ctx.c.request("head_lock", on=True)
        cx, cy, rim_z = pl["center"]
        table_z = rim_z - TASK.H
        ctx.log("plate", center=pl["center"], n_plates=pl["located"].get("n_plates"), diameter=pl["located"].get("diameter"),
                table_z_camera=table_z, table_z_from_legs=table_legs, pelvis_from_legs=pelvis)
        if P["check_n_plates"] and pl["located"].get("n_plates") not in (None, 1):
            raise Abort("perception sees %s plates: the lone grip wants ONE plate on the table (miscounted? --set lone_grip.check_n_plates=false)" % pl["located"].get("n_plates"))
        if abs(table_z - table_legs) > P["table_z_tol_m"]:
            raise Abort("the camera's table top (rim %.3f - 22 mm = %.3f) is %.0f mm from table_m %.3f - pelvis %.3f: wrong table_m, "
                        "or a stack?" % (rim_z, table_z, 1000 * (table_z - table_legs), P["table_m"], pelvis))
        return float(cx), float(cy), float(table_z), "camera"

    def plan(self, st, cxy, table_z, up=0.0):
        """IK of every target for both hands from the current arm state (the waist as the robot has it). Sets self.q
        (motor joints per side per target), self.tg, self.loc; returns the worst IK position miss (mm)."""
        rob = self.ctx.rob
        self.q11 = np.array(st["des"], float)
        self.hands0 = np.array(st["hands_cmd"], float)
        self.cxy, self.table_z, self.up = cxy, table_z, up
        self.q, self.tg, self.loc, self.ikr, self.chain = {}, {}, {}, {}, {}
        worst = 0.0
        for s in SIDES:
            ch = ArmChain(rob, s, "%s_base_link" % s, self.q11, None)
            self.chain[s] = ch
            tg, f, n = targets(self.g, s, cxy, table_z, self.P, up)
            loc = aim_point(self.g, s)
            ref = self.G["ref_q_rad"][s]
            self.tg[s], self.loc[s], self.q[s], self.ikr[s] = tg, loc, {}, {}
            prev = None
            for k in ("pre", "down", "inn", "sq", "lift", "high"):
                seeds = [np.array(ref["pre" if k == "high" else k], float)]
                if prev is not None:
                    seeds.append(prev)
                best = None
                for q0 in seeds:
                    q, info = palm_ik(ch, q0, tg[k], loc, f, n, self.P["ik_w_rot"])
                    c = info["pos_mm"] + 0.3 * (info["f_deg"] + info["n_deg"])
                    if best is None or c < best[0]:
                        best = (c, q, info)
                _, q, info = best
                self.q[s][k] = q - self.d11[SL[s]]                  # motor = real - offset
                self.ikr[s][k] = {a: round(b, 1) for a, b in info.items()}
                if k != "high":
                    prev = q
                worst = max(worst, info["pos_mm"])
        return worst

    def palm(self, s, q5_motor):
        """The aim point (pelvis frame) for a side's MOTOR joints."""
        T, _ = self.chain[s].fk(np.asarray(q5_motor, float) + self.d11[SL[s]])
        return T[:3, :3] @ self.loc[s] + T[:3, 3]

    def q11_at(self, k, corr=True):
        q = self.q11.copy()
        for s in SIDES:
            q[SL[s]] = self.q[s][k] + (self.corr[s] if corr else 0.0)
        return q

    def hands12(self, name):
        h = self.hands0.copy()
        v = np.zeros(6) if name == "open" else self.H[name]
        h[:6], h[6:12] = v, v
        return h

    def clearance(self, q_from, q_to, h12, n=15, edge_mm=10.0):
        """Lowest hand point (both hands) above the plate's rim top (mm) where it is over the plate or within edge_mm of
        its edge, over a joint smoothstep q_from -> q_to; +inf if never there."""
        rob = self.ctx.rob
        low = float("inf")
        top = self.table_z + self.up + TASK.H
        for u in np.linspace(0.0, 1.0, n):
            q = q_from + smooth(u) * (q_to - q_from)
            T = rob.fk(q, h12)
            for s in SIDES:
                for pts in hand_points(T, s).values():
                    r = np.hypot(pts[:, 0] - self.cxy[0], pts[:, 1] - self.cxy[1])
                    m = r < TASK.R + edge_mm / 1000
                    if m.any():
                        low = min(low, 1000 * float(pts[m, 2].min() - top))
        return low

    def report_plan(self, printer):
        for s in SIDES:
            for k in PLAN:
                t = self.tg[s][k]
                r = self.ikr[s][k]
                printer("  %-5s %-5s target (%.3f %+.3f %.3f): IK %.1f mm, fingers %.1f deg / palm %.1f deg off, %.0f deg from a limit%s" % (
                    s, k, t[0], t[1], t[2], r["pos_mm"], r["f_deg"], r["n_deg"], r["margin_deg"],
                    "" if r["pos_mm"] <= self.P["ik_max_mm"] else "  << OUT OF REACH"))

    # ================================================================== logging and checks
    def open_csv(self):
        cols = ["t", "phase"]
        for s in SIDES:
            cols += ["%s_q_des%d" % (s, i) for i in range(5)] + ["%s_q_meas%d" % (s, i) for i in range(5)]
            cols += ["%s_palm_cmd_%s" % (s, a) for a in "xyz"] + ["%s_palm_meas_%s" % (s, a) for a in "xyz"]
            cols += ["%s_palm_plan_%s" % (s, a) for a in "xyz"]
            cols += ["%s_corr%d" % (s, i) for i in range(5)]
        cols += ["hand_cmd%d" % i for i in range(12)] + ["hand_meas%d" % i for i in range(12)]
        cols += ["pad_%s_%s" % (s, f) for s in ("l", "r") for f in config.TOUCH_FINGERS]
        cols += ["cam_x", "cam_y", "cam_z"]
        self.csv_path = os.path.join(self.ctx.out_dir, "lone_grip_cmd.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols, extrasaction="ignore")
        self.csv.writeheader()
        self.t0 = time.time()

    def row(self, st):
        if self.csv is None:
            return
        now = time.time()
        if now - self._log_t < 1.0 / self.P["log_hz"]:
            return
        self._log_t = now
        r = dict(t=round(now - self.t0, 3), phase=self.phase)
        des, qm = np.array(st["des"], float), np.array(st["q"], float)
        for s in SIDES:
            for i in range(5):
                r["%s_q_des%d" % (s, i)] = round(float(des[SL[s]][i]), 5)
                r["%s_q_meas%d" % (s, i)] = round(float(qm[SL[s]][i]), 5)
                r["%s_corr%d" % (s, i)] = round(float(self.corr[s][i]), 5)
            for a, v in zip("xyz", self.palm(s, des[SL[s]])):
                r["%s_palm_cmd_%s" % (s, a)] = round(float(v), 5)
            for a, v in zip("xyz", self.palm(s, qm[SL[s]])):
                r["%s_palm_meas_%s" % (s, a)] = round(float(v), 5)
            if self.plan_now is not None:
                for a, v in zip("xyz", self.palm(s, self.plan_now[SL[s]])):
                    r["%s_palm_plan_%s" % (s, a)] = round(float(v), 5)
        for i in range(12):
            r["hand_cmd%d" % i] = round(float(st["hands_cmd"][i]), 4)
            r["hand_meas%d" % i] = round(float(st["hands"][i]), 4)
        t = st.get("touch")
        if t:
            for j, s in enumerate(("l", "r")):
                for i, f in enumerate(config.TOUCH_FINGERS):
                    r["pad_%s_%s" % (s, f)] = round(float(t[j][i][0]), 1)
        p = self.ctx.plate(st, max_age=0.5)
        if p and p.get("center"):
            r.update(cam_x=round(float(p["center"][0]), 4), cam_y=round(float(p["center"][1]), 4), cam_z=round(float(p["center"][2]), 4))
        self.csv.writerow(r)

    def check(self, st, k=None):
        """Every server cycle: the log row, and an arm held back from its command (lag > q_lag_max for q_lag_s)."""
        self.row(st)
        lags = np.abs(np.array(st["des"]) - np.array(st["q"]))[:10]
        lag = float(np.max(lags))
        now = time.time()
        if lag > self.P["q_lag_max_rad"]:
            self._lag_t = self._lag_t or now
            if now - self._lag_t > self.P["q_lag_s"]:
                return "%s is %.2f rad from its command for %.1f s (held back)" % (config.ARM_NAMES[int(np.argmax(lags))], lag, now - self._lag_t)
        else:
            self._lag_t = None
        return ""

    def palm_err(self, st, k):
        """Measured aim point minus the planned target `k`, mm, per side (FK of the measured joints)."""
        qm = np.array(st["q"], float)
        return {s: 1000 * (self.palm(s, qm[SL[s]]) - self.tg[s][k]) for s in SIDES}

    # ================================================================== motion
    def play(self, segs):
        """segs: [(to, hands12_to, seconds, tag)] from the last commanded pose; joint smoothsteps at 50 Hz. `to`: a plan key
        ("pre", "down",...: commanded with the aim correction), "now" (stay), "mark", or q11 (commanded as given)."""
        dt = 0.02
        Q, Qp, Hh, tags = [self.q_now.copy()], [self.p_now.copy()], [self.h_now.copy()], [segs[0][3]]
        for to, h_to, T, tag in segs:
            if isinstance(to, str) and to == "now":
                q_to, p_to = Q[-1], Qp[-1]
            elif isinstance(to, str) and to == "mark":
                q_to, p_to = self.q_mark, self.q_mark
            elif isinstance(to, str):
                q_to, p_to = self.q11_at(to), self.q11_at(to, corr=False)
            else:
                q_to, p_to = np.asarray(to, float), np.asarray(to, float)
            q_a, p_a, h_a = Q[-1], Qp[-1], Hh[-1]
            N = max(1, int(round(T / dt)))
            for i in range(1, N + 1):
                u = smooth(i / N)
                Q.append(q_a + u * (q_to - q_a))
                Qp.append(p_a + u * (p_to - p_a))
                Hh.append(h_a + u * (h_to - h_a))
                tags.append(tag)
        self.phase = tags[1] if len(tags) > 1 else tags[0]

        def chk(st, k):
            self.phase = tags[k]
            self.reached = tags[k]
            self.plan_now = Qp[k]
            return self.check(st)
        why, _ = self.ctx.play(np.array(Q), np.array(Hh), dt, check=chk, tags=["lone_grip " + t for t in tags], vmax=self.P["vmax_rad_s"])
        if why:
            raise Abort(why)
        self.q_now, self.p_now, self.h_now = Q[-1].copy(), Qp[-1].copy(), Hh[-1].copy()
        self.plan_now = self.p_now

    def settle(self, seconds, phase):
        self.phase = phase
        why = self.ctx.settle(seconds, lambda st: self.check(st))
        if why:
            raise Abort(why)

    def approach(self):
        """Hands shaped, joint move to `high` (clear of the plate), marked: the server's rewind home returns from here."""
        ctx, P = self.ctx, self.P
        q_high = self.q11_at("high", corr=False)
        h = self.hands12("pre")
        low = self.clearance(np.array(ctx.st()["des"], float), q_high, h)
        ctx.log("lone_grip approach", note="joint move to the high pose (the pre pose %.0f mm up)" % P["high_up_mm"],
                lowest_hand_point_above_rim_mm=low)
        if low < P["approach_clear_mm"]:
            raise Abort("the move to the high pose would pass %.0f mm above the rim over the plate" % low)
        self.phase = "approach"
        why = ctx.c.move_to(q_high, h, v=P["pre_v_rad_s"], tag="lone_grip approach", check=lambda st: self.check(st))
        if why:
            raise Abort("move to the high pose: %s" % why)
        ctx.c.set(vmax=P["vmax_rad_s"])
        self.settle(0.5, "approach")
        ctx.c.mark()
        self.q_mark, self.h_mark = q_high.copy(), h.copy()
        self.q_now, self.p_now, self.h_now = q_high.copy(), q_high.copy(), h.copy()
        self.plan_now = self.p_now

    def correct(self, key="down", n=None, label="correct"):
        """At plan pose `key`: settle, add (plan - measured) to the correction, command plan + correction; n times
        (default correct_n). Logs the palm error before / after each step. The correction stays on for every later target."""
        ctx, P = self.ctx, self.P
        out = []
        n = int(P["correct_n"]) if n is None else int(n)
        for it in range(n):
            self.settle(P["correct_settle_s"], "%s%d" % (label, it + 1))
            st = ctx.st()
            qm = np.array(st["q"], float)
            e = self.palm_err(st, key)
            step = {s: self.q[s][key] - qm[SL[s]] for s in SIDES}
            for s in SIDES:
                self.corr[s] = self.corr[s] + step[s]
            worst = max(float(np.max(np.abs(self.corr[s]))) for s in SIDES)
            rec = dict(at=key, n=it + 1, palm_err_before_mm={s: np.round(e[s], 1) for s in SIDES},
                       palm_err_before_norm_mm={s: round(float(np.linalg.norm(e[s])), 1) for s in SIDES},
                       step_mrad={s: np.round(1000 * step[s]).astype(int).tolist() for s in SIDES},
                       total_mrad={s: np.round(1000 * self.corr[s]).astype(int).tolist() for s in SIDES})
            ctx.log("lone_grip correction", **rec)
            out.append(rec)
            if worst > P["correct_max_rad"]:
                raise Abort("aim correction of %.0f mrad on one joint > %.0f: the arm is far from its plan (gravity offset? something "
                            "in the way?)" % (1000 * worst, 1000 * P["correct_max_rad"]))
            self.play([(key, self.h_now, P["correct_T_s"], "%s%d" % (label, it + 1))])
        self.settle(P["correct_settle_s"], "corrected")
        e = self.palm_err(ctx.st(), key)
        fin = dict(palm_err_after_mm={s: np.round(e[s], 1) for s in SIDES},
                   palm_err_after_norm_mm={s: round(float(np.linalg.norm(e[s])), 1) for s in SIDES},
                   total_mrad={s: np.round(1000 * self.corr[s]).astype(int).tolist() for s in SIDES})
        ctx.log("lone_grip corrected", at=key, note="simulation (at down): 0.2-0.6 mm after two corrections; R1 expected 1-3 mm", **fin)
        return dict(steps=out, **fin)

    def grip(self):
        T, P = self.P["T_s"], self.P
        self.play([("inn", self.h_now, T, "in")])
        self.play([("now", self.hands12("close"), T, "close")])
        self.play([("sq", self.h_now, T, "squeeze")])
        self.play([("now", self.hands12("close2"), T, "close2")])
        self.play([("lift", self.h_now, T * P["lift_T_factor"], "lift")])

    def hold(self):
        """hold_s, then hold2_s still: the aim points' drift (FK, measured) and the camera's plate over the still hold."""
        ctx, P = self.ctx, self.P
        self.settle(P["hold_s"], "hold")
        st0 = ctx.st()
        e0 = self.palm_err(st0, "lift")
        cam0 = ctx.plate(st0, max_age=0.5)
        self.settle(P["hold2_s"], "hold")
        st1 = ctx.st()
        e1 = self.palm_err(st1, "lift")
        cam1 = ctx.plate(st1, max_age=0.5)
        out = dict(palm_vs_lift_target_mm={s: np.round(e1[s], 1) for s in SIDES},
                   palm_drift_in_still_hold_mm={s: round(float(np.linalg.norm(e1[s] - e0[s])), 1) for s in SIDES})
        if cam0 and cam1 and cam0.get("center") and cam1.get("center"):
            c0, c1 = np.array(cam0["center"]), np.array(cam1["center"])
            out["camera_rim_z_rise_mm"] = round(1000 * (c1[2] - (self.table_z + TASK.H)), 1)
            out["camera_moved_in_still_hold_mm"] = round(1000 * float(np.linalg.norm(c1 - c0)), 1)
        else:
            out["camera"] = "no plate estimate in the hold (the camera may not follow a lifted plate): judge by the video"
        ctx.log("lone_grip hold", note="LOOK: is the plate level and still in the hands?", **out)
        return out

    def release(self):
        """Lower to the squeeze pose, open, out to `down`, up to `pre` and `high`; trim the server's path at the mark."""
        T, P = self.P["T_s"], self.P
        self.play([("sq", self.h_now, T * P["lift_T_factor"], "lower")])
        self.play([("now", self.hands12("pre"), P["release_open_s"], "open")])
        self.play([("inn", self.h_now, T, "out"), ("down", self.h_now, T, "out")])
        self.back_to_mark()

    def back_to_mark(self):
        T = self.P["T_s"]
        self.play([("pre", self.h_now, T, "up"), ("mark", self.hands12("open"), T, "up")])
        self.settle(0.3, "up")
        self.ctx.c.trim()
        self.ctx.c.hold(0.2)

    def safe_exit(self):
        """After an abort: let go of the plate and get back to the mark the way the grip came, from where it got to."""
        ctx = self.ctx
        r = self.reached
        ctx.log("lone_grip", note="aborted at '%s': letting go and backing off" % r)
        T = self.P["T_s"]
        if self.q_mark is None:
            return
        if r in ("lift", "hold"):
            self.play([("sq", self.h_now, 2 * T, "lower")])
        if r in GRIPPING or r in ("lower",):
            self.play([("now", self.hands12("pre"), self.P["release_open_s"], "open")])
        if r in GRIPPING or r in ("in", "lower", "open", "out"):
            self.play([("inn", self.h_now, T, "out"), ("down", self.h_now, T, "out")])
        self.back_to_mark()

    # ================================================================== the sequence
    def setup(self, need_camera):
        ctx, P = self.ctx, self.P
        st = ctx.st()
        up = P["air_lift_mm"] / 1000 if self.mode == "air" else 0.0
        cx, cy, table_z, src = self.plate_and_table(st, need_camera)
        off = math.hypot(cx - P["plate_xy_ref"][0], cy - P["plate_xy_ref"][1])
        if off > P["max_offset_m"]:
            raise Abort("the plate is %.0f mm from where the simulation planned it: move it to x %.2f y %.2f (pelvis frame)" % (
                1000 * off, P["plate_xy_ref"][0], P["plate_xy_ref"][1]))
        worst = self.plan(st, (cx, cy), table_z, up)
        ctx.log("lone_grip plan", grip=self.gname, mode=self.mode, plate_xy=[cx, cy], table_z=table_z, source=src,
                offset_from_sim_mm=1000 * off, raised_mm=1000 * up, worst_ik_mm=worst,
                ik={s: self.ikr[s] for s in SIDES}, hands=self.G["hand"])
        if worst > P["ik_max_mm"]:
            raise Abort("IK misses a target by %.1f mm (> %.1f): plate out of reach for this grip" % (worst, P["ik_max_mm"]))
        return dict(plate_xy=[round(cx, 4), round(cy, 4)], table_z=round(table_z, 4), plate_source=src, worst_ik_mm=round(worst, 1))

    def run(self):
        ctx, P = self.ctx, self.P
        res = dict(mode=self.mode, grip=self.gname, table_m=P["table_m"])
        res.update(self.setup(need_camera=(self.mode == "full" and P["use_camera"])))
        self.open_csv()
        res["log"] = self.csv_path
        try:
            self.approach()
            if self.mode == "shape":
                res.update(self.run_shape())
            else:
                T = P["T_s"]
                self.play([("pre", self.h_now, T, "pre")])
                if P["correct_at_pre_n"]:          # (deploy, not simulated): the descent to `down` passes 0-3 mm above the
                    # rim's edge in the plan; corrected here first, the hands come down where planned instead of 2-9 mm off
                    res["correction_at_pre"] = self.correct("pre", P["correct_at_pre_n"], "precorrect")
                self.play([("down", self.h_now, T, "down")])
                res["correction"] = self.correct()
                self.grip()
                res["hold"] = self.hold()
                self.release()
                res["result"] = "gripped, held and put down" if self.mode == "full" else "played in free air"
        except Abort as ex:
            res["result"] = "aborted: %s" % ex
            self.finish(res)
            try:
                self.safe_exit()
            except Exception as ex2:                                      # noqa: BLE001
                ctx.log("lone_grip", note="could not back off cleanly: %r (the server's rewind takes over)" % ex2)
            raise
        return self.finish(res)

    def run_shape(self):
        """At the high pose: pre, close, close2, each held shape_hold_s (look, film the thumb), then open."""
        ctx, P = self.ctx, self.P
        T = P["T_s"]
        for name in ("pre", "close", "close2"):
            self.play([("now", self.hands12(name), T, "shape " + name)])
            ctx.log("lone_grip shape", note="LOOK at both hands now: '%s' (%.0f s)" % (name, P["shape_hold_s"]), hand=self.G["hand"][name])
            self.settle(P["shape_hold_s"], "shape " + name)
        self.play([("now", self.hands12("open"), T, "open")])
        self.ctx.c.trim()
        return dict(result="shapes shown", hands=self.G["hand"])

    def finish(self, res):
        if self.csv is not None:
            self.csv_f.flush()
        self.ctx.results[self.name] = res
        return res

    # ================================================================== plan only (run_test.py --dry)
    def dry(self, st, plate):
        P = self.P
        up = P["air_lift_mm"] / 1000 if self.mode == "air" else 0.0
        pelvis = self.ctx.pelvis(st)
        table_legs = P["table_m"] - pelvis
        if self.mode == "full" and plate and plate.get("center"):
            cx, cy, rim_z = plate["center"]
            table_z = rim_z - TASK.H
            print("lone_grip %s, grip %s: plate (camera) x %.3f y %+.3f; table top %.3f by the camera, %.3f from table_m %.3f - pelvis %.3f "
                  "(%+.0f mm%s); located plates %s" % (
                      self.mode, self.gname, cx, cy, table_z, table_legs, P["table_m"], pelvis, 1000 * (table_z - table_legs),
                      "  << TOO FAR: the run would refuse" if abs(table_z - table_legs) > P["table_z_tol_m"] else "",
                      (plate.get("located") or {}).get("n_plates")))
        else:
            cx, cy = P["plate_xy"]
            table_z = table_legs
            print("lone_grip %s, grip %s: %s plate_xy x %.3f y %+.3f, table top %.3f (table_m %.3f - pelvis %.3f)%s" % (
                self.mode, self.gname, "NO camera plate: using" if self.mode == "full" else "no plate needed:", cx, cy, table_z,
                P["table_m"], pelvis, "; raised %.0f mm" % (1000 * up) if up else ""))
        off = math.hypot(cx - P["plate_xy_ref"][0], cy - P["plate_xy_ref"][1])
        print("  %.0f mm from where the simulation planned it (x %.2f y %.2f; limit %.0f mm)%s" % (
            1000 * off, P["plate_xy_ref"][0], P["plate_xy_ref"][1], 1000 * P["max_offset_m"],
            "  << TOO FAR" if off > P["max_offset_m"] else ""))
        worst = self.plan(st, (cx, cy), table_z, up)
        self.report_plan(print)
        low = self.clearance(np.array(st["des"], float), self.q11_at("high", corr=False), self.hands12("pre"))
        print("  move from here to 'high': lowest hand point %s above the rim over the plate%s" % (
            "never" if low == float("inf") else "%.0f mm" % low, "  << TOO LOW" if low < P["approach_clear_mm"] else ""))
        fmt = lambda v: "never there" if v == float("inf") else "%.0f mm above the rim" % v     # noqa: E731
        for a, b in (("high", "pre"), ("pre", "down")):
            qa, qb = self.q11_at(a, corr=False), self.q11_at(b, corr=False)
            print("  %s -> %s: lowest hand point over the plate %s; within 10 mm outside its edge %s" % (
                a, b, fmt(self.clearance(qa, qb, self.hands12("pre"), edge_mm=0.0)), fmt(self.clearance(qa, qb, self.hands12("pre")))))
        print("  hands (service order thumb bend, thumb rotation, index, middle, ring, little): %s" % self.G["hand"])
        print("  worst IK %.1f mm (limit %.1f): %s" % (worst, P["ik_max_mm"], "OK" if worst <= P["ik_max_mm"] else "WOULD REFUSE"))
