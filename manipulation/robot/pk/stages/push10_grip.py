"""push10 followed by the first grip: the left hand stays where the push ended and the right hand grips the side that
sticks out, using the lone grip's right-hand path (params/lone_grip.json).

    python3 run_test.py push10_grip --set push10.grip_lift_mm=0      # first: grip and squeeze only, no lift
    python3 run_test.py push10_grip                                  # with the lift (grip_lift_mm, default 30)
"""
import math

import numpy as np

from .. import config
from ..ik import ArmChain
from ..model import hand_points
from ..plate import TASK
from ..stage_base import Abort, smooth
from .lone_grip import aim_point, frames, palm_ik, targets
from .push10 import Push10

R = config.RIGHT


class Push10Grip(Push10):
    name = "push10_grip"

    def __init__(self, ctx, mode="full"):
        Push10.__init__(self, ctx, "full")
        self.LP = ctx.params("lone_grip")
        gname = self.P.get("grip_name") or self.LP["grip"]
        if gname not in self.LP["grips"]:
            raise Abort("push10_grip: no lone grip '%s' (have %s)" % (gname, sorted(self.LP["grips"])))
        self.gname = gname
        self.G = self.LP["grips"][gname]
        self.g = self.G["genes"]
        self.H = {k: np.array(v, float) for k, v in self.G["hand"].items()}
        self.d5r = np.asarray(ctx.rob.d11[R], float)
        self.rq = {}
        self.rcorr = np.zeros(5)
        self.r_reached = "none"
        self.rim_info = {}

    # ------------------------------------------------------------------ the push: push10's, unchanged, without the release
    def run(self):
        ctx, P = self.ctx, self.P
        self.setup()
        worst, _ = self.reach_report()
        if worst > 10.0:
            raise Abort("the set / press points are out of reach (IK %.0f mm off): is the stack where the simulation had it?" % worst)
        self.open_csv()
        res = dict(mode="grip", table=P["table_m"], log=self.csv_path, grip=self.gname)
        try:
            self.go_pre()
            self.phase_now = "pre"
            cam_start = self.cam_y() if P["use_camera"] else None
            res.update(self.push_loop(cam_start))
            if res.get("result") != "pushed":
                raise Abort("push10_grip: the push ended '%s' (camera %.0f mm): no grip" % (res.get("result"), res.get("camera_slide_mm", 0.0)))
            res["grip"] = self.right_grip()
            res["result"] = "pushed, gripped%s and put down" % (" (no lift)" if P["grip_lift_mm"] <= 0 else ", lifted")
        except Abort as ex:
            res.update(result="aborted: %s" % ex)
            self.finish(res)
            try:
                self.right_exit()
            except Exception as ex2:                              # noqa: BLE001
                ctx.log("push10_grip", note="right hand: could not back off cleanly: %r" % ex2)
            if self.q_mark is not None:
                ctx.log("push10", note="lifting off and back to the marked pose before handing the arms back")
                try:
                    self.release()
                except Exception as ex2:                              # noqa: BLE001
                    ctx.log("push10", note="could not lift off cleanly: %r" % ex2)
            raise
        self.release()
        self.finish(res)
        return res

    # ------------------------------------------------------------------ the right hand
    def right_tick(self, q5r, phase):
        """One 30 Hz command with the right arm at q5r (motor joints); the left held at its last IK solution."""
        self.q11[R] = q5r
        s = self.tick(self.q, "grip " + phase)
        st = self.ctx.st()
        lag = float(np.max(np.abs(np.array(st["des"])[R] - np.array(st["q"])[R])))
        if lag > self.P["q_lag_max_rad"]:
            self._rlag = (self._rlag + 1) if getattr(self, "_rlag", 0) else 1
            if self._rlag > self.P["q_lag_s"] * self.P["rate_hz"]:
                raise Abort("the right arm is %.2f rad from its command (held back)" % lag)
        else:
            self._rlag = 0
        return s

    def right_move(self, q_to, T, phase, hand_to=None, left_dz=0.0):
        """Joint smoothstep of the right arm to q_to (motor) over T s (longer if a joint would pass grip_v_rad_s); the right
        hand to hand_to; the left index target raised by left_dz (m) along the way."""
        P = self.P
        q_a = self.q11[R].copy()
        h_a = self.hands[6:12].copy()
        h_b = h_a if hand_to is None else np.asarray(hand_to, float)
        T = max(T, float(np.max(np.abs(q_to - q_a))) / P["grip_v_rad_s"])
        N = max(1, int(round(T * P["rate_hz"])))
        Pt0 = self.Pt.copy()
        for i in range(1, N + 1):
            u = smooth(i / float(N))
            self.hands[6:12] = h_a + u * (h_b - h_a)
            if left_dz:
                Pk = Pt0 + np.array([0.0, 0.0, u * left_dz])
                self.q, _, _ = self.ik(Pk, self.q, P["w_rot"])
                self.Pt = Pk
            self.right_tick(q_a + u * (q_to - q_a), phase)
        self.r_reached = phase

    def right_hold(self, seconds, phase):
        for _ in range(int(seconds * self.P["rate_hz"])):
            self.right_tick(self.q11[R], phase)

    def plate_now(self, n=10):
        """The camera's plate after the push: centre (x, y, tracker rim z; medians over n frames) and the rim plane
        (median z0, gx, gy over the frames that have one; None if none)."""
        xs, pl = [], []
        for _ in range(n):
            s = self.tick(self.q, "grip plate")
            if s["cam"] is not None:
                xs.append(np.array(s["cam"], float))
                p = self.ctx.plate(max_age=0.5)
                if p and p.get("plane"):
                    q = p["plane"]
                    pl.append([q["z0"], q["gx"], q["gy"], q["rms_mm"], q["n_sec"]])
        if not xs:
            raise Abort("push10_grip: no plate estimate from the camera after the push")
        plane = None
        if pl:
            m = np.median(np.array(pl, float), axis=0)
            plane = dict(z0=float(m[0]), gx=float(m[1]), gy=float(m[2]), rms_mm=float(m[3]), n_sec=int(m[4]), frames=len(pl))
        return np.median(np.array(xs), axis=0), plane

    def grip_rim_z(self, c, plane, xy_aim):
        """The rim height where the right hand grips: the rim plane at the rim point toward the hand's aim
        point. The pushed plate tilts ~5 deg toward the right hand; its gripped rim stands ~15 mm above the tracker's
        single height. grip_height "tracker" = the old single height."""
        P = self.P
        if P.get("grip_height", "plane") == "tracker":
            return float(c[2]), dict(source="tracker")
        if plane is None:
            raise Abort("push10_grip: no rim plane from perception (an old perception.py still running? restart it); "
                        "for the old single height: --set push10.grip_height='\"tracker\"'")
        if plane["n_sec"] < P.get("grip_plane_min_sectors", 12) or plane["rms_mm"] > P.get("grip_plane_max_rms_mm", 2.0):
            raise Abort("push10_grip: rim plane not trusted (%d sectors, rms %.1f mm)" % (plane["n_sec"], plane["rms_mm"]))
        a = math.atan2(xy_aim[1] - c[1], xy_aim[0] - c[0])
        z = plane["z0"] + TASK.R * (plane["gx"] * math.cos(a) + plane["gy"] * math.sin(a))
        tilt = math.degrees(math.atan(math.hypot(plane["gx"], plane["gy"])))
        info = dict(source="plane", rim_z=round(z, 4), vs_tracker_mm=round(1000 * (z - float(c[2])), 1),
                    tilt_deg=round(tilt, 2), at_deg=round(math.degrees(a), 1), sectors=plane["n_sec"], rms_mm=plane["rms_mm"])
        if abs(z - float(c[2])) > P.get("grip_plane_max_dz_mm", 30.0) / 1000:
            raise Abort("push10_grip: the rim plane puts the gripped rim %.0f mm from the tracker's height: not trusted"
                        % (1000 * (z - float(c[2]))))
        return z, info

    def plan_right(self, c, plane=None):
        """IK of the lone grip's right-hand targets for the plate centre c (x, y, rim z) and the rim plane. Motor joints
        in self.rq."""
        rob, LP = self.ctx.rob, self.LP
        g = self.g
        if self.P.get("grip_sq_mm") is not None:
            g = dict(g, sq=float(self.P["grip_sq_mm"]))
        tg0, _, _ = targets(g, "right", (float(c[0]), float(c[1])), float(c[2]) - TASK.H, LP, 0.0)
        rim_z, self.rim_info = self.grip_rim_z(c, plane, tg0["inn"][:2])
        table_z = rim_z - TASK.H + self.P["grip_dz_mm"] / 1000
        ch = ArmChain(rob, "right", "right_base_link", self.q11, None)
        self.rch = ch
        tg, f, n = targets(g, "right", (float(c[0]), float(c[1])), table_z, LP, 0.0)
        loc = aim_point(self.g, "right")
        ref = self.G["ref_q_rad"]["right"]
        out, prev, worst = {}, None, 0.0
        for k in ("pre", "down", "inn", "sq", "lift", "high"):
            seeds = [np.array(ref["pre" if k == "high" else k], float)]
            if prev is not None:
                seeds.append(prev)
            best = None
            for q0 in seeds:
                q, info = palm_ik(ch, q0, tg[k], loc, f, n, LP["ik_w_rot"])
                cst = info["pos_mm"] + 0.3 * (info["f_deg"] + info["n_deg"])
                if best is None or cst < best[0]:
                    best = (cst, q, info)
            _, q, info = best
            self.rq[k] = q - self.d5r
            out[k] = dict(target=[round(float(v), 4) for v in tg[k]], **{a: round(b, 1) for a, b in info.items()})
            if k != "high":
                prev = q
            worst = max(worst, info["pos_mm"])
        # the lift: the squeeze pose raised grip_lift_mm (no lift_in: the left hook holds the other side)
        lz = self.P["grip_lift_mm"] / 1000
        if lz > 0:
            q, info = palm_ik(ch, self.rq["sq"] + self.d5r, tg["sq"] + np.array([0, 0, lz]), loc, f, n, LP["ik_w_rot"])
            self.rq["lift"] = q - self.d5r
            out["lift"] = dict(target=[round(float(v), 4) for v in tg["sq"] + np.array([0, 0, lz])], **{a: round(b, 1) for a, b in info.items()})
            worst = max(worst, info["pos_mm"])
        self.r_tg, self.r_loc = tg, loc
        return worst, out, table_z

    def right_clear(self, q_from, q_to, c, n=15):
        """Lowest right-hand point above the rim where it is over the plate or within 10 mm of its edge (mm)."""
        rob = self.ctx.rob
        low = float("inf")
        h = self.hands.copy()
        h[6:12] = self.H["pre"]
        for u in np.linspace(0.0, 1.0, n):
            q = self.q11.copy()
            q[R] = q_from + smooth(u) * (q_to - q_from)
            T = rob.fk(q, h)
            for pts in hand_points(T, "right").values():
                r = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
                m = r < TASK.R + 0.010
                if m.any():
                    low = min(low, 1000 * float(pts[m, 2].min() - max(float(c[2]), self.rim_info.get("rim_z", float(c[2])))))
        return low

    def right_correct(self, key, n, label):
        """At plan pose key: settle, add (plan - measured) to the right arm's correction, command plan + correction."""
        ctx, LP = self.ctx, self.LP
        recs = []
        for it in range(int(n)):
            self.right_hold(LP["correct_settle_s"], label)
            qm = np.array(ctx.st()["q"], float)[R]
            step = self.rq[key] - qm
            self.rcorr = self.rcorr + step
            T, _ = self.rch.fk(qm + self.d5r)
            err = 1000 * (T[:3, :3] @ self.r_loc + T[:3, 3] - self.r_tg[key])
            recs.append(dict(at=key, n=it + 1, palm_err_mm=np.round(err, 1).tolist(), step_mrad=np.round(1000 * step).astype(int).tolist()))
            ctx.log("push10_grip correction", **recs[-1])
            if float(np.max(np.abs(self.rcorr))) > LP["correct_max_rad"]:
                raise Abort("right arm aim correction %.0f mrad > %.0f: far from its plan" % (1000 * float(np.max(np.abs(self.rcorr))), 1000 * LP["correct_max_rad"]))
            self.right_move(self.rq[key] + self.rcorr, LP["correct_T_s"], label)
        return recs

    def right_grip(self):
        ctx, P, LP = self.ctx, self.P, self.LP
        self.checking = False                       # the left index's force limit is for the push; the server's 45 N stays
        # the push's command runs 40+ mm ahead of the hand: hold the left where it MEASURES now, or it keeps pushing (the
        # simulated arm: +10.7 mm during the right hand's approach)
        qm = np.array(ctx.st()["q"], float)[config.LEFT]
        self.q = qm.copy()
        self.qc = qm.copy()                         # the command itself back to where the arm is (no step: it is there)
        self.Pt = np.asarray(self.idx(qm), float).copy()
        self.right_hold(0.5, "left stop")
        c, plane = self.plate_now()
        c_cam = c.copy()
        if P.get("grip_tap_bias", True):
            c[1] += self.cam_bias                   # the left taps' edge minus the camera's edge (2-12 mm on the robot)
        worst, ik, table_z = self.plan_right(c, plane)
        ctx.log("push10_grip plan", grip=self.gname, plate=[round(float(v), 4) for v in c],
                camera=[round(float(v), 4) for v in c_cam], rim=self.rim_info,
                tap_bias_mm=round(1000 * self.cam_bias, 1) if P.get("grip_tap_bias", True) else 0.0,
                squeeze_mm=P["grip_sq_mm"] if P.get("grip_sq_mm") is not None else self.g["sq"],
                lone_table_z=round(table_z, 4), worst_ik_mm=round(worst, 1), ik=ik, lift_mm=P["grip_lift_mm"])
        if worst > LP["ik_max_mm"]:
            raise Abort("push10_grip: right-hand IK misses by %.1f mm (> %.1f): out of reach" % (worst, LP["ik_max_mm"]))
        q0 = self.q11[R].copy()
        low = self.right_clear(q0, self.rq["high"], c)
        ctx.log("push10_grip approach", lowest_right_hand_point_above_rim_mm=round(low, 1))
        if low < LP["approach_clear_mm"]:
            raise Abort("push10_grip: the right hand's move to its high pose would pass %.0f mm above the rim" % low)
        T = LP["T_s"]
        self.right_move(self.rq["high"], 2.0, "approach", hand_to=self.H["pre"])
        self.right_move(self.rq["pre"], T, "pre")
        self.right_correct("pre", LP["correct_at_pre_n"], "precorrect")
        self.right_move(self.rq["down"] + self.rcorr, T, "down")
        corr = self.right_correct("down", LP["correct_n"], "correct")
        self.right_move(self.rq["inn"] + self.rcorr, T, "in")
        self.right_move(self.rq["inn"] + self.rcorr, T, "close", hand_to=self.H["close"])
        self.right_move(self.rq["sq"] + self.rcorr, T, "squeeze")
        self.right_move(self.rq["sq"] + self.rcorr, T, "close2", hand_to=self.H["close2"])
        out = dict(correction=corr, plate=[round(float(v), 4) for v in c])
        st = ctx.st()
        p0 = ctx.plate(st, max_age=0.5)
        if P["grip_lift_mm"] > 0:
            self.right_move(self.rq["lift"] + self.rcorr, T * LP["lift_T_factor"], "lift", left_dz=P["grip_lift_mm"] / 1000)
            self.right_hold(P["grip_hold_s"], "hold")
            p1 = ctx.plate(ctx.st(), max_age=0.5)
            if p0 and p1 and p0.get("center") and p1.get("center"):
                out["camera_rim_rise_mm"] = round(1000 * (p1["center"][2] - p0["center"][2]), 1)
            ctx.log("push10_grip hold", note="LOOK: is the plate up and still in the hands?", **out)
            self.right_move(self.rq["sq"] + self.rcorr, T * LP["lift_T_factor"], "lower", left_dz=-P["grip_lift_mm"] / 1000)
        else:
            self.right_hold(P["grip_hold_s"], "hold")
            ctx.log("push10_grip hold", note="grip only (grip_lift_mm 0): LOOK at the clamp", **out)
        self.right_open_out()
        return out

    def right_open_out(self):
        LP = self.LP
        T = LP["T_s"]
        self.right_move(self.q11[R], LP["release_open_s"], "open", hand_to=self.H["pre"])
        self.right_move(self.rq["inn"] + self.rcorr, T, "out")
        self.right_move(self.rq["down"] + self.rcorr, T, "out")
        self.right_move(self.rq["pre"] + self.rcorr, T, "up")
        self.right_move(self.rq["high"], T, "up", hand_to=np.zeros(6))
        self.r_reached = "clear"

    def right_exit(self):
        """After an abort during the grip: lower if lifted, open, back out the way it came."""
        r = self.r_reached
        if r in ("none", "clear") or not self.rq:
            return
        self.ctx.log("push10_grip", note="aborted at '%s': right hand lets go and backs off" % r)
        LP = self.LP
        if r in ("lift", "hold", "lower") and self.P["grip_lift_mm"] > 0:
            self.right_move(self.rq["sq"] + self.rcorr, 2 * LP["T_s"], "lower", left_dz=-self.P["grip_lift_mm"] / 1000)
        if r in ("approach", "pre", "precorrect", "down", "correct"):          # still outside the rim: straight back up
            self.right_move(self.rq["pre"] + self.rcorr, LP["T_s"], "up")
            self.right_move(self.rq["high"], LP["T_s"], "up", hand_to=np.zeros(6))
            self.r_reached = "clear"
            return
        self.right_open_out()
