"""Arm identification in free air: how the robot-left arm follows its commands with nothing touching it, so that the
simulated arm can be corrected. No plate may be in reach; z_floor_mm stops the arm if the pad goes too low.
"""
import csv
import os
import sys
import threading
import time

import numpy as np

from .. import config
from ..stage_base import Abort
from .push10 import PADS, SIDE, Push10, PAD_POINT


class ArmId(Push10):
    name = "armid"

    def __init__(self, ctx, mode="full"):
        ctx.overrides = dict(ctx.overrides, **{"push10.table_m": 0.747})     # push10's pose table is not used here
        Push10.__init__(self, ctx, mode)
        self.P = dict(self.P, **ctx.params("armid"), use_camera=False)
        self.pose = dict(hand_cmd=self.P["hand_cmd"], q_plan_rad=self.P["q_plan_rad"], di_mm=0.0)
        self.vmax = self.P["vmax_free_rad_s"]
        self.csv_name = "armid_cmd.csv"
        self.corr = np.array(self.P["q_correct_mrad"], float) / 1000
        self.q_tgt = None
        if mode not in ("full", "load"):                                     # at the real contact pose
            self.P.update(sweep_z_mm=self.P["contact_z_mm"], z_floor_mm=self.P["contact_floor_mm"])

    # ================================================================== the plan
    def centre(self):
        P = self.P
        return np.array([P["centre_xy"][0], P["centre_xy"][1], P["rim_z"]], float)

    def centre_pose(self):
        return self.centre() + np.array([0, 0, self.P["sweep_z_mm"]]) / 1000

    def waypoints(self, qc):
        """Joint-space corners around qc: half of the 2^5 sign combinations, so each joint goes up and down with the others
        both ways. Not Cartesian lines: a wide Cartesian grid here jumps between IK branches (seen 0.8 rad of
        shoulder between points 30 mm apart)."""
        amp = np.array(self.P["wp_rad"], float)
        signs = [np.array([(k >> i & 1) * 2 - 1 for i in range(4)]) for k in range(16)]
        return [qc + amp * np.r_[s, np.prod(s)] for s in signs]

    def check_plan(self):
        """Every point inside the joint limits, above the floor and near the centre, before anything moves."""
        P = self.P
        floor = float(P["rim_z"] + P["z_floor_mm"] / 1000)
        # up from the contact pose in small steps: the same arm configuration as at the rim
        qc, ec = self.q_plan.copy(), 0.0
        p0 = self.idx(self.q_plan)
        for u in np.linspace(0.0, 1.0, 12)[1:]:
            qc, ec, dc = self.ik(p0 + u * (self.centre_pose() - p0), qc, P["w_rot"])
        if ec > P["ik_max_mm"]:
            raise Abort("armid: the centre pose is out of reach (IK %.0f mm off)" % ec)
        qs = self.waypoints(qc)
        amp = float(P["sweep_rad"])
        tests = qs + [qc + sg * amp * np.eye(5)[j] for j in range(5) for sg in (-1, 1)]
        # the straight joint moves between the corners too, not only their ends
        path = [q0 + u * (q1 - q0) for q0, q1 in zip(qs[:-1], qs[1:]) for u in np.linspace(0.0, 1.0, 9)] + tests
        low = min(self.pad_low(x) for x in path)
        pads = np.array([self.idx(x) for x in path])
        far = 1000 * float(np.max(np.linalg.norm(pads - self.centre_pose(), axis=1)))
        rep = dict(corners=len(qs), centre_ik_mm=round(ec, 1), centre_q=np.round(qc, 3).tolist(), hand_range_mm=round(far, 0),
                   lowest_pad_mm_above_floor=round(1000 * (low - floor), 1))
        if low < floor:
            raise Abort("armid: the plan goes %.0f mm below the floor" % (1000 * (floor - low)))
        if far > P["hand_range_max_mm"]:
            raise Abort("armid: the plan takes the hand %.0f mm from the centre (limit %.0f)" % (far, P["hand_range_max_mm"]))
        if np.any(np.array(tests) <= self.ch.lo + 0.02) or np.any(np.array(tests) >= self.ch.hi - 0.02):
            raise Abort("armid: a corner is at a joint limit")
        return qs, qc, rep

    def pad_low(self, q5):
        """The lower of the index and thumb pads, pelvis z."""
        T, _ = self.ch_th.fk(q5)
        return min(float(self.idx(q5)[2]), float((T[:3, :3] @ PAD_POINT + T[:3, 3])[2]))

    # ================================================================== the log: push10's columns + dq, tau
    def open_csv(self, extra_cols=()):
        Push10.open_csv(self, ["dq%d" % i for i in range(5)] + ["tau%d" % i for i in range(5)] + ["q_tgt%d" % i for i in range(5)]
                        + ["sent%d" % i for i in range(5)] + ["hand_cmd%d" % i for i in range(6)] + ["roll", "pitch"] + list(extra_cols))
        self.csv_f.close()
        os.remove(self.csv_path)
        cols = self.csv.fieldnames
        self.csv_path = os.path.join(self.ctx.out_dir, self.csv_name)
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols, extrasaction="ignore")
        self.csv.writeheader()

    def row(self, st, s, phase, P, ik, extra):
        e = dict(extra or {})
        dq, tau, sent = np.array(st["dq"])[config.LEFT], np.array(st["tau"])[config.LEFT], np.array(st["sent"])[config.LEFT]
        for i in range(5):
            e["dq%d" % i] = round(float(dq[i]), 4)
            e["tau%d" % i] = round(float(tau[i]), 3)
            e["sent%d" % i] = round(float(sent[i]), 5)
            if self.q_tgt is not None:
                e["q_tgt%d" % i] = round(float(self.q_tgt[i]), 5)
        for i in range(6):
            e["hand_cmd%d" % i] = round(float(self.hands[i]), 4)
        rpy = st.get("rpy") or [0, 0, 0]
        e.update(roll=rpy[0], pitch=rpy[1])
        Push10.row(self, st, s, phase, P, ik, e)

    def tick(self, q_target, phase, P=None, ik=(0.0, 0.0), extra=None):
        self.q_tgt = np.array(q_target, float)
        s = Push10.tick(self, self.q_tgt + self.corr, phase, P, ik, extra)
        low = self.pad_low(s["qm"])
        if self.checking and low < self.P["rim_z"] + self.P["z_floor_mm"] / 1000:
            raise Abort("the hand is at the floor limit (%.0f mm above the rim's height): is it being pushed down?" % (
                1000 * (low - self.P["rim_z"])))
        return s

    def ev(self, what, **kw):
        kw["at_s"] = round(time.time() - self.t0, 2)
        self.events.append(dict(ev=what, **kw))
        self.ctx.log("armid " + what, **kw)

    # ================================================================== motion
    def start(self, q5, res):
        ctx, P = self.ctx, self.P
        ctx.c.set(hands=self.hands, tag="armid shape")
        why = ctx.c.hold(1.0)
        if why:
            raise Abort(why)
        q11 = self.q11.copy()
        q11[config.LEFT] = q5 + self.corr
        why = ctx.c.move_to(q11, self.hands, v=P["pre_v_rad_s"], tag="armid start")
        if why:
            raise Abort("move to the start: %s" % why)
        ctx.c.set(vmax=P["vmax_id_rad_s"] if self.mode in ("fric", "chirp") else P["vmax_rad_s"])
        ctx.c.hold(0.5)
        self.q11, self.qc, self.q = q11, q5 + self.corr, q5.copy()
        ctx.c.mark()
        self.q_mark = q11.copy()
        self.t_next = time.time()

    def go(self, q5, phase, P=None, tol=1e-4, max_s=20.0):
        """Command toward q5 at self.vmax (per joint, the tick's speed cap) until reached."""
        self.q = q5.copy()
        for _ in range(int(max_s * self.P["rate_hz"])):
            if float(np.max(np.abs(q5 + self.corr - self.qc))) < tol:
                return
            self.tick(q5, phase, P)

    def walk(self, qs, v, tag):
        """Joint moves through the corners at v rad/s (per joint), holding at each."""
        P = self.P
        self.Pt = self.centre_pose()
        self.vmax = float(v)
        for q in qs:
            self.go(q, "line_" + tag, self.Pt)
            self.hold_ticks(P["hold_s"] * P["rate_hz"], "hold_" + tag)
        self.vmax = P["vmax_rad_s"]

    def sweep(self, qc):
        P = self.P
        pc = self.centre_pose()
        self.Pt = pc.copy()
        amp = float(P["sweep_rad"])
        for v in P["sweep_v_rad_s"]:
            self.vmax = float(v)
            for j in range(5):
                tag = "j%d_v%g" % (j, v)
                self.ev("joint", joint=config.ARM_NAMES[j], v=v)
                for sign, name in ((1, "up"), (-1, "down"), (1, "up"), (0, "back")):
                    q = qc + sign * amp * np.eye(5)[j]
                    self.go(q, tag + "_" + name, pc)
                    self.hold_ticks(P["hold_s"] * P["rate_hz"], tag + "_hold_" + name)
        self.vmax = P["vmax_rad_s"]

    def end(self):
        """The hand open, back to the marked pose, and hand the path back to the server."""
        ctx = self.ctx
        self.checking = False
        self.hands[:6] = 0.0
        try:
            self.vmax = self.P["vmax_free_rad_s"]
            self.hold_ticks(self.P["release_hold_s"] * self.P["rate_hz"], "open")
            if self.q_mark is not None:
                why = ctx.c.move_to(self.q_mark, self.hands, v=0.2, tag="armid back")
                if not why:
                    ctx.c.hold(0.2)
                    ctx.c.trim()
                    ctx.c.hold(0.2)
        except Exception as ex:                                     # noqa: BLE001
            ctx.log("armid", note="could not go back to the mark: %r" % ex)

    def finish(self, res):
        res["events"] = self.events
        if self.csv is not None:
            self.csv_f.flush()
        res["seconds"] = round(time.time() - self.t0, 1)
        self.ctx.results["armid"] = res
        self.ctx.log("armid", result=res.get("result"), log=self.csv_path, seconds=res["seconds"])
        return res

    # ================================================================== the tests
    def labels(self):
        """Typed labels for armid_load, from a thread so the arm keeps being commanded (`load_auto`: offline test)."""
        q = []
        auto = self.P.get("load_auto")
        if auto:
            def timed():
                for lab, dt in auto:
                    time.sleep(dt)
                    q.append(lab)
            threading.Thread(target=timed, daemon=True).start()
        else:
            def typed():
                for line in sys.stdin:
                    if line.strip():
                        q.append(line.strip())
            threading.Thread(target=typed, daemon=True).start()
        return q

    # ================================================================== for the simulation's arm
    def clear_of_body(self, q5):
        """Hand and elbow clear of the body (pelvis frame): the only collision check here, the robot's meshes are not used."""
        P = self.P
        T, fr = self.ch_th.fk(q5)
        pts = [self.idx(q5), T[:3, :3] @ PAD_POINT + T[:3, 3]]
        return (min(float(x[0]) for x in pts) > P["grav_x_min_m"] and min(float(x[1]) for x in pts) > P["grav_y_min_m"]
                and float(fr[3][0][1]) > P["grav_elbow_y_min_m"])

    def pose_ok(self, q5):
        P = self.P
        m = float(P["grav_step_rad"]) + 0.05
        return (np.all(q5 > self.ch.lo + m) and np.all(q5 < self.ch.hi - m) and self.pad_low(q5) > P["rim_z"] + P["z_floor_mm"] / 1000
                and self.clear_of_body(q5))

    def grav_plan(self, qc):
        """Poses in a box around the contact, spread far apart (a fixed seed: the same poses each run), in a short order; every
        straight joint move between them checked. The first pose is the contact pose."""
        P = self.P
        rng = np.random.RandomState(1)
        amp = np.array(P["grav_amp_rad"], float)
        cand = [qc + amp * rng.uniform(-1, 1, 5) for _ in range(800)]
        cand = [q for q in cand if self.pose_ok(q)]
        chosen = [qc]
        while len(chosen) < int(P["grav_n"]) and cand:
            d = [min(float(np.max(np.abs((q - c) / amp))) for c in chosen) for q in cand]
            chosen.append(cand.pop(int(np.argmax(d))))
        order, rest = [chosen[0]], chosen[1:]
        while rest:
            k = int(np.argmin([np.max(np.abs(q - order[-1])) for q in rest]))
            q = rest.pop(k)
            mid = [order[-1] + u * (q - order[-1]) for u in np.linspace(0.1, 0.9, 5)]
            if all(self.pose_ok(x) for x in mid):
                order.append(q)
        return order

    def run_grav(self, qc, res):
        P = self.P
        with self.ctx.c.busy(60):                                   # thousands of FK calls: the server is told to wait
            order = self.grav_plan(qc)
        res["poses"] = np.round(order, 4).tolist()
        self.start(order[0], res)
        self.Pt = self.centre_pose()
        d = float(P["grav_step_rad"]) * np.ones(5)
        n_hold = P["grav_hold_s"] * P["rate_hz"]
        for k, q in enumerate(order):
            self.ev("pose", n=k, of=len(order))
            self.vmax = float(P["grav_v_rad_s"])
            self.go(q, "gmove", self.Pt)
            for name, sg in (("a", -1), ("b", 1)):
                self.vmax = float(P["grav_slow_rad_s"])
                self.go(q + sg * d, "gpre", self.Pt)
                self.go(q, "gpre", self.Pt)
                self.hold_ticks(n_hold, "g%d_%s" % (k, name))
        self.vmax = P["vmax_rad_s"]
        res["result"] = "played"

    def run_fric(self, qc, res):
        P = self.P
        self.start(qc, res)
        self.Pt = self.centre_pose()
        amp = float(P["fric_amp_rad"])
        rate = float(P["rate_hz"])
        for j in range(5):
            self.ev("joint", joint=config.ARM_NAMES[j])
            e = np.eye(5)[j]
            self.vmax = float(P["vmax_id_rad_s"]) / 2
            self.go(qc - amp * e, "fmove", self.Pt)
            for v in P["fric_v_rad_s"]:
                self.vmax = float(v)
                self.go(qc + amp * e, "f%d_v%g_up" % (j, v), self.Pt)
                self.hold_ticks(rate * P["fric_hold_s"], "fhold")
                self.go(qc - amp * e, "f%d_v%g_dn" % (j, v), self.Pt)
                self.hold_ticks(rate * P["fric_hold_s"], "fhold")
            self.vmax = float(P["vmax_id_rad_s"]) / 2
            self.go(qc, "fmove", self.Pt)
            self.hold_ticks(rate, "fhold")
            self.vmax = float(P["vmax_id_rad_s"])
            for a in P["fric_dither_mrad"]:
                self.sine(qc, e, a / 1000, float(P["fric_dither_hz"]), int(P["fric_dither_cycles"]), "d%d_a%g" % (j, a))
        res["result"] = "played"

    def sine(self, qc, e, amp, hz, cycles, phase):
        """A sine on one joint, from rest, from the centre, ending there."""
        rate = float(self.P["rate_hz"])
        n = int(round(cycles * rate / hz))
        for k in range(1, n + 1):
            self.q = qc + amp * np.sin(2 * np.pi * hz * k / rate) * e
            self.tick(self.q, phase, self.Pt)

    def run_chirp(self, qc, res):
        P = self.P
        self.start(qc, res)
        self.Pt = self.centre_pose()
        self.vmax = float(P["vmax_id_rad_s"])
        for j in range(5):
            self.ev("joint", joint=config.ARM_NAMES[j])
            e = np.eye(5)[j]
            for hz in P["chirp_hz"]:
                w = 2 * np.pi * hz
                self.sine(qc, e, min(P["chirp_amp_rad"], P["chirp_v_max"] / w, P["chirp_a_max"] / w ** 2), hz, int(P["chirp_cycles"]), "c%d_h%g" % (j, hz))
                self.hold_ticks(P["rate_hz"] * 0.7, "chold")
        res["result"] = "played"

    def run_hand(self, qc, res):
        """One finger at a time from the test's shape, the arm still: command against measured, and the pad's zero meanwhile."""
        P = self.P
        self.start(qc, res)
        self.Pt = self.centre_pose()
        rate = float(P["rate_hz"])
        base = self.hands[:6].copy()
        for c in range(6):
            a0 = float(base[c])
            a1 = a0 - P["hand_amp"] if a0 > 0.5 else a0 + P["hand_amp"]
            self.ev("finger", channel=c, frm=a0, to=a1)
            for seg, (x0, x1, T) in enumerate(((a0, a1, P["hand_ramp_s"]), (a1, a0, P["hand_ramp_s"]))):
                for u in np.linspace(0.0, 1.0, int(rate * T)):
                    self.hands[c] = x0 + u * (x1 - x0)
                    self.tick(self.q, "h%d_ramp%d" % (c, seg), self.Pt)
            for n, x in enumerate((a1, a0, a1, a0)):
                self.hands[c] = x
                self.hold_ticks(rate * P["hand_step_s"], "h%d_step%d" % (c, n))
        res["result"] = "played"

    def run_load(self, qc, res):
        P = self.P
        pc = self.centre_pose()
        self.start(qc, res)
        self.Pt = pc.copy()
        self.P["f_abort_N"] = P["load_f_abort_N"]
        print("\nHOLDING. Type a label + Enter for each load (0 first, nothing hanging), wait %.0f s after the swing stops, "
              "q + Enter to end.\n" % P["load_settle_s"], flush=True)
        q = self.labels()
        label = "none"
        t0 = time.time()
        while time.time() - t0 < P["load_max_s"]:
            if q:
                label = q.pop(0)
                if label == "q":
                    break
                self.ev("load", label=label)
                print("  logging as load %s" % label, flush=True)
            self.tick(self.q, "load_" + label, pc)
        res["result"] = "held"

    def run(self):
        ctx = self.ctx
        self.t0 = time.time()
        st = ctx.st()
        self.base_F, self.base_pad = np.zeros(3), 0.0
        self.build(st)
        self.cam_t = time.time()
        qs, qc, rep = self.check_plan()
        ctx.log("armid plan", **rep)
        self.open_csv()
        res = dict(mode=self.mode, log=self.csv_path, plan=rep)
        try:
            if self.mode == "load":
                self.run_load(qc, res)
            elif self.mode in ("grav", "fric", "chirp", "hand"):
                getattr(self, "run_" + self.mode)(qc, res)
            else:
                self.start(qc, res)
                self.walk(qs, self.P["v_slow_rad_s"], "slow")
                self.walk(qs[::-1], self.P["v_fast_rad_s"], "fast")
                self.vmax = self.P["v_fast_rad_s"]
                self.go(qc, "to_centre", self.Pt)
                self.hold_ticks(self.P["hold_s"] * self.P["rate_hz"], "hold_centre")
                self.sweep(qc)
                res["result"] = "played"
        except Abort as ex:
            res["result"] = "aborted: %s" % ex
            self.finish(res)
            self.end()
            raise
        self.end()
        return self.finish(res)

    def dry(self, st, plate):
        """Print the plan; nothing moves."""
        self.t0 = time.time()
        self.base_F, self.base_pad = np.zeros(3), 0.0
        self.build(st)
        qs, qc, rep = self.check_plan()
        P = self.P
        d = np.abs(np.diff(np.array(qs), axis=0)).max(axis=1)
        T = sum(d) / P["v_slow_rad_s"] + sum(d) / P["v_fast_rad_s"] + 2 * len(qs) * P["hold_s"]
        Ts = sum(5 * (4 * P["hold_s"] + 4 * P["sweep_rad"] / v) for v in P["sweep_v_rad_s"])
        print("armid plan: %s" % rep)
        print("centre x %.3f y %+.3f z %.3f, floor z %.3f; about %.0f s corners + %.0f s single joints" % (
            P["centre_xy"][0], P["centre_xy"][1], P["rim_z"] + P["sweep_z_mm"] / 1000, P["rim_z"] + P["z_floor_mm"] / 1000, T, Ts))
        print("distance from where the arm is now to the start: %.2f rad at most" % float(np.max(np.abs(qc - self.qc))))
        print("command correction %s mrad; the hand %.0f mm above the contact height" % (P["q_correct_mrad"], P["sweep_z_mm"]))
        if self.mode == "grav":
            order = self.grav_plan(qc)
            pads = np.array([self.idx(q) for q in order])
            print("armid_grav: %d poses; the index pad x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f" % (len(order), pads[:, 0].min(), pads[:, 0].max(), pads[:, 1].min(),
                                                                                              pads[:, 1].max(), pads[:, 2].min(), pads[:, 2].max()))
            for k, q in enumerate(order):
                print("  %2d  q %s  pad %s" % (k, np.round(q, 3), np.round(pads[k], 3)))


__all__ = ["ArmId", "PADS", "SIDE"]
