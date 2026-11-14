"""Arm identification across the working range, one arm at a time, in free air. Poses and motions come from
params/armwide_plan.json (calibration/armwide_plan.py). The table is not checked: pull the robot back first.
"""
import csv
import json
import os
import time

import numpy as np

from .. import config
from ..ik import ArmChain
from ..stage_base import Abort
from .push10 import PAD_POINT


class ArmWide(object):
    name = "armwide"

    def __init__(self, ctx, mode="poses"):
        self.ctx, self.mode = ctx, mode
        self.P = ctx.params("armwide")
        self.side = self.P["side"]
        if self.side not in ("left", "right"):
            raise Abort("armwide.side must be left or right, not %r" % (self.side,))
        self.sl = config.LEFT if self.side == "left" else config.RIGHT
        self.hand_i = 0 if self.side == "left" else 6
        self.plan = json.load(open(os.path.join(config.PARAMS_DIR, "armwide_plan.json")))
        self.csv = None
        self.events = []
        self.checking = True

    # ================================================================== the plan
    def pose_ids(self):
        pl = self.plan[self.side]
        ids = list(pl["load_poses"]) if float(self.P["load_kg"]) > 0 else list(range(len(pl["poses"])))
        n = int(self.P["n_poses"]) or len(ids)
        return ids[:n]

    def poses(self):
        return [self.plan[self.side]["poses"][k] for k in self.pose_ids()]

    def joints(self):
        js = self.P["joints"] or list(range(5))
        return [self.plan[self.side]["joints"][j] for j in js]

    def motions(self):
        ms = [m for m in self.plan[self.side]["motions"] if m["speed"] <= self.P["max_speed"]]
        n = int(self.P["n_motions"]) or len(ms)
        return ms[:n]

    def check_start(self, st):
        off = np.abs(np.array(st["des"], float)[:10] - np.array(self.plan["home"], float)[:10])
        if float(np.max(off)) > self.P["home_tol_rad"]:
            raise Abort("armwide: the arms are %.2f rad from the plan's pre-grip pose, and the plan was checked from there. Remake it: calibration/armwide_plan.py" % float(np.max(off)))

    def build(self, st):
        rob = self.ctx.rob
        self.q11 = np.array(st["des"], float)
        self.hands = np.array(st["hands_cmd"], float)
        self.ch = ArmChain(rob, self.side, "%s_index_touch_link" % self.side, self.q11, self.hands)
        self.qc = self.q11[self.sl].copy()
        self.q_home = self.qc.copy()

    def idx(self, q5):
        T, _ = self.ch.fk(q5)
        return T[:3, :3] @ PAD_POINT + T[:3, 3]

    # ================================================================== the log
    def open_csv(self):
        cols = ["t", "phase"]
        for name in ("q_cmd", "q_des", "q_meas", "dq", "tau", "sent", "grav"):
            cols += ["%s%d" % (name, i) for i in range(5)]
        cols += ["hand%d" % i for i in range(6)] + ["idx_fk_x", "idx_fk_y", "idx_fk_z", "f_hand", "roll", "pitch"]
        self.csv_path = os.path.join(self.ctx.out_dir, "armwide_cmd.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols)
        self.csv.writeheader()
        self.csv_n = 0

    def row(self, st, phase):
        r = dict(t=round(time.time() - self.t0, 3), phase=phase)
        qm = np.array(st["q"], float)[self.sl]
        for name, v, nd in (("q_cmd", self.qc, 5), ("q_des", np.array(st["des"])[self.sl], 5), ("q_meas", qm, 5), ("dq", np.array(st["dq"])[self.sl], 4),
                            ("tau", np.array(st["tau"])[self.sl], 3), ("sent", np.array(st["sent"])[self.sl], 5), ("grav", np.array(st["grav"])[self.sl], 4)):
            for i in range(5):
                r["%s%d" % (name, i)] = round(float(v[i]), nd)
        for i in range(6):
            r["hand%d" % i] = round(float(st["hands"][self.hand_i + i]), 4)
        p = self.idx(qm)
        r.update(idx_fk_x=round(float(p[0]), 4), idx_fk_y=round(float(p[1]), 4), idx_fk_z=round(float(p[2]), 4))
        rpy = st.get("rpy") or [0, 0, 0]
        r.update(f_hand=st.get("f_hand", {}).get(self.side), roll=rpy[0], pitch=rpy[1])
        self.csv.writerow(r)
        self.csv_n += 1
        if self.csv_n % 30 == 0:
            self.csv_f.flush()

    def ev(self, what, **kw):
        kw["at_s"] = round(time.time() - self.t0, 2)
        self.events.append(dict(ev=what, **kw))
        self.ctx.log("armwide " + what, **kw)

    # ================================================================== motion
    def check(self, st):
        if not self.checking:
            return ""
        lag = np.abs(np.array(st["des"], float) - np.array(st["q"], float))[self.sl]
        if float(np.max(lag)) > self.P["lag_abort_rad"]:
            return "a joint is %.2f rad behind its plan: has the arm hit something?" % float(np.max(lag))
        return ""

    def tick(self, q_target, phase):
        """One command: the target moves at most vmax/rate toward q_target. Waits for the tick's end, checking; logs."""
        P, c = self.P, self.ctx.c
        dt = 1.0 / P["rate_hz"]
        self.qc = self.qc + np.clip(np.asarray(q_target, float) - self.qc, -self.vmax * dt, self.vmax * dt)
        self.q11[self.sl] = self.qc
        c.set(q=self.q11, hands=self.hands, tag="armwide " + phase)
        self.t_next += dt
        now = time.time()
        if self.t_next < now - 0.1:
            self.t_next = now
        why = c.hold(max(0.0, self.t_next - now), self.check)
        if why:
            raise Abort(why)
        st = self.ctx.st()
        self.row(st, phase)
        return st

    def go(self, q5, v, phase, tol=1e-4, max_s=60.0):
        self.vmax = float(v)
        q5 = np.asarray(q5, float)
        for _ in range(int(max_s * self.P["rate_hz"])):
            if float(np.max(np.abs(q5 - self.qc))) < tol:
                return
            self.tick(q5, phase)
        raise Abort("did not reach a pose in %.0f s" % max_s)

    def hold(self, seconds, phase):
        for _ in range(int(seconds * self.P["rate_hz"])):
            self.tick(self.qc, phase)

    # ================================================================== the tests
    def run_poses(self, res):
        P = self.P
        poses = self.poses()
        d = float(P["approach_rad"]) * np.ones(5)
        ids = self.pose_ids()
        hold = P["load_hold_s"] if float(P["load_kg"]) > 0 else P["hold_s"]
        for k, q in enumerate(poses):
            q = np.array(q, float)
            self.ev("pose", n=k, of=len(poses), plan_pose=ids[k])
            if float(P["load_kg"]) > 0 or self.plan[self.side].get("via_home", {}).get(str(ids[k])):   # load poses were each checked from the pre-grip pose only
                self.go(self.q_home, P["move_v_rad_s"], "gmove")
            self.go(q, P["move_v_rad_s"], "gmove")
            for name, sg in (("a", -1), ("b", 1)):
                self.go(q + sg * d, P["approach_v_rad_s"], "gpre")
                self.go(q, P["approach_v_rad_s"], "gpre")
                self.hold(hold, "g%d_%s" % (k, name))
        res["poses"] = len(poses)

    def run_joint(self, res):
        """Each joint alone. Every move stays inside the plan's checked +-range around the plan's centre; the way there is checked from the pre-grip pose."""
        P = self.P
        for n, jp in enumerate(self.joints()):
            j, c, r = int(jp["joint"]), np.array(jp["centre"], float), float(jp["range"])
            self.ev("joint", joint=j, n=n, of=len(self.joints()), range=r)
            self.go(self.q_home, P["move_v_rad_s"], "jmove")
            self.go(c, P["move_v_rad_s"], "jmove")
            self.hold(1.0, "jmove")
            self.ctx.c.mark()

            def at(x):
                q = c.copy()
                q[j] = c[j] + x
                return q
            for v, half in P["sweeps"]:                                  # constant speed: torque = gravity + friction(speed), no inertia
                h = min(float(half), r)
                self.go(at(-h), P["move_v_rad_s"], "jmove")
                self.hold(0.7, "jhold")
                for _ in range(int(P["sweep_reps"])):
                    self.go(at(h), v, "j%d_v%g_up" % (j, v))
                    self.hold(0.7, "jhold")
                    self.go(at(-h), v, "j%d_v%g_dn" % (j, v))
                    self.hold(0.7, "jhold")
            self.go(c, P["move_v_rad_s"], "jmove")
            self.hold(1.0, "jhold")
            for amp in P["rev_amps"]:                                    # back and forth: how far the command goes before the joint follows
                for _ in range(int(P["rev_cycles"])):
                    self.go(at(amp), P["rev_v_rad_s"], "j%d_r%g" % (j, amp))
                    self.go(at(-amp), P["rev_v_rad_s"], "j%d_r%g" % (j, amp))
                self.go(c, P["rev_v_rad_s"], "j%d_r%g" % (j, amp))
                self.hold(1.0, "jhold")
            for size in P["step_sizes"]:                                 # as fast as the server allows: delay and how the joint settles
                for sg in (1, -1):
                    self.go(at(sg * size), 0.78, "j%d_s%g" % (j, sg * size))
                    self.hold(P["step_hold_s"], "j%d_s%g" % (j, sg * size))
                    self.go(c, 0.78, "j%d_s%g" % (j, sg * size))
                    self.hold(P["step_hold_s"], "j%d_s%g" % (j, sg * size))
            self.ctx.c.trim()
        self.go(self.q_home, P["move_v_rad_s"], "jmove")
        res["joints"] = [int(jp["joint"]) for jp in self.joints()]

    def run_motion(self, res):
        P = self.P
        w = 2 * np.pi * np.array(self.plan["freq"], float)
        T = float(P["motion_s"]) or float(self.plan["motion_s"])
        rate = float(P["rate_hz"])
        last = None
        ms = self.motions()
        for n, m in enumerate(ms):
            c = np.array(m["centre"], float)
            if last is None or float(np.max(np.abs(c - last))) > 1e-6:
                if last is not None:
                    self.go(self.q_home, P["move_v_rad_s"], "mmove")     # every centre was checked from the pre-grip pose, not from the others
                self.go(c, P["move_v_rad_s"], "mmove")
                self.hold(1.0, "mmove")
                self.ctx.c.mark()
            last = c
            self.ev("motion", n=n, of=len(ms), speed=m["speed"], peak_v=m["peak_v"], peak_a=m["peak_a"])
            amp, ph = np.array(m["amp"], float), np.array(m["phase"], float)
            self.vmax = 0.78
            for k in range(int(T * rate)):
                t = k / rate
                f = min(1.0, min(t, T - t) / 2.0)
                f = f * f * (3 - 2 * f)
                self.tick(c + f * np.sum(amp * np.sin(w[None] * t + ph), axis=1), "m%d_v%g" % (n, m["speed"]))
            self.go(c, P["move_v_rad_s"], "mback")
            self.hold(1.0, "mback")
            self.ctx.c.trim()                                            # a stop later retraces the way here, not the wiggles
        res["motions"] = len(ms)

    def run(self):
        ctx = self.ctx
        self.t0 = time.time()
        st = ctx.st()
        self.check_start(st)
        self.build(st)
        self.open_csv()
        meta = dict(side=self.side, mode=self.mode, load_kg=float(self.P["load_kg"]), load_link="%s_wrist_roll_link" % self.side,
                    kp=[float(x) for x in config.KP[self.sl]], kd=[float(x) for x in config.KD[self.sl]], rate_hz=self.P["rate_hz"])
        json.dump(meta, open(os.path.join(ctx.out_dir, "armwide_meta.json"), "w"))
        res = dict(mode=self.mode, side=self.side, log=self.csv_path)
        ctx.c.set(vmax=0.78, hands=self.hands, tag="armwide")
        self.t_next = time.time()
        try:
            getattr(self, "run_" + self.mode)(res)
            res["result"] = "played"
        except Abort as ex:                                              # the server retraces the commanded path home, which was checked
            res["result"] = "aborted: %s" % ex
            self.finish(res)
            raise
        return self.finish(res)

    def finish(self, res):
        res["events"] = self.events
        if self.csv is not None:
            self.csv_f.flush()
        res["seconds"] = round(time.time() - self.t0, 1)
        self.ctx.results["armwide"] = res
        self.ctx.log("armwide", result=res.get("result"), log=self.csv_path, seconds=res["seconds"])
        return res

    def dry(self, st, plate):
        """Print the plan; nothing moves."""
        self.check_start(st)
        pl, P = self.plan[self.side], self.P
        if self.mode == "joint":
            T = 0.0
            for jp in self.joints():
                r = float(jp["range"])
                T += 12 + sum(2 * min(h, r) / 0.25 + P["sweep_reps"] * (4 * min(h, r) / v + 1.4) for v, h in P["sweeps"])
                T += sum(P["rev_cycles"] * 4 * a / P["rev_v_rad_s"] + 1 for a in P["rev_amps"]) + 4 * len(P["step_sizes"]) * (P["step_hold_s"] + 0.3)
                print("  joint %d: around plan pose %s, +-%.2f rad; centre %s" % (jp["joint"], jp["pose"], r, np.round(jp["centre"], 2)))
            print("armwide_joint %s arm: joints %s, about %.0f min. Sweeps (rad/s, half range rad) %s, back-and-forth %s rad, steps %s rad" % (
                self.side, [jp["joint"] for jp in self.joints()], T / 60, P["sweeps"], P["rev_amps"], P["step_sizes"]))
            return
        if float(P["load_kg"]) > 0:
            print("WITH A LOAD of %.3f kg hung over the %s wrist, just behind the hand: only the plan's load poses %s" % (float(P["load_kg"]), self.side, self.pose_ids()))
        poses = np.array(self.poses())
        pads = np.array(pl["pads"])[self.pose_ids()]
        prev = np.array(self.plan["home"], float)[self.sl]
        T = 0.0
        for q in poses:
            T += float(np.max(np.abs(q - prev))) / P["move_v_rad_s"] + 8 * P["approach_rad"] / P["approach_v_rad_s"] + 2 * P["hold_s"]
            if float(P["load_kg"]) > 0:
                T += 2 * float(np.max(np.abs(q - prev))) / P["move_v_rad_s"] + 2 * (P["load_hold_s"] - P["hold_s"])
            prev = q
        print("armwide %s arm: %d poses, about %.0f min; %d motions, %.0f s each" % (self.side, len(poses), T / 60, len(self.motions()), float(P["motion_s"]) or self.plan["motion_s"]))
        print("index pad x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f (pelvis frame). Checked %.0f mm from the body, legs and the other arm, in MuJoCo; the table is not checked" % (
            pads[:, 0].min(), pads[:, 0].max(), pads[:, 1].min(), pads[:, 1].max(), pads[:, 2].min(), pads[:, 2].max(), 1000 * self.plan["clear_m"]))
        for k, q in enumerate(poses):
            print("  %2d  q %s  pad %s" % (k, np.round(q, 2), pads[k]))
        for m in self.motions():
            print("  motion around pose %d: peak %.2f rad/s, %.2f rad/s^2, range %s rad" % (m["pose"], m["peak_v"], m["peak_a"], m["range"]))
