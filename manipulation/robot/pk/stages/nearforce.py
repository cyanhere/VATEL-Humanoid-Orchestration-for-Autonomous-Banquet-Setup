"""Near-rim force test: the robot-left hand lands index and middle on the top plate's near rim, presses, and ramps a
sideways force until the plate moves or the fingers slip. Each push is one recorded try.
"""
import csv
import math
import os
import sys
import threading
import time

import numpy as np

from .. import config
from ..ik import ArmChain
from ..stage_base import Abort
from .push10 import IDX, PAD_POINT, SIDE, Push10

MID = "left_middle_touch_link"
I_IDX, I_MID = 1, 2                     # in rt/brainco/<side>/touch
EXTRA = (["H", "Fz", "try", "press_target", "stop"]
         + ["%s_%s" % (f, w) for f in ("idx", "mid") for w in ("normal", "tang", "dir", "prox", "count")]
         + ["mk_ok", "mk_x", "mk_y", "mk_angle", "mk_px", "cam_ok", "cam_cov", "cam_n", "cam_rms"]
         + ["mid_fk_x", "mid_fk_y", "mid_fk_z", "roll", "pitch", "waist_roll", "pelvis_m"]
         + ["q_tgt%d" % i for i in range(5)] + ["sent%d" % i for i in range(5)] + ["dq%d" % i for i in range(5)] + ["tau%d" % i for i in range(5)])


def dls(J, dx, lam=1e-3):
    """The joint step for a small hand step: damped, so that the near-singular shoulder cannot ask for a large one."""
    return J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), dx)


class NearForce(Push10):
    name = "nearforce"

    def __init__(self, ctx, mode="full"):
        ctx.overrides = dict(ctx.overrides, **{"push10.table_m": 0.747})     # push10's pose table is not used here
        Push10.__init__(self, ctx, mode)
        self.P = dict(self.P, **ctx.params("nearforce"))
        P = self.P
        if mode == "tip":
            P["n"] = 1
        key = "n%d_t%.3f" % (int(P["n"]), float(P["table_m"]))
        if key not in P["pose_by_stack"]:
            raise Abort("nearforce: no seed pose for %s (have %s)" % (key, sorted(P["pose_by_stack"])))
        self.pose = dict(P["pose_by_stack"][key], di_mm=0.0)
        self.corr = np.array(P["q_correct_mrad"], float) / 1000
        self.H, self.Fz, self.try_n, self.target = 0.0, 0.0, 0, 0.0
        self.base_pads = np.zeros(2)
        self.stop_reason = ""
        self.q_tgt = None
        self.land, self.landing_out, self.how = {}, {}, None
        self.follow = np.zeros(3)              # how far the touch point has followed the plate in this push
        self.ref = {}
        self.u = np.array([math.sin(math.radians(P["push_deg"])), -math.cos(math.radians(P["push_deg"])), 0.0])

    # ================================================================== kinematics
    def build(self, st):
        Push10.build(self, st)
        self.ch_mid = ArmChain(self.ctx.rob, SIDE, MID, self.q11, self.hands)

    def mid_pad(self, q5):
        T, _ = self.ch_mid.fk(q5)
        return T[:3, :3] @ PAD_POINT + T[:3, 3]

    def jac(self, q5):
        """Linear Jacobian at the index pad."""
        T, fr = self.ch.fk(q5)
        p = T[:3, :3] @ PAD_POINT + T[:3, 3]
        return np.array([np.cross(z, p - o) for o, z in fr]).T

    # ================================================================== the plan
    def setup(self, need_plate=True):
        Push10.setup(self, need_plate)
        self.base_z = float(self.z_edge - self.spec.z_top(self.spec.R))      # the plate's foot (camera)
        self.touch_pt = self.plan_touch()
        if self.mode == "tip":
            p = self.ctx.plate(max_age=5.0) or {}
            n = (p.get("located") or {}).get("n_plates")
            if n is not None and n != 1:
                raise Abort("nearforce_tip needs ONE plate on the table: the camera reads %s" % n)

    def plan_touch(self):
        P = self.P
        return np.array([self.cx + P["pad_rel_plate_mm"][0] / 1000, self.cy + P["pad_rel_plate_mm"][1] / 1000,
                         self.base_z + P["pad_above_foot_mm"] / 1000])

    def plan_points(self):
        pt = self.touch_pt
        return [("high", pt + np.array([0, 0, self.P["above_rim_mm"] / 1000]), self.P["w_rot"]),
                ("hover", pt + np.array([0, 0, self.P["hover_mm"] / 1000]), self.P["w_rot"]),
                ("touch", pt, self.P["w_rot"])]

    def solve_plan(self):
        """IK up from the simulation's contact joints in small steps, so the arm stays on the planned branch (a direct solve
        went to another branch in the simulation: shoulder pitch on its limit, hand 20 mm high). Returns q at touch, hover, high."""
        P = self.P
        q_touch, e_touch, d_touch = self.ik(self.touch_pt, self.q_plan, P["w_rot"])
        if e_touch > P["ik_max_mm"]:
            raise Abort("nearforce: the touch point is out of reach (IK %.0f mm off): is the stack at x %.2f y %.2f?" % (
                e_touch, config.STACK_XY_SIM[0], config.STACK_XY_SIM[1]))
        q, out = q_touch.copy(), {}
        for name, h in (("hover", P["hover_mm"]), ("high", P["above_rim_mm"])):
            for u in np.linspace(0.0, 1.0, int(P["ik_up_steps"]) + 1)[1:]:
                z0 = 0.0 if name == "hover" else P["hover_mm"]
                q, e, d = self.ik(self.touch_pt + np.array([0, 0, (z0 + u * (h - z0)) / 1000]), q, P["w_rot"])
            out[name] = (q.copy(), e, d)
        self.ctx.log("nearforce plan", touch=self.touch_pt, ik_touch_mm=round(e_touch, 1), palm_deg=round(d_touch, 1), ik_hover_mm=round(out["hover"][1], 1),
                     ik_high_mm=round(out["high"][1], 1), q_touch=q_touch, q_high=out["high"][0])
        return q_touch, out["hover"][0], out["high"][0]

    # ================================================================== sensing and the log
    def open_csv(self, extra_cols=()):
        Push10.open_csv(self, EXTRA)
        self.csv_f.close()
        os.remove(self.csv_path)
        cols = self.csv.fieldnames
        self.csv_path = os.path.join(self.ctx.out_dir, "nearforce_cmd.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols, extrasaction="ignore")
        self.csv.writeheader()

    def sense(self, st):
        s = Push10.sense(self, st)
        t = st.get("touch")
        s["raw"] = t
        s["pi"], s["pm"] = s["pads"][I_IDX] - self.base_pads[0], s["pads"][I_MID] - self.base_pads[1]
        s["prox_m"] = float(t[0][I_MID][3]) if t else 0.0
        p = self.ctx.plate(st, max_age=0.5)
        s["plate_msg"] = p
        mk = p.get("marker") if p else None
        s["mk"] = np.array(mk["xy"], float) if mk and mk.get("ok") else None
        s["mk_msg"] = mk
        if s["mk"] is not None:
            self.cam_t = time.time()
        return s

    def row(self, st, s, phase, P, ik, extra):
        e = dict(extra or {})
        e.update(H=round(self.H, 3), Fz=round(self.Fz, 3), press_target=self.target, stop=self.stop_reason)
        e["try"] = self.try_n
        t = s["raw"]
        for name, i in (("idx", I_IDX), ("mid", I_MID)):
            if t:
                e["%s_normal" % name], e["%s_tang" % name], e["%s_dir" % name], e["%s_prox" % name] = [round(float(v), 1) for v in t[0][i][:4]]
                e["%s_count" % name] = round(float(t[0][i][0]), 1)
        mk, p = s["mk_msg"], s["plate_msg"]
        if mk and mk.get("ok"):
            e.update(mk_ok=1, mk_x=mk["xy"][0], mk_y=mk["xy"][1], mk_angle=mk.get("angle_deg"), mk_px=mk.get("dark_px"))
        else:
            e["mk_ok"] = 0
        if p:
            e.update(cam_ok=int(bool(p.get("ok"))), cam_cov=p.get("coverage"), cam_n=p.get("n"), cam_rms=p.get("rms_mm"))
        m = self.mid_pad(s["qm"])
        e.update(mid_fk_x=round(float(m[0]), 4), mid_fk_y=round(float(m[1]), 4), mid_fk_z=round(float(m[2]), 4))
        rpy = st.get("rpy") or [0, 0, 0]
        e.update(roll=rpy[0], pitch=rpy[1], waist_roll=st.get("waist_roll"), pelvis_m=round(self.ctx.pelvis(st), 4))
        sent, dq, tau = np.array(st["sent"])[config.LEFT], np.array(st["dq"])[config.LEFT], np.array(st["tau"])[config.LEFT]
        for i in range(5):
            e["sent%d" % i], e["dq%d" % i], e["tau%d" % i] = round(float(sent[i]), 5), round(float(dq[i]), 4), round(float(tau[i]), 3)
            if self.q_tgt is not None:
                e["q_tgt%d" % i] = round(float(self.q_tgt[i]), 5)
        Push10.row(self, st, s, phase, P, ik, e)

    def tick(self, q_target, phase, P=None, ik=(0.0, 0.0), extra=None):
        """What the arm is asked to reach + the steady error it would otherwise sit off by. Pad limit: the stage's."""
        self.q_tgt = np.array(q_target, float)
        s = Push10.tick(self, self.q_tgt + self.corr, phase, P, ik, extra)
        if self.checking and max(s["pi"], s["pm"]) > self.P["stop_pad_max_N"]:
            raise Abort("a pad reads %.1f N (index %.1f, middle %.1f; limit %.0f)" % (max(s["pi"], s["pm"]), s["pi"], s["pm"], self.P["stop_pad_max_N"]))
        return s

    def catch_up(self, phase, tol=0.002, max_s=3.0):
        for _ in range(int(max_s * self.P["rate_hz"])):
            if float(np.max(np.abs(self.q + self.corr - self.qc))) < tol:
                break
            self.tick(self.q, phase, self.Pt)

    def ev(self, what, **kw):
        kw["at_s"] = round(time.time() - self.t0, 2)
        self.events.append(dict(ev=what, **kw))
        self.ctx.log("nearforce " + what, **kw)

    # ================================================================== the operator
    def start_input(self):
        self.typed = []
        if self.P.get("auto_answer") is not None:
            return

        def read():
            for line in sys.stdin:
                self.typed.append(line.strip().lower())
        threading.Thread(target=read, daemon=True).start()

    def ask(self, question, phase):
        """y / n typed in this terminal while the arm keeps being commanded. No answer in time: no."""
        P = self.P
        if P.get("auto_answer") is not None:
            self.ev("question", q=question, answer=P["auto_answer"], auto=True)
            return bool(P["auto_answer"])
        del self.typed[:]
        print("\n>>> %s  Type y or n, then Enter.\n" % question, flush=True)
        for _ in range(int(P["confirm_timeout_s"] * P["rate_hz"])):
            self.tick(self.q, phase, self.Pt)
            if self.typed:
                a = self.typed.pop(0)
                self.ev("question", q=question, answer=a[:1] == "y")
                return a[:1] == "y"
        self.ev("question", q=question, answer=False, why="no answer")
        return False

    # ================================================================== 1 approach
    def go_pre(self):
        ctx, P = self.ctx, self.P
        with ctx.c.busy(20.0):                       # IK and a sweep of FK: longer than the server's 0.6 s client time-out when the CPU is busy
            q_touch, q_hover, q_high = self.solve_plan()
            self.q_hover = q_hover
            z_low, r_low = self.sweep_low(self.qc, q_high)
        if r_low < self.spec.R + 0.04 and z_low < self.z_edge + P["pre_clear_mm"] / 1000:
            raise Abort("the move to the start would pass %.0f mm above the rim, %.0f mm from the stack centre" % (1000 * (z_low - self.z_edge), 1000 * r_low))
        ctx.c.set(hands=self.hands, tag="nearforce shape")
        why = ctx.c.hold(1.0)
        if why:
            raise Abort(why)
        q11 = self.q11.copy()
        q11[config.LEFT] = q_high + self.corr
        why = ctx.c.move_to(q11, self.hands, v=P["pre_v_rad_s"], tag="nearforce pre")
        if why:
            raise Abort("move to the start: %s" % why)
        ctx.c.set(vmax=P["vmax_rad_s"])
        ctx.c.hold(0.5)
        self.q11, self.qc, self.q = q11, (q_high + self.corr).copy(), q_high.copy()
        self.Pt = self.plan_points()[0][1].copy()
        ctx.c.mark()
        self.q_mark = q11.copy()
        self.t_next = time.time()
        self.checking = False
        self.vmax = P["vmax_free_rad_s"]
        hover = self.plan_points()[1][1]
        low = float(hover[2]) - P["hover_low_mm"] / 1000

        def too_low(s, P_):
            return "low" if s["z"] < low else None
        how, s = self.line(hover, P["speed_hover"], "pre", stop=too_low)
        if how:
            raise Abort("on the way to the hover the hand went %.0f mm below it: stopped before the plate (slower with --set nearforce.speed_hover=...)" % (1000 * (float(hover[2]) - s["z"])))
        self.catch_up("pre")
        self.vmax = P["vmax_rad_s"]
        self.tare("pre")
        self.checking = True

    def tare(self, phase):
        """Settle, then the pads' and the joint-lag force's zero and how far the joints sit from the command, standing still above the plate."""
        P = self.P
        n = int(P["settle_s"] * P["rate_hz"])
        self.hold_ticks(n, phase)
        Fs, pads, qs = [], [], []
        for _ in range(n):
            s = self.tick(self.q, phase, self.Pt)
            Fs.append(s["F_raw"])
            pads.append([s["pads"][I_IDX], s["pads"][I_MID]])
            qs.append(s["qm"])
        self.base_F = np.median(np.array(Fs), axis=0)
        self.base_pads = np.median(np.array(pads), axis=0)
        self.e_free = self.q - np.median(np.array(qs), axis=0)
        self.ctx.log("nearforce tare", lag_force=self.base_F, pads_N=self.base_pads, q_error_mrad=np.round(1000 * self.e_free, 1))
        self.z_hover_err = 1000 * float(self.idx(s["qm"])[2] - self.Pt[2])

    # ================================================================== 2 descend
    def touched_fn(self):
        """Proximity (or a pad, if the rim reaches its sensor) held for touch_ticks; free air reached 16000 for 3 ticks on the R1, contact passed 30000
        on both fingers within 0.2 s. Ends with both fingers, or touch_both_mm of command after the first."""
        P = self.P
        k = int(P["touch_ticks"])
        n = [0, 0]
        self.land = {}
        first = [None]
        z_max = float(self.touch_pt[2]) + P["touch_z_max_mm"] / 1000

        def touched(s, P_):
            mid = self.mid_pad(s["qm"])
            for i, (name, prox, pad, z) in enumerate((("index", s["prox"], s["pi"], s["z"]), ("middle", s["prox_m"], s["pm"], float(mid[2])))):
                n[i] = n[i] + 1 if s["z"] < z_max and (prox >= P["touch_prox"] or pad > P["contact_N"]) else 0
                if n[i] >= k and name not in self.land:
                    self.land[name] = dict(z_mm=round(1000 * z, 2), prox=round(prox), at_s=round(time.time() - self.t0, 2))
                    first[0] = first[0] if first[0] is not None else float(P_[2])
            if len(self.land) == 2:
                return "both"
            if first[0] is not None and 1000 * (first[0] - float(P_[2])) > P["touch_both_mm"]:
                return "one finger"
            return None
        return touched

    def descend(self):
        """Down until touched. Returns how."""
        P = self.P
        low = self.touch_pt - np.array([0, 0, P["below_rim_mm"] / 1000])
        how, s = self.line(low, P["speed_descend"], "touch", stop=self.touched_fn())
        if not how:
            return None, None
        self.ev("touch", by=how, fingers=self.land, z_vs_plan_mm=round(1000 * (s["z"] - self.touch_pt[2]), 1), prox=[s["prox"], s["prox_m"]], lead_mm=round(1000 * (s["z"] - float(self.Pt[2])), 1))
        return how, s

    # ================================================================== 3 zero
    def zero_load(self):
        """The command back, slowly, to where the joints would sit from it in free air. From here on: q_base + the force law."""
        P = self.P
        st = self.ctx.st()
        qm = np.array(st["q"], float)[config.LEFT]
        q1, q0 = qm + self.e_free, self.q.copy()
        z0 = float(self.idx(qm)[2])
        n = max(1, int(P["zero_s"] * P["rate_hz"]))
        for k in range(1, n + 1):
            self.q = q0 + (q1 - q0) * k / n
            self.tick(self.q, "zero", self.Pt)
        s = self.hold_ticks(P["zero_settle_s"] * P["rate_hz"], "zero")
        st = self.ctx.st()
        self.q_base, self.J0 = self.q.copy(), self.jac(self.q)
        self.tau0 = np.array(st["tau"], float)[config.LEFT]
        self.follow = np.zeros(3)
        self.P_contact = self.idx(s["qm"]).copy()
        self.Pt = self.P_contact.copy()
        self.z_contact = s["z"]
        self.ev("zero load", command_back_mrad=np.round(1000 * (q1 - q0), 1).tolist(), hand_rose_mm=round(1000 * (s["z"] - z0), 2), prox=[s["prox"], s["prox_m"]])
        return s

    def landing(self, s):
        """Where the fingers are on the plate: for the simulation, and how far out on the rim they land."""
        out = dict(plan_xy_mm=self.P["pad_rel_plate_mm"], plate_xy_mm=[round(1000 * self.cx, 1), round(1000 * self.cy, 1)])
        for name, p in (("index", self.idx(s["qm"])), ("middle", self.mid_pad(s["qm"]))):
            d = p[:2] - np.array([self.cx, self.cy])
            r = float(np.linalg.norm(d))
            surf = self.base_z + float(self.spec.z_top(min(r, self.spec.R)))
            at = self.land.get(name, {})
            out[name] = dict(xy_mm=np.round(1000 * d, 1).tolist(), r_mm=round(1000 * r, 1), edge_gap_mm=round(1000 * (self.spec.R - r), 1), z_mm=round(1000 * float(p[2]), 1),
                             surface_z_mm=round(1000 * surf, 1), above_surface_mm=round(1000 * (float(p[2]) - surf), 1),
                             above_surface_at_touch_mm=round(at["z_mm"] - 1000 * surf, 1) if at else None)
        self.landing_out = out
        self.ev("landing", **out)
        return out

    def contact(self, confirm=True):
        """Hover -> fingers on the rim, the load taken off. Raises Abort when not."""
        how, _ = self.descend()
        if not how:
            raise Abort("nothing touched down to %.0f mm below the expected rim" % self.P["below_rim_mm"])
        self.how = how
        self.in_contact = True
        s = self.zero_load()
        L = self.landing(s)
        if (confirm and self.P["operator_confirm"]) or how != "both":
            lead = "" if how == "both" else "Proximity read only the %s. " % "/".join(self.land)
            if not self.ask("%sAre both fingertips on the rim, as in figures/contact_pose.png? By the arm: index %.0f mm, middle %.0f mm in from the plate's edge. Measure the gap with a ruler now." % (
                    lead, L["index"]["edge_gap_mm"], L["middle"]["edge_gap_mm"]), "contact"):
                raise Abort("the operator did not confirm the contact")
        return self.hold_ticks(3, "contact")

    # ================================================================== force commands
    def force_tick(self, phase):
        """One force tick: q at zero + a small step for how far the plate has gone + Kq^-1 J^T F."""
        self.q = self.q_base + dls(self.J0, self.follow)
        st = self.ctx.st()
        qm = np.array(st["q"], float)[config.LEFT]
        F = self.H * self.u + np.array([0.0, 0.0, -self.Fz])
        dq = (self.jac(qm).T @ F) / np.array(st["kp"], float)[config.LEFT]
        self.Pt = self.P_contact + self.follow
        rise = float(np.max(np.abs(np.array(st["tau"], float)[config.LEFT] - self.tau0)))
        if self.checking and rise > self.P["stop_tau_rise_Nm"]:
            raise Abort("a joint's torque rose %.1f N m since zero (limit %.1f)" % (rise, self.P["stop_tau_rise_Nm"]))
        return self.tick(self.q + dq, phase, self.Pt)

    def zero_plate(self, s):
        """From here on plate_move() is the travel since now."""
        self.ref = {}
        self.plate_move(s)

    def plate_move(self, s):
        """The plate's travel in the plane (m): the marker's when it has one (a test reference), else the rim tracker's. The two
        are different points on the plate: never compared, each against its own start."""
        cam = s["cam"][:2] if s["cam"] is not None else None
        if cam is not None and "cam" not in self.ref:
            self.ref["cam"] = cam - (s["mk"] - self.ref["mk"] if s["mk"] is not None and "mk" in self.ref else 0.0)
        if s["mk"] is not None:
            if "mk" not in self.ref:
                self.ref["mk"] = s["mk"] - (cam - self.ref["cam"] if cam is not None and "cam" in self.ref else 0.0)
            return s["mk"] - self.ref["mk"]
        return cam - self.ref["cam"] if cam is not None else None

    def press_stop(self, s):
        """The press's stops, each with its event. True: stop."""
        P = self.P
        d = self.plate_move(s)
        sink = 1000 * (self.z_contact - s["z"])
        if d is not None and 1000 * np.linalg.norm(d) > P["press_moved_mm"]:
            self.stop_reason = "moved in press"
            self.ev("the plate moved during the press", moved_mm=round(1000 * float(np.linalg.norm(d)), 1), Fz=round(self.Fz, 2), prox=[s["prox"], s["prox_m"]])
            return True
        if sink > P["stop_sink_mm"]:
            self.stop_reason = "sink"
            self.ev("the hand sank during the press", sink_mm=round(sink, 1), Fz=round(self.Fz, 2), prox=[s["prox"], s["prox_m"]])
            return True
        return False

    def press(self, target):
        """Fz up at fz_rate to the target, commanded; settle. Returns (the last sensors, ok)."""
        P = self.P
        step = P["fz_rate_N_s"] / P["rate_hz"]
        self.target = float(target)
        self.Fz = 0.0
        s = self.force_tick("press")
        self.zero_plate(s)
        target = min(float(target), P["fz_max_N"])
        for k in range(int(target / step) + int(P["press_settle_s"] * P["rate_hz"])):
            self.Fz = min(self.Fz + step, target)
            s = self.force_tick("press")
            if self.press_stop(s):
                return s, False
        self.ev("pressed", target=target, Fz=round(self.Fz, 2), prox=[s["prox"], s["prox_m"]], normal_counts=[s["pi"] / P["pad_N_per_count"], s["pm"] / P["pad_N_per_count"]],
                lag_Fz=round(float(s["F"][2]), 2), sink_mm=round(1000 * (self.z_contact - s["z"]), 2), pelvis_m=round(self.ctx.pelvis(), 4))
        return s, True

    def unload(self, phase="unload"):
        """H, then Fz, to 0."""
        P = self.P
        step = P["unload_N_s"] / P["rate_hz"]
        while self.H > 0 or self.Fz > 0:
            if self.H > 0:
                self.H = max(0.0, self.H - step)
            else:
                self.Fz = max(0.0, self.Fz - step)
            self.force_tick(phase)

    # ================================================================== one push
    def push(self):
        """H ramps until a stop. Returns the try's result and the numbers at the stop."""
        P = self.P
        dt = 1.0 / P["rate_hz"]
        s0 = self.force_tick("settled")
        self.zero_plate(s0)
        if self.plate_move(s0) is None:
            raise Abort("no plate estimate at the start of the push")
        hand0 = self.idx(s0["qm"])
        both = min(s0["prox"], s0["prox_m"]) >= P["touch_prox"]
        lost = 0
        t0 = time.time()
        self.H = 0.0
        s = s0
        res = None
        moved = hand = 0.0
        while res is None:
            self.H += P["h_rate_N_s"] * dt
            s = self.force_tick("push")
            self.cam_fresh()
            d = self.plate_move(s)
            if d is not None:
                moved = 1000 * float(d @ self.u[:2])
                self.follow[:2] = d
            hand = 1000 * float((self.idx(s["qm"]) - hand0)[:2] @ self.u[:2])
            lost = lost + 1 if both and min(s["prox"], s["prox_m"]) < P["touch_prox"] else 0
            if moved >= P["stop_moved_mm"]:
                res = "moved"
            elif hand - moved > P["stop_slip_mm"]:
                res = "slip"
            elif lost >= P["stop_lost_ticks"]:
                res = "finger_lost"
            elif 1000 * (self.z_contact - s["z"]) > P["stop_sink_mm"]:
                res = "sink"
            elif self.H >= P["h_cap_N"] or time.time() - t0 > P["try_timeout_s"]:
                res = "cap"
        self.stop_reason = res
        return res, s, moved, hand

    def sheet(self, res, s, moved, s_press):
        """One line per try, the columns of the robot's record sheet."""
        tg = s["raw"][0] if s["raw"] else [[0, 0]] * 5
        d = dict(plates=self.P["n"], table_m=self.P["table_m"], press_target_N=self.target,
                 idx_prox_at_push_start=s_press["prox"], mid_prox_at_push_start=s_press["prox_m"], result=res,
                 commanded_push_N_at_result=round(self.H, 2), lag_push_N_at_result=round(-float(s["F"][:2] @ self.u[:2]), 2),
                 idx_tangential_counts_at_result=tg[I_IDX][1], mid_tangential_counts_at_result=tg[I_MID][1], plate_moved_mm=round(moved, 1),
                 source="marker" if s["mk"] is not None else "rim tracker")
        self.ctx.log("nearforce TRY", **d)
        return d

    # ================================================================== lift off
    def lift(self):
        P = self.P
        self.checking = False
        self.in_contact = False
        self.vmax = P["vmax_free_rad_s"]
        self.q_tgt = None
        if self.H > 0 or self.Fz > 0:
            self.unload()
        self.up(P["lift_mm"], P["speed_release"], "release")
        self.hands[:6] = 0.0
        self.hold_ticks(P["release_hold_s"] * P["rate_hz"], "release")
        self.back_to_mark()

    def up(self, mm, speed, phase):
        """Straight up in small joint steps from where the command is: no IK solve, which at contact can ask for a jump."""
        K = max(1, int(mm / speed * self.P["rate_hz"]))
        dz = np.array([0.0, 0.0, mm / 1000 / K])
        s = None
        for _ in range(K):
            self.q = self.q + dls(self.jac(self.q), dz)
            self.Pt = self.Pt + dz
            s = self.tick(self.q, phase, self.Pt)
        return s

    def finish(self, res):
        res["events"] = self.events
        res["seconds"] = round(time.time() - self.t0, 1)
        return Push10.finish(self, res)

    # ================================================================== the modes
    def run(self):
        ctx, P = self.ctx, self.P
        self.setup()
        st = ctx.st()
        if not st.get("touch"):
            raise Abort("no fingertip data (rt/brainco/left/touch): start the touch-publishing hand server first (./server.sh)")
        self.start_input()
        self.open_csv()
        res = dict(mode=self.mode, plates=P["n"], table=P["table_m"], press_N=P["press_N"], log=self.csv_path, plate=[self.cx, self.cy, self.z_edge],
                   q_correct_mrad=P["q_correct_mrad"], pad_N_per_count=P["pad_N_per_count"])
        try:
            self.go_pre()
            res["hover_z_error_mm"] = round(self.z_hover_err, 1)
            res.update(getattr(self, "run_" + self.mode)())
        except Abort as ex:
            res["result"] = "aborted: %s" % ex
            self.stop_reason = self.stop_reason or "abort"
            self.finish(res)
            if self.q_mark is not None:
                ctx.log("nearforce", note="lifting off and back to the marked pose")
                try:
                    self.lift()
                except Exception as ex2:                          # noqa: BLE001
                    ctx.log("nearforce", note="could not lift off cleanly: %r" % ex2)
            raise
        self.lift()
        return self.finish(res)

    def run_shape(self):
        P = self.P
        st = self.ctx.st()
        self.checking = False
        qm = np.array(st["q"], float)[config.LEFT]
        out = dict(index_pad=self.idx(qm), middle_pad=self.mid_pad(qm), middle_minus_index_z_mm=round(1000 * float(self.mid_pad(qm)[2] - self.idx(qm)[2]), 1),
                   hands_measured=np.array(st["hands"])[:6], hands_commanded=self.hands[:6])
        self.ctx.log("nearforce shape", note="LOOK at the hand now (%.0f s), 20 mm above the rim" % P["shape_hold_s"], **out)
        self.hold_ticks(P["shape_hold_s"] * P["rate_hz"], "shape")
        return dict(result="shape held", **{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in out.items()})

    def run_touch(self):
        s = self.contact()
        s = self.hold_ticks(self.P["touch_hold_s"] * self.P["rate_hz"], "contact")
        return dict(result="touch by %s" % self.how, landing=self.landing_out, prox=[s["prox"], s["prox_m"]], z_vs_plan_mm=round(1000 * (self.z_contact - self.touch_pt[2]), 1))

    def run_press(self):
        """A ladder of commanded presses, up and down: how far the hand gives (arm and finger), proximity against force."""
        P = self.P
        self.contact()
        s = self.force_tick("press")
        self.zero_plate(s)
        step = P["fz_rate_N_s"] / P["rate_hz"]
        ladder, stopped = [], False
        for lv in P["press_levels_N"]:
            self.target = min(float(lv), P["fz_max_N"])
            while abs(self.Fz - self.target) > 1e-6 and not stopped:
                self.Fz = self.Fz + float(np.clip(self.target - self.Fz, -step, step))
                s = self.force_tick("press")
                stopped = self.press_stop(s)
            z, pi, pm = [], [], []
            for _ in range(int(P["press_step_hold_s"] * P["rate_hz"])):
                if stopped:
                    break
                s = self.force_tick("hold")
                stopped = self.press_stop(s)
                z.append(s["z"])
                pi.append(s["prox"])
                pm.append(s["prox_m"])
            if stopped:
                break
            ladder.append(dict(Fz=self.target, sink_mm=round(1000 * (self.z_contact - float(np.median(z))), 2), prox=[round(float(np.median(pi))), round(float(np.median(pm)))]))
            self.ev("level", **ladder[-1])
        self.unload()
        return dict(result=self.stop_reason if stopped else "pressed", ladder=ladder, landing=self.landing_out)

    def run_full(self):
        P = self.P
        self.try_n = 1
        self.contact()
        s_press, ok = self.press(P["press_N"])
        if not ok:
            self.unload()
            return dict(result=self.stop_reason, landing=self.landing_out)
        res, s, moved, hand = self.push()
        self.ev(res, H=round(self.H, 2), Fz=round(self.Fz, 2), moved_mm=round(moved, 1), hand_mm=round(hand, 1), lag_F=np.round(s["F"], 2).tolist(), prox=[s["prox"], s["prox_m"]])
        d = self.sheet(res, s, moved, s_press)
        H = round(self.H, 2)
        self.unload()
        return dict(result=res, sheet=d, moved_mm=round(moved, 1), hand_mm=round(hand, 1), H=H, landing=self.landing_out)

    def run_tip(self):
        """One plate alone: the press ramps until it tips. Lever from where the fingers are; force from the plate's weight."""
        P = self.P
        dt = 1.0 / P["rate_hz"]
        self.contact()
        s = self.force_tick("tip")
        self.zero_plate(s)
        z0 = s["z"]
        self.Fz = 0.0
        n, how = 0, None
        while how is None:
            self.Fz += P["tip_rate_N_s"] * dt
            s = self.force_tick("tip")
            d = self.plate_move(s)
            n = n + 1 if d is not None and 1000 * np.linalg.norm(d) > P["tip_marker_mm"] else 0
            if n >= 3:
                how = "marker"
            elif 1000 * (z0 - s["z"]) > P["tip_sink_mm"]:
                how = "sink"
            elif self.Fz >= P["tip_cap_N"]:
                how = "cap"
        self.stop_reason = "tip by %s" % how
        ri = float(np.linalg.norm(self.idx(s["qm"])[:2] - np.array([self.cx, self.cy])))
        rm = float(np.linalg.norm(self.mid_pad(s["qm"])[:2] - np.array([self.cx, self.cy])))
        r = 500 * (ri + rm)
        F_true = P["plate_N"] * P["foot_r_mm"] / max(r - P["foot_r_mm"], 1.0)
        out = dict(result=how, Fz_at_tip=round(self.Fz, 2), prox_at_tip=[s["prox"], s["prox_m"]], normal_counts_at_tip=[s["raw"][0][I_IDX][0], s["raw"][0][I_MID][0]], lag_Fz_at_tip=round(float(s["F"][2]), 2),
                   lever_r_mm=round(r, 1), lever_r_index_mm=round(1000 * ri, 1), lever_r_middle_mm=round(1000 * rm, 1), force_from_plate_N=round(F_true, 2),
                   true_over_commanded=round(F_true / max(self.Fz, 0.1), 2), sink_mm=round(1000 * (z0 - s["z"]), 1), landing=self.landing_out)
        self.ev("tip", **out)
        self.unload()
        return out

    def run_slip(self):
        """The plate taped down: press levels, each: push ramp until the fingers slide over the glaze; lift, back, again."""
        P = self.P
        tries, first = [], True
        for press_N in [v for v in P["slip_press_N"] for _ in range(int(P["slip_repeats"]))]:
            self.try_n += 1
            self.stop_reason = ""
            self.H, self.Fz = 0.0, 0.0
            if not first:
                self.checking = False
                self.tare("pre")
                self.checking = True
            self.contact(confirm=first)
            first = False
            s_press, ok = self.press(press_N)
            if not ok:
                tries.append(dict(press=press_N, result=self.stop_reason))
                self.unload()
                self.back_up(P)
                continue
            s0 = self.force_tick("settled")
            self.zero_plate(s0)
            hand0 = self.idx(s0["qm"])
            res = None
            H1, H3 = None, None
            dt = 1.0 / P["rate_hz"]
            while res is None:
                self.H += P["h_rate_N_s"] * dt
                s = self.force_tick("push")
                self.cam_fresh()
                d = self.plate_move(s)
                mv = 1000 * float(np.linalg.norm(d)) if d is not None else 0.0
                hand = 1000 * float((self.idx(s["qm"]) - hand0)[:2] @ self.u[:2])
                H1 = round(self.H, 2) if H1 is None and hand > 1.0 else H1
                H3 = round(self.H, 2) if H3 is None and hand > 3.0 else H3
                if mv > P["slip_plate_moved_mm"]:
                    res = "plate moved"
                elif hand > P["slip_stop_mm"]:
                    res = "slip"
                elif self.H >= P["h_cap_N"]:
                    res = "cap"
            t = dict(press=press_N, result=res, H=round(self.H, 2), H_hand_1mm=H1, H_hand_3mm=H3, prox_press=[s_press["prox"], s_press["prox_m"]], prox_end=[s["prox"], s["prox_m"]],
                     hand_mm=round(hand, 1), plate_mm=round(mv, 1), landing=self.landing_out)
            self.stop_reason = res
            self.ev("slip try", **t)
            tries.append(t)
            self.unload()
            if res == "plate moved":
                raise Abort("the taped plate moved %.1f mm: tape it better" % mv)
            self.back_up(P)
        return dict(result="played", tries=tries, landing=self.landing_out)

    def back_up(self, P):
        """Off the rim, above the hover, and down to the hover in joint space: the next landing comes from above, as the first."""
        self.checking = False
        self.in_contact = False
        self.vmax = P["vmax_free_rad_s"]
        self.up(P["slip_lift_mm"], P["speed_release"], "retry")
        self.q = self.q_hover.copy()
        self.Pt = self.plan_points()[1][1].copy()
        self.catch_up("retry", max_s=5.0)
        self.vmax = P["vmax_rad_s"]
        self.checking = True

    # ================================================================== plan only
    def dry(self, st, plate):
        P = self.P
        self.t0 = time.time()
        self.base_F, self.base_pad = np.zeros(3), 0.0
        self.build(st)
        if plate and plate.get("center"):
            cx, cy, rim_z = plate["center"]
        else:
            cx, cy = P["plate_xy"]
            rim_z = 0.0726 + (float(P["table_m"]) - 0.747) + (int(P["n"]) - 3) * 0.0068     # SIM: 3 plates, table 0.747, pelvis 0.71
            print("NO plate from perception: using params/push10.json plate_xy and the simulation's rim")
        self.cx, self.cy = float(cx), float(cy)
        self.z_edge = float(rim_z - self.spec.H + self.spec.z_top(self.spec.R))
        self.base_z = float(self.z_edge - self.spec.z_top(self.spec.R))
        self.touch_pt = self.plan_touch()
        print("nearforce_%s: plate x %.3f y %+.3f, rim %.3f; %d plates asked, the camera reads %s; stack %.0f mm from the reference spot; correction %s mrad" % (
            self.mode, cx, cy, rim_z, P["n"], (plate.get("located") or {}).get("n_plates") if plate else None, 1000 * np.hypot(cx - config.STACK_XY_SIM[0], cy - config.STACK_XY_SIM[1]), P["q_correct_mrad"]))
        q_touch, e, d = self.ik(self.touch_pt, self.q_plan, P["w_rot"])
        print("  touch (%.3f %+.3f %.3f): IK %.1f mm, palm %.1f deg off the plan; q %s" % (tuple(self.touch_pt) + (e, d, np.round(q_touch, 3))))
        print("  middle pad there: %s (index %s) pelvis frame" % (np.round(self.mid_pad(q_touch), 3), np.round(self.idx(q_touch), 3)))
        for name, x in (("index", self.idx(q_touch)), ("middle", self.mid_pad(q_touch))):
            print("  %s planned %.0f mm in from the plate's edge" % (name, 1000 * (self.spec.R - float(np.linalg.norm(x[:2] - np.array([self.cx, self.cy]))))))
        _, _, q_high = self.solve_plan()
        z_low, r_low = self.sweep_low(self.qc, q_high)
        print("  move from here to the start: %.2f rad at most; lowest pad %.0f mm above the rim, %.0f mm from the stack centre%s" % (
            float(np.max(np.abs(q_high + self.corr - self.qc))), 1000 * (z_low - self.z_edge), 1000 * r_low,
            "  << TOO LOW: refused" if (r_low < self.spec.R + 0.04 and z_low < self.z_edge + P["pre_clear_mm"] / 1000) else ""))
