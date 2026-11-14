"""Stage 1, push10: hooked-middle side push. The robot-left hand, middle finger curled, finds the top plate's rim with
the index pad, hooks the outer edge with the middle finger and pushes the plate toward the robot's right, so that
its far side overhangs the stack.
"""
import csv
import math
import os
import time

import numpy as np

from .. import config
from ..ik import ArmChain
from ..plate import TASK
from ..stage_base import Abort

PAD_POINT = np.array([0.0, 0.0, 0.001])      # the pad centre in <side>_<finger>_touch_link, as the simulation (platepick/ik.py)
SIDE = "left"
PALM = "left_base_link"
IDX = "left_index_touch_link"
THB = "left_thumb_touch_link"
PADS = ["thumb", "index", "middle", "ring", "pinky"]


def _xyz(name, v):
    return [("%s_%s" % (name, a), v[i]) for i, a in enumerate("xyz")]


class Push10(object):
    name = "push10"

    def __init__(self, ctx, mode="full"):
        self.ctx = ctx
        self.mode = mode
        self.P = ctx.params("push10")
        self.spec = TASK
        key = "%.3f" % float(self.P["table_m"])
        if key not in self.P["pose_by_table"]:
            raise Abort("push10: no simulated pose for table %s (have %s)" % (key, sorted(self.P["pose_by_table"])))
        self.pose = self.P["pose_by_table"][key]
        self.events = []
        self.csv = None
        self.checking = True
        self.in_contact = False
        self.q_mark = None
        self.vmax = self.P["vmax_rad_s"]
        self.yE_now = None                 # the camera's edge at a retry
        self.z_top_touch = None            # the top plate's rim top by the first touch (FK)
        self.cam_bias = 0.0                # the tap-found edge minus the camera's edge (applied to the camera from then on)
        self.y_guess = None

    # ================================================================== kinematics
    def build(self, st):
        """The arm chain to the index pad (the hand as commanded), the thumb pad's chain, the palm offset, and the
        planned palm orientation (FK of the simulation's q_plan)."""
        rob = self.ctx.rob
        self.q11 = np.array(st["des"], float)
        self.hands = np.array(st["hands_cmd"], float)
        self.hands[:6] = self.pose["hand_cmd"]
        self.ch = ArmChain(rob, SIDE, IDX, self.q11, self.hands)
        self.ch_th = ArmChain(rob, SIDE, THB, self.q11, self.hands)
        T = rob.fk(self.q11, self.hands)
        self.R_ip = T[PALM][:3, :3].T @ T[IDX][:3, :3]          # palm <- index pad link, fixed while the hand is
        # the simulation's angles are the real joints; with arm offsets on, the motors read (real - offset)
        self.d5 = np.asarray(rob.d11[config.LEFT], float)
        self.q_plan = np.array(self.pose["q_plan_rad"], float) - self.d5
        qp = self.q11.copy()
        qp[config.LEFT] = self.q_plan
        qp[config.WAIST] = -rob.d11[config.WAIST]                # the simulation's waist 0
        self.R_plan = rob.fk(qp, self.hands)[PALM][:3, :3]
        self.qc = self.q11[config.LEFT].copy()                    # what is commanded
        self.q = self.qc.copy()                                   # the IK's last solution (the next seed)
        self.hook_link, self.hook_pt = self.hook_point(rob, qp)
        self.ch_hook = ArmChain(rob, SIDE, self.hook_link, self.q11, self.hands)

    def hook_point(self, rob, qp):
        """The curled middle's lowest mesh point at the planned push pose, in its own link (the hook: what meets the
        plate's outer edge). (link, point)."""
        T = rob.fk(qp, self.hands)
        z = np.load(os.path.join(config.MODEL_DIR, "hand_points.npz"))
        best = None
        for ln in ("left_middle_proximal_link", "left_middle_distal_link"):
            if ln not in z.files or ln not in T:
                continue
            loc = z[ln].astype(float)
            w = loc @ T[ln][:3, :3].T + T[ln][:3, 3]
            k = int(np.argmin(w[:, 2]))
            if best is None or w[k, 2] < best[2]:
                best = (ln, loc[k], w[k, 2])
        if best is None:
            return IDX, PAD_POINT
        return best[0], best[1]

    def hook(self, q5):
        """The hook point (FK), pelvis frame."""
        T, _ = self.ch_hook.fk(q5)
        return T[:3, :3] @ self.hook_pt + T[:3, 3]

    def idx(self, q5):
        T, _ = self.ch.fk(q5)
        return T[:3, :3] @ PAD_POINT + T[:3, 3]

    def inner_y(self, q5):
        """The innermost (-y) of the index and thumb pads, FK (the simulation's edge rule)."""
        T, _ = self.ch_th.fk(q5)
        return min(self.idx(q5)[1], float((T[:3, :3] @ PAD_POINT + T[:3, 3])[1]))

    def ik(self, P, q0, w):
        """Index pad point to P, palm toward the plan with weight w: the simulation's ik_to (damped least squares,
        damping 1e-4, steps <= 0.1 rad, keep the best iterate). Returns (q5, pos_mm, rot_deg)."""
        q = np.array(q0, float)
        best = None
        Rp = self.R_plan
        for _ in range(int(self.P["ik_iters"])):
            T, fr = self.ch.fk(q)
            p = T[:3, :3] @ PAD_POINT + T[:3, 3]
            e = P - p
            R = T[:3, :3] @ self.R_ip.T
            er = 0.5 * (np.cross(R[:, 0], Rp[:, 0]) + np.cross(R[:, 1], Rp[:, 1]) + np.cross(R[:, 2], Rp[:, 2]))
            c = float(e @ e + (w * np.linalg.norm(er)) ** 2)
            if best is None or c < best[0]:
                best = (c, q.copy(), e, er)
            if np.linalg.norm(e) < 2e-4 and np.linalg.norm(er) < 2e-3:
                break
            Jp = np.array([np.cross(z, p - o) for o, z in fr]).T
            Jr = np.array([z for o, z in fr]).T
            J = np.vstack([Jp, w * Jr])
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), np.r_[e, w * er])
            m = float(np.max(np.abs(dq)))
            if m < 1e-6:                                           # converged (the 5-DoF arm cannot zero both)
                break
            dq *= min(1.0, 0.1 / m)
            q = np.clip(q + dq, self.ch.lo, self.ch.hi)
        _, q, e, er = best
        return q, 1000 * float(np.linalg.norm(e)), math.degrees(float(np.linalg.norm(er)))

    # ================================================================== sensing
    def sense(self, st):
        ctx, P = self.ctx, self.P
        qm = np.array(st["q"], float)[config.LEFT]
        F_raw = ctx.force(st, SIDE, IDX, PAD_POINT)
        t = st.get("touch")
        pads = [0.0] * 5
        prox = 0.0
        if t:
            pads = [float(t[0][i][0]) * P["pad_N_per_count"] for i in range(5)]
            prox = float(t[0][1][3])
        cam = None
        if P["use_camera"]:
            p = ctx.plate(st, max_age=0.5)
            if p and p.get("ok") and p.get("center"):
                cam = np.array(p["center"], float)
                self.cam_t = time.time()
                self.cam_last = (self.cam_t, float(cam[1]))
        return dict(qm=qm, z=float(self.idx(qm)[2]), F=F_raw - self.base_F, F_raw=F_raw, pad=pads[1] - self.base_pad,
                    pads=pads, prox=prox, cam=cam)

    def check(self, st):
        """Every 10 ms while waiting for the next tick: the left arm held back from its plan (the force is checked once
        per tick, in tick())."""
        if not self.checking:
            return ""
        lag = float(np.max(np.abs(np.array(st["des"])[config.LEFT] - np.array(st["q"])[config.LEFT])))
        now = time.time()
        if lag > self.P["q_lag_max_rad"]:
            self._lag_t = getattr(self, "_lag_t", None) or now
            if now - self._lag_t > self.P["q_lag_s"]:
                return "the left arm is %.2f rad from its command for %.1f s (held back)" % (lag, now - self._lag_t)
        else:
            self._lag_t = None
        return ""

    # ================================================================== motion
    def tick(self, q_target, phase, P=None, ik=(0.0, 0.0), extra=None):
        """One 30 Hz command, as the simulation: the joint target moves at most vmax/30 toward q_target; wait for the
        tick's end (checking); returns the sensors after it."""
        ctx, Pp = self.ctx, self.P
        dt = 1.0 / Pp["rate_hz"]
        step = np.clip(q_target - self.qc, -self.vmax * dt, self.vmax * dt)
        self.qc = np.clip(self.qc + step, self.ch.lo, self.ch.hi)
        self.q11[config.LEFT] = self.qc
        ctx.c.set(q=self.q11, hands=self.hands, tag="push10 " + phase)
        self.t_next += dt
        now = time.time()
        if self.t_next < now - 0.1:                                 # late (a slow IK): do not catch up
            self.t_next = now
        why = ctx.c.hold(max(0.0, self.t_next - now), self.check)
        if why:
            raise Abort(why)
        st = ctx.st()
        s = self.sense(st)
        self.row(st, s, phase, P, ik, extra)
        # the force on the index: median of the last 5 ticks (a single dynamic spike is not a force)
        self._fh = (getattr(self, "_fh", []) + [float(np.linalg.norm(s["F"]))])[-5:]
        Fm = float(np.median(self._fh))
        if self.checking and Fm > Pp["f_abort_N"]:
            raise Abort("force on the index %.1f N > %.0f N" % (Fm, Pp["f_abort_N"]))
        return s

    def line(self, P_to, speed, phase, w=None, stop=None):
        """Straight line of the index pad target from self.P to P_to at `speed` mm/s (the simulation's cart_move).
        stop(s, P) -> reason ends it early. Returns (reason or None, s)."""
        w = self.P["w_rot"] if w is None else w
        P0 = self.Pt.copy()
        L = 1000 * float(np.linalg.norm(P_to - P0))
        K = max(1, int(L / speed * self.P["rate_hz"]))
        s = None
        for k in range(1, K + 1):
            Pk = P0 + (P_to - P0) * k / K
            self.q, e_mm, e_deg = self.ik(Pk, self.q, w)
            self.Pt = Pk
            s = self.tick(self.q, phase, Pk, (e_mm, e_deg))
            if stop is not None:
                r = stop(s, Pk)
                if r:
                    return r, s
        return None, s

    def hold_ticks(self, n, phase):
        s = None
        for _ in range(int(n)):
            s = self.tick(self.q, phase, self.Pt)
        return s

    def catch_up(self, phase, tol=0.002, max_s=3.0):
        """Hold the target until the speed-capped command has reached the IK's solution."""
        for _ in range(int(max_s * self.P["rate_hz"])):
            if float(np.max(np.abs(self.q - self.qc))) < tol:
                break
            self.tick(self.q, phase, self.Pt)

    def ev(self, what, **kw):
        kw["at_s"] = round(time.time() - self.t0, 2)
        self.events.append(dict(ev=what, **kw))
        self.ctx.log("push10 " + what, **kw)

    # ================================================================== the log (30 Hz, the reference's columns)
    def open_csv(self, extra_cols=()):
        cols = (["t", "phase"] + ["q_cmd%d" % i for i in range(5)] + ["q_des%d" % i for i in range(5)] + ["q_meas%d" % i for i in range(5)]
                + ["hand%d" % i for i in range(6)] + ["idx_fk_" + a for a in "xyz"] + ["tgt_" + a for a in "xyz"] + ["ik_mm", "ik_deg"]
                + ["cam_" + a for a in "xyz"] + ["idx_pad_N", "idx_prox"] + ["pad_%s_N" % f for f in PADS]
                + ["lag_F" + a for a in "xyz"] + ["lag_raw_F" + a for a in "xyz"] + list(extra_cols))
        self.csv_path = os.path.join(self.ctx.out_dir, "push10_cmd.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols, extrasaction="ignore")
        self.csv.writeheader()
        self.csv_n = 0

    def row(self, st, s, phase, P, ik, extra):
        if self.csv is None:
            return
        r = dict(t=round(time.time() - self.t0, 3), phase=phase)
        des = np.array(st["des"])[config.LEFT]
        for i in range(5):
            r["q_cmd%d" % i] = round(float(self.qc[i]), 5)
            r["q_des%d" % i] = round(float(des[i]), 5)
            r["q_meas%d" % i] = round(float(s["qm"][i]), 5)
        for i in range(6):
            r["hand%d" % i] = round(float(st["hands"][i]), 4)
        r.update((k, round(float(v), 4)) for k, v in _xyz("idx_fk", self.idx(s["qm"])))
        if P is not None:
            r.update((k, round(float(v), 4)) for k, v in _xyz("tgt", P))
        r.update(ik_mm=round(ik[0], 1), ik_deg=round(ik[1], 1))
        if s["cam"] is not None:
            r.update((k, round(float(v), 4)) for k, v in _xyz("cam", s["cam"] - np.array([0, 0, self.spec.H])))   # plate base, as the reference
        r.update(idx_pad_N=round(s["pad"], 2), idx_prox=round(s["prox"], 0))
        r.update(("pad_%s_N" % f, round(v, 2)) for f, v in zip(PADS, s["pads"]))
        r.update(("lag_F" + a, round(float(v), 2)) for a, v in zip("xyz", s["F"]))
        r.update(("lag_raw_F" + a, round(float(v), 2)) for a, v in zip("xyz", s["F_raw"]))
        if extra:
            r.update(extra)
        self.csv.writerow(r)
        self.csv_n += 1
        if self.csv_n % 30 == 0:
            self.csv_f.flush()

    # ================================================================== setup
    def setup(self, need_plate=True):
        ctx, P = self.ctx, self.P
        self.t0 = time.time()
        st = ctx.st()
        self.base_F = np.zeros(3)
        self.base_pad = 0.0
        self.build(st)
        self.cam_t = time.time()
        ctx.c.set(vmax=P["vmax_rad_s"])
        if need_plate:
            if P["use_camera"]:
                t_req = time.time()
                ctx.c.request("locate")
                ctx.log("perception", note="asked for a fresh locate")
                pl = ctx.wait_plate(timeout=P["locate_timeout_s"], need_located_after=t_req - 0.5)
                ctx.c.request("head_lock", on=True)
                loc_z = float(pl["center"][2])
                # the first locate can be a few mm off in height; let the tracker settle and plan from its median
                cs = []
                t_s = time.time()
                while time.time() - t_s < P.get("rim_settle_s", 3.0) + 1.0:
                    ctx.c.hold(0.1)
                    q = ctx.plate(max_age=0.3)
                    if time.time() - t_s >= P.get("rim_settle_s", 3.0) and q and q.get("ok") and q.get("center"):
                        cs.append(q["center"])
                if cs:
                    cx, cy, rim_z = [float(v) for v in np.median(np.array(cs, float), axis=0)]
                else:
                    cx, cy, rim_z = pl["center"]
                    ctx.log("plate", note="no tracked plate after the settle time: the locate's height is used")
                ctx.log("plate", center=[cx, cy, rim_z], located_center=pl["center"], tracked_vs_located_z_mm=round(1000 * (rim_z - loc_z), 1),
                        frames=len(cs), top_height=pl.get("top_height"), n_plates=pl["located"].get("n_plates"),
                        diameter=pl["located"].get("diameter"))
                if abs(rim_z - loc_z) > 0.003:
                    ctx.log("plate", note="LOOK: the tracker's rim is %.1f mm %s the locate's: the locate may have misread the stack "
                            "(n_plates %s)" % (abs(1000 * (rim_z - loc_z)), "below" if rim_z < loc_z else "above", pl["located"].get("n_plates")))
            else:
                cx, cy = P["plate_xy"]
                rim_z = P["rim_z"] if P["rim_z"] is not None else self.pose["rim_z_sim"]
                ctx.log("plate", note="NO CAMERA: given position, no pushed / slip check", center=[cx, cy, rim_z])
            off = np.hypot(cx - config.STACK_XY_SIM[0], cy - config.STACK_XY_SIM[1])
            if off > P["max_offset_m"]:
                raise Abort("the stack is %.0f mm from the reference spot: move it to x %.2f y %.2f (pelvis frame)" % (
                    1000 * off, config.STACK_XY_SIM[0], config.STACK_XY_SIM[1]))
            self.cx, self.cy = float(cx), float(cy)
            base = rim_z - self.spec.H                                    # the camera's plate base (its rim top - 22 mm)
            self.z_edge = float(base + self.spec.z_top(self.spec.R))      # rim top at the edge (= the camera's rim top)
            self.yE = self.cy + self.spec.R                               # the camera's robot-left edge
            ctx.log("push10 plan", table=P["table_m"], stack_offset_mm=1000 * off, z_edge=self.z_edge, y_edge=self.yE,
                    pelvis_from_legs=ctx.pelvis(st), hand=self.pose["hand_cmd"])

    def plan_points(self):
        """Where the sequence goes (camera's plate): name -> (point, palm weight)."""
        P = self.P
        x, yE, z = self.cx, self.yE, self.z_edge
        di = self.pose["di_mm"] / 1000
        P_pre = np.array([x, yE + P["out0"] / 1000, z + P["up"] / 1000])
        P_set = np.array([x, yE - di, z - 0.004])
        P_press = P_set + np.array([0, -P["lead_y"], -P["lead_z"]]) / 1000
        if P.get("edge_mode", "slide") == "tap":
            edge = ("outermost tap", np.array([x, yE + P["out_max_mm"] / 1000, z - (P["below"] + P["stall_lead"]) / 1000]), P["w_rot"])
        else:
            edge = ("edge slide end", np.array([x, yE + 0.035, z - P["edge_lead"] / 1000]), P["w_rot_edge"])
        return [("high", P_pre + np.array([0, 0.03, 0.08]), P["w_rot"]), ("pre", P_pre, P["w_rot"]),
                ("search bottom", np.array([x, yE + P["out0"] / 1000, z - (P["below"] + P["stall_lead"]) / 1000]), P["w_rot"]),
                edge, ("set", P_set, P["w_rot"]), ("press", P_press, P["w_rot"]),
                ("push end", P_press + np.array([0, -P["push_max"] / 1000, 0]), P["w_rot"])]

    def reach_report(self):
        """IK of every planned point from q_plan (logged); returns the worst position miss at set / press, mm (the 5-DoF
        arm keeps the palm angle at a cost in position: the simulation's own pre pose missed by 21 mm, the push by 20)."""
        q = self.q_plan.copy()
        worst = 0.0
        out = {}
        for name, pt, w in self.plan_points():
            q, e_mm, e_deg = self.ik(pt, q, w)
            out[name] = (round(e_mm, 1), round(e_deg, 1))
            if name in ("set", "press"):
                worst = max(worst, e_mm)
        self.ctx.log("push10 reach", note="IK miss per planned point (mm, palm deg); the simulation's misses: <= 20 mm on the push", **{
            k.replace(" ", "_"): v for k, v in out.items()})
        return worst, out

    def sweep_low(self, q_from, q_to, n=25):
        """Lowest index / thumb pad height (pelvis z) over a joint-space move, and where (radius from the stack)."""
        low = (1e9, 0.0)
        for u in np.linspace(0.0, 1.0, n):
            u = u * u * (3 - 2 * u)
            q5 = q_from + u * (q_to - q_from)
            for ch in (self.ch, self.ch_th):
                T, _ = ch.fk(q5)
                p = T[:3, :3] @ PAD_POINT + T[:3, 3]
                if p[2] < low[0]:
                    low = (float(p[2]), float(np.hypot(p[0] - self.cx, p[1] - self.cy)))
        return low

    def go_pre(self):
        """Shape the hand, joint move to above / outside the edge (marked: the server's rewind returns from here), line to
        P_pre. The force baseline is taken on that line (moving, as the search will)."""
        ctx, P = self.ctx, self.P
        pts = dict((n, p) for n, p, _ in self.plan_points())
        self.q_high, e_mm, e_deg = self.ik(pts["high"], self.q_plan, P["w_rot"])
        if e_mm > 30.0:
            raise Abort("the pre pose is out of reach (IK %.0f mm off; the simulation's 21)" % e_mm)
        z_low, r_low = self.sweep_low(self.qc, self.q_high)
        if r_low < self.spec.R + 0.04 and z_low < self.z_edge + P["pre_clear_mm"] / 1000:
            raise Abort("the move to the pre pose would pass %.0f mm above the rim, %.0f mm from the stack centre" % (
                1000 * (z_low - self.z_edge), 1000 * r_low))
        ctx.log("push10 pre", note="hand shape, then a joint move to above the edge", hand=self.pose["hand_cmd"],
                lowest_mm_above_rim=1000 * (z_low - self.z_edge))
        ctx.c.set(hands=self.hands, tag="push10 shape")
        why = ctx.c.hold(1.0)
        if why:
            raise Abort(why)
        q11 = self.q11.copy()
        q11[config.LEFT] = self.q_high
        why = ctx.c.move_to(q11, self.hands, v=P["pre_v_rad_s"], tag="push10 pre")
        if why:
            raise Abort("move to the pre pose: %s" % why)
        ctx.c.set(vmax=P["vmax_rad_s"])
        ctx.c.hold(0.5)                                             # the arm catches up with the move's end
        self.q11 = q11
        self.qc = self.q_high.copy()
        self.q = self.q_high.copy()
        self.Pt = pts["high"].copy()
        ctx.c.mark()
        self.q_mark = q11.copy()
        self.t_next = time.time()
        # baseline of the pad (still) and of the joint-lag force (moving along the line to P_pre, its last 60 %)
        pads = []
        for _ in range(10):
            st = ctx.st()
            t = st.get("touch")
            if t:
                pads.append(float(t[0][1][0]) * P["pad_N_per_count"])
            ctx.c.hold(1.0 / P["rate_hz"])
        self.base_pad = float(np.median(pads)) if pads else 0.0
        self.t_next = time.time()
        acc = []
        # free air: the line from the high pose swings the shoulder (pitch and yaw together, up to the speed limit; the
        # simulation's lag there 0.06 rad): the stage's own lag / force stops start with the search (the server's stay)
        self.checking = False
        self.vmax = P["vmax_free_rad_s"]
        P0 = self.Pt.copy()
        L = 1000 * float(np.linalg.norm(pts["pre"] - P0))
        K = max(1, int(L / P["speed_pre"] * P["rate_hz"]))
        for k in range(1, K + 1):
            Pk = P0 + (pts["pre"] - P0) * k / K
            self.q, e_mm, e_deg = self.ik(Pk, self.q, P["w_rot"])
            self.Pt = Pk
            s = self.tick(self.q, "pre", Pk, (e_mm, e_deg))
            if k > 0.4 * K:
                acc.append(s["F_raw"])
        self.base_F = np.median(np.array(acc), axis=0) if acc else np.zeros(3)
        self.catch_up("pre")
        self.vmax = P["vmax_rad_s"]
        self.checking = True
        self.ctx.log("push10 baseline", lag_force=self.base_F, pad_N=self.base_pad)

    def back_to_mark(self):
        """Off the plate: back to the marked pose and trim the server's path there (its rewind home then skips the
        plate). A joint move when it stays clear of the plate (the hand is up after the release), else up and over."""
        ctx, P = self.ctx, self.P
        if self.q_mark is None:
            return
        q_to = self.q_mark[config.LEFT]
        z_low, r_low = self.sweep_low(self.qc, q_to)
        if r_low < self.spec.R + 0.04 and z_low < self.z_edge + P["pre_clear_mm"] / 1000:
            high = self.plan_points()[0][1]
            if self.Pt[2] < high[2]:
                self.line(np.array([self.Pt[0], self.Pt[1], high[2]]), P["speed_release"], "back")
            self.line(high, P["speed_release"], "back")
        why = ctx.c.move_to(self.q_mark, self.hands, v=0.2, tag="push10 back")
        if not why:
            ctx.c.hold(0.2)
            ctx.c.trim()
            ctx.c.hold(0.2)

    def release(self):
        """8: up 25, out 25, up 60, open the hand, hold 1 s (the simulation's release); then back to the mark."""
        P = self.P
        self.checking = False
        self.in_contact = False
        # up first, until the MEASURED index is clear of the rim (up 25 from the press target left the real
        # index near the rim and the move out dragged the plate back), only then out
        self.lift_clear("release", raise_on_fail=False)
        self.vmax = P["vmax_free_rad_s"]
        self.line(self.Pt + np.array([0, 0.025, 0]), P["speed_release"], "release")
        self.line(self.Pt + np.array([0, 0, 0.060]), P["speed_release_up"], "release")
        self.hands[:6] = 0.0
        self.hold_ticks(P["release_hold_s"] * P["rate_hz"], "release")
        self.back_to_mark()

    def rim_ref(self):
        """The top plate's rim top for clearances: the first touch on it (FK) when there is one, else the camera's."""
        return self.z_top_touch if getattr(self, "z_top_touch", None) is not None else self.z_edge

    def lift_clear(self, phase, z=None, need_mm=None, raise_on_fail=True, speed=None):
        """Straight up to z (default lift_clear_mm above the rim), then wait until the measured index pad is at least need_mm
        above the rim (at most lift_wait_s). The arm lags 5-15 mm when leaving the plate, so nothing moves sideways until
        the measured clearance is reached.
        """
        P = self.P
        rim = self.rim_ref()
        z = rim + P["lift_clear_mm"] / 1000 if z is None else z
        need = rim + (P["lift_need_mm"] if need_mm is None else need_mm) / 1000
        if self.Pt[2] < z:
            self.line(np.array([self.Pt[0], self.Pt[1], z]), speed or P["speed_retry_lift"], phase)
        s = None
        for _ in range(int(P["lift_wait_s"] * P["rate_hz"]) + 1):
            s = self.tick(self.q, phase, self.Pt)
            if s["z"] >= need:
                return True
        self.ev("lift not reached", phase=phase, above_rim_mm=round(1000 * (s["z"] - rim), 1), need_mm=round(1000 * (need - rim), 1),
                target_above_rim_mm=round(1000 * (z - rim), 1), ik_above_rim_mm=round(1000 * (float(self.idx(self.q)[2]) - rim), 1))
        if raise_on_fail:
            raise Abort("the index is only %.0f mm above the rim after %.1f s (needs %.0f): not moving sideways" % (
                1000 * (s["z"] - rim), P["lift_wait_s"], 1000 * (need - rim)))
        return False

    def hover_to(self, y, z):
        """Sideways to y at height z, the index first raised (measured: `tap_need_mm` above the rim), so it does not
        slide on the plate."""
        P = self.P
        self.lift_clear("search", z=z, need_mm=P["tap_need_mm"], speed=P.get("speed_tap_lift"))
        if abs(self.Pt[1] - y) > 1e-4:
            self.line(np.array([self.Pt[0], y, z]), P["speed_search"], "search")

    # ================================================================== the sequence
    def touched_fn(self):
        """The simulation's touch rule: index pad > pad_touch_N for 3 ticks, or a stall (FK z moved < 1 mm over 10
        ticks while the command is > stall_lead below it) with the joint-lag force up > f_stall_N or the proximity."""
        P = self.P
        hist, pads = [], []

        def touched(s, P_):
            hist.append(s["z"])
            pads.append(s["pad"])
            if len(pads) >= 3 and min(pads[-3:]) > P["pad_touch_N"]:
                return "pad"
            if (len(hist) > 10 and hist[-10] - s["z"] < 0.001 and s["z"] - P_[2] > P["stall_lead"] / 1000
                    and (s["F"][2] > P["f_stall_N"] or s["prox"] >= P["prox_touch_counts"])):
                return "stall"
            return None
        return touched

    def down(self, x, y, z_to, speed, phase):
        """Line down to z_to with the touch rule; then hold there `tap_hold_s` with the rule still on: the arm lags 5-10 mm,
        so a stall (the R1's usual touch: its pad often reads 0) may show only after the command has stopped. Default 0 (off):
        taps only 12 mm under the rim missed every rim on the simulated arm without pad data, hold or not."""
        stop = self.touched_fn()
        how, s = self.line(np.array([x, y, z_to]), speed, phase, stop=stop)
        for _ in range(int(self.P.get("tap_hold_s", 0.0) * self.P["rate_hz"])):
            if how:
                break
            s = self.tick(self.q, phase, self.Pt)
            how = stop(s, self.Pt)
        return how, s

    def descend(self, y, z_from, z_to, speed, phase):
        """Down at y from z_from to z_to until touched; a stall is confirmed by a second touch-down. Returns
        (how, z_touch by FK) or (None, None)."""
        P = self.P
        x = self.Pt[0]
        how, s = self.down(x, y, z_to, speed, phase)
        if not how:
            return None, None
        z1 = s["z"]
        if how == "stall" and P["confirm_touch"]:
            self.line(np.array([x, y, z1 + P["confirm_lift_mm"] / 1000]), speed, phase)
            how2, s2 = self.down(x, y, z_to, speed, phase)
            if not how2 or abs(s2["z"] - z1) * 1000 > P["confirm_mm"]:
                self.ev("touch not confirmed", first_mm=round(1000 * (z1 - self.z_edge), 1),
                        second_mm=None if not how2 else round(1000 * (s2["z"] - self.z_edge), 1))
                return None, None
            how, z1 = how + " (confirmed)", s2["z"]
        return how, z1

    def search_touch_edge(self, y_guess=None, z_ref=None):
        """2 search / touch, 3 edge, from the edge guess y_guess (default the camera's at the start) with the top plate's
        rim top at z_ref (default the camera's). Returns (z_touch, y_edge) or raises Abort."""
        y_guess = self.yE if y_guess is None else y_guess
        z_ref = self.z_edge if z_ref is None else z_ref
        if self.P.get("edge_mode", "slide") == "tap":
            return self.search_tap(y_guess, z_ref)
        return self.search_slide(y_guess)

    def tap(self, y, z_ref, n):
        """One tap at y: from `tap_hover_mm` above the rim (reached, measured, before moving sideways) straight down.
        Returns ("top" | "lower" | "miss", z_touch or None); the index is left where it stopped."""
        P = self.P
        z_hover = z_ref + P["tap_hover_mm"] / 1000
        self.hover_to(y, z_hover)
        # (R1: 3.7 s a tap): fast down to tap_fast_mm above the rim, then speed_touch only as deep as a TOP-plate
        # touch can be (tap_below_mm: anything lower is "plate below", a miss anyway)
        z_fast = z_ref + P.get("tap_fast_mm", P["tap_hover_mm"]) / 1000
        if z_fast < z_hover - 1e-4:
            self.line(np.array([self.Pt[0], y, z_fast]), P["speed_search"], "touch")
        how, z = self.descend(y, z_fast, z_ref - (P.get("tap_below_mm", P["below"]) + P["stall_lead"]) / 1000, P["speed_touch"], "touch")
        y_cam = None if self.yE_now is None else round(1000 * (y - self.yE_now), 1)
        if not how:
            self.ev("tap", n=n, y_vs_guess_mm=round(1000 * (y - self.y_guess), 1), result="miss")
            return "miss", None
        dz = round(1000 * (z - z_ref), 1)
        if z < z_ref - P["lower_rim_mm"] / 1000:
            self.ev("tap", n=n, y_vs_guess_mm=round(1000 * (y - self.y_guess), 1), result="plate below", z_vs_rim_mm=dz, by=how)
            return "lower", z
        self.ev("tap", n=n, y_vs_guess_mm=round(1000 * (y - self.y_guess), 1), result="top", z_vs_rim_mm=dz, by=how,
                y_vs_camera_now_mm=y_cam)
        return "top", z

    def search_tap(self, y_guess, z_ref):
        """2 + 3 by taps (edge_mode "tap"). The first tap at `tap_start_mm` outside the guess (the camera's edge). It lands on
        the top plate -> taps `step` further OUT until one misses; it misses -> `step` further IN until one lands. The edge
        lies between the last miss and the first touch (+ tap_edge_offset_mm): usually 2 taps. `tap_refine`: one more tap half-way. The index never slides on the
        plate: every sideways move is at `tap_hover_mm` above its rim, after the measured index got there."""
        P = self.P
        self.y_guess = y_guess
        self.phase_now = "search"
        step = P["step"] / 1000
        y = y_guess + P["tap_start_mm"] / 1000
        n = 0
        res, z = self.tap(y, z_ref, n)
        y_hit = y_miss = None
        z_touch = z
        if res == "top":
            y_hit = y
            while y_hit - y_guess < P["out_max_mm"] / 1000 - 1e-6:          # landed: out until a miss
                n += 1
                y = y_hit + step
                res, z = self.tap(y, z_ref, n)
                if res != "top":
                    y_miss = y
                    break
                y_hit, z_touch = y, z
        else:
            y_miss = y
            while True:                                                     # missed: in until it lands
                n += 1
                # never walk further in than tap_in_max_mm inside the camera's edge (the search went 52 mm in
                # and the curled middle rode over the rim)
                if n >= int(P["steps"]) or (y_miss - step) < y_guess - P.get("tap_in_max_mm", 20.0) / 1000 - 1e-6:
                    self.lift_clear("search", raise_on_fail=False)
                    raise Abort("no rim found under the index (%d taps, from %.0f mm outside the edge guess in to %.0f mm inside)" % (
                        n, P["tap_start_mm"], 1000 * (y_guess - y)))
                y = y_miss - step
                res, z = self.tap(y, z_ref, n)
                if res == "top":
                    y_hit, z_touch = y, z
                    break
                y_miss = y
        if y_miss is not None and P["tap_refine"]:
            n += 1
            ym = 0.5 * (y_miss + y_hit)
            r2, z2 = self.tap(ym, z_ref, n)
            if r2 == "top":
                y_hit, z_touch = ym, z2
            else:
                y_miss = ym
        self.in_contact = False
        self.hover_to(self.Pt[1], z_ref + P["tap_hover_mm"] / 1000)
        if y_miss is None:
            y_edge = y_hit + P["tap_edge_offset_mm"] / 1000
            self.ev("edge not bracketed: the outermost tap touched", y_vs_guess_mm=round(1000 * (y_edge - y_guess), 1))
        else:
            y_edge = 0.5 * (y_miss + y_hit) + P["tap_edge_offset_mm"] / 1000
        self.ev("edge", by="taps", y_vs_guess_mm=round(1000 * (y_edge - y_guess), 1), bracket_mm=None if y_miss is None else round(1000 * (y_miss - y_hit), 1),
                z_vs_rim_mm=round(1000 * (z_touch - z_ref), 1), taps=n + 1)
        if abs(y_edge - y_guess) * 1000 > P["edge_tol"]:
            self.ev("edge implausible, guess used", found_vs_guess_mm=round(1000 * (y_edge - y_guess), 1))
            y_edge = y_guess
        return z_touch, y_edge

    def search_slide(self, y_guess):
        """2 search / touch, 3 edge by the simulation's slide (edge_mode "slide"). Returns (z_touch, y_edge)."""
        P = self.P
        y_try = y_guess + P["out0"] / 1000
        z_touch = None
        for k in range(int(P["steps"])):
            self.line(np.array([self.Pt[0], y_try, self.z_edge + 0.010]), P["speed_search"], "search")
            how, z_touch = self.descend(y_try, self.z_edge + 0.010, self.z_edge - (P["below"] + P["stall_lead"]) / 1000,
                                        P["speed_touch"], "touch")
            if not how:
                y_try -= P["step"] / 1000
                continue
            self.in_contact = True
            self.Pt = np.array([self.Pt[0], self.Pt[1], z_touch])
            self.line(self.Pt - np.array([0, 0, P["edge_lead"] / 1000]), P["speed_edge_press"], "edge")
            s = self.hold_ticks(0.5 * P["rate_hz"], "edge")
            z_set = s["z"]
            if z_touch - z_set > P["press_sink"] / 1000:
                self.ev("touch off the rim", sink_mm=round(1000 * (z_touch - z_set), 1), y_try_vs_camera_mm=round(1000 * (y_try - y_guess), 1))
                self.line(np.array([self.Pt[0], self.Pt[1], self.z_edge + 0.010]), P["speed_edge_up"], "search")
                y_try -= P["step"] / 1000
                z_touch = None
                continue
            self.ev("touch", by=how, z_vs_camera_mm=round(1000 * (z_touch - self.z_edge), 1), y_vs_camera_edge_mm=round(1000 * (y_try - y_guess), 1),
                    pad_N=round(s["pad"], 2), lag_Fz=round(float(s["F"][2]), 1))
            break
        if z_touch is None:
            raise Abort("no rim found under the index (%d tries from %.0f mm outside the camera's edge)" % (P["steps"], P["out0"]))

        def off_edge(s, P_):
            return "drop" if s["z"] < z_set - P["edge_drop"] / 1000 else None
        how, s = self.line(self.Pt + np.array([0, 0.070, 0]), P["speed_edge_slide"], "edge", w=P["w_rot_edge"], stop=off_edge)
        if not how:
            raise Abort("edge not found: slid 70 mm out without the index dropping")
        y_edge = self.inner_y(s["qm"]) - P["edge_bias"] / 1000
        self.line(self.Pt + np.array([0, 0, 0.010]), P["speed_edge_up"], "edge")
        self.in_contact = False
        self.ev("edge", y_vs_camera_mm=round(1000 * (y_edge - y_guess), 1), index_y_mm=round(1000 * self.idx(s["qm"])[1], 1))
        if abs(y_edge - y_guess) * 1000 > P["edge_tol"]:
            self.ev("edge implausible, guess used", found_vs_camera_mm=round(1000 * (y_edge - y_guess), 1))
            y_edge = y_guess
        return z_touch, y_edge

    def cam_y(self, n=8):
        """Median camera y over a few ticks' estimates (while holding). None yet: keep holding up to camera_timeout_s;
        still none: the last estimate if it is younger than `camera_stale_s`, else Abort."""
        ys = []
        t_end = time.time() + self.P["camera_timeout_s"]
        k = 0
        while k < n or (not ys and time.time() < t_end):
            s = self.tick(self.q, self.phase_now, self.Pt)
            k += 1
            if s["cam"] is not None:
                ys.append(s["cam"][1])
                self.cam_last = (time.time(), float(s["cam"][1]))
        if ys:
            return float(np.median(ys))
        last = getattr(self, "cam_last", None)
        if last is not None and time.time() - last[0] < self.P["camera_stale_s"]:
            self.ev("camera: no fresh estimate, last one used", age_s=round(time.time() - last[0], 1))
            return last[1]
        raise Abort("no plate estimate from the camera")

    def run(self):
        if self.mode == "air":
            return self.run_air()
        ctx, P = self.ctx, self.P
        self.setup()
        worst, _ = self.reach_report()
        if worst > 10.0:
            raise Abort("the set / press points are out of reach (IK %.0f mm off): is the stack where the simulation had it?" % worst)
        self.open_csv()
        res = dict(mode=self.mode, table=P["table_m"], log=self.csv_path)
        try:
            self.go_pre()
            if self.mode == "shape":
                return self.run_shape(res)
            self.phase_now = "pre"
            cam_start = self.cam_y() if P["use_camera"] else None
            res.update(self.push_loop(cam_start))
        except Abort as ex:
            res.update(result="aborted: %s" % ex)
            self.finish(res)
            if self.q_mark is not None:
                ctx.log("push10", note="lifting off and back to the marked pose before handing the arms back")
                try:
                    self.release()
                except Exception as ex2:                              # noqa: BLE001
                    ctx.log("push10", note="could not lift off cleanly: %r" % ex2)
            raise
        self.release()
        self.finish(res)
        if self.mode == "full" and res.get("result") != "pushed":
            raise Abort("push10: %s after %d tries (camera saw the plate move %.0f mm); lifted off" % (
                res.get("result"), res.get("retries", 0), res.get("camera_slide_mm", 0.0)))
        return res

    def push_loop(self, cam_start):
        ctx, P = self.ctx, self.P
        tries, result, lead_z = 0, "no push", P["lead_z"]
        slide = 0.0
        y_guess, z_ref = self.yE, self.z_edge
        while tries <= P["retries"]:
            self.phase_now = "search"
            if tries > 0 and cam_start is not None:
                # where the plate is NOW (the retry searched from the start's edge, half-pushed plate -> it
                # found the plate below's edge): the camera's edge, the hand already up and clear
                self.yE_now = self.cam_y() + self.spec.R + self.cam_bias
                y_guess = self.yE_now
                self.ev("retry from the camera's edge now", edge_vs_start_mm=round(1000 * (self.yE_now - self.yE), 1), n=tries,
                        camera_bias_mm=round(1000 * self.cam_bias, 1))
            z_touch, y_edge = self.search_touch_edge(y_guess, z_ref)
            if cam_start is not None and P.get("edge_mode", "slide") == "tap" and P["cam_bias_from_taps"]:
                # the taps measured the edge; the camera's centre can be off by several mm (-1..+5 vs the slide's
                # edge): its offset here corrects the camera's edge for the hook test and the next retry
                b = y_edge - (self.cam_y() + self.spec.R)
                self.cam_bias = float(np.clip(b, -P["edge_tol"] / 1000, P["edge_tol"] / 1000))
                self.ev("camera edge bias from the taps", bias_mm=round(1000 * self.cam_bias, 1))
            if self.z_top_touch is None:
                self.z_top_touch = z_touch                          # the top plate's rim by FK: the next searches' reference
                z_ref = z_touch
            if self.mode == "edge":
                return dict(result="edge found", z_touch_vs_camera_mm=round(1000 * (z_touch - self.z_edge), 1),
                            edge_vs_camera_mm=round(1000 * (y_edge - self.yE), 1), events=self.events)
            # 4 set
            x = self.Pt[0]
            self.line(np.array([x, self.Pt[1], z_touch + 0.008]), P["speed_set"], "set")
            self.line(np.array([x, y_edge - self.pose["di_mm"] / 1000, z_touch + 0.008]), P["speed_set"], "set")
            how, s = self.line(self.Pt - np.array([0, 0, 0.030]), P["speed_set_down"], "set", stop=self.touched_fn())
            if not how:
                tries += 1
                result = "set: no touch"
                self.ev("set: no touch within 30 mm", n=tries)
                self.lift_clear("retry")
                continue
            self.in_contact = True
            self.ev("set", by=how, z_vs_touch_mm=round(1000 * (s["z"] - z_touch), 1), y_vs_edge_mm=round(1000 * (self.Pt[1] - y_edge), 1))
            # 5 press (position: the command lead_y in, lead_z down)
            P_press = self.Pt + np.array([0, -P["lead_y"], -lead_z]) / 1000
            self.line(P_press, P["speed_press"], "press")
            # 6 engage
            self.phase_now = "press"
            y_plate0 = self.cam_y(3) if cam_start is not None else None
            dist = 0.0
            cam_n = 0
            dt = 1.0 / P["rate_hz"]
            engaged = None
            while dist < P["gap_max"] / 1000:
                dist += P["speed_engage"] / 1000 * dt
                Pk = P_press + np.array([0, -dist, 0])
                self.q, e_mm, e_deg = self.ik(Pk, self.q, P["w_rot"])
                self.Pt = Pk
                s = self.tick(self.q, "engage", Pk, (e_mm, e_deg))
                if s["cam"] is not None and y_plate0 is not None:
                    cam_n = cam_n + 1 if (y_plate0 - s["cam"][1]) * 1000 > P["cam_engage_mm"] else 0
                self.cam_fresh()
                if s["F"][1] > P["f_engage"]:
                    engaged = "force"
                    break
                if cam_n >= P["cam_frames"]:
                    engaged = "camera"
                    break
            self.ev("engaged" if engaged else "no edge contact", by=engaged, travel_mm=round(1000 * dist, 1), lag_Fy=round(float(s["F"][1]), 1),
                    hook_vs_camera_edge_mm=None if s["cam"] is None else round(1000 * (self.hook(s["qm"])[1] - s["cam"][1] - self.spec.R), 1))
            # 7 push
            y_hand0 = self.idx(s["qm"])[1]
            if y_plate0 is not None:
                y_plate0 = s["cam"][1] if s["cam"] is not None else y_plate0
            done = None
            cy_last = y_plate0
            cz0 = cz_last = None if s["cam"] is None else float(s["cam"][2])    # the camera's rim height: a pushed plate rises
            hooked = None                                           # (hand y, plate y) when the hook reached the edge
            over_n = 0
            while dist < P["push_max"] / 1000:
                dist += P["v"] / 1000 * dt
                Pk = P_press + np.array([0, -dist, 0])
                self.q, e_mm, e_deg = self.ik(Pk, self.q, P["w_rot"])
                self.Pt = Pk
                s = self.tick(self.q, "push", Pk, (e_mm, e_deg))
                if cam_start is None or s["cam"] is None:
                    self.cam_fresh()
                    continue
                cy = cy_last = s["cam"][1]
                cz_last = float(s["cam"][2])
                cz0 = cz_last if cz0 is None else cz0
                y_hand = self.idx(s["qm"])[1]
                if (cam_start - cy) * 1000 >= P["stop"]:
                    done = "pushed"
                    break
                if P.get("slip_mode", "hand") == "hook":
                    gap = self.hook(s["qm"])[1] - (cy + self.spec.R + self.cam_bias)    # + = the hook still outside the plate's edge
                    if hooked is None and gap * 1000 <= P["hook_tol_mm"]:
                        hooked = (y_hand, cy)
                        self.ev("hook at the edge", travel_mm=round(1000 * dist, 1), camera_slide_mm=round(1000 * (cam_start - cy), 1))
                    over_n = over_n + 1 if -gap * 1000 > P["slip_hook_mm"] else 0
                    if over_n >= P["slip_frames"]:
                        done = "slip"
                        self.ev("slip: the hook is over the plate", hook_inside_edge_mm=round(-1000 * gap, 1))
                        break
                    if hooked is not None and ((hooked[0] - y_hand) - (hooked[1] - cy)) * 1000 > P["slip"]:
                        done = "slip"
                        self.ev("slip: the hand ran ahead of the plate since the hook reached it",
                                hand_mm=round(1000 * (hooked[0] - y_hand), 1), plate_mm=round(1000 * (hooked[1] - cy), 1))
                        break
                elif ((y_hand0 - y_hand) - (y_plate0 - cy)) * 1000 > P["slip"]:
                    done = "slip"
                    break
            slide = (cam_start - cy_last) * 1000 if cam_start is not None else 0.0
            self.ev(done or "push travel limit", camera_slide_mm=round(slide, 1), command_mm=round(1000 * dist, 1),
                    lead_z_mm=lead_z, n=tries, camera_rise_mm=None if cz0 is None else round(1000 * (cz_last - cz0), 1))
            if done == "pushed":
                result = "pushed"
                break
            tries += 1
            result = done or "push travel limit"
            self.in_contact = False
            if tries <= P["retries"]:
                lead_z += P["lead_z_step"]
                # straight up, clear of the rim by the MEASURED index, before anything moves sideways (up 20 mm
                # from the press target left the real index at the rim and the move out dragged the plate back)
                self.lift_clear("retry")
        return dict(result=result, retries=tries, camera_slide_mm=round(float(slide), 1), events=self.events)

    def cam_fresh(self):
        if self.P["use_camera"] and time.time() - self.cam_t > self.P["camera_timeout_s"]:
            raise Abort("no plate estimate for %.1f s during the push" % (time.time() - self.cam_t))

    def finish(self, res):
        if self.csv is not None:
            self.csv_f.flush()
        res.setdefault("events", self.events)
        self.ctx.results[self.name] = res
        self.ctx.memory["push10_end"] = dict(q=list(self.q11), result=res.get("result"))
        return res

    # ================================================================== the other modes
    def run_shape(self, res):
        """push10_shape: at the pre pose, hold `shape_hold_s` to look at the hand; log where the curled middle hangs."""
        ctx, P = self.ctx, self.P
        st = ctx.st()
        T = ctx.fk(st=st)
        from ..model import hand_points
        hp = hand_points(T, SIDE)
        mid = np.vstack([v for k, v in hp.items() if "middle" in k])
        ip = T[IDX][:3, :3] @ PAD_POINT + T[IDX][:3, 3]
        low = mid[np.argmin(mid[:, 2])]
        out = dict(index_pad=ip, middle_lowest=low, middle_below_index_mm=round(1000 * (ip[2] - low[2]), 1),
                   middle_outward_of_index_mm=round(1000 * (low[1] - ip[1]), 1), hands_measured=np.array(st["hands"])[:6])
        ctx.log("push10 shape", note="LOOK at the hand now (%.0f s): the curled middle should hang below the index pad by about the "
                "rim's thickness and point at the rim" % P["shape_hold_s"], **out)
        self.checking = False
        self.hold_ticks(P["shape_hold_s"] * P["rate_hz"], "shape")
        self.hands[:6] = 0.0
        self.hold_ticks(P["release_hold_s"] * P["rate_hz"], "release")
        self.back_to_mark()
        res.update(result="shape held", **out)
        return self.finish(res)

    def run_air(self):
        """push10_air: the simulation's reference COMMAND path (index pad targets, hand commands; tgt_* of
        refs/push10/<air_ref>) played `air_lift_mm` higher, in free air: robot back from the table or no plate. Logs the
        index (FK) vs the target per tick next to the simulated arm's, and a per-phase summary."""
        ctx, P = self.ctx, self.P
        self.t0 = time.time()
        st = ctx.st()
        self.base_F, self.base_pad = np.zeros(3), 0.0
        self.build(st)
        self.cam_t = time.time()
        P["use_camera"] = False
        path = os.path.join(config.REFS_DIR, "push10", self.pose["air_ref"])
        rows = list(csv.DictReader(open(path)))
        f = lambda r, k: np.array([float(r[k + "_" + a]) for a in "xyz"])      # noqa: E731
        # the reference starts with a 3 s joint move to its first point (the target does not move): start after it
        k0 = 0
        while k0 + 1 < len(rows) and np.allclose(f(rows[k0 + 1], "tgt"), f(rows[0], "tgt")):
            k0 += 1
        rows = rows[k0:]
        lift = np.array([0.0, 0.0, P["air_lift_mm"] / 1000])
        tg = np.array([f(r, "tgt") for r in rows]) + lift
        self.cx, self.cy = config.STACK_XY_SIM
        self.yE = self.cy + self.spec.R
        # the reference's rim top at the edge (its camera's plate base + 22 mm), lifted as the path
        self.z_edge = float(f(rows[0], "cam")[2]) + self.spec.z_top(self.spec.R) + lift[2]
        ctx.log("push10 air", note="FREE AIR: the reference command path %.0f mm higher; nothing should touch" % P["air_lift_mm"],
                ref=self.pose["air_ref"], rows=len(rows), lowest_target_z=float(tg[:, 2].min()))
        self.open_csv(["sim_" + c for c in ("idx_fk_x", "idx_fk_y", "idx_fk_z", "tgt_x", "tgt_y", "tgt_z", "lag_Fx", "lag_Fy", "lag_Fz")])
        res = dict(mode="air", table=P["table_m"], ref=self.pose["air_ref"], lift_mm=P["air_lift_mm"], log=self.csv_path)
        self.q_high, e_mm, _ = self.ik(tg[0], self.q_plan, P["w_rot"])
        if e_mm > 30.0:
            raise Abort("the reference path's start is out of reach (IK %.0f mm off)" % e_mm)
        ctx.c.set(hands=self.hands, tag="push10 shape")
        ctx.c.hold(1.0)
        q11 = self.q11.copy()
        q11[config.LEFT] = self.q_high
        why = ctx.c.move_to(q11, self.hands, v=P["pre_v_rad_s"], tag="push10 pre")
        if why:
            raise Abort("move to the path's start: %s" % why)
        ctx.c.set(vmax=P["vmax_rad_s"])
        ctx.c.hold(0.5)
        self.q11, self.qc, self.q, self.Pt = q11, self.q_high.copy(), self.q_high.copy(), tg[0].copy()
        ctx.c.mark()
        self.q_mark = q11.copy()
        self.t_next = time.time()
        err = {}
        try:
            for k, r in enumerate(rows):
                ph = r["phase"]
                slide = ph == "edge" and k > 0 and tg[k, 1] - tg[k - 1, 1] > 1e-4 and abs(tg[k, 2] - tg[k - 1, 2]) < 1e-5
                w = P["w_rot_edge"] if slide else P["w_rot"]
                self.vmax = P["vmax_free_rad_s"] if ph in ("pre", "release") else P["vmax_rad_s"]
                self.q, e_mm, e_deg = self.ik(tg[k], self.q, w)
                self.Pt = tg[k]
                self.hands[:6] = [float(r["hand%d" % i]) for i in range(6)]
                sim = dict(("sim_" + c, r[c]) for c in ("idx_fk_x", "idx_fk_y", "idx_fk_z", "tgt_x", "tgt_y", "tgt_z", "lag_Fx", "lag_Fy", "lag_Fz"))
                s = self.tick(self.q, ph, tg[k], (e_mm, e_deg), sim)
                e_r = 1000 * (self.idx(s["qm"]) - tg[k])
                e_s = 1000 * (f(r, "idx_fk") - f(r, "tgt"))
                err.setdefault(ph, []).append(np.r_[e_r, e_s])
        except Abort as ex:
            res["result"] = "aborted: %s" % ex
            self.finish(res)
            raise
        finally:
            self.hands[:6] = 0.0
            self.checking = False
            try:
                self.back_to_mark()
            except Exception as ex2:                                      # noqa: BLE001
                ctx.log("push10", note="could not go back to the mark: %r" % ex2)
        summ = {}
        for ph, v in err.items():
            v = np.array(v)
            summ[ph] = dict(robot_mean_mm=np.round(v[:, :3].mean(0), 1).tolist(), robot_max_mm=np.round(np.abs(v[:, :3]).max(0), 1).tolist(),
                            sim_mean_mm=np.round(v[:, 3:].mean(0), 1).tolist(), sim_max_mm=np.round(np.abs(v[:, 3:]).max(0), 1).tolist())
            ctx.log("push10 air phase", phase=ph, **summ[ph])
        res.update(result="played", per_phase_index_minus_target=summ)
        return self.finish(res)

    # ================================================================== plan only (run_test.py --dry)
    def dry(self, st, plate):
        """Print the plan from the current pose and a plate estimate (or the params' position); nothing moves."""
        P = self.P
        self.t0 = time.time()
        self.base_F, self.base_pad = np.zeros(3), 0.0
        self.build(st)
        if plate and plate.get("center"):
            cx, cy, rim_z = plate["center"]
        else:
            cx, cy = P["plate_xy"]
            rim_z = P["rim_z"] if P["rim_z"] is not None else self.pose["rim_z_sim"]
            print("NO plate from perception: using params/push10.json plate_xy / rim_z")
        self.cx, self.cy = float(cx), float(cy)
        self.z_edge = float(rim_z - self.spec.H + self.spec.z_top(self.spec.R))
        self.yE = self.cy + self.spec.R
        print("push10, table %.3f: plate x %.3f y %+.3f, rim top at the edge %.3f, edge y %.3f; stack %.0f mm from the reference spot" % (
            P["table_m"], cx, cy, self.z_edge, self.yE, 1000 * np.hypot(cx - config.STACK_XY_SIM[0], cy - config.STACK_XY_SIM[1])))
        q = self.q_plan.copy()
        for name, pt, w in self.plan_points():
            q, e_mm, e_deg = self.ik(pt, q, w)
            print("  %-15s (%.3f %+.3f %.3f): IK %5.1f mm, palm %4.1f deg off the plan" % (name, pt[0], pt[1], pt[2], e_mm, e_deg))
            if name == "high":
                z_low, r_low = self.sweep_low(self.qc, q)
                print("  move from here to 'high': lowest pad %.0f mm above the rim, %.0f mm from the stack centre%s" % (
                    1000 * (z_low - self.z_edge), 1000 * r_low,
                    "  << TOO LOW" if (r_low < self.spec.R + 0.04 and z_low < self.z_edge + P["pre_clear_mm"] / 1000) else ""))
