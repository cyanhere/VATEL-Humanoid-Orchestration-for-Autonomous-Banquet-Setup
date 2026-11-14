"""Stage 1, rim push with the curled little finger: the robot-left hand pushes the top plate toward the robot's right
until the camera sees it slide done_mm.
"""
import math
import time
from collections import deque

import numpy as np

from .. import config
from ..ik import ArmChain, solve_point_dir
from ..model import PINKY_END, PINKY_LOW
from ..plate import TASK
from ..stage_base import Abort, smooth

# the pushed rim edge sinks as the plate slides over the one below (simulation, 3 plates): mm slid -> mm dropped
EDGE_S = [0.0, 8.0, 45.0, 80.0]
EDGE_DROP = [0.0, 0.0, 4.5, 4.6]
X, Y, Z = np.eye(3)


class Push(object):
    name = "push"

    def __init__(self, ctx):
        self.ctx = ctx
        self.P = ctx.params("push")
        self.spec = TASK

    # ------------------------------------------------------------------ helpers
    def drop(self, s):
        return float(np.interp(s, EDGE_S, EDGE_DROP))

    def nrm(self, g):
        return -math.cos(g) * self.out - math.sin(g) * self.up

    def go_to(self, pt, n, T):
        """Move the pinky end to pt with the finger pointing along n, straight line over T s. Returns False when the
        IK cannot follow (the arm then holds)."""
        ctx = self.ctx
        p0, n0 = self.pt.copy(), self.n.copy()
        t0 = time.time()
        while True:
            u = smooth((time.time() - t0) / max(T, 1e-3)) if T > 0 else 1.0
            ok = self.step(p0 + u * (pt - p0), n0 + u * (n - n0))
            if not ok:
                return False
            if u >= 1.0:
                return True
            why = ctx.c.hold(0.02, self.check)
            if why:
                raise Abort(why)

    # ------------------------------------------------------------------ where the finger is
    def set_feedback(self, axes, gain, cap_mm=None, note=None):
        """Which directions the fingertip correction may act in (unit vectors; leave out the expected contact
        direction), how fast (1/s), and at most how far from where it is now (mm; None = only fb_max_mm)."""
        self.fb_axes = [np.asarray(a, float) / np.linalg.norm(a) for a in axes]
        self.fb_gain = float(gain)
        self.fb_cap = None if cap_mm is None else (self.off.copy(), cap_mm / 1000.0)
        if note:
            self.ctx.log("feedback", note=note, off_mm=1000 * self.off)

    def feedback(self, st):
        """self.off += gain x (where the pinky end should be - where it measures), on the allowed axes only."""
        e, low = self.ctx.pinky(self.ctx.fk(st=st))
        self.e, self.low = e, low
        now = time.time()
        dt = min(now - self.fb_t, 0.1) if self.fb_t else 0.0
        self.fb_t = now
        if dt <= 0.0 or not self.fb_axes:
            return
        err = self.pt - e                                       # the last target vs where the finger is
        d = np.zeros(3)
        for a in self.fb_axes:
            d += (err @ a) * a
        step = self.fb_gain * dt * d
        lim = self.P["fb_rate_mm_s"] / 1000.0 * dt
        n = float(np.linalg.norm(step))
        if n > lim:
            step *= lim / n
        off = self.off + step
        if self.fb_cap is not None:
            c, r = self.fb_cap
            dv = off - c
            n = float(np.linalg.norm(dv))
            if n > r:
                off = c + dv * (r / n)
        n = float(np.linalg.norm(off))
        m = self.P["fb_max_mm"] / 1000.0
        if n > m:
            off *= m / n
        self.off = off

    def step(self, pt, n, st=None):
        """One IK step toward where the pinky end should BE; the command is that plus the correction self.off (updated
        here from where the finger measures). False if out of reach."""
        self.feedback(st if st is not None else self.ctx.st())
        n = n / np.linalg.norm(n)
        q5, info = solve_point_dir(self.ch, self.q5, np.asarray(pt, float) + self.off, PINKY_END["left"], [0.0, 0.0, 1.0], n,
                                   w=0.05, iters=25)
        if info["pos_mm"] > 2.0 or info.get("ang_deg", 0) > 6.0:
            self.ctx.log("ik", note="target out of reach, holding", pos_mm=info["pos_mm"], ang_deg=info.get("ang_deg"))
            return False
        self.q5 = q5
        self.q11[config.LEFT] = q5
        self.pt, self.n = np.asarray(pt, float).copy(), n.copy()
        self.ctx.c.set(q=self.q11)
        return True

    def check(self, st):
        """Absolute limit, every cycle, on the median of the last 5 estimates (100 ms): the estimate assumes slow
        motion, a single dynamic spike is not a force."""
        F = float(np.linalg.norm(self.f(st)))
        self._fh = (getattr(self, "_fh", []) + [F])[-5:]
        Fm = float(np.median(self._fh))
        if Fm > self.P["f_abort_N"]:
            return "force on the finger %.1f N > %.0f N" % (Fm, self.P["f_abort_N"])
        return ""

    def f(self, st):
        return self.ctx.force(st, "left") - self.base_f

    def touch(self, st):
        t = self.ctx.touch(st, "left", "Pinky")
        return None if t is None else t - self.base_touch

    def baseline(self, seconds=0.3):
        """Force and touch with nothing touching (averaged)."""
        ctx = self.ctx
        fs, ts = [], []
        t0 = time.time()
        while time.time() - t0 < seconds:
            st = ctx.st()
            fs.append(ctx.force(st, "left"))
            t = ctx.touch(st, "left", "Pinky")
            if t is not None:
                ts.append(t)
            ctx.c.hold(0.02)
        self.base_f = np.mean(fs, axis=0)
        self.base_touch = float(np.median(ts)) if ts else 0.0

    def progress(self, st):
        """mm the camera has seen the plate move along the push direction since self.pl0; None if no fresh estimate."""
        p = self.ctx.plate(st)
        if not p or not p.get("ok") or "center" not in p:
            return None
        d = np.array(p["center"][:2]) - self.pl0
        return float(d @ (-self.out[:2])) * 1000.0

    @staticmethod
    def stalled(hist, now, window_s, move_mm, lead_mm):
        """hist: (t, measured position along the motion, lead = target - measured along it). True when over the last
        window_s the finger moved less than move_mm while the command stayed at least lead_mm ahead of it."""
        if not hist or now - hist[0][0] < window_s:
            return False
        old = [h for h in hist if h[0] >= now - window_s]
        if len(old) < 5:
            return False
        moved = abs(old[-1][1] - old[0][1]) * 1000.0
        return moved < move_mm and min(h[2] for h in old) * 1000.0 >= lead_mm

    # ------------------------------------------------------------------ the touch-down
    def descend(self, C, nrm0, z_from, band, depth, label):
        """Lower the pinky end straight down over C from z_from until it meets something inside band (lowest point of
        the finger, pelvis z, (lo, hi)). Returns dict(low, end, F, touch, by) or None."""
        ctx, P = self.ctx, self.P
        lead_max, lead_min = P["lead_max_mm"] / 1000, P["lead_min_mm"] / 1000
        zc, t_prev, t_desc = z_from, time.time(), time.time()
        fs, ts, based, n_on, free = [], [], self.base_known, 0, True
        hist = deque()
        self.set_feedback([X, Y, Z], P["fb_gain_free"])
        z_moved, t_moved = zc, time.time()
        while zc > z_from - depth:
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = ctx.st()
            e = getattr(self, "e", None)
            # the target goes down, but never more than lead_max below where the finger IS: a small, capped press
            zc -= P["descend_mm_s"] / 1000 * dt
            if e is not None:
                zc = max(zc, e[2] - lead_max)
            if zc < z_moved - 0.001:
                z_moved, t_moved = zc, now
            elif now - t_moved > P["blocked_s"]:
                raise Abort("%s: the finger stopped at %.0f mm, outside the heights where the rim can be (%.0f-%.0f mm): "
                            "camera and standing height both off, or something else under it" % (
                                label, 1000 * self.low, 1000 * band[0], 1000 * band[1]))
            if not self.step(np.array([C[0], C[1], zc]), nrm0, st):
                raise Abort("touch-down left the arm's reach")
            why = ctx.c.hold(0.02, self.check)
            if why:
                raise Abort(why)
            low_now = self.low - (self.e[2] - zc)               # where the finger's lowest point is being sent
            if free and low_now < band[1]:
                # entering the heights where the rim can be: from now on never correct downward (contact direction)
                free = False
                self.set_feedback([X, Y], P["fb_gain"], note="%s: near the rim, height correction held" % label)
            if not based:
                # the force baseline is taken while ALREADY moving down (joint friction flips sign between resting and
                # moving: ~2 N at the fingertip), over the first moving_baseline_s
                fs.append(ctx.force(st, "left"))
                t_ = ctx.touch(st, "left", "Pinky")
                if t_ is not None:
                    ts.append(t_)
                if now - t_desc > P["moving_baseline_s"]:
                    self.base_f = np.mean(fs[len(fs) // 3:], axis=0)
                    self.base_touch = float(np.median(ts)) if ts else 0.0
                    based = self.base_known = True
                    ctx.log("baseline", force=self.base_f, touch=self.base_touch, after_mm=1000 * (z_from - zc))
                continue
            lead = self.e[2] - zc                               # command below the finger: pressing down by this
            hist.append((now, self.low, lead))
            while hist and hist[0][0] < now - 2.0:
                hist.popleft()
            if not (band[0] <= self.low <= band[1]):
                n_on = 0
                continue                                        # no contact possible up here: a stall is friction
            F, tc = self.f(st), self.touch(st)
            by = []
            if tc is not None and tc > P["touch_on"]:
                by.append("pad")
            if F[2] > P["f_on_N"] and lead > lead_min:
                by.append("force")
            n_on = n_on + 1 if by else 0
            stall = self.stalled(hist, now, P["stall_s"], P["stall_mm"], P["lead_min_mm"])
            if stall or n_on >= P["contact_n"]:
                Tm = ctx.fk(st=st)
                e_, low = ctx.pinky(Tm)
                return dict(low=low, end=e_, F=F, touch=tc, by="+".join(by + (["stall"] if stall else [])) or "stall", z_cmd=zc)
        return None

    # ------------------------------------------------------------------ the edge, by proximity (push.edge_search)
    def prox(self, st):
        """The little finger's proximity, raw counts: median of the last 5 readings; None without touch data."""
        t = st.get("touch")
        if not t:
            return None
        self._px = (getattr(self, "_px", []) + [float(t[0][4][3])])[-5:]
        return float(np.median(self._px))

    def gap_mm(self, counts):
        """How far above a porcelain surface the finger's lowest point is, from its proximity (prox_curve, measured on
        the R1 the little finger descending onto a plate on the stack)."""
        g, n = self.P["prox_curve"]["gap_mm"], self.P["prox_curve"]["counts"]
        return float(np.interp(-counts, [-v for v in n], g))

    def lowest(self, st):
        """The curled little finger's lowest point (pelvis frame) where the arm measures it."""
        M = self.ctx.fk(st=st)["left_pinky_distal_link"]
        pts = PINKY_LOW["left"] @ M[:3, :3].T + M[:3, 3]
        return pts[np.argmin(pts[:, 2])]

    def edge_search(self, top, rim_z, rim_legs, nrm0):
        """Find the top plate's edge and rim height with the little finger's proximity sensor, in the arm's own frame.

        Go down over the rim, edge_start_in_mm inside the camera's edge, until the reading is edge_set; then move outward
        holding that reading until the surface falls away. If nothing is found, or less than edge_min_follow_mm of rim was
        followed, go up, move edge_retry_in_mm further in and retry.
        Returns dict(edge (x, y), rim_z, r_edge, camera_error_mm).
        """
        ctx, P = self.ctx, self.P
        if self.prox(ctx.st()) is None:
            raise Abort("edge search: no proximity data (the touch topic): is the 13 Oct hand server running?")
        starts = [P["edge_start_in_mm"] + v for v in [0.0] + list(P["edge_retry_in_mm"])]
        whys = []
        for k, start_in in enumerate(starts):
            once = self._edge_steps if P.get("edge_mode", "steps") == "steps" else self._edge_once
            res, why = once(top, rim_z, rim_legs, nrm0, start_in)
            if res is not None:
                return res
            whys.append(why)
            ctx.log("edge retry", why=why, n=k + 1, next_start_in_mm=starts[k + 1] if k + 1 < len(starts) else None)
        raise Abort("edge search failed: %s" % "; ".join(whys))

    def _probe(self, xy, z_from, z_to, nrm0):
        """Down at (x, y) from z_from (pinky end) until the proximity reads edge_set, then still: the surface height under
        the finger's lowest point (its height minus the gap the RAW readings say, averaged while still), or None if
        nothing by z_to."""
        ctx, P = self.ctx, self.P
        z, t_prev = z_from, time.time()
        self._px = []
        while True:
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = ctx.st()
            p = self.prox(st)
            if p is None:
                raise Abort("edge search: the touch data stopped")
            if p >= P["edge_set"]:
                break
            z -= P["edge_descend_mm_s"] / 1000 * dt
            if z < z_to:
                return None, z
            if not self.step(np.array([xy[0], xy[1], z]), nrm0, st):
                raise Abort("edge search left the arm's reach")
            why = ctx.c.hold(0.02, self.check)
            if why:
                raise Abort(why)
        # still: hold the command, let the joints settle, then measure
        t0 = time.time()
        while time.time() - t0 < P["edge_settle_s"]:
            self.step(np.array([xy[0], xy[1], z]), nrm0)
            ctx.c.hold(0.02)
        hs, t0 = [], time.time()
        while time.time() - t0 < 0.2:
            st = ctx.st()
            t = st.get("touch")
            if t:
                hs.append(float(self.lowest(st)[2]) - self.gap_mm(float(t[0][4][3])) / 1000)
            ctx.c.hold(0.02)
        return (float(np.median(hs)) if hs else None), z

    def _edge_steps(self, top, rim_z, rim_legs, nrm0, start_in_mm):
        """Step and probe outward: at every edge_step_mm the surface height (still finger, see _probe), until it drops
        by more than edge_fall_mm below the rim followed so far, or is gone; then the bracket is halved once. Immune to
        the arm's stick-slip (it measures only when still). (result, "") or (None, why to try further in)."""
        ctx, P, spec = self.ctx, self.P, self.spec
        c = np.asarray(top[:2], float)
        out = self.out[:2]
        r = spec.R - start_in_mm / 1000
        S = np.r_[c + r * out, rim_z + P["edge_above_mm"] / 1000]
        ctx.log("edge", note="to above the rim (step and probe)", point=S, start_in_mm=start_in_mm)
        up = np.array([self.pt[0], self.pt[1], max(self.pt[2], S[2])])
        if not self.go_to(up, nrm0, max(0.5, float(np.linalg.norm(up - self.pt)) / 0.03)):
            raise Abort("edge search: cannot lift")
        mid = np.array([S[0], S[1], up[2]])
        if not self.go_to(mid, nrm0, max(1.0, float(np.linalg.norm(mid - up)) / 0.04)):
            raise Abort("edge search: cannot reach above the rim")
        if not self.go_to(S, nrm0, max(0.8, float(np.linalg.norm(S - mid)) / 0.03)):
            raise Abort("edge search: cannot reach above the rim")
        self.set_feedback([X, Y, Z], P["fb_gain_free"])
        z_floor = min(rim_z, rim_legs) - P["edge_depth_mm"] / 1000
        lift = (self.gap_mm(P["edge_set"]) + P["edge_lift_mm"]) / 1000      # hover this far above the last surface
        stations = []                                    # (r of the finger's lowest point from c, surface height or None)
        h, z_cmd = self._probe(S[:2], S[2], z_floor, nrm0)
        lw = self.lowest(ctx.st())
        ctx.log("edge probe", note="first", surface=h, z_end_cmd=z_cmd, finger_low=lw, z_floor=z_floor)
        if h is None:
            return None, "nothing under the finger down to %.0f mm, %.0f mm inside the camera's edge" % (1000 * z_floor, start_in_mm)

        def r_low():
            lw = self.lowest(ctx.st())
            return float(np.linalg.norm(lw[:2] - c))
        stations.append((r_low(), h))
        rim_hi = h
        n_max = int(P["edge_max_out_mm"] / P["edge_step_mm"]) + 1
        r_on = r_off = None
        for _ in range(n_max):
            z_hover = z_cmd + lift
            self.go_to(np.r_[self.pt[:2], z_hover], nrm0, 0.3)                              # up off the surface
            r += P["edge_step_mm"] / 1000
            xy = c + r * out
            self.go_to(np.r_[xy, z_hover], nrm0, max(0.3, P["edge_step_mm"] / 1000 / 0.01))  # across, 10 mm/s
            h, z_cmd2 = self._probe(xy, z_hover, rim_hi - P["edge_miss_mm"] / 1000, nrm0)
            rr = r_low()
            stations.append((rr, h))
            if h is None or h < rim_hi - P["edge_fall_mm"] / 1000:
                r_off, z_cmd = r, z_cmd2
                break
            rim_hi = max(rim_hi, h)
            r_on, z_cmd = r, z_cmd2
        else:
            raise Abort("edge search: no edge within %.0f mm outward" % P["edge_max_out_mm"])
        followed = 1000 * (stations[-2][0] - stations[0][0]) if len(stations) > 1 else 0.0
        if r_on is None or followed < P["edge_min_follow_mm"]:
            ctx.log("edge stations", r_mm=[1000 * v[0] for v in stations], surface_mm=[None if v[1] is None else 1000 * v[1] for v in stations])
            self.go_to(np.r_[self.pt[:2], rim_hi + lift + 0.004], nrm0, 0.4)
            return None, "only %.1f mm of rim followed before the edge (started at or past it)" % followed
        # halve the bracket once: one more probe between the last station on the rim and the first off it
        z_hover = rim_hi + lift
        self.go_to(np.r_[self.pt[:2], z_hover + 0.002], nrm0, 0.4)
        rm = 0.5 * (r_on + r_off)
        xy = c + rm * out
        self.go_to(np.r_[xy, z_hover + 0.002], nrm0, 0.4)
        h, _ = self._probe(xy, z_hover, rim_hi - P["edge_miss_mm"] / 1000, nrm0)
        rr = r_low()
        stations.append((rr, h))
        if h is None or h < rim_hi - P["edge_fall_mm"] / 1000:
            r_off = rm
        else:
            r_on = rm
            rim_hi = max(rim_hi, h)
        # the edge between the last probe on the rim and the first off it; a probe a little past the edge still finds the
        # rim's corner (sideways) and reads the fall only where the corner is sqrt(gap^2 - (gap - fall)^2) away
        g = self.gap_mm(P["edge_set"])
        reach = np.sqrt(max(g ** 2 - max(g - P["edge_fall_mm"], 0.0) ** 2, 0.0)) / 1000
        on = [st_ for st_ in stations if st_[1] is not None and st_[1] >= rim_hi - P["edge_fall_mm"] / 1000]
        off = [st_ for st_ in stations if st_[1] is None or st_[1] < rim_hi - P["edge_fall_mm"] / 1000]
        r_last_on = max(v[0] for v in on)
        r_first_off = min(v[0] for v in off if v[0] > r_last_on - 1e-9) if any(v[0] > r_last_on - 1e-9 for v in off) else r_last_on
        r_edge = 0.5 * (r_last_on + r_first_off) - 0.5 * reach - P["edge_r_bias_mm"] / 1000
        r_edge = max(r_edge, r_last_on)
        edge = c + r_edge * out
        rim_top = rim_hi
        res = dict(edge=edge, rim_z=rim_top, r_edge=r_edge, camera_error_mm=[1000 * (r_edge - spec.R), 1000 * (rim_z - rim_top)],
                   samples=len(stations), followed_mm=followed)
        ctx.log("edge stations", r_mm=[1000 * v[0] for v in stations], surface_mm=[None if v[1] is None else 1000 * v[1] for v in stations])
        ctx.log("edge found", edge=edge, rim_z=rim_top, r_edge_mm=1000 * r_edge, followed_mm=followed,
                bracket_mm=[1000 * r_last_on, 1000 * r_first_off],
                camera_edge_error_mm=1000 * (spec.R - r_edge), camera_rim_error_mm=1000 * (rim_z - rim_top),
                note="the camera's edge is %.1f mm %s, its rim %.1f mm %s" % (
                    abs(1000 * (spec.R - r_edge)), "outward of the real one" if r_edge < spec.R else "inward of the real one",
                    abs(1000 * (rim_z - rim_top)), "high" if rim_z > rim_top else "low"))
        self.go_to(np.r_[self.pt[:2], rim_hi + lift + 0.008], nrm0, 0.5)              # up, off the edge
        return res, ""

    def _edge_once(self, top, rim_z, rim_legs, nrm0, start_in_mm):
        """One search from start_in_mm inside the camera's edge: (result, "") or (None, why to try further in)."""
        ctx, P, spec = self.ctx, self.P, self.spec
        c = np.asarray(top[:2], float)
        out = self.out[:2]
        r0 = spec.R - start_in_mm / 1000
        S = np.r_[c + r0 * out, rim_z + P["edge_above_mm"] / 1000]
        ctx.log("edge", note="to above the rim", point=S, start_in_mm=start_in_mm)
        up = np.array([self.pt[0], self.pt[1], max(self.pt[2], S[2])])        # up first, then across: never sideways low
        if not self.go_to(up, nrm0, max(0.5, float(np.linalg.norm(up - self.pt)) / 0.03)):
            raise Abort("edge search: cannot lift")
        mid = np.array([S[0], S[1], up[2]])
        if not self.go_to(mid, nrm0, max(1.0, float(np.linalg.norm(mid - up)) / 0.04)):
            raise Abort("edge search: cannot reach above the rim")
        if not self.go_to(S, nrm0, max(0.8, float(np.linalg.norm(S - mid)) / 0.03)):
            raise Abort("edge search: cannot reach above the rim")
        self.set_feedback([X, Y, Z], P["fb_gain_free"])
        for _ in range(int(0.5 / 0.02)):
            self.step(S, nrm0)
            ctx.c.hold(0.02)
        # down until the reading says ~3 mm (a contact by force is a backstop: then it stops there)
        z_floor = min(rim_z, rim_legs) - P["edge_depth_mm"] / 1000
        z, t_prev = S[2], time.time()
        self._px = []
        while True:
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = ctx.st()
            p = self.prox(st)
            if p is None:
                raise Abort("edge search: the touch data stopped")
            if p >= P["edge_set"]:
                break
            z -= P["edge_descend_mm_s"] / 1000 * dt
            if z < z_floor:
                return None, "nothing under the finger down to %.0f mm, %.0f mm inside the camera's edge" % (1000 * z_floor, start_in_mm)
            if not self.step(np.array([S[0], S[1], z]), nrm0, st):
                raise Abort("edge search left the arm's reach")
            why = ctx.c.hold(0.02, self.check)
            if why:
                raise Abort(why)
        low = self.lowest(st)
        ctx.log("edge", note="over the rim", proximity=p, finger_low=low, gap_mm=self.gap_mm(p))
        # outward, holding the reading
        s, trace, surf = 0.0, [], deque()
        gap_set = self.gap_mm(P["edge_set"]) / 1000
        while True:
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = ctx.st()
            p = self.prox(st)
            if p is None:
                raise Abort("edge search: the touch data stopped")
            # geometry from the RAW reading of this instant with the finger's height of this instant (the filtered one
            # lags: with stick-slip the finger jumps, and a lagging reading paired with the new height is mm off)
            raw = self._px[-1]
            low = self.lowest(st)
            r = float(np.linalg.norm(low[:2] - c))
            trace.append((now, r, float(low[2]), raw, low[0], low[1]))
            if raw >= P["edge_lost"]:
                surf.append((now, float(low[2]) - self.gap_mm(raw) / 1000))
            while surf and surf[0][0] < now - 0.5:
                surf.popleft()
            local = float(np.median([v for _, v in surf])) if surf else None
            if p < P["edge_lost"] and local is not None and low[2] < local + gap_set - P["edge_drop_mm"] / 1000:
                break                                                   # the surface under it is gone: past the edge
            vz = float(np.clip(P["edge_kz_mm_s"] * math.log(max(p, 1.0) / P["edge_set"]), -P["edge_vz_mm_s"], P["edge_vz_mm_s"]))
            z += vz / 1000 * dt
            s += P["edge_slide_mm_s"] / 1000 * dt
            if s > P["edge_max_out_mm"] / 1000:
                raise Abort("edge search: no edge within %.0f mm outward (proximity %.0f)" % (P["edge_max_out_mm"], p))
            if not self.step(np.r_[c + (r0 + s) * out, z], nrm0, st):
                raise Abort("edge search left the arm's reach")
            why = ctx.c.hold(0.02, self.check)
            if why:
                raise Abort(why)
        tr = np.array(trace)
        ctx.log("edge trace", r_mm=1000 * tr[:, 1], z_mm=1000 * tr[:, 2], prox=tr[:, 3])
        # the edge: where the reading fell through half the set point for the last time, going out
        half = P["edge_set"] / 2.0
        k = len(tr) - 1
        while k > 0 and tr[k, 3] < half:
            k -= 1
        if k == len(tr) - 1 or k == 0:
            return None, "the reading never fell through %.0f" % half
        a, b = tr[k], tr[k + 1]
        u = (a[3] - half) / max(a[3] - b[3], 1e-9)
        r_cross = a[1] + u * (b[1] - a[1])
        z_cross = a[2] + u * (b[2] - a[2])
        xy = a[4:6] + u * (b[4:6] - a[4:6])
        followed = 1000 * (r_cross - tr[0, 1])
        if followed < P["edge_min_follow_mm"]:
            return None, "only %.1f mm of rim followed before the edge (started at or past it)" % followed
        # the rim's top: the surface the finger followed just before the edge (6 to 1.5 mm before the crossing, where the
        # reading is steep and the corner has not started to fall away), its upper part (the rim rises to its edge)
        near = (tr[:, 1] > r_cross - 0.006) & (tr[:, 1] < r_cross - 0.0015) & (tr[:, 3] >= half)
        if near.sum() < 3:
            near = (tr[:, 1] > r_cross - 0.010) & (tr[:, 3] >= P["edge_lost"])
        hts = tr[near, 2] - np.array([self.gap_mm(v) for v in tr[near, 3]]) / 1000
        if not len(hts):
            return None, "no reading of the rim's top before the edge"
        rim_top = float(np.percentile(hts, 75))
        # past the edge the finger still senses the rim's corner: the reading falls through half where the corner is
        # gap(half) away, i.e. sqrt(gap(half)^2 - height^2) beyond the edge (height above the rim's top there)
        hgt = max(z_cross - rim_top, 0.0)
        g_half = self.gap_mm(half) / 1000
        r_edge = r_cross - math.sqrt(max(g_half ** 2 - hgt ** 2, 0.0)) - P["edge_r_bias_mm"] / 1000
        edge = c + r_edge * (xy - c) / max(np.linalg.norm(xy - c), 1e-9)
        res = dict(edge=edge, rim_z=rim_top, r_edge=r_edge, camera_error_mm=[1000 * (r_edge - spec.R), 1000 * (rim_z - rim_top)],
                   samples=len(tr), followed_mm=followed)
        ctx.log("edge found", edge=edge, rim_z=rim_top, r_edge_mm=1000 * r_edge, followed_mm=followed,
                camera_edge_error_mm=1000 * (spec.R - r_edge), camera_rim_error_mm=1000 * (rim_z - rim_top),
                note="the camera's edge is %.1f mm %s, its rim %.1f mm %s" % (
                    abs(1000 * (spec.R - r_edge)), "outward of the real one" if r_edge < spec.R else "inward of the real one",
                    abs(1000 * (rim_z - rim_top)), "high" if rim_z > rim_top else "low"))
        here = self.pt.copy()
        self.go_to(here + np.array([0.0, 0.0, 0.010]), nrm0, 0.5)               # off the edge, up
        return res, ""

    # ------------------------------------------------------------------ the stage
    def setup(self):
        """The finger, the chain, and where the plate is (camera, standing height): everything before the rim is
        touched. Returns dict(top, rim_z, rim_legs, cx, cy, g0, u_push, lat, h_pelvis)."""
        ctx, P, spec = self.ctx, self.P, self.spec
        st = ctx.st()
        self.q11 = np.array(st["des"])
        hands = np.array(st["hands_cmd"])
        if abs(hands[5] - P["pinky_curl"]) > 0.02:
            ctx.log("pinky", note="curling the pushing little finger", to=P["pinky_curl"])
            hands[5] = P["pinky_curl"]
            ctx.c.set(hands=hands)
            ctx.c.hold(1.0)
        self.ch = ArmChain(ctx.rob, "left", "left_pinky_distal_link", self.q11, hands)
        self.q5 = self.q11[config.LEFT].copy()
        T, _ = self.ch.fk(self.q5)
        self.pt = T[:3, :3] @ PINKY_END["left"] + T[:3, 3]
        self.n = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
        self.base_f, self.base_touch, self.base_known = np.zeros(3), 0.0, False
        self.off = np.zeros(3)                       # command - target of the pinky end, from where it measures
        self.fb_axes, self.fb_gain, self.fb_cap, self.fb_t = [X, Y, Z], P["fb_gain_free"], None, 0.0
        self.out = np.array([math.sin(math.radians(P["phi_deg"])), math.cos(math.radians(P["phi_deg"])), 0.0])
        self.up = np.array([0.0, 0.0, 1.0])
        b = math.radians(P["beta_deg"])
        u_push = -math.cos(b) * self.out - math.sin(b) * self.up
        lat = np.cross(self.up, self.out)            # horizontal, across the push
        g0 = math.radians(P["gamma_deg"])

        # ---- where is the plate
        if P.get("use_camera", True):
            t_req = time.time()
            ctx.c.request("locate")
            ctx.log("perception", note="asked for a fresh locate")
            pl = ctx.wait_plate(timeout=P["locate_timeout_s"], need_located_after=t_req - 0.5)
            ctx.c.request("head_lock", on=True)
            cx, cy, rim_z = pl["center"]
            ctx.log("plate", center=pl["center"], top_height=pl.get("top_height"), n_plates=pl["located"].get("n_plates"),
                    diameter=pl["located"].get("diameter"))
        else:
            cx, cy = P["plate_xy"]
            rim_z = P["rim_z"]
            ctx.log("plate", note="NO CAMERA: given position", center=[cx, cy, rim_z])
        h_pelvis = ctx.pelvis(st)
        rim_legs = P["table_m"] - h_pelvis + spec.H + (P["n_plates"] - 1) * spec.floor_top
        ctx.log("heights", pelvis_from_legs=h_pelvis, rim_z_camera=rim_z, rim_z_from_legs_and_table=rim_legs,
                note="the touch-down decides; these only set where it starts and where a contact is possible")
        top = np.array([cx, cy, rim_z - spec.H])
        # stack position check against the reference (where every path was tuned)
        off_xy = np.array([cx, cy]) - np.array(config.STACK_XY_SIM)
        if np.linalg.norm(off_xy) > P["max_offset_m"]:
            raise Abort("the stack is %.0f mm from the reference spot (x %.3f, y %.3f): move it to x %.2f y %.2f (pelvis frame)" % (
                1000 * np.linalg.norm(off_xy), cx, cy, config.STACK_XY_SIM[0], config.STACK_XY_SIM[1]))
        return dict(top=top, rim_z=rim_z, rim_legs=rim_legs, cx=cx, cy=cy, g0=g0, u_push=u_push, lat=lat, h_pelvis=h_pelvis)

    def run(self):
        ctx, P, spec = self.ctx, self.P, self.spec
        S = self.setup()
        top, rim_z, rim_legs, cx, cy = S["top"], S["rim_z"], S["rim_legs"], S["cx"], S["cy"]
        g0, u_push, lat, h_pelvis = S["g0"], S["u_push"], S["lat"], S["h_pelvis"]
        if P.get("edge_search"):
            # ---- 1. the edge and the rim's height found by the finger's proximity (instead of the touch-down)
            nrm0 = self.nrm(g0)
            res = self.edge_search(top, rim_z, rim_legs, nrm0)
            c_new = res["edge"] - spec.R * self.out[:2]               # the plate whose edge lies where the finger found it
            cx, cy = float(c_new[0]), float(c_new[1])
            top = np.array([cx, cy, rim_z - spec.H])
            dz = res["rim_z"] - rim_z
            rim_true = res["rim_z"]
            ctx.memory["rim_z"] = rim_true
            ctx.memory["edge"] = res
            self.set_feedback([X, Y, Z], P["fb_gain_free"])
        else:
            rim_true, dz = self.touch_down(top, rim_z, rim_legs, g0)
        self.approach_and_push(top, rim_z, rim_true, dz, cx, cy, g0, u_push, lat, h_pelvis)
        return self.ctx.results["push"]

    def touch_down(self, top, rim_z, rim_legs, g0):
        """1. touch-down on the rim top, cal_r inside the edge: the rim's real height. Returns (rim_true, dz)."""
        ctx, P, spec = self.ctx, self.P, self.spec
        rc = spec.R - P["cal_r_mm"] / 1000
        C = top + rc * self.out + np.array([0, 0, spec.z_top(rc)])
        z_lo, z_hi = sorted((C[2], C[2] + rim_legs - rim_z))           # the contact height by camera / by legs + table
        band = (z_lo - P["rim_band_mm"][0] / 1000, z_hi + P["rim_band_mm"][1] / 1000)
        z_start = band[1] + P["search_above_mm"] / 1000
        hover = np.array([C[0], C[1], z_start])
        nrm0 = self.nrm(g0)
        ctx.log("move", to="above the touch-down point", point=hover, contact_band_mm=[1000 * band[0], 1000 * band[1]])
        mid = np.array([hover[0], hover[1], max(self.pt[2], hover[2])])     # over, then down
        if not self.go_to(mid, nrm0, max(1.5, float(np.linalg.norm(mid - self.pt)) / 0.04)):
            raise Abort("cannot reach above the rim")
        if not self.go_to(hover, nrm0, max(0.8, float(np.linalg.norm(hover - mid)) / 0.03)):
            raise Abort("cannot reach above the rim")
        for _ in range(int(0.6 / 0.02)):                                # settle there, still correcting
            self.step(hover, nrm0)
            ctx.c.hold(0.02)
        ctx.log("touch-down", note="descending", speed_mm_s=P["descend_mm_s"], correction_mm=1000 * self.off)
        depth = (z_start - band[0]) + 0.005
        touches, hit = [], None
        z_from = hover[2]
        for k in range(P["touch_tries"]):
            h_ = self.descend(C, nrm0, z_from, band, depth, "touch %d" % (k + 1))
            if h_ is None:
                raise Abort("no touch-down between %.0f and %.0f mm (camera rim %.3f, legs %.3f): is the plate there?" % (
                    1000 * band[0], 1000 * band[1], rim_z, rim_legs))
            touches.append(h_)
            ctx.log("touch", n=k + 1, low_mm=1000 * h_["low"], by=h_["by"], force=h_["F"], touch=h_["touch"])
            if not P["confirm_touch"]:
                hit = h_
                break
            if len(touches) >= 2 and abs(touches[-1]["low"] - touches[-2]["low"]) * 1000 <= P["confirm_mm"]:
                hit = touches[-1] if touches[-1]["low"] < touches[-2]["low"] else touches[-2]
                break
            # lift a little and come down again: a joint stuck by friction does not stop at the same height twice
            z_from = h_["end"][2] + P["confirm_lift_mm"] / 1000
            self.set_feedback([X, Y, Z], P["fb_gain_free"])
            self.go_to(np.array([C[0], C[1], z_from]), nrm0, 0.5)
            ctx.c.hold(0.2)
            depth = (z_from - band[0]) + 0.005
        if hit is None:
            raise Abort("the touch-down did not repeat (lowest point %s mm): friction, or the plate moved" % (
                [round(1000 * t_["low"], 1) for t_ in touches]))
        rr = float(np.hypot(hit["end"][0] - top[0], hit["end"][1] - top[1]))
        dz = hit["low"] - (top[2] + spec.z_top(min(rr, spec.R))) - P["cal_off_mm"] / 1000
        rim_true = rim_z + dz
        ctx.log("touched", rim_z_measured=rim_true, camera_error_mm=-1000 * dz, by=hit["by"], force=hit["F"], touch=hit["touch"],
                radius_mm=1000 * rr, touches_mm=[1000 * t_["low"] for t_ in touches])
        ctx.memory["rim_z"] = rim_true
        self.set_feedback([X, Y, Z], P["fb_gain_free"])
        self.go_to(np.array([C[0], C[1], hit["end"][2] + 0.008]), nrm0, 0.4)
        ctx.c.hold(0.3)
        return rim_true, dz

    def approach_and_push(self, top, rim_z, rim_true, dz, cx, cy, g0, u_push, lat, h_pelvis):
        """2. guarded approach outside the edge at the measured height, 3. the push."""
        ctx, P, spec = self.ctx, self.P, self.spec
        nrm0 = self.nrm(g0)
        h = dz + P["lift_mm"] / 1000
        ce_r, ce_z = spec.edge_c
        E = top + ce_r * self.out + np.array([0, 0, ce_z])
        P0 = E - spec.edge_r * nrm0
        P_pre = P0 - 0.025 * nrm0 + np.array([0, 0, h])
        self.go_to(P_pre + np.array([0, 0, 0.012]), nrm0, 1.0)
        self.go_to(P_pre, nrm0, 0.6)
        for _ in range(int(0.5 / 0.02)):                                # settle on the start point, still correcting
            self.step(P_pre, nrm0)
            ctx.c.hold(0.02)
        self.baseline(0.4)
        self.pl0 = None
        if P.get("use_camera", True):
            p = ctx.wait_plate(timeout=3.0)
            self.pl0 = np.array(p["center"][:2])
        c_xy = np.array([cx, cy])
        s, s_c, s0, contact, start = 0.0, 0.0, 0.0, None, P_pre.copy()
        nrm_i, corr, prog, retries = nrm0, 0.0, 0.0, 0
        cam_n = f_n = 0
        base_acc = []
        hist = deque()
        near = False
        e_contact = None
        sc_moved, t_moved = 0.0, time.time()
        t_prev = time.time()
        last_cam = time.time()
        ctx.log("push", note="guarded approach", speed_mm_s=P["speed_mm_s"], correction_mm=1000 * self.off)
        while True:
            now = time.time()
            dt = min(now - t_prev, 0.1)
            t_prev = now
            st = ctx.st()
            F = self.f(st)
            tc = self.touch(st)
            pg = self.progress(st) if self.pl0 is not None else None
            if pg is not None:
                prog, last_cam = pg, now
            elif self.pl0 is not None and now - last_cam > P["camera_timeout_s"]:
                raise Abort("no plate estimate for %.1f s during the push" % (now - last_cam))
            if contact is None:
                d_app = nrm_i if s == 0 else u_push
                e = getattr(self, "e", self.pt)
                # the target moves on, but never more than lead_max ahead of where the finger IS along the approach
                s_c = min(s_c + P["speed_mm_s"] * dt, 1000 * ((e - start) @ d_app) + P["lead_max_mm"])
                if s_c > sc_moved + 1.0:
                    sc_moved, t_moved = s_c, now
                elif now - t_moved > P["blocked_s"]:
                    raise Abort("approach: the finger is held %.0f mm from the plate's centre (edge at %.0f), before the "
                                "edge: friction or something in the way" % (1000 * float(np.linalg.norm(e[:2] - c_xy)), 1000 * spec.R))
                pt = start + d_app * s_c / 1000
                if not self.step(pt, nrm_i, st):
                    raise Abort("approach left the arm's reach")
                # force baseline taken while ALREADY moving (friction flips sign between resting and moving), over the
                # first part of the approach, which cannot touch the edge yet (it starts 25 mm / 8 mm outside it)
                free_mm = (P["approach_free_mm"] if s == 0 else P["retry_free_mm"])
                if s_c <= free_mm:
                    base_acc.append(ctx.force(st, "left"))
                    if s_c + P["speed_mm_s"] * 0.02 > free_mm and base_acc:
                        self.base_f = np.mean(base_acc[len(base_acc) // 3:], axis=0)
                    ctx.c.hold(0.02)
                    continue
                if not near:
                    near = True                                 # from here never correct along the approach
                    w = np.cross(d_app, lat)
                    self.set_feedback([lat, w], P["fb_gain"], note="approach: near the edge, correction across the approach only")
                e = self.e
                along = float((e - start) @ d_app)
                hist.append((now, along, (s_c / 1000) - along))
                while hist and hist[0][0] < now - 2.0:
                    hist.popleft()
                r_meas = float(np.linalg.norm(e[:2] - c_xy))
                at_edge = r_meas <= spec.R + P["edge_gate_mm"] / 1000
                pushed = -(F @ u_push)
                # edge contact: the finger stalls while the command runs ahead, or the camera sees the plate move.
                # Only checked near the edge; a stall further out is friction.
                cam_moving = pg is not None and pg - (s0 if s > 0 else 0) > P["cam_contact_mm"]
                cam_n = cam_n + 1 if (cam_moving and at_edge) else 0                 # 3 frames in a row: not noise
                f_n = f_n + 1 if (pushed > P["f_on_N"] and at_edge) else 0           # contact_n cycles: not a spike
                stall = at_edge and self.stalled(hist, now, P["stall_s"], P["stall_mm"], P["lead_min_mm"])
                if P.get("debug") and now - getattr(self, "_dbg_t", 0) > 0.25:
                    self._dbg_t = now
                    ctx.log("approach", s_c=s_c, along_mm=1000 * along, r_mm=1000 * r_meas, pushed_N=pushed, F=F, plate_mm=pg,
                            cam_n=cam_n, off_mm=1000 * self.off)
                if f_n >= P["contact_n"] or cam_n >= 3 or stall:
                    contact = pt.copy()
                    e_contact = e.copy()
                    s = s0 = max(prog, s0)                          # the edge's sink schedule starts where the plate is
                    ctx.log("contact", after_mm=s_c, by="+".join([w_ for w_, c_ in (("force", f_n >= P["contact_n"]), ("camera", cam_n >= 3),
                                                                                  ("stall", stall)) if c_]),
                            force_along_push=pushed, touch=tc, plate_moved_mm=pg, finger_r_mm=1000 * r_meas, correction_mm=1000 * self.off)
                    # pushing: never correct along the push, and the height correction may move only fb_push_mm more
                    self.set_feedback([lat, self.up], P["fb_gain"], cap_mm=P["fb_push_mm"])
                    if P["press_mm"] > 0:
                        contact = contact + nrm_i * P["press_mm"] / 1000
                        self.go_to(contact, nrm_i, 0.3)
                elif s_c > (P["passed_mm"] if s == 0 else P["passed_mm_retry"]) or (along * 1000 > (P["passed_mm"] if s == 0 else P["passed_mm_retry"])):
                    # well past where the edge is, no contact, the plate has not moved: the finger went over it
                    retries += 1
                    ctx.log("retry", why="passed over the edge", approach_mm=s_c, finger_r_mm=1000 * r_meas, n=retries)
                    if retries > P["retries"]:
                        raise Abort("no contact with the edge after %d tries (rim measured at %.3f)" % (retries, rim_true))
                    self.set_feedback([X, Y, Z], P["fb_gain_free"])
                    back = start + np.array([0, 0, 0.006])
                    self.go_to(pt + np.array([0, 0, 0.006]), nrm_i, 0.4)
                    self.go_to(back, nrm_i, 0.8)
                    h -= P["retry_dz_mm"] / 1000
                    start = start - np.array([0, 0, P["retry_dz_mm"] / 1000])
                    self.go_to(start, nrm_i, 0.4)
                    s_c, cam_n, base_acc, near = 0.0, 0, [], False
                    sc_moved, t_moved = 0.0, time.time()
                    hist.clear()
                    continue
            else:
                s += P["speed_mm_s"] * dt
                g = math.radians(P["gamma_deg"] + P["roll_deg"] * min(1.0, s / P["dist_mm"]))
                nrm_i = self.nrm(g)
                pt = contact + u_push * (s - s0) / 1000 - self.up * P["follow"] * (self.drop(s) - self.drop(s0)) / 1000 + self.up * corr
                if not self.step(pt, nrm_i, st):
                    raise Abort("push left the arm's reach at %.0f mm" % s)
                Fn = float(np.linalg.norm(F))
                s_meas = s0 + 1000 * float((self.e - e_contact) @ u_push)    # how far the finger REALLY went
                if prog >= P["done_mm"] or s > P["dist_mm"] + 25:
                    ctx.log("done", finger_mm=s_meas, finger_cmd_mm=s, plate_mm=prog, force_N=Fn)
                    break
                bad = "rode over" if (self.pl0 is not None and s_meas - prog > P["ride_mm"]) else ("too hard" if Fn > P["f_hi_N"] else "")
                if bad:
                    retries += 1
                    ctx.log("retry", why=bad, finger_mm=s_meas, finger_cmd_mm=s, plate_mm=prog, force_N=Fn, n=retries)
                    if retries > P["retries"]:
                        raise Abort("%s %d times" % (bad, retries))
                    h_new = -P["retry_dz_mm"] / 1000 if bad == "rode over" else P["retry_dz_mm"] / 1000
                    self.set_feedback([X, Y, Z], P["fb_gain_free"])
                    self.go_to(pt + 0.008 * self.out + np.array([0, 0, 0.004]), nrm_i, 0.4)
                    edge_now = contact + u_push * (prog - s0) / 1000 - self.up * P["follow"] * (self.drop(prog) - self.drop(s0)) / 1000
                    start = edge_now + 0.008 * self.out + np.array([0, 0, h_new])
                    self.go_to(start, nrm_i, 0.4)
                    self.baseline(0.3)
                    contact, s_c, s0, s = None, 0.0, prog, prog
                    cam_n, near = 0, False
                    base_acc = []
                    sc_moved, t_moved = 0.0, time.time()
                    hist.clear()
                    h += h_new
                    continue
            why = self.check(st)
            if why:
                raise Abort(why)
            ctx.c.hold(0.02)
        st = ctx.st()
        Tm = ctx.fk(st=st)
        e, low = ctx.pinky(Tm)
        ctx.memory["push_end"] = dict(q=list(self.q11), pinky=e, plate_mm=prog, finger_mm=s, retries=retries,
                                      plate=(ctx.plate(st) or {}).get("center"))
        ctx.results["push"] = dict(plate_mm=prog, finger_mm=s, retries=retries, rim_z=rim_true, camera_rim_error_mm=-1000 * dz,
                                   pelvis_from_legs=h_pelvis, correction_mm=[round(1000 * v, 1) for v in self.off])
        return ctx.results["push"]


class Edge(Push):
    """`run_test.py edge`: only the proximity edge search, no push. Where the little finger finds the top plate's edge
    and its rim's height, against the camera's; then the finger lifts off and the arms go home."""
    name = "edge"

    def run(self):
        S = self.setup()
        res = self.edge_search(S["top"], S["rim_z"], S["rim_legs"], self.nrm(S["g0"]))
        out = dict(edge=[float(v) for v in res["edge"]], rim_z=res["rim_z"], r_edge_mm=1000 * res["r_edge"],
                   camera_edge_error_mm=-res["camera_error_mm"][0], camera_rim_error_mm=res["camera_error_mm"][1],
                   camera_centre=[S["cx"], S["cy"], S["rim_z"]])
        self.ctx.results["edge"] = out
        return out
