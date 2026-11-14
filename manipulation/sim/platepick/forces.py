"""Contact forces on the top plate per hand part, and between the top plate and the plate below. Keeps the raw peak and
the peak of a 50 ms moving average.
"""
from collections import deque

import mujoco
import numpy as np

PARTS = ("index", "middle", "ring", "pinky", "thumb", "palm")


class ForceMeter:
    def __init__(self, m, hand, top, below=None, window_s=0.05, hands=None):
        self.m, self.top, self.below = m, top, below
        hands = hands or [hand]
        self.part = {}                      # geom -> (hand, part)
        for g in range(m.ngeom):
            nm = m.body(m.geom_bodyid[g]).name
            for h in hands:
                if nm.startswith(h + "_"):
                    p = next((f for f in PARTS[:5] if f"_{f}_" in nm), "palm")
                    self.part[g] = (h, p)
        self.keys = [(h, p) for h in hands for p in PARTS]
        self.n = max(1, int(round(window_s / m.opt.timestep)))
        self.buf = {k: deque(maxlen=self.n) for k in self.keys + [("plate", "below")]}
        self.peak = {k: 0.0 for k in self.buf}
        self.peak_avg = {k: 0.0 for k in self.buf}
        self.hand_peak = {h: 0.0 for h in hands}
        self.hand_peak_avg = {h: 0.0 for h in hands}
        self.hbuf = {h: deque(maxlen=self.n) for h in hands}
        self.now = {}
        self.w = np.zeros(6)
        self.t_peak = {}                    # time (s) of each part's 50 ms peak
        self.trace = []                     # (t, {part: N}) every 5 ms, kept only with trace=True
        self.keep_trace = False
        self.k = 0
        self.th = {h: dict(lock=None, travel=0.0, end=0.0, off_s=0.0, in_min=1e9, in_end=None, t_lock=None, F=0.0, path=[]) for h in hands}
        self.R_edge = None                  # plate radius (m) for "in from the edge"; set by the caller

    def update(self, d):
        m = self.m
        cur = {k: 0.0 for k in self.buf}
        tp = {}
        if d.ncon == 0:
            g1 = g2 = np.zeros(0, int)
        else:
            g1, g2 = np.asarray(d.contact.geom1[:d.ncon]), np.asarray(d.contact.geom2[:d.ncon])
        bb1, bb2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
        for i in np.nonzero((bb1 == self.top) | (bb2 == self.top))[0]:
            b1, b2 = bb1[i], bb2[i]
            other_g = g2[i] if b1 == self.top else g1[i]
            other_b = b2 if b1 == self.top else b1
            k = self.part.get(other_g)
            if k is None and other_b != self.below:
                continue
            mujoco.mj_contactForce(m, d, i, self.w)
            f = float(np.linalg.norm(self.w[:3]))
            cur[k if k is not None else ("plate", "below")] += f
            if k is not None and k[1] == "thumb":
                tp.setdefault(k[0], [0.0, np.zeros(3)])
                tp[k[0]][0] += f
                tp[k[0]][1] += f * d.contact.pos[i]
        for k, v in cur.items():
            self.buf[k].append(v)
            self.peak[k] = max(self.peak[k], v)
            av = sum(self.buf[k]) / len(self.buf[k])
            if av > self.peak_avg[k]:
                self.peak_avg[k] = av
                self.t_peak[k] = round(float(d.time), 3)
        for h in self.hbuf:
            v = sum(cur[(h, p)] for p in PARTS)
            self.hbuf[h].append(v)
            self.hand_peak[h] = max(self.hand_peak[h], v)
            self.hand_peak_avg[h] = max(self.hand_peak_avg[h], sum(self.hbuf[h]) / len(self.hbuf[h]))
        self.now = cur
        self.k += 1
        P, R = d.xpos[self.top], d.xmat[self.top].reshape(3, 3)
        for h, st in self.th.items():
            F = tp.get(h, [0.0])[0]
            if F > 0.5:
                lp = R.T @ (tp[h][1] / F - P)
                if st["lock"] is None and F > 5.0:
                    st["lock"], st["t_lock"] = lp, round(float(d.time), 3)
                if st["lock"] is not None:
                    st["end"] = float(np.linalg.norm(lp - st["lock"])) * 1000
                    st["travel"] = max(st["travel"], st["end"])
                    if self.R_edge:
                        st["in_end"] = (self.R_edge - float(np.hypot(lp[0], lp[1]))) * 1000
                        st["in_min"] = min(st["in_min"], st["in_end"])
                    if self.k % max(1, int(round(0.05 / m.opt.timestep))) == 0:
                        st["path"].append((round(float(d.time), 2), round(st["end"], 1), round(st["in_end"], 1) if st["in_end"] is not None else None, round(F, 1)))
            elif st["lock"] is not None:
                st["off_s"] += m.opt.timestep
                st["end"] = float("nan")
            st["F"] = F
        if self.keep_trace and self.k % max(1, int(round(0.005 / m.opt.timestep))) == 0:
            self.trace.append((round(float(d.time), 3), {f"{a}_{b}": round(v, 1) for (a, b), v in cur.items() if v > 0.05}))
        return cur

    def summary(self):
        r = lambda x: round(float(x), 1)  # noqa: E731
        out = {}
        for h in self.hbuf:
            out[f"{h}_hand_peak_N"] = r(self.hand_peak[h])
            out[f"{h}_hand_peak50ms_N"] = r(self.hand_peak_avg[h])
            out[f"{h}_part_peak50ms_N"] = {p: r(self.peak_avg[(h, p)]) for p in PARTS if self.peak_avg[(h, p)] > 0.05}
            out[f"{h}_part_peak_N"] = {p: r(self.peak[(h, p)]) for p in PARTS if self.peak[(h, p)] > 0.05}
            out[f"{h}_part_max50ms_N"] = r(max([self.peak_avg[(h, p)] for p in PARTS]))
            out[f"{h}_part_peak_time_s"] = {p: self.t_peak[(h, p)] for p in PARTS if (h, p) in self.t_peak and self.peak_avg[(h, p)] > 0.05}
            st = self.th[h]
            if st["lock"] is not None:
                out[f"{h}_thumb"] = dict(t_lock=st["t_lock"], travel_max_mm=r(st["travel"]), travel_end_mm=r(st["end"]) if st["end"] == st["end"] else None,
                                         off_after_lock_s=r(st["off_s"]), F_end_N=r(st["F"]), path_t_travel_in_F=st["path"],
                                         **(dict(in_from_edge_min_mm=r(st["in_min"]), in_from_edge_end_mm=r(st["in_end"])) if st["in_end"] is not None else {}))
        out["plate_on_plate_peak_N"] = r(self.peak[("plate", "below")])
        out["plate_on_plate_peak50ms_N"] = r(self.peak_avg[("plate", "below")])
        return out
