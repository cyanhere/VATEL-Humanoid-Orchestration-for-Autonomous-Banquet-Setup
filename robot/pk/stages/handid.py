"""Hand identification: how each Revo2 finger channel follows its command, both hands (params/handid.json)."""
import csv
import json
import os
import sys
import threading
import time

import numpy as np

from .. import config
from ..stage_base import Abort
from .armwide import ArmWide

CH = ["thumb bend", "thumb rot", "index", "middle", "ring", "little"]
LETTER = dict(t="Thumb", i="Index", m="Middle", r="Ring", p="Pinky")


class HandId(ArmWide):
    name = "handid"

    def __init__(self, ctx, mode="track"):
        self.ctx, self.mode = ctx, mode
        if mode not in ("track", "load"):
            raise Abort("handid: track or load, not %r" % (mode,))
        self.P = ctx.params("handid")
        self.side = self.P["side"]
        if self.side not in (("left", "right", "both") if mode == "track" else ("left", "right")):
            raise Abort("handid_%s: handid.side cannot be %r" % (mode, self.side))
        arm = "left" if self.side == "both" else self.side
        self.sl = config.LEFT if arm == "left" else config.RIGHT
        self.arm = arm
        self.hand_ks = [0, 6] if self.side == "both" else [0 if self.side == "left" else 6]
        self.plan = json.load(open(os.path.join(config.PARAMS_DIR, "armwide_plan.json")))
        self.csv = None
        self.events = []
        self.checking = True

    # ================================================================== the log
    def build(self, st):
        self.q11 = np.array(st["des"], float)
        self.hands = np.array(st["hands_cmd"], float)
        self.qc = self.q11[self.sl].copy()
        self.q_home = self.qc.copy()
        self.h_start = self.hands.copy()

    def open_csv(self):
        cols = ["t", "phase"] + ["hcmd%d" % i for i in range(12)] + ["hq%d" % i for i in range(12)]
        cols += ["tn_%s%d" % (s, i) for s in "LR" for i in range(5)] + ["tp_%s%d" % (s, i) for s in "LR" for i in range(5)]
        cols += ["q_meas%d" % i for i in range(10)] + ["q_cmd%d" % i for i in range(5)] + ["f_hand_L", "f_hand_R"]
        self.csv_path = os.path.join(self.ctx.out_dir, "handid_cmd.csv")
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv = csv.DictWriter(self.csv_f, fieldnames=cols)
        self.csv.writeheader()
        self.csv_n = 0

    def row(self, st, phase):
        r = dict(t=round(time.time() - self.t0, 3), phase=phase)
        hc, hq = np.array(st["hands_cmd"], float), np.array(st["hands"], float)
        for i in range(12):
            r["hcmd%d" % i] = round(float(hc[i]), 4)
            r["hq%d" % i] = round(float(hq[i]), 4)
        tc = st.get("touch")
        for k, s in enumerate("LR"):
            for i in range(5):
                r["tn_%s%d" % (s, i)] = tc[k][i][0] if tc else ""
                r["tp_%s%d" % (s, i)] = tc[k][i][3] if tc else ""
        q = np.array(st["q"], float)
        for i in range(10):
            r["q_meas%d" % i] = round(float(q[i]), 5)
        for i in range(5):
            r["q_cmd%d" % i] = round(float(self.qc[i]), 5)
        fh = st.get("f_hand", {})
        r.update(f_hand_L=fh.get("left"), f_hand_R=fh.get("right"))
        self.csv.writerow(r)
        self.csv_n += 1
        if self.csv_n % 30 == 0:
            self.csv_f.flush()

    # ================================================================== hands
    def hand_go(self, target6, speed, phase):
        """Each tested hand toward its target (6 values, or 12 for both hands' own) at `speed` per second, the arm held."""
        target = np.asarray(target6, float)
        tg = np.array(self.hands)
        for k in self.hand_ks:
            tg[k:k + 6] = target[k:k + 6] if len(target) == 12 else target
        dt = 1.0 / self.P["rate_hz"]
        for _ in range(int(60 * self.P["rate_hz"])):
            if float(np.max(np.abs(tg - self.hands))) < 1e-4:
                return
            self.hands = self.hands + np.clip(tg - self.hands, -speed * dt, speed * dt)
            self.tick(self.qc, phase)
        raise Abort("hands did not get there in 60 s")

    def hand_hold(self, seconds, phase):
        for _ in range(int(seconds * self.P["rate_hz"])):
            self.tick(self.qc, phase)

    def base(self, c=None, v=None):
        b = np.zeros(6)
        if c is not None:
            b[c] = v
        return b

    def run_track(self, res):
        P = self.P
        fast = float(P["step_speed"])
        self.vmax = 0.0
        self.hand_go(self.base(), 0.3, "hopen")
        self.hand_hold(1.0, "hopen")
        for c in (P["channels"] or list(range(6))):
            self.ev("channel", channel=c, name=CH[c])
            for s in P["step_sizes"]:                                   # a step: the command jumps, the finger follows
                self.hand_go(self.base(c, s), fast, "c%d_st%g" % (c, s))
                self.hand_hold(P["step_hold_s"], "c%d_st%g" % (c, s))
                self.hand_go(self.base(), fast, "c%d_st%g" % (c, -s))
                self.hand_hold(P["step_hold_s"], "c%d_st%g" % (c, -s))
            for v in P["ramp_speeds"]:                                  # ramps open -> closed -> open
                self.hand_go(self.base(c, P["ramp_hi"]), v, "c%d_rp%g" % (c, v))
                self.hand_hold(0.8, "c%d_rp%g" % (c, v))
                self.hand_go(self.base(), v, "c%d_rp%g" % (c, v))
                self.hand_hold(0.8, "c%d_rp%g" % (c, v))
            m = float(P["rev_centre"])
            self.hand_go(self.base(c, m), 0.3, "c%d_mv" % c)
            self.hand_hold(1.0, "c%d_mv" % c)
            for a in P["rev_amps"]:                                     # back and forth: the dead band
                for _ in range(int(P["rev_cycles"])):
                    self.hand_go(self.base(c, m + a), P["rev_speed"], "c%d_rv%g" % (c, a))
                    self.hand_go(self.base(c, m - a), P["rev_speed"], "c%d_rv%g" % (c, a))
                self.hand_go(self.base(c, m), P["rev_speed"], "c%d_rv%g" % (c, a))
                self.hand_hold(0.7, "c%d_rv%g" % (c, a))
            levels = list(P["levels"])
            for n, x in enumerate(levels + levels[-2::-1]):             # up, then down: film these from above
                self.hand_go(self.base(c, x), 0.3, "c%d_mv" % c)
                self.hand_hold(P["level_hold_s"], "c%d_lv%g_%s" % (c, x, "up" if n < len(levels) else "dn"))
            self.hand_go(self.base(), 0.3, "c%d_mv" % c)
        if P["multisine_s"] > 0:
            self.ev("multisine", seconds=P["multisine_s"])
            rng = np.random.RandomState(int(P["multisine_seed"]))
            w = 2 * np.pi * np.array(P["multisine_hz"], float)
            hi = np.array(P["multisine_max"], float)
            amp = rng.uniform(0.5, 1.0, (6, len(w))) / (1.0 + w / w[0]) ** 0.5
            ph = rng.uniform(0, 2 * np.pi, (6, len(w)))
            t = np.arange(0.0, float(P["multisine_s"]), 1.0 / P["rate_hz"])
            U = np.sum(amp[None] * np.sin(w[None, None] * t[:, None, None] + ph[None]), axis=2)
            U = (U - U.min(0)) / np.maximum(U.max(0) - U.min(0), 1e-9)
            f = np.minimum(1.0, np.minimum(t, t[-1] - t) / 2.0)[:, None]
            U = 0.05 + f * U * (hi - 0.05)                              # fade in and out from almost open
            U[:, hi <= 0] = 0.0
            self.hand_go(U[0], 0.3, "ms_mv")
            for row in U:
                for k in self.hand_ks:
                    self.hands[k:k + 6] = row
                self.tick(self.qc, "ms")
            self.hand_go(self.base(), 0.3, "ms_mv")
        self.hand_go(self.h_start, 0.3, "hback")
        res["channels"] = P["channels"] or list(range(6))

    # ================================================================== load
    def labels(self):
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

    def run_load(self, res):
        P = self.P
        pu = self.plan[self.arm].get("palm_up")
        if not pu:
            raise Abort("no palm-up pose in the plan: run calibration/armwide_plan.py")
        self.go(np.array(pu["q"], float), P["move_v_rad_s"], "lmove")
        shape = np.array(P["load_shape"], float)
        self.hand_go(shape, 0.3, "lshape")
        self.hold(1.0, "lshape")
        print("\nPALM UP, HOLDING. Hang a weight on one finger (a loop over the pad), wait until it stops swinging, then type its label + Enter: "
              "i0, i50, i100 ... (t i m r p, grams; 0 = the loop alone). 'curl 0.6' + Enter re-curls the four fingers. q + Enter ends.\n", flush=True)
        q = self.labels()
        label, t0, n = "none", time.time(), 0
        while time.time() - t0 < P["load_max_s"]:
            if q:
                lab = q.pop(0)
                if lab == "q":
                    break
                if lab.startswith("curl"):
                    try:
                        v = float(lab.split()[1])
                    except (IndexError, ValueError):
                        print("  'curl 0.6': a number 0..%.1f" % P["curl_max"], flush=True)
                        continue
                    v = min(max(v, 0.0), float(P["curl_max"]))
                    shape[2:] = v
                    self.hand_go(shape, 0.3, "lshape")
                    self.ev("curl", value=v)
                    print("  fingers curled to %.2f" % v, flush=True)
                    label = "none"
                    continue
                if lab[0] not in LETTER:
                    print("  label: finger letter t i m r p and grams, e.g. i100", flush=True)
                    continue
                label = lab.replace(" ", "")
                n += 1
                self.ev("load", label=label, curl=float(shape[2]))
                print("  logging as %s at curl %.2f" % (label, shape[2]), flush=True)
            self.tick(self.qc, "load_%s_c%g" % (label, shape[2]))
        self.hand_go(self.h_start, 0.3, "lback")
        self.go(self.q_home, P["move_v_rad_s"], "lback")
        res["loads"] = n

    # ================================================================== run
    def run(self):
        ctx = self.ctx
        self.t0 = time.time()
        st = ctx.st()
        self.check_start(st)
        if not st.get("hands_ok", True):
            raise Abort("no hand state: is the hand server running?")
        self.build(st)
        self.open_csv()
        meta = dict(side=self.side, mode=self.mode, arm=self.arm, channels=CH, rate_hz=self.P["rate_hz"], touch_fingers=config.TOUCH_FINGERS)
        json.dump(meta, open(os.path.join(ctx.out_dir, "handid_meta.json"), "w"))
        res = dict(mode=self.mode, side=self.side, log=self.csv_path)
        ctx.c.set(vmax=0.78, hands=self.hands, tag="handid")
        self.t_next = time.time()
        try:
            getattr(self, "run_" + self.mode)(res)
            res["result"] = "played"
        except Abort as ex:
            res["result"] = "aborted: %s" % ex
            self.finish(res)
            raise
        return self.finish(res)

    def finish(self, res):
        res["events"] = self.events
        if self.csv is not None:
            self.csv_f.flush()
        res["seconds"] = round(time.time() - self.t0, 1)
        self.ctx.results["handid"] = res
        self.ctx.log("handid", result=res.get("result"), log=self.csv_path, seconds=res["seconds"])
        return res

    def dry(self, st, plate):
        self.check_start(st)
        P = self.P
        if self.mode == "track":
            T = 0.0
            for _ in (P["channels"] or range(6)):
                T += sum(2 * (s / P["step_speed"] + P["step_hold_s"]) for s in P["step_sizes"])
                T += sum(2 * (P["ramp_hi"] / v + 0.8) for v in P["ramp_speeds"])
                T += sum(P["rev_cycles"] * 4 * a / P["rev_speed"] + 0.7 for a in P["rev_amps"]) + 4
                T += (2 * len(P["levels"]) - 1) * (P["level_hold_s"] + 1.0)
            T += P["multisine_s"] + 6
            print("handid_track, %s hand(s), arms still at the pre-grip pose: channels %s, about %.0f min" % (self.side, P["channels"] or list(range(6)), T / 60))
            print("  steps %s at %.1f/s; ramps at %s/s to %.2f; back-and-forth %s around %.2f; holds %s for %.0f s each (film from above); %.0f s all fingers, max %s" % (
                P["step_sizes"], P["step_speed"], P["ramp_speeds"], P["ramp_hi"], P["rev_amps"], P["rev_centre"], P["levels"], P["level_hold_s"],
                P["multisine_s"], P["multisine_max"]))
            return
        pu = self.plan[self.arm].get("palm_up")
        q0 = np.array(st["des"], float)[self.sl]
        print("handid_load, %s hand: palm-up pose %s (%.2f rad from where the arm is now), index pad at %s (pelvis frame); shape %s" % (
            self.arm, pu and np.round(pu["q"], 2), float(np.max(np.abs(np.array(pu["q"]) - q0))) if pu else float("nan"), pu and pu["pad"], P["load_shape"]))
