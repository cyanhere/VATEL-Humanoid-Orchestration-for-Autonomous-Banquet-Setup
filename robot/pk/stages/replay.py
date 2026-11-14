"""Stages 2-4 from the simulation's reference: first grip (robot right hand), carry, second grip (robot left hand)."""
import time

import numpy as np

from .. import config
from ..ik import shift_path
from ..plate import TASK
from ..refs import Reference
from ..stage_base import Abort, smooth

STAGE_OF = {"grip1": "2grip1", "carry": "3carry", "grip2": "4grip2"}
SIDE_OF = {"grip1": "right", "carry": "right", "grip2": "left"}
LOCK_PHASES = ("thumb locks",)


class Replay(object):
    def __init__(self, ctx, which):
        self.ctx, self.which = ctx, which
        self.name = which
        self.P = ctx.params(which)
        from ..refs import cached
        with ctx.c.busy(20.0):                       # reading the reference takes ~1 s the first time
            self.ref = cached()
        self.seg = self.ref.stage(STAGE_OF[which])
        self.side = SIDE_OF[which]
        self.other = "left" if self.side == "right" else "right"

    # ------------------------------------------------------------------ where the plate is
    def plate_now(self, st):
        ctx, which = self.ctx, self.which
        if which == "grip1":
            p = ctx.wait_plate(timeout=5.0)
            rim = ctx.memory.get("rim_z", p["center"][2])
            return np.array([p["center"][0], p["center"][1], rim - TASK.H]), "camera + touch-measured rim height"
        if which == "carry":
            return None, "grip 1's offset"
        # grip2: the plate in the right hand
        rel = ctx.memory.get("palm_to_plate")
        if rel is None:
            raise Abort("grip 2 needs grip 1 and the carry first (where the right hand holds the plate)")
        T = ctx.fk(st=st)
        M = T["right_base_link"]
        return M[:3, :3] @ np.array(rel) + M[:3, 3], "right hand x palm-to-plate offset"

    # ------------------------------------------------------------------ the stage
    def run(self):
        ctx, P, seg, side = self.ctx, self.P, self.seg, self.side
        st = ctx.st()
        q_now = np.array(st["des"])
        sl = config.LEFT if side == "left" else config.RIGHT
        so = config.RIGHT if side == "left" else config.LEFT
        n = len(seg["t"])
        dt = float(np.median(np.diff(seg["t"])))
        phase = list(seg["phase"])
        # 1. offset of the real plate from the simulation's
        if self.which == "carry":
            delta = np.array(ctx.memory.get("delta_grip1", [0.0, 0.0, 0.0]))
            how = "same as grip 1"
        else:
            p_real, how = self.plate_now(st)
            delta = p_real - seg["plate0"]
        ctx.log("offset", stage=self.which, delta_mm=1000 * delta, source=how, sim_plate=seg["plate0"])
        # a different standing height moves the table AND the stack: the vertical shift keeps every clearance, the IK
        # check below says whether it is reachable; a sideways shift changes the reach, so it is held tighter
        dxy, dz = 1000 * float(np.linalg.norm(delta[:2])), 1000 * abs(float(delta[2]))
        if dxy > P["max_shift_xy_mm"] or dz > P["max_shift_z_mm"]:
            raise Abort("the plate is %.0f mm sideways / %.0f mm in height from where the simulation had it (limits %.0f / %.0f mm): "
                        "too far to trust the reference path; move the stack closer to x %.2f y %.2f" % (
                            dxy, dz, P["max_shift_xy_mm"], P["max_shift_z_mm"], config.STACK_XY_SIM[0], config.STACK_XY_SIM[1]))
        # 2. the path: the gripping arm shifted (ramped in over the approach), both arms starting where they are
        ap_end = max([i for i, ph in enumerate(phase) if ph == "approach"] + [0])
        ramp = np.array([smooth(k / max(ap_end, 1)) for k in range(n)]) if ap_end > 0 else np.ones(n)
        deltas = ramp[:, None] * delta[None, :]
        slot = np.array(P["slot_mm"]) / 1000.0
        with ctx.c.busy(60.0):
            q_path, worst = shift_path(ctx.rob, side, "%s_base_link" % side, slot, seg["q"], seg["h"], deltas, w_rot=P["w_rot"],
                                       waist=q_now[config.WAIST])
        ctx.log("plan", note="reference path moved onto the plate (solved for the robot's waist)", worst_ik_mm=worst, waypoints=n,
                waist_rad=float(q_now[config.WAIST]))
        if worst > 3.0:
            raise Abort("the moved path leaves the arm's reach (%.1f mm)" % worst)
        # both arms: start where they really are; the offset fades out over the approach (gripping arm) or, for the
        # other arm, holds (it is somewhere real, e.g. on the far rim) and fades during "left away"
        off_g = q_now[sl] - q_path[0, sl]
        off_o = q_now[so] - q_path[0, so]
        fade_o = np.ones(n)
        for i, ph in enumerate(phase):
            if ph == "left away":
                ks = [k for k, p_ in enumerate(phase) if p_ == "left away"]
                fade_o[i] = 1.0 - smooth((i - ks[0]) / max(len(ks) - 1, 1))
            elif ks_after(phase, i, "left away"):
                fade_o[i] = 0.0
        q_path = q_path.copy()
        q_path[:, sl] += (1.0 - ramp)[:, None] * off_g[None, :]
        q_path[:, so] += fade_o[:, None] * off_o[None, :]
        q_path[:, config.WAIST] = q_now[config.WAIST]
        if P.get("use_correction"):
            # tune.py track <stage> --learn: what this arm missed along this path, added (it fades in with the approach)
            import json as _json
            import os as _os
            cp = _os.path.join(config.REFS_DIR, "corrections", "%s.json" % self.which)
            if _os.path.exists(cp):
                corr = np.array(_json.load(open(cp))["dq"])
                if corr.shape == q_path.shape:
                    q_path = q_path + ramp[:, None] * corr
                    ctx.log("correction", note="learned tracking correction added", largest_rad=float(np.abs(corr).max()))
        big = np.max(np.abs(q_path[0] - q_now))
        if big > 0.02:
            raise Abort("internal: the path does not start where the arms are (%.3f rad)" % big)
        # 3. the fingers: reference commands + the porcelain rules
        h_path = seg["h"].copy()
        hb = 0 if side == "left" else 6
        margin = np.array([P["close_margin_rad"]] * 6) / config.HAND_FULL_RAD
        lock_idx = [i for i, ph in enumerate(phase) if ph in LOCK_PHASES]
        closing = np.zeros(6, bool)
        if lock_idx:
            closing = h_path[lock_idx[-1], hb:hb + 6] > h_path[lock_idx[0], hb:hb + 6] + 1e-3
        state = dict(hold=None, k_last=-1)

        def hand_rule(st_, k, h):
            h = np.array(h, float)
            meas = np.array(st_["hands"])
            ph = phase[k]
            after_lock = bool(lock_idx) and k > lock_idx[-1]
            if ph in LOCK_PHASES:
                for i in range(6):
                    j = hb + i
                    if closing[i] and h[j] > meas[j] + margin[i]:
                        h[j] = meas[j] + margin[i]
            elif after_lock and P["relax"]:
                if state["hold"] is None:            # once, at the end of the locking: hold where the rim stopped them
                    hold = h[hb:hb + 6].copy()
                    for i in range(6):
                        if closing[i]:
                            r = P["relax_thumb_rad"] if i <= 1 else P["relax_rad"]
                            hold[i] = meas[hb + i] + r / config.HAND_FULL_RAD[i]
                    state["hold"] = hold
                    ctx.log("fingers", note="locked; holding at measured + relax", held=hold, measured=meas[hb:hb + 6])
                for i in range(6):
                    if closing[i]:
                        h[hb + i] = state["hold"][i]
            # touch cap: a pad pressing hard closes no further
            t = st_.get("touch")
            if t:
                side_i = 0 if side == "left" else 1
                for fi, f in enumerate(config.TOUCH_FINGERS):
                    if t[side_i][fi][0] > P["touch_max"]:
                        for i in ((0, 1) if f == "Thumb" else (config.FINGERS.index(f),)):
                            h[hb + i] = min(h[hb + i], float(st_["hands_cmd"][hb + i]))
            return np.clip(h, 0, 1)

        f_peak = dict(v=0.0)

        def check(st_, k):
            t = st_.get("touch")
            if t:
                side_i = 0 if side == "left" else 1
                mx = max(t[side_i][fi][0] for fi in range(5))
                f_peak["v"] = max(f_peak["v"], mx)
            return ""

        # carry: the plate's weight on the right hand
        if self.which in ("carry", "grip2") and P.get("payload_kg", 0) > 0:
            ctx.c.set(payload=dict(side="right", mass=P["payload_kg"], link="right_base_link", p=list(slot)))
            ctx.log("payload", kg=P["payload_kg"], note="plate weight added to the right arm's gravity compensation")
        k_split = phase.index("both hold") if (self.which == "grip2" and "both hold" in phase and P.get("arm_relax")) else None
        if k_split is None:
            why, k_end = ctx.play(q_path, h_path, dt, speed=P["speed"], check=check, tags=phase, hand_rule=hand_rule)
            if why:
                raise Abort(why)
        else:
            why, k_end = ctx.play(q_path[:k_split + 1], h_path[:k_split + 1], dt, speed=P["speed"], check=check,
                                  tags=phase[:k_split + 1], hand_rule=hand_rule)
            if why:
                raise Abort(why)
            # both hands hold one rigid plate: re-anchor both arms to where they ARE (no fight through the plate),
            # and the first hand eases to just past where its fingers touch
            st = ctx.st()
            q_meas = np.array(st["q"])
            shift = q_meas - q_path[k_split]
            shift[config.WAIST] = 0.0
            rest = q_path[k_split:] + shift[None, :]
            rest[:, config.WAIST] = q_now[config.WAIST]
            ctx.c.move_to(rest[0], v=0.05, tag="both hold: re-anchor")
            meas_h = np.array(st["hands"])
            h_rest = h_path[k_split:].copy()
            ob = 6 if side == "left" else 0                        # the OTHER (first-grip) hand
            for i in range(6):
                if h_rest[0, ob + i] > 0.05:
                    h_rest[:, ob + i] = min(h_rest[0, ob + i], meas_h[ob + i] + P["relax_r_rad"] / config.HAND_FULL_RAD[i])
            ctx.log("both hold", arms_moved_rad=shift, first_hand_eased_to=h_rest[0, ob:ob + 6])
            why, k_end = ctx.play(rest, h_rest, dt, speed=P["speed"], check=check, tags=phase[k_split:], hand_rule=hand_rule)
            if why:
                raise Abort(why)
        if self.which == "carry" and P.get("enc_fb", 0) > 0:
            # sag correction at the end of the carry (grip_centre.py --enc_fb): target += k (planned - measured)
            st = ctx.st()
            q_end = q_path[-1].copy()
            err = q_end - np.array(st["q"])
            q_corr = q_end.copy()
            q_corr[sl] += P["enc_fb"] * err[sl]
            ctx.c.move_to(q_corr, v=0.1, tag="carry sag correction")
            ctx.log("sag", note="corrected the end of the carry", err_rad=err[sl])
        st = ctx.st()
        res = dict(delta_mm=list(1000 * delta), touch_peak=f_peak["v"])
        if self.which == "grip1":
            ctx.memory["delta_grip1"] = list(delta)
            # where the plate sits in the right hand: from the camera (it is still near the stack: grip 1 lifts 0 mm)
            p = ctx.plate(st)
            M = ctx.fk(st=st)["right_base_link"]
            if p and p.get("ok"):
                c = np.array([p["center"][0], p["center"][1], ctx.memory.get("rim_z", p["center"][2]) - TASK.H])
                src = "camera"
            else:
                c = seg["plate1"] + delta
                src = "simulation + offset"
            ctx.memory["palm_to_plate"] = list(M[:3, :3].T @ (c - M[:3, 3]))
            res["plate_in_hand_from"] = src
        ctx.results[self.which] = res
        return res


def ks_after(phase, i, name):
    """True when index i comes after the last `name` phase."""
    idx = [k for k, p in enumerate(phase) if p == name]
    return bool(idx) and i > idx[-1]
