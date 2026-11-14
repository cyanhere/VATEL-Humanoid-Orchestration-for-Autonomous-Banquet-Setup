"""Offline run of nearforce against the simulated arm and a simple spring plate (logic only).

    python tests/test_nearforce_mock.py --all          # every case below; figure logs/test_nearforce_mock.png
    python tests/test_nearforce_mock.py --case slide
"""
import argparse
import csv
import json
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from mock_world import C, K, FakePerception  # noqa: E402
from pk import config  # noqa: E402
from pk.client import Client, operator  # noqa: E402
from pk.plate import TASK  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.push10 import PAD_POINT  # noqa: E402

PORT = 9725
KT = 2000.0         # N/m, a pad's tangential stiffness while stuck


class Plate(object):
    def __init__(self, center, rim_z, mu_pad=0.25, mu_plate=0.38, weight=5.126, n_above_foot=None, taped=False, alone=False, mid_low_mm=0.5, step_mm=0.0):   # mid_low: both fingers land within about 0.3 s, as on the R1; the mock's fingers do not flex
        self.c = np.array(center, float)
        self.c0 = self.c.copy()
        self.rim_z, self.mu_pad, self.mu_plate, self.W, self.taped, self.alone = rim_z, mu_pad, mu_plate, weight, taped, alone
        self.mid_low = mid_low_mm / 1000
        self.tilt = 0.0
        self.lock = threading.Lock()
        self.anchor = {}
        self.N, self.Ft, self.r = {}, {}, {}
        self.peak_N = 0.0
        self.slipped = False
        self.moved_by = 0.0
        self.step = step_mm / 1000
        self.n_plates = 1 if alone else 3

    def marker(self):
        return self.c + np.array([0.0, 1.0]) * 0.25 * self.tilt        # a tilt reads as travel, as on the R1

    def contact(self, p, v, name):
        F = np.zeros(3)
        with self.lock:
            d = p[:2] - self.c
            r = float(np.linalg.norm(d))
            base = self.rim_z - TASK.H - self.tilt * max(r - 0.0775, 0.0)
            zs = base + TASK.z_top(min(r, TASK.R)) + 0.0024 + (self.mid_low if name == "middle" else 0.0)
            if r < TASK.R + 0.004 and p[2] < zs:
                N = max(0.0, K * (zs - p[2]) - C * float(v[2]))
                F[2] = N
                a = self.anchor.setdefault(name, p[:2].copy())
                t = KT * (p[:2] - a)
                m = float(np.linalg.norm(t))
                lim = self.mu_pad * N
                if m > lim:                                        # the pad slides over the glaze
                    t = t * lim / max(m, 1e-9)
                    self.anchor[name] = p[:2] - t / KT
                    if m > 0.5:
                        self.slipped = True
                F[:2] -= t
                self.N[name], self.Ft[name], self.r[name] = N, t, r
            else:
                self.anchor.pop(name, None)
                self.N[name], self.Ft[name], self.r[name] = 0.0, np.zeros(2), r
            Nsum = sum(self.N.values())
            self.peak_N = max(self.peak_N, Nsum)
            T = sum(self.Ft.values()) if self.Ft else np.zeros(2)
            lim = self.mu_plate * (self.W + Nsum)
            if not self.taped and np.linalg.norm(T) > lim:          # the plate goes with the pads
                ex = T * (1 - lim / np.linalg.norm(T))
                dx = ex / (2 * KT)
                self.c += dx
                self.moved_by += float(np.linalg.norm(dx))
                for k in self.anchor:
                    self.anchor[k] = self.anchor[k] + dx
            if self.alone:
                moment = sum(self.N[k] * (self.r[k] - 0.0775) for k in self.N)
                if moment > self.W * 0.0775:
                    self.tilt += 4e-4
        return F


def prox_of(N):
    """As the R1 roughly: nothing in free air, 100k or more at contact, rising with the load."""
    return 0.0 if N < 0.02 else 100000.0 + 60000.0 * N


def set_touch(io):
    io.pad_counts_per_N, io.pad_dead_N, io.prox_contact = 3.0, 4.0, prox_of


def world(case):
    rim_z = 0.0726
    alone = case == "tip"
    if alone:
        rim_z -= 2 * 0.0068
    P = dict(center=(0.3535, 0.0885) if case == "offset" else (0.36, 0.06), rim_z=rim_z, alone=alone, taped=case == "slip")
    if case == "onefinger":
        P.update(mid_low_mm=-40.0)                  # nothing under the middle finger
    if case == "padslip":
        P.update(mu_pad=0.05)
    if case == "cap":
        P.update(mu_plate=3.0, mu_pad=3.0)
    if case in ("slide", "shape", "touch", "press", "offset"):
        P.update(mu_plate=0.3, mu_pad=1.5)          # a pad that grips: not a prediction, only so that this stop is reached
    return Plate(**P)


def run_case(case, no_touch=False, answer=1, marker=True, cor=None):
    plate = world(case)
    io = MockBackend(grav_scale=1.2, friction=0.3)
    io.touch_alive = not no_touch
    set_touch(io)
    if case == "noprox":
        io.prox_contact = None
    io.contact = plate.contact
    io.contact_pts = [("index", "left_index_touch_link", PAD_POINT, None if no_touch else "Index"), ("middle", "left_middle_touch_link", PAD_POINT, None if no_touch else "Middle")]
    srv = Server(io, port=PORT, verbose=False, log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
    run = dict(stop=False)

    def loop():
        nxt = time.time()
        while not run["stop"]:
            srv.step()
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))
    threading.Thread(target=loop, daemon=True).start()
    time.sleep(0.3)
    operator("take", port=PORT)
    while srv.mode != "HELD":
        time.sleep(0.05)
    operator("home", port=PORT)
    while srv.mode != "HOME":
        time.sleep(0.05)
    per = PlatePerception(plate, marker)
    time.sleep(0.5)
    c = Client("test nearforce mock", port=PORT)
    out = os.path.join(os.path.dirname(HERE), "logs", "test", "nearforce_%s_%s" % (case, time.strftime("%H%M%S")))
    ov = {"nearforce.auto_answer": answer, "nearforce.shape_hold_s": 1.0, "nearforce.press_step_hold_s": 0.7, "nearforce.slip_repeats": 1, "nearforce.slip_press_N": [3.0, 5.0],
          "nearforce.fz_max_N": 40.0 if case == "tip" else 12.0, "nearforce.tip_cap_N": 25.0}
    if cor is not None:
        ov["nearforce.q_correct_mrad"] = cor
    if case == "cap":
        ov["nearforce.h_cap_N"] = 6.0
    with c.busy(20):
        ctx = Ctx(c, out, overrides=ov)
    from pk.stages.nearforce import NearForce
    mode = {"slide": "full", "padslip": "full", "cap": "full", "offset": "full", "nofingers": "touch", "refuse": "touch", "noprox": "touch", "onefinger": "touch"}.get(case, case)
    ok, res, why = True, {}, ""
    t0 = time.time()
    try:
        res = NearForce(ctx, mode).run()
    except Abort as ex:
        ok, why = False, str(ex)
        res = ctx.results.get("nearforce", {})
    c.bye()
    json.dump(dict(ended="completed" if ok else "aborted: %s" % why, results={"nearforce": res}), open(os.path.join(out, "summary.json"), "w"), default=lambda x: np.asarray(x).tolist())
    t1 = time.time()
    while srv.mode != "HOME" and time.time() - t1 < 120:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    run["stop"] = True
    per.stop = True
    time.sleep(0.1)
    srv.close()
    return dict(case=case, ok=ok, why=why, res=res, home=home, plate=plate, seconds=time.time() - t0, moved=1000 * float(np.linalg.norm(plate.c - plate.c0)), out=out)


class PlatePerception(FakePerception):
    def __init__(self, plate, marker):
        FakePerception.__init__(self, plate_stack(plate), port=PORT, noise_mm=0.2)
        self.extra = (lambda: dict(marker=dict(ok=True, xy=[round(float(x), 5) for x in plate.marker()], angle_deg=0.0, dark_px=100))) if marker else None


def plate_stack(plate):
    plate.pitch = 0.0068
    return plate


def largest_step(out):
    """The largest one-tick change of the arm's target once in contact, mrad."""
    p = os.path.join(out, "nearforce_cmd.csv")
    if not os.path.exists(p):
        return 0.0
    R = list(csv.DictReader(open(p)))
    PH = ("zero", "contact", "press", "hold", "settled", "push", "tip", "unload")
    m = 0.0
    for a, b in zip(R[:-1], R[1:]):
        if a["phase"] in PH and b["phase"] in PH and a.get("q_tgt0") and b.get("q_tgt0"):
            m = max(m, max(abs(float(a["q_tgt%d" % i]) - float(b["q_tgt%d" % i])) for i in range(5)))
    return 1000 * m


def check(r):
    case, res = r["case"], r["res"]
    checks = [("arms back home", r["home"])]
    ev = {e["ev"]: e for e in res.get("events", [])}
    step = largest_step(r["out"])
    checks.append(("no jump at contact: largest target step %.1f mrad (the R1's first press: 188)" % step, step < 10.0))
    if case == "shape":
        checks.append(("shape held, no touch, plate still (%.1f mm)" % r["moved"], res.get("result") == "shape held" and r["moved"] < 0.5))
    if case in ("touch", "press", "slide", "offset"):
        L = res.get("landing") or {}
        checks.append(("both fingers landed: %s; in from the edge index %s, middle %s mm" % (ev.get("touch", {}).get("by"), (L.get("index") or {}).get("edge_gap_mm"), (L.get("middle") or {}).get("edge_gap_mm")),
                       ev.get("touch", {}).get("by") == "both" and bool(L)))
        z = ev.get("zero load") or {}
        checks.append(("the load came off at zero: %s" % z, abs(z.get("hand_rose_mm", 99.0)) < 3.0))
    if case == "press":
        lad = res.get("ladder", [])
        checks.append(("ladder played, up and down: %s; peak in the mock %.1f N" % ([(x["Fz"], x["sink_mm"]) for x in lad], r["plate"].peak_N), len(lad) == 8 and 5.0 < r["plate"].peak_N < 12.0))
        checks.append(("proximity follows the press (%s at 8 N, %s at 0 N)" % (lad[3]["prox"][0] if len(lad) > 3 else None, lad[-1]["prox"][0] if lad else None), len(lad) == 8 and lad[3]["prox"][0] > lad[-1]["prox"][0]))
    if case == "noprox":
        checks.append(("no proximity and no pad: stops at the depth limit (%s); peak in the mock %.1f N" % (r["why"], r["plate"].peak_N), not r["ok"] and "nothing touched" in r["why"] and r["plate"].peak_N < 25.0))
    if case == "onefinger":
        checks.append(("one finger only: asks, and says so (%s)" % ev.get("touch", {}).get("by"), ev.get("touch", {}).get("by") == "one finger" and "question" in ev and "only" in ev["question"]["q"]))
    if case == "offset":
        checks.append(("stack 30 mm off, as on the R1: still pushes to a stop (%s)" % res.get("result"), res.get("result") in ("moved", "slip", "finger_lost", "cap")))
    if case == "slide":
        checks.append(("moved stop (%s; plate %.1f mm; H %s)" % (res.get("result"), r["moved"], res.get("H")), res.get("result") == "moved" and 8 < r["moved"] < 25))
    if case == "padslip":
        checks.append(("slip or finger_lost stop (%s, plate %.1f mm)" % (res.get("result"), r["moved"]), res.get("result") in ("slip", "finger_lost") and r["moved"] < 1.0))
    if case == "cap":
        checks.append(("cap stop (%s, plate %.1f mm)" % (res.get("result"), r["moved"]), res.get("result") == "cap" and r["moved"] < 1.0))
    if case == "tip":
        checks.append(("tipped, stopped by the marker (%s)" % {k: res.get(k) for k in ("result", "Fz_at_tip", "force_from_plate_N", "true_over_commanded", "lever_r_mm", "sink_mm")},
                       res.get("result") == "marker" and 0.5 < (res.get("true_over_commanded") or 0) < 1.5))
    if case == "slip":
        t = res.get("tries", [])
        checks.append(("slip levels played: %s" % [(x["press"], x["result"], x.get("H"), x.get("H_hand_1mm")) for x in t], len(t) == 2 and all(x["result"] == "slip" for x in t)))
        by = [e.get("by") for e in res.get("events", []) if e["ev"] == "touch"]
        checks.append(("every landing on the rim, none in the air: %s" % by, by == ["both", "both"]))
    if case == "refuse":
        checks.append(("the operator's no stops it (%s)" % r["why"], not r["ok"] and "confirm" in r["why"]))
    if case == "nofingers":
        checks.append(("no touch data: refused before moving (%s)" % r["why"], not r["ok"] and "touch" in r["why"] and r["seconds"] < 30))
    return checks


def plot(rs, png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    use = [r for r in rs if os.path.exists(os.path.join(r["out"], "nearforce_cmd.csv")) and r["case"] in ("press", "slide", "padslip", "tip", "slip")]
    fig, ax = plt.subplots(3, len(use), figsize=(4.4 * len(use), 8), sharex="col", squeeze=False)
    for i, r in enumerate(use):
        R = list(csv.DictReader(open(os.path.join(r["out"], "nearforce_cmd.csv"))))
        g = lambda k: np.array([float(x[k]) if x.get(k) not in (None, "") else np.nan for x in R])  # noqa: E731
        t = g("t")
        ax[0, i].plot(t, g("H"), label="H commanded", color="#2a78b5")
        ax[0, i].plot(t, g("Fz"), label="Fz commanded", color="#d9722b")
        ax[0, i].plot(t, g("idx_prox") / 1e5, label="index proximity / 1e5", color="#3a9d6a", lw=0.8)
        ax[0, i].plot(t, g("mid_prox") / 1e5, label="middle proximity / 1e5", color="#8d62b0", lw=0.8)
        ax[0, i].set_title(r["case"] + ": " + str(r["res"].get("result"))[:40], fontsize=9)
        ax[1, i].plot(t, 1000 * (g("idx_fk_z") - g("idx_fk_z")[0]), color="#2a78b5")
        ax[2, i].plot(t, 1000 * (g("mk_x") - g("mk_x")[0]), label="marker x", color="#2a78b5")
        ax[2, i].plot(t, 1000 * (g("mk_y") - g("mk_y")[0]), label="marker y", color="#d9722b")
        ax[2, i].plot(t, 1000 * (g("idx_fk_y") - g("idx_fk_y")[-40]), label="index y", color="#3a9d6a", lw=0.8)
        ax[2, i].set_xlabel("s")
        for a in ax[:, i]:
            a.grid(alpha=0.25)
    ax[0, 0].set_ylabel("N")
    ax[1, 0].set_ylabel("index pad height, mm from start")
    ax[2, 0].set_ylabel("mm")
    ax[0, 0].legend(fontsize=7)
    ax[2, 0].legend(fontsize=7)
    fig.suptitle("nearforce's logic on the SIMULATED ARM (MockBackend + spring contacts, not MuJoCo; says nothing about the real plate)", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(png, dpi=95)
    print(png)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="slide")
    ap.add_argument("--all", action="store_true")
    a = ap.parse_args()
    cases = ["shape", "touch", "press", "slide", "offset", "padslip", "cap", "tip", "slip", "refuse", "nofingers", "noprox", "onefinger"] if a.all else [a.case]
    rs, bad = [], 0
    for cs in cases:
        print("==== %s" % cs, flush=True)
        r = run_case(cs, no_touch=cs == "nofingers", answer=0 if cs == "refuse" else 1)
        rs.append(r)
        for what, good in check(r):
            print(("PASS " if good else "FAIL ") + what, flush=True)
            bad += 0 if good else 1
    try:
        plot(rs, os.path.join(os.path.dirname(HERE), "logs", "test_nearforce_mock.png"))
    except Exception as ex:                                     # noqa: BLE001
        print("(no figure: %r)" % ex)
    print("FAILED %d" % bad if bad else "all passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
