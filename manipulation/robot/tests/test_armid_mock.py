"""Offline run of the armid stage against the simulated arm (MockBackend, not MuJoCo): plan, moves, log, stops, return home.

    python3 tests/test_armid_mock.py                 # armid
    python3 tests/test_armid_mock.py --mode load     # armid_load, labels from a timer
    python3 tests/test_armid_mock.py --floor         # the floor limit raised above the plan: must refuse before moving
"""
import argparse
import csv
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk.client import Client, operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.armid import ArmId  # noqa: E402

PORT = 9724


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="full", choices=["full", "load", "pose", "grav", "fric", "chirp", "hand"])
    ap.add_argument("--floor", action="store_true")
    ap.add_argument("--short", action="store_true", help="fewer waypoints and shorter holds")
    a = ap.parse_args()
    io = MockBackend(grav_scale=1.2, friction=0.3)
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
    c = Client("test armid mock", port=PORT)
    out = os.path.join(os.path.dirname(HERE), "logs", "test", "armid_%s_%s" % (a.mode, time.strftime("%H%M%S")))
    ov = {}
    if a.short:
        ov.update({"armid.hold_s": 0.2, "armid.sweep_v_rad_s": [0.2], "armid.wp_rad": [0.05, 0.05, 0.05, 0.05, 0.05], "armid.grav_n": 4, "armid.grav_hold_s": 0.5, "armid.fric_v_rad_s": [0.1, 0.4],
                   "armid.fric_dither_mrad": [20.0], "armid.chirp_hz": [1.0], "armid.hand_ramp_s": 1.0, "armid.hand_step_s": 0.3, "armid.q_correct_mrad": [0, 8, 0, -40, 0]})
    if a.mode == "load":
        ov["armid.load_auto"] = [["0", 1.0], ["5.1", 2.0], ["q", 2.0]]
    if a.floor:
        ov["armid.z_floor_mm"] = 100.0
    with c.busy(20):
        ctx = Ctx(c, out, overrides=ov)
    ok, res, why = True, {}, ""
    t0 = time.time()
    try:
        res = ArmId(ctx, a.mode).run()
    except Abort as ex:
        ok, why = False, str(ex)
    moved_s = time.time() - t0
    c.bye()
    t1 = time.time()
    while srv.mode != "HOME" and time.time() - t1 < 120:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    run["stop"] = True
    time.sleep(0.1)
    srv.close()
    checks = [("arms back home", home)]
    if a.floor:
        checks.append(("refused before moving (%s)" % why, not ok and "floor" in why and moved_s < 5))
    elif not ok:
        checks.append(("ran: %s" % why, False))
    else:
        checks.append(("ran: %s" % res.get("result"), True))
        rows = list(csv.DictReader(open(res["log"])))
        ph = [r["phase"] for r in rows]
        checks.append(("%d rows in %.0f s" % (len(rows), moved_s), len(rows) > 50))
        qc = np.array([[float(r["q_cmd%d" % i]) for i in range(5)] for r in rows])
        qm = np.array([[float(r["q_meas%d" % i]) for i in range(5)] for r in rows])
        checks.append(("dq and tau logged", rows[0].get("dq0") not in (None, "") and rows[0].get("tau0") not in (None, "")))
        checks.append(("tracking rms %.1f mrad, peak %.0f" % (1000 * np.sqrt(((qc - qm) ** 2).mean()), 1000 * np.abs(qc - qm).max()), np.abs(qc - qm).max() < 0.15))
        if a.mode == "load":
            checks.append(("labels logged %s" % sorted(set(ph)), "load_0" in ph and "load_5.1" in ph))
        elif a.mode == "grav":
            n = sorted({p.split("_")[0] for p in ph if p.startswith("g") and p[1].isdigit()})
            checks.append(("poses held from both sides: %s" % n, len(n) >= 3 and all(p + "_a" in ph and p + "_b" in ph for p in n)))
            te = np.array([[float(r["q_tgt%d" % i]) for i in range(5)] for r in rows]) - qc
            checks.append(("correction applied: command - asked %s mrad" % np.round(1000 * np.median(te, axis=0), 1), not a.short or abs(np.median(te[:, 3]) - 0.04) < 1e-3))
        elif a.mode == "fric":
            checks.append(("each joint at each speed and sine", all("f%d_v0.1_up" % j in ph and "d%d_a20" % j in ph for j in range(5))))
        elif a.mode == "chirp":
            checks.append(("each joint's sine", all("c%d_h1" % j in ph for j in range(5))))
        elif a.mode == "hand":
            hc = np.array([[float(r["hand_cmd%d" % i]) for i in range(6)] for r in rows])
            checks.append(("each finger moved: %s" % np.round(np.ptp(hc, axis=0), 2), np.all(np.ptp(hc, axis=0) > 0.5)))
        else:
            for j in range(5):
                k = [i for i, p in enumerate(ph) if p.startswith("j%d_" % j)]
                move = np.ptp(qc[k, j])
                others = max(np.ptp(qc[k, o]) for o in range(5) if o != j)
                checks.append(("joint %d moved %.3f rad alone (others %.4f)" % (j, move, others), move > 0.07 and others < 1e-3))
            checks.append(("both speeds ran", "line_slow" in ph and "line_fast" in ph))
    for what, good in checks:
        print(("PASS " if good else "FAIL ") + what)
    if res.get("plan"):
        print("plan:", res["plan"])
    return 0 if all(g for _, g in checks) else 1


if __name__ == "__main__":
    sys.exit(main())
