"""Offline run of the lone grip against the simulated arm and a fake camera (logic only, no contact).

    python3 tests/test_lone_grip_mock.py                       # lone_grip, grip v2_thumb
    python3 tests/test_lone_grip_mock.py --mode air
    python3 tests/test_lone_grip_mock.py --mode shape --set lone_grip.shape_hold_s=1
    python3 tests/test_lone_grip_mock.py --set lone_grip.grip=v2_slot2
    python3 tests/test_lone_grip_mock.py --plate-xy 0.40 0.0 # 70 mm off: must refuse
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

from mock_world import FakePerception, MockStack  # noqa: E402
from pk import config  # noqa: E402
from pk.client import Client, operator  # noqa: E402
from pk.plate import TASK  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.lone_grip import LoneGrip  # noqa: E402

PORT = 9725


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", default="full", choices=["full", "air", "shape"])
    ap.add_argument("--table", type=float, default=0.755)
    ap.add_argument("--plate-xy", type=float, nargs=2, default=(0.33, 0.0))
    ap.add_argument("--grav-scale", type=float, default=1.2)
    ap.add_argument("--friction", type=float, default=0.4)
    ap.add_argument("--set", action="append", default=[], help="lone_grip.key=value (JSON)")
    ap.add_argument("--expect-refuse", action="store_true", help="the stage must refuse before moving")
    ap.add_argument("--expect-abort", action="store_true", help="the stage must abort mid-way, let go and come home")
    a = ap.parse_args()
    rim_z = a.table - config.PELVIS_SIM + TASK.H
    plate = MockStack(center=a.plate_xy, rim_z=rim_z)
    plate.n_plates = 1
    io = MockBackend(grav_scale=a.grav_scale, friction=a.friction)
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
    per = FakePerception(plate, port=PORT)
    time.sleep(0.5)
    c = Client("test lone_grip mock", port=PORT)
    out = os.path.join(os.path.dirname(HERE), "logs", "test", "lone_grip_%s_%s" % (a.mode, time.strftime("%H%M%S")))
    overrides = {"lone_grip.table_m": a.table}
    for kv in a.set:
        k, v = kv.split("=", 1)
        try:
            overrides[k.strip()] = json.loads(v)
        except ValueError:
            overrides[k.strip()] = v                       # a bare word (as run_test.py --set)
    with c.busy(20):
        ctx = Ctx(c, out, overrides=overrides)
    home0 = np.array(c.state()["des"], float)
    ok, res, why = True, {}, ""
    try:
        res = LoneGrip(ctx, a.mode).run()
        print("== lone_grip %s: %s" % (a.mode, res.get("result")))
    except Abort as ex:
        ok, why = False, str(ex)
        res = ctx.results.get("lone_grip", {})
        print("ABORT: %s" % ex)
    c.bye()
    t0 = time.time()
    while srv.mode != "HOME" and time.time() - t0 < 120:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    run["stop"] = True
    per.stop = True
    time.sleep(0.1)
    srv.close()
    checks = []
    if a.expect_refuse:
        checks.append(("refused before moving (%s)" % why, not ok and not res.get("log")))
        ok = True
    elif a.expect_abort:
        checks.append(("aborted mid-way (%s)" % why, not ok and bool(res.get("log"))))
        ok = True
    else:
        checks.append(("completed: %s" % (res.get("result") or why), ok))
        if a.mode in ("full", "air") and ok:
            cor = res["correction"]
            for s in ("left", "right"):
                e0 = cor["steps"][0]["palm_err_before_norm_mm"][s]
                e1 = cor["palm_err_after_norm_mm"][s]
                checks.append(("%s palm error at 'down': %.1f mm before the correction, %.1f mm after" % (s, e0, e1), e1 <= 3.0))
            print("info: aim-point drift over the still hold %s mm (the simulated arm still creeping after the lift; no plate here)" %
                  res["hold"]["palm_drift_in_still_hold_mm"])
    checks.append(("arms back home after the test", home))
    for what, good in checks:
        print(("PASS " if good else "FAIL ") + what)
    if res.get("log"):
        try:
            plot(res["log"], os.path.join(os.path.dirname(HERE), "logs", "test_lone_grip_mock_%s.png" % a.mode), a, res)
        except Exception as ex:                                     # noqa: BLE001
            print("(no figure: %r)" % ex)
    return 0 if ok and all(g for _, g in checks) else 1


def plot(path, png, a, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return
    g = lambda k: np.array([float(r[k]) if r.get(k) not in (None, "") else np.nan for r in rows])   # noqa: E731
    t = g("t")
    ph = [r["phase"] for r in rows]
    fig, ax = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    for s, col in (("left", "C0"), ("right", "C3")):
        for i, ax_name in enumerate("yz"):
            ax[i].plot(t, 1000 * g("%s_palm_meas_%s" % (s, ax_name)), color=col, label="%s aim point %s, measured (FK)" % (s, ax_name))
            ax[i].plot(t, 1000 * g("%s_palm_plan_%s" % (s, ax_name)), "--", color=col, lw=0.8, label="%s, plan (FK, no correction)" % s)
            ax[i].plot(t, 1000 * g("%s_palm_cmd_%s" % (s, ax_name)), ":", color=col, lw=0.8, label="%s, commanded = plan + correction (FK)" % s)
        err = np.sqrt(sum((g("%s_palm_meas_%s" % (s, x)) - g("%s_palm_plan_%s" % (s, x))) ** 2 for x in "xyz"))
        ax[2].plot(t, 1000 * err, color=col, label="%s |measured - plan| aim point (the error the correction removes)" % s)
        ax[3].plot(t, 1000 * np.max(np.abs(np.array([g("%s_corr%d" % (s, i)) for i in range(5)])), axis=0), color=col,
                   label="%s largest joint correction" % s)
    ax[0].set_ylabel("y mm (pelvis)")
    ax[1].set_ylabel("z mm (pelvis)")
    ax[1].axhline(1000 * res["table_z"], color="k", ls=":", lw=0.8, label="table top")
    ax[1].axhline(1000 * (res["table_z"] + TASK.H), color="gray", ls=":", lw=0.8, label="rim top")
    ax[2].set_ylabel("mm")
    ax[3].set_ylabel("mrad")
    ax[3].set_xlabel("s")
    for x in ax:
        x.legend(fontsize=7, loc="upper right")
        x.grid(alpha=0.3)
        last = None
        for ti, p in zip(t, ph):
            if p != last:
                x.axvline(ti, color="gray", lw=0.5, alpha=0.5)
                if x is ax[0]:
                    x.text(ti, x.get_ylim()[1], p, fontsize=6, rotation=90, va="top")
                last = p
    fig.suptitle("lone grip on the SIMULATED ARM (MockBackend: gravity x%.1f the server's model, friction %.1f; no plate contact; not MuJoCo)\n"
                 "mode %s, grip %s, table %.3f" % (a.grav_scale, a.friction, a.mode, res.get("grip"), a.table))
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    print("figure: %s" % png)


if __name__ == "__main__":
    sys.exit(main())
