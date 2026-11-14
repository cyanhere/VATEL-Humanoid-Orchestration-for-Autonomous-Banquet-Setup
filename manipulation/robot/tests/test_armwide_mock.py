"""Offline run of the armwide stage against the simulated arm, and check that calibration/armwide_fit.py recovers the
simulated arm's gravity and friction.

    python tests/test_armwide_mock.py
"""
import csv
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
sys.path.insert(0, PICK)
from pk import config  # noqa: E402
from pk.client import Client, operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server, load_home  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.armwide import ArmWide  # noqa: E402

PORT = 9726
SHORT = {"armwide.hold_s": 1.0, "armwide.motion_s": 12.0, "armwide.n_motions": 2, "armwide.move_v_rad_s": 0.4, "armwide.approach_v_rad_s": 0.2}


def run(mode, side, n_poses=0, extra=None):
    io = MockBackend(grav_scale=1.2, friction=0.3)
    # HOME = the pre-grip pose, where these stages were planned (the server's default is the neutral
    # pose; run_test.py moves there first, these tests call the stages directly)
    srv = Server(io, home=load_home("pregrip"), port=PORT, verbose=False, log_dir=os.path.join(PICK, "logs", "test"))
    st = dict(stop=False)

    def loop():
        nxt = time.time()
        while not st["stop"]:
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
    c = Client("test armwide mock", port=PORT)
    out = os.path.join(PICK, "logs", "test", "armwide_%s_%s_%s" % (mode, side, time.strftime("%H%M%S")))
    ov = dict(SHORT, **{"armwide.side": side, "armwide.n_poses": n_poses})
    ov.update(extra or {})
    with c.busy(20):
        ctx = Ctx(c, out, overrides=ov)
    ok, res, why = True, {}, ""
    try:
        res = ArmWide(ctx, mode).run()
    except Abort as ex:
        ok, why = False, str(ex)
    c.bye()
    t1 = time.time()
    while srv.mode != "HOME" and time.time() - t1 < 300:
        time.sleep(0.1)
    home = srv.mode == "HOME"
    st["stop"] = True
    time.sleep(0.1)
    srv.close()
    return dict(ok=ok, why=why, res=res, home=home, out=out)


def main():
    bad = 0

    def report(what, good):
        nonlocal bad
        print("%s %s" % ("PASS" if good else "FAIL", what), flush=True)
        bad += 0 if good else 1
    plan = json.load(open(os.path.join(PICK, "params", "armwide_plan.json")))
    r = run("poses", "left", extra={"armwide.home_tol_rad": 1e-6})
    report("refuses when the arms are not where the plan starts: %s" % r["why"], not r["ok"] and "pre-grip" in r["why"] and r["home"])
    for side in ("left", "right"):
        sl = config.LEFT if side == "left" else config.RIGHT
        rp = run("poses", side)
        report("%s poses: %s, %s poses, %.0f s; arms home %s" % (side, rp["why"] or rp["res"].get("result"), rp["res"].get("poses"), rp["res"].get("seconds", 0), rp["home"]), rp["ok"] and rp["home"])
        rm = run("motion", side)
        report("%s motion: %s, %s motions, %.0f s; arms home %s" % (side, rm["why"] or rm["res"].get("result"), rm["res"].get("motions"), rm["res"].get("seconds", 0), rm["home"]), rm["ok"] and rm["home"])
        if not (rp["ok"] and rm["ok"]):
            continue
        R = list(csv.DictReader(open(os.path.join(rp["out"], "armwide_cmd.csv"))))
        Q = np.array([[float(x["q_cmd%d" % i]) for i in range(5)] for x in R])
        M = np.array([[float(x["q_meas%d" % i]) for i in range(5)] for x in R])
        reach = max(float(np.min(np.max(np.abs(M - np.array(q)), axis=1))) for q in plan[side]["poses"])
        report("%s: every pose reached, worst %.1f mrad; joint range covered %s rad" % (side, 1000 * reach, np.round(M.max(0) - M.min(0), 2)), reach < 0.06)
        step = float(np.max(np.abs(np.diff(Q, axis=0))))
        report("%s: largest command step %.1f mrad" % (side, 1000 * step), step < 0.03)
        R = list(csv.DictReader(open(os.path.join(rm["out"], "armwide_cmd.csv"))))
        V = np.array([[float(x["dq%d" % i]) for i in range(5)] for x in R if x["phase"][0] == "m" and x["phase"][1:].split("_")[0].isdigit()])
        report("%s: motion moved, peak joint speed %.2f rad/s" % (side, float(np.max(np.abs(V)))), 0.15 < float(np.max(np.abs(V))) < 0.85)
        js = os.path.join(rp["out"], "sim_arm_wide.json")
        p = subprocess.run([sys.executable, os.path.join(PICK, "calibration", "armwide_fit.py"), rp["out"], rm["out"], "--out", js], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
        print("\n".join("    " + x[:150] for x in p.stdout.strip().splitlines()))
        if p.returncode:
            report("%s: fit ran" % side, False)
            continue
        F = json.load(open(js))["parts"]
        s, co = np.array(F["grav"]["scale"]), np.array(F["fric"]["coulomb_nm"])
        big = np.max(np.abs(np.array(F["grav"]["server_rms_nm"]))) >= 0
        report("%s: gravity scale recovered on the big joints: %s (true 1.2)" % (side, np.round(s[:4], 2)), big and np.all(np.abs(s[:4] - 1.2) < 0.15))
        report("%s: Coulomb friction recovered: %s (true 0.3)" % (side, np.round(co, 2)), np.all(np.abs(co - 0.3) < 0.15))
    print("all passed" if not bad else "FAILED %d" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
