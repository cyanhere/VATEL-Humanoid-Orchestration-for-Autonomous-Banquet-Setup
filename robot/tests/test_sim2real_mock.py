"""Offline run of armwide_joint, armwide_poses (load), handid_track and handid_load against the simulated arm, and check
that the fits recover the values the simulated arm was given.

    python tests/test_sim2real_mock.py [--keep]
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
from pk.client import Client, operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server, load_home  # noqa: E402
from pk.stage_base import Abort, Ctx  # noqa: E402
from pk.stages.armwide import ArmWide  # noqa: E402
from pk.stages.handid import HandId  # noqa: E402

PORT = 9727
SHORT_ARM = {"armwide.hold_s": 1.0, "armwide.load_hold_s": 1.5, "armwide.move_v_rad_s": 0.4, "armwide.approach_v_rad_s": 0.2,
             "armwide.sweeps": [[0.02, 0.05], [0.1, 0.3], [0.3, 0.3]], "armwide.rev_amps": [0.01, 0.03], "armwide.rev_cycles": 2, "armwide.step_hold_s": 1.0}
SHORT_HAND = {"handid.step_sizes": [0.1, 0.6], "handid.step_hold_s": 1.0, "handid.ramp_speeds": [0.3, 1.0], "handid.rev_amps": [0.02],
              "handid.rev_cycles": 2, "handid.levels": [0.0, 0.5, 1.0], "handid.level_hold_s": 1.0, "handid.multisine_s": 10.0}


def run(stage_cls, mode, ov, payload=None):
    io = MockBackend(grav_scale=1.2, friction=0.3)
    io.payload = payload
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
    c = Client("test sim2real mock", port=PORT)
    out = os.path.join(PICK, "logs", "test", "%s_%s_%s" % (stage_cls.name, mode, time.strftime("%H%M%S")))
    with c.busy(20):
        ctx = Ctx(c, out, overrides=ov)
    ok, res, why = True, {}, ""
    t0 = time.time()
    try:
        res = stage_cls(ctx, mode).run()
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
    return dict(ok=ok, why=why, res=res, home=home, out=out, s=time.time() - t0)


def rows(path):
    return list(csv.DictReader(open(path)))


def fit(script, dirs, out):
    p = subprocess.run([sys.executable, os.path.join(PICK, "calibration", script)] + dirs + ["--out", out, "--label", "the simulated arm (MockBackend), not the R1 or MuJoCo"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                       env=dict(os.environ, PYTHONIOENCODING="utf-8", MPLBACKEND="Agg"))
    print("\n".join("    " + x[:160] for x in p.stdout.strip().splitlines()))
    return p.returncode


def main():
    bad = 0

    def report(what, good):
        nonlocal bad
        print("%s %s" % ("PASS" if good else "FAIL", what), flush=True)
        bad += 0 if good else 1
    plan = json.load(open(os.path.join(PICK, "params", "armwide_plan.json")))
    only = [a for a in sys.argv[1:] if not a.startswith("--")]

    if not only or "arm" in only:
        r = run(ArmWide, "joint", dict(SHORT_ARM, **{"armwide.side": "left", "armwide.joints": [0, 3]}))
        report("armwide_joint left [0, 3]: %s, %.0f s, arms home %s" % (r["why"] or r["res"].get("result"), r["s"], r["home"]), r["ok"] and r["home"])
        if r["ok"]:
            R = rows(os.path.join(r["out"], "armwide_cmd.csv"))
            for j in (0, 3):
                jp = plan["left"]["joints"][j]
                M = np.array([[float(x["q_meas%d" % i]) for i in range(5)] for x in R if x["phase"].startswith("j%d_" % j)])
                others = [i for i in range(5) if i != j]
                span = M[:, j].max() - M[:, j].min()
                drift = float(np.max(np.abs(M[:, others] - np.array(jp["centre"])[others])))
                report("joint %d: swept %.2f rad (plan +-%.2f, capped by the test's sweeps), the other joints within %.0f mrad of the centre" % (j, span, jp["range"], 1000 * drift),
                       span > 0.5 and drift < 0.05)
            phases = {x["phase"] for x in R}
            report("phases present: sweeps, back-and-forth, steps", all(any(p.startswith(pre) for p in phases) for pre in ("j0_v0.02_up", "j0_v0.3_dn", "j3_r0.03", "j3_s-0.1")))
        joint_dir = r["out"]
        rp = run(ArmWide, "poses", dict(SHORT_ARM, **{"armwide.side": "left", "armwide.n_poses": 8}))
        report("armwide_poses left, 8 poses: %s, arms home %s" % (rp["why"] or rp["res"].get("result"), rp["home"]), rp["ok"] and rp["home"])
        rl = run(ArmWide, "poses", dict(SHORT_ARM, **{"armwide.side": "left", "armwide.load_kg": 0.5}), payload=("left", 0.5, "left_wrist_roll_link", [0.0, 0.0, 0.0]))
        R = rows(os.path.join(rl["out"], "armwide_cmd.csv")) if rl["ok"] else []
        n_g = len({x["phase"].split("_")[0] for x in R if x["phase"][0] == "g" and x["phase"][1].isdigit()})
        report("armwide_poses left with 0.5 kg: %s, %d load poses (plan %s), arms home %s" % (rl["why"] or rl["res"].get("result"), n_g, plan["left"]["load_poses"], rl["home"]),
               rl["ok"] and rl["home"] and n_g == len(plan["left"]["load_poses"]))
        rm = run(ArmWide, "motion", dict(SHORT_ARM, **{"armwide.side": "left", "armwide.motion_s": 12.0, "armwide.n_motions": 2}))
        report("armwide_motion left, 2 short motions: %s" % (rm["why"] or rm["res"].get("result")), rm["ok"])
        js = os.path.join(PICK, "logs", "test", "sim_arm_model_left.json")
        if all(x["ok"] for x in (r, rp, rl, rm)):
            code = fit("arm_model_fit.py", [joint_dir, rp["out"], rl["out"], rm["out"], "--check", rm["out"]], js)
            report("arm_model_fit ran", code == 0)
            if code == 0:
                F = json.load(open(js))
                fr = F["friction"]
                co = np.array([fr["joints"][j]["coulomb_nm"] for j in (0, 3)])
                vi = np.array([fr["joints"][j]["viscous_nm_s"] for j in (0, 3)])
                report("Coulomb on the swept joints %s (true 0.3)" % np.round(co, 2), np.all(np.abs(co - 0.3) < 0.12))
                report("viscous on the swept joints %s (true 1.0)" % np.round(vi, 2), np.all(np.abs(vi - 1.0) < 0.35))
                g = F["gravity"]
                report("gravity: chose %s; held-out rms %s N m vs URDF %s" % (g["chosen"], np.round(g["loo_rms_nm"][g["chosen"]], 2), np.round(g["loo_rms_nm"]["urdf"], 2)),
                       np.max(g["loo_rms_nm"][g["chosen"]][:4]) < 0.5 * np.max(g["loo_rms_nm"]["urdf"][:4]))
                report("torque reading scale from the hung 0.5 kg: %.2f (true 1.0)" % g["torque_scale"], abs(g["torque_scale"] - 1.0) < 0.2)

    if not only or "hand" in only:
        rt = run(HandId, "track", dict(SHORT_HAND, **{"handid.channels": [1, 2]}))
        report("handid_track both hands, channels thumb rot and index: %s, %.0f s, arms home %s" % (rt["why"] or rt["res"].get("result"), rt["s"], rt["home"]), rt["ok"] and rt["home"])
        if rt["ok"]:
            R = rows(os.path.join(rt["out"], "handid_cmd.csv"))
            H = np.array([[float(x["hcmd%d" % i]) for i in range(12)] for x in R])
            Q = np.array([[float(x["q_meas%d" % i]) for i in range(10)] for x in R])
            report("both hands got the same index commands, max %.2f" % H[:, 2].max(), np.allclose(H[:, 2], H[:, 8]) and H[:, 2].max() > 0.85)
            report("arms still: %.1f mrad" % (1000 * float(np.max(Q.max(0) - Q.min(0)))), float(np.max(Q.max(0) - Q.min(0))) < 0.02)
            report("ends at the shape it started from (max %.3f off)" % float(np.max(np.abs(H[-1] - H[0]))), float(np.max(np.abs(H[-1] - H[0]))) < 0.03)
            hs = os.path.join(PICK, "logs", "test", "sim_hand_model.json")
            code = fit("handid_fit.py", [rt["out"]], hs)
            report("handid_fit ran", code == 0)
            if code == 0:
                F = json.load(open(hs))
                ch = F["hands"]["left"]["index"]
                r_ = ch["rate_per_s"] or 0.0
                report("index: model %s, speed limit %.2f /s (true 1.2; the server's own limit 1.5)" % (ch["chosen"], r_), ch["chosen"].startswith("rate") and abs(r_ - 1.2) < 0.2)
        rh = run(HandId, "load", dict(SHORT_HAND, **{"handid.side": "left", "handid.load_auto": [["i0", 3], ["i100", 2], ["x7", 1], ["curl 0.6", 1], ["i100", 2], ["q", 2]]}))
        report("handid_load left: %s, %s loads, arms home %s" % (rh["why"] or rh["res"].get("result"), rh["res"].get("loads"), rh["home"]), rh["ok"] and rh["home"] and rh["res"].get("loads") == 3)
        if rh["ok"]:
            R = rows(os.path.join(rh["out"], "handid_cmd.csv"))
            Q = np.array([[float(x["q_meas%d" % i]) for i in range(5)] for x in R if x["phase"].startswith("load_")])
            pu = np.array(plan["left"]["palm_up"]["q"])
            report("held at the palm-up pose: %.0f mrad" % (1000 * float(np.max(np.abs(np.median(Q, 0) - pu)))), float(np.max(np.abs(np.median(Q, 0) - pu))) < 0.06)
            labs = sorted({x["phase"] for x in R if x["phase"].startswith("load_")})
            report("labels %s" % labs, "load_i100_c0.6" in labs and "load_i100_c0.3" in labs and not any("x7" in x for x in labs))
    print("all passed" if not bad else "FAILED %d" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    code = main()
    sys.stdout.flush()
    os._exit(code)
