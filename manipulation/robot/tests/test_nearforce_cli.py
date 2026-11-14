"""Offline run of the real nearforce command line (run_test.py as a separate process) against the simulated arm.

    python tests/test_nearforce_cli.py
"""
import glob
import json
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
sys.path.insert(0, PICK)
sys.path.insert(0, HERE)

import test_nearforce_mock as T  # noqa: E402
from pk import config  # noqa: E402
from pk.client import operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402

PORT = T.PORT


def call(args, stdin=None, timeout=200):
    p = subprocess.run([sys.executable, os.path.join(PICK, "run_test.py")] + args + ["--port", str(PORT)], input=stdin, capture_output=True, text=True, timeout=timeout, cwd=PICK,
                       env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    return p.returncode, p.stdout + p.stderr


def call_answering(args, answer, timeout=200):
    """As an operator: the answer is typed only after the question has been printed. Input typed earlier is thrown away by the stage."""
    p = subprocess.Popen([sys.executable, "-u", os.path.join(PICK, "run_test.py")] + args + ["--port", str(PORT)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, cwd=PICK, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    lines = []
    t_ans = [None]

    def read():
        for line in p.stdout:
            lines.append(line)
            if line.startswith(">>>"):
                time.sleep(2.0)
                t_ans[0] = time.time()
                p.stdin.write(answer + "\n")
                p.stdin.flush()
    th = threading.Thread(target=read, daemon=True)
    th.start()
    rc = p.wait(timeout)
    th.join(2.0)
    return rc, "".join(lines)


def latest_summary(name):
    ds = sorted(glob.glob(os.path.join(config.LOG_DIR, "tests", "*_" + name)))
    return json.load(open(os.path.join(ds[-1], "summary.json")))


def main():
    plate = T.world("press")
    io = MockBackend(grav_scale=1.2, friction=0.3)
    T.set_touch(io)
    io.contact = plate.contact
    io.contact_pts = [("index", "left_index_touch_link", T.PAD_POINT, "Index"), ("middle", "left_middle_touch_link", T.PAD_POINT, "Middle")]
    srv = Server(io, port=PORT, verbose=False, log_dir=os.path.join(PICK, "logs", "test"))
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
    per = T.PlatePerception(plate, True)
    time.sleep(0.5)
    bad = 0

    def report(what, good, text=""):
        nonlocal bad
        print(("PASS " if good else "FAIL ") + what, flush=True)
        if not good:
            bad += 1
            print(text[-1800:])
    for stage, sets in (() if "--typed" in sys.argv or "--armwide-only" in sys.argv else (("nearforce", []), ("nearforce_shape", []), ("nearforce_tip", []), ("nearforce_slip", []), ("nearforce_press", ["nearforce.n=2"]),
                        ("nearforce", ["nearforce.table_m=0.762"]), ("armid_grav", []), ("armid_pose", []), ("armid_fric", []), ("armid_chirp", []), ("armid_hand", []))):
        rc, out = call([stage, "--dry"] + sum((["--set", s] for s in sets), []))
        report("%s --dry %s" % (stage, sets), rc == 0 and "Traceback" not in out and "TOO LOW" not in out, out)
        if stage in ("nearforce", "armid_grav"):
            print("    " + "\n    ".join(x[:170] for x in out.strip().splitlines()[:6]))
    for stage, sets in (("armwide_joint", []), ("armwide_joint", ["armwide.side=right"]), ("armwide_poses", ["armwide.load_kg=0.5"]),
                        ("armwide_poses", ["armwide.side=right", "armwide.load_kg=0.5"]), ("handid_track", []), ("handid_load", ["handid.side=left"]),
                        ("handid_load", ["handid.side=right"])):
        rc, out = call([stage, "--dry"] + sum((["--set", x] for x in sets), []))
        report("%s --dry %s" % (stage, sets), rc == 0 and "Traceback" not in out, out)
        print("    " + "\n    ".join(x[:200] for x in out.strip().splitlines() if not x.startswith("  ")))
    for sets in ([], ["armwide.side=right"]):
        for stage in ("armwide_poses", "armwide_motion"):
            rc, out = call([stage, "--dry"] + sum((["--set", x] for x in sets), []))
            report("%s --dry %s" % (stage, sets), rc == 0 and "Traceback" not in out, out)
            print("    " + out.strip().splitlines()[0][:170])
    rc, out = call(["armwide_poses", "--set", "armwide.side=right", "--set", "armwide.n_poses=2", "--set", "armwide.hold_s=0.5"])
    report("armwide_poses through run_test.py, right arm, 2 poses: exit %s" % rc, rc == 0 and "Traceback" not in out, out)
    time.sleep(1.0)
    if "--armwide-only" in sys.argv:
        run["stop"] = True
        per.stop = True
        time.sleep(0.1)
        srv.close()
        print("all passed" if not bad else "FAILED %d" % bad)
        return 1 if bad else 0
    bad_set = ["nearforce.shape_hold_s=1"]
    # a typed y: run_test.py's own stdin
    rc, out = call_answering(["nearforce_touch"] + sum((["--set", s] for s in bad_set), []), "y")
    s = latest_summary("nearforce_touch")
    report("nearforce_touch, typed y: %s; %s" % (s["ended"], s["results"].get("nearforce_touch", {}).get("result")), rc == 0 and s["ended"] == "completed" and "answer True" in out, out)
    time.sleep(1.0)
    rc, out = call_answering(["nearforce_touch"] + sum((["--set", s] for s in bad_set), []), "n")
    s = latest_summary("nearforce_touch")
    report("nearforce_touch, typed n: %s" % s["ended"], rc != 0 and "did not confirm" in s["ended"] and "no answer" not in out, out)
    t0 = time.time()
    while srv.mode != "HOME" and time.time() - t0 < 60:
        time.sleep(0.1)
    report("arms home at the end (%s)" % srv.mode, srv.mode == "HOME")
    report("the plate was only touched (%.1f mm)" % (1000 * sum((plate.c - plate.c0) ** 2) ** 0.5), 1000 * sum((plate.c - plate.c0) ** 2) ** 0.5 < 2.0)
    run["stop"] = True
    per.stop = True
    time.sleep(0.1)
    srv.close()
    print("FAILED %d" % bad if bad else "all passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
