"""run_test.py end to end against the server with the simulated arm: log, summary, and return to the pre-grip pose
however a test ends.

    python3 tests/test_run_test_offline.py
"""
import glob
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import run_test  # noqa: E402
from pk import config  # noqa: E402
from pk.client import operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server, arm_off  # noqa: E402
from pk.stage_base import Abort, _short  # noqa: E402

PORT = 9741
OK = []


def check(cond, what):
    OK.append((bool(cond), what))
    print(("PASS " if cond else "FAIL ") + what, flush=True)


class Stand(object):
    """A stage stand-in: moves the arms a little (something to retrace), then ends as told."""

    def __init__(self, ctx, how):
        self.ctx, self.how = ctx, how

    def run(self):
        q = np.array(self.ctx.c.target()[0])
        q[0] += 0.08
        q[8] -= 0.06
        why = self.ctx.c.move_to(q, v=0.3, tag="stand-in")
        if why:
            raise Abort("stand-in move: %s" % why)
        self.ctx.log("stand-in", moved_to=q, names=["a", "b"], nested={"x": [1, "y"]})
        if self.how == "abort":
            raise Abort("stand-in abort")
        if self.how == "ctrlc":
            raise KeyboardInterrupt()
        if self.how == "bug":
            raise ValueError("stand-in bug")
        return dict(plate_mm=np.float64(42.3), delta_mm=np.array([5.9, 4.2, -21.2]), source="camera", n=np.int64(3))


def newest_summary(before):
    dirs = [d for d in glob.glob(os.path.join(config.LOG_DIR, "tests", "*")) if d not in before]
    for d in sorted(dirs, key=os.path.getmtime, reverse=True):
        f = os.path.join(d, "summary.json")
        if os.path.exists(f):
            import json
            return json.load(open(f))
    return None


def main():
    check(_short(["push"]) == "[push]" and _short([1.0, 2.5]) == "[1 2.5]" and _short({"a": 1}) == "{'a': 1}",
          "log values: text lists print as text (%s), numbers as numbers (%s)" % (_short(["push"]), _short([1.0, 2.5])))
    io = MockBackend()
    srv = Server(io, port=PORT, verbose=False, log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
    stop = [False]

    def loop():
        nxt = time.time()
        while not stop[0]:
            srv.step()
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))
    threading.Thread(target=loop, daemon=True).start()
    time.sleep(0.3)
    operator("take", port=PORT)
    t0 = time.time()
    while srv.mode != "HELD" and time.time() - t0 < 6:
        time.sleep(0.05)
    operator("home", port=PORT)
    while srv.mode != "HOME" and time.time() - t0 < 40:
        time.sleep(0.05)
    check(srv.mode == "HOME", "simulated server at the pre-grip pose")
    for how, rc_want, ended in (("done", 0, "completed"), ("abort", 2, "aborted: stand-in abort"), ("ctrlc", 2, "Ctrl-C"),
                                ("bug", 2, "error: ValueError('stand-in bug')")):
        run_test.make = lambda ctx, name, how=how: Stand(ctx, how)
        before = set(glob.glob(os.path.join(config.LOG_DIR, "tests", "*")))
        sys.argv = ["run_test.py", "push", "--port", str(PORT), "--set", "push.press_mm=7"]
        rc = run_test.main()
        summ = newest_summary(before)
        t1 = time.time()
        while srv.mode != "HOME" and time.time() - t1 < 20:
            time.sleep(0.05)
        check(rc == rc_want and summ is not None and summ["ended"] == ended,
              "%-5s: exit %s, summary '%s'" % (how, rc, summ and summ["ended"]))
        check(srv.mode == "HOME" and srv.owner is None and arm_off(srv.des, srv.home_q) < 0.004,
              "%-5s: the arms went back to the pre-grip pose (mode %s)" % (how, srv.mode))
        if how == "done":
            r = summ["results"]["push"]
            check(abs(r["plate_mm"] - 42.3) < 1e-6 and r["delta_mm"] == [5.9, 4.2, -21.2] and r["n"] == 3,
                  "the summary keeps numpy results as numbers: %s" % r)
        time.sleep(1.1)                                      # one test folder per second
    sys.argv = ["run_test.py", "push", "--dry", "--port", str(PORT)]
    run_test.make = None
    try:
        rc = run_test.main()
        check(rc == 0, "push --dry runs (no perception: the fixed position)")
    except Exception as ex:                                  # noqa: BLE001
        check(False, "push --dry: %r" % ex)
    stop[0] = True
    time.sleep(0.1)
    srv.close()
    n_bad = sum(1 for ok, _ in OK if not ok)
    print("\n%d checks, %d failed" % (len(OK), n_bad))
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
