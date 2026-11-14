"""server.py as a terminal program with the simulated arm: typed commands, Ctrl-C, kill, stop.sh.

    python3 tests/test_server_process.py              # Linux: everything but stop.sh; Windows: typed commands only
    python3 tests/test_server_process.py --stop-sh    # also stop.sh -- NEVER on the robot while anything runs there:
"""
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PICK = os.path.dirname(HERE)
PORT = 9731
POSIX = os.name == "posix"
OK = []


def check(cond, what):
    OK.append((bool(cond), what))
    print(("PASS " if cond else "FAIL ") + what, flush=True)


class Proc(object):
    """python3 server.py --mock on PORT, its output collected line by line."""

    def __init__(self):
        self.p = subprocess.Popen([sys.executable, "-u", "server.py", "--mock", "--port", str(PORT)], cwd=PICK,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                  universal_newlines=True, bufsize=1)
        self.lines = []
        self.seen = 0
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for ln in self.p.stdout:
            self.lines.append(ln.rstrip())

    def type(self, txt):
        self.p.stdin.write(txt + "\n")
        self.p.stdin.flush()

    def expect(self, text, timeout):
        """True when a line containing `text` appears after the last one found."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            for i in range(self.seen, len(self.lines)):
                if text in self.lines[i]:
                    self.seen = i + 1
                    return True
            time.sleep(0.05)
        return False

    def exit_code(self, timeout):
        try:
            return self.p.wait(timeout)
        except subprocess.TimeoutExpired:
            return None

    def end(self):
        if self.p.poll() is None:
            self.p.kill()
            self.p.wait()

    def tail(self, n=8):
        return "\n      ".join(self.lines[-n:])


def typed():
    P = Proc()
    check(P.expect("type `take`", 15), "server.py --mock starts")
    P.type("take")
    check(P.expect("arms taken and held", 8), "take -> held")
    P.type("home")
    check(P.expect("moving to HOME", 3), "home -> moving")
    time.sleep(1.0)
    P.type("release")
    check(P.expect("RELEASING", 3), "`release` typed DURING the home move releases\n      " + P.tail())
    rc = P.exit_code(8)
    check(rc == 0 and P.expect("server stopped", 1), "... and the server stops by itself (exit %s)" % rc)
    P.end()

    P = Proc()
    P.expect("type `take`", 15)
    P.type("take")
    P.expect("arms taken and held", 8)
    t0 = time.time()
    P.type("home")
    check(P.expect("at HOME", 25), "home ENDS (%.1f s; the simulated waist is 0.010 rad off the pose file)" % (
        time.time() - t0))
    P.type("f")
    check(P.expect("FROZEN", 2), "f -> frozen")
    P.type("f")
    check(P.expect("already frozen", 2), "f again answers")
    if POSIX:
        P.p.send_signal(signal.SIGINT)
        check(P.expect("Ctrl-C again within 3 s = RELEASE", 2), "Ctrl-C while frozen answers (no release on one press)")
        time.sleep(3.5)
        P.p.send_signal(signal.SIGINT)
        check(P.expect("Ctrl-C again within 3 s = RELEASE", 2) and P.p.poll() is None,
              "Ctrl-C 3.5 s later: answers again, still frozen (2026-10-20: nothing at all)")
        time.sleep(0.5)
        P.p.send_signal(signal.SIGINT)
        check(P.expect("RELEASING", 2), "a 2nd Ctrl-C within 3 s releases")
    else:
        P.type("release")
        check(P.expect("RELEASING", 2), "release from frozen")
    rc = P.exit_code(8)
    check(rc == 0, "... and the server stops (exit %s)" % rc)
    P.end()


def signals():
    P = Proc()
    P.expect("type `take`", 15)
    P.p.send_signal(signal.SIGINT)
    rc = P.exit_code(3)
    check(rc == 0, "Ctrl-C with nothing taken: stops at once (exit %s)" % rc)
    P.end()
    P = Proc()
    P.expect("type `take`", 15)
    P.p.send_signal(signal.SIGTERM)
    rc = P.exit_code(3)
    check(rc == 0, "kill (SIGTERM) with nothing taken: stops at once (exit %s)" % rc)
    P.end()
    P = Proc()
    P.expect("type `take`", 15)
    P.type("take")
    P.expect("arms taken and held", 8)
    P.p.send_signal(signal.SIGTERM)
    check(P.expect("RELEASING", 2), "kill (SIGTERM) with the arms held: releases")
    rc = P.exit_code(8)
    check(rc == 0, "... then stops (exit %s)" % rc)
    P.end()
    P = Proc()
    P.expect("type `take`", 15)
    P.type("take")
    P.expect("arms taken and held", 8)
    P.p.send_signal(signal.SIGINT)
    check(P.expect("FROZEN", 2), "Ctrl-C with the arms held: frozen")
    time.sleep(0.3)
    P.type("take")
    check(P.expect("already taken", 2), "take while frozen and taken answers")
    P.type("release")
    check(P.expect("RELEASING", 2), "release typed after a Ctrl-C freeze")
    time.sleep(1.0)
    P.type("take")
    check(P.expect("release STOPPED", 2) and P.expect("arms taken and held", 5), "take during the release stops it")
    P.type("release")
    rc = P.exit_code(8)
    check(rc == 0, "release again -> stops (exit %s)" % rc)
    P.end()


def stop_sh():
    P = Proc()
    P.expect("type `take`", 15)
    P.type("take")
    P.expect("arms taken and held", 8)
    # a stand-in for a running test: its command line ends in run_test.py, it sleeps in this folder
    T = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", "run_test.py"], cwd=PICK)
    time.sleep(0.5)
    out = subprocess.run(["bash", "stop.sh", "list"], cwd=PICK, stdout=subprocess.PIPE, universal_newlines=True).stdout
    check(str(P.p.pid) in out and str(T.pid) in out, "stop.sh list shows the server and the test\n      " + out.replace("\n", "\n      "))
    r = subprocess.run(["bash", "stop.sh"], cwd=PICK, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=30)
    print("      " + r.stdout.replace("\n", "\n      "))
    check(T.poll() is not None, "stop.sh stopped the test")
    check(P.p.poll() == 0 and any("RELEASING" in ln for ln in P.lines), "stop.sh: the server released, then stopped (exit %s)" % P.p.poll())
    check(r.returncode == 0, "stop.sh exit code 0")
    P.end()
    if T.poll() is None:
        T.kill()
    P = Proc()
    P.expect("type `take`", 15)
    P.type("take")
    P.expect("arms taken and held", 8)
    r = subprocess.run(["bash", "stop.sh", "-9"], cwd=PICK, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=30)
    rc = P.exit_code(3)
    check(rc is not None and rc != 0 and "all gone" in r.stdout, "stop.sh -9 kills the server at once (exit %s)" % rc)
    P.end()


def hands_tool():
    """hands.py: opens them with no server, refuses
    while one runs."""
    cmd = [sys.executable, "hands.py", "open", "--mock", "--seconds", "1", "--port", str(PORT)]
    r = subprocess.run(cmd, cwd=PICK, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
    after = [ln for ln in r.stdout.splitlines() if ln.startswith("fingers after")]
    vals = [float(x) for x in after[0].replace("|", " ").split() if x.replace(".", "", 1).isdigit()] if after else []
    check(r.returncode == 0 and len(vals) == 12 and max(vals) < 0.02, "hands.py open (simulated hands closed at 0.8): %s" % (
        after[0] if after else r.stdout[-200:]))
    P = Proc()
    P.expect("type `take`", 15)
    r = subprocess.run(cmd, cwd=PICK, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True, timeout=60)
    check(r.returncode == 1 and "arm server is running" in r.stdout, "hands.py refuses while an arm server runs")
    P.type("quit")
    P.exit_code(5)
    P.end()


def main():
    typed()
    hands_tool()
    if POSIX:
        signals()
        if "--stop-sh" in sys.argv:
            stop_sh()
    else:
        print("(Windows: the Ctrl-C, kill and stop.sh checks need Linux)")
    n_bad = sum(1 for ok, _ in OK if not ok)
    print("\n%d checks, %d failed" % (len(OK), n_bad))
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
