"""
Run every offline test suite. No robot, no camera, no ROS.

    python3 run_tests.py            # all of them
    python3 run_tests.py plan       # just the ones whose name contains "plan"

Run this before copying anything to the robot. Passing does NOT mean a change is
safe on hardware -- it means the logic is right and the refusals still refuse. The
staged hold/nudge/ready discipline still applies.

The times printed swing a lot (measured: the same 49-iteration IK solve took 0.16 s
and 0.84 s minutes apart on the same laptop). The planning suite runs ~900 dual-arm
IK solves, so it can take anywhere from 2 to 8 minutes. A slow run is the laptop,
not a regression -- compare check COUNTS, not seconds.
"""
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def main(argv):
    pattern = argv[1] if len(argv) > 1 else ""
    suites = sorted(os.path.basename(p) for p in glob.glob(os.path.join(HERE, "test_*.py"))
                    if pattern in os.path.basename(p))
    if not suites:
        print("no test files match %r" % pattern)
        return 2

    failed, total = [], 0
    for name in suites:
        started = time.time()
        r = subprocess.run([sys.executable, name], cwd=HERE,
                           capture_output=True, text=True)
        body = r.stdout + r.stderr
        passed = body.count("PASS")
        total += passed
        head = "%-36s %3d checks  %5.1fs" % (name, passed, time.time() - started)
        if r.returncode == 0:
            print("  ok    %s" % head)
        else:
            failed.append(name)
            print("  FAIL  %s" % head)
            for line in body.splitlines():
                if "FAIL" in line or "Error" in line or "Traceback" in line:
                    print("        %s" % line.strip())

    print("\n%d checks across %d suites%s"
          % (total, len(suites),
             ": ALL PASS" if not failed else " -- FAILED: %s" % ", ".join(failed)))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
