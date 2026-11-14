#!/usr/bin/env python3
"""Open the Revo2 hands, or read their state, when no arm server is running.

    python3 hands.py open                 # both hands open over 2 s
    python3 hands.py open --side left     # only the robot's LEFT hand (your RIGHT)
    python3 hands.py status               # where the fingers are; nothing moves
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from pk import config  # noqa: E402
from pk.server_core import another_server  # noqa: E402


def fingers(h):
    return " ".join("%.2f" % v for v in h)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("what", choices=("open", "status"))
    ap.add_argument("--side", choices=("both", "left", "right"), default="both", help="the ROBOT's side (left = your right)")
    ap.add_argument("--seconds", type=float, default=2.0, help="how long the opening takes")
    ap.add_argument("--iface", default="eth10")
    ap.add_argument("--port", type=int, default=config.SERVER_PORT, help="where an arm server would answer")
    ap.add_argument("--mock", action="store_true", help="simulated hands, for an offline check")
    a = ap.parse_args()
    if another_server(a.port):
        print("an arm server is running: it owns the hands. Type `release` there (it opens them), or stop it first")
        return 1
    if a.mock:
        from pk.robot_io import MockBackend
        io = MockBackend()
        io.s.hand_q[:] = io.hcmd[:] = 0.8                 # as if a grip had been left closed
    else:
        from pk.robot_io import DDSBackend
        io = DDSBackend(a.iface)
    s = io.read()
    t0 = time.time()
    while not s.hands_ok() and time.time() - t0 < 3.0:
        time.sleep(0.05)
        s = io.read()
    if not s.hands_ok():
        print("no hand state: is brainco_hand_server running?")
        return 1
    print("fingers (%s): robot left %s | robot right %s" % (", ".join(config.FINGERS), fingers(s.hand_q[:6]), fingers(s.hand_q[6:])))
    if a.what == "status":
        return 0
    sides = [0, 1] if a.side == "both" else [0] if a.side == "left" else [1]
    h0 = s.hand_q.copy()
    io.open_hands()
    T = max(a.seconds, 0.5)
    t0 = time.time()
    while True:                                           # from where the fingers ARE to open
        u = min(1.0, (time.time() - t0) / T)
        for k in sides:
            io.send_hand(k, h0[6 * k:6 * k + 6] * (1.0 - u))
        io.read()
        if u >= 1.0:
            break
        time.sleep(0.02)
    for _ in range(25):                                   # the open command for 0.5 s more (best-effort topic)
        for k in sides:
            io.send_hand(k, np.zeros(6))
        time.sleep(0.02)
    s = io.read()
    print("fingers after: robot left %s | robot right %s" % (fingers(s.hand_q[:6]), fingers(s.hand_q[6:])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
