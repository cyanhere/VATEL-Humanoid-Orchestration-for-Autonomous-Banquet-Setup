#!/usr/bin/env python3
"""Watch the arm server, or send it a command, from any terminal.

    python3 monitor.py              # live: mode, owner, tracking error, torques, fingers, touch, plate (1 line / 0.5 s)
    python3 monitor.py f            # FREEZE now (also: freeze / stop)
    python3 monitor.py home         # go straight back to HOME (the neutral pose)
    python3 monitor.py home path    # go back to HOME along the path the arms came
    python3 monitor.py status
    python3 monitor.py take | release # release: from any mode, the robot lowers the arms (table away); `take` stops it
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from pk import config  # noqa: E402
from pk.client import Client, ServerGone, operator  # noqa: E402


def main():
    if len(sys.argv) > 1:
        said = []
        mode = operator(" ".join(sys.argv[1:]), said=said)
        for ln in said:
            print("server: " + ln)
        print("server mode: %s" % mode if mode else "no answer from the server (is server.py running?)")
        return 0 if mode else 1
    try:
        c = Client("monitor", role="monitor")
    except ServerGone as ex:
        print(ex)
        return 1
    print("Ctrl-C quits the monitor only (the robot is not affected). `python3 monitor.py f` freezes.")
    while True:
        try:
            st = c.state(max_age=1.0)
        except ServerGone:
            print("server silent")
            time.sleep(1.0)
            continue
        c._send(dict(op="sub"))
        q, des = np.array(st["q"]), np.array(st["des"])
        e = np.abs(des - q)[:10]
        j = int(np.argmax(e))
        tau = np.array(st["tau"])
        k = int(np.argmax(np.abs(tau)))
        hands = np.array(st["hands"])
        touch = st.get("touch")
        tch = ("touch L-pinky %4.0f R idx/mid/ring %4.0f/%4.0f/%4.0f" % (touch[0][4][0], touch[1][1][0], touch[1][2][0], touch[1][3][0])) if touch else "touch -"
        p = st.get("plate")
        pl = ("plate (%.3f %+.3f %.3f) %.1fs" % (tuple(p["center"]) + (st["plate_age"],))) if isinstance(p, dict) and p.get("center") else "plate -"
        print("%s %-7s %-12s %-10s err %.3f %-12s |tau| %4.1f %-12s hands L %s R %s %s %s" % (
            time.strftime("%H:%M:%S"), st["mode"], (st["owner"] or "-")[:12], (st["tag"] or "-")[:10], e[j], config.ARM_NAMES[j],
            abs(tau[k]), config.ARM_NAMES[k], " ".join("%.2f" % v for v in hands[:6]), " ".join("%.2f" % v for v in hands[6:]), tch, pl)
              + ("  FROZEN: " + st["freeze"] if st.get("freeze") else ""), flush=True)
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
