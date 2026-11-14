#!/usr/bin/env python3
"""Arm server: the only program that commands the arms, head and hands. Keep it running in its own terminal.

    python3 server.py                 # the robot (eth10)
    python3 server.py --mock          # no robot: simulated arms, for trying the tests offline
"""
import argparse
import os
import queue
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk.server_core import Server  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--iface", default="eth10", help="robot network interface on the NX (eth10)")
    ap.add_argument("--mock", action="store_true", help="simulated robot, no DDS")
    ap.add_argument("--kp-scale", type=float, default=1.0, help="scale Unitree's arm kp (%.1f..%.1f; the waist keeps its own)" % config.GAIN_SCALE_KP)
    ap.add_argument("--kd-scale", type=float, default=1.0, help="scale Unitree's arm kd (%.1f..%.1f); with kp x s, kd x sqrt(s) keeps the damping" % config.GAIN_SCALE_KD)
    ap.add_argument("--fric", type=float, default=0.0, help="friction lead: this fraction of the fitted joint friction is "
                    "supplied along the planned motion (0 = off, the default; try 0.5). Also `fric A` while running")
    ap.add_argument("--no-comp", action="store_true", help="no gravity compensation (as dp_play)")
    ap.add_argument("--vmax", type=float, default=config.V_MAX, help="joint speed limit for every client, rad/s")
    ap.add_argument("--take", action="store_true", help="take the arms at once (no typed `take`)")
    ap.add_argument("--home", default="neutral", help="HOME: neutral (params/home_neutral.json, the default since 2026-11-06), "
                    "pregrip (params/premotion.json, the earlier HOME), or a path")
    ap.add_argument("--dq-ff", action="store_true", help="send the planned joint speed as the motors' speed target (also `dqff on`)")
    ap.add_argument("--port", type=int, default=config.SERVER_PORT, help="UDP port (tests use another; clients use %d)" % config.SERVER_PORT)
    a = ap.parse_args()
    if a.mock:
        from pk.robot_io import MockBackend
        io = MockBackend()
    else:
        from pk.robot_io import DDSBackend
        io = DDSBackend(a.iface)
    try:
        from pk.server_core import load_home
        srv = Server(io, home=load_home(a.home), port=a.port, kp_scale=a.kp_scale, comp=not a.no_comp, v_max=a.vmax, kd_scale=a.kd_scale,
                     fric=a.fric, dq_ff=a.dq_ff)
    except RuntimeError as ex:
        print("NOT STARTED: %s" % ex)
        return 1
    print(__doc__.split("It starts")[0].strip())
    print("log: %s | gravity: %s | kp x%.2f kd x%.2f | friction lead %s | speed limit %.2f rad/s | HOME %s | dq target %s" % (
        srv.log_path, srv.comp.source if srv.comp.enabled else "OFF", srv.kp_scale, srv.kd_scale,
        ("x%.2f" % srv.fric_alpha) if srv.fric_alpha > 0 else "off", srv.v_max, srv.home_name, "planned" if srv.dq_ff else "0"))
    s = io.read()
    t0 = time.time()
    while s.age() > 1.0 and time.time() - t0 < 5.0:
        time.sleep(0.05)
        s = io.read()
    if s.age() > 1.0:
        print("NO rt/lowstate on %s: is this the robot's NX, and is the robot on?" % a.iface)
        return 1
    print("robot: arms %s | hands %s | touch %s" % (" ".join("%.2f" % v for v in s.q), "ok" if s.hands_ok() else "NO STATE (hand server?)",
                                                    "ok" if s.touch_ok() else "none"))
    print("type `take` to take the arms where they are (robot in its running mode; after a kill -9 of a server, wait\n"
          "until the arms have dropped, ~10 s: the old command holds them until then)")

    lines = queue.Queue()

    def reader():
        while True:
            try:
                ln = sys.stdin.readline()
            except (OSError, ValueError):
                return
            if not ln:
                return                      # terminal gone: keep running, use monitor.py
            lines.put(ln)
    threading.Thread(target=reader, daemon=True).start()
    presses = []                                     # times of every Ctrl-C; each one is answered (Server.ctrl_c)

    def on_int(*_):
        presses.append(time.time())
    signal.signal(signal.SIGINT, on_int)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, signal.SIG_IGN)
    term = dict(flag=False)
    signal.signal(signal.SIGTERM, lambda *_: term.__setitem__("flag", True))
    if a.take:
        srv.operator("take")

    period = 1.0 / config.RATE_HZ
    nxt = time.time()
    late = 0
    handled = 0
    while True:
        while not lines.empty():
            ln = lines.get()
            if ln.strip():
                srv.operator(ln)
                print("   -> mode %s, weight %.2f%s" % (srv.mode, srv.weight, (" (" + srv.freeze_reason + ")") if srv.mode == "FREEZE" else ""), flush=True)
        stop = False
        while handled < len(presses):
            handled += 1
            stop = srv.ctrl_c(presses[handled - 1]) or stop
        if term["flag"]:                             # kill / pkill / stop.sh: release, then stop
            term["flag"] = False
            if srv.mode == "WAIT_TAKE":
                stop = True
            else:
                srv.release(time.time(), "SIGTERM (kill or stop.sh)")
        if stop or srv.quit:
            break
        try:
            srv.step()
        except Exception as ex:                       # never let one bad cycle drop the stream
            print("cycle error (holding): %r" % ex)
            try:
                srv.freeze("server error: %r" % ex, at_measured=False)
                io.send_arm(srv.sent, srv.head_cmd, srv.weight, srv.kp, srv.kd)
            except Exception:                         # noqa: BLE001
                pass
        if srv.mode == "RELEASE" and srv.release_done and time.time() - srv.release_t > 0.5:
            break                                    # weight 0 (and open hands) sent for 0.5 s first
        nxt += period
        d = nxt - time.time()
        if d > 0:
            time.sleep(d)
        else:
            late += 1
            if d < -0.1:
                nxt = time.time()
    srv.close()
    print("server stopped. log: %s" % srv.log_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
