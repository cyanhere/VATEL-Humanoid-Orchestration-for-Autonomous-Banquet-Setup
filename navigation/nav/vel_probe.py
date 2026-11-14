#!/usr/bin/env python3
"""Find the slowest speed the G1's gait will act on.

Sends ONE fixed velocity for a few seconds on /api/sport/request - the exact
path teleop_keyboard and nav_bridge use - ramped in and out so it cannot stomp.

    source ~/.scuba/sdk_teleop/env.sh
    /usr/bin/python3 ~/.scuba/nav/vel_probe.py 0.10          # forward 0.10 m/s, 2 s
    /usr/bin/python3 ~/.scuba/nav/vel_probe.py 0.15
    /usr/bin/python3 ~/.scuba/nav/vel_probe.py 0.20          # teleop's speed: known to walk
    /usr/bin/python3 ~/.scuba/nav/vel_probe.py 0 0.10        # sideways
    /usr/bin/python3 ~/.scuba/nav/vel_probe.py 0 0 0.20      # turn only

Refuses to run while nav_bridge is up: with /cmd_vel quiet, the bridge streams
ZERO velocity at 20 Hz, which would override this probe and make every speed
look like "does not walk".

Not the SDK LocoClient path: that service acknowledges SetVelocity (code 0)
and then ignores it on this robot.
"""
import argparse
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response

GET_FSM, SET_VEL = 7001, 7105
LIN_ACC, ANG_ACC = 0.35, 0.70           # same ramp as teleop / nav_bridge
CAP = (0.40, 0.30, 0.60)


class Probe(Node):
    def __init__(self):
        super().__init__('vel_probe')
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Response, '/api/sport/response', self._on_resp, 10)
        self._lock = threading.Lock()
        self._resp = {}

    def _on_resp(self, m):
        with self._lock:
            self._resp[m.header.identity.id] = m

    def req(self, api_id, param, rid=0):
        r = Request()
        r.header.identity.api_id = api_id
        r.header.identity.id = rid
        r.header.lease.id = 0
        r.header.policy.priority = 0
        r.header.policy.noreply = False
        r.parameter = json.dumps(param)
        self.pub.publish(r)

    def fsm(self, timeout=2.0):
        rid = time.time_ns()
        self.req(GET_FSM, {}, rid)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                m = self._resp.pop(rid, None)
            if m is not None:
                try:
                    d = json.loads(m.data)
                    return d.get('data') if isinstance(d, dict) else d
                except (ValueError, TypeError):
                    return None
        return None

    def vel(self, vx, vy, w):
        self.req(SET_VEL, {'velocity': [vx, vy, w], 'duration': 0.5})


def step(cur, tgt, lim):
    return tgt if abs(tgt - cur) <= lim else cur + (lim if tgt > cur else -lim)


def main():
    ap = argparse.ArgumentParser(description='send one velocity to find the gait threshold')
    ap.add_argument('vx', type=float)
    ap.add_argument('vy', type=float, nargs='?', default=0.0)
    ap.add_argument('vyaw', type=float, nargs='?', default=0.0)
    ap.add_argument('--secs', type=float, default=2.0, help='hold time at speed')
    ap.add_argument('--force', action='store_true', help='skip the FSM check')
    a = ap.parse_args()
    tgt = [a.vx, a.vy, a.vyaw]
    for i, (v, c) in enumerate(zip(tgt, CAP)):
        if abs(v) > c:
            sys.exit('!! %s=%.2f exceeds the probe cap %.2f' % (('vx', 'vy', 'vyaw')[i], v, c))

    rclpy.init()
    n = Probe()

    def refuse(msg):               # checks fail BEFORE any motion: send nothing
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(msg)

    t0 = time.time()
    while time.time() - t0 < 1.5:                     # let discovery see other nodes
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'g1_nav_bridge' in n.get_node_names():
        refuse('!! nav_bridge is running - it streams zero velocity and would '
               'override this probe.\n   Ctrl-C it (terminal D) and re-run.')
    f = n.fsm()
    print('FSM: %s' % f)
    if f not in (500, 801) and not a.force:
        refuse('!! robot is not in a walk FSM (500/801) - set it with the remote.')

    cur = [0.0, 0.0, 0.0]
    try:
        dt = 0.05
        lims = (LIN_ACC * dt, LIN_ACC * dt, ANG_ACC * dt)
        print('commanding vx=%.2f vy=%.2f vyaw=%.2f, hold %.1f s - WATCH THE ROBOT'
              % (a.vx, a.vy, a.vyaw, a.secs))
        phase_end = None
        t_last = 0.0
        while True:
            reached = all(abs(c - t) < 1e-9 for c, t in zip(cur, tgt))
            if reached and phase_end is None:
                phase_end = time.time() + a.secs
            if phase_end is not None and time.time() >= phase_end:
                break
            cur = [step(c, t, l) for c, t, l in zip(cur, tgt, lims)]
            n.vel(*cur)
            if time.time() - t_last > 0.5:
                print('  sent  vx=%+.3f vy=%+.3f vyaw=%+.3f' % tuple(cur))
                t_last = time.time()
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        # ramp down, then make sure a zero lands
        try:
            zero = [0.0, 0.0, 0.0]
            lims = (LIN_ACC * 0.1, LIN_ACC * 0.1, ANG_ACC * 0.1)   # 2x decel
            for _ in range(40):
                if all(abs(c) < 1e-9 for c in cur):
                    break
                cur = [step(c, 0.0, l) for c, l in zip(cur, lims)]
                n.vel(*cur)
                time.sleep(0.05)
        except BaseException:
            pass
        for _ in range(3):
            try:
                n.vel(0.0, 0.0, 0.0)
            except BaseException:
                pass
        print('stopped.  Did the robot WALK at this speed?  (yes / stepped in place / stood still)')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
