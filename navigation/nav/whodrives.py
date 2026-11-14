#!/usr/bin/env python3
"""Who is commanding the robot right now: DWB, or send_goal's yaw finisher?

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/whodrives.py          # until Ctrl-C

Read-only. Publishes nothing.

They are distinguishable without needing publisher identity:
  * DWB streams continuously while a FollowPath goal is ACTIVE, at
    controller_frequency (10 Hz here).
  * the finisher publishes a dense burst (~50 Hz) for the step duration,
    then a run of explicit zeros, then a silent gap while it reads TF.
So a burst/gap pattern with no active FollowPath goal is the finisher; a
steady stream with an active goal is DWB.
"""
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray


def main():
    rclpy.init()
    n = Node('whodrives')
    ev = deque()
    act = {'status': None, 't': 0.0}

    def on_cmd(m):
        ev.append((time.time(), m.linear.x, m.angular.z))

    def on_status(m):
        live = [s for s in m.status_list if s.status in (1, 2)]   # accepted/exec
        act['status'] = 'ACTIVE' if live else 'idle'
        act['t'] = time.time()

    n.create_subscription(Twist, '/cmd_vel', on_cmd, 50)
    n.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                          on_status, 10)
    print('watching /cmd_vel and the FollowPath action. Ctrl-C to stop.')
    print(' %8s %7s %8s %8s %9s %10s  %s'
          % ('t', 'msgs', 'rate', 'vx', 'wz', 'FollowPath', 'verdict'))
    t0 = time.time()
    last = t0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.05)
            now = time.time()
            if now - last < 1.0:
                continue
            last = now
            win = [e for e in ev if now - e[0] <= 1.0]
            ev.clear(); ev.extend(win)
            nmsg = len(win)
            rate = nmsg / 1.0
            mvx = max((abs(e[1]) for e in win), default=0.0)
            mwz = max((abs(e[2]) for e in win), default=0.0)
            fp = act['status'] or '--'
            if now - act['t'] > 3.0:
                fp = '--'
            if nmsg == 0:
                v = 'nobody commanding'
            elif fp == 'ACTIVE' and rate >= 6:
                v = 'DWB (goal active, steady stream)'
            elif fp != 'ACTIVE' and rate >= 25:
                v = 'FINISHER (burst, no active goal)'
            elif fp == 'ACTIVE':
                v = 'DWB-ish (goal active, rate %.0f)' % rate
            else:
                v = 'someone else (no goal, rate %.0f)' % rate
            print(' %7.1fs %7d %7.0f/s %8.2f %9.2f %10s  %s'
                  % (now - t0, nmsg, rate, mvx, mwz, fp, v))
    except KeyboardInterrupt:
        print('\nstopped')
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
