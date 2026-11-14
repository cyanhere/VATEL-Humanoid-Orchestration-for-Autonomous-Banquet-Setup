#!/usr/bin/env python3
"""Where does the costmap actually block the planner?

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/costmap_profile.py

Prints cost as a function of distance from a REAL obstacle. The band at
cost 99 (inscribed) is what NavFn refuses to plan through, and its width is
the costmap's robot_radius. Run it before and after changing that radius -
if the band does not change width, the change did NOT take effect.

Read-only. Publishes nothing, moves nothing.
"""
import argparse
import math
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from nav_msgs.msg import OccupancyGrid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/global_costmap/costmap')
    ap.add_argument('--max', type=float, default=1.6, help='m to profile')
    ap.add_argument('--timeout', type=float, default=15.0)
    a = ap.parse_args()

    rclpy.init()
    n = Node('costmap_profile')
    grid = {}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(OccupancyGrid, a.topic,
                          lambda m: grid.setdefault('m', m), q)
    t0 = time.time()
    while 'm' not in grid and time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' not in grid:
        print('!! no costmap on %s - is Nav2 running?' % a.topic)
        n.destroy_node(); rclpy.shutdown()
        return 1
    g = grid['m']
    W, H, res = g.info.width, g.info.height, g.info.resolution
    d = g.data
    print('%s  %dx%d @ %.2f m' % (a.topic, W, H, res))

    # 8-connected BFS from every lethal cell, so diagonals are not
    # overestimated the way a 4-connected one does
    INF = 10 ** 9
    dist = [INF] * (W * H)
    dq = deque()
    for k, v in enumerate(d):
        if v >= 100:
            dist[k] = 0
            dq.append(k)
    if not dq:
        print('!! no lethal cells - nothing marked yet')
        n.destroy_node(); rclpy.shutdown()
        return 1
    nb = ((1, 0, 10), (-1, 0, 10), (0, 1, 10), (0, -1, 10),
          (1, 1, 14), (1, -1, 14), (-1, 1, 14), (-1, -1, 14))
    while dq:
        k = dq.popleft()
        j, i = divmod(k, W)
        for di, dj, w in nb:
            ii, jj = i + di, j + dj
            if 0 <= ii < W and 0 <= jj < H:
                k2 = jj * W + ii
                if dist[k2] > dist[k] + w:
                    dist[k2] = dist[k] + w
                    dq.append(k2)

    buckets = {}
    for k, v in enumerate(d):
        if dist[k] >= INF or v < 0:
            continue
        r = (dist[k] / 10.0) * res
        if r > a.max:
            continue
        buckets.setdefault(round(r, 1), []).append(v)

    print()
    print(' %8s %9s %6s %6s  %s' % ('dist', 'cells', 'min', 'max', 'meaning'))
    block_end = 0.0
    for b in sorted(buckets):
        vs = buckets[b]
        mn, mx = min(vs), max(vs)
        if mx >= 100:
            m = 'LETHAL'
        elif mx >= 99:
            m = 'INSCRIBED - planner REFUSES the robot centre here'
            block_end = max(block_end, b)
        elif mx > 0:
            m = 'soft gradient - avoided, but passable'
        else:
            m = 'free'
        print(' %7.1fm %9d %6d %6d  %s' % (b, len(vs), mn, mx, m))
    print()
    print('BLOCKING BAND ends at about %.1f m.' % block_end)
    print('That width IS the costmap robot_radius. It reserves a %.1f m'
          % (2 * block_end))
    print('corridor; the robot is 0.56 m wide.')
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
