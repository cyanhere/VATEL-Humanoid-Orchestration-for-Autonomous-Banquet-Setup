#!/usr/bin/env python3
"""Why Nav2 cannot reach the stage poses: check them against the costmap.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/reach.py

Read-only. Publishes nothing, moves nothing.
"""
import json
import math
import os
import sys
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy)
from nav_msgs.msg import OccupancyGrid


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--registry-url', default='http://localhost:8094/')
    ap.add_argument('--costmap', default='/global_costmap/costmap')
    ap.add_argument('--side-len', type=float, default=1.40)
    ap.add_argument('--stage', type=float, default=0.90)
    ap.add_argument('--standoff', type=float, default=0.30)
    ap.add_argument('--robot-radius', type=float, default=0.28)
    ap.add_argument('--timeout', type=float, default=15.0)
    a = ap.parse_args()

    try:
        doc = json.loads(urllib.request.urlopen(a.registry_url,
                                                timeout=3).read().decode())
    except Exception as e:
        print('!! cannot read the registry at %s: %s' % (a.registry_url, e))
        return 1
    ents = doc.get('entries', [])
    print('registry: %d entries' % len(ents))
    if not ents:
        print('!! no table entries - nothing to check')
        return 1

    rclpy.init()
    n = Node('reach_check')
    grid = {}
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(OccupancyGrid, a.costmap,
                          lambda m: grid.setdefault('m', m), q)
    import time
    t0 = time.time()
    while 'm' not in grid and time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' not in grid:
        print('!! no costmap on %s - is Nav2 running?' % a.costmap)
        n.destroy_node(); rclpy.shutdown()
        return 1
    g = grid['m']
    W, H = g.info.width, g.info.height
    res = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    d = g.data
    print('costmap %dx%d @ %.2f m, origin (%.2f, %.2f)' % (W, H, res, ox, oy))
    # a published OccupancyGrid is rescaled 0-100: 100 lethal, 99 inscribed
    lethal = sum(1 for v in d if v >= 99)
    unknown = sum(1 for v in d if v < 0)
    print('  cells: %d lethal/inscribed, %d unknown, %d free'
          % (lethal, unknown, len(d) - lethal - unknown))

    def cost_at(x, y):
        i = int((x - ox) / res)
        j = int((y - oy) / res)
        if not (0 <= i < W and 0 <= j < H):
            return None
        return d[j * W + i]

    def clearance(x, y, rmax=1.2):
        """Distance to the nearest lethal/inscribed cell."""
        best = rmax
        steps = int(rmax / res)
        ci = int((x - ox) / res); cj = int((y - oy) / res)
        for dj in range(-steps, steps + 1):
            for di in range(-steps, steps + 1):
                i, j = ci + di, cj + dj
                if not (0 <= i < W and 0 <= j < H):
                    continue
                if d[j * W + i] >= 99:
                    r = math.hypot(di, dj) * res
                    if r < best:
                        best = r
        return best

    print()
    print(' %-9s %-5s %-16s %6s %10s %s'
          % ('table', 'side', 'stage pose', 'cost', 'clearance', 'verdict'))
    nreach = ntot = 0
    for e in ents:
        sp = e.get('standoff_poses') or []
        cx, cy = e['cx'], e['cy']
        rs = a.side_len / 2 + a.stage + a.robot_radius
        for pose in sp:
            sd = pose.get('side', '?')
            vx, vy = pose['x'] - cx, pose['y'] - cy
            L = math.hypot(vx, vy) or 1.0
            sx, sy = cx + vx / L * rs, cy + vy / L * rs
            c = cost_at(sx, sy)
            cl = clearance(sx, sy)
            ntot += 1
            if c is None:
                v = 'OUTSIDE the costmap'
            elif c < 0:
                v = 'UNKNOWN space'
            elif c >= 99:
                v = 'BLOCKED (lethal/inscribed)'
            elif cl < 0.50:
                v = 'too tight (needs 0.50)'
            else:
                v = 'reachable'
                nreach += 1
            print(' %-9s %-5s (%6.2f,%6.2f) %6s %9.2fm  %s'
                  % (e['id'], sd, sx, sy, c if c is not None else '--', cl, v))

    # how far apart are the tables? that is what decides whether a stage pose
    # at rs metres from one centre can ever sit clear of the next
    print()
    cs = [(x['id'], x['cx'], x['cy']) for x in ents]
    for i, (idi, xi, yi) in enumerate(cs):
        best = None
        for j, (idj, xj, yj) in enumerate(cs):
            if i == j:
                continue
            dd = math.hypot(xi - xj, yi - yj)
            if best is None or dd < best[1]:
                best = (idj, dd)
        if best:
            print('  %-9s nearest neighbour %-9s %.2f m   '
                  '(stage radius is %.2f m)' % (idi, best[0], best[1], rs))

    print()
    print('REACHABLE: %d of %d stage poses' % (nreach, ntot))
    if nreach == 0:
        print()
        print('Stage radius is %.2f m from the table centre '
              '(%.2f/2 + %.2f + %.2f).' % (rs, a.side_len, a.stage,
                                           a.robot_radius))
        print('If tables sit ~3 m apart, that lands ON the neighbour.')
        print('Try a smaller --stage, or approach only the outward sides.')
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
