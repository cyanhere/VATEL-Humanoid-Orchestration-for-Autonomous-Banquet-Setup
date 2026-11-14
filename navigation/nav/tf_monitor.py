#!/usr/bin/env python3
"""Show the three transforms of the localization chain, live.

    /usr/bin/python3 ~/.scuba/nav/tf_monitor.py

        map ──────────► camera_init ──────────► body
             (1) relocalizer          (2) FAST-LIO odometry
             └──────────── (3) ────────────────┘

  (1) map -> camera_init   where FAST-LIO's origin sits on the map (the ICP correction)
  (2) camera_init -> body  how far the robot has moved since FAST-LIO started
  (3) map -> body          where the robot IS on the map   <-- what Nav2 plans from

Reading it:
  * (2) should be ~0 at startup and GROW as the robot walks. That is odometry.
  * (1) and (3) are only zero if FAST-LIO / the robot happen to sit at the map
    origin. They should be CORRECT, not zero.
  * The test that matters: does (3) match where the robot physically stands?

Stdlib + rclpy + tf2 only - no numpy, so the ~/.local numpy/scipy clash cannot
bite here.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException


def yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


class TFMonitor(Node):
    def __init__(self, mapf, odomf, basef, rate):
        super().__init__('tf_monitor')
        self.f = (mapf, odomf, basef)
        self.buf = Buffer()
        self.lis = TransformListener(self.buf, self)
        self.first = True
        self.create_timer(1.0 / rate, self.tick)

    def _lookup(self, a, b):
        try:
            t = self.buf.lookup_transform(a, b, rclpy.time.Time())
            tr, q = t.transform.translation, t.transform.rotation
            return (tr.x, tr.y, tr.z, yaw_deg(q)), None
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            return None, type(e).__name__

    def tick(self):
        mapf, odomf, basef = self.f
        rows = [
            ('1  %-14s -> %-12s' % (mapf, odomf), self._lookup(mapf, odomf),
             'relocalizer correction'),
            ('2  %-14s -> %-12s' % (odomf, basef), self._lookup(odomf, basef),
             'odometry since FAST-LIO start'),
            ('3  %-14s -> %-12s' % (mapf, basef), self._lookup(mapf, basef),
             'ROBOT ON THE MAP  <-- Nav2 uses this'),
        ]
        if not self.first:
            sys.stdout.write('\033[%dA' % (len(rows) + 3))
        self.first = False
        print('=' * 92)
        print('%-32s %9s %9s %9s %9s   %s'
              % ('transform', 'x', 'y', 'z', 'yaw deg', 'meaning'))
        print('-' * 92)
        for label, (v, err), meaning in rows:
            if v is None:
                print('%-32s %-40s %s   ' % (label, '  !! ' + err, meaning))
            else:
                x, y, z, yw = v
                d = math.hypot(x, y)
                print('%-32s %+9.3f %+9.3f %+9.3f %+9.1f   %s (|xy|=%.2f m)   '
                      % (label, x, y, z, yw, meaning, d))
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--rate', type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    n = TFMonitor(a.map_frame, a.odom_frame, a.base_frame, a.rate)
    print('waiting for TF ...')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print('\n')
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
