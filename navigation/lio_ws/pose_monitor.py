#!/usr/bin/env python3
"""Live FAST-LIO pose readout. Foxy-safe (no --field flag needed).

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/lio_ws/pose_monitor.py

Shows position, heading, distance travelled and rate - so you can walk the
robot and see whether LIO is tracking sensibly.
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.create_subscription(Odometry, '/Odometry', self.cb, 10)
        self.n = 0
        self.t0 = None
        self.last = None
        self.dist = 0.0
        self.start = None
        self.get_logger().info('waiting for /Odometry ...')

    def cb(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.start = (p.x, p.y, p.z)
        self.n += 1
        cur = (p.x, p.y, p.z)
        if self.last is not None:
            self.dist += math.dist(cur, self.last)
        self.last = cur
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                      1 - 2 * (q.y * q.y + q.z * q.z)))
        drift = math.dist(cur, self.start)
        hz = self.n / max(now - self.t0, 1e-6)
        print("\r x=%+7.3f y=%+7.3f z=%+7.3f  yaw=%+7.1f deg | "
              "travelled %6.2f m  from start %6.2f m | %5.1f Hz  n=%d   "
              % (p.x, p.y, p.z, yaw, self.dist, drift, hz, self.n), end="", flush=True)


def main():
    rclpy.init()
    n = PoseMonitor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        print()
        if n.last:
            print("final: x=%+.3f y=%+.3f z=%+.3f  travelled %.2f m  "
                  "net %.2f m from start" %
                  (n.last[0], n.last[1], n.last[2], n.dist,
                   math.dist(n.last, n.start)))
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
