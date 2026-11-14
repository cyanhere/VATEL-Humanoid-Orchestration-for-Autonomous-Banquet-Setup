#!/usr/bin/env python3
"""Show WHERE in the TF chain the frame inversion lives.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/reloc/tf_frames.py

For each link it prints R[2,2] and the angle between the two frames' z-axes.
That pins down which transform carries the ~150 deg flip between the inverted
sensor and the gravity-aligned map:

    R[2,2] ~ +1  the two frames' z agree      (no flip here)
    R[2,2] ~ -1  the two frames' z oppose     (flip lives HERE)
"""
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros

LINKS = [('map', 'camera_init', 'relocalizer correction'),
         ('camera_init', 'body', 'FAST-LIO odometry'),
         ('map', 'body', 'what Nav2 consumes'),
         ('body', 'base_link', 'static mount correction')]


class TfFrames(Node):
    def __init__(self):
        super().__init__('tf_frames')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def get(self, parent, child):
        t = self.buf.lookup_transform(parent, child, rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
        x, y, z, w = x/n, y/n, z/n, w/n
        R = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])
        return R, np.array([v.x, v.y, v.z])


def main():
    rclpy.init()
    n = TfFrames()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 4.0:
        rclpy.spin_once(n, timeout_sec=0.1)

    print('%-26s %9s %8s %9s   %s' % ('link', 'R[2,2]', 'z-angle', 'yaw', 'note'))
    print('-' * 84)
    for p, c, note in LINKS:
        try:
            R, t = n.get(p, c)
        except Exception as e:
            msg = str(e).split('\n')[0][:34]
            print('%-26s %9s %8s %9s   %s' % ('%s -> %s' % (p, c), '--', '--', '--', msg))
            continue
        zang = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
        flag = 'FLIPPED' if R[2, 2] < 0 else ''
        print('%-26s %+9.4f %7.1f%s %+8.1f%s   %s %s'
              % ('%s -> %s' % (p, c), R[2, 2], zang, 'd', yaw, 'd', note, flag))
        print('%-26s translation [%+.2f %+.2f %+.2f]' % ('', t[0], t[1], t[2]))

    print()
    print('The link whose R[2,2] is negative is where the inverted mount is')
    print('being compensated. If NO link is negative, nothing compensates it')
    print('and every consumer of these frames sees an upside-down robot.')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
