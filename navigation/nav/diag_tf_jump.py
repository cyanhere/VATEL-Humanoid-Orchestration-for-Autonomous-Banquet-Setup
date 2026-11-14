#!/usr/bin/env python3
"""Find WHICH transform is jumping: the relocalizer's or FAST-LIO's.

map -> body is a chain:   map -> camera_init   (relocalizer, ICP, ~1 Hz)
                          camera_init -> body  (FAST-LIO, ~10 Hz)
nav_log only records the product, so a jump there could come from either. This
samples each link separately and reports steps that exceed what the robot could
physically do.

SUBSCRIBE-ONLY: no publisher, no service client. Safe during a live run.

    /usr/bin/python3 ~/.scuba/nav/diag_tf_jump.py --seconds 60

Run it while the robot is STANDING STILL for the cleanest read: then every step
is spurious by definition, and the culprit is whichever link shows them.
"""
import argparse
import math
import time

import numpy as np


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--hz', type=float, default=20.0)
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--yaw-step', type=float, default=1.0,
                    help='deg; a step bigger than this is counted')
    ap.add_argument('--xy-step', type=float, default=0.05,
                    help='m; a step bigger than this is counted')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    import tf2_ros

    LINKS = [('reloc  map->camera_init', a.map_frame, a.odom_frame),
             ('lio    camera_init->body', a.odom_frame, a.base_frame),
             ('TOTAL  map->body', a.map_frame, a.base_frame)]

    class D(Node):
        def __init__(self):
            super().__init__('diag_tf_jump')
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            self.prev = {}
            self.rec = {k[0]: [] for k in LINKS}   # (dt, dyaw_deg, dxy, stamp)
            self.stamps = {k[0]: [] for k in LINKS}
            self.t0 = time.time()
            self.create_timer(1.0 / a.hz, self.tick)
            self.create_timer(5.0, self.beat)

        def tick(self):
            now = time.time()
            for label, parent, child in LINKS:
                try:
                    tr = self.buf.lookup_transform(parent, child,
                                                   rclpy.time.Time())
                except Exception:
                    continue
                st = (tr.header.stamp.sec
                      + tr.header.stamp.nanosec * 1e-9)
                t = tr.transform.translation
                yaw = yaw_of(tr.transform.rotation)
                p = self.prev.get(label)
                if p is not None:
                    dt = now - p[0]
                    if dt > 1e-6:
                        self.rec[label].append(
                            (dt, math.degrees(wrap(yaw - p[1])),
                             math.hypot(t.x - p[2], t.y - p[3]), st))
                self.prev[label] = (now, yaw, t.x, t.y)
                self.stamps[label].append(st)

        def beat(self):
            el = time.time() - self.t0
            self.get_logger().info(
                '%5.1fs  ' % el + '  '.join(
                    '%s:%d' % (k.split()[0], len(v)) for k, v in self.rec.items()))
            if a.seconds > 0 and el >= a.seconds:
                raise SystemExit(0)

    rclpy.init()
    n = D()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass

    W = 76
    print('\n' + '=' * W)
    print('which transform is jumping?   %.0f s' % (time.time() - n.t0))
    print('=' * W)
    print('\nA step is spurious when it implies a rate the robot cannot reach.')
    print('For reference the fastest yaw ever measured on this robot is ~39 deg/s')
    print('and max_vel_x is 0.25 m/s.\n')
    for label, _, _ in LINKS:
        R = n.rec[label]
        if len(R) < 10:
            print('%-26s NO DATA (%d samples)' % (label, len(R)))
            continue
        A = np.array([(r[0], r[1], r[2]) for r in R], dtype=float)
        dt, dyaw, dxy = A[:, 0], np.abs(A[:, 1]), A[:, 2]
        yaw_rate = dyaw / dt
        xy_rate = dxy / dt
        big_y = int(np.sum(dyaw > a.yaw_step))
        big_x = int(np.sum(dxy > a.xy_step))
        # how often does the transform actually change at all?
        st = np.array(n.stamps[label])
        uniq = len(np.unique(np.round(st, 4)))
        print('%-26s %d samples, %d distinct stamps (~%.1f Hz)'
              % (label, len(R), uniq, uniq / max(time.time() - n.t0, 1e-6)))
        print('   yaw  steps >%.1f deg : %5d   max step %7.2f deg'
              ' -> %8.1f deg/s' % (a.yaw_step, big_y, dyaw.max(), yaw_rate.max()))
        print('   xy   steps >%.2f m   : %5d   max step %7.3f m'
              '   -> %8.2f m/s' % (a.xy_step, big_x, dxy.max(), xy_rate.max()))
        print('   IMPOSSIBLE (>39 deg/s): %5d of %d samples (%.1f%%)'
              % (int(np.sum(yaw_rate > 39.0)), len(R),
                 100.0 * np.sum(yaw_rate > 39.0) / len(R)))
        print()

    r_j = int(np.sum(np.array([r[1] for r in n.rec[LINKS[0][0]]] or [0.0]) != 0))
    l_j = int(np.sum(np.array([r[1] for r in n.rec[LINKS[1][0]]] or [0.0]) != 0))
    print('-' * W)
    print('VERDICT: the link whose "IMPOSSIBLE" count is high is the culprit.')
    print('  reloc high, lio ~0  -> ICP is snapping between local minima;')
    print('                         rate-limit the correction and gate on')
    print('                         inlier fraction, not rmse.')
    print('  lio high            -> FAST-LIO tracking is breaking up.')
    print('  both ~0             -> the jumps are in the CHAIN, i.e. TF')
    print('                         timing/extrapolation, not either estimate.')
    print('-' * W)

    if a.csv:
        import os
        with open(os.path.expanduser(a.csv), 'w') as f:
            f.write('link,dt,dyaw_deg,dxy_m,stamp\n')
            for label, _, _ in LINKS:
                for r in n.rec[label]:
                    f.write('%s,%.4f,%.4f,%.4f,%.4f\n'
                            % (label.split()[0], r[0], r[1], r[2], r[3]))
        print('per-sample steps written to %s' % a.csv)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
