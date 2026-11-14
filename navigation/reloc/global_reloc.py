#!/usr/bin/env python3
"""Find the robot anywhere in the map - no seed, no guess at the heading.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/reloc/global_reloc.py                # search, report
    /usr/bin/python3 ~/.scuba/reloc/global_reloc.py --apply        # ...and use it
    /usr/bin/python3 ~/.scuba/reloc/global_reloc.py --around 0 0 4 # search near (0,0)

WHY THIS EXISTS
ICP only refines a pose that is already within ~0.5 m and ~10 deg of the truth.
Handed anything further out it does not fail loudly - it slides into the nearest
wall alignment and reports a healthy-looking RMSE at a pose that is metres and
tens of degrees wrong.  This brute-forces the whole map instead: every free cell
on a 0.75 m grid, 24 headings each, cheap-scored and then ICP-refined.

It ranks by INLIER FRACTION, not RMSE.  In a room of parallel walls a wrong pose
still yields a fine RMSE from the handful of points that happen to line up; what
separates right from wrong is how much of the scan finds any support at all.
Offline against this map: correct to 2-6 cm in 4 of 5 trials, and the 5th was
correctly reported as LOW CONFIDENCE rather than answered wrongly.

Numpy only - no scipy, so it is immune to the numpy/scipy version clash.
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import globloc as G

GATES = (1.5, 0.8, 0.4, 0.25)
DEF_MAP = '~/.scuba/maps/ilounge_02_loc_cloud.npy'
DEF_YAML = '~/.scuba/maps/ilounge_02_map.yaml'


def pc2_xyz(msg):
    n = msg.width * msg.height
    step = msg.point_step
    off = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
    out = np.empty((n, 3), dtype=np.float64)
    for i, ax in enumerate('xyz'):
        out[:, i] = buf[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


class GlobalReloc(Node):
    def __init__(self):
        super().__init__('global_reloc')
        self.accum, self.n = [], 0
        self.create_subscription(PointCloud2, '/cloud_registered', self._cb, 5)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _cb(self, msg):
        P = pc2_xyz(msg)
        P = P[(P[:, 2] > -0.5) & (P[:, 2] < 2.5)]
        if len(P):
            self.accum.append(P)
            self.n += 1
            if len(self.accum) > 10:
                self.accum.pop(0)

    def odom_to_body(self):
        """camera_init -> body, so the scan can be put in the robot's frame."""
        t = self.buf.lookup_transform('camera_init', 'body', rclpy.time.Time())
        q = t.transform.rotation
        v = t.transform.translation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        T = np.eye(4)
        T[:3, :3] = G.yaw_to_R(yaw)
        T[:3, 3] = (v.x, v.y, v.z)
        return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=DEF_MAP)
    ap.add_argument('--yaml', default=DEF_YAML)
    ap.add_argument('--spacing', type=float, default=0.75)
    ap.add_argument('--yaws', type=int, default=24)
    ap.add_argument('--around', nargs=3, type=float, metavar=('X', 'Y', 'R'),
                    help='limit the search to R metres around (X,Y) - much faster')
    ap.add_argument('--top', type=int, default=5)
    ap.add_argument('--apply', action='store_true',
                    help='publish the winner to /initialpose')
    ap.add_argument('--force', action='store_true',
                    help='apply even when the result is not CONFIDENT')
    ap.add_argument('--z', default='auto',
                    help="sensor height above the map floor: 'auto' (match the "
                         "floor planes, correct for a rack or tripod of any "
                         "height) or a number in metres")
    a = ap.parse_args()

    mp = os.path.expanduser(a.map)
    yp = os.path.expanduser(a.yaml)
    for p in (mp, yp):
        if not os.path.exists(p):
            sys.exit('!! missing %s\n   pass --map / --yaml with the right paths' % p)

    print('loading map ...')
    M = np.load(mp).astype(np.float64)
    M = M[(M[:, 2] > -0.5) & (M[:, 2] < 2.5)]
    M = G.voxel_downsample(M, 0.10)
    nn = G.MultiNN(M, GATES)
    occ, res, org = G.read_pgm_map(yp)
    pos = G.free_positions(occ, res, org, spacing=a.spacing,
                           bounds=tuple(a.around) if a.around else None)
    if not len(pos):
        sys.exit('!! no free candidate positions - check --yaml / --around')
    print('map %d pts | %d free positions x %d yaws = %d coarse evals'
          % (len(M), len(pos), a.yaws, len(pos) * a.yaws))

    rclpy.init()
    node = GlobalReloc()
    print('collecting scans (needs FAST-LIO running) ...')
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.n >= 8:
            break
    if not node.accum:
        sys.exit('!! no /cloud_registered - is run_lio.sh running in terminal A?')
    try:
        T_ob = node.odom_to_body()
    except Exception as e:
        sys.exit('!! no camera_init->body TF (%s)' % e)

    S = np.vstack(node.accum)
    S = (S - T_ob[:3, 3]) @ T_ob[:3, :3]          # odom frame -> body frame
    S = G.voxel_downsample(S, 0.15)
    print('scan %d pts from %d clouds' % (len(S), node.n))

    # The search must know how high the sensor is. Scoring at z=0 while the
    # sensor sits at another height puts every scan point a fixed offset off
    # its surface, which reads as a poor fit everywhere - so a rack at a
    # different height than the mapping run would look like a broken map.
    if a.z == 'auto':
        z = G.estimate_z(S, M)
        print('sensor height (floor-matched): %.2f m above the map floor' % z)
    else:
        z = float(a.z)
        print('sensor height (given): %.2f m' % z)

    print('searching ...')
    t0 = time.time()
    out = G.sweep(nn, S, pos, n_yaw=a.yaws, gates=GATES, z=z,
                  progress=lambda i, n: print('\r  %d/%d positions' % (i, n),
                                              end='', flush=True))
    print('\rsearched %d positions in %.0f s        ' % (len(pos), time.time() - t0))

    print('\n%5s %9s %8s %9s %9s %9s' % ('rank', 'inliers', 'rmse', 'x', 'y', 'yaw'))
    print('-' * 56)
    for i, (inl, rm, x, y, yw, _) in enumerate(out[:a.top], 1):
        print('%5d %8.0f%% %8.3f %+9.2f %+9.2f %+9.1f'
              % (i, 100 * inl, rm, x, y, math.degrees(yw)))

    ok, msg = G.verdict(out)
    print('\n%s' % msg)

    if a.apply and (ok or a.force):
        inl, rm, x, y, yw, _ = out[0]
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = node.get_clock().now().to_msg()
        m.pose.pose.position.x = float(x)
        m.pose.pose.position.y = float(y)
        m.pose.pose.orientation.z = math.sin(yw / 2)
        m.pose.pose.orientation.w = math.cos(yw / 2)
        for _ in range(3):
            node.pub.publish(m)
            rclpy.spin_once(node, timeout_sec=0.1)
        print('published to /initialpose: (%.2f, %.2f) yaw %.1f deg'
              % (x, y, math.degrees(yw)))
        print('watch terminal B for the seed-override line, then check tf_monitor row 3.')
    elif a.apply:
        print('NOT applied - result is not confident. Re-run with --force to override.')
    else:
        print('to use this result, re-run with --apply')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
