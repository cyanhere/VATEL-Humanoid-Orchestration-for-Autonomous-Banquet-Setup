#!/usr/bin/env python3
"""Search for the robot's pose by sweeping heading (and optionally position).

    # sweep 24 headings at the current TF position
    /usr/bin/python3 find_pose.py

    # sweep headings at a position you specify
    /usr/bin/python3 find_pose.py --x -2.0 --y 2.0

    # also sweep +/-1.5 m of position on a grid
    /usr/bin/python3 find_pose.py --x -2.0 --y 2.0 --xy-range 1.5 --xy-step 0.5

    # accept the best result and publish it to /initialpose
    /usr/bin/python3 find_pose.py --apply

ICP only refines a pose that is already close; it cannot find a heading that is
60 deg out. This brute-forces the heading, scores each with the same ICP the
relocalizer uses, and ranks them. A clear winner means the pose is unambiguous;
several near-equal scores mean the geometry genuinely is ambiguous there.
"""
import os
import sys

if not os.environ.get('PYTHONNOUSERSITE'):
    os.environ['PYTHONNOUSERSITE'] = '1'          # ~/.local numpy vs system scipy
    os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.expanduser('~/.scuba/reloc'))
_src = open(os.path.expanduser('~/.scuba/reloc/relocalizer.py')).read()
_ns = {}
exec(compile(_src.split("class Relocalizer")[0], 'reloc', 'exec'), _ns)
pc2_xyz = _ns['pc2_xyz']; voxel_downsample = _ns['voxel_downsample']
yaw_to_R = _ns['yaw_to_R']; icp = _ns['icp']; compute_normals = _ns['compute_normals']


class Finder(Node):
    def __init__(self, args):
        super().__init__('find_pose')
        self.a = args
        self.cloud = None
        self.accum = []
        self.create_subscription(PointCloud2, '/cloud_registered', self.cb, 1)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

    def cb(self, msg):
        P = pc2_xyz(msg)
        P = P[(P[:, 2] > -0.5) & (P[:, 2] < 2.5)]
        if len(P):
            self.accum.append(P)
            if len(self.accum) > 8:
                self.accum.pop(0)
            self.cloud = voxel_downsample(np.vstack(self.accum), 0.10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser('~/.scuba/maps/ilounge_02_loc_cloud.npy'))
    ap.add_argument('--x', type=float, default=None)
    ap.add_argument('--y', type=float, default=None)
    ap.add_argument('--yaws', type=int, default=24, help='headings to try over 360 deg')
    ap.add_argument('--xy-range', type=float, default=0.0, help='+/- metres to sweep')
    ap.add_argument('--xy-step', type=float, default=0.5)
    ap.add_argument('--apply', action='store_true', help='publish the winner to /initialpose')
    ap.add_argument('--top', type=int, default=8)
    a = ap.parse_args()

    M = np.load(os.path.expanduser(a.map)).astype(np.float64)
    M = M[(M[:, 2] > -0.5) & (M[:, 2] < 2.5)]
    M = voxel_downsample(M, 0.10)
    tree = cKDTree(M)
    nrm = compute_normals(M, tree, k=12,
                          cache=os.path.splitext(os.path.expanduser(a.map))[0]
                          + '_normals_10cm.npy')
    print('map: %d pts' % len(M))

    rclpy.init()
    n = Finder(a)
    print('collecting scans ...')
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 12:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.cloud is not None and len(n.cloud) > 600 and time.time() - t0 > 4:
            break
    if n.cloud is None:
        sys.exit('!! no /cloud_registered - is run_lio.sh running?')
    S = n.cloud
    print('scan: %d pts' % len(S))

    cx = a.x if a.x is not None else 0.0
    cy = a.y if a.y is not None else 0.0
    xs = [cx] if a.xy_range <= 0 else list(np.arange(cx - a.xy_range, cx + a.xy_range + 1e-9, a.xy_step))
    ys = [cy] if a.xy_range <= 0 else list(np.arange(cy - a.xy_range, cy + a.xy_range + 1e-9, a.xy_step))
    yaws = [2 * math.pi * i / a.yaws for i in range(a.yaws)]
    total = len(xs) * len(ys) * len(yaws)
    print('trying %d poses (%d x, %d y, %d yaw) ...' % (total, len(xs), len(ys), len(yaws)))

    out, k = [], 0
    for x in xs:
        for y in ys:
            for yw in yaws:
                T0 = np.eye(4)
                T0[:3, :3] = yaw_to_R(yw)
                T0[:3, 3] = [x, y, 0.0]
                T, rmse, ratio = icp(S, tree, T0, iters=12,
                                     gates=[1.0, 0.5, 0.3], normals=nrm)
                fy = math.degrees(math.atan2(T[1, 0], T[0, 0]))
                out.append((rmse, ratio, T[0, 3], T[1, 3], fy, math.degrees(yw)))
                k += 1
                print('\r  %d/%d' % (k, total), end='', flush=True)
    print()

    out.sort(key=lambda r: (r[0] if np.isfinite(r[0]) else 9e9))
    print('\n%5s %8s %8s %9s %9s %10s %10s' %
          ('rank', 'rmse', 'inliers', 'x', 'y', 'yaw', 'from yaw'))
    print('-' * 70)
    for i, (rm, ra, x, y, fy, sy) in enumerate(out[:a.top], 1):
        print('%5d %8.3f %7.0f%% %+9.2f %+9.2f %+9.1f %+10.0f'
              % (i, rm, 100 * ra, x, y, fy, sy))

    best = out[0]
    if len(out) > 1:
        gap = out[1][0] - best[0]
        print('\nbest rmse %.3f; next best %.3f (gap %.3f)' % (best[0], out[1][0], gap))
        if gap < 0.02:
            print('  !! AMBIGUOUS - two poses fit almost equally well.')
            print('     Move the robot somewhere with more distinctive geometry and retry.')
        else:
            print('  clear winner.')

    if a.apply:
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = float(best[2])
        m.pose.pose.position.y = float(best[3])
        yw = math.radians(best[4])
        m.pose.pose.orientation.z = math.sin(yw / 2)
        m.pose.pose.orientation.w = math.cos(yw / 2)
        for _ in range(3):
            n.pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.1)
        print('\npublished to /initialpose: (%.2f, %.2f) yaw %.1f deg'
              % (best[2], best[3], best[4]))
    else:
        print('\nto apply:  add --apply    (or seed the relocalizer with these values)')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
