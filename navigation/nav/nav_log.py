#!/usr/bin/env python3
"""Log everything a navigation run went through, to one CSV.

    source ~/.scuba/sdk_teleop/env.sh        # for unitree_api (the SENT column)
    /usr/bin/python3 ~/.scuba/nav/nav_log.py
    /usr/bin/python3 ~/.scuba/nav/nav_log.py --rate 50 --duration 120
    /usr/bin/python3 ~/.scuba/nav/nav_log.py --out ~/.scuba/logs/turn_test.csv

Ctrl-C to stop; the summary prints then.

WHAT IT RECORDS, one row per sample at --rate:

    pose        x, y, heading, tilt          map -> body from TF (level heading)
    DWB         cmd_vx, cmd_vy, cmd_wz       /cmd_vel, what the controller asked
    SENT        sent_vx, sent_vy, sent_wz    SetVelocity actually put on
                                             /api/sport/request - the real
                                             instruction the robot received,
                                             after the bridge clamped and ramped
    measured    meas_fwd, meas_left, meas_wz /odom_2d, what the robot then did
    fitness     rmse                         relocalizer fit

Those three velocity groups are the point: asked / sent / achieved, side by
side and time-aligned, which is what tells you whether a command was clipped by
the bridge or ignored by the gait.

Subscribe-only: it commands nothing. Safe to leave running during any test.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf2_ros

try:
    from unitree_api.msg import Request
except ImportError:                       # not in the lio_ws env
    Request = None

SET_VEL = 7105
COLS = ['t', 'x', 'y', 'heading_deg', 'tilt_deg',
        'cmd_vx', 'cmd_vy', 'cmd_wz',
        'sent_vx', 'sent_vy', 'sent_wz',
        'meas_fwd', 'meas_left', 'meas_wz', 'rmse', 'frame']


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Logger(Node):
    def __init__(self, a):
        super().__init__('nav_log')
        self.a = a
        self.lock = threading.Lock()
        self.cmd = self.sent = self.meas = (None, None, None)
        self.rmse = None
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Twist, a.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Odometry, a.odom2d_topic, self.on_odom, 10)
        self.create_subscription(Float32, a.fitness_topic, self.on_fit, 10)
        self.have_sent = False
        if Request is not None and a.api_topic:
            self.create_subscription(Request, a.api_topic, self.on_api, 20)
            self.have_sent = True

    def on_cmd(self, m):
        with self.lock:
            self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def on_odom(self, m):
        t = m.twist.twist
        with self.lock:
            self.meas = (t.linear.x, t.linear.y, t.angular.z)

    def on_fit(self, m):
        with self.lock:
            self.rmse = m.data

    def on_api(self, m):
        """SetVelocity as the robot receives it - after clamp and ramp."""
        if m.header.identity.api_id != SET_VEL:
            return
        try:
            v = json.loads(m.parameter).get('velocity')
        except (ValueError, TypeError):
            return
        if isinstance(v, list) and len(v) >= 3:
            with self.lock:
                self.sent = (float(v[0]), float(v[1]), float(v[2]))

    def pose(self):
        for parent, tag in ((self.a.map_frame, 'map'), (self.a.odom_frame, 'odom')):
            try:
                t = self.buf.lookup_transform(parent, self.a.base_frame, rclpy.time.Time())
            except Exception:
                continue
            q, v = t.transform.rotation, t.transform.translation
            R = quat_to_R(q.x, q.y, q.z, q.w)
            return (v.x, v.y, math.degrees(math.atan2(R[1, 0], R[0, 0])),
                    math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2]))))), tag)
        return (None, None, None, None, 'none')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', default='')
    ap.add_argument('--rate', type=float, default=20.0, help='samples per second')
    ap.add_argument('--duration', type=float, default=0.0, help='0 = until Ctrl-C')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--api-topic', default='/api/sport/request')
    ap.add_argument('--odom2d-topic', default='/odom_2d')
    ap.add_argument('--fitness-topic', default='/relocalizer/fitness')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    a = ap.parse_args()

    out = os.path.expanduser(a.out) if a.out else os.path.expanduser(
        '~/.scuba/logs/nav_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)

    rclpy.init()
    n = Logger(a)
    if not n.have_sent:
        print('note: unitree_api not importable - the SENT columns stay empty.')
        print('      source ~/.scuba/sdk_teleop/env.sh to record them.')
    print('logging to %s at %.0f Hz  (Ctrl-C to stop)' % (out, a.rate))

    rows = []
    dt = 1.0 / a.rate
    t0 = time.time()
    f = open(out, 'w')
    f.write(','.join(COLS) + '\n')
    try:
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=dt * 0.5)
            now = time.time()
            if a.duration and now - t0 > a.duration:
                break
            x, y, hd, tilt, frame = n.pose()
            with n.lock:
                cmd, sent, meas, rmse = n.cmd, n.sent, n.meas, n.rmse
            row = [now - t0, x, y, hd, tilt, *cmd, *sent, *meas, rmse, frame]
            rows.append(row)
            f.write(','.join('' if v is None else (v if isinstance(v, str) else '%.4f' % v)
                             for v in row) + '\n')
            if len(rows) % int(max(1, a.rate)) == 0:
                f.flush()
                d = '--' if x is None else '%.2f,%.2f' % (x, y)
                print('\r  %6.1fs  %5d rows  pose %s (%s)  cmd %s  sent %s   '
                      % (now - t0, len(rows), d, frame,
                         '--' if cmd[0] is None else '%+.2f' % cmd[0],
                         '--' if sent[0] is None else '%+.2f' % sent[0]),
                      end='', flush=True)
            time.sleep(max(0.0, dt - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        n.destroy_node()
        rclpy.shutdown()

    print('\n\n%d rows over %.1f s -> %s' % (len(rows), time.time() - t0, out))
    if not rows:
        return
    A = [r for r in rows if r[1] is not None]
    if len(A) > 1:
        dist = sum(math.dist((A[i][1], A[i][2]), (A[i + 1][1], A[i + 1][2]))
                   for i in range(len(A) - 1))
        turn = sum(abs((A[i + 1][3] - A[i][3] + 180) % 360 - 180) for i in range(len(A) - 1))
        print('  travelled %.2f m, turned %.0f deg total' % (dist, turn))
    def col(i):
        return [r[i] for r in rows if r[i] is not None]
    for name, i in (('DWB  fwd', 5), ('SENT fwd', 8), ('measured fwd', 11),
                    ('DWB  turn', 7), ('SENT turn', 10), ('measured turn', 13)):
        c = col(i)
        if c:
            print('  %-14s n=%-5d  min %+.2f  max %+.2f  mean|v| %.3f'
                  % (name, len(c), min(c), max(c), sum(abs(v) for v in c) / len(c)))
    # the failure this whole project keeps hitting: told to move, did not move
    ign = [r for r in rows if r[8] is not None and r[11] is not None
           and abs(r[8]) > 0.05 and abs(r[11]) < 0.3 * abs(r[8])]
    if ign:
        print('  !! %d rows (%.0f%%) where the robot was SENT forward motion but barely moved'
              % (len(ign), 100.0 * len(ign) / len(rows)))
    clip = [r for r in rows if r[5] is not None and r[8] is not None
            and abs(abs(r[5]) - abs(r[8])) > 0.02]
    if clip:
        print('  %d rows (%.0f%%) where the bridge clipped or ramped DWB\'s forward command'
              % (len(clip), 100.0 * len(clip) / len(rows)))


if __name__ == '__main__':
    main()
