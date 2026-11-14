#!/usr/bin/env python3
"""Drive one fixed (forward, turn) command straight to the gait, and measure it.

    source ~/.scuba/sdk_teleop/env.sh
    /usr/bin/python3 ~/.scuba/nav/turn_test.py left  0.0  0.4     # turn on the spot
    /usr/bin/python3 ~/.scuba/nav/turn_test.py right 0.0  0.5
    /usr/bin/python3 ~/.scuba/nav/turn_test.py left  0.20 0.3     # arc while walking
    /usr/bin/python3 ~/.scuba/nav/turn_test.py left  0.0  0.4 --no-ramp

  DIRECTION      left | right   (left = anticlockwise seen from above)
  FORWARD_SPEED  m/s, 0 for a static turn
  YAW_SPEED      rad/s, magnitude only - DIRECTION gives the sign

No Nav2, no planner: this publishes SetVelocity on /api/sport/request, the same
path teleop and nav_bridge use, so a result here is about the gait alone.

It also REPORTS WHAT HAPPENED, from FAST-LIO odometry plus IMU gravity:
turn actually achieved, mean turn rate, distance travelled. That is the number
to compare against what Nav2 commands later - "commanded 0.40, achieved 0.31"
is a fact; "it looked like it turned" is not.

Heading is measured about REAL up, not the frame's z: camera_init inherits the
inverted lidar mount, so a yaw read straight from it runs backwards.

Refuses to run while nav_bridge is up: that node streams zero velocity at 20 Hz
whenever /cmd_vel is quiet, which would cancel every command sent here.
"""
import argparse
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from unitree_api.msg import Request, Response

GET_FSM, SET_VEL = 7001, 7105
LIN_ACC, ANG_ACC = 0.35, 0.70           # same ramp as teleop / nav_bridge
CAP_VX, CAP_W = 0.40, 0.80


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_align(u, z=(0.0, 0.0, 1.0)):
    u = np.asarray(u, dtype=np.float64); u = u / np.linalg.norm(u)
    z = np.asarray(z, dtype=np.float64)
    v = np.cross(u, z); s = np.linalg.norm(v); c = float(u @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + V + V @ V * ((1 - c) / s ** 2)


def headings(samples, up_body):
    """Unwrapped heading about real up, and horizontal distance, per sample.

    samples: [(t, R_camera_init_body, p_camera_init), ...]
    """
    R0, p0 = samples[0][1], samples[0][2]
    up_ci = R0 @ up_body                       # gravity expressed in camera_init
    up_ci /= np.linalg.norm(up_ci)
    L = R_align(up_ci)                         # camera_init -> level
    ex = np.array([1.0, 0.0, 0.0])
    hs, ds, ts = [], [], []
    prev = None
    for t, R, p in samples:
        v = L @ (R @ ex)                       # body forward, in the level frame
        h = math.atan2(v[1], v[0])
        if prev is not None:                   # unwrap
            h = prev + (h - prev + math.pi) % (2 * math.pi) - math.pi
        prev = h
        d = L @ (p - p0)
        hs.append(h); ds.append(math.hypot(d[0], d[1])); ts.append(t)
    return np.array(ts), np.array(hs), np.array(ds)


class Driver(Node):
    def __init__(self, a):
        super().__init__('turn_test')
        self.a = a
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Response, '/api/sport/response', self._resp_cb, 10)
        self.create_subscription(Imu, a.imu_topic, self._imu_cb, 50)
        self.create_subscription(Odometry, a.odom_topic, self._odom_cb, 50)
        self._lock = threading.Lock()
        self._resp = {}
        self.imu = []
        self.odom = []
        self.recording = False

    def _resp_cb(self, m):
        with self._lock:
            self._resp[m.header.identity.id] = m

    def _imu_cb(self, m):
        a = m.linear_acceleration
        v = np.array([a.x, a.y, a.z])
        n = np.linalg.norm(v)
        if n > 1e-6:
            with self._lock:
                self.imu.append(v / n)
                if len(self.imu) > 400:
                    self.imu.pop(0)

    def _odom_cb(self, m):
        if not self.recording:
            return
        q, p = m.pose.pose.orientation, m.pose.pose.position
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self._lock:
            self.odom.append((t, quat_to_R(q.x, q.y, q.z, q.w),
                              np.array([p.x, p.y, p.z])))

    def req(self, api_id, param, rid=0):
        r = Request()
        r.header.identity.api_id = api_id
        r.header.identity.id = rid
        r.header.lease.id = 0
        r.header.policy.priority = 0
        r.header.policy.noreply = False
        r.parameter = json.dumps(param)
        self.pub.publish(r)

    def fsm(self, timeout=2.0):
        rid = time.time_ns()
        self.req(GET_FSM, {}, rid)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._lock:
                m = self._resp.pop(rid, None)
            if m is not None:
                try:
                    d = json.loads(m.data)
                    return d.get('data') if isinstance(d, dict) else d
                except (ValueError, TypeError):
                    return None
        return None

    def vel(self, vx, vy, w):
        self.req(SET_VEL, {'velocity': [vx, vy, w], 'duration': self.a.duration})


def step(cur, tgt, lim):
    return tgt if abs(tgt - cur) <= lim else cur + (lim if tgt > cur else -lim)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('direction', help='left | right (left = anticlockwise from above)')
    ap.add_argument('forward', type=float, help='forward speed, m/s (0 = static turn)')
    ap.add_argument('yaw', type=float, help='turn rate magnitude, rad/s')
    ap.add_argument('--secs', type=float, default=3.0, help='hold time at the command')
    ap.add_argument('--no-ramp', action='store_true',
                    help='step straight to the command, like DWB does')
    ap.add_argument('--duration', type=float, default=0.5, help='SetVelocity duration field')
    ap.add_argument('--rate', type=float, default=20.0, help='command rate, Hz')
    ap.add_argument('--csv', default='', help='write the per-sample log here')
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--odom-topic', default='/Odometry')
    ap.add_argument('--force', action='store_true', help='skip the FSM check')
    a = ap.parse_args()

    d = a.direction.lower()
    if d in ('left', 'l', 'ccw', 'anticlockwise'):
        sign, dname = +1.0, 'LEFT (anticlockwise)'
    elif d in ('right', 'r', 'cw', 'clockwise'):
        sign, dname = -1.0, 'RIGHT (clockwise)'
    else:
        sys.exit('!! direction must be left or right, got %r' % a.direction)
    if abs(a.forward) > CAP_VX:
        sys.exit('!! forward %.2f exceeds the cap %.2f' % (a.forward, CAP_VX))
    if abs(a.yaw) > CAP_W:
        sys.exit('!! yaw %.2f exceeds the cap %.2f' % (a.yaw, CAP_W))
    tgt = [a.forward, 0.0, sign * abs(a.yaw)]

    rclpy.init()
    n = Driver(a)

    def refuse(msg):
        n.destroy_node(); rclpy.shutdown(); sys.exit(msg)

    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)
    if 'g1_nav_bridge' in n.get_node_names():
        refuse('!! nav_bridge is running - it streams zero velocity and would cancel\n'
               '   every command sent here. Ctrl-C it (terminal D) and re-run.')
    f = n.fsm()
    print('FSM: %s' % f)
    if f not in (500, 801) and not a.force:
        refuse('!! robot is not in a walk FSM (500/801) - set it with the remote.')
    with n._lock:
        nimu = len(n.imu)
        up = np.mean(n.imu, axis=0) if nimu >= 20 else None
    if up is None:
        print('   (no IMU on %s: commanding anyway, but nothing will be measured)' % a.imu_topic)
    else:
        up = up / np.linalg.norm(up)

    print('commanding %s   forward %.2f m/s   turn %+.2f rad/s   for %.1f s%s'
          % (dname, tgt[0], tgt[2], a.secs, '  (no ramp)' if a.no_ramp else ''))
    print('WATCH THE ROBOT.')
    dt = 1.0 / a.rate
    cur = [0.0, 0.0, 0.0]
    log = []
    n.recording = True
    try:
        t_start = time.time()
        hold_end = None
        while True:
            if a.no_ramp:
                cur = list(tgt)
            else:
                cur = [step(c, t, l) for c, t, l in
                       zip(cur, tgt, (LIN_ACC * dt, LIN_ACC * dt, ANG_ACC * dt))]
            reached = all(abs(c - t) < 1e-9 for c, t in zip(cur, tgt))
            if reached and hold_end is None:
                hold_end = time.time() + a.secs
                t_motion = time.time()
            if hold_end is not None and time.time() >= hold_end:
                break
            n.vel(*cur)
            log.append((time.time() - t_start, cur[0], cur[1], cur[2]))
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(dt)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        try:
            for _ in range(40):
                if all(abs(c) < 1e-9 for c in cur):
                    break
                cur = [step(c, 0.0, l) for c, l in
                       zip(cur, (2 * LIN_ACC * dt, 2 * LIN_ACC * dt, 2 * ANG_ACC * dt))]
                n.vel(*cur)
                time.sleep(dt)
        except BaseException:
            pass
        for _ in range(3):
            try:
                n.vel(0.0, 0.0, 0.0)
            except BaseException:
                pass
        time.sleep(0.4)                      # let the last odometry arrive
        for _ in range(20):
            rclpy.spin_once(n, timeout_sec=0.02)
        n.recording = False

    with n._lock:
        samples = list(n.odom)
    print()
    if up is None or len(samples) < 5:
        print('no measurement: %d odometry samples (is FAST-LIO running?)' % len(samples))
    else:
        ts, hs, ds = headings(samples, up)
        dur = ts[-1] - ts[0]
        turned = math.degrees(hs[-1] - hs[0])
        rate = math.radians(turned) / dur if dur > 0 else 0.0
        want = math.degrees(tgt[2]) * a.secs
        print('%-22s %18s   %s' % ('', 'COMMANDED', 'ACHIEVED'))
        print('%-22s %14.2f rad/s   %.2f rad/s  (%.0f%%)'
              % ('turn rate', tgt[2], rate, 100 * rate / tgt[2] if tgt[2] else 0))
        print('%-22s %14.1f deg     %.1f deg' % ('turn over the hold', want, turned))
        print('%-22s %14.2f m/s     %.2f m/s'
              % ('forward speed', tgt[0], ds[-1] / dur if dur > 0 else 0))
        print('%-22s %14s       %.2f m' % ('distance moved', '-', ds[-1]))
        print('%-22s %14s       %.1f s, %d samples' % ('window', '-', dur, len(samples)))
        if tgt[2] and abs(rate) < 0.15 * abs(tgt[2]):
            print('\n-> the gait IGNORED this turn command.')
        elif tgt[2] and rate * tgt[2] < 0:
            print('\n-> it turned the WRONG WAY.')
        elif tgt[2]:
            print('\n-> it turned.')
        if tgt[0] == 0 and ds[-1] > 0.25:
            print('   note: it drifted %.2f m while turning - not a clean pivot.' % ds[-1])

    if a.csv and log:
        with open(a.csv, 'w') as f:
            f.write('t,cmd_vx,cmd_vy,cmd_wz\n')
            for r in log:
                f.write('%.3f,%.4f,%.4f,%.4f\n' % r)
        print('\nwrote %s (%d commands)' % (a.csv, len(log)))

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
