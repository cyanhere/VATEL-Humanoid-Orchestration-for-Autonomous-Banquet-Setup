#!/usr/bin/env python3
"""Does a sub-threshold forward term suppress a turn the robot would otherwise do?

Observed 2026-10-22: DWB held (vx 0.05-0.11, vyaw 0.29) for 22 s and odom
measured turn 0.00, while an earlier run's (vx 0.068, vyaw 0.355) did turn
(meas 0.174). vel_sweep had already shown that 0.25 rad/s turns fine WITH
vx == 0. This probes the pairs directly: the same yaw rate, with and without a
sub-threshold forward term beside it.

Run nav_bridge with the gait snap OFF, or this measures the snap, not the gait:

    ros2 run g1_teleop nav_bridge --ros-args -p enable:=true \
        -p gait_vx:=0.0 -p gait_vyaw:=0.0 -p max_ang_acc:=2.0

    /usr/bin/python3 ~/.scuba/nav/mix_probe.py            # plan only
    /usr/bin/python3 ~/.scuba/nav/mix_probe.py --go

No mode commands: only Twist on /cmd_vel. Robot must already be in FSM501.
It turns in place, so it needs little floor space - but the mixed steps DO
creep forward, so leave a couple of metres ahead.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaws', type=float, nargs='+',
                    default=[0.25, 0.30, 0.35, 0.40])
    ap.add_argument('--vxs', type=float, nargs='+', default=[0.0, 0.10])
    ap.add_argument('--secs', type=float, default=5.0)
    ap.add_argument('--rest', type=float, default=5.0)
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--alternate', action='store_true',
                    help='flip turn direction each step to stay near the start heading')
    ap.add_argument('--go', action='store_true')
    a = ap.parse_args()

    pairs = [(vx, w) for w in a.yaws for vx in a.vxs]
    print('%d steps (%d yaw x %d vx), %.1f s each + %.1f s rest -> %.0f s total'
          % (len(pairs), len(a.yaws), len(a.vxs), a.secs, a.rest,
             len(pairs) * (a.secs + a.rest)))
    for vx, w in pairs:
        print('    vx %.2f  vyaw %.2f%s' % (vx, w, '   <- pure turn' if vx == 0 else ''))
    print('  mode: NOT TOUCHED - only Twist on %s' % a.cmd_topic)
    if not a.go:
        print('\nplan only - nothing published. Add --go to run it.')
        return

    rclpy.init()
    n = Node('mix_probe')
    pub = n.create_publisher(Twist, a.cmd_topic, 10)
    buf = Buffer()
    TransformListener(buf, n)

    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    def pose():
        try:
            tr = buf.lookup_transform(a.map_frame, a.base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        return (t.x, t.y, math.degrees(yaw_of(tr.transform.rotation)))

    if pose() is None:
        print('!! no %s -> %s transform. FAST-LIO and the relocalizer must be up.'
              % (a.map_frame, a.base_frame))
        return
    if pub.get_subscription_count() == 0:
        print('!! nothing subscribed to %s - nav_bridge is not running, so '
              'nothing would reach the robot.' % a.cmd_topic)
        return

    def drive(vx, w, dur):
        """Publish, accumulating unwrapped heading and path length as we go."""
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(w)
        prev = pose()
        turned = dist = 0.0
        t1 = time.time()
        while time.time() - t1 < dur:
            pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
            cur = pose()
            if cur is not None and prev is not None:
                turned += (cur[2] - prev[2] + 180.0) % 360.0 - 180.0
                dist += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            if cur is not None:
                prev = cur
        return turned, dist

    print()
    print('  %6s %6s  %9s %9s  %8s  %s'
          % ('cmd vx', 'vyaw', 'cmd deg/s', 'got deg/s', 'moved m', 'verdict'))
    print('  ' + '-' * 68)
    rows = []
    sign = 1.0
    try:
        for vx, w in pairs:
            turned, dist = drive(vx, sign * w, a.secs)
            if a.alternate:
                sign = -sign
            got = abs(turned) / a.secs
            cmd = math.degrees(w)
            moved = 'TURNED' if got > 0.5 * cmd * 0.3 else 'no turn'
            if got < 1.0:
                moved = 'DEAD'
            rows.append((vx, w, cmd, got, dist, moved))
            print('  %6.2f %6.2f  %9.2f %9.2f  %8.3f  %s'
                  % (vx, w, cmd, got, dist, moved))
            drive(0.0, 0.0, a.rest)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        for _ in range(10):
            pub.publish(Twist())
            rclpy.spin_once(n, timeout_sec=0.01)
            time.sleep(0.02)
        print('zero velocity sent')

    if rows:
        print()
        print('  --- pure (vx=0) vs mixed, same yaw ---')
        by_w = {}
        for vx, w, cmd, got, dist, verdict in rows:
            by_w.setdefault(w, {})[vx] = got
        for w in sorted(by_w):
            d = by_w[w]
            pure = d.get(0.0)
            for vx in sorted(k for k in d if k != 0.0):
                if pure is None:
                    continue
                mixed = d[vx]
                if pure > 1.0:
                    pct = 100.0 * mixed / pure
                    note = ('MIXED SUPPRESSES THE TURN' if pct < 50 else 'comparable')
                else:
                    pct = float('nan')
                    note = 'pure step was itself dead'
                print('   vyaw %.2f: pure %.2f deg/s  vs  vx=%.2f -> %.2f deg/s '
                      ' (%.0f%%)  %s' % (w, pure, vx, mixed, pct, note))


if __name__ == '__main__':
    main()
