#!/usr/bin/env python3
"""Sweep forward velocity and measure what the robot ACTUALLY does.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/vel_sweep.py            # plan only, no motion
    /usr/bin/python3 ~/.scuba/nav/vel_sweep.py --go

Holds each speed for --secs, then zero for --rest, then the next. Distance is
measured from TF map->body, so the table shows COMMANDED against ACHIEVED and
the gait's real threshold falls out of it.

This drives through /cmd_vel and nav_bridge - the same path DWB uses - so the
answer applies to navigation. ~/.scuba/nav/vel_probe.py tests the SDK path
directly instead, one speed at a time.

Changes no mode. Set FSM 501 yourself. It drives BLIND: no obstacle
avoidance, no camera. Check the space ahead.
"""
import argparse
import math
import os
import sys
import time


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--go', action='store_true', help='actually move')
    ap.add_argument('--from', dest='v0', type=float, default=0.05)
    ap.add_argument('--to', dest='v1', type=float, default=0.40)
    ap.add_argument('--step', type=float, default=0.05)
    ap.add_argument('--secs', type=float, default=5.0, help='hold at speed')
    ap.add_argument('--rest', type=float, default=5.0, help='zero between steps')
    ap.add_argument('--alternate', action='store_true',
                    help='reverse each step so the robot stays near one spot')
    ap.add_argument('--rate', type=float, default=30.0, help='publish Hz')
    ap.add_argument('--turn', action='store_true',
                    help='sweep angular velocity instead of forward')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    a = ap.parse_args()

    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit('!! ROS_DISTRO is %r, not foxy. New terminal, then '
                         'source ~/.scuba/lio_ws/env.sh' % d)

    steps = []
    v = a.v0
    while v <= a.v1 + 1e-9:
        steps.append(round(v, 4))
        v += a.step
    unit = 'rad/s' if a.turn else 'm/s'
    total_t = len(steps) * (a.secs + a.rest)
    travel = sum(s * a.secs for s in steps)
    print('%d steps: %s %s' % (len(steps), steps, unit))
    print('  %.1f s at each, %.1f s rest  ->  %.0f s total'
          % (a.secs, a.rest, total_t))
    if not a.turn:
        print('  WORST CASE TRAVEL %.1f m%s - check the space ahead'
              % (travel, ' (alternating, so it stays near one spot)'
                 if a.alternate else ''))
    print('  mode: NOT TOUCHED - only Twist on %s' % a.cmd_topic)

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import tf2_ros

    rclpy.init()
    n = Node('vel_sweep')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    pub = n.create_publisher(Twist, a.cmd_topic, 10)

    def spin(sec):
        t1 = time.time()
        while time.time() - t1 < sec:
            rclpy.spin_once(n, timeout_sec=0.02)
            time.sleep(0.005)

    spin(1.5)

    def pose():
        try:
            t = buf.lookup_transform(a.map_frame, a.base_frame,
                                     rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    if pose() is None:
        print('!! no %s -> %s transform. FAST-LIO and the relocalizer must be '
              'running to measure anything.' % (a.map_frame, a.base_frame))
        n.destroy_node(); rclpy.shutdown()
        return 1

    if not a.go:
        print('\nplan only - nothing published. Add --go to run it.')
        n.destroy_node(); rclpy.shutdown()
        return 0

    for _ in range(60):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(n, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('!! nothing subscribed to %s - nav_bridge is not running, so '
              'the robot cannot move.' % a.cmd_topic)
        n.destroy_node(); rclpy.shutdown()
        return 1

    def send(val, dur):
        """Publish for dur, accumulating unwrapped heading and path length."""
        t = Twist()
        if a.turn:
            t.angular.z = float(val)
        else:
            t.linear.x = float(val)
        t1 = time.time()
        prev = pose()
        turned = 0.0
        dist = 0.0
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

    def stop(dur):
        z = Twist()
        t1 = time.time()
        while time.time() - t1 < dur:
            pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)

    print()
    hdr = ('      cmd', 'achieved', 'ratio', 'moved', 'drift')
    print('  %9s %10s %7s %9s %9s  %s'
          % (hdr[0], hdr[1], hdr[2], hdr[3], hdr[4], 'verdict'))
    rows = []
    sign = 1.0
    try:
        for s in steps:
            p0 = pose()
            turned, pathlen = send(sign * s, a.secs)
            stop(0.4)                      # let it halt before measuring
            spin(0.6)
            p1 = pose()
            stop(max(0.0, a.rest - 1.0))
            if p0 is None or p1 is None:
                print('  %9.2f   no TF' % s)
                continue
            if a.turn:
                moved = abs(turned)             # accumulated, no wrap problem
                ach = moved / a.secs
                drift = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                cmd_disp = math.degrees(s)
                ach_disp = ach
                u1, u2 = 'd/s', 'deg'
            else:
                # straight-line displacement, not path length: a robot
                # that wanders covers ground without getting anywhere
                moved = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                ach = moved / a.secs
                drift = abs(turned)
                cmd_disp, ach_disp = s, ach
                u1, u2 = 'm/s', 'm'
            ratio = ach_disp / cmd_disp if cmd_disp else 0.0
            v = ('WALKS' if ratio > 0.5 else
                 'partial' if ratio > 0.15 else 'IGNORED')
            print('  %8.2f%s %8.3f%s %7.2f %8.3f%s %8.2f  %s'
                  % (cmd_disp, u1, ach_disp, u1, ratio, moved, u2, drift, v))
            rows.append((cmd_disp, ach_disp, ratio))
            if a.alternate:
                sign = -sign
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        stop(0.5)
        print('zero velocity sent')

    walks = [r for r in rows if r[2] > 0.5]
    if walks:
        print()
        print('  lowest command that actually walks: %.2f %s'
              % (min(w[0] for w in walks), unit))
    ign = [r for r in rows if r[2] <= 0.15]
    if ign:
        print('  ignored up to: %.2f %s' % (max(i[0] for i in ign), unit))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
