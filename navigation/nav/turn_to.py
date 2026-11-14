#!/usr/bin/env python3
"""Turn to an absolute heading. Nothing else commands the robot.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/turn_to.py 90          # face +90 deg in map
    /usr/bin/python3 ~/.scuba/nav/turn_to.py 90 --dry-run

No Nav2, no planner, no DWB - this publishes Twist on /cmd_vel itself, so the
only yaw command in the system is this one. Use it to check the stepping in
isolation when a goal run misbehaves.

It steps ONE direction at a fixed size and stops the moment the signed
heading error changes sign, which is the whole concept: turn-per-command
varies 40-60% with gait phase on this robot, so sizing a correction is
guessing; crossing zero only needs the direction to be right.

Changes no mode. Set FSM 501 yourself.
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
    ap.add_argument('yaw_deg', type=float, help='target heading in the map frame')
    ap.add_argument('--tol', type=float, default=4.0,
                    help='deg; the smallest reliable turn measured is ~4')
    # Step duration scales with the error: measured 10.21 deg/s, so a 90 deg
    # error no longer needs 20 identical steps. --step forces a fixed one.
    ap.add_argument('--step', type=float, default=0.0,
                    help='s per step; 0 = adaptive from --rate')
    ap.add_argument('--rate', type=float, default=10.2,
                    help='deg/s actually achieved (measured 10.21)')
    ap.add_argument('--min-step', type=float, default=0.35,
                    help='s; shorter commands may not move the robot at all')
    ap.add_argument('--max-step', type=float, default=1.2,
                    help='s; the rate falls off beyond this')
    ap.add_argument('--w', type=float, default=0.40, help='rad/s')
    ap.add_argument('--settle', type=float, default=0.8,
                    help='s to let the robot stop before believing TF')
    ap.add_argument('--max-steps', type=int, default=20)
    ap.add_argument('--no-flip', action='store_true',
                    help='never reverse: one direction only')
    ap.add_argument('--dry-run', action='store_true',
                    help='read the heading, publish nothing')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    a = ap.parse_args()

    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit('!! ROS_DISTRO is %r, not foxy. New terminal, then '
                         'source ~/.scuba/lio_ws/env.sh' % d)

    sys.path.insert(0, os.path.expanduser('~/.scuba/livemap'))
    from edge_servo import step_to_zero, adaptive_step

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    import tf2_ros

    rclpy.init()
    n = Node('turn_to')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    pub = n.create_publisher(Twist, a.cmd_topic, 10)

    def spin_for(sec):
        t1 = time.time()
        while time.time() - t1 < sec:
            rclpy.spin_once(n, timeout_sec=0.02)
            time.sleep(0.005)

    spin_for(1.5)                       # let TF fill

    def heading():
        try:
            t = buf.lookup_transform(a.map_frame, a.base_frame,
                                     rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def err():
        spin_for(0.15)
        h = heading()
        if h is None:
            return None
        return (a.yaw_deg - h + 180.0) % 360.0 - 180.0

    e0 = err()
    if e0 is None:
        print('!! no %s -> %s transform. Is FAST-LIO and the relocalizer up?'
              % (a.map_frame, a.base_frame))
        n.destroy_node(); rclpy.shutdown()
        return 1
    print('heading now %+.2f deg, want %+.2f -> error %+.2f'
          % (heading(), a.yaw_deg, e0))
    sched = (a.step if a.step > 0 else
             adaptive_step(a.rate, a.min_step, a.max_step))
    if a.step > 0:
        print('stepping a fixed %.2f s at %.2f rad/s, tol %.1f deg'
              % (a.step, a.w, a.tol))
    else:
        print('stepping adaptively (|err|/%.1f deg/s, clamped %.2f-%.2f s) '
              'at %.2f rad/s, tol %.1f deg'
              % (a.rate, a.min_step, a.max_step, a.w, a.tol))
        print('  this error wants %.2f s' % sched(e0))
    if a.dry_run:
        print('[dry run] nothing published')
        n.destroy_node(); rclpy.shutdown()
        return 0

    subs = pub.get_subscription_count()
    if subs == 0:
        for _ in range(40):
            if pub.get_subscription_count():
                break
            rclpy.spin_once(n, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('!! nothing subscribed to %s - nav_bridge is not running, so '
              'the robot cannot move' % a.cmd_topic)
        n.destroy_node(); rclpy.shutdown()
        return 1

    def stop():
        z = Twist()
        for _ in range(15):
            pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)

    def turn(dirn, dur):
        t = Twist()
        t.angular.z = float(dirn * a.w)
        t1 = time.time()
        while time.time() - t1 < dur:
            pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)
        stop()

    try:
        ok = step_to_zero(err, turn, tol=a.tol, step_dur=sched,
                          settle=a.settle, max_steps=a.max_steps,
                          log=print, wait=spin_for,
                          autoflip=not a.no_flip)
    except KeyboardInterrupt:
        ok = False
        print('\ninterrupted')
    finally:
        stop()
    e1 = err()
    print('%s: %+.2f -> %+.2f deg   (heading %+.2f, wanted %+.2f)'
          % ('REACHED' if ok else 'NOT CONVERGED', e0,
             e1 if e1 is not None else float('nan'),
             heading(), a.yaw_deg))
    print('zero velocity sent')
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
