#!/usr/bin/env python3
"""Does the robot respond to /cmd_vel at all? No camera, no logic.

Sends ONE sustained forward velocity for a fixed time, then stops. That is
all. It changes no mode - set FSM 501 yourself.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/walk_probe.py              # shows what it WOULD do
    /usr/bin/python3 ~/.scuba/livemap/walk_probe.py --go         # 0.22 m/s for 1.0 s (~22 cm)
    /usr/bin/python3 ~/.scuba/livemap/walk_probe.py --go --dur 2.0

CHECK THE SPACE AHEAD FIRST - this drives blind, with no obstacle avoidance
and no camera. At the default it travels about 22 cm.

Watch the nav_bridge terminal while it runs. You should see
    DWB +0.22 +0.00 +0.00 | sent +0.22 +0.00 +0.00
If you see that and the robot does NOT move, the command is reaching the SDK
and being refused or ignored by the robot's current FSM - not a ROS problem.
"""
import argparse
import os
import sys
import time


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--go', action='store_true',
                    help='actually publish; without it nothing is sent')
    ap.add_argument('--vx', type=float, default=0.22,
                    help='m/s. Below ~0.20 this gait ignores it outright')
    ap.add_argument('--wz', type=float, default=0.0, help='rad/s')
    ap.add_argument('--dur', type=float, default=1.0, help='seconds')
    ap.add_argument('--rate', type=float, default=30.0, help='publish Hz')
    ap.add_argument('--topic', default='/cmd_vel')
    a = ap.parse_args()

    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit('!! ROS_DISTRO is %r, not foxy. New terminal, then '
                         'source ~/.scuba/lio_ws/env.sh' % d)

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    n = Node('walk_probe')
    pub = n.create_publisher(Twist, a.topic, 10)
    for _ in range(60):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(n, timeout_sec=0.05)
    subs = pub.get_subscription_count()
    print('subscribers on %s : %d%s'
          % (a.topic, subs, '' if subs else '   <-- nav_bridge is NOT running'))
    print('would send vx %+.2f  wz %+.2f  for %.2f s  (~%.2f m)'
          % (a.vx, a.wz, a.dur, abs(a.vx) * a.dur))
    print('mode: NOT TOUCHED - only Twist is published')

    if not a.go:
        print()
        print('dry run - nothing published. Add --go to actually move.')
        n.destroy_node()
        rclpy.shutdown()
        return 0

    print()
    print('publishing for %.2f s ... watch the nav_bridge terminal' % a.dur)
    t = Twist()
    t.linear.x = float(a.vx)
    t.angular.z = float(a.wz)
    t0 = time.time()
    sent = 0
    try:
        while time.time() - t0 < a.dur:
            pub.publish(t)
            sent += 1
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        print('interrupted')
    finally:
        z = Twist()
        for _ in range(15):
            pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)
        print('sent %d commands, then zero. Stopped.' % sent)
        n.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
