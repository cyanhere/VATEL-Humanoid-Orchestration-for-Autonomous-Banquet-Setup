#!/usr/bin/env python3
"""Measure the lidar's mount orientation from gravity. Robot must be STILL.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/reloc/check_mount.py            # /livox/imu
    /usr/bin/python3 ~/.scuba/reloc/check_mount.py --topic X  # some other IMU

A stationary accelerometer measures the reaction to gravity, so its vector
points along "up" expressed in the SENSOR's own axes. That one reading gives
the mount orientation directly - no plane fitting, no assumptions about the
room - and it separates the two cases turn geometry cannot:

    accel ~ (0, 0, +1g)  -> sensor z is UP    (normal mount)
    accel ~ (0, 0, -1g)  -> sensor z is DOWN  (inverted mount)

Needs the Livox driver running (terminal A).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MountCheck(Node):
    def __init__(self, topic):
        super().__init__('check_mount')
        self.a, self.w = [], []
        self.create_subscription(Imu, topic, self.cb, 50)

    def cb(self, m):
        la, av = m.linear_acceleration, m.angular_velocity
        self.a.append([la.x, la.y, la.z])
        self.w.append([av.x, av.y, av.z])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/livox/imu')
    ap.add_argument('--samples', type=int, default=200)
    ap.add_argument('--wait', type=float, default=20.0)
    a_ = ap.parse_args()

    rclpy.init()
    n = MountCheck(a_.topic)
    print('listening on %s - KEEP THE ROBOT COMPLETELY STILL ...' % a_.topic,
          flush=True)
    t0 = time.time()
    while rclpy.ok() and len(n.a) < a_.samples and time.time() - t0 < a_.wait:
        rclpy.spin_once(n, timeout_sec=0.1)

    if len(n.a) < 20:
        print('\n!! only %d samples on %s' % (len(n.a), a_.topic), flush=True)
        print('   Is the Livox driver running? (terminal A: '
              '~/.scuba/lio_ws/run_lio.sh)', flush=True)
        print('   Check with:  ros2 topic list | grep -i imu', flush=True)
        n.destroy_node(); rclpy.shutdown(); sys.exit(1)

    A = np.array(n.a); W = np.array(n.w)
    a = A.mean(axis=0)
    mag = float(np.linalg.norm(a))
    unit = 'g' if mag < 3.0 else 'm/s^2'
    g = 1.0 if unit == 'g' else 9.81
    spin = float(np.linalg.norm(W.mean(axis=0)))

    print('\nsamples      : %d' % len(A))
    print('accel mean   : [%+.4f %+.4f %+.4f]  |a| = %.4f  (units look like %s)'
          % (a[0], a[1], a[2], mag, unit))
    print('accel stddev : [%.4f %.4f %.4f]' % tuple(A.std(axis=0)))
    print('gyro mean    : %.4f rad/s  %s'
          % (spin, '(still - good)' if spin < 0.02 else '<< MOVING, redo stationary'))
    if abs(mag - g) > 0.25 * g:
        print('!! |a| = %.2f, expected ~%.2f - robot may be moving.' % (mag, g))

    u = a / mag
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(u[2])))))
    print('\nup-vector in sensor axes            : [%+.4f %+.4f %+.4f]' % tuple(u))
    print('angle between sensor +z and true UP : %.1f deg' % tilt)
    print()
    if tilt < 30:
        print('  => sensor +z points UP. Normal mount, %.1f deg of tilt.' % tilt)
    elif tilt > 150:
        print('  => sensor +z points DOWN. INVERTED mount, %.1f deg off '
              'straight-down.' % (180 - tilt))
    elif tilt > 90:
        print('  => sensor +z points DOWNWARD-ish, %.1f deg past horizontal. '
              'Effectively inverted.' % (tilt - 90))
    else:
        print('  => sensor +z is %.1f deg off vertical: a large lean, not '
              'inverted.' % tilt)
    print()
    print('  Yaw measured about sensor +z runs BACKWARDS when this angle')
    print('  exceeds 90 deg, and is scaled by about cos(angle) either way.')
    print('  cos(%.1f) = %+.3f' % (tilt, math.cos(math.radians(tilt))))

    roll = math.degrees(math.atan2(float(u[1]), float(u[2])))
    pitch = math.degrees(math.atan2(-float(u[0]),
                                    math.hypot(float(u[1]), float(u[2]))))
    print('\nA gravity-aligned child frame needs a static transform of')
    print('  roll  %+.2f deg' % roll)
    print('  pitch %+.2f deg' % pitch)
    print("  (yaw is unconstrained by gravity - set it from the robot's facing)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
