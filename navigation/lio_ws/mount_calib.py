#!/usr/bin/env python3
"""Measure the lidar's mount orientation from gravity, at every startup.

    measure, print, save (run_lio.sh does this while the robot is still):
        /usr/bin/python3 mount_calib.py --save ~/.scuba/lio_ws/mount_calib.json

    then publish the static body -> base_link from that file:
        /usr/bin/python3 mount_calib.py --publish --from-file ~/.scuba/lio_ws/mount_calib.json

WHY
The Mid-360 is mounted at whatever angle the bracket holds it - inverted and
leaning, and that lean CHANGES when the mount is adjusted. A stationary
accelerometer points along "up" in the sensor's own axes, so one reading at
startup gives the mount orientation with no hand-measured constant anywhere.

base_link is the gravity-levelled version of FAST-LIO's body frame: same
origin, z along real up, x = body +x projected onto the ground (the robot's
forward). With a level mount this is the identity; with this robot's mount it
is not, which is why publishing it as identity was wrong.

Nothing else needs the number: the relocalizer levels its seed from the same
IMU, and the path viewer measures frame-vs-gravity live. This exists so the
angle is recorded, checked for change, and exported as a usable frame.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np


def quat_from_R(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s]
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s]
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s]


def body_to_base(up_body):
    """Rotation of the levelled child frame, expressed in the body frame.

    z_base = gravity up; x_base = body +x flattened onto the ground plane
    (the robot's forward); y completes a right-handed frame. Returns the
    matrix whose COLUMNS are those axes, i.e. the body->base_link rotation.
    """
    z = np.asarray(up_body, dtype=np.float64)
    z = z / np.linalg.norm(z)
    x = np.array([1.0, 0.0, 0.0]) - z[0] * z          # e_x minus its z component
    if np.linalg.norm(x) < 1e-6:                      # body +x points along gravity
        x = np.array([0.0, 1.0, 0.0]) - z[1] * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rpy_from_R(R):
    """roll, pitch, yaw (radians) in the ZYX order static_transform_publisher uses."""
    sy = -max(-1.0, min(1.0, float(R[2, 0])))
    pitch = math.asin(sy)
    if abs(R[2, 0]) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                             # gimbal lock
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def measure(a):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Reader(Node):
        def __init__(self):
            super().__init__('mount_calib')
            self.acc, self.gyr = [], []
            self.create_subscription(Imu, a.imu_topic, self.cb, 100)

        def cb(self, m):
            la, av = m.linear_acceleration, m.angular_velocity
            self.acc.append([la.x, la.y, la.z])
            self.gyr.append([av.x, av.y, av.z])

    rclpy.init()
    n = Reader()
    t0 = time.time()
    while rclpy.ok() and len(n.acc) < a.samples and time.time() - t0 < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
    acc, gyr = np.array(n.acc), np.array(n.gyr)
    n.destroy_node()
    rclpy.shutdown()

    if len(acc) < 20:
        print('!! only %d IMU samples on %s in %.0f s - is the Livox driver running?'
              % (len(acc), a.imu_topic, a.timeout))
        return None
    spin = float(np.linalg.norm(gyr.mean(axis=0)))
    mean = acc.mean(axis=0)
    mag = float(np.linalg.norm(mean))
    if mag < 1e-6:
        print('!! accelerometer reads zero')
        return None
    up = mean / mag
    unit = 'g' if mag < 3.0 else 'm/s^2'
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    R = body_to_base(up)
    roll, pitch, yaw = rpy_from_R(R)
    out = dict(time=time.time(), stamp=time.strftime('%Y-%m-%d %H:%M:%S'),
               samples=int(len(acc)), gyro_mean=round(spin, 5), accel_mag=round(mag, 4),
               accel_units=unit, up_body=[round(float(v), 6) for v in up],
               tilt_deg=round(tilt, 2), inverted=bool(tilt > 90),
               quat_body_base=[round(float(v), 6) for v in quat_from_R(R)],
               rpy_body_base_deg=[round(math.degrees(v), 3) for v in (roll, pitch, yaw)])
    out['moving'] = bool(spin > a.still_gyro)
    return out


def report(cur, prev, a):
    print('mount: sensor +z is %.1f deg from up  (%s, %.1f deg off %s)'
          % (cur['tilt_deg'], 'INVERTED' if cur['inverted'] else 'upright',
             180 - cur['tilt_deg'] if cur['inverted'] else cur['tilt_deg'],
             'straight-down' if cur['inverted'] else 'vertical'))
    print('       up in sensor axes [%+.4f %+.4f %+.4f]   %d samples, gyro %.4f rad/s%s'
          % (tuple(cur['up_body']) + (cur['samples'], cur['gyro_mean'],
             '  << MOVING: re-run stationary' if cur['moving'] else '')))
    print('       body->base_link  roll %+.2f  pitch %+.2f  yaw %+.2f deg'
          % tuple(cur['rpy_body_base_deg']))
    if prev:
        d = math.degrees(math.acos(max(-1.0, min(1.0,
            float(np.dot(cur['up_body'], prev['up_body']))))))
        if d > a.warn_deg:
            print('       >> MOUNT MOVED %.1f deg since %s. The prior map is still '
                  'valid (it is gravity-aligned), but re-check localisation.'
                  % (d, prev.get('stamp', 'last run')))
        else:
            print('       unchanged since %s (%.1f deg)' % (prev.get('stamp', 'last run'), d))


def publish(a):
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    try:
        d = json.load(open(os.path.expanduser(a.from_file)))
    except Exception as e:
        sys.exit('!! cannot read %s (%s) - run the measuring mode first'
                 % (a.from_file, e))
    q = d['quat_body_base']
    rclpy.init()
    n = Node('mount_tf')
    b = StaticTransformBroadcaster(n)
    t = TransformStamped()
    t.header.stamp = n.get_clock().now().to_msg()
    t.header.frame_id = a.parent
    t.child_frame_id = a.child
    t.transform.rotation.x, t.transform.rotation.y = float(q[0]), float(q[1])
    t.transform.rotation.z, t.transform.rotation.w = float(q[2]), float(q[3])
    b.sendTransform(t)
    n.get_logger().info('%s -> %s levelled from gravity: tilt %.1f deg, '
                        'roll %+.2f pitch %+.2f (measured %s)'
                        % (a.parent, a.child, d['tilt_deg'], d['rpy_body_base_deg'][0],
                           d['rpy_body_base_deg'][1], d.get('stamp', '?')))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--imu-topic', default='/livox/imu')
    ap.add_argument('--samples', type=int, default=200)
    ap.add_argument('--timeout', type=float, default=15.0)
    ap.add_argument('--still-gyro', type=float, default=0.03,
                    help='rad/s above which the robot counts as moving')
    ap.add_argument('--warn-deg', type=float, default=3.0,
                    help='warn if the mount moved more than this since last run')
    ap.add_argument('--save', default='', help='write the measurement to this JSON')
    ap.add_argument('--publish', action='store_true',
                    help='broadcast the static parent->child transform and spin')
    ap.add_argument('--from-file', default='~/.scuba/lio_ws/mount_calib.json')
    ap.add_argument('--parent', default='body')
    ap.add_argument('--child', default='base_link')
    a = ap.parse_args()

    if a.publish:
        return publish(a)

    prev = None
    path = os.path.expanduser(a.save or a.from_file)
    if os.path.exists(path):
        try:
            prev = json.load(open(path))
        except Exception:
            prev = None
    cur = measure(a)
    if cur is None:
        sys.exit(1)
    report(cur, prev, a)
    if a.save:
        p = os.path.expanduser(a.save)
        with open(p, 'w') as f:
            json.dump(cur, f, indent=1)
        print('       saved %s' % p)
    sys.exit(2 if cur['moving'] else 0)


if __name__ == '__main__':
    main()
