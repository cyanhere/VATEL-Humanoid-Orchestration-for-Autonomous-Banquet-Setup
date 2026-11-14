#!/usr/bin/env python3
"""Velocity feedback for DWB: the robot's own 2D velocity, gravity-level.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/odom_2d.py        # started by nav_dwb.launch.py

WHY THIS EXISTS
DWB builds each cycle's velocity window around the robot's CURRENT velocity:
    max = current + acc_lim * (1 / controller_frequency)
FAST-LIO's /Odometry never fills the twist, so DWB always saw 0 and could
never command more than 0 + 0.5 * 0.1 = 0.05 m/s forward or 0.1 rad/s turning
- a third of the configured maximum.

WHY IT CANNOT JUST RE-EXPRESS /Odometry
/Odometry is camera_init -> body, and on this G1 both frames are the Mid-360's
own axes, which point ~150 deg away from up (the lidar is mounted inverted). A
yaw rate measured about that z-axis runs backwards. So velocities are rotated
into the level map frame first, then expressed as the robot's forward / left /
anticlockwise-about-real-up - the convention DWB and nav_bridge both use.

    forward, left : odometry displacement, rotated by map->camera_init, then
                    projected on the robot's heading (body +x on the ground)
    yaw rate      : body angular velocity projected on real 'up'

Differentiating FAST-LIO (smooth, 10 Hz) rather than map->body avoids the
small steps the relocalizer adds once a second.
"""
import math

import numpy as np


# ---------------------------------------------------------------- maths
def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
                (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s,
                (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s,
            (R[1, 0] - R[0, 1]) / s)


def rotvec(R):
    """Axis * angle of a rotation matrix (log map), stable near 0."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    a = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-6:
        return 0.5 * v
    return v * (a / (2.0 * math.sin(a)))


def heading(R):
    """Robot heading: body +x projected onto the level ground plane."""
    return math.atan2(R[1, 0], R[0, 0])


def velocity_2d(R_mc, R_cb0, p_c0, R_cb1, p_c1, dt):
    """(forward, left, yaw_rate) between two odometry samples.

    R_mc      map <- camera_init rotation (map is level)
    R_cb, p_c camera_init <- body pose at the two sample times
    """
    v_map = R_mc @ ((p_c1 - p_c0) / dt)
    R_mb = R_mc @ R_cb1
    h = heading(R_mb)
    c, s = math.cos(h), math.sin(h)
    fwd = c * v_map[0] + s * v_map[1]
    left = -s * v_map[0] + c * v_map[1]
    w_body = rotvec(R_cb0.T @ R_cb1) / dt           # body-frame angular velocity
    up_body = R_mb.T @ np.array([0.0, 0.0, 1.0])     # real 'up' in body axes
    return fwd, left, float(w_body @ up_body)


# ---------------------------------------------------------------- node
def main():
    import argparse
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    import tf2_ros

    ap = argparse.ArgumentParser()
    ap.add_argument('--in', dest='src', default='/Odometry')
    ap.add_argument('--out', default='/odom_2d')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--alpha', type=float, default=0.5,
                    help='low-pass weight of the newest sample (1 = no filter)')
    a, ros_args = ap.parse_known_args()

    class Odom2D(Node):
        def __init__(self):
            super().__init__('odom_2d')
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            self.pub = self.create_publisher(Odometry, a.out, 10)
            self.create_subscription(Odometry, a.src, self.on_odom, 20)
            self.prev = None
            self.v = np.zeros(3)
            self.warned = False

        def on_odom(self, m):
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            q, p = m.pose.pose.orientation, m.pose.pose.position
            R_cb = quat_to_R(q.x, q.y, q.z, q.w)
            p_c = np.array([p.x, p.y, p.z])
            try:
                tf = self.buf.lookup_transform(a.map_frame, a.odom_frame,
                                               rclpy.time.Time())
            except Exception:
                if not self.warned:
                    self.get_logger().warn(
                        'no %s -> %s yet (is the relocalizer running?) - holding'
                        % (a.map_frame, a.odom_frame))
                    self.warned = True
                self.prev = (t, R_cb, p_c)
                return
            self.warned = False
            r, tr = tf.transform.rotation, tf.transform.translation
            R_mc = quat_to_R(r.x, r.y, r.z, r.w)
            p_mc = np.array([tr.x, tr.y, tr.z])
            if self.prev is not None:
                t0, R0, p0 = self.prev
                dt = t - t0
                if 1e-3 < dt < 0.5:
                    raw = np.array(velocity_2d(R_mc, R0, p0, R_cb, p_c, dt))
                    self.v = a.alpha * raw + (1.0 - a.alpha) * self.v
                elif dt >= 0.5:
                    self.v[:] = 0.0                     # gap in odometry: say stopped
            self.prev = (t, R_cb, p_c)

            R_mb = R_mc @ R_cb
            out = Odometry()
            out.header.stamp = m.header.stamp
            out.header.frame_id = a.map_frame
            out.child_frame_id = m.child_frame_id or 'body'
            pm = R_mc @ p_c + p_mc
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, pm)
            qx, qy, qz, qw = R_to_quat(R_mb)
            o = out.pose.pose.orientation
            o.x, o.y, o.z, o.w = float(qx), float(qy), float(qz), float(qw)
            out.twist.twist.linear.x = float(self.v[0])
            out.twist.twist.linear.y = float(self.v[1])
            out.twist.twist.angular.z = float(self.v[2])
            self.pub.publish(out)

    rclpy.init(args=ros_args)
    node = Odom2D()
    node.get_logger().info('%s -> %s  (forward, left, yaw-rate about real up)'
                           % (a.src, a.out))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
