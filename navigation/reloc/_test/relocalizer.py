#!/usr/bin/env python3
"""map -> odom relocalization by scan registration against a prebuilt cloud.

    FAST-LIO  /Odometry + /cloud_registered   (odom frame, drifting)
                        │
                  ICP vs ilounge_02_loc      (map frame, fixed)
                        │
                  TF map -> odom  +  /relocalization/fitness

Seeding (option 1 + 3):
  * a fixed seed pose from parameters (default), and
  * /initialpose (PoseWithCovarianceStamped) to override at runtime.

IMPORTANT - which map:
  Register against the NON-de-tilted map (ilounge_02_loc). Live FAST-LIO output
  carries the same ~1 deg mount/gravity-init error the de-tilt removed, so raw
  scans align with the raw map directly. Registering against the de-tilted
  planning map would need R applied to every scan.
  Convert to the planning frame ONCE at the boundary, using
  ilounge_02_dt_transform.npz.

Foxy-safe: no read_points_numpy, no Open3D. numpy + scipy.cKDTree only.
"""
import os
import sys

# The user site-packages carries numpy 1.24.4, which shadows the system 1.17.4.
# System scipy 1.3.3 still uses np.typeDict (removed in numpy 1.21), so that
# pairing raises AttributeError on "from scipy.spatial import cKDTree".
# Re-exec ignoring the user site so we get the numpy/scipy pair Ubuntu ships.
if not os.environ.get("PYTHONNOUSERSITE"):
    os.environ["PYTHONNOUSERSITE"] = "1"
    os.execv(sys.executable, [sys.executable] + sys.argv)

import math
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial import cKDTree

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster


# ---------------------------------------------------------------- helpers
def pc2_xyz(msg):
    """PointCloud2 -> (N,3). Raw buffer parse; read_points_numpy is not on Foxy."""
    off = {f.name: f.offset for f in msg.fields}
    for k in ('x', 'y', 'z'):
        if k not in off:
            return np.empty((0, 3))
    dt = np.dtype(np.float32).newbyteorder('>' if msg.is_bigendian else '<')
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8)[:n * msg.point_step]
    if buf.size < n * msg.point_step:
        return np.empty((0, 3))
    buf = buf.reshape(n, msg.point_step)
    cols = [buf[:, off[k]:off[k] + 4].copy().view(dt).ravel() for k in ('x', 'y', 'z')]
    p = np.stack(cols, axis=1)
    return p[np.isfinite(p).all(axis=1)]


def voxel_downsample(P, v):
    if len(P) == 0:
        return P
    key = np.floor(P / v).astype(np.int64)
    _, idx = np.unique(key, axis=0, return_index=True)
    return P[idx]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def yaw_to_R(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def R_align(u, z=(0.0, 0.0, 1.0)):
    """Rotation taking unit vector u onto z."""
    u = np.asarray(u, dtype=np.float64); u = u / np.linalg.norm(u)
    z = np.asarray(z, dtype=np.float64)
    v = np.cross(u, z); s = np.linalg.norm(v); c = float(u @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + V + V @ V * ((1 - c) / s ** 2)


def leveled_R(up_body, heading):
    """map->body rotation from gravity plus a heading.

    up_body is 'up' measured by the IMU in the body frame; it pins roll and
    pitch. heading is where the robot's +x points on the map; it pins yaw.
    Validated with this file's icp() on simulated inverted-mount scans:
    yaw-only seeds converged upside down 0/10, leveled seeds 10/10 (1 cm).
    """
    Rl = R_align(up_body)
    v = Rl @ np.array([1.0, 0.0, 0.0])
    return yaw_to_R(heading - math.atan2(v[1], v[0])) @ Rl


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


def compute_normals(M, tree, k=12, cache=None):
    """Per-point surface normals by PCA over k nearest neighbours.

    The smallest-eigenvalue eigenvector of the local covariance is the plane
    normal. Batched over all points, then cached - it only changes if the map
    changes.
    """
    if cache and os.path.isfile(cache):
        N = np.load(cache)
        if len(N) == len(M):
            return N
    _, idx = tree.query(M, k=k)
    nb = M[idx]                              # (N,k,3)
    nb = nb - nb.mean(axis=1, keepdims=True)
    C = np.einsum('nki,nkj->nij', nb, nb) / k
    w, v = np.linalg.eigh(C)                 # ascending; [:,:,0] = smallest
    N = v[:, :, 0]
    N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
    if cache:
        try:
            np.save(cache, N)
        except Exception:
            pass
    return N


def _icp_plane(src, tree, normals, T0, iters, max_corr, tol):
    """Point-to-PLANE ICP.

    Minimises only the component of the residual along the surface normal, so
    sliding along a wall costs nothing - which is correct, because the scan and
    the map sampled the same wall at different points. Point-to-point counts
    that sampling offset as error and cannot remove it.

    Small-angle linearisation (Low 2004): for each correspondence
        [ (p x n) | n ] . [r ; t] = -(p - q).n
    stacked into a 6-column least-squares system.
    """
    T = T0.copy()
    prev, rmse, ratio = None, float('inf'), 0.0
    for _ in range(iters):
        P = src @ T[:3, :3].T + T[:3, 3]
        d, j = tree.query(P, k=1, distance_upper_bound=max_corr)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            break
        ratio = float(ok.mean())
        p, q = P[ok], tree.data[j[ok]]
        n = normals[j[ok]]
        r = np.einsum('ij,ij->i', p - q, n)          # signed point-plane distance
        A = np.hstack([np.cross(p, n), n])           # (M,6)
        try:
            x, *_ = np.linalg.lstsq(A, -r, rcond=None)
        except np.linalg.LinAlgError:
            break
        a, b, c = x[0], x[1], x[2]
        dR = np.array([[1.0, -c, b], [c, 1.0, -a], [-b, a, 1.0]])
        U, _, Vt = np.linalg.svd(dR)                 # re-orthonormalise
        dR = U @ Vt
        dT = np.eye(4); dT[:3, :3], dT[:3, 3] = dR, x[3:]
        T = dT @ T
        rmse = float(np.sqrt((r ** 2).mean()))       # plane-metric rmse
        if prev is not None and abs(prev - rmse) < tol:
            break
        prev = rmse
    return T, rmse, ratio


def icp(src, tree, T0, iters=20, max_corr=1.0, tol=1e-4, gates=None,
        normals=None, plane_all=False):
    """Coarse-to-fine point-to-point ICP (Kabsch).

    A single tight gate gives good accuracy but NO recovery basin: once the pose
    drifts further than the gate, zero correspondences are found and ICP is
    stuck (observed: 21 s of rmse=inf, 0% inliers). Sweeping wide -> narrow
    keeps the recovery basin of a loose gate and the accuracy of a tight one.
    """
    if gates is None:
        gates = [max_corr]
    T = T0.copy()
    rmse, ratio = float('inf'), 0.0
    # coarse passes. Default point-to-point: robust when the pose is far off,
    # where the small-angle linearisation of point-to-plane breaks down and a
    # wide gate can match a floor point to a wall normal.
    # plane_all=True uses point-to-plane throughout - usually fine during normal
    # tracking (small per-cycle drift), riskier during recovery.
    for g in gates[:-1]:
        if plane_all and normals is not None:
            T, rmse, ratio = _icp_plane(src, tree, normals, T, iters, g, tol)
        else:
            T, rmse, ratio = _icp_one(src, tree, T, iters, g, tol)
    # fine pass: point-to-plane when normals are available
    if normals is not None:
        T, rmse, ratio = _icp_plane(src, tree, normals, T, iters, gates[-1], tol)
    else:
        T, rmse, ratio = _icp_one(src, tree, T, iters, gates[-1], tol)
    return T, rmse, ratio


def _icp_one(src, tree, T0, iters, max_corr, tol):
    T = T0.copy()
    prev = None
    rmse = float('inf')
    ratio = 0.0
    for _ in range(iters):
        P = src @ T[:3, :3].T + T[:3, 3]
        d, j = tree.query(P, k=1, distance_upper_bound=max_corr)
        ok = np.isfinite(d)
        if ok.sum() < 30:
            break
        ratio = float(ok.mean())
        A, B = P[ok], tree.data[j[ok]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, _, Vt = np.linalg.svd(H)
        Rk = Vt.T @ U.T
        if np.linalg.det(Rk) < 0:
            Vt[2] *= -1
            Rk = Vt.T @ U.T
        tk = cb - Rk @ ca
        dT = np.eye(4)
        dT[:3, :3], dT[:3, 3] = Rk, tk
        T = dT @ T
        rmse = float(np.sqrt((d[ok] ** 2).mean()))
        if prev is not None and abs(prev - rmse) < tol:
            break
        prev = rmse
    return T, rmse, ratio


# ---------------------------------------------------------------- node
class Relocalizer(Node):
    def __init__(self):
        super().__init__('relocalizer')
        p = self.declare_parameter
        p('map_npy', os.path.expanduser('~/.scuba/maps/ilounge_02_loc_cloud.npy'))
        p('cloud_topic', '/cloud_registered')
        p('odom_topic', '/Odometry')
        p('map_frame', 'map')
        p('odom_frame', 'camera_init')      # FAST-LIO's odom frame
        p('map_voxel', 0.10)
        p('scan_voxel', 0.20)
        p('max_scan_pts', 8000)
        p('accum_scans', 5)   # merge N scans before registering
        p('rate_hz', 1.0)
        p('max_corr', 0.30)          # final (fine) gate
        p('coarse_gates', [1.5, 0.8, 0.4])   # swept before the fine gate
        p('recovery_gate', 3.0)      # used when tracking is lost
        p('recovery_inliers', 0.30)  # below this -> recovery mode next cycle
        p('plane_all', False)        # True = point-to-plane on ALL gates
        p('iters', 15)
        p('z_band', [-0.5, 2.5])            # ignore ceiling/far-above returns
        # seed (option 1) - the marked floor spot, in MAP frame
        # seed_z is the SENSOR height above the map floor: build_map.py writes the
        # cloud with the floor at z=0, and the G1's Mid-360 sits ~1.2 m up.
        p('seed_x', 0.0); p('seed_y', 0.0); p('seed_z', 1.2); p('seed_yaw', 0.0)
        p('fitness_warn', 0.35)             # rmse above this -> unhealthy
        # Level the seed with gravity from the lidar's IMU. This G1's Mid-360 is
        # mounted INVERTED (~150 deg from up) and camera_init inherits that, so a
        # yaw-only seed is upside down. ICP only refines locally and cannot flip
        # 150 deg: it settles into a mirrored solution that still reports ~80%
        # inliers and 0.10 m rmse while being metres off.
        p('level_seed', True)
        p('imu_topic', '/livox/imu')
        g = lambda n: self.get_parameter(n).value

        mp = os.path.expanduser(g('map_npy'))
        if not os.path.isfile(mp):
            raise SystemExit('!! map not found: %s' % mp)
        M = np.load(mp).astype(np.float64)
        zb = g('z_band')
        M = M[(M[:, 2] > zb[0]) & (M[:, 2] < zb[1])]
        M = voxel_downsample(M, g('map_voxel'))
        self.tree = cKDTree(M)
        nc = os.path.splitext(mp)[0] + '_normals_%dcm.npy' % int(g('map_voxel') * 100)
        t0 = time.time()
        self.normals = compute_normals(M, self.tree, k=12, cache=nc)
        self.get_logger().info('normals: %d in %.1fs (%s)'
                               % (len(self.normals), time.time() - t0,
                                  'cached' if os.path.isfile(nc) else 'computed'))
        self.get_logger().info('map: %s  %d pts after %.2f m voxel'
                               % (os.path.basename(mp), len(M), g('map_voxel')))

        self.map_frame, self.odom_frame = g('map_frame'), g('odom_frame')
        self.scan_voxel, self.max_pts = g('scan_voxel'), g('max_scan_pts')
        self.accum_n = int(g('accum_scans'))
        self.accum = []
        self.max_corr, self.iters = g('max_corr'), g('iters')
        self.gates = list(g('coarse_gates')) + [g('max_corr')]
        self.rec_gate, self.rec_thresh = g('recovery_gate'), g('recovery_inliers')
        self.plane_all = bool(g('plane_all'))
        self.lost = False
        self.zb, self.warn = zb, g('fitness_warn')

        # T_seed is map->BODY (where the robot is). T (map->odom) is derived
        # from it once the first /Odometry arrives - see _apply_seed().
        self.T_seed = np.eye(4)
        self.T_seed[:3, :3] = yaw_to_R(g('seed_yaw'))
        self.T_seed[:3, 3] = [g('seed_x'), g('seed_y'), g('seed_z')]
        self.T = self.T_seed.copy()
        self.T_odom_body = np.eye(4)
        self.get_logger().info('icp: %s'
                               % ('point-to-plane on ALL gates' if bool(g('plane_all'))
                                  else 'point-to-point coarse + point-to-plane fine'))
        self.get_logger().info('seed: x=%.2f y=%.2f z=%.2f yaw=%.1f deg'
                               % (g('seed_x'), g('seed_y'), g('seed_z'),
                                  math.degrees(g('seed_yaw'))))

        self.lock = threading.Lock()
        self.cloud = None
        self.have_odom = False
        self.level = bool(g('level_seed'))
        self.seed_yaw, self.seed_z = g('seed_yaw'), g('seed_z')
        self.imu_buf = []            # recent accel directions, body frame
        self.seeded = False          # no TF and no ICP until the seed is placed
        self.odom_first = None
        self.br = TransformBroadcaster(self)
        self.fit_pub = self.create_publisher(Float32, '~/fitness', 10)

        self.create_subscription(PointCloud2, g('cloud_topic'), self.on_cloud, 1)
        self.create_subscription(Odometry, g('odom_topic'), self.on_odom, 10)
        if self.level:
            self.create_subscription(Imu, g('imu_topic'), self.on_imu, 50)
        # option 3: runtime override (rviz "2D Pose Estimate", or a script)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initialpose, 1)
        self.create_timer(1.0 / g('rate_hz'), self.tick)
        self.create_timer(0.05, self.publish_tf)     # 20 Hz TF, independent of ICP

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [p.x, p.y, p.z]
        with self.lock:
            self.T_odom_body = T
            if not self.have_odom:
                self.have_odom = True
                self.odom_first = time.time()
            if self.seeded:
                return
            if not self.level or len(self.imu_buf) >= 50:
                self._apply_seed(T)
            elif time.time() - self.odom_first > 3.0:
                self.get_logger().error(
                    'no IMU after 3 s - seeding WITHOUT gravity leveling. On this '
                    'robot (inverted Mid-360) that seed is upside down: expect a '
                    'wrong pose. Check the imu_topic parameter.')
                self.level = False
                self._apply_seed(T)

    def on_imu(self, msg):
        a = msg.linear_acceleration
        v = np.array([a.x, a.y, a.z])
        n = float(np.linalg.norm(v))
        if n < 1e-6:
            return
        with self.lock:
            self.imu_buf.append(v / n)
            if len(self.imu_buf) > 200:
                self.imu_buf.pop(0)

    def _seed_R(self, heading):
        """Seed orientation. Caller holds the lock."""
        if not self.level or len(self.imu_buf) < 50:
            return yaw_to_R(heading), None
        up = np.mean(self.imu_buf, axis=0)
        up /= np.linalg.norm(up)
        return leveled_R(up, heading), up

    def _apply_seed(self, T_odom_body):
        """Interpret the seed as WHERE THE ROBOT IS NOW, not where odom started.

        The TF we publish is map->odom, but a user naturally specifies where the
        *robot* is. Those differ by however far FAST-LIO has already travelled:

            T_map_odom = T_map_body(seed) @ inv(T_odom_body)

        Seeding map->odom directly means any motion between FAST-LIO startup and
        seeding is silently added on top - which puts the initial guess far from
        the truth and can leave ICP outside its convergence basin.
        """
        d = np.linalg.norm(T_odom_body[:3, 3])
        R, up = self._seed_R(self.seed_yaw)
        self.T_seed[:3, :3] = R
        self.T = self.T_seed @ np.linalg.inv(T_odom_body)
        self.seeded = True
        self.get_logger().info(
            'seed applied: robot at map (%.2f, %.2f, z %.2f); odom had already moved '
            '%.2f m since FAST-LIO start' %
            (self.T_seed[0, 3], self.T_seed[1, 3], self.T_seed[2, 3], d))
        if up is not None:
            self.get_logger().info(
                'seed LEVELED with IMU gravity: sensor +z is %.1f deg from up '
                '(~150 expected: inverted mount)'
                % math.degrees(math.acos(max(-1.0, min(1.0, float(up[2]))))))

    def on_cloud(self, msg):
        P = pc2_xyz(msg)
        if len(P) == 0:
            return
        # z_band is defined in the MAP frame (floor at 0), so apply it there with
        # the current estimate. In camera_init - the inverted sensor's frame here -
        # raw z points roughly DOWN: banding raw z kept the floor and dropped the
        # upper walls.
        with self.lock:
            T, ok = self.T.copy(), self.seeded
        z = P @ T[2, :3] + T[2, 3] if ok else P[:, 2]
        P = P[(z > self.zb[0]) & (z < self.zb[1])]
        with self.lock:
            self.accum.append(P)
            if len(self.accum) > self.accum_n:
                self.accum.pop(0)
            Q = np.vstack(self.accum)
        Q = voxel_downsample(Q, self.scan_voxel)
        if len(Q) > self.max_pts:
            Q = Q[np.random.choice(len(Q), self.max_pts, replace=False)]
        with self.lock:
            self.cloud = Q

    def on_initialpose(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        pz = msg.pose.pose.position.z
        Ts = np.eye(4)
        # RViz and global_reloc send z=0 and a yaw-only quaternion: take x, y and
        # heading from the message, and height + roll/pitch from seed_z and gravity.
        Ts[:3, 3] = [msg.pose.pose.position.x, msg.pose.pose.position.y,
                     pz if abs(pz) > 1e-6 else self.seed_z]
        with self.lock:
            Ts[:3, :3], _ = self._seed_R(yaw)
            self.T_seed = Ts
            self.T = Ts @ np.linalg.inv(self.T_odom_body)
            self.accum = []            # banded with the old estimate
            self.cloud = None
            self.seeded = True
        self.get_logger().warn('seed OVERRIDDEN via /initialpose: x=%.2f y=%.2f yaw=%.1f deg'
                               % (msg.pose.pose.position.x, msg.pose.pose.position.y,
                                  math.degrees(yaw)))

    def tick(self):
        with self.lock:
            if not self.seeded:
                return
            P = None if self.cloud is None else self.cloud.copy()
            T0 = self.T.copy()
        if P is None or len(P) < 50:
            return
        t0 = time.time()
        gates = ([self.rec_gate] + self.gates) if self.lost else self.gates
        T, rmse, ratio = icp(P, self.tree, T0, iters=self.iters,
                             max_corr=self.max_corr, gates=gates,
                             normals=self.normals, plane_all=self.plane_all)
        dt = (time.time() - t0) * 1000
        was_lost, self.lost = self.lost, (ratio < self.rec_thresh or not np.isfinite(rmse))
        if was_lost and not self.lost:
            self.get_logger().info('RE-ACQUIRED (rmse %.3f, inliers %.0f%%)' % (rmse, 100*ratio))
        with self.lock:
            self.T = T
        self.fit_pub.publish(Float32(data=float(rmse)))
        msg = ('rmse %.3f m  inliers %4.1f%%  %d pts  %.0f ms' %
               (rmse, 100 * ratio, len(P), dt))
        if self.lost:
            self.get_logger().warn('TRACKING LOST (widening gate to %.1f m): %s'
                                   % (self.rec_gate, msg))
        elif rmse > self.warn or ratio < 0.4:
            self.get_logger().warn('LOW CONFIDENCE: ' + msg)
        else:
            self.get_logger().info(msg)

    def publish_tf(self):
        if not self.seeded:
            return
        with self.lock:
            T = self.T.copy()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(T[0, 3])
        t.transform.translation.y = float(T[1, 3])
        t.transform.translation.z = float(T[2, 3])
        qx, qy, qz, qw = R_to_quat(T[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y = float(qx), float(qy)
        t.transform.rotation.z, t.transform.rotation.w = float(qz), float(qw)
        self.br.sendTransform(t)


def main():
    rclpy.init()
    try:
        rclpy.spin(Relocalizer())
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
