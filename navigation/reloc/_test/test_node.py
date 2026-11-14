"""Drive the PATCHED Relocalizer node through its real callbacks with synthetic messages."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                     # patched relocalizer.py lives here
import numpy as np
import rclpy
import relocalizer as RL
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
assert hasattr(RL, 'leveled_R'), 'not the patched module'

MAP = os.path.expanduser('~/.scuba/maps/ilounge_02_loc_cloud.npy')
ZSH = 1.2077
vis = RL.voxel_downsample(np.load(MAP).astype(np.float64), 0.08)
d = np.loadtxt(os.path.join(HERE, 'traj.txt'))
rng = np.random.default_rng(7)

def sim_scan(T):
    R, t = T[:3, :3], T[:3, 3]
    P = (vis - t) @ R
    r = np.linalg.norm(P, axis=1); k = (r > 0.4) & (r < 25.0); P, r = P[k], r[k]
    el = np.degrees(np.arcsin(np.clip(P[:, 2] / r, -1, 1)))
    k = (el > -7.0) & (el < 52.0); P, r, el = P[k], r[k], el[k]
    az = np.degrees(np.arctan2(P[:, 1], P[:, 0]))
    key = np.floor((az + 180) / 0.4).astype(np.int64) * 1000 + np.floor((el + 7) / 0.5).astype(np.int64)
    o = np.lexsort((r, key)); key = key[o]
    first = np.ones(len(key), bool); first[1:] = key[1:] != key[:-1]
    return P[o[first]]

def pc2(P):
    m = PointCloud2(); m.header.frame_id = 'camera_init'
    m.height, m.width = 1, len(P)
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyz')]
    m.is_bigendian, m.point_step, m.row_step, m.is_dense = False, 12, 12 * len(P), True
    m.data = P.astype('<f4').tobytes()
    return m

def truth(i):
    T = np.eye(4); T[:3, :3] = RL.quat_to_R(*d[i, 4:8]); T[:3, 3] = d[i, 1:4] + [0, 0, ZSH]
    return T

def err(T, Tt):
    Re = T[:3, :3].T @ Tt[:3, :3]
    return (float(np.linalg.norm(T[:2, 3] - Tt[:2, 3])),
            math.degrees(math.acos(max(-1, min(1, (np.trace(Re) - 1) / 2)))))

def run(i, level, via_initialpose=False, dxy=0.3, dyaw=10.0):
    Tt = truth(i)
    up = Tt[:3, :3].T @ np.array([0.0, 0.0, 1.0])            # what the IMU reads
    hd = math.atan2(Tt[1, 0], Tt[0, 0]) + math.radians(dyaw)
    sx, sy = Tt[0, 3] + dxy, Tt[1, 3] - dxy / 2
    base = ['-p', 'max_scan_pts:=20000', '-p', 'scan_voxel:=0.12', '-p', 'plane_all:=true',
            '-p', 'accum_scans:=8', '-p', 'level_seed:=%s' % ('true' if level else 'false')]
    seed = [] if via_initialpose else ['-p', 'seed_x:=%.4f' % sx, '-p', 'seed_y:=%.4f' % sy,
                                        '-p', 'seed_yaw:=%.5f' % hd]
    rclpy.init(args=['--ros-args'] + base + seed)
    n = RL.Relocalizer()
    try:
        for _ in range(60):                                    # IMU first, as on the robot
            m = Imu(); a = up + rng.normal(0, 0.003, 3)
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, a)
            n.on_imu(m)
        o = Odometry(); o.pose.pose.orientation.w = 1.0       # camera_init == body at start
        n.on_odom(o)
        assert n.seeded, 'node did not seed'
        if via_initialpose:
            ip = PoseWithCovarianceStamped(); ip.header.frame_id = 'map'
            ip.pose.pose.position.x, ip.pose.pose.position.y = sx, sy   # z left 0, like RViz
            ip.pose.pose.orientation.z, ip.pose.pose.orientation.w = math.sin(hd / 2), math.cos(hd / 2)
            n.on_initialpose(ip)
        P = sim_scan(Tt)
        for _ in range(8):
            n.on_cloud(pc2(P + rng.normal(0, 0.02, P.shape)))
        for _ in range(5):
            n.tick()
        n.publish_tf()                                          # must not raise
        e = err(n.T, Tt)
        tilt = math.degrees(math.acos(max(-1, min(1, n.T[2, 2]))))
        return e, tilt, float(n.T[2, 3]), len(n.cloud) if n.cloud is not None else 0
    finally:
        n.destroy_node(); rclpy.shutdown()

print('\n%-5s %-24s %8s %8s %7s %7s %7s' % ('pose', 'path', 'xy err', 'rot err', 'tilt', 'z', 'pts'))
print('-' * 72)
res = {}
for i in (600, 2400, 3600):
    for label, lv, ip in (('OLD  yaw-only seed', False, False),
                          ('NEW  leveled seed', True, False),
                          ('NEW  via /initialpose', True, True)):
        (dxy, dr), tilt, z, npts = run(i, lv, ip)
        res.setdefault(label, []).append((dxy, dr))
        print('%-5d %-24s %7.2fm %7.1fd %6.1fd %+6.2f %7d' % (i, label, dxy, dr, tilt, z, npts))
print('-' * 72)
print('true sensor tilt in the map ~%.0f deg, true sensor z ~%+.2f'
      % (math.degrees(math.acos(np.clip(truth(600)[2, 2], -1, 1))), truth(600)[2, 3]))
for k, v in res.items():
    a = np.array(v); ok = ((a[:, 0] < 0.15) & (a[:, 1] < 3)).sum()
    print('%-24s %d/%d correct   median xy %.2f m, rot %.1f deg' % (k, ok, len(a), np.median(a[:, 0]), np.median(a[:, 1])))
