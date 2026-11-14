"""Does a gravity-leveled seed fix the relocalizer? Uses the relocalizer's OWN icp().

Truth = the mapping trajectory (sensor pose in the map, inverted mount included).
The scan is simulated from the map in the SENSOR frame with the Mid-360's real
FOV (-7..+52 deg in its own frame) and occlusion, then processed like on_cloud.
"""
import math, os, sys
sys.path.insert(0, os.path.expanduser('~/.scuba/reloc'))
import numpy as np
from scipy.spatial import cKDTree
import relocalizer as RL

HERE = os.path.dirname(os.path.abspath(__file__))
MAP = os.path.expanduser('~/.scuba/maps/ilounge_02_loc_cloud.npy')
ZSH = 1.2077            # build_map subtracts z_shift from the cloud, not the traj
ZB = (-0.5, 2.5); GATES = [1.5, 0.8, 0.4, 0.30]

raw = np.load(MAP).astype(np.float64)
M = raw[(raw[:, 2] > ZB[0]) & (raw[:, 2] < ZB[1])]
M = RL.voxel_downsample(M, 0.10)
tree = cKDTree(M)
nrm = RL.compute_normals(M, tree, k=12, cache=os.path.splitext(MAP)[0] + '_normals_10cm.npy')
vis = RL.voxel_downsample(raw, 0.08)          # what the lidar can see: floor/ceiling too
print('map %d pts (banded)   visibility cloud %d pts' % (len(M), len(vis)))

d = np.loadtxt(os.path.join(HERE, 'traj.txt'))

def R_align(u, z=np.array([0.0, 0.0, 1.0])):
    u = u / np.linalg.norm(u); v = np.cross(u, z); s = np.linalg.norm(v); c = float(u @ z)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    V = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + V + V @ V * ((1 - c) / s ** 2)

def leveled(up_body, heading):
    Rl = R_align(up_body)
    v = Rl @ np.array([1.0, 0.0, 0.0])
    return RL.yaw_to_R(heading - math.atan2(v[1], v[0])) @ Rl

def sim_scan(T, rng):
    R, t = T[:3, :3], T[:3, 3]
    P = (vis - t) @ R                               # map -> sensor frame
    r = np.linalg.norm(P, axis=1)
    k = (r > 0.4) & (r < 25.0)
    P, r = P[k], r[k]
    el = np.degrees(np.arcsin(np.clip(P[:, 2] / r, -1, 1)))
    k = (el > -7.0) & (el < 52.0)                   # Mid-360 FOV, SENSOR frame
    P, r, el = P[k], r[k], el[k]
    az = np.degrees(np.arctan2(P[:, 1], P[:, 0]))
    key = np.floor((az + 180) / 0.4).astype(np.int64) * 1000 + np.floor((el + 7) / 0.5).astype(np.int64)
    o = np.lexsort((r, key)); key, o = key[o], o
    first = np.ones(len(key), bool); first[1:] = key[1:] != key[:-1]
    P = P[o[first]]                                  # nearest hit per ray: occlusion
    return P + rng.normal(0, 0.02, P.shape)

def prep(P, T_band):
    """on_cloud: z_band, voxel, cap. T_band=None -> band on raw camera_init z (current)."""
    if T_band is None:
        z = P[:, 2]
    else:
        z = (P @ T_band[:3, :3].T + T_band[:3, 3])[:, 2]
    P = P[(z > ZB[0]) & (z < ZB[1])]
    P = RL.voxel_downsample(P, 0.12)
    return P

def run(P, T0, ticks=5):
    T = T0.copy()
    for _ in range(ticks):
        T, rmse, ratio = RL.icp(P, tree, T, iters=15, max_corr=0.30, gates=GATES,
                                normals=nrm, plane_all=True)
    return T, rmse, ratio

def err(T, Tt):
    dxy = float(np.linalg.norm(T[:2, 3] - Tt[:2, 3]))
    Re = T[:3, :3].T @ Tt[:3, :3]
    ang = math.degrees(math.acos(max(-1, min(1, (np.trace(Re) - 1) / 2))))
    return dxy, ang

rng = np.random.default_rng(1)
idx = [300, 1200, 2100, 3000, 3900]
print('\n%-5s %-26s %7s %7s %7s %7s %7s' % ('pose', 'seed', 'xy err', 'rot err', 'tilt', 'inl', 'rmse'))
print('-' * 76)
summ = {'current': [], 'leveled': []}
for i in idx:
    Tt = np.eye(4)
    Tt[:3, :3] = RL.quat_to_R(*d[i, 4:8])
    Tt[:3, 3] = d[i, 1:4] + [0, 0, ZSH]
    up_body = Tt[:3, :3].T @ np.array([0.0, 0.0, 1.0])     # what the IMU measures
    hd = math.atan2(Tt[1, 0], Tt[0, 0])
    P = sim_scan(Tt, rng)
    for dxy_off, dyaw in [(0.0, 0.0), (0.3, 10.0)]:
        off = np.array([dxy_off, -dxy_off * 0.5, 0.0])
        h = hd + math.radians(dyaw)
        # CURRENT: yaw-only upright seed, seed_z 0, band on raw camera_init z
        T0 = np.eye(4); T0[:3, :3] = RL.yaw_to_R(h); T0[:3, 3] = Tt[:3, 3] * [1, 1, 0] + off
        Tc, rm, ra = run(prep(P, None), T0)
        e = err(Tc, Tt); tilt = math.degrees(math.acos(max(-1, min(1, Tc[2, 2]))))
        print('%-5d %-26s %6.2fm %6.1fd %6.1fd %6.0f%% %7.3f' % (i, 'current (yaw-only, z=0)', e[0], e[1], tilt, 100 * ra, rm))
        summ['current'].append(e)
        # LEVELED: IMU-leveled orientation, z = sensor height, band in map frame
        T0 = np.eye(4); T0[:3, :3] = leveled(up_body, h); T0[:3, 3] = Tt[:3, 3] * [1, 1, 0] + [0, 0, 1.2] + off
        Tl, rm, ra = run(prep(P, T0), T0)
        e = err(Tl, Tt); tilt = math.degrees(math.acos(max(-1, min(1, Tl[2, 2]))))
        print('%-5s %-26s %6.2fm %6.1fd %6.1fd %6.0f%% %7.3f' % ('', 'leveled (IMU, z=1.2)', e[0], e[1], tilt, 100 * ra, rm))
        summ['leveled'].append(e)
print('-' * 76)
print('true tilt of the sensor in the map: %.1f deg' % math.degrees(math.acos(
    np.clip(RL.quat_to_R(*d[idx[0], 4:8])[2, 2], -1, 1))))
for k, v in summ.items():
    a = np.array(v)
    ok = ((a[:, 0] < 0.15) & (a[:, 1] < 3.0)).sum()
    print('%-8s  %d/%d within 15 cm & 3 deg   median xy %.2f m  median rot %.1f deg'
          % (k, ok, len(a), np.median(a[:, 0]), np.median(a[:, 1])))
