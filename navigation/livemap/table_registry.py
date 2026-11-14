#!/usr/bin/env python3
"""Track tables in the accumulated lidar map, across repeated detections.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/table_registry.py            # live
    /usr/bin/python3 ~/.scuba/livemap/table_registry.py \
        --from-file ~/cloud.npy --once                             # offline test

Subscribes  /accumulated_cloud   (PointCloud2, map frame, from accumulator.py)
Publishes   /table_registry      (std_msgs/String, JSON)
Subscribes  /table_event         (std_msgs/String, JSON) - the mission executive
                                 reports CONFIRMED / VISITED / REJECTED here
Serves      http://<robot>:8094  the same JSON, for a human to look at

WHY A REGISTRY AND NOT JUST A DETECTOR
Re-running the detector gives different cluster IDs every time: as the map
grows a table seen from one side merges with its other side, or one cluster
splits in two. If state were keyed to the detector's output, "visited" would
be lost exactly when the map improved. So an ENTRY OWNS A POSITION, not a
cluster. Each pass matches clusters to existing entries by nearest centroid
within --gate, and entries survive changes in the segmentation underneath.

THE TABLE IS A KNOWN 1.4 m SQUARE
Measured on this robot's clouds, the fitted long side reads 1.29 +- 0.07 m
against a 1.40 m truth - a systematic ~11 cm underestimate, because the
rectangle is fitted through occupied CELL CENTRES and returns at the rim are
often missed. So the size is NOT estimated: clusters are classified against
the known square and the known 1.40 m is what downstream geometry uses.

  TABLE corner   both sides ~1.4   -> position and yaw fully constrained
  TABLE face     one side ~1.4     -> yaw known, centre ambiguous in depth;
                                      resolved because the table is on the far
                                      side of the face from the robot
  MERGED+chairs  long side > 1.9   -> a table plus chairs, or several tables
  fragment       too small to judge
"""
import argparse
import collections
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

SIDE = 1.20                      # table edge, measured in the room (120 cm)
RES = 0.15
# The table surface itself. Measured on robot_run02 after de-tilting: the z
# histogram has a clean secondary peak at exactly 0.85 m.
SLAB_LO, SLAB_HI = 0.68, 0.86    # brackets a 0.76 m top. Was 0.78-0.95 for
                                 # a 0.85 m top, and a 0.76 m table falls
                                 # BELOW that band - no cell would ever be
                                 # classed as surface and the table would be
                                 # invisible to the registry entirely.
# A return in this band means the cell holds a wall, not a table. The OLD gate
# took the TOPMOST return per cell and required it in 0.70-1.00, which threw
# away 87% of the cells that actually held a 0.78-0.95 m surface (measured on
# run02: 5314 of 6131), because something taller shared the 0.15 m column.
# Worse, /accumulated_cloud only ever GAINS points (memory_layer runs with
# clearing:false), so a cell's topmost return only ever moves up - a cell
# poisoned once could never recover, and detection decayed over a run.
TALL_LO, TALL_HI = 1.20, 1.60
# A banquet chair back tops out at 0.85-0.95 m, i.e. INSIDE the surface slab,
# so a table fuses with its ring of chairs into ONE solid blob and no amount
# of morphological opening separates them (tried: the chair tops form a
# continuous solid ring, not thin lines). On run02 this was the dominant real
# object: 10 blobs, long 2.69 +- 0.54 m, short 2.11 +- 0.26 m, fill 56 +- 13%,
# on a regular 2.98 +- 0.19 m grid. 1.40 table + 0.35 chair each side = 2.10.
# So the blob IS the signature, and the table centre is the blob centre.
CHAIR = 0.35
# SQUARE-ONLY acceptance. A table is a square chunk; everything else is not
# a table, however table-height it is. This replaces the old size-band
# classes, which let 1.4 x 0.7 m edge strips and 2.9 x 1.6 m part-blobs
# through as 'TABLE face' / 'partial' and made the registry noisy.
# Tuned against the 10 hand-verified tables in robot_run02. Those clusters
# measured aspect 1.02-1.27, fill 41-67%, LENGTH 2.38-2.90 m; the tightest
# NON-table cluster in the same scene had aspect 1.33. So the gap sits at
# 1.30 and these values give 6 of 6 detectable tables with 0 false positives,
# where 1.25/0.55/2.60 gave only 2 of 6.
SQ_ASPECT = 1.30     # long/short. 1.0 is a perfect square
SQ_FILL = 0.40       # solid chunk, not a sparse outline
SQ_LO, SQ_HI = 0.95, 3.00   # a bare 1.20 top up to a table + its chair ring.
                            # 0.95 not 1.15: at 1.15 a 1.20 m table had only
                            # 0.05 m of margin, so any partial view measured
                            # it short and it was rejected as 'small'.
MIN_SLAB = 2
# A wall cell has returns at EVERY height, so it has far more tall returns
# than slab returns; a table cell has almost none. Comparing the two counts
# rejects walls without the old absolute "zero returns above 1.20 m" veto,
# which was killing real table cells outright (87% of them, measured).
TALL_FRAC = 0.5

STATE = {'json': '{}'}
LOCK = threading.Lock()


# ----------------------------------------------------------------- detection
def label_grid(mask):
    """8-connected labelling; background 0."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    nxt = 1
    for y in range(h):
        for x in np.nonzero(mask[y])[0]:
            nb = []
            if x > 0 and lab[y, x - 1]:
                nb.append(lab[y, x - 1])
            if y > 0:
                for dx in (-1, 0, 1):
                    xx = x + dx
                    if 0 <= xx < w and lab[y - 1, xx]:
                        nb.append(lab[y - 1, xx])
            if not nb:
                lab[y, x] = nxt
                parent.append(nxt)
                nxt += 1
            else:
                m = min(nb)
                lab[y, x] = m
                for v in nb:
                    union(m, v)
    remap, out, cnt = {}, np.zeros_like(lab), 0
    for y, x in zip(*np.nonzero(lab)):
        r = find(lab[y, x])
        if r not in remap:
            cnt += 1
            remap[r] = cnt
        out[y, x] = remap[r]
    return out, cnt


def convex_hull(P):
    P = np.unique(np.asarray(P, float), axis=0)
    if len(P) < 3:
        return P
    P = P[np.lexsort((P[:, 1], P[:, 0]))]

    def half(pts):
        h = []
        for p in pts:
            # explicit 2-D cross; np.cross on 2-vectors is deprecated in numpy 2
            while len(h) >= 2 and ((h[-1][0]-h[-2][0])*(p[1]-h[-2][1])
                                   - (h[-1][1]-h[-2][1])*(p[0]-h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h

    return np.array(half(P)[:-1] + half(P[::-1])[:-1])


def min_area_rect(P):
    """(centre, long side, short side, unit vector along long side)."""
    Hl = convex_hull(P)
    if len(Hl) < 3:
        mn, mx = P.min(0), P.max(0)
        c = (mn + mx) / 2
        d = mx - mn
        return c, float(max(d)), float(min(d)), np.array([1.0, 0.0])
    best = None
    n = len(Hl)
    for i in range(n):
        e = Hl[(i + 1) % n] - Hl[i]
        ln = float(np.hypot(*e))
        if ln < 1e-12:
            continue
        u = e / ln
        v = np.array([-u[1], u[0]])
        pu, pv = Hl @ u, Hl @ v
        w, h = pu.max() - pu.min(), pv.max() - pv.min()
        if best is None or w * h < best[0]:
            c = ((pu.max() + pu.min()) / 2) * u + ((pv.max() + pv.min()) / 2) * v
            best = (w * h, w, h, u, v, c)
    _, w, h, u, v, c = best
    return (c, float(w), float(h), u) if w >= h else (c, float(h), float(w), v)


def classify(L, W, fill=1.0):
    """Square-only. Returns 'TABLE' for a square chunk, else why it is not.

    Only 'TABLE' is kept by default (--keep). The reject reasons are still
    reported so the viewer and the logs can show what was looked at.
    """
    if L > SQ_HI:
        return 'MERGED'                  # two tables, or a table plus a wall
    if L / max(W, 1e-9) > SQ_ASPECT:
        return 'strip'                   # wall, chair row, or one table edge
    if W < SQ_LO:
        return 'small'                   # square but too small to be a table
    if fill < SQ_FILL:
        return 'sparse'                  # square outline, not a solid chunk
    return 'TABLE'


def fit_floor(P, tile=1.0, min_pts=25, iters=4):
    """Robust floor plane z = a*x + b*y + c from a cloud that still has floor.

    run02's map was tilted 1.30 deg (2.3 cm/m) with the floor at z = -1.14,
    not 0. Over a 7 m sensing radius that is 0.16 m of height error against a
    0.17 m wide surface slab, so far tables slid out of the band entirely -
    which is exactly why near tables read correctly and far ones did not.
    The relocalizer's auto_z measures the floor OFFSET; it does not measure
    TILT, so this has to be done here.

    Tiles XY and takes a low percentile of z per tile so that furniture and
    people cannot pull the fit upward, then IRLS-trims outlying tiles.
    Returns (a, b, c, tilt_deg, rms) or None.
    """
    if len(P) < 2000:
        return None
    x0, y0 = P[:, 0].min(), P[:, 1].min()
    tx = ((P[:, 0] - x0) / tile).astype(np.int64)
    ty = ((P[:, 1] - y0) / tile).astype(np.int64)
    W = int(tx.max()) + 1
    lin = ty * W + tx
    order = np.lexsort((P[:, 2], lin))
    ls, zs = lin[order], P[order, 2]
    edges = np.flatnonzero(np.diff(ls)) + 1
    starts = np.concatenate([[0], edges])
    ends = np.concatenate([edges, [len(ls)]])
    cx, cy, cz = [], [], []
    for s, e in zip(starts, ends):
        n = e - s
        if n < min_pts:
            continue
        t = ls[s]
        cx.append(x0 + (t % W) * tile + tile / 2.0)
        cy.append(y0 + (t // W) * tile + tile / 2.0)
        cz.append(zs[s + int(0.05 * n)])          # 5th percentile = local floor
    if len(cz) < 12:
        return None
    A = np.column_stack([cx, cy, np.ones(len(cz))])
    z = np.asarray(cz, float)
    coef = np.zeros(3)
    for _ in range(iters):
        if len(z) < 8:
            break
        coef, *_ = np.linalg.lstsq(A, z, rcond=None)
        r = z - A @ coef
        s = 1.4826 * np.median(np.abs(r - np.median(r)))
        keep = np.abs(r) < max(2.5 * s, 0.05)
        if keep.sum() < 8:
            break
        A, z = A[keep], z[keep]
    r = z - A @ coef
    a, b, c = float(coef[0]), float(coef[1]), float(coef[2])
    return (a, b, c,
            float(math.degrees(math.atan(math.hypot(a, b)))),
            float(np.sqrt((r ** 2).mean())),
            float(np.median(cz)), len(cz))


def detect(P, min_cells, plane=None):
    """P: (N,3) in the map frame. Returns cluster dicts.

    `plane` is (a,b,c) from fit_floor(); when given, heights are taken
    relative to that plane instead of assuming the floor is exactly z=0.
    """
    if len(P) < 50:
        return []
    x0, y0 = P[:, 0].min(), P[:, 1].min()
    ix = ((P[:, 0] - x0) / RES).astype(np.int32)
    iy = ((P[:, 1] - y0) / RES).astype(np.int32)
    W, H = int(ix.max()) + 1, int(iy.max()) + 1
    if W * H > 4_000_000:
        return []
    N = W * H
    lin = iy.astype(np.int64) * W + ix
    if plane is None:
        z = P[:, 2]
    else:
        z = P[:, 2] - (plane[0] * P[:, 0] + plane[1] * P[:, 1] + plane[2])
    nsurf = np.bincount(lin[(z >= SLAB_LO) & (z <= SLAB_HI)], minlength=N)
    ntall = np.bincount(lin[(z >= TALL_LO) & (z <= TALL_HI)], minlength=N)
    # "holds a surface slab AND nothing tall above it" - not "topmost return
    # happens to land in the band". See the TALL_LO comment.
    cand = (nsurf >= MIN_SLAB) & (ntall <= TALL_FRAC * nsurf)
    lab, n = label_grid(cand.reshape(H, W))
    areas = np.bincount(lab.ravel())
    out = []
    for cid in range(1, n + 1):
        if areas[cid] < min_cells:
            continue
        cells = np.argwhere(lab == cid)
        Q = np.column_stack([cells[:, 1] * RES + x0, cells[:, 0] * RES + y0])
        c, L, Wd, u = min_area_rect(Q)
        L += RES                               # rect runs through cell CENTRES
        Wd += RES
        if L > 4.5:
            continue
        ncell = int(areas[cid])
        fill = ncell * RES * RES / max(L * Wd, 1e-9)
        cls = classify(L, Wd, fill)
        # A chair ring is near-square, so min_area_rect has no stable axis on
        # it: on run02 the fitted orientations of the ten table blobs ranged
        # 7-79 deg. Table YAW cannot come from the lidar blob - only the
        # camera edge detector gives it. The CENTRE is still good.
        # A square chunk has no stable long axis - on run02 the fitted
        # orientations of ten table blobs ranged 7-79 deg. Table yaw comes
        # from the camera edge detector, never from the lidar chunk.
        yaw_ok = False
        out.append(dict(cx=float(c[0]), cy=float(c[1]), L=L, W=Wd,
                        yaw=float(math.atan2(u[1], u[0])),
                        yaw_ok=yaw_ok, fill=float(fill),
                        n=ncell, cls=cls))
    return out

# ------------------------------------------------------------------ registry
class Registry:
    """Entries own a position. Clusters are matched to them, never the reverse."""

    def __init__(self, gate, keep_classes):
        self.e = {}
        self.next = 1
        self.gate = gate
        self.keep = keep_classes

    def update(self, clusters, now):
        seen = set()
        for c in clusters:
            if c['cls'] not in self.keep:
                continue
            best, bd = None, 1e9
            for k, en in self.e.items():
                d = math.hypot(en['cx'] - c['cx'], en['cy'] - c['cy'])
                if d < bd:
                    best, bd = k, d
            if best is not None and bd <= self.gate:
                en = self.e[best]
                if en['state'] in ('CANDIDATE', 'CONFIRMED'):
                    # the geometry may improve as the map grows; the STATE never
                    # regresses, which is what makes "do not revisit" hold
                    en.update(cx=c['cx'], cy=c['cy'], L=c['L'], W=c['W'],
                              yaw=c['yaw'], n=c['n'], cls=c['cls'])
                en['last_seen'] = now
                en['n_obs'] += 1
                seen.add(best)
            else:
                k = 'table_%d' % self.next
                self.next += 1
                self.e[k] = dict(id=k, state='CANDIDATE', first_seen=now,
                                 last_seen=now, n_obs=1, visited_sides=[], **c)
                seen.add(k)
        return seen

    def event(self, ev):
        k = ev.get('id')
        if k not in self.e:
            return False
        st = ev.get('state')
        if st in ('CONFIRMED', 'VISITED', 'REJECTED'):
            self.e[k]['state'] = st
        if 'side' in ev:
            v = self.e[k]['visited_sides']
            if ev['side'] not in v:
                v.append(ev['side'])
        return True

    def ranked(self, rx, ry):
        """Un-visited candidates, nearest first - the mission order."""
        out = []
        for en in self.e.values():
            if en['state'] in ('VISITED', 'REJECTED'):
                continue
            d = math.hypot(en['cx'] - rx, en['cy'] - ry)
            out.append(dict(en, range=d))
        out.sort(key=lambda r: r['range'])
        return out


def standoff_poses(en, standoff, robot_radius):
    """Four poses, one per side of the KNOWN square, facing the table."""
    c = np.array([en['cx'], en['cy']])
    th = en['yaw']
    u = np.array([math.cos(th), math.sin(th)])
    v = np.array([-u[1], u[0]])
    out = []
    for name, n in (('A', u), ('B', v), ('C', -u), ('D', -v)):
        p = c + n * (SIDE / 2 + standoff + robot_radius)
        out.append(dict(side=name, x=float(p[0]), y=float(p[1]),
                        yaw=float(math.atan2(-n[1], -n[0]))))
    return out



def require_ros2():
    """Fail clearly if this ROS 2 node was launched from a ROS 1 shell.

    Both ROS versions live on this robot - Foxy for the nav stack, Noetic for
    the RealSense driver - and sourcing FOXY FIRST then NOETIC puts ROS 1's
    tf2_ros ahead on the path. The node then tries to reach a ROS 1 master and
    dies inside xmlrpc with ConnectionRefused, which says nothing about the
    real cause. Order matters: noetic-then-foxy is fine, foxy-then-noetic is
    not, and a shell cannot be un-sourced.
    """
    distro = os.environ.get('ROS_DISTRO', '')
    if distro and distro != 'foxy':
        raise SystemExit(
            '!! ROS_DISTRO is %r, not foxy.\n'
            '!! This is a ROS 2 node, but the shell has ROS 1 on its path -\n'
            '!! it would try to contact a ROS 1 master and fail in xmlrpc.\n'
            '!! Open a NEW terminal and source only:\n'
            '!!     source ~/.scuba/lio_ws/env.sh' % distro)
    try:
        import tf2_ros
    except Exception:
        return
    p = getattr(tf2_ros, '__file__', '') or ''
    if '/noetic/' in p:
        raise SystemExit(
            '!! tf2_ros resolved to the ROS 1 build:\n!!   %s\n'
            '!! Open a NEW terminal and source only ~/.scuba/lio_ws/env.sh' % p)

def main():
    ap = argparse.ArgumentParser()
    # /cloud_registered is the LIVE scan: no clearing:false ratchet, so a
    # cell poisoned by a passer-by recovers within --window-s instead of
    # never. It is VOLATILE and in the odom frame, both handled below.
    ap.add_argument('--cloud-topic', default='/cloud_registered')
    ap.add_argument('--window-s', type=float, default=3.0,
                    help='rolling window of scans to detect on. One scan '
                         'is ~1240 pts over a 15 m radius, far too sparse '
                         'for a 0.15 m grid; 3 s is ~30 scans')
    ap.add_argument('--window-voxel', type=float, default=0.10)
    ap.add_argument('--cloud-qos', choices=['volatile', 'transient_local'],
                    default='volatile',
                    help='/cloud_registered is VOLATILE; a TRANSIENT_LOCAL sub never matches it')
    ap.add_argument('--period', type=float, default=10.0,
                    help='seconds between detection passes')
    ap.add_argument('--gate', type=float, default=0.75,
                    help='m; a cluster this close to an entry IS that entry')
    ap.add_argument('--min-cells', type=int, default=10)
    ap.add_argument('--standoff', type=float, default=0.30)
    ap.add_argument('--robot-radius', type=float, default=0.28)
    ap.add_argument('--keep', default='TABLE',
                    help='classes that become registry entries')
    ap.add_argument('--port', type=int, default=8094)
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--from-file', default='',
                    help='analyse a saved .npy instead of subscribing')
    # /accumulated_cloud is published band-limited to 0.20-1.60 m, so it
    # holds NO floor and the tilt cannot be fitted from it. The on-disk
    # store keeps the full -0.5..2.5 band, so fit from there. The floor
    # does not move, so a stale snapshot is fine for this purpose.
    ap.add_argument('--slab', type=float, nargs=2, default=[SLAB_LO, SLAB_HI],
                    help='table surface band, m above the fitted floor')
    ap.add_argument('--tall', type=float, nargs=2, default=[TALL_LO, TALL_HI],
                    help='a return here means wall, not table')
    ap.add_argument('--min-slab', type=int, default=MIN_SLAB,
                    help='slab returns a cell needs')
    # /cloud_registered is NOT band-limited, so the floor is right there in
    # the window and there is no need to read the accumulator's store.
    ap.add_argument('--floor-from', choices=['cloud', 'file'], default='cloud')
    ap.add_argument('--floor-file',
                    default='~/.scuba/livemap/maps/live/live_cloud.npy',
                    help="only used with --floor-from file")
    ap.add_argument('--sq-aspect', type=float, default=SQ_ASPECT)
    ap.add_argument('--sq-fill', type=float, default=SQ_FILL)
    ap.add_argument('--sq-size', type=float, nargs=2,
                    default=[SQ_LO, SQ_HI])
    ap.add_argument('--tall-frac', type=float, default=TALL_FRAC,
                    help='big value disables the wall veto')
    ap.add_argument('--max-floor-rms', type=float, default=0.06,
                    help='above this the tilt is noise; keep offset only')
    ap.add_argument('--min-floor-tiles', type=int, default=60,
                    help='1 m tiles needed before a tilt is trusted')
    ap.add_argument('--refit-s', type=float, default=60.0,
                    help='re-fit the floor plane this often')
    ap.add_argument('--once', action='store_true')
    a = ap.parse_args()
    _m = sys.modules[__name__]
    _m.SLAB_LO, _m.SLAB_HI = a.slab
    _m.TALL_LO, _m.TALL_HI = a.tall
    _m.MIN_SLAB = a.min_slab
    _m.SQ_ASPECT = a.sq_aspect
    _m.SQ_FILL = a.sq_fill
    _m.SQ_LO, _m.SQ_HI = a.sq_size
    _m.TALL_FRAC = a.tall_frac
    keep = [s.strip() for s in a.keep.split(',')]

    reg = Registry(a.gate, keep)

    # ---- offline mode: no ROS at all, for testing against a captured cloud
    if a.from_file:
        P = np.load(os.path.expanduser(a.from_file)).astype(np.float64)
        t0 = time.time()
        pl = fit_floor(P)
        if pl is None:
            print('floor plane: NOT FITTED (assuming floor at z=0)')
        else:
            print('floor plane: tilt %.2f deg, offset %+.3f m at origin, '
                  'fit rms %.3f m, %d tiles' % (pl[3], pl[2], pl[4], pl[6]))
        cl = detect(P, a.min_cells, pl[:3] if pl else None)
        reg.update(cl, time.time())
        print('%d clusters -> %d entries  (%.1f s)'
              % (len(cl), len(reg.e), time.time() - t0))
        print(' %-10s %-14s %8s %8s %6s %7s  %s'
              % ('id', 'class', 'LENGTH', 'width', 'cells', 'range', 'centre'))
        for en in reg.ranked(0.0, 0.0):
            print(' %-10s %-14s %7.2fm %7.2fm %6d %6.1fm  (%6.2f,%6.2f)'
                  % (en['id'], en['cls'], en['L'], en['W'], en['n'],
                     en['range'], en['cx'], en['cy']))
        nearest = reg.ranked(0.0, 0.0)
        if nearest:
            print('\nstandoff poses for %s (known %.2f m square, %.2f m standoff):'
                  % (nearest[0]['id'], SIDE, a.standoff))
            for p in standoff_poses(nearest[0], a.standoff, a.robot_radius):
                print('   side %s  go to (%6.2f, %6.2f)  facing %+6.1f deg'
                      % (p['side'], p['x'], p['y'], math.degrees(p['yaw'])))
        return

    require_ros2()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    import tf2_ros

    def pc2_xyz(msg):
        off = {f.name: f.offset for f in msg.fields}
        st = msg.point_step
        b = np.frombuffer(bytes(msg.data), np.uint8)
        n = len(b) // st
        b = b[:n * st].reshape(n, st)
        P = np.stack([b[:, off[k]:off[k] + 4].copy().view(np.float32).ravel()
                      for k in ('x', 'y', 'z')], 1).astype(np.float64)
        return P[np.isfinite(P).all(1)]

    def tf_matrix(tr):
        """(R, t) from a TransformStamped."""
        q = tr.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        t = tr.transform.translation
        return R, np.array([t.x, t.y, t.z])

    class Node_(Node):
        def __init__(self):
            super().__init__('table_registry')
            self.cloud = None
            self.win = collections.deque()   # (t, Nx3 map-frame points)
            self.n_drop = 0
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            dur = (DurabilityPolicy.VOLATILE if a.cloud_qos == 'volatile'
                   else DurabilityPolicy.TRANSIENT_LOCAL)
            q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                           durability=dur,
                           history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(PointCloud2, a.cloud_topic, self.on_cloud, q)
            self.create_subscription(String, '/table_event', self.on_event, 10)
            self.pub = self.create_publisher(String, '/table_registry', 10)
            self.plane = None
            self.plane_t = 0.0
            self.plane_tilt = 0.0
            self.plane_rms = 0.0
            self.create_timer(a.period, self.pass_)
            self.get_logger().info(
                'registry: %s every %.0f s, gate %.2f m, keeping %s'
                % (a.cloud_topic, a.period, a.gate, keep))

        def on_cloud(self, m):
            P = pc2_xyz(m)
            if len(P) == 0:
                return
            src = m.header.frame_id or a.odom_frame
            if src != a.map_frame:
                # /cloud_registered is in the FAST-LIO odom frame; the registry
                # publishes map-frame positions, so this has to be applied or
                # every table lands wherever the relocalizer's correction is.
                try:
                    tr = self.buf.lookup_transform(a.map_frame, src,
                                                   m.header.stamp)
                except Exception:
                    try:
                        tr = self.buf.lookup_transform(a.map_frame, src,
                                                       rclpy.time.Time())
                    except Exception:
                        self.n_drop += 1
                        if self.n_drop % 50 == 1:
                            self.get_logger().warn(
                                'no %s <- %s transform yet (%d scans dropped)'
                                % (a.map_frame, src, self.n_drop))
                        return
                R, t = tf_matrix(tr)
                P = P @ R.T + t
            now = time.time()
            self.win.append((now, P))
            cut = now - a.window_s
            while self.win and self.win[0][0] < cut:
                self.win.popleft()

        def window(self):
            """The rolling window as one voxel-deduplicated cloud."""
            if not self.win:
                return None
            P = np.vstack([p for _, p in self.win])
            v = a.window_voxel
            if v > 0 and len(P):
                k = np.floor(P / v).astype(np.int64)
                _, idx = np.unique(k, axis=0, return_index=True)
                P = P[np.sort(idx)]
            return P

        def on_event(self, m):
            try:
                ev = json.loads(m.data)
            except Exception:
                return
            if reg.event(ev):
                self.get_logger().info('event: %s -> %s %s'
                                       % (ev.get('id'), ev.get('state', ''),
                                          ev.get('side', '')))

        def robot(self):
            try:
                tr = self.buf.lookup_transform(a.map_frame, a.base_frame,
                                               rclpy.time.Time())
                return tr.transform.translation.x, tr.transform.translation.y
            except Exception:
                return None

        def floor_plane(self, P=None):
            """Cached robust floor plane.

            With --floor-from cloud the plane comes from the live window,
            which still contains the floor (/cloud_registered is not
            band-limited). That drops the dependency on the accumulator's
            on-disk store entirely.
            """
            now = time.time()
            if self.plane_t and now - self.plane_t < a.refit_s:
                return self.plane
            self.plane_t = now
            if a.floor_from == 'cloud':
                if P is None or len(P) < 2000:
                    return self.plane
                F = P
            elif not a.floor_file:
                return None
            else:
                try:
                    F = np.load(
                        os.path.expanduser(a.floor_file)).astype(np.float64)
                except Exception as exc:
                    self.get_logger().warn('floor file %s: %s'
                                           % (a.floor_file, exc))
                    return self.plane
            pl = fit_floor(F)
            if pl is None:
                self.get_logger().warn(
                    'floor plane not fitted from %d pts - assuming z=0'
                    % len(F))
                return self.plane
            if pl[4] > a.max_floor_rms or pl[6] < a.min_floor_tiles:
                flat = (0.0, 0.0, pl[5])
                self.get_logger().warn(
                    'floor fit rms %.3f m over %d tiles (need <%.3f over %d) '
                    '- using level plane at %+.3f m, no tilt'
                    % (pl[4], pl[6], a.max_floor_rms, a.min_floor_tiles,
                       pl[5]))
                self.plane, self.plane_tilt, self.plane_rms = flat, 0.0, pl[4]
                return self.plane
            if (self.plane is None
                    or abs(pl[3] - self.plane_tilt) > 0.05):
                self.get_logger().info(
                    'floor plane: tilt %.2f deg (%.1f cm/m), offset %+.3f m, '
                    'rms %.3f m, from %d pts'
                    % (pl[3], 100 * math.hypot(pl[0], pl[1]), pl[2], pl[4],
                       len(F)))
            self.plane, self.plane_tilt, self.plane_rms = pl[:3], pl[3], pl[4]
            return self.plane

        def pass_(self):
            P = self.window()
            if P is None:
                self.get_logger().warn('no cloud on %s yet' % a.cloud_topic)
                return
            t0 = time.time()
            cl = detect(P, a.min_cells, self.floor_plane(P))
            reg.update(cl, time.time())
            rp = self.robot()
            rx, ry = rp if rp else (0.0, 0.0)
            rank = reg.ranked(rx, ry)
            doc = dict(stamp=time.time(), robot=[rx, ry],
                       n_clusters=len(cl), n_entries=len(reg.e),
                       n_scans=len(self.win), n_pts=int(len(P)),
                       rejects={k: sum(1 for c in cl if c['cls'] == k)
                                for k in ('strip', 'small', 'sparse',
                                          'MERGED')},
                       side=SIDE, standoff=a.standoff,
                       tilt_deg=self.plane_tilt, floor_rms=self.plane_rms,
                       entries=[dict(e, standoff_poses=standoff_poses(
                           e, a.standoff, a.robot_radius)) for e in rank],
                       all_states={k: v['state'] for k, v in reg.e.items()})
            s = json.dumps(doc, default=float)
            with LOCK:
                STATE['json'] = s
            self.pub.publish(String(data=s))
            nxt = rank[0]['id'] if rank else '-'
            self.get_logger().info(
                '%d clusters -> %d entries (%d open)  next: %s  %.1f s'
                % (len(cl), len(reg.e), len(rank), nxt, time.time() - t0))

    class H(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def do_GET(self):
            with LOCK:
                b = STATE['json'].encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(b)))
            self.end_headers()
            self.wfile.write(b)

    rclpy.init()
    n = Node_()
    srv = ThreadingHTTPServer(('0.0.0.0', a.port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('table registry on http://0.0.0.0:%d' % a.port)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
