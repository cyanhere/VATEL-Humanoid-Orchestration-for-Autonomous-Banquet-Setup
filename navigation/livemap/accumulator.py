#!/usr/bin/env python3
"""Grow the localization cloud during a run, and persist it for the next one.

    /usr/bin/python3 ~/.scuba/growmap/accumulator.py

It publishes ONE topic, /accumulated_cloud, for nav2's memory_layer to mark
from - no command topic, no service client, no action client, so it still
cannot move the robot.

WHAT IT DOES
  /cloud_registered arrives in camera_init - FAST-LIO's frame, which on this
  robot is INVERTED and drifts. Points are transformed into `map` with the
  relocalizer's current map->camera_init before storing, so the file always has
  the floor at z=0 and matches what relocalizer_grow.py expects to load.
  Points are voxel-deduplicated and written atomically (temp + rename) so the
  relocalizer can never read a half-written map.

THE INSERTION GATE, which is the whole reason this is not a plain recorder
  Inserting while the pose is wrong writes a SECOND copy of a wall a metre from
  the first. After that every registration is ambiguous and the map degrades
  further on each pass, with no recovery. So insertion is gated on the inlier
  fraction published by the relocalizer (inliers discriminate; rmse does not):

     inliers > gate_hi   well localized          -> insert
     inliers < gate_lo   no overlap, new ground  -> insert (odometry-only)
     in between          partial/ambiguous       -> DO NOT INSERT

  and insertion also pauses for settle_s after any large map->camera_init jump,
  so a correction is never baked in mid-flight.

WHAT IT DOES NOT DO
  It cannot improve the map's absolute accuracy - whatever drift existed when a
  region was first seen is frozen in. What the cycle buys is that the live pose
  stops drifting RELATIVE to the map, which is what Nav2 needs, since it plans
  in the same frame.
"""
import argparse
import math
import os
import threading
import time

import numpy as np


def quat_to_R(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def voxel_keys(P, v):
    return np.floor(P / v).astype(np.int64)


def dedupe(P, v):
    """one point per voxel, keeping the first seen"""
    if len(P) == 0:
        return P
    k = voxel_keys(P, v)
    _, idx = np.unique(k, axis=0, return_index=True)
    return np.ascontiguousarray(P[np.sort(idx)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='~/.scuba/livemap/maps/live/live_cloud.npy')
    ap.add_argument('--cloud-topic', default='/cloud_registered')
    ap.add_argument('--inliers-topic', default='/relocalizer/inliers')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--voxel', type=float, default=0.15)
    ap.add_argument('--snapshot-s', type=float, default=20.0)
    # --- the viewer's live-map rule (path_viewer.py State.ingest) -----------
    ap.add_argument('--cell', type=float, default=0.05,
                    help='voxel edge for hit counting and for the stored map')
    ap.add_argument('--min-hits', type=int, default=2,
                    help='how many DIFFERENT scans must see a cell before it '
                         'enters the map. 1 disables the filter.')
    ap.add_argument('--band', type=float, default=0.0,
                    help='0 = OFF (default). If set, keep only points within '
                         'this many metres of the SENSOR height, as the viewer '
                         'live map does. DO NOT enable it for the stored map: '
                         'relocalizer_live measures the floor from this cloud '
                         'and registers against it, and at a ~1.25 m sensor '
                         'height a 1.0 m band excludes the floor entirely '
                         '(|0 - 1.25| > 1.0). The floor is what pins z and '
                         'tilt in ICP. The costmap already gets a banded view '
                         'through --pub-band.')
    ap.add_argument('--max-cells', type=int, default=2000000,
                    help='stop counting new cells past this, to bound memory')
    ap.add_argument('--keep-existing', action='store_true',
                    help='do NOT clear the map file at startup. By default the '
                         'mission starts from zero: the old map is removed so '
                         'the relocalizer cannot half-register against a room '
                         'that has since changed.')
    ap.add_argument('--gate-hi', type=float, default=0.50)
    ap.add_argument('--gate-lo', type=float, default=0.15)
    ap.add_argument('--settle-s', type=float, default=2.0)
    ap.add_argument('--jump-m', type=float, default=0.15,
                    help='map->odom step larger than this pauses insertion')
    ap.add_argument('--jump-deg', type=float, default=3.0)
    # A relocalizer RESTART re-seeds the origin at wherever the robot is now,
    # so `map` is redefined. Points already in the store were recorded against
    # the OLD origin; merging new ones on top produces a map that is several
    # different maps piled together - fragments, streaks, no room outline.
    # A step this large is never a registration correction, so treat it as a
    # new origin and start the store over.
    ap.add_argument('--reset-m', type=float, default=2.0)
    ap.add_argument('--reset-deg', type=float, default=30.0)
    ap.add_argument('--no-reset', action='store_true',
                    help='keep accumulating across an origin change (unsafe)')
    ap.add_argument('--z-band', type=float, nargs=2, default=[-0.5, 2.5])
    ap.add_argument('--max-points', type=int, default=3000000)
    # nav2's obstacle_layer marks from the LIVE scan only, and raytracing
    # clears ~130x more cells per scan than the scan marks, so the costmap
    # keeps no memory of anywhere the robot has been (measured: nothing beyond
    # 4.9 m despite a 60 m traverse). Republishing the accumulated map lets a
    # marking-only costmap layer remember it.
    ap.add_argument('--pub-topic', default='/accumulated_cloud')
    ap.add_argument('--pub-s', type=float, default=5.0,
                    help='0 disables publishing')
    ap.add_argument('--pub-voxel', type=float, default=0.10,
                    help='match the global costmap resolution; finer only '
                         'costs the costmap time, it cannot mark sub-cell')
    ap.add_argument('--pub-band', type=float, nargs=2, default=[0.20, 1.60],
                    help='publish only what the costmap would mark anyway')
    ap.add_argument('--no-gate', action='store_true',
                    help='insert unconditionally - for a deliberate clean '
                         'survey run where you know the pose is good')
    a = ap.parse_args()

    out = os.path.expanduser(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)

    import rclpy
    from rclpy.node import Node
    import tf2_ros
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Float32
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                           HistoryPolicy)

    def make_cloud(P, stamp, frame):
        """XYZ float32 PointCloud2."""
        m = PointCloud2()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.height = 1
        m.width = len(P)
        m.fields = [PointField(name=n, offset=4 * i,
                               datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z'))]
        m.is_bigendian = False
        m.point_step = 12
        m.row_step = 12 * len(P)
        m.is_dense = True
        m.data = np.ascontiguousarray(P, dtype=np.float32).tobytes()
        return m

    def pc2_xyz(msg):
        """x,y,z as float32, tolerating extra fields and row padding"""
        names = [f.name for f in msg.fields]
        for n in ('x', 'y', 'z'):
            if n not in names:
                return np.zeros((0, 3))
        off = {f.name: f.offset for f in msg.fields}
        step = msg.point_step
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        n = len(buf) // step
        buf = buf[:n * step].reshape(n, step)
        cols = []
        for k in ('x', 'y', 'z'):
            o = off[k]
            cols.append(buf[:, o:o + 4].copy().view(np.float32).ravel())
        P = np.stack(cols, axis=1).astype(np.float64)
        return P[np.isfinite(P).all(axis=1)]

    class Acc(Node):
        def __init__(self):
            super().__init__('map_accumulator')
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            self.lock = threading.Lock()
            self.store = np.zeros((0, 3))
            self.hits = {}             # packed cell key -> how many scans saw it
            self.capped = False
            self.pending = []
            # START FROM ZERO. The relocalizer reads this same file and reloads
            # it as it grows, so leaving the previous run's map in place makes
            # it register PARTIALLY against a room that has changed: measured
            # 2026-10-22, that held inliers at 0.41-0.44, inside the ambiguous
            # band, so the gate skipped 1749 scans and nothing was ever stored.
            # Removing the file puts the relocalizer in its documented
            # "no prior map" state instead, where inliers sit below gate_lo and
            # insertion proceeds on odometry until the map has substance.
            if not a.keep_existing and os.path.exists(out):
                try:
                    os.remove(out)
                    self.get_logger().info(
                        'removed the previous map %s - this mission builds '
                        'from zero' % out)
                except OSError as exc:
                    self.get_logger().error(
                        'cannot remove %s (%s) - the relocalizer may '
                        'half-register against it' % (out, exc))
            self.inliers = None
            self.inl_t = 0.0
            self.prev_T = None
            self.hold_until = 0.0
            self.n_in = self.n_skip_gate = self.n_skip_tf = self.n_skip_jump = 0
            self.snapshots = 0
            self.t0 = time.time()
            self.create_subscription(PointCloud2, a.cloud_topic, self.on_cloud, 1)
            self.create_subscription(Float32, a.inliers_topic, self.on_inl, 10)
            self.create_timer(a.snapshot_s, self.snapshot)
            self.create_timer(5.0, self.beat)
            self.pub = None
            if a.pub_s > 0:
                # TRANSIENT_LOCAL so a costmap that starts later still gets the
                # map without waiting for the next publish
                self.pub = self.create_publisher(
                    PointCloud2, a.pub_topic,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               history=HistoryPolicy.KEEP_LAST))
                self.create_timer(a.pub_s, self.publish_map)
            self.get_logger().info(
                'accumulating %s -> %s | cell %.2f m | min_hits %d | band '
                '%s | snapshot %.0f s | gate %.2f/%.2f%s%s'
                % (a.cloud_topic, out, a.cell, a.min_hits,
                   ('+/-%.2f m' % a.band) if a.band > 0 else 'off (floor kept)',
                   a.snapshot_s, a.gate_lo, a.gate_hi,
                   '  [GATE DISABLED]' if a.no_gate else '',
                   '  [KEEPING OLD MAP]' if a.keep_existing else '  [FROM ZERO]'))

        def on_inl(self, m):
            self.inliers = float(m.data)
            self.inl_t = time.time()

        def gate_ok(self):
            """None = insert, str = reason to skip"""
            if a.no_gate:
                return None
            now = time.time()
            if now < self.hold_until:
                return 'settling after a correction'
            if self.inliers is None or now - self.inl_t > 5.0:
                # no relocalizer yet: bootstrap, pure odometry, insert
                return None
            if self.inliers >= a.gate_hi or self.inliers <= a.gate_lo:
                return None
            return 'inliers %.2f in the ambiguous band' % self.inliers

        def on_cloud(self, msg):
            try:
                tr = self.buf.lookup_transform(a.map_frame, a.odom_frame,
                                               rclpy.time.Time())
            except Exception:
                self.n_skip_tf += 1
                return
            t = tr.transform.translation
            q = tr.transform.rotation
            R = quat_to_R(q.x, q.y, q.z, q.w)
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [t.x, t.y, t.z]

            # a big step in map->odom means the relocalizer just moved the world;
            # anything inserted across it lands in two places at once
            if self.prev_T is not None:
                d = float(np.linalg.norm(T[:3, 3] - self.prev_T[:3, 3]))
                dR = R @ self.prev_T[:3, :3].T
                ang = math.degrees(math.acos(
                    max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))))
                if (d > a.reset_m or ang > a.reset_deg) and not a.no_reset:
                    with self.lock:
                        n_lost = len(self.store)
                        self.store = np.zeros((0, 3))
                        self.hits = {}
                        self.capped = False
                        self.pending = []
                    self.prev_T = T
                    self.hold_until = time.time() + a.settle_s
                    self.get_logger().error(
                        'ORIGIN CHANGED (%.2f m / %.1f deg) - the relocalizer '
                        'was restarted or re-seeded. Discarded %d points '
                        'recorded against the old origin; starting over. '
                        'Restart this node too if you did not expect it.'
                        % (d, ang, n_lost))
                    return
                if d > a.jump_m or ang > a.jump_deg:
                    self.hold_until = time.time() + a.settle_s
                    self.n_skip_jump += 1
                    self.get_logger().warn(
                        'map->odom jumped %.2f m / %.1f deg - pausing insertion '
                        '%.1f s' % (d, ang, a.settle_s))
            self.prev_T = T

            why = self.gate_ok()
            if why:
                self.n_skip_gate += 1
                return
            P = pc2_xyz(msg)
            if len(P) == 0:
                return
            P = P @ T[:3, :3].T + T[:3, 3]           # camera_init -> map
            # The viewer bands about the sensor because it draws a 2D
            # obstacle picture and wants the floor OUT. The stored map is a
            # different consumer: the relocalizer measures the floor from it
            # and uses that plane to pin z and tilt, so banding it here would
            # throw away the strongest ICP constraint. Off unless asked.
            if a.band > 0:
                P = P[np.abs(P[:, 2] - float(T[2, 3])) < a.band]
            P = P[(P[:, 2] > a.z_band[0]) & (P[:, 2] < a.z_band[1])]
            if len(P) == 0:
                return
            # --- the live-map rule: count each cell at most ONCE per scan, and
            # admit it only once --min-hits DIFFERENT scans agree. A single
            # scan can smear a walking person across open floor; two scans
            # agreeing is the cheapest filter that rejects that, and it is
            # exactly what the viewer's min_hits does.
            ij = np.floor(P / a.cell).astype(np.int64)
            if np.abs(ij).max() >= 32768:
                ij_ok = (np.abs(ij) < 32768).all(axis=1)
                P, ij = P[ij_ok], ij[ij_ok]
                if len(P) == 0:
                    return
            key = (((ij[:, 0] + 32768) * 65536 + (ij[:, 1] + 32768)) * 65536
                   + (ij[:, 2] + 32768))
            uk, idx = np.unique(key, return_index=True)
            new = []
            with self.lock:
                hits, need = self.hits, a.min_hits
                room = (len(hits) < a.max_cells)
                for k_, i_ in zip(uk.tolist(), idx.tolist()):
                    c = hits.get(k_)
                    if c is None:
                        if not room:
                            continue
                        c = 0
                    c += 1
                    hits[k_] = c
                    if c == need:
                        new.append(P[i_])
                if not room and not self.capped:
                    self.capped = True
                    self.get_logger().warn(
                        'cell table hit --max-cells %d; no NEW cells will be '
                        'counted (known ones still accumulate)' % a.max_cells)
                if new:
                    self.pending.append(np.asarray(new))
            self.n_in += 1

        def snapshot(self):
            with self.lock:
                pend, self.pending = self.pending, []
            if not pend and self.snapshots:
                return
            allp = [self.store] + pend if len(self.store) else pend
            if not allp:
                return
            M = dedupe(np.vstack(allp), a.cell)
            if len(M) > a.max_points:
                M = M[np.random.choice(len(M), a.max_points, replace=False)]
                self.get_logger().warn('map capped at %d points' % a.max_points)
            self.store = M
            # ends in .npy so np.save does not append it, then rename over
            # the real file: the relocalizer can never see a partial map
            tmp = out + '.tmp.npy'
            np.save(tmp, M.astype(np.float32))
            os.replace(tmp, out)
            self.snapshots += 1
            self.get_logger().info(
                'snapshot %d: %d points written (%.1f MB)'
                % (self.snapshots, len(M), os.path.getsize(out) / 1e6))

        def publish_map(self):
            with self.lock:
                M = self.store
            if self.pub is None or len(M) == 0:
                return
            lo, hi = a.pub_band
            B = M[(M[:, 2] > lo) & (M[:, 2] < hi)]
            if len(B) == 0:
                return
            B = dedupe(B, a.pub_voxel)
            self.pub.publish(make_cloud(B, self.get_clock().now().to_msg(),
                                        a.map_frame))
            self.pub_n = getattr(self, 'pub_n', 0) + 1
            if self.pub_n % 6 == 1:
                self.get_logger().info(
                    'published %d pts (band %.2f-%.2f, voxel %.2f) to %s'
                    % (len(B), lo, hi, a.pub_voxel, a.pub_topic))

        def beat(self):
            g = ('inliers %.2f' % self.inliers) if self.inliers is not None \
                else 'no inliers yet'
            self.get_logger().info(
                '%5.0fs  stored %d | scans in %d  skipped: gate %d, tf %d, '
                'jump %d | %s' % (time.time() - self.t0, len(self.store),
                                  self.n_in, self.n_skip_gate, self.n_skip_tf,
                                  self.n_skip_jump, g))

    rclpy.init()
    n = Acc()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.snapshot()
    print('\nfinal map: %d points -> %s' % (len(n.store), out))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
