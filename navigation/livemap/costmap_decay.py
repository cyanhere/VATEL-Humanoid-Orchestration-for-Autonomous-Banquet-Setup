#!/usr/bin/env python3
"""A global memory grid that FORGETS: moving-human trails fade, walls persist.

WHY NOT THE EXISTING memory_layer
  global_costmap's memory_layer is an ObstacleLayer on /accumulated_cloud with
  clearing:false and combination_method Max. Max means the live layer can only
  ADD to it, and clearing:false means nothing ever removes a mark - so a person
  who walks through is remembered at every cell they passed, for the rest of
  the run, whatever the accumulator later publishes. The config says so itself:
  "a table that MOVES stays remembered at its old spot".

WHAT THIS DOES INSTEAD
  Publishes an OccupancyGrid for a nav2 StaticLayer, whose incomingMap()
  REPLACES the layer's contents, so a cell this node drops really disappears
  from the planner's costmap.

  Evidence per cell, updated once per --decay-s window, and ONLY inside a
  --box metre square centred on the robot:

      cell hit at least once in the window  ->  credit = min(credit+1, --cap)
      cell not hit, but inside the box      ->  credit -= 1
      credit <= 0                           ->  cell forgotten

  Outside the box nothing changes: the lidar has no current opinion there, so
  neither do we. That is the whole trick - decay only where we can see.

  A window, not a scan, because the Mid-360 returns ~1240 points per scan and a
  6x6 m box at 0.10 m holds 3600 cells. Per-scan misses would erase walls
  faster than they are re-seen. Over a 2 s window (~20 scans, ~25k points) a
  real surface is hit repeatedly; a person who has moved on is not.

  --cap sets how long a surface survives occlusion: a wall at full credit
  outlives --cap windows hidden behind something. A person lingering N windows
  only reaches credit N, so they fade in about N windows after moving.

GEOMETRY. The grid is published at exactly the global costmap's size, origin
and resolution. nav2's StaticLayer resizes a non-rolling master costmap to
match whatever map it receives, so a mismatch here would silently resize the
planner's whole costmap.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/costmap_decay.py

Publishes ONE topic. No command topic, no action client: it cannot move the
robot. Inspect it before wiring it into nav2 with
    path_viewer.py --costmap-topic /memory_grid
"""
import argparse
import array
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
import tf2_ros


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pc2_xyz(msg):
    """Foxy has no sensor_msgs_py: read x/y/z by field offset."""
    off = {f.name: f.offset for f in msg.fields}
    for k in ('x', 'y', 'z'):
        if k not in off:
            return np.zeros((0, 3))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = len(raw) // msg.point_step
    if n == 0:
        return np.zeros((0, 3))
    grid = raw[:n * msg.point_step].reshape(n, msg.point_step)
    cols = []
    for k in ('x', 'y', 'z'):
        o = off[k]
        cols.append(np.ascontiguousarray(grid[:, o:o + 4]).view(np.float32).ravel())
    P = np.stack(cols, axis=1).astype(np.float64)
    return P[np.isfinite(P).all(axis=1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cloud-topic', default='/cloud_registered')
    ap.add_argument('--pub-topic', default='/memory_grid')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    # match global_costmap exactly, or StaticLayer resizes the master costmap
    ap.add_argument('--width', type=float, default=60.0)
    ap.add_argument('--height', type=float, default=60.0)
    ap.add_argument('--res', type=float, default=0.05,
                    help='MUST equal global_costmap resolution: nav2 StaticLayer\n'
                         'resizes a non-rolling master costmap to match the map it\n'
                         'receives, so a mismatch silently reshapes the planner grid')
    ap.add_argument('--origin', type=float, nargs=2, default=[-30.0, -30.0])
    # the decay rule
    ap.add_argument('--box', type=float, default=6.0,
                    help='side of the robot-centred square where decay applies. '
                         '0 = UNLIMITED: decay wherever there is free evidence. '
                         'The box was a proxy for "can we judge this cell", '
                         'needed when a miss was only "not hit". Raytracing '
                         'carries its own visibility test, so the box is now '
                         'redundant - and it is SMALLER than the raytrace '
                         'reach (3 m per axis against 5 m of free evidence).')
    ap.add_argument('--decay-s', type=float, default=2.0,
                    help='evaluation window')
    ap.add_argument('--cap', type=int, default=10,
                    help='max credit; how many windows a surface survives occlusion')
    # --- density: accumulate before forgetting -----------------------------
    # The Mid-360 returns ~1240 points per scan. A 6 m box at 0.05 m holds
    # 14400 cells, so ONE window cannot touch most of them, and charging a miss
    # every window erodes real surfaces faster than they are re-seen. Finer
    # resolution makes this worse, not better: 4x the cells for the same points.
    ap.add_argument('--warmup-s', type=float, default=20.0,
                    help='accumulate ONLY for this long after start - hits are '
                         'counted, misses are not charged. Lets the map reach '
                         'density before anything can be forgotten. 0 = off.')
    ap.add_argument('--decay-every', type=int, default=3,
                    help='charge the miss penalty only every Nth window. Hits '
                         'still count every window, so N>1 means evidence '
                         'accumulates faster than it decays. Occlusion grace '
                         'becomes cap * decay_every * decay_s.')
    ap.add_argument('--occ-min', type=int, default=2,
                    help='credit needed before a cell is published as lethal')
    # which points count, matching the costmap's own band
    ap.add_argument('--z-band', type=float, nargs=2, default=[0.20, 1.60])
    ap.add_argument('--max-age-s', type=float, default=0.0,
                    help='AGE BACKSTOP, seconds. 0 = off. A confirmed cell that '
                         'no beam has HIT for this long is dropped outright, '
                         'whether or not anything proved it empty. This is what '
                         "nav2's spatio_temporal_voxel_layer does: it decays on "
                         'AGE and uses visibility only to accelerate. Ray '
                         'evidence can only reach --raytrace-range, so without '
                         'this a person walking further out leaves a permanent '
                         'trail. Applied GLOBALLY, independent of --box. The '
                         'accepted cost is the same one STVL accepts: real '
                         'static structure driven away from and never re-seen '
                         'also expires. Macenski recommends 15-45 s for a '
                         'global costmap for exactly this balance.')
    ap.add_argument('--raytrace-range', type=float, default=5.0,
                    help='metres. Mark cells a beam passed THROUGH as FREE '
                         'evidence, so a miss means "known empty" instead of '
                         '"not hit". 0 disables it and restores the old '
                         'window-counting behaviour, for an A/B in one run.')
    ap.add_argument('--clear-radius', type=float, default=0.35,
                    help='always keep this radius around the robot FREE. '
                         'nav2 ObstacleLayer does this itself via '
                         'footprint_clearing_enabled; StaticLayer has no such '
                         'option, so the grid must do it. Without it a stray '
                         'self-return or a cell remembered from before the '
                         'robot stood there makes its own start cell lethal, '
                         'and NavFn then reports no path to anywhere - '
                         'measured 2026-10-22: 34 of 49 cells within 0.35 m '
                         'of the robot were >= 253 and every goal failed.')
    ap.add_argument('--log-s', type=float, default=5.0)
    a = ap.parse_args()

    W = int(round(a.width / a.res))
    H = int(round(a.height / a.res))
    OX, OY = a.origin

    rclpy.init()
    n = Node('costmap_decay')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    stats_decay = {'n': 0}
    t_start = time.time()
    credit = np.zeros((H, W), dtype=np.int16)
    # HYSTERESIS. Without it a cell at credit == occ_min that misses a single
    # window drops below the threshold and disappears, so an intermittently hit
    # wall flickers in and out of the planner's costmap - measured on a static
    # room: 22 deaths in 7 windows with nothing actually moving. So: a cell is
    # ADMITTED when its credit reaches occ_min, and only FORGOTTEN when the
    # credit decays all the way to zero. Rise is slow, fall is slow, and the
    # threshold is crossed once in each direction instead of repeatedly.
    confirmed = np.zeros((H, W), dtype=bool)
    seen = np.zeros((H, W), dtype=bool)
    # FREE evidence: a beam went through this cell and terminated further away.
    # Distinct from "not in seen", which also covers OCCLUDED and UNOBSERVED.
    cleared = np.zeros((H, W), dtype=bool)
    # window index when each cell was last HIT. -2**30 = never seen, so an
    # unseen cell's age is enormous - harmless, because expiry only touches
    # cells that are currently confirmed.
    last_seen = np.full((H, W), -(2 ** 30), dtype=np.int32)
    stats = {'scans': 0, 'pts': 0, 'born': 0, 'died': 0, 'windows': 0,
             'skip_tf': 0, 'marked_out': 0, 'footprint': 0, 'rays': 0,
             'w_hit': 0, 'w_free': 0, 'w_occl': 0, 'expired': 0, 'w_obs': 0}

    pub = n.create_publisher(
        OccupancyGrid, a.pub_topic,
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST))

    def on_cloud(msg):
        try:
            tr = buf.lookup_transform(a.map_frame, a.odom_frame,
                                      rclpy.time.Time())
        except Exception:
            stats['skip_tf'] += 1
            return
        Praw = pc2_xyz(msg)
        if len(Praw) == 0:
            return
        t = tr.transform.translation
        q = tr.transform.rotation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        Pall = Praw @ R.T + np.array([t.x, t.y, t.z])
        stats['scans'] += 1
        # MARK: only a return INSIDE the band occupies a cell
        P = Pall[(Pall[:, 2] > a.z_band[0]) & (Pall[:, 2] < a.z_band[1])]
        if len(P):
            i = np.floor((P[:, 0] - OX) / a.res).astype(np.int64)
            j = np.floor((P[:, 1] - OY) / a.res).astype(np.int64)
            ok = (i >= 0) & (i < W) & (j >= 0) & (j < H)
            if ok.any():
                seen[j[ok], i[ok]] = True
                stats['pts'] += int(ok.sum())
        # CLEAR: every return casts a ray, banded by its interpolated height
        if a.raytrace_range > 0:
            raytrace(Pall)

    n.create_subscription(PointCloud2, a.cloud_topic, on_cloud, 1)

    def sensor_origin():
        """The LIDAR's position in the map frame, as metres.

        map->base_frame, NOT camera_init's origin: /cloud_registered is already
        expressed in the camera_init WORLD frame, so that frame's origin is
        wherever FAST-LIO started, not where the sensor is. Raytracing from
        there would clear a corridor from the map origin to every return.
        Sanity check: z should be ~1.25, the Mid-360's height.
        """
        for base in (a.base_frame, 'base_link'):
            try:
                tr = buf.lookup_transform(a.map_frame, base, rclpy.time.Time())
            except Exception:
                continue
            t = tr.transform.translation
            return np.array([t.x, t.y, t.z], dtype=np.float64)
        return None

    def raytrace(Pall):
        """Mark cells the beams passed through as FREE evidence.

        The grid is a 2D projection of a 3D sensor, so the ray's HEIGHT is
        interpolated along it and a sample clears only where that height is
        inside the marking band. Without that, a beam passing OVER a table
        would clear the table's own column. This is stricter than nav2's
        ObstacleLayer, which approximates it by only raytracing points that
        already passed the height filter - here every return casts a ray, so
        there is MORE free evidence, which is what the density problem wants.
        """
        org = sensor_origin()
        if org is None:
            return
        d = Pall[:, :2] - org[:2]
        L = np.hypot(d[:, 0], d[:, 1])
        use = L > a.res
        if not use.any():
            return
        d, Lu, Pz = d[use], L[use], Pall[use, 2]
        u = d / Lu[:, None]
        K = int(a.raytrace_range / a.res) + 1
        ts = (np.arange(K, dtype=np.float64) * a.res)[None, :]
        # stop one cell short of the return: never clear the cell that was HIT
        lim = np.minimum(Lu, a.raytrace_range)[:, None] - a.res
        m = (ts > 0.0) & (ts <= lim)
        if not m.any():
            return
        # interpolated height of the beam at each sample
        z = org[2] + (ts / Lu[:, None]) * (Pz - org[2])[:, None]
        m &= (z > a.z_band[0]) & (z < a.z_band[1])
        if not m.any():
            return
        ii = np.floor((org[0] + u[:, 0:1] * ts - OX) / a.res).astype(np.int64)
        jj = np.floor((org[1] + u[:, 1:2] * ts - OY) / a.res).astype(np.int64)
        m &= (ii >= 0) & (ii < W) & (jj >= 0) & (jj < H)
        if m.any():
            cleared[jj[m], ii[m]] = True
        stats['rays'] += int(len(Lu))

    def robot_cell():
        for base in (a.base_frame, 'base_link'):
            try:
                tr = buf.lookup_transform(a.map_frame, base, rclpy.time.Time())
            except Exception:
                continue
            t = tr.transform.translation
            return (int(math.floor((t.x - OX) / a.res)),
                    int(math.floor((t.y - OY) / a.res)))
        return None

    def window():
        """One decay step, then publish."""
        rc = robot_cell()
        if rc is None:
            seen[:] = False
            cleared[:] = False
            return
        ci, cj = rc
        if a.box > 0:
            half = int(round((a.box / 2.0) / a.res))
            i0, i1 = max(0, ci - half), min(W, ci + half + 1)
            j0, j1 = max(0, cj - half), min(H, cj + half + 1)
        else:
            # UNLIMITED. A cell earns free evidence only when a beam went
            # THROUGH it and terminated further away, so raytracing already
            # answers "can we judge this cell" - at any range. Cells we cannot
            # see earn nothing and are left alone wherever they are, which is
            # what the box used to approximate badly.
            i0, i1, j0, j1 = 0, W, 0, H
        if i1 <= i0 or j1 <= j0:
            seen[:] = False
            cleared[:] = False
            return

        # The robot cannot be an obstacle to itself. Do this BEFORE the
        # hit/miss update so a self-return cannot even earn credit.
        if a.clear_radius > 0:
            cr = max(1, int(round(a.clear_radius / a.res)))
            fi0, fi1 = max(0, ci - cr), min(W, ci + cr + 1)
            fj0, fj1 = max(0, cj - cr), min(H, cj + cr + 1)
            if fi1 > fi0 and fj1 > fj0:
                yy, xx = np.ogrid[fj0:fj1, fi0:fi1]
                disc = ((xx - ci) ** 2 + (yy - cj) ** 2) <= cr * cr
                csub = credit[fj0:fj1, fi0:fi1]
                fsub = confirmed[fj0:fj1, fi0:fi1]
                ssub = seen[fj0:fj1, fi0:fi1]
                stats['footprint'] += int((fsub & disc).sum())
                csub[disc] = 0
                fsub[disc] = False
                ssub[disc] = False

        box_c = credit[j0:j1, i0:i1]
        box_s = seen[j0:j1, i0:i1]
        box_f = confirmed[j0:j1, i0:i1]
        before = int(box_f.sum())

        box_cl = cleared[j0:j1, i0:i1]
        # reward what we saw
        np.minimum(box_c + 1, a.cap, out=box_c, where=box_s)
        # Charge ONLY cells with real free evidence - a beam went through them
        # and stopped further away. A cell with NEITHER hit nor free evidence is
        # OCCLUDED or unobserved and is left alone. Before raytracing every such
        # cell was charged, which is why the decay needed --warmup-s and
        # --decay-every to stop it eroding real walls.
        free = box_cl & ~box_s
        stats['w_hit'] = int(box_s.sum())
        stats['w_free'] = int(free.sum())
        stats['w_occl'] = int((~box_s & ~box_cl).sum())
        # cells with ANY evidence this window. More informative than w_occl,
        # which in --box 0 counts the whole 60x60 m grid and reads as noise.
        stats['w_obs'] = stats['w_hit'] + stats['w_free']
        # misses are charged only sometimes: during warm-up never, and
        # afterwards one window in --decay-every. Hits always count, so
        # evidence builds faster than it fades.
        warm = (a.warmup_s > 0 and (time.time() - t_start) < a.warmup_s)
        stats_decay['n'] += 1
        charge = (not warm) and (stats_decay['n'] % max(1, a.decay_every) == 0)
        if charge:
            box_c[free] -= 1
        np.clip(box_c, 0, a.cap, out=box_c)

        # admit at occ_min, forget only at zero
        box_f |= (box_c >= a.occ_min)
        box_f &= (box_c > 0)

        # ---- AGE BACKSTOP, global ------------------------------------------
        # Ray evidence reaches only --raytrace-range, so a cell further out can
        # never be proved empty and would keep its mark for ever. Decay it on
        # AGE instead: a confirmed cell no beam has hit for --max-age-s is
        # dropped. Global on purpose - this is the half that does NOT depend on
        # the box, and it is what makes a person walking at 8 m clear at all.
        widx = stats['windows']
        last_seen[seen] = widx
        if a.max_age_s > 0:
            age_w = max(1, int(round(a.max_age_s / a.decay_s)))
            stale = confirmed & ((widx - last_seen) > age_w)
            k_exp = int(stale.sum())
            if k_exp:
                credit[stale] = 0
                confirmed[stale] = False
                stats['expired'] += k_exp

        # ---- OUTSIDE the box: the ORIGINAL behaviour ------------------
        # A return marks the cell and nothing ever removes it, exactly as the
        # old ObstacleLayer memory_layer did. Decay applies only where the
        # lidar has a current opinion, and out here it does not.
        #
        # A cell marked out here is admitted with occ_min credit, not cap, so
        # when the robot later comes close enough to judge it: a real surface
        # is re-seen immediately (it is within metres) and climbs to cap, while
        # a stray return or a person who has walked on fades in occ_min
        # windows. Giving it cap instead would make every distant one-off
        # return take the full occlusion grace period to clear.
        out = seen.copy()
        out[j0:j1, i0:i1] = False
        newly = out & ~confirmed
        k_new = 0 if a.box <= 0 else int(newly.sum())
        if k_new:
            # item assignment, NOT `confirmed |= newly`: an augmented
            # assignment to the name would make `confirmed` local to this
            # function and shadow the grid, which is exactly the
            # UnboundLocalError this hit the first time.
            confirmed[newly] = True
            credit[newly] = a.occ_min
            stats['marked_out'] += k_new

        after = int(box_f.sum())
        if after > before:
            stats['born'] += after - before
        else:
            stats['died'] += before - after
        stats['windows'] += 1
        seen[:] = False
        cleared[:] = False

        g = OccupancyGrid()
        g.header.frame_id = a.map_frame
        g.header.stamp = n.get_clock().now().to_msg()
        g.info.resolution = a.res
        g.info.width = W
        g.info.height = H
        g.info.origin.position.x = OX
        g.info.origin.position.y = OY
        g.info.origin.orientation.w = 1.0
        occ = np.where(confirmed, np.int8(100), np.int8(0))
        # array.array, not .tolist(): at 0.05 m this is 1.44e6 cells and
        # tolist() alone costs hundreds of ms, which would eat the window.
        g.data = array.array('b', occ.ravel().astype(np.int8).tobytes())
        pub.publish(g)

    n.create_timer(a.decay_s, window)

    t0 = time.time()

    def beat():
        live = int(confirmed.sum())
        held = int((credit > 0).sum())
        n.get_logger().info(
            '%4.0fs  lethal %6d | any credit %6d | windows %4d | scans %5d '
            '| in-box born %5d died %5d | marked outside %6d | '
            'footprint cleared %5d | skip_tf %d | hit/free %d/%d '
            '(obs %d = %.1f%% of grid) | expired %5d | rays %d%s'
            % (time.time() - t0, live, held, stats['windows'], stats['scans'],
               stats['born'], stats['died'], stats['marked_out'],
               stats['footprint'], stats['skip_tf'],
               stats['w_hit'], stats['w_free'], stats['w_obs'],
               100.0 * stats['w_obs'] / float(W * H), stats['expired'],
               stats['rays'],
               '  [WARM-UP]' if (a.warmup_s > 0
                                 and time.time() - t_start < a.warmup_s)
               else ''))

    n.create_timer(a.log_s, beat)

    n.get_logger().info(
        'memory grid %dx%d @%.2f m origin %.1f,%.1f -> %s | band %.2f..%.2f'
        % (W, H, a.res, OX, OY, a.pub_topic, a.z_band[0], a.z_band[1]))
    n.get_logger().info(
        '%s: hits every %.1f s, misses every %d window(s), cap %d, '
        'admit at %d, forget at 0'
        % (('INSIDE a %.1f m box on the robot' % a.box) if a.box > 0
           else 'EVERYWHERE there is free evidence (--box 0, no distance limit)',
           a.decay_s, a.decay_every, a.cap, a.occ_min))
    n.get_logger().info(
        'age backstop: %s' % (
            ('a confirmed cell unhit for %.0f s (%d windows) is DROPPED, '
             'globally, regardless of --box - this is the only thing that '
             'clears a trail beyond the %.1f m ray reach'
             % (a.max_age_s, max(1, int(round(a.max_age_s / a.decay_s))),
                a.raytrace_range))
            if a.max_age_s > 0 else
            'OFF - nothing beyond the %.1f m ray reach can ever be forgotten'
            % a.raytrace_range))
    n.get_logger().info(
        'warm-up %.0f s with NO decay; occlusion grace %.0f s '
        '(cap %d x every %d x %.1f s)'
        % (a.warmup_s, a.cap * a.decay_every * a.decay_s, a.cap,
           a.decay_every, a.decay_s))
    n.get_logger().info(
        'raytrace %s: a MISS now means a beam went THROUGH the cell, not '
        'merely that it was not hit; cells with neither are occluded and are '
        'left alone'
        % (('%.1f m' % a.raytrace_range) if a.raytrace_range > 0
           else 'OFF (window counting)'))
    if a.box > 0:
        n.get_logger().info(
            'OUTSIDE it: mark-only, additive, never removed - the original '
            'behaviour. Cells enter the box with %d credit and are re-judged '
            'there.' % a.occ_min)
    else:
        n.get_logger().info(
            'No outside regime: a cell beyond the beams simply earns nothing '
            'and keeps whatever credit it had, so distant marks persist until '
            'a beam actually passes through them. Decay reach is therefore '
            'set by --raytrace-range (%.1f m), not by a box.'
            % a.raytrace_range)
    n.get_logger().info(
        'admitted at credit %d, forgotten at 0 (hysteresis). A surface at full '
        'credit survives %d windows (%.0f s) of occlusion; a transient seen in '
        'k windows fades ~k windows (%.0f s for k=%d) after it moves on'
        % (a.occ_min, a.cap, a.cap * a.decay_s, a.occ_min * a.decay_s,
           a.occ_min))
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
