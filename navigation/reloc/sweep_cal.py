#!/usr/bin/env python3
"""Guided manual sweep: push the robot on a rack, measure what the stack thinks.

The robot does NOT walk. You move it by hand while this prints one instruction
at a time ("MOVE FORWARD 1.00 m"), and after each move it records what odometry
and localization report. The point is evidence, not autonomy: where in the room
localization is trustworthy, how far odometry drifts, and how much error to
expect when you set Nav2's tolerances.

    # terminal A: ~/.scuba/lio_ws/run_lio.sh     (odometry)
    # terminal B: relocalizer.py                 (map pose)
    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/reloc/sweep_cal.py

    # custom route, in your own words
    /usr/bin/python3 ~/.scuba/reloc/sweep_cal.py --route "F1,L90,F2,R90,B1"

ROUTE TOKENS
    F<m>    forward          B<m>   back
    SL<m>   strafe left      SR<m>  strafe right
    L<deg>  turn left        R<deg> turn right
    P       pause and sample where you are (no movement)

The default route is a closed loop that returns to the start, so the gap between
where it started and where the stack thinks it ended is a direct read of total
drift.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import globloc as G

DEF_MAP = '~/.scuba/maps/ilounge_02_loc_cloud.npy'
GATES = (1.5, 0.8, 0.4, 0.25)
DEF_ROUTE = ('P,F1,P,F1,P,L90,F1,P,F1,P,L90,F1,P,F1,P,L90,F1,P,F1,P,L90,P')
# A closed 2 m square: it ENDS WHERE IT STARTED, facing the same way,
# so the start-to-end gap really is the accumulated drift. (The old
# default finished 1 m from its start, which made that figure junk.)


def parse_route(s):
    out = []
    for tok in [t.strip().upper() for t in s.split(',') if t.strip()]:
        if tok == 'P':
            out.append(('pause', 0.0)); continue
        m = re.match(r'^(SL|SR|F|B|L|R)([0-9]*\.?[0-9]+)$', tok)
        if not m:
            raise ValueError('bad route token %r' % tok)
        k, v = m.group(1), float(m.group(2))
        out.append(({'F': 'forward', 'B': 'back', 'SL': 'strafe_left',
                     'SR': 'strafe_right', 'L': 'turn_left',
                     'R': 'turn_right'}[k], v))
    if not out:
        raise ValueError('route is empty')
    return out


# "left" must mean the ROBOT's left, not the pusher's. Standing behind the
# rack these agree; standing in front of it they are opposite, which silently
# inverts every turn measurement.
HUMAN = {'forward': "MOVE FORWARD %.2f m  (direction the robot's chest faces)",
         'back': 'MOVE BACK %.2f m',
         'strafe_left': "SLIDE %.2f m to the ROBOT's LEFT",
         'strafe_right': "SLIDE %.2f m to the ROBOT's RIGHT",
         'turn_left': "TURN LEFT %.0f deg      (toward the robot's own left)",
         'turn_right': "TURN RIGHT %.0f deg     (toward the robot's own right)",
         'pause': 'HOLD STILL (sampling where you are)'}

# Sign relating a physical "anticlockwise seen from above" turn to the yaw
# reported by FAST-LIO/TF. +1 is the ROS convention (REP-103: CCW positive).
# Set -1 with --turn-sign if measured yaw runs the other way on this robot.
TURN_SIGN = 1.0


# expected body-frame delta (dx, dy, dyaw_rad) for each move
def expected(kind, v):
    t = TURN_SIGN * math.radians(v)
    return {'forward': (v, 0.0, 0.0), 'back': (-v, 0.0, 0.0),
            'strafe_left': (0.0, v, 0.0), 'strafe_right': (0.0, -v, 0.0),
            'turn_left': (0.0, 0.0, t),
            'turn_right': (0.0, 0.0, -t),
            'pause': (0.0, 0.0, 0.0)}[kind]


class Sweep(Node):
    def __init__(self, no_map=False):
        super().__init__('sweep_cal')
        self.no_map = no_map
        self.accum = []
        self.create_subscription(PointCloud2, '/cloud_registered', self._cb, 5)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)

    def _cb(self, msg):
        P = pc2_xyz(msg)
        P = P[(P[:, 2] > -0.5) & (P[:, 2] < 2.5)]
        if len(P):
            self.accum.append(P)
            if len(self.accum) > 6:
                self.accum.pop(0)

    def tf(self, parent, child):
        """Full 4x4 transform, roll and pitch included.

        Rebuilding a pose from yaw alone throws away the mount's tilt. A rack
        holds the lidar at a different tilt than the walking robot the map was
        built from, and 3 deg of tilt puts a point at 5 m range 26 cm off its
        surface - well outside the correspondence gate. Scoring that way reads
        as a localization failure when localization is in fact fine, because
        the relocalizer's own ICP solves all six degrees of freedom.
        """
        t = self.buf.lookup_transform(parent, child, rclpy.time.Time())
        q, v = t.transform.rotation, t.transform.translation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = (v.x, v.y, v.z)
        return T

    def sample(self, node_secs=1.5):
        """Spin briefly, then return (T_map_body, T_odom_body, scan)."""
        self.accum = []
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < node_secs:
            rclpy.spin_once(self, timeout_sec=0.1)
        To = self.tf('camera_init', 'body')
        # In no-map mode the odom frame IS the reference frame. camera_init is
        # gravity-aligned at IMU init, so turn angles and tilt stay meaningful;
        # only map-relative quantities are lost.
        Tm = To if self.no_map else self.tf('map', 'body')
        scan = np.vstack(self.accum) if self.accum else None
        return Tm, To, scan


def pc2_xyz(msg):
    n = msg.width * msg.height
    off = {f.name: f.offset for f in msg.fields}
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float64)
    for i, ax in enumerate('xyz'):
        out[:, i] = buf[:, off[ax]:off[ax] + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_yaw(T):
    return math.atan2(T[1, 0], T[0, 0])


def dir_word(dyaw_deg):
    """LEFT or RIGHT for a yaw change.

    The map frame is right-handed and z-up: yaw 0 faces +x, yaw +90 faces +y.
    So turning LEFT increases yaw and turning RIGHT decreases it. Verified from
    sweep run 2 - at yaw +93 a forward move advanced +y, at yaw +9 it advanced
    +x.
    """
    return 'LEFT' if dyaw_deg > 0 else 'RIGHT'


def T_tilt(T):
    """Angle in degrees between the body's up-axis and the map's up-axis."""
    return math.degrees(math.acos(max(-1.0, min(1.0, T[2, 2]))))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default=DEF_ROUTE)
    ap.add_argument('--map', default=DEF_MAP)
    ap.add_argument('--out', default='~/.scuba/maps/sweep_cal.csv')
    ap.add_argument('--settle', type=float, default=1.5,
                    help='seconds to sample after each move')
    ap.add_argument('--no-map', action='store_true',
                    help='odometry only: no prior map needed. Measures turn '
                         'direction, turn angle and distance from FAST-LIO '
                         'alone, so it works in a room you have not mapped. '
                         'Localization numbers (inliers, rmse, jitter) are '
                         'not available in this mode.')
    ap.add_argument('--turn-sign', type=float, default=1.0, choices=[1.0, -1.0],
                    help='-1 if an anticlockwise turn makes the reported yaw '
                         'DECREASE on this robot (default +1, the ROS convention)')
    a = ap.parse_args()

    global TURN_SIGN
    TURN_SIGN = a.turn_sign
    if TURN_SIGN < 0:
        print('turn-sign -1: an anticlockwise turn is expected to DECREASE yaw.')

    try:
        route = parse_route(a.route)
    except ValueError as e:
        sys.exit('!! %s' % e)

    nn = None
    if a.no_map:
        print('NO-MAP MODE: odometry only. Turn direction, turn angle and '
              'distance are measured;\n              inliers / rmse / '
              'jitter-at-rest are NOT (they need a prior map of THIS room).')
    else:
        mp = os.path.expanduser(a.map)
        if not os.path.exists(mp):
            sys.exit('!! missing %s' % mp)
        print('loading map ...')
        M = np.load(mp).astype(np.float64)
        M = M[(M[:, 2] > -0.5) & (M[:, 2] < 2.5)]
        M = G.voxel_downsample(M, 0.10)
        nn = G.MultiNN(M, GATES)

    rclpy.init()
    n = Sweep(no_map=a.no_map)
    print('waiting for TF and /cloud_registered ...')
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
        try:
            n.tf('map', 'body'); n.tf('camera_init', 'body')
            if n.accum:
                break
        except Exception:
            pass
    else:
        pass
    try:
        n.tf('camera_init', 'body')
    except Exception as e:
        sys.exit('!! no camera_init->body TF (%s)\n   terminal A (FAST-LIO) '
                 'must be running.' % e)
    if not a.no_map:
        try:
            n.tf('map', 'body')
        except Exception as e:
            sys.exit('!! no map->body TF (%s)\n   terminal B (relocalizer) must '
                     'be running too.\n   If this room is not mapped, re-run '
                     'with --no-map.' % e)

    print("""
================================================================
 MANUAL SWEEP - the robot does not walk. You push it.
 Keep the rack height and the lidar tilt the SAME throughout,
 and the same as when the map was recorded, or the scan will
 sit at the wrong height against the map.
 Move smoothly, stop fully, THEN press Enter.
================================================================
""")
    rows, first, prev = [], None, None
    for i, (kind, v) in enumerate(route, 1):
        print('-' * 60)
        print(' STEP %d/%d   %s' % (i, len(route), HUMAN[kind] % v
                                    if '%' in HUMAN[kind] else HUMAN[kind]))
        print('-' * 60)
        try:
            input('   ...then press Enter (Ctrl-C to stop): ')
        except (KeyboardInterrupt, EOFError):
            print('\ninterrupted - writing what we have')
            break

        Tm, To, scan = n.sample(a.settle)
        mxyz, myaw = Tm[:3, 3], T_yaw(Tm)
        oxyz, oyaw = To[:3, 3], T_yaw(To)
        tilt = T_tilt(Tm)
        inl = rmse = float('nan')
        if nn is not None and scan is not None and len(scan) > 300:
            S = G.voxel_downsample(scan, 0.15)
            # Full 6-DOF both ways: rotate the scan into the body frame with
            # the complete odom rotation, then score it with the complete
            # map->body transform. Using yaw only would discard the mount's
            # tilt and make a healthy fit look broken.
            Sb = (S - To[:3, 3]) @ To[:3, :3]           # odom -> body
            inl, rmse = G.score(nn, Sb, Tm, 0.25)

        ex, ey, eyaw = expected(kind, v)
        dm = do = float('nan')
        eyaw_err = float('nan')
        got_yaw_deg = float('nan')
        if prev is not None:
            (pm, pmy), (po, poy) = prev
            dm = float(np.linalg.norm(mxyz[:2] - pm[:2]))
            do = float(np.linalg.norm(oxyz[:2] - po[:2]))
            got_yaw = wrap(myaw - pmy)
            got_yaw_deg = math.degrees(got_yaw)
            eyaw_err = math.degrees(wrap(got_yaw - eyaw))
        cmd = math.hypot(ex, ey)

        if a.no_map:
            print('   odom (%+.2f, %+.2f) yaw %+6.1f   moved %.2f m (asked %.2f)'
                  % (oxyz[0], oxyz[1], math.degrees(oyaw), do, cmd))
            print('   tilt %.1f deg from the startup orientation' % tilt)
        else:
            print('   map  (%+.2f, %+.2f) yaw %+6.1f   moved %.2f m (asked %.2f)'
                  % (mxyz[0], mxyz[1], math.degrees(myaw), dm, cmd))
            print('   odom (%+.2f, %+.2f) yaw %+6.1f   moved %.2f m'
                  % (oxyz[0], oxyz[1], math.degrees(oyaw), do))
            q = ('GOOD' if inl > 0.58 else 'WEAK' if inl > 0.45 else 'BAD')
            print('   fit  inliers %.0f%%  rmse %.3f m   tilt %.1f deg  -> %s'
                  % (100 * inl, rmse, tilt, q))
        if eyaw != 0 and not math.isnan(got_yaw_deg):
            asked_deg = math.degrees(eyaw)
            if abs(abs(asked_deg) - 180.0) < 1.0:
                # A 180 deg turn ends on the same heading either way round, so
                # direction genuinely cannot be recovered from the end pose.
                print('   turned %.1f deg, asked 180 deg  (error %+.1f)'
                      % (abs(got_yaw_deg), eyaw_err))
                print('   note: 180 deg ends on the same heading whether you go '
                      'left or right - use 90 deg turns to check direction.')
            else:
                print('   turned %s %.1f deg, asked %s %.0f deg  (error %+.1f)'
                      % (dir_word(got_yaw_deg), abs(got_yaw_deg),
                         dir_word(asked_deg), abs(asked_deg), eyaw_err))
                if dir_word(got_yaw_deg) != dir_word(asked_deg):
                    print('   >> WRONG DIRECTION: it went %s, you were asked %s.'
                          % (dir_word(got_yaw_deg), dir_word(asked_deg)))

        rows.append(dict(step=i, kind=kind, cmd=v,
                         mx=mxyz[0], my=mxyz[1], myaw=math.degrees(myaw),
                         ox=oxyz[0], oy=oxyz[1], oyaw=math.degrees(oyaw),
                         moved_map=dm, moved_odom=do, cmd_dist=cmd,
                         yaw_err=eyaw_err, got_yaw=got_yaw_deg,
                         tilt=tilt, inliers=inl, rmse=rmse))
        if first is None:
            first = ((mxyz.copy(), myaw), (oxyz.copy(), oyaw))
        prev = ((mxyz.copy(), myaw), (oxyz.copy(), oyaw))

    # ---- summary ----
    if rows:
        out = os.path.expanduser(a.out)
        keys = list(rows[0].keys())
        with open(out, 'w') as f:
            f.write(','.join(keys) + '\n')
            for r in rows:
                f.write(','.join('%.4f' % r[k] if isinstance(r[k], float)
                                 else str(r[k]) for k in keys) + '\n')
        print('\nwrote %s' % out)

        inls = [] if a.no_map else [r['inliers'] for r in rows
                                    if not math.isnan(r['inliers'])]
        print('\n===== SUMMARY =====')
        if a.no_map:
            print('(no-map mode: odometry only - no localization numbers)')
        if inls:
            print('localization fit : min %.0f%%  median %.0f%%  max %.0f%%'
                  % (100 * min(inls), 100 * float(np.median(inls)), 100 * max(inls)))
            bad = [r['step'] for r in rows if r['inliers'] < 0.45]
            weak = [r['step'] for r in rows if 0.45 <= r['inliers'] < 0.58]
            if bad:
                print('  BAD  at steps %s - localization is not usable there'
                      % ', '.join(map(str, bad)))
            if weak:
                print('  WEAK at steps %s' % ', '.join(map(str, weak)))
            if not bad and not weak:
                print('  every waypoint was GOOD')

        moves = [r for r in rows if r['cmd_dist'] > 0.05
                 and not math.isnan(r['moved_map'])]
        if moves:
            sc = [r['moved_map'] / r['cmd_dist'] for r in moves]
            print('distance         : measured/commanded median %.3f' % float(np.median(sc)))
            print('  (1.00 = the stack agrees with your tape measure)')
        turns = [r for r in rows if r['kind'].startswith('turn')
                 and not math.isnan(r['got_yaw'])]
        if turns:
            wrong = [r for r in turns
                     if abs(abs(r['cmd']) - 180.0) >= 1.0
                     and dir_word(r['got_yaw']) != ('LEFT' if r['kind'] ==
                                                    'turn_left' else 'RIGHT')]
            print('turn direction   : %d of %d turns went the intended way'
                  % (len(turns) - len(wrong), len(turns)))
            if wrong:
                print('  wrong direction at steps %s'
                      % ', '.join(str(r['step']) for r in wrong))

        yerr = [r['yaw_err'] for r in rows
                if not math.isnan(r['yaw_err']) and r['kind'].startswith('turn')]
        if yerr:
            print('turn error       : median %+.1f deg, worst %+.1f deg'
                  % (float(np.median(yerr)), max(yerr, key=abs)))

        # The single most diagnostic number here: how far the MAP pose moves
        # while the robot is provably still (odometry confirms it). Any motion
        # is the relocalizer alone, so this isolates its stability from
        # everything else.
        jit = [] if a.no_map else [
            r['moved_map'] for r in rows
            if r['kind'] == 'pause' and not math.isnan(r['moved_map'])
            and r['moved_odom'] < 0.03]
        if jit:
            print('relocalizer jitter at rest: median %.2f m, worst %.2f m'
                  % (float(np.median(jit)), max(jit)))
            print('  (robot stationary per odometry, so this is the '
                  'relocalizer moving on its own)')
            if max(jit) > 0.20:
                print('  >> UNSTABLE. Nav2 cannot hold a goal tolerance '
                      'smaller than this.')

        tl = [r['tilt'] for r in rows if not math.isnan(r['tilt'])]
        if tl:
            print('mount tilt       : median %.1f deg (range %.1f-%.1f)'
                  % (float(np.median(tl)), min(tl), max(tl)))

        # Dead-reckon the commanded route to find out whether it closes, so
        # the end-vs-start figure is only called "drift" when that is true.
        rx = ry = rth = 0.0
        for kind, v in route[:len(rows)]:
            dx, dy, dth = expected(kind, v)
            rx += dx * math.cos(rth) - dy * math.sin(rth)
            ry += dx * math.sin(rth) + dy * math.cos(rth)
            rth = wrap(rth + dth)
        closes = math.hypot(rx, ry) < 0.05 and abs(rth) < math.radians(5)

        if first is not None and prev is not None and len(rows) > 2:
            (fm, fmy), _ = first
            (lm, lmy), _ = prev
            gap = float(np.linalg.norm(lm[:2] - fm[:2]))
            dyaw = math.degrees(wrap(lmy - fmy))
            print('start-to-end gap : %.2f m, %+.1f deg' % (gap, dyaw))
            if closes:
                print('  route returns to its start, so this IS accumulated drift')
            else:
                print('  route does NOT close (it should end %.2f m, %+.1f deg '
                      'from the start), so this is NOT drift'
                      % (math.hypot(rx, ry), math.degrees(rth)))

        if jit:
            print('\nSuggested Nav2 xy_goal_tolerance >= %.2f m'
                  % max(0.25, 2.0 * max(jit)))
            print('  (a goal tolerance below the relocalizer jitter can never '
                  'be met - the pose moves further than that on its own)')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
