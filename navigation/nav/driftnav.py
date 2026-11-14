#!/usr/bin/env python3
"""Drive N out-and-back laps with send_goal and log the drift.

    # see the plan, drive nothing
    /usr/bin/python3 ~/.scuba/nav/driftnav.py --laps 5 --distance 5.0 --dry-run

    # run A, relocalizer on
    /usr/bin/python3 ~/.scuba/nav/driftnav.py --laps 5 --distance 5.0 \\
        --note navlaps-with-reloc

    # run B: restart localization with seed_only:=true first, then the SAME line
    /usr/bin/python3 ~/.scuba/nav/driftnav.py --laps 5 --distance 5.0 \\
        --note navlaps-seed-only

*** THIS MOVES THE ROBOT, through Nav2. *** It never changes mode - you set
FSM501 by hand. Ctrl-C stops the lap, cancels the FollowPath goal and halts.

WHY NAVIGATE INSTEAD OF REPLAYING VELOCITIES
This measures the thing that actually breaks the mission. With the relocalizer
running, a map coordinate stays pinned to the room, so the robot physically
arrives where the map said. With it off, the map frame drifts along with
odometry: a fixed map coordinate slides relative to the room and the robot
walks confidently to the WRONG PHYSICAL PLACE while believing it arrived. The
tape measures exactly that.

It also sidesteps the turn-rate calibration entirely, because Nav2 steers.

WHY SEVERAL LAPS
`goal_checker.xy_goal_tolerance` is 0.30 m, so any single arrival may be
0.30 m off for reasons that have nothing to do with drift. But that error does
NOT accumulate - each arrival is independent - whereas drift grows with
distance. Five 5 m laps is 50 m travelled: at even 2% drift that is 1 m of
signal against an unchanged +/-0.30 m of noise. One lap would be swamped.

GOALS ARE RELATIVE TO WHERE YOU START
The out goal is `--distance` metres along the START heading, and the back goal
is the start pose itself. Nothing assumes the robot begins at the map origin.

OUTPUT
driftlog.py's CSV schema plus a `lap` column, so afterwards:
    driftlog.py annotate <csv> --dx F --dy L [--dyaw D] --closed
    driftlog.py compare  <csvs>
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time


def yaw_of(r):
    """Ground heading from the rotation matrix +X column; correct under the
    171 deg `body` inversion, unlike a blind atan2 shortcut."""
    return math.degrees(math.atan2(2.0 * (r.x * r.y + r.w * r.z),
                                   1.0 - 2.0 * (r.y * r.y + r.z * r.z)))


def median(v):
    s = sorted(v)
    n = len(s)
    if not n:
        return float('nan')
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def main():
    ap = argparse.ArgumentParser(
        description='Out-and-back laps under Nav2, logged for a drift test. '
                    'MOVES THE ROBOT. Never changes mode.')
    ap.add_argument('--laps', type=int, default=5)
    ap.add_argument('--distance', type=float, default=5.0,
                    help='m out along the START heading. Pick the longest '
                         'clear straight you have: drift must clear the '
                         '0.30 m goal tolerance to be measurable')
    ap.add_argument('--note', default='navlaps')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/.scuba/nav/logs'))
    ap.add_argument('--settle', type=float, default=8.0)
    ap.add_argument('--still-tol', type=float, default=0.05)
    ap.add_argument('--period', type=float, default=0.1)
    ap.add_argument('--path-min-step', type=float, default=0.05)
    ap.add_argument('--send-goal', default=os.path.expanduser('~/.scuba/nav/send_goal.py'))
    ap.add_argument('--goal-timeout', type=float, default=180.0,
                    help='s before a single goal is abandoned')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--yes', action='store_true')
    a = ap.parse_args()

    print('\nPLAN: %d laps x (out %.2f m, back) = %.1f m of travel'
          % (a.laps, a.distance, 2.0 * a.laps * a.distance))
    print('   goal tolerance is 0.30 m and does NOT accumulate;')
    print('   drift does. That is why the laps are repeated.')
    if a.dry_run:
        print('\n--dry-run: nothing started, nothing moved.')
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32
    import tf2_ros

    rclpy.init()
    n = Node('driftnav')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    live = {'inliers': float('nan'), 'fitness': float('nan')}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(Float32, '/relocalizer/inliers',
                          lambda m: live.__setitem__('inliers', m.data), qos)
    n.create_subscription(Float32, '/relocalizer/fitness',
                          lambda m: live.__setitem__('fitness', m.data), qos)

    PAIRS = [('map', a.base_frame), ('map', a.odom_frame),
             (a.odom_frame, a.base_frame)]
    KEYS = ['map_body', 'map_ci', 'ci_body']

    def sample():
        out = {}
        for key, (p, c) in zip(KEYS, PAIRS):
            try:
                tr = buf.lookup_transform(p, c, rclpy.time.Time())
            except Exception:
                return None
            t = tr.transform.translation
            out[key] = (t.x, t.y, yaw_of(tr.transform.rotation))
        return out

    def cancel_follow_path():
        """Cancel EVERY FollowPath goal.

        Needed because killing send_goal does not stop the robot:
        controller_server keeps driving an orphaned goal. An empty goal id with
        a zero stamp is the ROS 2 'cancel all' convention.
        """
        try:
            from action_msgs.srv import CancelGoal
            cli = n.create_client(CancelGoal, '/follow_path/_action/cancel_goal')
            if not cli.wait_for_service(timeout_sec=3.0):
                return False
            fut = cli.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(n, fut, timeout_sec=3.0)
            return fut.done()
        except Exception:
            return False

    print('\nwaiting for TF ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 30.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        if sample() is not None:
            break
    else:
        print('!! no TF map->%s. Is start_sensing.sh up?' % a.base_frame)
        n.destroy_node(); rclpy.shutdown(); return 1

    if not os.path.exists(a.send_goal):
        print('!! missing %s' % a.send_goal)
        n.destroy_node(); rclpy.shutdown(); return 1

    if not a.yes:
        print('\n' + '=' * 64)
        print('  THE ROBOT WILL NAVIGATE %.1f m OVER %d LAPS.'
              % (2.0 * a.laps * a.distance, a.laps))
        print('  Nav2 avoids what is in the costmap; it does not see')
        print('  everything. Clear the route. FSM501 must already be set.')
        print('=' * 64)
        try:
            input('  press Enter to start, Ctrl-C to abort: ')
        except (KeyboardInterrupt, EOFError):
            print('\naborted.'); n.destroy_node(); rclpy.shutdown(); return 1

    os.makedirs(a.out_dir, exist_ok=True)
    path = os.path.join(a.out_dir, 'drift_%s_%s.csv'
                        % (a.note, time.strftime('%m%d_%H%M%S')))
    fh = open(path, 'w', newline='')
    wr = csv.writer(fh)
    wr.writerow(['t', 'phase',
                 'map_body_x', 'map_body_y', 'map_body_yaw',
                 'map_ci_x', 'map_ci_y', 'map_ci_yaw',
                 'ci_body_x', 'ci_body_y', 'ci_body_yaw',
                 'inliers', 'fitness', 'lap'])
    print('logging to %s' % path, flush=True)

    run_t0 = time.time()
    phase = ['settle-start']
    lap_lbl = ['-']
    rows = []
    stop = {'f': False}

    def on_sig(s, f):
        stop['f'] = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    def push(s):
        t = time.time() - run_t0
        rows.append((t, phase[0], s))
        wr.writerow([round(t, 3), phase[0]]
                    + ['%.5f' % v for v in s['map_body']]
                    + ['%.5f' % v for v in s['map_ci']]
                    + ['%.5f' % v for v in s['ci_body']]
                    + ['%.4f' % live['inliers'], '%.4f' % live['fitness'],
                       lap_lbl[0]])
        fh.flush()

    def still_window(secs, label):
        got = []
        t_begin = time.time()
        warned = 0.0
        while not stop['f']:
            rclpy.spin_once(n, timeout_sec=0.02)
            s = sample()
            if s is not None:
                push(s); got.append(s)
            got = got[-(int(secs / a.period) + 5):]
            if len(got) >= max(5, int(secs / a.period)):
                xs = [g['map_body'][0] for g in got]
                ys = [g['map_body'][1] for g in got]
                sp = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                if sp <= a.still_tol:
                    return got, sp
                if time.time() - warned > 3.0:
                    warned = time.time()
                    print('   %s: moving (%.3f m > %.3f) - hold still'
                          % (label, sp, a.still_tol), flush=True)
            if time.time() - t_begin > 90.0:
                print('   %s: never settled in 90 s - continuing' % label,
                      flush=True)
                return got, -1.0
            time.sleep(a.period)
        return got, -1.0

    def run_goal(gx, gy, gyaw, label):
        """send_goal as a SUBPROCESS - it owns its own node and publishes
        /cmd_vel, so in-process would mean two publishers in one executor.
        We keep logging while it runs by polling it."""
        cmd = ['/usr/bin/python3', a.send_goal,
               '%.4f' % gx, '%.4f' % gy, '%.1f' % gyaw, '--finish-yaw']
        print('   %-14s -> (%+.2f, %+.2f) yaw %+.0f' % (label, gx, gy, gyaw),
              flush=True)
        pr = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                              stderr=subprocess.STDOUT)
        t_start = time.time()
        while pr.poll() is None:
            rclpy.spin_once(n, timeout_sec=0.02)
            s = sample()
            if s is not None:
                push(s)
            if stop['f']:
                print('   !! aborting: SIGINT to send_goal, cancelling the path',
                      flush=True)
                try:
                    pr.send_signal(signal.SIGINT)
                except Exception:
                    pass
                time.sleep(2.0)
                if pr.poll() is None:
                    pr.kill()
                cancel_follow_path()
                return False
            if time.time() - t_start > a.goal_timeout:
                print('   !! goal exceeded %.0f s - giving up on this lap'
                      % a.goal_timeout, flush=True)
                try:
                    pr.send_signal(signal.SIGINT)
                except Exception:
                    pass
                time.sleep(2.0)
                if pr.poll() is None:
                    pr.kill()
                cancel_follow_path()
                return False
            time.sleep(0.05)
        ok = (pr.returncode == 0)
        print('      %s (exit %d)' % ('REACHED' if ok else 'NOT reached',
                                      pr.returncode), flush=True)
        return ok

    start = end = None
    laps_done = 0
    try:
        print('\nhold STILL on the start mark ...', flush=True)
        sw, sp0 = still_window(a.settle, 'start')
        if stop['f'] or not sw:
            raise KeyboardInterrupt
        start = {k: tuple(median([g[k][i] for g in sw]) for i in range(3))
                 for k in KEYS}
        sx, sy, syaw = start['map_body']
        print('start pose locked (%+.3f, %+.3f) yaw %+.1f  [spread %.3f m]'
              % (sx, sy, syaw, sp0), flush=True)

        h = math.radians(syaw)
        ox, oy = sx + a.distance * math.cos(h), sy + a.distance * math.sin(h)
        print('out goal  (%+.2f, %+.2f)   back goal (%+.2f, %+.2f)\n'
              % (ox, oy, sx, sy), flush=True)

        phase[0] = 'run'
        for i in range(1, a.laps + 1):
            if stop['f']:
                break
            print('=== lap %d/%d ===' % (i, a.laps), flush=True)
            lap_lbl[0] = '%d-out' % i
            if not run_goal(ox, oy, syaw, 'lap %d out' % i):
                print('   lap %d failed on the way out - stopping' % i)
                break
            lap_lbl[0] = '%d-back' % i
            if not run_goal(sx, sy, syaw, 'lap %d back' % i):
                print('   lap %d failed on the way back - stopping' % i)
                break
            laps_done = i
        lap_lbl[0] = '-'

        if not stop['f']:
            phase[0] = 'settle-end'
            print('\nhold STILL - locking the end pose ...', flush=True)
            ew, sp1 = still_window(a.settle, 'end')
            end = ({k: tuple(median([g[k][i] for g in ew]) for i in range(3))
                    for k in KEYS} if ew else start)
        else:
            end = {k: tuple(rows[-1][2][k]) for k in KEYS} if rows else start
    except KeyboardInterrupt:
        print('\n!! interrupted', flush=True)
        cancel_follow_path()
        if end is None:
            end = {k: tuple(rows[-1][2][k]) for k in KEYS} if rows else start
    finally:
        cancel_follow_path()
        fh.close()

    if start is None or end is None:
        print('\nno usable start/end pose. CSV: %s' % path)
        n.destroy_node(); rclpy.shutdown(); return 1

    dxw = end['map_body'][0] - start['map_body'][0]
    dyw = end['map_body'][1] - start['map_body'][1]
    h0 = math.radians(start['map_body'][2])
    est_fwd = dxw * math.cos(h0) + dyw * math.sin(h0)
    est_left = -dxw * math.sin(h0) + dyw * math.cos(h0)
    dyaw = (end['map_body'][2] - start['map_body'][2] + 180) % 360 - 180

    run = [r for r in rows if r[1] == 'run']
    filt = 0.0
    if run:
        anchor = run[0][2]['map_body']
        for _, _, s in run:
            d = math.hypot(s['map_body'][0] - anchor[0],
                           s['map_body'][1] - anchor[1])
            if d >= a.path_min_step:
                filt += d
                anchor = s['map_body']

    print('\n' + '=' * 64)
    print('laps completed            : %d of %d' % (laps_done, a.laps))
    print('estimated distance walked : %.2f m  (nominal %.1f m)'
          % (filt, 2.0 * laps_done * a.distance))
    print('ESTIMATOR says it ended   : fwd %+.3f m, left %+.3f m, yaw %+.1f deg'
          % (est_fwd, est_left, dyaw))
    print('   i.e. it believes it is %.3f m from the start mark'
          % math.hypot(est_fwd, est_left))
    print('')
    print('NOW MEASURE where it physically is, relative to the start mark:')
    print('  forward along the START heading, and to its LEFT. Then:')
    print('')
    print('  /usr/bin/python3 ~/.scuba/nav/driftlog.py annotate \\')
    print('      %s \\' % path)
    print('      --dx <forward m> --dy <left m> [--dyaw <deg>] --closed')
    print('=' * 64)
    if laps_done < a.laps:
        print('!! only %d of %d laps completed. The distance figure reflects '
              'what was\n   actually walked, so the %% drift stays valid - but '
              'say so when you\n   compare runs of different length.'
              % (laps_done, a.laps))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
