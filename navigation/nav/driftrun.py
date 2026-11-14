#!/usr/bin/env python3
"""Drive a FIXED open-loop velocity route and log the pose, for drift tests.

    # see the plan, publish nothing
    /usr/bin/python3 ~/.scuba/nav/driftrun.py --route outback --dry-run

    # the real thing
    /usr/bin/python3 ~/.scuba/nav/driftrun.py --route outback --note with-reloc-1

*** THIS SCRIPT MOVES THE ROBOT. ***

It publishes Twist on /cmd_vel and nothing else. It NEVER changes mode - you
set FSM501 by hand, as always - and it sends no Nav2 goal. Stand clear and keep
a hand on Ctrl-C, which halts immediately.

WHY DRIVE IT FROM A SCRIPT
Hand-driving makes the two runs different experiments: different speeds,
different turn angles, different distance. Replaying one fixed command sequence
makes the INPUT identical, so the only thing that differs between run A and
run B is whether the relocalizer was correcting. What the robot physically did
still varies - that is what the tape measures - but the commanded trajectory no
longer does.

WHY THE ROUTES LOOK THE WAY THEY DO
MEASURED on this robot: sustained turn rate is **10.21 deg/s** (0.40 rad/s
commanded, gain 0.45), so a 90 deg turn costs **8.8 s** and a 180 costs 17.6 s.
A four-corner square therefore spends ~35 s turning before it walks anywhere,
which is why `outback` - two legs and two half-turns - is the default: it
returns near the start for a closure measurement while spending most of its
time translating.

Forward speed defaults to 0.25 m/s: above the measured 0.20 m/s gait threshold
(below it the robot does not walk at all) and at the DWB ceiling.

OPEN-LOOP TURNS ARE IMPRECISE, ON PURPOSE
Turn-per-command varies 40-60% with gait phase, so the robot will NOT come back
exactly to the mark. That is fine and expected - the tape measures where it
actually ended, and both runs receive the identical command sequence.

OUTPUT
The same CSV schema driftlog.py writes, so afterwards:
    driftlog.py annotate <csv> --dx F --dy L [--dyaw D] [--closed]
    driftlog.py compare  <csvs>
"""
import argparse
import csv
import math
import os
import signal
import sys
import time


def yaw_of(r):
    """Ground heading from the rotation matrix's +X column - correct for any
    tilt, which matters because `body` is inverted by 171 deg here."""
    w, x, y, z = r.w, r.x, r.y, r.z
    return math.degrees(math.atan2(2.0 * (x * y + w * z),
                                   1.0 - 2.0 * (y * y + z * z)))


def median(v):
    s = sorted(v)
    n = len(s)
    if not n:
        return float('nan')
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


# deg/s actually delivered per second of SUSTAINED 0.40 rad/s command.
#
# MEASURED 2026-11-11 from drift_with-reloc-1_1111_160215.csv: two commanded
# 17.6 s turns produced +480.0 deg net, i.e. 13.64 deg/s including the ramp in
# and out. Mid-turn, once steady, it runs 15.5-16.0 deg/s.
#
# The old value here was 10.21, which came from 0.40 s BURSTS - and a burst
# pays a fixed gait-initiation cost that a long turn amortises away. Applying
# it to a 17.6 s command turned 283 deg instead of 180. Same class of mistake
# as the gait-snap regression: a number measured under one condition used
# under another.
#
# 13.64 is the right constant for SIZING A DURATION, because the command
# window includes the ramp. 16.04 would under-turn.
TURN_RATE = 13.64


def build_route(a):
    """[(label, vx, wz, seconds), ...]  with a pause between segments."""
    def turn_secs(deg):
        return abs(deg) / a.turn_rate
    segs = []

    def add(label, vx, wz, secs):
        segs.append((label, vx, wz, secs))
        if a.pause > 0:
            segs.append(('pause', 0.0, 0.0, a.pause))

    if a.route == 'calib':
        # one long turn, nothing else: time it, read the heading change,
        # divide. Re-run this whenever the gait or accel limits change.
        add('calibration turn', 0.0, a.wz, a.calib_secs)
    elif a.route == 'straight':
        add('forward %.0fs' % a.leg, a.vx, 0.0, a.leg)
    elif a.route == 'outback':
        add('leg 1 forward', a.vx, 0.0, a.leg)
        add('turn 180', 0.0, a.wz, turn_secs(180))
        add('leg 2 forward', a.vx, 0.0, a.leg)
        add('turn 180 back', 0.0, a.wz, turn_secs(180))
    elif a.route == 'square':
        for i in range(4):
            add('side %d forward' % (i + 1), a.vx, 0.0, a.leg)
            add('corner %d turn 90' % (i + 1), 0.0, a.wz, turn_secs(90))
    else:
        raise SystemExit('unknown route %r' % a.route)
    if segs and segs[-1][0] == 'pause':
        segs.pop()
    return segs


def main():
    ap = argparse.ArgumentParser(
        description='Drive a fixed open-loop route and log the pose. MOVES '
                    'THE ROBOT. Never changes mode; publishes only Twist.')
    ap.add_argument('--turn-rate', type=float, default=TURN_RATE,
                    help='deg/s delivered per second of sustained turn '
                         'command, used to size turn durations. Default %.2f, '
                         'MEASURED 2026-11-11 over two 17.6 s turns. Re-measure '
                         'with --route calib if the gait changes.' % TURN_RATE)
    ap.add_argument('--calib-secs', type=float, default=20.0,
                    help='s of continuous turn for --route calib')
    ap.add_argument('--route', choices=('outback', 'square', 'straight', 'calib'),
                    default='outback',
                    help='outback = leg, 180, leg, 180 (ends near the start, '
                         'only two turns). square = four corners, expensive in '
                         'time because a 90 deg turn costs 8.8 s. straight = '
                         'pure translation, open route.')
    ap.add_argument('--leg', type=float, default=8.0,
                    help='s of forward walking per leg (default 8 = ~2.0 m at '
                         '0.25 m/s)')
    ap.add_argument('--vx', type=float, default=0.25,
                    help='m/s forward. MEASURED: below 0.20 the gait does not '
                         'walk at all; 0.25 is the DWB ceiling')
    ap.add_argument('--wz', type=float, default=0.40,
                    help='rad/s turn. MEASURED: 0.40 is delivered honestly and '
                         'is the nav_bridge clamp')
    ap.add_argument('--pause', type=float, default=1.5,
                    help='s of stillness between segments, so the gait settles '
                         'and turns repeat better')
    ap.add_argument('--settle', type=float, default=8.0,
                    help='s of stillness used to fix the start and end pose')
    ap.add_argument('--still-tol', type=float, default=0.05)
    ap.add_argument('--note', default='run')
    ap.add_argument('--out-dir', default=os.path.expanduser('~/.scuba/nav/logs'))
    ap.add_argument('--path-min-step', type=float, default=0.05)
    ap.add_argument('--period', type=float, default=0.1)
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the plan and exit. Publishes nothing.')
    ap.add_argument('--yes', action='store_true',
                    help='skip the are-you-ready prompt')
    a = ap.parse_args()

    if a.vx > 0.30 or abs(a.wz) > 0.40:
        raise SystemExit('!! refusing: vx %.2f / wz %.2f exceeds the measured '
                         'nav_bridge clamps (0.30 / 0.40)' % (a.vx, a.wz))

    segs = build_route(a)
    total = sum(s[3] for s in segs)
    print('\nROUTE: %s' % a.route)
    for label, vx, wz, secs in segs:
        print('   %-18s vx %+.2f  wz %+.2f  for %5.1f s' % (label, vx, wz, secs))
    print('   %-18s %28.1f s of motion' % ('TOTAL', total))
    print('   plus %.0f s of stillness at each end' % a.settle)
    fwd_time = sum(s[3] for s in segs if s[1] != 0)
    print('   forward distance if the gait delivers %.2f m/s: ~%.2f m'
          % (a.vx, a.vx * fwd_time))
    turn_deg = sum(abs(s[2]) / 0.40 * a.turn_rate * s[3] for s in segs if s[2] != 0)
    print('   total commanded turn: ~%.0f deg (at %.2f deg/s)'
          % (turn_deg, a.turn_rate))
    print('   NOTE: turn-per-command varies 40-60%% with gait phase, so the')
    print('         robot will NOT land exactly on the mark. Measure where it')
    print('         actually stops - that is the point.')

    if a.dry_run:
        print('\n--dry-run: nothing published, nothing moved.')
        return 0

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32
    import tf2_ros

    rclpy.init()
    n = Node('driftrun')
    pub = n.create_publisher(Twist, a.cmd_topic, 10)
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

    def halt(times=40):
        """Zero velocity, repeatedly. Called from every exit path."""
        z = Twist()
        for _ in range(times):
            try:
                pub.publish(z)
                rclpy.spin_once(n, timeout_sec=0.002)
            except Exception:
                pass
            time.sleep(0.02)

    stopping = {'flag': False}

    def on_sig(signum, frame):
        stopping['flag'] = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    print('\nwaiting for TF ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < 30.0:
        rclpy.spin_once(n, timeout_sec=0.05)
        if sample() is not None:
            break
    else:
        print('!! no TF map->%s. Is the stack up? Not driving.' % a.base_frame)
        n.destroy_node(); rclpy.shutdown()
        return 1

    if not a.yes:
        print('\n' + '=' * 62)
        print('  THE ROBOT IS ABOUT TO WALK %.0f s UNDER OPEN-LOOP CONTROL.' % total)
        print('  It will NOT avoid obstacles. Clear the route.')
        print('  FSM501 must already be set - this script never changes mode.')
        print('=' * 62)
        try:
            input('  press Enter to start, Ctrl-C to abort: ')
        except (KeyboardInterrupt, EOFError):
            print('\naborted, nothing sent.')
            n.destroy_node(); rclpy.shutdown()
            return 1

    os.makedirs(a.out_dir, exist_ok=True)
    path = os.path.join(a.out_dir, 'drift_%s_%s.csv'
                        % (a.note, time.strftime('%m%d_%H%M%S')))
    fh = open(path, 'w', newline='')
    wr = csv.writer(fh)
    wr.writerow(['t', 'phase',
                 'map_body_x', 'map_body_y', 'map_body_yaw',
                 'map_ci_x', 'map_ci_y', 'map_ci_yaw',
                 'ci_body_x', 'ci_body_y', 'ci_body_yaw',
                 'inliers', 'fitness'])
    print('logging to %s' % path, flush=True)

    run_t0 = time.time()
    phase = ['settle-start']
    rows = []
    start = None
    end = None
    unwrap = {'total': 0.0, 'prev': None}      # cumulative heading, not wrapped

    def push(s):
        t = time.time() - run_t0
        h = s['map_body'][2]
        if unwrap['prev'] is not None and phase[0] == 'run':
            unwrap['total'] += (h - unwrap['prev'] + 180.0) % 360.0 - 180.0
        unwrap['prev'] = h
        rows.append((t, phase[0], s))
        wr.writerow([round(t, 3), phase[0]]
                    + ['%.5f' % v for v in s['map_body']]
                    + ['%.5f' % v for v in s['map_ci']]
                    + ['%.5f' % v for v in s['ci_body']]
                    + ['%.4f' % live['inliers'], '%.4f' % live['fitness']])
        fh.flush()

    def still_window(secs, label):
        got = []
        t_begin = time.time()
        warned = 0.0
        while not stopping['flag']:
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
                    print('   %s: still moving (%.3f m > %.3f) - holding'
                          % (label, sp, a.still_tol), flush=True)
            if time.time() - t_begin > 90.0:
                print('   %s: never settled in 90 s - continuing anyway'
                      % label, flush=True)
                return got, -1.0
            time.sleep(a.period)
        return got, -1.0

    try:
        print('\nhold STILL on the start mark ...', flush=True)
        sw, sp0 = still_window(a.settle, 'start')
        if stopping['flag'] or not sw:
            raise KeyboardInterrupt
        start = {k: tuple(median([g[k][i] for g in sw]) for i in range(3))
                 for k in KEYS}
        print('start pose locked  (%+.3f, %+.3f) yaw %+.1f   [spread %.3f m]'
              % (start['map_body'][0], start['map_body'][1],
                 start['map_body'][2], sp0), flush=True)

        phase[0] = 'run'
        print('\n>>> DRIVING\n', flush=True)
        for label, vx, wz, secs in segs:
            if stopping['flag']:
                break
            print('   %-18s vx %+.2f wz %+.2f  %.1f s' % (label, vx, wz, secs),
                  flush=True)
            tseg = time.time()
            tw = Twist()
            tw.linear.x = float(vx)
            tw.angular.z = float(wz)
            while time.time() - tseg < secs and not stopping['flag']:
                pub.publish(tw)
                rclpy.spin_once(n, timeout_sec=0.01)
                s = sample()
                if s is not None:
                    push(s)
                time.sleep(0.05)
            halt(6)          # a short zero between segments

        halt()
        aborted = stopping['flag']
        if aborted:
            print('\n!! aborted mid-route - halted. No end window; the pose at')
            print('   the moment of the stop is the last logged row.', flush=True)
            end = {k: tuple(rows[-1][2][k]) for k in KEYS} if rows else start
        else:
            phase[0] = 'settle-end'
            print('\nstopped. hold STILL - locking the end pose ...', flush=True)
            ew, sp1 = still_window(a.settle, 'end')
            end = {k: tuple(median([g[k][i] for g in ew]) for i in range(3))
                   for k in KEYS} if ew else start
    except KeyboardInterrupt:
        print('\n!! interrupted - halting', flush=True)
        halt()
        if end is None:
            end = ({k: tuple(rows[-1][2][k]) for k in KEYS}
                   if rows else start)
    finally:
        halt()
        fh.close()

    # ---- what happened
    if start is None or end is None:
        print('\nno usable start/end pose - nothing to report. CSV: %s' % path)
        n.destroy_node(); rclpy.shutdown()
        return 1
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

    print('\n' + '=' * 62)
    print('route %s, note %s' % (a.route, a.note))
    print('estimated distance walked : %.2f m' % filt)
    print('ESTIMATOR says it ended   : fwd %+.3f m, left %+.3f m, yaw %+.1f deg'
          % (est_fwd, est_left, dyaw))
    cmd_turn = sum(abs(s[2]) / 0.40 * a.turn_rate * s[3] for s in segs if s[2] != 0)
    print('total heading SWEPT       : %+.1f deg (unwrapped)   commanded ~%.0f'
          % (unwrap['total'], cmd_turn))
    if cmd_turn > 30:
        tsec = sum(s[3] for s in segs if s[2] != 0)
        print('   => delivered %.2f deg/s against the assumed %.2f'
              % (abs(unwrap['total']) / tsec if tsec else 0.0, a.turn_rate))
        print('   => if that ratio is not ~1.00, pass --turn-rate %.2f next time'
              % (abs(unwrap['total']) / tsec if tsec else a.turn_rate))
    print('')
    print('NOW MEASURE where it actually is, relative to the start mark:')
    print('  forward along the START heading, and to its LEFT. Then:')
    print('')
    print('  /usr/bin/python3 ~/.scuba/nav/driftlog.py annotate \\')
    print('      %s \\' % path)
    print('      --dx <forward m> --dy <left m> [--dyaw <deg>]')
    print('=' * 62)
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
