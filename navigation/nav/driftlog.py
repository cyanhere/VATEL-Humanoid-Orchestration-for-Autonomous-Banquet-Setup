#!/usr/bin/env python3
"""Measure pose drift over a WALKING run, with and without the relocalizer.

    # ---- run A: relocalizer + accumulator (the normal stack)
    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/driftlog.py record --note with-reloc

    # ---- run B: odometry only
    #   restart localization with ICP disabled, THEN record:
    #   ~/.scuba/livemap/start_localization.sh --ros-args -p seed_only:=true
    /usr/bin/python3 ~/.scuba/nav/driftlog.py record --note seed-only

    # ---- compare
    /usr/bin/python3 ~/.scuba/nav/driftlog.py compare \\
        ~/.scuba/nav/logs/drift_with-reloc_*.csv \\
        ~/.scuba/nav/logs/drift_seed-only_*.csv

READ-ONLY. It subscribes and reads TF. It publishes NOTHING, sends no velocity,
and changes no mode. You drive the robot; this only watches.

WHY THIS EXISTS
Every drift figure on record was measured with the robot STANDING STILL, which
is the one condition under which odometry cannot fail: drift accumulates with
distance travelled, and a stationary robot travels none. Those windows showed
odometry-only at 5.2 mm against 14.0 mm for the full chain, which invites the
conclusion that the relocalizer makes things worse. It does add ~9 mm of
jitter - but its job is bounding error over DISTANCE, and no run has ever
measured that.

THE PROTOCOL
  1. Stand still at a physically marked spot. Start this script.
  2. Wait for "start pose locked".
  3. Walk a CLOSED loop - 20-40 m - and return to the same marked spot.
  4. Stand still again until "end pose locked", then Ctrl-C.

The metric is return-to-start error divided by distance travelled. Odometry
should show a percentage of distance; a working relocalizer should be bounded
by the map's own accuracy no matter how far you walked.

WHY BOTH RUNS SHARE A FRAME
Run B uses `seed_only:=true`, which holds the gravity-levelled seed and never
runs ICP. `map->camera_init` then stays fixed, so `map->body` becomes pure
FAST-LIO - but it is still the SAME map frame as run A. Comparing
`camera_init->body` against `map->body` instead would compare a chain against
one of its own links, which is the flaw in the stationary numbers.

PATH LENGTH IS FILTERED, DELIBERATELY
Summing every consecutive sample inflates distance with jitter: 80 samples of
0.003 m sums to 0.26 m and looks like travel. Distance only accumulates once
the robot has moved `--path-min-step` (default 0.05 m, just under the measured
p90 gait step of 0.143 m) from the last counted point. Both raw and filtered
totals are reported so the difference is visible.
"""
import argparse
import csv
import glob
import math
import os
import sys
import time


# ---------------------------------------------------------------- geometry
def yaw_of(r):
    """Ground heading from a quaternion, via the rotation matrix's +X column.

    NOT the naive atan2(2(wz+xy), 1-2(y^2+z^2)) shortcut applied blindly: this
    robot's `body` frame is inverted (tilt 171.3 deg). Measured 2026-11-02 the
    two agree to 0.14 deg here, because the inversion is a roll about the
    forward axis - but projecting the actual +X axis is correct for any tilt,
    so it is what this uses.
    """
    w, x, y, z = r.w, r.x, r.y, r.z
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    return math.degrees(math.atan2(fy, fx))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def median(v):
    s = sorted(v)
    n = len(s)
    if not n:
        return float('nan')
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


# ---------------------------------------------------------------- recording
def do_record(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32
    import tf2_ros

    PAIRS = [('map', a.base_frame), ('map', a.odom_frame),
             (a.odom_frame, a.base_frame)]
    KEYS = ['map_body', 'map_ci', 'ci_body']

    rclpy.init()
    n = Node('driftlog')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)

    live = {'inliers': float('nan'), 'fitness': float('nan')}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(Float32, '/relocalizer/inliers',
                          lambda m: live.__setitem__('inliers', m.data), qos)
    n.create_subscription(Float32, '/relocalizer/fitness',
                          lambda m: live.__setitem__('fitness', m.data), qos)

    def sample():
        """All three transforms at once, or None if any is missing."""
        out = {}
        for key, (p, c) in zip(KEYS, PAIRS):
            try:
                tr = buf.lookup_transform(p, c, rclpy.time.Time())
            except Exception:
                return None
            t = tr.transform.translation
            out[key] = (t.x, t.y, yaw_of(tr.transform.rotation))
        return out

    print('waiting for TF ...', flush=True)
    t0 = time.time()
    while time.time() - t0 < a.tf_timeout:
        rclpy.spin_once(n, timeout_sec=0.05)
        if sample() is not None:
            break
    else:
        print('!! no TF for map->%s / map->%s in %.0fs. Is the stack up?'
              % (a.base_frame, a.odom_frame, a.tf_timeout))
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

    rows = []
    phase = ['settle-start']

    def push(s, t):
        rows.append((t, phase[0], s, live['inliers'], live['fitness']))
        wr.writerow([round(t, 3), phase[0]]
                    + ['%.5f' % v for v in s['map_body']]
                    + ['%.5f' % v for v in s['map_ci']]
                    + ['%.5f' % v for v in s['ci_body']]
                    + ['%.4f' % live['inliers'], '%.4f' % live['fitness']])
        fh.flush()

    def still_window(secs):
        """Collect `secs` of samples and return them once the spread is small.

        A single sample is not a pose: measured per-sample jump p90 is 0.0113 m
        even on a motionless robot, so the start and end references are the
        MEDIAN of a window, not one reading.
        """
        got = []
        t_begin = time.time()
        last_warn = 0.0
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            s = sample()
            now = time.time()
            if s is not None:
                push(s, now - run_t0)
                got.append(s)
            got = got[-(int(secs / a.period) + 5):]
            if len(got) >= max(5, int(secs / a.period)):
                xs = [g['map_body'][0] for g in got]
                ys = [g['map_body'][1] for g in got]
                spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
                if spread <= a.still_tol:
                    return got, spread
                if now - last_warn > 3.0:
                    print('   still moving (%.3f m spread over %.0fs, need '
                          '<= %.3f) - hold position'
                          % (spread, secs, a.still_tol), flush=True)
                    last_warn = now
            if now - t_begin > a.still_timeout:
                xs = [g['map_body'][0] for g in got]
                ys = [g['map_body'][1] for g in got]
                sp = math.hypot(max(xs) - min(xs), max(ys) - min(ys)) if got else -1
                print('   !! never settled below %.3f m in %.0fs (spread '
                       '%.3f). Using it anyway - treat the result as soft.'
                      % (a.still_tol, a.still_timeout, sp), flush=True)
                return got, sp
            time.sleep(a.period)

    run_t0 = time.time()
    print('\nhold STILL at the marked start point ...', flush=True)
    start_win, sp0 = still_window(a.settle)
    start = {k: tuple(median([g[k][i] for g in start_win]) for i in range(3))
             for k in KEYS}
    print('start pose locked  map->%s (%+.3f, %+.3f) yaw %+.1f  [spread %.3f m]'
          % (a.base_frame, start['map_body'][0], start['map_body'][1],
             start['map_body'][2], sp0), flush=True)
    print('\n>>> WALK THE LOOP NOW. Return to the same spot, stand still,')
    print('>>> then press Ctrl-C.\n', flush=True)

    phase[0] = 'run'
    raw_path = 0.0
    filt_path = 0.0
    ci_path = 0.0                      # how far map->camera_init itself moved:
    ci_prev = start['map_ci']          # the odometry error the relocalizer cancelled
    anchor = start['map_body']
    prev = start['map_body']
    peak = 0.0
    interrupted = False
    last_print = 0.0
    try:
        while True:
            rclpy.spin_once(n, timeout_sec=0.02)
            s = sample()
            now = time.time()
            if s is not None:
                push(s, now - run_t0)
                d = math.hypot(s['map_body'][0] - prev[0],
                               s['map_body'][1] - prev[1])
                raw_path += d
                da = math.hypot(s['map_body'][0] - anchor[0],
                                s['map_body'][1] - anchor[1])
                if da >= a.path_min_step:
                    filt_path += da
                    anchor = s['map_body']
                prev = s['map_body']
                ci_path += math.hypot(s['map_ci'][0] - ci_prev[0],
                                      s['map_ci'][1] - ci_prev[1])
                ci_prev = s['map_ci']
                peak = max(peak, math.hypot(s['map_body'][0] - start['map_body'][0],
                                            s['map_body'][1] - start['map_body'][1]))
                if now - last_print > 2.0:
                    last_print = now
                    print('   t %5.0fs  travelled %6.2f m  now %5.2f m from '
                          'start  (peak %5.2f)  inliers %.2f'
                          % (now - run_t0, filt_path, da if da else 0.0,
                             peak, live['inliers']), flush=True)
            if a.secs and (now - run_t0) > a.secs:
                break
            time.sleep(a.period)
    except KeyboardInterrupt:
        interrupted = True
        print('\n(stopping)', flush=True)

    phase[0] = 'settle-end'
    print('\nhold STILL - locking the end pose ...', flush=True)
    try:
        end_win, sp1 = still_window(a.settle)
    except KeyboardInterrupt:
        print('!! interrupted before the end pose settled; using the last '
              'samples as-is - treat the result as soft')
        end_win = [r[2] for r in rows[-20:]] or [start]
        sp1 = -1.0
    end = {k: tuple(median([g[k][i] for g in end_win]) for i in range(3))
           for k in KEYS}
    fh.close()

    # -------- which run is this? ASK, do not infer ---------------------------
    # Inferring seed_only from "the origin barely moved" is wrong: a HEALTHY
    # relocalizer barely moves it either. Measured 2026-11-03, an 8 s
    # stationary run with ICP demonstrably active (inliers 1.000, rmse 0.017)
    # was mislabelled "ICP was off". The parameter is authoritative.
    ci_moved = math.hypot(end['map_ci'][0] - start['map_ci'][0],
                          end['map_ci'][1] - start['map_ci'][1])
    seedlike = ask_seed_only(n, rclpy)
    if seedlike is None:
        seedlike = ci_moved < 0.005
        print('   (could not read /relocalizer seed_only - falling back to the '
              'motion heuristic, which is unreliable on a short run)')

    res = summarise(start, end, filt_path, raw_path, peak, ci_moved, seedlike,
                    [r[3] for r in rows if r[3] == r[3]],
                    [r[4] for r in rows if r[4] == r[4]], a.note, path,
                    ci_path=ci_path, loop_tol=a.loop_tol,
                    truth=load_truth(path))
    print_summary(res)
    with open(path.replace('.csv', '_summary.txt'), 'w') as sfh:
        sfh.write(fmt_summary(res))
    print('\nsummary written beside the CSV.')
    n.destroy_node(); rclpy.shutdown()
    return 0


def ask_seed_only(node, rclpy):
    """True / False from the relocalizer's own parameter, or None if unknown."""
    try:
        from rcl_interfaces.srv import GetParameters
        cli = node.create_client(GetParameters, '/relocalizer/get_parameters')
        if not cli.wait_for_service(timeout_sec=4.0):
            return None
        req = GetParameters.Request()
        req.names = ['seed_only']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=4.0)
        if not fut.done() or fut.result() is None:
            return None
        vals = fut.result().values
        if not vals:
            return None
        v = vals[0]
        if getattr(v, 'type', 0) == 1:          # PARAMETER_BOOL
            return bool(v.bool_value)
        return None
    except Exception:
        return None


def truth_path(csv_path):
    return csv_path.replace('.csv', '.truth.json')


def load_truth(csv_path):
    """Ground truth you measured with a tape, or None.

    Keys: dx, dy (metres the robot ACTUALLY ended from its start, expressed in
    the START heading's frame: +dx forward, +dy left), optional dyaw (degrees),
    and `closed` when the robot was returned to the mark.
    """
    import json
    try:
        with open(truth_path(csv_path)) as fh:
            return json.load(fh)
    except Exception:
        return None


def summarise(start, end, filt, raw, peak, ci_moved, seedlike, inl, fit,
              note, path, ci_path=0.0, loop_tol=0.60, truth=None):
    # Displacement the ESTIMATOR claims, rotated into the START heading frame
    # so it is directly comparable with a tape measured forward/left.
    dxw = end['map_body'][0] - start['map_body'][0]
    dyw = end['map_body'][1] - start['map_body'][1]
    h0 = math.radians(start['map_body'][2])
    est_fwd = dxw * math.cos(h0) + dyw * math.sin(h0)
    est_left = -dxw * math.sin(h0) + dyw * math.cos(h0)

    e_xy = math.hypot(dxw, dyw)
    e_yaw = wrap180(end['map_body'][2] - start['map_body'][2])
    o_xy = math.hypot(end['ci_body'][0] - start['ci_body'][0],
                      end['ci_body'][1] - start['ci_body'][1])
    # THE METRIC, when ground truth exists: the estimator's endpoint error is
    # (what it claimed) - (what actually happened). With a closed loop the
    # truth is (0,0) and this reduces to the classic closure error.
    gt = None
    if truth is not None:
        tfwd = float(truth.get('dx', 0.0))
        tleft = float(truth.get('dy', 0.0))
        ex, ey = est_fwd - tfwd, est_left - tleft
        gt = dict(fwd=est_fwd, left=est_left, tfwd=tfwd, tleft=tleft,
                  err=math.hypot(ex, ey), ex=ex, ey=ey,
                  pct=(100.0 * math.hypot(ex, ey) / filt) if filt > 0.5 else float('nan'),
                  yaw_err=(wrap180(e_yaw - float(truth['dyaw']))
                           if truth.get('dyaw') is not None else None),
                  deg_per_m=((abs(wrap180(e_yaw - float(truth['dyaw']))) / filt)
                             if truth.get('dyaw') is not None and filt > 0.5
                             else None))
    return dict(note=note, path=path, travelled=filt, raw_path=raw, peak=peak,
                est_fwd=est_fwd, est_left=est_left, gt=gt,
                err_xy=e_xy, err_yaw=e_yaw, odom_err_xy=o_xy,
                pct=(100.0 * e_xy / filt) if filt > 0.5 else float('nan'),
                ci_moved=ci_moved, ci_path=ci_path, seedlike=seedlike,
                closed=(e_xy <= loop_tol),
                corr_pct=(100.0 * ci_path / filt) if filt > 0.5 else float('nan'),
                inl=median(inl) if inl else float('nan'),
                fit=median(fit) if fit else float('nan'))


def fmt_summary(r):
    L = []
    L.append('run                     : %s' % r['note'])
    L.append('distance travelled      : %.2f m   (raw sum %.2f m - the gap is '
             'jitter)' % (r['travelled'], r['raw_path']))
    L.append('furthest from start     : %.2f m' % r['peak'])
    L.append('')
    if r['seedlike']:
        L.append('DRIFT ABSORBED BY THE RELOCALIZER: n/a, ICP was off.')
        L.append('  map->body here is raw odometry for the whole run.')
    else:
        L.append('DRIFT ABSORBED BY THE RELOCALIZER  <-- needs no closed loop')
        L.append('  correction path length: %.3f m   (%.2f %% of distance walked)'
                 % (r['ci_path'],
                    r['corr_pct'] if r['corr_pct'] == r['corr_pct'] else float('nan')))
        L.append('  net origin shift      : %.3f m' % r['ci_moved'])
        L.append('  Every metre map->camera_init moves is odometry error the')
        L.append('  relocalizer had to cancel. Path >> net means it was')
        L.append('  jittering; path ~= net means it was tracking real drift.')
    L.append('')
    g = r.get('gt')
    if g:
        L.append('ENDPOINT ERROR vs the tape  <-- THE NUMBER')
        L.append('  estimator says it moved : fwd %+.3f m, left %+.3f m'
                 % (g['fwd'], g['left']))
        L.append('  you measured            : fwd %+.3f m, left %+.3f m'
                 % (g['tfwd'], g['tleft']))
        L.append('  ERROR                   : %.3f m  (fwd %+.3f, left %+.3f)'
                 % (g['err'], g['ex'], g['ey']))
        L.append('  as %% of distance walked : %s'
                 % ('%.2f %%' % g['pct'] if g['pct'] == g['pct'] else 'n/a'))
        if g['yaw_err'] is not None:
            L.append('  heading error           : %+.1f deg  (%.2f deg/m)'
                     % (g['yaw_err'], g['deg_per_m']))
        else:
            L.append('  heading error           : not measured')
        L.append('')
    else:
        L.append('NO GROUND TRUTH ATTACHED. Measure where the robot actually')
        L.append('ended, then:  driftlog.py annotate %s --dx F --dy L [--dyaw D]'
                 % os.path.basename(r['path']))
        L.append('')
    if r['closed']:
        L.append('RETURN-TO-START ERROR, map->body  (route closed)')
        L.append('  translation           : %.3f m' % r['err_xy'])
        L.append('  heading               : %+.1f deg' % r['err_yaw'])
        L.append('  as %% of distance      : %s'
                 % ('%.2f %%' % r['pct'] if r['pct'] == r['pct'] else 'n/a (walk further)'))
        L.append('  same figure in camera_init->body (raw odometry): %.3f m'
                 % r['odom_err_xy'])
    else:
        L.append('RETURN-TO-START: WITHHELD - the route is OPEN.')
        L.append('  Ended %.2f m from the start, beyond --loop-tol. That number'
                 % r['err_xy'])
        L.append('  only means drift if you physically returned to the mark, so')
        L.append('  reporting it here would be misleading. Use the correction')
        L.append('  figure above, or walk a closed loop.')
    L.append('  inliers median %.3f   rmse median %.3f m' % (r['inl'], r['fit']))
    L.append('')
    if r['seedlike']:
        L.append('seed_only = TRUE: ICP never ran, so map->body here IS pure')
        L.append('odometry in the same map frame. This is a RUN B.')
        L.append('  (map->camera_init moved %.3f m, which should be ~0)'
                 % r['ci_moved'])
    else:
        L.append('seed_only = FALSE: the relocalizer was registering.')
        L.append('This is a RUN A.  inliers median %.3f, rmse median %.3f m'
                 % (r['inl'], r['fit']))
    return '\n'.join(L)


def print_summary(r):
    print('\n' + '=' * 66)
    print(fmt_summary(r))
    print('=' * 66)


# ---------------------------------------------------------------- compare
def do_compare(a):
    files = []
    for pat in a.files:
        files.extend(sorted(glob.glob(os.path.expanduser(pat))))
    files = [f for f in files if f.endswith('.csv')]
    if len(files) < 1:
        print('no CSVs matched'); return 2

    runs = []
    for f in files:
        with open(f) as fh:
            rows = list(csv.DictReader(fh))
        # The writer flushes every row, so a run killed mid-write leaves a
        # TRUNCATED final line, which DictReader fills with None. Drop any row
        # missing a field rather than dying on float(None).
        need = ('map_body_x', 'map_body_y', 'map_body_yaw',
                'map_ci_x', 'map_ci_y', 'ci_body_x', 'ci_body_y')
        full = [r for r in rows
                if all(r.get(k) not in (None, '') for k in need)
                and r.get('phase')]
        if len(full) != len(rows):
            print('  %s: dropped %d incomplete row(s)'
                  % (os.path.basename(f), len(rows) - len(full)))
        rows = full
        if len(rows) < 20:
            print('  skipping %s (%d usable rows)' % (f, len(rows))); continue
        st = [r for r in rows if r['phase'] == 'settle-start']
        en = [r for r in rows if r['phase'] == 'settle-end'] or rows[-20:]
        run = [r for r in rows if r['phase'] == 'run']

        def med(sub, col):
            return median([float(r[col]) for r in sub])
        start = dict(map_body=(med(st, 'map_body_x'), med(st, 'map_body_y'),
                               med(st, 'map_body_yaw')),
                     map_ci=(med(st, 'map_ci_x'), med(st, 'map_ci_y'), 0.0),
                     ci_body=(med(st, 'ci_body_x'), med(st, 'ci_body_y'), 0.0))
        end = dict(map_body=(med(en, 'map_body_x'), med(en, 'map_body_y'),
                             med(en, 'map_body_yaw')),
                   map_ci=(med(en, 'map_ci_x'), med(en, 'map_ci_y'), 0.0),
                   ci_body=(med(en, 'ci_body_x'), med(en, 'ci_body_y'), 0.0))

        raw = filt = peak = ci_path = 0.0
        ci_prev = (float(run[0]['map_ci_x']), float(run[0]['map_ci_y'])) if run else (0.0, 0.0)
        anchor = prev = (float(run[0]['map_body_x']), float(run[0]['map_body_y'])) if run else start['map_body'][:2]
        for r in run:
            p = (float(r['map_body_x']), float(r['map_body_y']))
            raw += math.hypot(p[0] - prev[0], p[1] - prev[1])
            da = math.hypot(p[0] - anchor[0], p[1] - anchor[1])
            if da >= a.path_min_step:
                filt += da; anchor = p
            prev = p
            cp = (float(r['map_ci_x']), float(r['map_ci_y']))
            ci_path += math.hypot(cp[0] - ci_prev[0], cp[1] - ci_prev[1])
            ci_prev = cp
            peak = max(peak, math.hypot(p[0] - start['map_body'][0],
                                        p[1] - start['map_body'][1]))
        def nums(col):
            out = []
            for r in run:
                v = r.get(col)
                if v in (None, '', 'nan'):
                    continue
                try:
                    f_ = float(v)
                except (TypeError, ValueError):
                    continue
                if f_ == f_:
                    out.append(f_)
            return out
        inl, fit = nums('inliers'), nums('fitness')
        ci_moved = math.hypot(end['map_ci'][0] - start['map_ci'][0],
                              end['map_ci'][1] - start['map_ci'][1])
        note = os.path.basename(f).split('_')[1] if '_' in os.path.basename(f) else '?'
        runs.append(summarise(start, end, filt, raw, peak, ci_moved,
                              ci_moved < 0.005, inl, fit, note, f,
                              ci_path=ci_path, loop_tol=a.loop_tol,
                              truth=load_truth(f)))

    print('\n%-14s %8s %11s %8s %10s  %s'
          % ('run', 'walked', 'DRIFT', 'of dist', 'yaw err', 'mode'))
    print('-' * 78)
    for r in runs:
        g = r.get('gt')
        print('%-14s %7.2fm %10s %8s %10s  %s'
              % (r['note'][:14], r['travelled'],
                 '%.3f m' % g['err'] if g else 'no tape',
                 ('%.2f%%' % g['pct']) if g and g['pct'] == g['pct'] else '-',
                 ('%+.1f deg' % g['yaw_err']) if g and g['yaw_err'] is not None else '-',
                 'odometry only' if r['seedlike'] else 'relocalizer on'))
    print()
    # ONLY closed loops may enter this comparison. On an open route the end
    # pose is just where you stopped walking, not drift - comparing those
    # produced a confident "223x better" from two numbers that measured
    # nothing of the kind.
    withs = [r for r in runs if not r['seedlike'] and r.get('gt')]
    without = [r for r in runs if r['seedlike'] and r.get('gt')]
    if withs and without:
        w = median([r['gt']['err'] for r in withs])
        o = median([r['gt']['err'] for r in without])
        print('median ENDPOINT DRIFT vs the tape')
        print('  relocalizer on : %.3f m   (%d run(s))' % (w, len(withs)))
        print('  odometry only  : %.3f m   (%d run(s))' % (o, len(without)))
        if w > 0 and o > 0:
            print('  ratio          : %.1fx %s with the relocalizer'
                  % ((o / w) if o > w else (w / o),
                     'better' if o > w else 'WORSE'))
        print('\nBoth rows are map->body, so this is like-for-like: run B holds')
        print('the same map frame with ICP off, rather than comparing a chain')
        print('against one of its own links.')
    else:
        print('No return-to-start comparison: that needs one CLOSED loop of')
        print('each kind. Closed runs found - relocalizer on: %d, odometry '
              'only: %d.' % (len(withs), len(without)))
        openruns = [r['note'] for r in runs if not r['closed']]
        if openruns:
            print('Open routes (end pose is where you stopped, not drift): %s'
                  % ', '.join(openruns))
        print('\nThe correction column above still works on an open route:')
        print('every metre map->camera_init moves is odometry error the')
        print('relocalizer had to cancel. For the headline number, walk a')
        print('closed loop back to a marked spot.')
        print('\nRun B is: start_localization.sh --ros-args -p seed_only:=true')
    return 0


# ---------------------------------------------------------------- cli
def do_annotate(a):
    """Attach the tape measurement to a recorded run."""
    import json
    f = os.path.expanduser(a.csv)
    if not os.path.exists(f):
        print('no such file: %s' % f); return 2
    truth = dict(dx=a.dx, dy=a.dy, closed=bool(a.closed))
    if a.dyaw is not None:
        truth['dyaw'] = a.dyaw
    with open(truth_path(f), 'w') as fh:
        json.dump(truth, fh, indent=2)
    print('ground truth attached to %s' % os.path.basename(f))
    print('   ended  fwd %+.3f m, left %+.3f m%s%s'
          % (a.dx, a.dy,
             ', yaw %+.1f deg' % a.dyaw if a.dyaw is not None else '',
             '   [closed loop]' if a.closed else ''))
    print('\nnow:  driftlog.py compare <the csvs>')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description='Walking-run pose drift, with and without the relocalizer. '
                    'READ-ONLY: publishes nothing, commands nothing.')
    sub = ap.add_subparsers(dest='cmd')

    r = sub.add_parser('record', help='log one walking run')
    r.add_argument('--note', default='run',
                   help='label for the file, e.g. with-reloc / seed-only')
    r.add_argument('--out-dir', default=os.path.expanduser('~/.scuba/nav/logs'))
    r.add_argument('--settle', type=float, default=8.0,
                   help='s of stillness used to fix the start and end pose. '
                        'A single sample is not a pose: per-sample jump p90 is '
                        '0.0113 m on a motionless robot, so both ends are a '
                        'MEDIAN over this window')
    r.add_argument('--still-tol', type=float, default=0.05,
                   help='m of spread allowed within the settle window')
    r.add_argument('--still-timeout', type=float, default=90.0)
    r.add_argument('--path-min-step', type=float, default=0.05,
                   help='m. Distance accrues only after moving this far from '
                        'the last counted point, so jitter does not inflate it')
    r.add_argument('--period', type=float, default=0.1, help='s between samples')
    r.add_argument('--secs', type=float, default=40.0,
                   help='auto-stop the WALK phase after this long (0 = until '
                        'Ctrl-C). The still windows at each end are extra.')
    r.add_argument('--loop-tol', type=float, default=0.60,
                   help='m. If the route ends within this of where it started '
                        'it is treated as CLOSED and return-to-start error is '
                        'reported. Otherwise the route is open and that number '
                        'would be meaningless, so it is withheld.')
    r.add_argument('--tf-timeout', type=float, default=30.0)
    r.add_argument('--map-frame', default='map')
    r.add_argument('--odom-frame', default='camera_init')
    r.add_argument('--base-frame', default='body',
                   help="Nav2's robot_base_frame; keep it the same in both runs")

    an = sub.add_parser('annotate',
                        help='attach your tape measurement to a recorded run')
    an.add_argument('csv')
    an.add_argument('--dx', type=float, required=True,
                    help='metres the robot ACTUALLY ended FORWARD of its start, '
                         'measured along the START heading. Negative = behind.')
    an.add_argument('--dy', type=float, required=True,
                    help='metres it ACTUALLY ended to the LEFT of its start, '
                         'same frame. Negative = right.')
    an.add_argument('--dyaw', type=float, default=None,
                    help='degrees its FINAL heading differed from its start, '
                         'CCW positive. Omit if unmeasured - the position '
                         'error still works without it.')
    an.add_argument('--closed', action='store_true',
                    help='you returned it to the mark; dx/dy are the residual')

    c = sub.add_parser('compare', help='compare recorded runs')
    c.add_argument('files', nargs='+')
    c.add_argument('--path-min-step', type=float, default=0.05)
    c.add_argument('--loop-tol', type=float, default=0.60)

    a = ap.parse_args()
    if a.cmd == 'record':
        return do_record(a)
    if a.cmd == 'annotate':
        return do_annotate(a)
    if a.cmd == 'compare':
        return do_compare(a)
    ap.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
