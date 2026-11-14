#!/usr/bin/env python3
"""Measure how accurately this robot turns, using the table edge as protractor.

Stand the robot where the plane detector has a solid lock on an edge, then:

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/turn_calib.py            # dry, shows the plan
    /usr/bin/python3 ~/.scuba/livemap/turn_calib.py --go

It commands OPEN-LOOP turns of known duration and measures what edge_angle
actually did. Directions alternate, so the robot stays near where it started.

Changes no mode - only Twist on /cmd_vel. Set FSM 501 yourself.

Reports commanded-vs-achieved gain, repeatability, and the smallest turn that
reliably beats the measurement noise. That last number is the one to use as
--ang-tol: asking for finer than that is asking it to chase noise.
"""
import argparse
import json
import math
import os
import sys
import time
import urllib.request


def read_avg(url, n, gap=0.12):
    """Mean of n readings. A single sample is not enough while the robot is
    re-balancing: measured on the robot, averaging 8 cuts the spread from
    0.27 to 0.10 deg."""
    got = []
    for _ in range(n):
        v = read(url, tries=1)
        if v is not None:
            got.append(v)
        time.sleep(gap)
    return (sum(got) / len(got)) if got else None


def wait_settled(url, tol, window, timeout, gap=0.15):
    """Block until edge_angle stops DRIFTING.

    A standing humanoid keeps re-balancing after it is handled, and that
    drift is not noise: a calibration run measured 3.47 deg of "noise" that
    was really the angle sliding from -8.2 to -11.1 while it settled.
    Returns (settled, spread).
    """
    t0 = time.time()
    buf = []
    while time.time() - t0 < timeout:
        v = read(url, tries=1)
        if v is not None:
            buf.append(v)
            if len(buf) > window:
                buf.pop(0)
            if len(buf) == window:
                spread = max(buf) - min(buf)
                if spread <= tol:
                    return True, spread
        time.sleep(gap)
    return False, (max(buf) - min(buf)) if len(buf) > 1 else float('nan')


def read(url, tries=3):
    for _ in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=0.5) as r:
                d = json.loads(r.read().decode())
            if d.get('ok') and d.get('edge_angle') is not None:
                return float(d['edge_angle'])
        except Exception:
            pass
        time.sleep(0.1)
    return None


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--go', action='store_true')
    ap.add_argument('--out',
                    default='~/.scuba/livemap/turn_calib.json',
                    help="'' to not write results")
    ap.add_argument('--plane-url', default='http://localhost:8093/stat')
    ap.add_argument('--w-turn', type=float, default=0.40, help='rad/s')
    ap.add_argument('--durations', default='0.15,0.25,0.4,0.6,0.9',
                    help='s, comma separated')
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--avg', type=int, default=8,
                    help='samples averaged per reading')
    ap.add_argument('--settle-tol', type=float, default=0.6,
                    help='deg; wait until the angle stops drifting by more '
                         'than this before measuring')
    ap.add_argument('--settle-window', type=int, default=8)
    ap.add_argument('--settle-timeout', type=float, default=25.0)
    ap.add_argument('--max-rel', type=float, default=0.5,
                    help='spread/achieved above this = not reliable')
    ap.add_argument('--settle', type=float, default=1.5)
    ap.add_argument('--rate', type=float, default=30.0)
    ap.add_argument('--ang-accel', type=float, default=2.0,
                    help="the bridge's max_ang_acc, for the prediction column")
    a = ap.parse_args()

    durs = [float(x) for x in a.durations.split(',') if x.strip()]

    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit('!! ROS_DISTRO is %r, not foxy.' % d)

    ang = read(a.plane_url)
    if ang is None:
        print('!! no edge_angle from %s' % a.plane_url)
        print('!! the detector must have a solid lock on an edge for this to')
        print('!! measure anything. Check http://<robot>:8093 first.')
        return 1
    print('edge_angle right now: %+.2f deg' % ang)
    print('plan: %d durations x %d reps, alternating direction, w=%.2f rad/s'
          % (len(durs), a.reps, a.w_turn))
    print()
    print('  %-8s %-12s %-10s' % ('dur', 'open-loop deg', 'predicted'))
    for t in durs:
        # ramp-limited: peak w is min(w_turn, accel*t); angle is the integral
        wp = min(a.w_turn, a.ang_accel * t)
        if wp >= a.w_turn:
            tr = a.w_turn / a.ang_accel
            pred = a.w_turn * (t - tr) + 0.5 * a.w_turn * tr
        else:
            pred = 0.5 * wp * t
        print('  %-8.2f %-12.1f %-10s'
              % (t, math.degrees(a.w_turn * t), '%.1f' % math.degrees(pred)))
    if not a.go:
        print()
        print('dry run - nothing published. Add --go to measure.')
        return 0

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    rclpy.init()
    n = Node('turn_calib')
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    for _ in range(60):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(n, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('!! nothing subscribed to /cmd_vel - nav_bridge is not running')
        n.destroy_node(); rclpy.shutdown()
        return 1

    def spin_for(wz, dur):
        t = Twist()
        t.angular.z = float(wz)
        t0 = time.time()
        while time.time() - t0 < dur:
            pub.publish(t)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
        z = Twist()
        for _ in range(15):
            pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)

    # noise floor: how much does edge_angle wander with the robot still?
    print()
    print('waiting for the robot to stop drifting (it re-balances for several')
    print('seconds after being handled) ...')
    ok, sp = wait_settled(a.plane_url, a.settle_tol, a.settle_window,
                          a.settle_timeout)
    if ok:
        print('  settled: %.2f deg over %d samples' % (sp, a.settle_window))
    else:
        print('  !! still drifting %.2f deg after %.0f s.' % (sp, a.settle_timeout))
        print('  !! Measuring now would record that drift as turn error.')
        print('  !! Let it stand, or raise --settle-tol if this is as good')
        print('  !! as it gets.')
    print()
    print('measuring detector noise with the robot STILL ...')
    base = []
    for _ in range(16):
        v = read(a.plane_url)
        if v is not None:
            base.append(v)
        time.sleep(0.2)
    noise = (max(base) - min(base)) if len(base) > 2 else float('nan')
    print('  edge_angle spread over %d samples: %.2f deg' % (len(base), noise))
    if len(base) > 2:
        half = len(base) // 2
        drift = abs(sum(base[half:]) / (len(base) - half)
                    - sum(base[:half]) / half)
        print('  of which DRIFT (2nd half - 1st half): %.2f deg' % drift)
        if drift > 0.5 * noise and noise > 0.3:
            print('  !! most of that is drift, not noise - it is still '
                  'settling')

    print()
    print('  %-7s %-5s %-10s %-10s %-8s' %
          ('dur', 'dir', 'before', 'after', 'achieved'))
    results = {}
    sign = 1.0
    try:
        for t in durs:
            got = []
            for r in range(a.reps):
                before = read_avg(a.plane_url, a.avg)
                if before is None:
                    print('  lost the edge - stopping')
                    raise KeyboardInterrupt
                spin_for(sign * a.w_turn, t)
                time.sleep(a.settle)
                after = read_avg(a.plane_url, a.avg)
                if after is None:
                    print('  lost the edge after turning - stopping')
                    raise KeyboardInterrupt
                ach = abs(after - before)
                got.append(ach)
                print('  %-7.2f %-5s %-10.2f %-10.2f %-8.2f'
                      % (t, '+' if sign > 0 else '-', before, after, ach))
                sign = -sign
            results[t] = got
    except KeyboardInterrupt:
        print('  interrupted')
    finally:
        z = Twist()
        for _ in range(15):
            pub.publish(z)
            rclpy.spin_once(n, timeout_sec=0.005)
            time.sleep(0.02)
        n.destroy_node()
        rclpy.shutdown()

    if not results:
        return 1
    print()
    print('=' * 64)
    print('  %-8s %-10s %-10s %-9s %-10s %s'
          % ('dur', 'commanded', 'achieved', 'spread', 'gain', 'verdict'))
    smallest = None
    for t in sorted(results):
        g = results[t]
        if not g:
            continue
        mean = sum(g) / len(g)
        spread = max(g) - min(g)
        cmd = math.degrees(a.w_turn * t)
        rel = spread / mean if mean > 0 else 9.9
        print('  %-8.2f %-10.1f %-10.2f %-9.2f %-10.2f %s'
              % (t, cmd, mean, spread, mean / cmd if cmd else 0.0,
                 'usable' if (mean > 3 * max(noise, 0.15) and rel < a.max_rel)
                 else 'too noisy (rel %.2f)' % rel))
        rel = spread / mean if mean > 0 else 9.9
        if (smallest is None and mean > 3 * max(noise, 0.15)
                and rel < a.max_rel):
            smallest = (t, mean, spread)
    print('=' * 64)
    print('detector noise (robot still)      : %.2f deg' % noise)
    if smallest:
        t, mean, spread = smallest
        # The robot cannot reliably correct an error SMALLER than the
        # smallest turn it can reliably make, so that is the deadzone.
        tol = math.ceil(mean * 2.0) / 2.0
        print('smallest RELIABLE turn            : %.2f deg (at dur %.2f s)'
              % (mean, t))
        print('repeatability at that size        : %.2f deg' % spread)
        print()
        print('  -> use  --ang-tol %.1f' % tol)
        print('     Anything tighter is chasing noise: the robot cannot')
        print('     reliably make a correction smaller than this.')
        if a.out:
            try:
                path = os.path.expanduser(a.out)
                with open(path, 'w') as f:
                    json.dump(dict(ang_tol=tol, noise=noise,
                                   smallest_turn=mean, spread=spread,
                                   w_turn=a.w_turn, dur=t,
                                   gain={str(k): sum(v) / len(v)
                                         for k, v in results.items()},
                                   stamp=time.time()), f, indent=1)
                print()
                print('  wrote %s - edge_approach_test.py picks this up'
                      % path)
            except Exception as exc:
                print('  (could not write %s: %s)' % (a.out, exc))
    else:
        print('no commanded turn rose above the noise - either the robot is')
        print('not turning at all, or w_turn is under the gait threshold.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
