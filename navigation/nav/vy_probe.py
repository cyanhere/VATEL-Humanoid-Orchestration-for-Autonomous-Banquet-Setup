#!/usr/bin/env python3
"""Characterise the G1's SIDE STEP: what vy moves it, and how far one pulse goes.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/vy_probe.py                      # plan only
    /usr/bin/python3 ~/.scuba/nav/vy_probe.py --go                 # range sweep
    /usr/bin/python3 ~/.scuba/nav/vy_probe.py --mode pulse --vy 0.25 --go

WHY THIS EXISTS
The per-edge sequence needs a lateral correction after the orientation step:
    reach edge -> orientation correction -> SIDE STEP back to the edge centre
    -> walk forward to the plane -> minor orientation correction -> stop at 0.30
The side-step algorithm will pulse sideways until the left/right error changes
sign, so it needs two numbers this script measures:
    1. the smallest vy the gait actually honours (the RANGE phase)
    2. how far ONE pulse moves the robot, and how repeatable that is (PULSE)
The second is the quantum of the sign-flip search, exactly as min_turn_deg is
for the heading version in edge_servo.

WHAT IT MEASURES, PER PULSE
  lateral   displacement along the robot's OWN left axis - the wanted motion
  forward   displacement along its own forward axis - unwanted coupling
  yaw drift accumulated heading change - the one that matters most, because the
            whole point of the side step is to fix position WITHOUT disturbing
            the orientation that was just corrected. A side step that rotates
            the robot defeats its own purpose.

Displacement is projected onto the robot's axes using the heading at pulse
START. Heading is accumulated DURING the pulse, never differenced end to end:
an endpoint difference wraps at +/-180 and reads a large turn as a small
opposite one.

REPEATS ARE NOT OPTIONAL
Measured on this gait, turn-per-command varies 40-60% with gait phase. A single
sample of a lateral pulse is therefore meaningless. Every setting is run
--repeats times and reported as mean +/- sd, with the spread being the number
the algorithm actually has to tolerate.

CLAMPS THAT WILL SILENTLY RUIN THE MEASUREMENT
  nav_bridge max_vy   default 0.20  <-- a sweep above this is CLIPPED, not
                                        measured. This already happened on the
                                        forward axis: the 0.35 and 0.40 steps
                                        were clipped to max_vx 0.30 and the
                                        sweep's top end measured nothing.
  nav_bridge deadband default 0.02  below this, vy is zeroed outright
  nav_bridge max_lin_acc            slews vy too; a short pulse may never
                                    reach the commanded value at all
This script QUERIES those parameters and refuses to run a sweep that would be
clipped. Raise the clamp instead:
    ros2 run g1_teleop nav_bridge --ros-args -p enable:=true \\
        -p max_vy:=0.50 -p max_lin_acc:=2.0

SAFETY
Publishes Twist on /cmd_vel only. No mode commands - the robot must already be
in FSM501. --alternate is ON by default so the robot oscillates about its start
position instead of walking sideways across the room. Zero velocity is sent on
exit and on Ctrl-C.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def mat_vec(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def yaw_of_R(R):
    return math.atan2(R[1][0], R[0][0])


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def bridge_params(node, names, bridge='/g1_nav_bridge', timeout=4.0):
    """Read nav_bridge's clamps so we can refuse a sweep it would clip."""
    cli = node.create_client(GetParameters, bridge + '/get_parameters')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetParameters.Request()
    req.names = list(names)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        return None
    out = {}
    for n, v in zip(names, res.values):
        out[n] = v.double_value if v.type == 3 else (
            v.integer_value if v.type == 2 else None)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['range', 'pulse'], default='range',
                    help='range = sweep |vy| at a fixed duration (find the '
                         'floor and the gain). pulse = sweep the duration at a '
                         'fixed vy (find the step quantum).')
    # range mode
    ap.add_argument('--from', dest='v_from', type=float, default=0.05)
    ap.add_argument('--to', dest='v_to', type=float, default=0.40)
    ap.add_argument('--step', type=float, default=0.05)
    ap.add_argument('--secs', type=float, default=3.0,
                    help='pulse length in range mode')
    # pulse mode
    ap.add_argument('--vy', type=float, default=0.25,
                    help='the vy used in pulse mode')
    ap.add_argument('--durations', type=float, nargs='+',
                    default=[0.2, 0.3, 0.4, 0.6, 0.8, 1.2],
                    help='pulse lengths to characterise, seconds')
    # both
    ap.add_argument('--repeats', type=int, default=3,
                    help='samples per setting. 1 is NOT enough on this gait.')
    ap.add_argument('--pre', type=float, default=10.0,
                    help='hold still this long BEFORE the command, then take '
                         'the baseline. Drift across this window is the noise '
                         'floor of the measurement.')
    ap.add_argument('--post', type=float, default=10.0,
                    help='hold still this long AFTER the command before reading '
                         'the end pose. MEASURED 2026-10-29: at --post 5 s, 9 of '
                         '25 pulses were STILL MOVING at the read, and the tail '
                         'of one pulse bled into the next baseline window and '
                         'appeared as 0.69 m of "noise". 10 s.')
    ap.add_argument('--quiet-eps', type=float, default=0.005,
                    help='m per sample below which the robot counts as still; '
                         'used only to report when motion ceased')
    ap.add_argument('--rest', type=float, default=0.0,
                    help='extra gap between pulses; --pre already settles')
    ap.add_argument('--rate', type=float, default=10.0)
    ap.add_argument('--alternate', dest='alternate', action='store_true',
                    default=True, help='flip direction each pulse (default)')
    ap.add_argument('--no-alternate', dest='alternate', action='store_false')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--base-frame', default='body',
                    help="'body' to match Nav2's robot_base_frame")
    ap.add_argument('--ref', choices=['frozen', 'map'], default='frozen',
                    help="frozen (default) = freeze map->camera_init at pulse "
                         "start and compose with live camera_init->body, so "
                         "relocalizer corrections during the pulse cannot look "
                         "like robot motion. map = the naive way; on this robot "
                         "that carries ~0.14 m of jitter per 3 s pulse and made "
                         "the first run unreadable. Kept only for comparison.")
    ap.add_argument('--bridge', default='/g1_nav_bridge')
    ap.add_argument('--skip-clamp-check', action='store_true',
                    help='run even if the sweep exceeds nav_bridge max_vy')
    ap.add_argument('--go', action='store_true', help='actually publish')
    a = ap.parse_args()

    # ---- build the plan -------------------------------------------------
    if a.mode == 'range':
        n = int(round((a.v_to - a.v_from) / a.step)) + 1
        mags = [round(a.v_from + i * a.step, 4) for i in range(n)]
        plan = [(v, a.secs) for v in mags for _ in range(a.repeats)]
        peak = max(mags)
    else:
        plan = [(a.vy, d) for d in a.durations for _ in range(a.repeats)]
        peak = abs(a.vy)

    total_t = sum(d for _, d in plan) + (a.pre + a.post + a.rest) * len(plan)
    worst = sum(abs(v) * d for v, d in plan)
    print('mode %s | %d pulses (%d settings x %d repeats) | %.0f s total'
          % (a.mode, len(plan), len(plan) // max(a.repeats, 1), a.repeats,
             total_t))
    print('  protocol per pulse: %.0f s still -> baseline -> command -> '
          '%.0f s still -> read' % (a.pre, a.post))
    if a.mode == 'range':
        print('  |vy| %s m/s, %.1f s each' % (mags, a.secs))
    else:
        print('  vy %.2f m/s, durations %s s' % (a.vy, a.durations))
    print('  commanded lateral travel if nothing cancels: %.1f m' % worst)
    if a.alternate:
        print('  --alternate ON: direction flips each pulse, so the robot '
              'oscillates about its start pose')
    else:
        print('  !! --no-alternate: travel ACCUMULATES to one side, up to '
              '%.1f m. Check the space.' % worst)
    print('  mode: NOT TOUCHED - only Twist on %s' % a.cmd_topic)

    if not a.go:
        print('\nplan only - nothing published. Add --go to run it.')
        return 0

    rclpy.init()
    node = Node('vy_probe')
    pub = node.create_publisher(Twist, a.cmd_topic, 10)
    buf = Buffer()
    TransformListener(buf, node)

    # ---- refuse a sweep the bridge would clip ---------------------------
    p = bridge_params(node, ['max_vy', 'deadband', 'max_lin_acc'], a.bridge)
    if p is None:
        print('\n!! could not read %s parameters - cannot verify the clamp.'
              % a.bridge)
        print('   nav_bridge may not be running. Continuing blind.')
    else:
        print('\nnav_bridge: max_vy %.2f  deadband %.3f  max_lin_acc %.2f'
              % (p.get('max_vy', -1), p.get('deadband', -1),
                 p.get('max_lin_acc', -1)))
        mv = p.get('max_vy')
        if mv and peak > mv + 1e-9:
            print('   !! this sweep peaks at %.2f but max_vy is %.2f.' % (peak, mv))
            print('      Everything above %.2f would be CLIPPED, not measured'
                  ' - the same way the' % mv)
            print('      forward sweep silently measured nothing above max_vx.')
            print('      Relaunch the bridge with  -p max_vy:=%.2f' % (peak + 0.05))
            if not a.skip_clamp_check:
                print('      Refusing to run. --skip-clamp-check overrides.')
                node.destroy_node(); rclpy.shutdown()
                return 1
        acc = p.get('max_lin_acc')
        if acc:
            shortest = min(d for _, d in plan)
            t_ramp = peak / acc
            if t_ramp > shortest * 0.5:
                print('   note: at max_lin_acc %.2f, reaching %.2f takes %.2f s,'
                      % (acc, peak, t_ramp))
                print('         but the shortest pulse is %.2f s - short pulses'
                      % shortest)
                print('         will never reach the commanded vy. Raise'
                      ' max_lin_acc to 2.0.')

    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    def grab(parent, child):
        """(R, t) of parent->child, or None."""
        try:
            tr = buf.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            return None
        q = tr.transform.rotation
        v = tr.transform.translation
        return (quat_R(q), [v.x, v.y, v.z])

    def pose(mc=None):
        """Robot pose in a GRAVITY-LEVELLED, relocalizer-free frame.

        mc is map->camera_init FROZEN at pulse start. Composing it with live
        camera_init->body gives a levelled frame that the relocalizer cannot
        move during the pulse.

        Measuring in `map` directly is what ruined the first run: on a
        stationary robot map->camera_init moved 1.870 m in 40 s (0.047 m/s),
        so a 3 s pulse carried ~0.14 m of pure localisation jitter - larger
        than every real lateral motion below vy 0.25, with a random sign.

        Measuring in `camera_init` directly is also wrong: its z points down
        (the Mid-360 is mounted inverted), so yaw runs backwards there and
        "left" would come out as right. That is why odom_2d exists.
        """
        cb = grab(a.odom_frame, a.base_frame)
        if cb is None:
            return None
        if mc is None:                      # --ref map: the old, noisy way
            mb = grab(a.map_frame, a.base_frame)
            if mb is None:
                return None
            R, t = mb
        else:
            R = mat_mul(mc[0], cb[0])
            t = [mc[1][i] + mat_vec(mc[0], cb[1])[i] for i in range(3)]
        return (t[0], t[1], math.degrees(yaw_of_R(R)))

    if pose(None) is None:
        print('\n!! no %s -> %s transform. FAST-LIO and the relocalizer must '
              'be up.' % (a.map_frame, a.base_frame))
        node.destroy_node(); rclpy.shutdown()
        return 1
    if pub.get_subscription_count() == 0:
        print('\n!! nothing subscribed to %s - nav_bridge is not running, so '
              'nothing would reach the robot.' % a.cmd_topic)
        node.destroy_node(); rclpy.shutdown()
        return 1

    def drive(vy, dur):
        """Publish vy for dur. Returns (lateral, forward, yaw_drift).

        Projection uses the heading at pulse START; heading is accumulated
        during the pulse so it cannot alias at +/-180.
        """
        msg = Twist()
        msg.linear.y = float(vy)
        # FREEZE map->camera_init for the whole pulse, so the relocalizer's
        # corrections cannot masquerade as robot motion.
        mc = None if a.ref == 'map' else grab(a.map_frame, a.odom_frame)
        if a.ref != 'map' and mc is None:
            return None
        # ---- PRE: hold still, THEN take the baseline -----------------------
        # Two reasons. The baseline must come from a settled robot, not from
        # whatever it was still doing after the previous pulse. And the drift
        # measured across this quiet window is the NOISE FLOOR of the whole
        # experiment: any lateral reading below it means nothing.
        halt = Twist()
        pre_a = pose(mc)
        t_pre = time.time()
        while time.time() - t_pre < a.pre:
            pub.publish(halt)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
        p0 = pose(mc)
        if p0 is None:
            return None
        h0 = math.radians(p0[2])
        pre_drift = (math.hypot(p0[0] - pre_a[0], p0[1] - pre_a[1])
                     if pre_a else float('nan'))

        prev = p0
        turned = 0.0
        t1 = time.time()
        while time.time() - t1 < dur:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
            cur = pose(mc)
            if cur is not None and prev is not None:
                turned += wrap(cur[2] - prev[2])
            if cur is not None:
                prev = cur
        # The robot keeps moving for ~2 s AFTER the command ends. MEASURED
        # 2026-10-29: a 2.0 s vy 0.30 pulse moved 0.094 m during the command
        # and a further 0.158 m afterwards - and it did not start moving at all
        # for the first ~1 s. nav_bridge's cmd_duration is 0.5 s and the gait
        # must finish its step cycle, so the overrun is structural.
        #
        # A fixed short settle therefore captures an arbitrary FRACTION of each
        # motion, varying with gait phase. That is what made the first run
        # unreadable: random signs and enormous spread. So wait for the robot to
        # actually stop, not for a fixed time.
        p_cmd = pose(mc)
        # ---- POST: hold still for a FIXED window, then read the end pose ----
        # Fixed, not adaptive: --post 5 s is comfortably beyond the measured
        # ~2 s overrun, and a fixed window needs no threshold to tune. The time
        # at which motion actually ceased is still recorded, so we learn whether
        # 5 s is enough or more than needed.
        ceased = float('nan')
        t2 = time.time()
        while time.time() - t2 < a.post:
            pub.publish(halt)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(1.0 / a.rate)
            cur = pose(mc)
            if cur is not None and prev is not None:
                turned += wrap(cur[2] - prev[2])
                if math.hypot(cur[0] - prev[0], cur[1] - prev[1]) >= a.quiet_eps:
                    ceased = time.time() - t2      # last time it was still moving
            if cur is not None:
                prev = cur
        p1 = pose(mc)
        if p1 is None:
            return None

        def split(pa, pb):
            dx, dy = pb[0] - pa[0], pb[1] - pa[1]
            return (-dx * math.sin(h0) + dy * math.cos(h0),   # +lat = LEFT
                    dx * math.cos(h0) + dy * math.sin(h0))

        lat, fwd = split(p0, p1)                 # everything, incl. overrun
        lat_cmd = split(p0, p_cmd)[0] if p_cmd else float('nan')
        return (lat, fwd, turned, lat_cmd, ceased, pre_drift)

    print()
    print('  %6s %6s  %6s  %8s %7s %8s %7s %6s  %s'
          % ('vy', 'dur', 'noise', 'lat m', 'in cmd', 'fwd m', 'yaw deg',
             'ceas', 'note'))
    print('  ' + '-' * 92)
    rows = []
    sign = 1.0
    try:
        for vy, dur in plan:
            cmd_vy = sign * vy
            r = drive(cmd_vy, dur)
            if a.alternate:
                sign = -sign
            if r is None:
                print('  %6.2f %6.2f   %8s' % (vy, dur, 'no TF'))
                continue
            lat, fwd, turned, lat_cmd, ceased, pre_drift = r
            # Does the motion go the way it was asked? +vy is LEFT in REP-103,
            # but this robot's yaw convention already surprised us once, so the
            # lateral sign is measured rather than assumed.
            agrees = (lat * cmd_vy) > 0
            note = ''
            if abs(lat) < 0.01:
                note = 'DEAD'
            elif not agrees:
                note = 'WRONG WAY - +vy is not left on this robot'
            elif abs(turned) > 3.0:
                note = 'yaw drift!'
            if ceased == ceased and ceased > a.post - 0.5:
                note = (note + ' ' if note else '') + 'STILL MOVING AT READ'
            if pre_drift == pre_drift and abs(lat) < 2.0 * pre_drift:
                note = (note + ' ' if note else '') + 'below noise'
            rows.append((vy, dur, abs(lat), fwd, turned, agrees, pre_drift))
            print('  %6.2f %6.2f  %6.3f  %8.3f %7.3f %+8.3f %+7.2f %6s  %s'
                  % (vy, dur, pre_drift, abs(lat), abs(lat_cmd), fwd, turned,
                     ('%.1f' % ceased) if ceased == ceased else '-', note))
            # --pre already settles the robot; --rest is an extra gap
            t3 = time.time()
            while time.time() - t3 < a.rest:
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.01)
                time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        for _ in range(12):
            pub.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.02)
        print('zero velocity sent')

    # ---- summary --------------------------------------------------------
    if not rows:
        node.destroy_node(); rclpy.shutdown()
        return 0

    groups = {}
    noise = [r[6] for r in rows if r[6] == r[6]]
    for vy, dur, lat, fwd, turned, agrees, pre in rows:
        groups.setdefault((vy, dur), []).append((lat, fwd, turned, agrees))
    if noise:
        nf = statistics.mean(noise)
        print()
        print('  === NOISE FLOOR ===')
        print('  drift while commanded ZERO, over %.0f s: mean %.3f m, max %.3f m'
              % (a.pre, nf, max(noise)))
        print('  Any lateral reading near or below that is not motion. This is')
        print('  the number the first run lacked, which is why 0.0x m readings')
        print('  looked like real but tiny side steps.')

    n_wrong = sum(1 for r in rows if r[2] >= 0.01 and not r[5])
    n_moved = sum(1 for r in rows if r[2] >= 0.01)
    if n_moved and n_wrong:
        print()
        print('  !! LATERAL SIGN: %d of %d moving pulses went OPPOSITE to the'
              % (n_wrong, n_moved))
        print('     commanded vy. If that is all of them, +vy is RIGHT on this')
        print('     robot, not left, and the side-step algorithm must negate.')
        print('     If it is only some, the measurement is unreliable, not the'
              ' convention.')

    print()
    print('  === per setting, %d repeats ===' % a.repeats)
    print('  %6s %6s   %16s %14s %14s'
          % ('vy', 'dur', 'lateral m', 'fwd drift m', 'yaw drift deg'))
    def ms(v):
        if len(v) == 1:
            return '%.3f' % v[0]
        return '%.3f +/-%.3f' % (statistics.mean(v), statistics.stdev(v))
    live = []
    for (vy, dur), vals in sorted(groups.items()):
        lats = [x[0] for x in vals]
        fwds = [x[1] for x in vals]
        yaws = [abs(x[2]) for x in vals]
        print('  %6.2f %6.2f   %16s %14s %14s'
              % (vy, dur, ms(lats), ms(fwds), ms(yaws)))
        if statistics.mean(lats) >= 0.01:
            live.append((vy, dur, statistics.mean(lats),
                         statistics.mean(yaws),
                         (statistics.stdev(lats) if len(lats) > 1 else 0.0)))

    print()
    if not live:
        print('  NOTHING MOVED at any setting. Either vy is below the gait '
              'floor everywhere')
        print('  tested, or the bridge is clamping/deadbanding it. Check '
              'max_vy and deadband.')
    else:
        lo = min(live, key=lambda r: r[0])
        print('  lowest vy that moved: %.2f m/s -> %.3f m per %.2f s pulse'
              % (lo[0], lo[2], lo[1]))
        print('  gain at that setting: %.2f (achieved / commanded x time)'
              % (lo[2] / max(lo[0] * lo[1], 1e-9)))
        worst_yaw = max(live, key=lambda r: r[3])
        print('  worst yaw drift seen: %.2f deg at vy %.2f / %.2f s'
              % (worst_yaw[3], worst_yaw[0], worst_yaw[1]))
        print()
        print('  FOR THE SIDE-STEP ALGORITHM')
        print('  The sign-flip search resolution is the smallest pulse that')
        print('  moves RELIABLY - mean lateral well above its own spread:')
        for vy, dur, m, y, sd in sorted(live, key=lambda r: r[2]):
            ok = 'usable' if (sd == 0.0 or m > 3 * sd) else 'too variable'
            print('    vy %.2f for %.2f s -> %.3f m (sd %.3f, yaw %.1f deg)  %s'
                  % (vy, dur, m, sd, y, ok))
        print()
        print('  A side step must not disturb the heading the orientation step')
        print('  just set. Any setting with yaw drift above the edge servo'
              ' tolerance')
        print('  (ang_tol 1.0 deg) needs a heading re-check after each pulse.')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
