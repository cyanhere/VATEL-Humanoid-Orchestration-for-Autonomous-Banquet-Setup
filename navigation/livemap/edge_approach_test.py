#!/usr/bin/env python3
"""Stage test: walk forward onto a table edge and stop at a set standoff.

This is the mission's search_forward + approach stages ON THEIR OWN - no Nav2,
no registry, no table selection, no mode changes. You stand the robot square to
an edge by hand; this drives it in until the camera sees the tabletop and then
servos to --target metres from the edge.

    source ~/.scuba/lio_ws/env.sh

    # 1. look only, no motion - check the placement first
    /usr/bin/python3 ~/.scuba/livemap/edge_approach_test.py --measure

    # 2. rehearse: prints every burst it WOULD send, moves nothing
    /usr/bin/python3 ~/.scuba/livemap/edge_approach_test.py --dry-run

    # 3. for real (nav_bridge must be up with enable:=true, robot in walk mode)
    /usr/bin/python3 ~/.scuba/livemap/edge_approach_test.py

Needs the ROS 1 plane detector running in its OWN terminal:
    source ~/.scuba/camview/env.sh
    python3 ~/.scuba/camview/plane_detect.py --cam-h 1.25 --table-h 0.85

It servos on edge_x - the camera's HORIZONTAL ground distance to the edge,
which the detector derives from the drop it actually measures, so it does not
inherit a cam_h guess. --servo depth uses edge_depth and --pitch instead.
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time

MISSION = os.path.expanduser('~/.scuba/livemap/table_mission.py')


def load_mission():
    """Reuse the mission's OWN Plane client and approach_move.

    Importing rather than copying is the point: if this test passes, the code
    the mission runs is the code that passed.
    """
    spec = importlib.util.spec_from_file_location('table_mission', MISSION)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def fmt(d, target, pitch):
    """One detector reading as a line."""
    if d is None:
        return 'no reading'
    if not d.get('ok'):
        return 'ok=False  (%s)' % d.get('why', '?')
    ang = d.get('edge_angle')
    return ('edge_x %.3f m  (target %.2f, err %+.3f)  depth %.3f  '
            'angle %s  rms %.0f mm  cover %.3f  bins %s  %s'
            % (d['edge_x'], target, d['edge_x'] - target, d['edge_depth'],
               ('%+.1f deg' % ang) if ang is not None else '  n/a  ',
               1000 * d['rms'], d.get('cover', float('nan')),
               d.get('edge_bins', '?'),
               'TRIGGER' if d.get('trigger') else 'no trigger'))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--target', type=float, default=0.30,
                    help='metres from the camera to the table edge to stop at')
    # Give the depth to servo to DIRECTLY, measured at the real standoff,
    # instead of deriving it from cam_h/table_h/pitch. Measuring it beats
    # computing it: edge_x is computed by the detector from the drop IT
    # measures, so if its --cam-h is wrong, edge_x is wrong too and servoing
    # on it quietly aims at the wrong distance.
    ap.add_argument('--target-depth', type=float, default=-1.0,
                    help='edge_depth, in m, that means "at --target". '
                         'Overrides the cam_h/table_h/pitch calculation and '
                         'switches --servo to depth')
    ap.add_argument('--servo', choices=['edge_x', 'depth'], default='edge_x')
    ap.add_argument('--plane-url', default='http://localhost:8093/stat')
    ap.add_argument('--plane-max-age', type=float, default=2.0)
    ap.add_argument('--measure', action='store_true',
                    help='read the detector forever, never move')
    ap.add_argument('--dry-run', action='store_true',
                    help='print the bursts, publish nothing')
    ap.add_argument('--no-align', action='store_true',
                    help='skip the squaring-up stage')
    ap.add_argument('--align-after', action='store_true',
                    help='square up AGAIN after the approach')
    ap.add_argument('--ang-accel', type=float, default=0.70,
                    help="nav_bridge's max_ang_acc. Smallest possible turn is "
                         "w_turn^2/this - at 0.70 with w 0.40 that is 13 deg, "
                         "far coarser than --ang-tol. Raise with "
                         "-p max_ang_acc:=2.0")
    ap.add_argument('--flip-align', action='store_true',
                    help='invert the edge_angle sign convention')
    ap.add_argument('--no-autoflip', action='store_true',
                    help='do not auto-detect an inverted edge_angle sign')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    # motion, matched to what this gait actually honours: under ~0.20 m/s the
    # robot ignores the command outright, so SPEED is fixed and only the burst
    # LENGTH varies
    ap.add_argument('--v-fwd', type=float, default=0.22)
    ap.add_argument('--w-turn', type=float, default=0.40)
    ap.add_argument('--burst-max', type=float, default=0.8, help='s')
    ap.add_argument('--settle', type=float, default=1.2, help='s after a burst')
    ap.add_argument('--search-burst', type=float, default=0.25,
                    help='s per burst while hunting for the tabletop; '
                         '0.25 s at 0.22 m/s is about 5.5 cm')
    ap.add_argument('--tol', type=float, default=0.02, help='m')
    # -1 = derive it from what the robot can actually do. Asking for a
    # correction finer than the hardware's own repeatability just makes it
    # chase noise, which is what an arbitrary 3.0 was doing.
    ap.add_argument('--ang-tol', type=float, default=-1.0,
                    help='deg; -1 derives it from w_turn/ang_accel/poll')
    ap.add_argument('--turn-gain-err', type=float, default=0.13,
                    help='measured gait overshoot: 0.60 commanded read 0.68')
    # Measures what actually matters for a CLOSED-LOOP aligner: where it
    # lands, not how repeatable one blind turn is. Open-loop turn-per-command
    # on this robot varies 40-60% (gait phase), but the loop stops on the
    # MEASURED angle, so that variance changes the speed, not the endpoint.
    # Turn-per-command on this robot varies 40-60% with gait phase
    # (measured: the same 0.40 s command gave 1.68, 1.91 and 4.65 deg), so
    # predicting a correction size does not work. Stepping until the angle
    # CROSSES ZERO does not need a prediction - it only needs the direction,
    # and the residual is bounded by one step.
    ap.add_argument('--align-mode', choices=['step', 'continuous'],
                    default='step')
    ap.add_argument('--step-dur', type=float, default=0.4,
                    help='s per correction step')
    ap.add_argument('--cam-delay', type=float, default=0.5,
                    help='s to wait after a step before believing the camera')
    ap.add_argument('--max-steps', type=int, default=12)
    ap.add_argument('--refine', type=int, default=0,
                    help='after crossing, halve the step and cross back, N '
                         'times. Each pass roughly halves the residual.')
    ap.add_argument('--align-only', type=int, default=0,
                    help='run N align cycles and report the residual, then '
                         'stop. No search, no approach.')
    ap.add_argument('--angle-avg', type=int, default=8,
                    help='edge_angle samples averaged per decision; 1 '
                         'disables (the robot drifts while re-balancing)')
    ap.add_argument('--settle-tol', type=float, default=0.6,
                    help='deg; wait for drift below this before aligning')
    ap.add_argument('--settle-timeout', type=float, default=15.0)
    ap.add_argument('--turn-calib',
                    default='~/.scuba/livemap/turn_calib.json',
                    help='measured turn accuracy, overrides the estimate')
    ap.add_argument('--angle-noise', type=float, default=0.3,
                    help='deg; edge_angle spread with the robot still. '
                         'turn_calib.py measures this')
    ap.add_argument('--max-iters', type=int, default=10)
    # HARD cap. Nothing here uses Nav2, so there is no obstacle avoidance while
    # it drives - this is the only thing stopping a runaway.
    ap.add_argument('--max-travel', type=float, default=1.50,
                    help='m of total forward motion before giving up')
    ap.add_argument('--pitch', type=float, default=63.6,
                    help='only used by --servo depth')
    ap.add_argument('--cam-h', type=float, default=1.25)
    ap.add_argument('--table-h', type=float, default=0.85)
    # nav_bridge slews toward the commanded velocity at max_lin_acc, so a
    # SHORT burst never reaches a speed the gait will honour: at 0.35 m/s^2 a
    # 0.25 s burst peaks at 0.0875 m/s, the gait ignores anything under ~0.20,
    # and the robot just twitches. Observed on the robot and reproduced in the
    # simulator. Tell this script what the bridge is set to so it can refuse
    # impossible tolerances instead of hunting forever.
    ap.add_argument('--accel', type=float, default=0.35,
                    help="nav_bridge's max_lin_acc. Raise BOTH together: "
                         "ros2 run g1_teleop nav_bridge --ros-args "
                         "-p enable:=true -p max_lin_acc:=2.0")
    ap.add_argument('--gait-min', type=float, default=0.20,
                    help='forward speed the gait ignores below (measured)')
    ap.add_argument('--poll', type=float, default=0.05,
                    help='s between detector reads while driving')
    ap.add_argument('--coarse', type=float, default=0.10,
                    help='m; above this error drive continuously, below it '
                         'pulse')
    ap.add_argument('--stall-after', type=float, default=0.20,
                    help='m of commanded motion with no edge_x change = stall')
    ap.add_argument('--no-bridge-check', action='store_true')
    a = ap.parse_args()

    M = load_mission()
    M.require_ros2()
    plane = M.Plane(a.plane_url, a.plane_max_age)

    # depth target, for --servo depth and for reporting
    th = math.radians(a.pitch)
    if a.target_depth > 0:
        tgt_depth = a.target_depth
        a.servo = 'depth'
        drop_implied = (tgt_depth - a.target * math.cos(th)) / math.sin(th)
        print('target depth given directly: %.3f m at %.2f m standoff'
              % (tgt_depth, a.target))
        print('  -> implies a camera-to-tabletop drop of %.3f m'
              % drop_implied)
        print('  -> implies a camera height of %.3f m (tabletop %.2f m)'
              % (a.table_h + drop_implied, a.table_h))
        print('  -> plane_detect must be started with --cam-h %.3f or its'
              % (a.table_h + drop_implied))
        print('     trigger band will not cover this drop and it never fires')
        print()
    else:
        tgt_depth = ((a.cam_h - a.table_h) * math.sin(th)
                     + a.target * math.cos(th))

    print('plane detector : %s' % a.plane_url)
    print('target standoff: %.2f m   (edge_depth would read %.3f m)'
          % (a.target, tgt_depth))
    print('servo variable : %s' % a.servo)
    print('travel cap     : %.2f m%s'
          % (a.max_travel, '   [DRY RUN - nothing is published]'
             if a.dry_run else ''))
    # The ONLY thing this script ever sends is Twist on --cmd-topic. It never
    # calls nav_bridge's ~/start, ~/damp or ~/estop services, which are the
    # only things in that node that change FSM. Set the mode yourself.
    print('mode           : NOT TOUCHED - this script only publishes Twist on')
    print('                 %s. Set FSM 501 yourself before running.'
          % a.cmd_topic)
    # --- what the robot can actually resolve in yaw ---
    coast_deg = math.degrees(a.w_turn ** 2 / (2.0 * max(a.ang_accel, 1e-6)))
    poll_deg = math.degrees(a.w_turn * a.poll)
    floor_deg = math.hypot(poll_deg, a.turn_gain_err * coast_deg) + a.angle_noise
    floor_deg = math.ceil(floor_deg * 2.0) / 2.0
    # A MEASURED number beats a derived one. turn_calib.py writes this.
    cal = {}
    try:
        with open(os.path.expanduser(a.turn_calib)) as f:
            cal = json.load(f)
    except Exception:
        pass
    if a.ang_tol < 0 and cal.get('ang_tol'):
        a.ang_tol = float(cal['ang_tol'])
        a.angle_noise = float(cal.get('noise', a.angle_noise))
        auto = ' (MEASURED by turn_calib, smallest reliable turn %.2f deg)' % (
            cal.get('smallest_turn', float('nan')))
    elif a.ang_tol < 0:
        a.ang_tol = floor_deg
        auto = ' (derived - run turn_calib.py to measure it instead)'
    else:
        auto = ''
    print('turn resolution:')
    print('  coast after stop      %.1f deg   w^2/(2*ang_accel)' % coast_deg)
    print('  travel between polls  %.1f deg   w*poll' % poll_deg)
    print('  gain error on coast   %.1f deg   %.0f%% of coast'
          % (a.turn_gain_err * coast_deg, 100 * a.turn_gain_err))
    print('  detector noise        %.1f deg' % a.angle_noise)
    print('  -> floor %.1f deg,  --ang-tol %.1f%s' % (floor_deg, a.ang_tol, auto))
    if a.align_mode == 'step':
        print('  (step mode stops dead between steps, so the coast term above')
        print('   does not apply - the residual is bounded by --ang-tol.')
        print('   Measured on this robot: --ang-tol 1.0 -> 0.67 deg residual)')
    elif a.ang_tol < floor_deg:
        print('  !! --ang-tol %.1f is FINER than the %.1f deg floor for '
              'CONTINUOUS mode' % (a.ang_tol, floor_deg))
        print('  !! - it will chase the coast. Use --align-mode step.')
    print('  smallest DISCRETE turn %.1f deg   w^2/ang_accel'
          % math.degrees(a.w_turn ** 2 / max(a.ang_accel, 1e-6)))
    print()

    mm = a.v_fwd ** 2 / max(a.accel, 1e-6)
    print('min possible move: %.3f m  (v^2/accel, ramp up + down at '
          '%.2f m/s^2)' % (mm, a.accel))
    if mm > a.tol:
        new_tol = math.ceil(mm * 1000.0 / 5.0) * 5.0 / 1000.0
        print('  --tol %.3f m is finer than that, so raising it to %.3f m'
              % (a.tol, new_tol))
        print('  (to go finer, raise max_lin_acc further or lower --v-fwd:')
        print('   the minimum move is v^2/accel)')
        a.tol = new_tol
    print()

    d = plane.read()
    if d is None:
        print('!! the plane detector is not answering at %s' % a.plane_url)
        print('!! start it in its OWN terminal (it is ROS 1, not foxy):')
        print('!!     source ~/.scuba/camview/env.sh')
        print('!!     python3 ~/.scuba/camview/plane_detect.py '
              '--cam-h %.2f --table-h %.2f' % (a.cam_h, a.table_h))
        return 1
    print('first reading  : %s' % fmt(d, a.target, a.pitch))
    # Cross-check the depth target against the drop the camera actually
    # measures. A depth target only means a standoff via the drop, so if the
    # assumed and measured drops disagree the robot stops somewhere else
    # entirely - and nothing else in the loop would ever notice.
    if a.target_depth > 0 and d.get('ok') and d.get('drop'):
        md = float(d['drop'])
        real_x = (tgt_depth - md * math.sin(th)) / math.cos(th)
        print()
        print('cross-check against the drop the camera MEASURES (%.4f m):'
              % md)
        print('  edge_depth %.3f  ->  standoff %.3f m' % (tgt_depth, real_x))
        if abs(real_x - a.target) > 0.02:
            print('  !! that is %+.3f m from the --target %.2f m you asked for.'
                  % (real_x - a.target, a.target))
            print('  !! For %.2f m the target depth would be %.3f m.'
                  % (a.target, md * math.sin(th)
                     + a.target * math.cos(th)))
            print('  !! Using %.3f anyway - pass --target %.2f to silence '
                  'this.' % (tgt_depth, real_x))
        else:
            print('  matches --target %.2f m' % a.target)
    print()

    if a.measure:
        print('--measure: reading only, nothing will move. Ctrl-C to stop.')
        try:
            while True:
                print('  %s  %s' % (time.strftime('%H:%M:%S'),
                                    fmt(plane.read(), a.target, a.pitch)))
                time.sleep(0.5)
        except KeyboardInterrupt:
            print('\nstopped')
        return 0

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node('edge_approach_test')
    pub = node.create_publisher(Twist, a.cmd_topic, 10)

    if not a.dry_run and not a.no_bridge_check:
        for _ in range(40):
            if pub.get_subscription_count() > 0:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() == 0:
            print('!! nothing is subscribed to %s - nav_bridge is not running,'
                  % a.cmd_topic)
            print('!! so the robot would not move. Start it with:')
            print('!!     source ~/.scuba/sdk_teleop/env.sh')
            print('!!     ros2 run g1_teleop nav_bridge --ros-args '
                  '-p enable:=true')
            print('!! (--no-bridge-check to run anyway)')
            node.destroy_node()
            rclpy.shutdown()
            return 1

    state = dict(travel=0.0, seen_move=0.0, stage='init', used={})

    def send(vx, wz):
        if a.dry_run:
            return
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        pub.publish(t)

    def halt():
        for _ in range(10):
            send(0.0, 0.0)
            time.sleep(0.02)

    def min_move():
        """Smallest discrete move possible: ramp to v and back is v^2/accel."""
        return a.v_fwd ** 2 / max(a.accel, 1e-6)

    def drive(vx, wz, why, pred=None, cap=None):
        """Drive CONTINUOUSLY until pred() is true, or cap metres commanded.

        Continuous beats bursting here: the bridge ramps at --accel, so one
        sustained push pays the ramp once instead of every burst.

        ALWAYS returns a 3-tuple (reason, commanded_metres, measured_or_None).
        The early returns used to be 2-tuples, which crashed the caller with
        "not enough values to unpack" the moment the travel budget ran out.
        """
        cap = min(cap if cap is not None else 1e9,
                  max(0.0, a.max_travel - state['travel']))
        if cap <= 0.0:
            print('      drive %-20s REFUSED: travel budget spent '
                  '(%.2f of %.2f m)' % (why, state['travel'], a.max_travel))
            return 'cap', 0.0, None
        print('      drive %-20s vx %+.2f wz %+.2f  cap %.3f m'
              % (why, vx, wz, cap))
        if a.dry_run:
            print('      [dry run] nothing published')
            return 'dry', 0.0, None
        x0 = edge_x_now()
        t0 = time.time()
        last = t0
        moved = 0.0
        reason = 'cap'
        try:
            while True:
                send(vx, wz)
                rclpy.spin_once(node, timeout_sec=0.005)
                time.sleep(0.02)
                now = time.time()
                moved = abs(vx) * (now - t0)
                if moved >= cap:
                    reason = 'cap'
                    break
                if now - last >= a.poll:
                    last = now
                    if pred is not None and pred():
                        reason = 'target'
                        break
        finally:
            halt()
            time.sleep(a.settle)
        state['travel'] += moved
        state['used'][state['stage']] = state['used'].get(
            state['stage'], 0.0) + moved
        x1 = edge_x_now()
        seen = None
        if x0 is not None and x1 is not None:
            seen = abs(x1 - x0)
            state['seen_move'] += seen
            print('      commanded %.3f m   edge_x %.3f -> %.3f (moved %.3f)'
                  % (moved, x0, x1, seen))
        else:
            print('      commanded %.3f m   (edge_x not in frame - movement '
                  'cannot be verified here)' % moved)
        return reason, moved, seen

    def lead_of(v, acc):
        """Distance still covered after the command stops: v^2/(2a).

        Driving in pulses is what made the robot twitch: every pulse pays the
        ramp again and most of it sits under the gait's threshold. Instead we
        drive CONTINUOUSLY and stop this far short, letting the ramp-down
        carry us onto the target.
        """
        return v * v / (2.0 * max(acc, 1e-6))

    def depth_now():
        d = plane.read()
        if d and d.get('ok') and 'edge_depth' in d:
            return float(d['edge_depth'])
        return None

    def angle_now(n=None):
        """Averaged edge_angle.

        One sample is not enough: a standing humanoid keeps re-balancing and
        the angle DRIFTS - a calibration run recorded 3.47 deg of apparent
        noise that was really the robot settling. Measured on the robot,
        averaging 8 samples cuts the spread from 0.27 to 0.10 deg.
        """
        n = a.angle_avg if n is None else n
        got = []
        for _ in range(max(1, n)):
            d = plane.read()
            if d and d.get('ok') and d.get('edge_angle') is not None:
                got.append(float(d['edge_angle']))
            if n > 1:
                time.sleep(0.1)
        return (sum(got) / len(got)) if got else None

    def edge_x_now():
        d = plane.read()
        return d['edge_x'] if (d and d.get('ok') and 'edge_x' in d) else None

    def stall_check(cmd, moved_seen):
        """Commanded a lot and the camera saw nothing move -> say so.

        moved_seen is None when the tabletop was out of frame, which is normal
        during the search. Only a MEASURED near-zero counts as a stall.
        """
        if cmd >= a.stall_after and moved_seen is not None and moved_seen < 0.03:
            print()
            print('  !! STALL: commanded %.2f m but edge_x barely changed '
                  '(%.3f m).' % (cmd, moved_seen))
            print('  !! The robot is receiving the command but not walking.')
            print('  !! At accel %.2f m/s^2 a burst reaches only %.3f m/s and '
                  'the' % (a.accel, a.accel * 0.25))
            print('  !! gait ignores anything under %.2f m/s. Restart the '
                  'bridge with' % a.gait_min)
            print('  !!     -p max_lin_acc:=2.0')
            return True
        return False

    def reading():
        return plane.fresh_ok()

    def turn_step(wz, dur):
        """One fixed-length turn, then stop. No prediction, no servoing."""
        if a.dry_run:
            print('      [dry run] would turn wz %+.2f for %.2f s' % (wz, dur))
            return
        t0 = time.time()
        while time.time() - t0 < dur:
            send(0.0, wz)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.02)
        halt()

    def settled_angle(tol=None, timeout=None):
        """Read the angle only once it has stopped moving.

        Measured on the robot: the in-loop reading and a reading taken right
        after the loop differed by 2.25 deg with nothing commanded in between
        (-0.45 vs -2.70). The robot keeps settling after a step, so a final
        number taken immediately is not the number it ends up at.
        """
        tol = a.settle_tol if tol is None else tol
        timeout = a.settle_timeout if timeout is None else timeout
        t0 = time.time()
        buf = []
        while time.time() - t0 < timeout:
            v = angle_now(1)
            if v is not None:
                buf.append(v)
                if len(buf) > 6:
                    buf.pop(0)
                if len(buf) == 6 and max(buf) - min(buf) <= tol:
                    return sum(buf) / len(buf)
            time.sleep(0.15)
        return (sum(buf[-6:]) / len(buf[-6:])) if buf else None

    def do_align_step(step_dur=None):
        """Step toward zero until the angle changes SIGN.

        Deliberately dumb: fixed-size steps, re-read after each one. The
        robot's turn-per-command is not repeatable, so anything that tries to
        size the correction is guessing; crossing zero only needs the
        direction to be right. The camera lags, so nothing is believed until
        --cam-delay after the robot has stopped.
        """
        ang = angle_now()
        if ang is None:
            print('    no edge to align to')
            return False
        s0 = 1.0 if ang > 0 else -1.0
        sign = -1.0 if a.flip_align else 1.0
        step_dur = a.step_dur if step_dur is None else step_dur
        print('    start %+.2f deg, stepping %.2f s at %.2f rad/s until the '
              'sign flips' % (ang, step_dur, a.w_turn))
        for i in range(a.max_steps):
            if abs(ang) <= a.ang_tol:
                print('    %+.2f deg is inside --ang-tol %.1f - stopping'
                      % (ang, a.ang_tol))
                return True
            wz = sign * M.align_move(ang) * a.w_turn
            print('      step %d: %+.2f deg -> turning wz %+.2f for %.2f s'
                  % (i + 1, ang, wz, step_dur))
            turn_step(wz, step_dur)
            time.sleep(a.cam_delay)          # the camera lags the motion
            new = angle_now()
            if new is None:
                print('    lost the edge while turning')
                return False
            moved = new - ang
            print('      -> %+.2f deg  (moved %+.2f)' % (new, moved))
            s1 = 1.0 if new > 0 else -1.0
            # CROSSING IS CHECKED FIRST. Overshooting through zero also makes
            # |angle| grow, so testing "did it get worse" before "did it
            # cross" mistook a successful crossing for an inverted sign
            # convention - observed on the robot: -2.66 -> +4.23 was flagged
            # INVERTED when it had in fact crossed.
            if s1 != s0:
                print('    crossed zero: %+.2f -> %+.2f  (square to within '
                      'one step)' % (ang, new))
                return True
            if (i == 0 and not a.no_autoflip
                    and abs(new) > abs(ang) + 1.0):
                # same side AND further out = genuinely the wrong way
                sign = -sign
                print('      wrong direction (same side, error grew '
                      '%+.2f -> %+.2f) - flipping the sign convention'
                      % (ang, new))
            elif abs(moved) < 0.2:
                print('      (that step did almost nothing - gait phase; '
                      'continuing)')
            ang = new
        print('    !! %d steps without crossing zero (now %+.2f deg)'
              % (a.max_steps, ang))
        return False

    def do_align():
        if a.align_mode == 'step':
            ok = do_align_step()
            dur = a.step_dur
            for r in range(a.refine):
                if not ok:
                    break
                dur = dur / 2.0
                ang = settled_angle()
                if ang is None or abs(ang) <= a.ang_tol:
                    break
                print('    refine %d/%d: step %.2f s, now %+.2f deg'
                      % (r + 1, a.refine, dur, ang))
                ok = do_align_step(dur)
            return ok
        return do_align_continuous()

    def do_align_continuous():
        """Rotate until the edge runs square across the view.

        Continuous, with the same coast lead as the approach, because a short
        turn has exactly the problem a short forward burst has: at
        max_ang_acc %s the smallest possible turn is w^2/acc.

        The edge_angle SIGN has never been checked against the real robot, so
        the first correction is verified: if the error grows, the convention
        is inverted and the sign is flipped for the rest of the run.
        """
        sign = -1.0 if a.flip_align else 1.0
        # let it stop re-balancing first, or the first reading is a drifting
        # one and the correction chases a number that was never true
        t0 = time.time()
        buf = []
        while time.time() - t0 < a.settle_timeout:
            v = angle_now(1)
            if v is not None:
                buf.append(v)
                if len(buf) > 8:
                    buf.pop(0)
                if len(buf) == 8 and max(buf) - min(buf) <= a.settle_tol:
                    print('  settled (%.2f deg drift over 8 samples)'
                          % (max(buf) - min(buf)))
                    break
            time.sleep(0.15)
        else:
            if buf:
                print('  !! still drifting %.2f deg - aligning anyway'
                      % (max(buf) - min(buf)))
        lead_rad = lead_of(a.w_turn, a.ang_accel)
        min_turn = a.w_turn ** 2 / max(a.ang_accel, 1e-6)
        print('  smallest possible turn %.1f deg  (w^2/ang_accel at %.2f '
              'rad/s^2)' % (math.degrees(min_turn), a.ang_accel))
        if math.degrees(min_turn) > a.ang_tol:
            print('  !! coarser than --ang-tol %.1f deg. Restart the bridge '
                  'with' % a.ang_tol)
            print('  !!     -p max_ang_acc:=2.0   (smallest turn then %.1f '
                  'deg)' % math.degrees(a.w_turn ** 2 / 2.0))
        checked = a.no_autoflip
        for i in range(a.max_iters):
            ang = angle_now()
            if ang is None:
                print('    no edge to align to')
                return False
            print('    %d/%d  edge angle %+.1f deg' % (i + 1, a.max_iters, ang))
            if abs(ang) <= a.ang_tol:
                print('    %+.1f deg is inside the %.1f deg the robot can '
                      'resolve - not turning' % (ang, a.ang_tol))
                return True
            stop_deg = max(a.ang_tol, math.degrees(lead_rad))
            if abs(ang) <= stop_deg:
                print('    remaining %.1f deg is inside the %.1f deg coast - '
                      'as square as this ang_accel allows'
                      % (abs(ang), math.degrees(lead_rad)))
                return True
            w = sign * M.align_move(ang) * a.w_turn
            why, cmd, seen = drive(
                0.0, w, 'align %+.1f deg' % ang,
                pred=lambda: abs(angle_now(1) or 0.0) <= stop_deg,
                cap=1e9)
            after = angle_now()
            if after is None:
                print('    lost the edge while turning')
                return False
            if not checked:
                checked = True
                if abs(after) > abs(ang) + 1.0:
                    sign = -sign
                    print('    edge_angle sign was INVERTED (error grew '
                          '%+.1f -> %+.1f) - flipped for the rest of the run'
                          % (ang, after))
                else:
                    print('    sign convention confirmed (%+.1f -> %+.1f)'
                          % (ang, after))
        print('    !! did not square up in %d iterations' % a.max_iters)
        return False

    rc = 1
    try:
        if a.align_only:
            print('ALIGN-ONLY: %d cycles, measuring where the closed loop '
                  'lands' % a.align_only)
            print('(open-loop turn size varies with gait phase; what matters '
                  'here is the residual)')
            print()
            res = []
            for k in range(a.align_only):
                start = angle_now()
                if start is None:
                    print('  no edge - aborting')
                    break
                print('  --- cycle %d/%d, starting at %+.2f deg'
                      % (k + 1, a.align_only, start))
                do_align()
                end = settled_angle()
                if end is None:
                    print('  lost the edge')
                    break
                res.append(end)
                print('  --- landed at %+.2f deg (from %+.2f)' % (end, start))
            if res:
                worst = max(abs(x) for x in res)
                mean = sum(abs(x) for x in res) / len(res)
                print()
                print('=' * 60)
                print('CLOSED-LOOP ALIGN RESIDUAL over %d cycles' % len(res))
                print('  landed at : %s'
                      % ', '.join('%+.2f' % x for x in res))
                print('  mean |err|: %.2f deg' % mean)
                print('  worst     : %.2f deg' % worst)
                print()
                print('  -> --ang-tol %.1f is the honest deadzone: the loop'
                      % (math.ceil(worst * 2) / 2.0))
                print('     cannot reliably land inside its own residual.')
                print('=' * 60)
            rc = 0
            raise KeyboardInterrupt

        # ---- stage 1: creep forward until the tabletop is in frame at all
        state['stage'] = 'search'
        print('STAGE 1  search forward for the tabletop')
        if reading() is not None:
            print('  already visible, no search needed')
        else:
            print('    %s' % fmt(plane.read(), a.target, a.pitch))
            why, cmd, seen = drive(a.v_fwd, 0.0, 'search',
                                   pred=lambda: reading() is not None)
            # no stall check here: the tabletop is out of frame during the
            # search by definition, so there is nothing to measure against
            if reading() is None:
                print('  !! no tabletop after %.2f m commanded - stopping '
                      'rather than driving on' % state['travel'])
                return 1
        print('  acquired (%.2f m commanded so far)' % state['travel'])
        print()

        # ---- stage 2: square up on the edge BEFORE closing in
        if not a.no_align:
            print('STAGE 2  align square to the edge')
            if not do_align():
                print('  !! align did not converge - continuing anyway, but '
                      'the approach will be measured off-square')
            print()

        # ---- stage 3: servo to the target standoff
        state['stage'] = 'approach'
        print('STAGE 3  approach to %.2f m' % a.target)
        for i in range(a.max_iters):
            d = reading()
            if d is None:
                print('    lost the plane, re-acquiring')
                drive(a.v_fwd, 0.0, 'reacquire',
                      pred=lambda: reading() is not None, cap=0.20)
                continue
            print('    %d/%d  %s' % (i + 1, a.max_iters,
                                     fmt(d, a.target, a.pitch)))
            if a.servo == 'edge_x':
                ground = d['edge_x'] - a.target
            else:
                ground = M.approach_move(d['edge_depth'], tgt_depth, a.pitch)
            if abs(ground) <= a.tol:
                print('    within %.0f mm - done' % (1000 * a.tol))
                rc = 0
                break
            v = a.v_fwd if ground > 0 else -a.v_fwd
            lead = lead_of(a.v_fwd, a.accel)
            if abs(ground) <= lead + a.tol:
                # Coasting alone would overshoot. Nothing useful left to
                # command - saying so beats twitching at it forever.
                print('    remaining %.3f m is inside the %.3f m coast '
                      'distance' % (abs(ground), lead))
                print('    -> as close as this accel allows. Raise '
                      'max_lin_acc to do better.')
                rc = 0
                break
            # stop SHORT by the coast distance and let the ramp-down finish it
            if a.servo == 'edge_x':
                stop_at = a.target + (lead if v > 0 else -lead)
                if v > 0:
                    pred = lambda: (edge_x_now() or 1e9) <= stop_at
                else:
                    pred = lambda: (edge_x_now() or -1e9) >= stop_at
            else:
                dlead = lead * math.cos(th)
                stop_d = tgt_depth + (dlead if v > 0 else -dlead)
                if v > 0:
                    pred = lambda: (depth_now() or 1e9) <= stop_d
                else:
                    pred = lambda: (depth_now() or -1e9) >= stop_d
            why, cmd, seen = drive(v, 0.0, 'approach %+.3f m' % ground,
                                   pred=pred, cap=abs(ground) + 0.15)
            if why == 'cap' and cmd <= 0.0:
                print('    !! out of travel budget (%.2f of %.2f m used).'
                      % (state['travel'], a.max_travel))
                print('    !! Still %+.3f m from target - raise --max-travel '
                      'or start closer.' % ground)
                break
            if stall_check(cmd, seen):
                break
            if a.dry_run:
                print('    [dry run] that is the first move it would make; '
                      'stopping here')
                rc = 0
                break
        else:
            print('    !! did not converge in %d iterations' % a.max_iters)

        # ---- optional: square up (OFF by default)
        if a.align_after:
            print()
            print('STAGE 4  re-align after the approach')
            do_align()

        # ---- report
        print()
        print('=' * 68)
        d = plane.read()
        print('RESULT   commanded %.3f m of %.2f m budget   '
              '(camera saw %.3f m of change)'
              % (state['travel'], a.max_travel, state['seen_move']))
        if state['used']:
            print('  spent: %s'
                  % ', '.join('%s %.2f m' % (k, v)
                              for k, v in sorted(state['used'].items())))
        if d and d.get('ok'):
            print('  camera to table edge   %.3f m   (asked for %.2f)'
                  % (d['edge_x'], a.target))
            print('  edge_depth reported    %.3f m   (target %.3f)'
                  % (d['edge_depth'], tgt_depth))
            ang = d.get('edge_angle')
            if ang is not None:
                print('  edge angle             %+.1f deg   %s' %
                      (ang, 'square' if abs(ang) <= a.ang_tol
                       else 'NOT square - how did you place it?'))
            print('  plane rms / cover      %.0f mm / %.3f'
                  % (1000 * d['rms'], d.get('cover', float('nan'))))
            print()
            print('  Now measure body-front to table edge with a tape. It '
                  'should read')
            print('  about %.2f m. A consistent offset means the camera sits '
                  'that far' % a.target)
            print('  behind the body front, and --target should absorb it.')
        else:
            print('  no valid reading at the end: %s'
                  % (d.get('why') if d else 'detector silent'))
        print('=' * 68)
    except KeyboardInterrupt:
        print('\ninterrupted')
    finally:
        halt()
        print('zero velocity sent')
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
