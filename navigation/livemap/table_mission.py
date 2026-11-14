#!/usr/bin/env python3
"""Visit every table: find, approach, square up, circle all four sides, repeat.

    source ~/.scuba/lio_ws/env.sh          # FOXY ONLY - never source noetic here
    /usr/bin/python3 ~/.scuba/livemap/table_mission.py --dry-run   # no motion
    /usr/bin/python3 ~/.scuba/livemap/table_mission.py             # drives

WHAT IT TALKS TO
  /table_registry   (sub)  JSON from table_registry.py - the candidate list
  /table_event      (pub)  JSON - reports VISITED so a table is never redone
  /cmd_vel          (pub)  the fine approach; nav_bridge forwards it
  Nav2 actions             compute_path_to_pose + follow_path, for the long haul
  :8093/stat        (HTTP) the plane detector, which lives in ROS 1 with the
                           RealSense driver. There is no ros1_bridge installed,
                           so HTTP is the join between the two graphs.

WHY BURSTS AND NOT A SERVO LOOP
Measured on this gait: forward commands below ~0.20 m/s are IGNORED (0.10 and
0.15 produced no movement at all), and turns overshoot what is commanded. So
the robot cannot creep the last few centimetres. Every fine motion is a short
burst at a speed the gait honours, then a full stop, then a fresh measurement.
Expect 0.30 +- 0.03-0.05 m, not an exact 0.30.

WHY IT BACKS OFF BEFORE MOVING ON
The global costmap treats everything within 0.50 m of an obstacle as
untraversable (robot_radius 0.50 there, deliberately). At the 0.30 m standoff
the robot is INSIDE that band, so A* cannot plan a path out. It must first
reverse clear under velocity control, then hand back to Nav2.
"""
import argparse
import json
import math
import re
import os
import sys
import subprocess
import threading
import time
import urllib.request


def _load_plane_proc():
    """Import camview/plane_proc.py by path.

    Not via sys.path: ~/.scuba/camview is a ROS 1 tree and putting it on the
    path of a ROS 2 process is how the two distros get mixed. plane_proc
    itself imports nothing from ROS, so loading the single file is safe.
    """
    import importlib.util
    f = os.path.expanduser('~/.scuba/camview/plane_proc.py')
    if not os.path.exists(f):
        return None
    try:
        s = importlib.util.spec_from_file_location('plane_proc', f)
        m = importlib.util.module_from_spec(s)
        s.loader.exec_module(m)
        return m
    except Exception as e:
        print('!! could not load plane_proc.py: %s' % e)
        return None


PLANE_PROC = _load_plane_proc()


def plane_answers(url):
    """True when the detector's HTTP endpoint responds at all.

    The readiness gate. A fixed warmup timer is not enough: if the forward
    search starts before the detector serves, search() reads "no plane" for
    its first seconds and can conclude the table is not there.
    """
    try:
        with urllib.request.urlopen(url, timeout=0.5) as r:
            r.read()
        return True
    except Exception:
        return False


class _Borrowed:
    """Wraps an ALREADY-OPEN stack for one edge, without owning it.

    --camera-scope mission opens the stack once; each edge still goes through
    `with`, so the per-edge window must not stop what it did not start. Same
    rule as RealSense's adopt path.
    """

    def __init__(self, stack):
        self.stack = stack
        self.ok = bool(stack is not None and getattr(stack, 'ok', False))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


class _NullPlane:
    """Stand-in when the detector is managed by hand (--plane-manage off)."""

    ok = True

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


def require_ros2():
    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit(
            '!! ROS_DISTRO is %r, not foxy. This is a ROS 2 node; a shell with\n'
            '!! ROS 1 on its path will fail in xmlrpc. Open a NEW terminal:\n'
            '!!     source ~/.scuba/lio_ws/env.sh' % d)


def norm_id(s):
    """'table_01', 'Table_1', 't1', '1' all mean the registry's 'table_1'.

    The registry numbers entries table_1, table_2, ... with no zero padding,
    but an operator naturally types table_01, so accept both.
    """
    s = str(s).strip().lower().replace('-', '_').replace(' ', '')
    m = re.match(r'^(?:table_?|t)?0*(\d+)$', s)
    return 'table_%d' % int(m.group(1)) if m else s


def approach_move(edge_depth, target, pitch_deg):
    """(ground metres to move, +forward/-back). Pure, so the SIGN is testable.

    edge_depth is what the camera reports for the table edge; target is what it
    should read at the wanted standoff. Reading LONGER than target means the
    table is too far, so move forward. The depth error converts to ground
    distance by /cos(pitch) because the camera looks down at the surface.
    """
    err = edge_depth - target
    return err / math.cos(math.radians(pitch_deg))


def align_move(angle_deg):
    """Yaw rate SIGN to square up on the edge.

    edge_angle > 0 means the edge's right-hand end is further away. Yawing the
    robot LEFT by psi reduces the measured angle by psi (the right axis rotates
    counterclockwise with the robot), so a positive angle needs a positive -
    left, counterclockwise - yaw.
    """
    return 1.0 if angle_deg > 0 else -1.0


SEND_GOAL = os.path.expanduser('~/.scuba/nav/send_goal.py')


def side_poses(cx, cy, yaw, side_len, robot_radius, dists):
    """Standoff poses on all four sides of the known square, facing inward.

    dists maps a tag to a distance from the table EDGE, so the pose sits at
    side_len/2 + dist + robot_radius from the centre.
    """
    u = (math.cos(yaw), math.sin(yaw))
    v = (-u[1], u[0])
    out = {}
    for name, n in (('A', u), ('B', v), ('C', (-u[0], -u[1])),
                    ('D', (-v[0], -v[1]))):
        for tag, dist in dists.items():
            r = side_len / 2.0 + dist + robot_radius
            out[(name, tag)] = (cx + n[0] * r, cy + n[1] * r,
                                math.atan2(-n[1], -n[0]))
    return out


class Plane:
    """The ROS 1 plane detector, reached over HTTP."""

    def __init__(self, url, max_age):
        self.url = url
        self.max_age = max_age
        self.last = None
        self.last_t = 0.0

    def read(self):
        try:
            with urllib.request.urlopen(self.url, timeout=0.5) as r:
                self.last = json.loads(r.read().decode())
                self.last_t = time.time()
        except Exception:
            pass
        if time.time() - self.last_t > self.max_age:
            return None
        return self.last

    def fresh_ok(self):
        d = self.read()
        return d if (d and d.get('ok') and d.get('trigger')) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dry-run', action='store_true',
                    help='run the whole state machine but publish NO /cmd_vel '
                         'and send no Nav2 goal')
    ap.add_argument('--plane-url', default='http://localhost:8093/stat')
    ap.add_argument('--plane-max-age', type=float, default=2.0)
    ap.add_argument('--only', default='', help='visit just this table id')
    ap.add_argument('--sides', default='A,B,C,D')
    # Default is OPERATOR-DRIVEN: the robot starts still and does nothing
    # until a table id arrives on --cmd-topic. --auto restores the old
    # behaviour of sweeping every open candidate, nearest first.
    ap.add_argument('--cmd-topic', default='/table_command')
    ap.add_argument('--auto', action='store_true',
                    help='visit every open table without being asked')
    ap.add_argument('--max-tables', type=int, default=0, help='0 = no limit')
    # geometry
    ap.add_argument('--side-len', type=float, default=1.20)
    ap.add_argument('--standoff', type=float, default=0.30)
    # --- lateral correction (side step), measured 2026-10-29 -------------
    ap.add_argument('--side-step', choices=['on', 'off'], default='on',
                    help='lateral correction to the edge midpoint after the '
                         'nav stage, before the camera approach')
    ap.add_argument('--side-vy', type=float, default=0.20,
                    help='m/s. 0.20 is the lowest that moves; +vy is LEFT')
    ap.add_argument('--side-dur-coarse', type=float, default=0.60,
                    help='s. MEASURED 0.074 m, yaw 1.60 deg avg (3 of 5 '
                         'pulses over 1.0 deg), 21.6 deg/m. Costs ~44%% more '
                         'heading per metre than 0.40 s (15.0 deg/m) but '
                         '0.074 m is ~= --side-tol 0.08, so it usually needs '
                         'ONE pulse instead of two, saving a 6 s settle and '
                         'a heading re-check.')
    ap.add_argument('--side-dur-fine', type=float, default=0.30,
                    help='s. 0.028 m. UNREACHABLE while --side-tol (0.08) >= '
                         '--side-coarse-m (0.045): every error big enough to '
                         'step is also big enough to be coarse. Raise '
                         '--side-coarse-m above --side-tol to use it.')
    ap.add_argument('--side-coarse-m', type=float, default=0.045,
                    help='use the coarse pulse while the error exceeds this')
    ap.add_argument('--side-min-cos', type=float, default=0.70,
                    help='refuse to side-step unless |cos(heading - edge '
                         'normal)| is at least this (0.70 = within ~45 deg, '
                         'facing in OR out). Below it the robot\'s left axis '
                         'points along the edge normal and a vy pulse drives '
                         'it at the table instead of sideways.')
    ap.add_argument('--side-stop', choices=['cross', 'tol'], default='cross',
                    help="cross = keep pulsing until the robot CROSSES the "
                         "edge centre (sign flip), the user's spec. tol = the "
                         "older behaviour, stop once inside --side-tol. "
                         "cross costs more heading (every pulse is ~1.6 deg "
                         "and the drift is an 87%% clockwise BIAS, so "
                         "reversals accumulate rather than cancel) but it "
                         "does not leave the robot parked short of centre on "
                         "a tolerance that is wider than the gait step.")
    ap.add_argument('--side-tol', type=float, default=0.08,
                    help='m. Below one coarse step - going finer than this '
                         'chases a centre estimate only good to ~0.25 m')
    ap.add_argument('--side-max-pulses', type=int, default=8,
                    help='hard cap. Raised from 3: stopping on a CROSSING '
                         'needs enough pulses to reach the centre, and 0.30 m '
                         'at 0.074 m per pulse is ~5. Each pulse costs ~1.6 '
                         'deg of heading, so 8 pulses can cost ~13 deg - '
                         'heading_recheck afterwards is not optional.')
    ap.add_argument('--side-max-err', type=float, default=0.60,
                    help='m. A larger offset means a bad centre estimate, not '
                         'drift, so refuse to step rather than drive into it')
    ap.add_argument('--side-settle', type=float, default=6.0,
                    help='s after each pulse. MEASURED: lateral motion was '
                         'still finishing 4-9 s after a 0.30 s command')
    ap.add_argument('--recheck-yaw', choices=['on', 'off'], default='on',
                    help='re-square the heading after the side step; it costs '
                         '~0.84 deg per pulse with an 87%% clockwise bias')
    ap.add_argument('--trim', choices=['on', 'off'], default='on',
                    help="on = after the plane align, drive to the MEASURED "
                         "--standoff. off = stop wherever the tabletop first "
                         "entered the detector's band, which is the shorter "
                         "sequence but leaves the final distance uncontrolled.")
    ap.add_argument('--stage', type=float, default=0.50,
                    help='m from the edge for the Nav2 goal; must exceed the '
                         'global costmap block (0.50) plus its goal tolerance')
    ap.add_argument('--robot-radius', type=float, default=0.28)
    ap.add_argument('--cam-h', type=float, default=1.262)
    ap.add_argument('--table-h', type=float, default=0.76)
    ap.add_argument('--pitch', type=float, default=63.6)
    # motion, all constrained by what the gait actually honours
    ap.add_argument('--v-fwd', type=float, default=0.22)
    ap.add_argument('--w-turn', type=float, default=0.40)
    ap.add_argument('--burst-max', type=float, default=0.8, help='s')
    ap.add_argument('--settle', type=float, default=1.2, help='s after a burst')
    ap.add_argument('--dist-tol', type=float, default=0.03, help='m')
    ap.add_argument('--ang-tol', type=float, default=2.0,
                    help='deg. Was 1.0, which is UNREACHABLE: --step-dur 0.4 '
                         'at --w-turn 0.40 moves ~4.08 deg (max 5.98), so the '
                         'step was twice the 2.0-deg-wide window and could '
                         'only overshoot. 2.0 gives a 4.0-deg window that a '
                         '2.5-3.5 deg step CAN land inside - the same '
                         'argument that makes send_goal --yaw-tol 2.0 work.')
    ap.add_argument('--max-iters', type=int, default=8)
    # The Nav2 goal leaves the robot ~--stage from the table edge, but the
    # camera only sees a tabletop within ~0.45 m horizontally, so from there
    # the table is not in frame at all. Creep in until it appears.
    ap.add_argument('--search-burst', type=float, default=0.25,
                    help='s per forward burst while looking for the tabletop; '
                         '0.25 s at 0.22 m/s is about 5.5 cm a step')
    ap.add_argument('--search-max', type=float, default=1.10,
                    help='m; give up (and do NOT drive further) after this')
    ap.add_argument('--max-attempts', type=int, default=2,
                    help='tries per table before it is left alone')
    ap.add_argument('--plane-manage', choices=('on', 'off'), default='on',
                    help='on = the mission STARTS plane_detect for the camera '
                         'phase and stops it afterwards, so it is not running '
                         'during edge-to-edge navigation. It was measured at '
                         '251%% CPU (7 BLAS worker threads), which starved '
                         'FAST-LIO until map->body was 1.86 s stale. off = you '
                         'run it yourself in a camview terminal, as before.')
    ap.add_argument('--camera-manage', choices=('on', 'off'), default='on',
                    help='on = the mission also starts and stops the ROS 1 '
                         'RealSense launch. MEASURED 2026-10-30: the nodelet '
                         'costs 65-97%% CPU and is only needed for the camera '
                         'phase. It will NEVER stop a camera it did not start: '
                         'if yours is already up it is adopted and left alone.')
    ap.add_argument('--camera-scope', choices=('edge', 'mission'),
                    default='edge',
                    help='edge = up and down around each table edge, which '
                         'frees the CPU during all edge-to-edge navigation. '
                         'mission = up once for the whole run. Use mission if '
                         'the RealSense misbehaves over USB when cycled - '
                         'enumeration is the flaky part, not the launch.')
    ap.add_argument('--depth-topic',
                    default='/camera/aligned_depth_to_color/image_raw',
                    help='the ROS 1 topic whose presence means the camera is '
                         'ready. Checked against the ROS 1 master over plain '
                         'XML-RPC, so it needs no ROS 1 libraries here.')
    ap.add_argument('--ros1-master', default='http://localhost:11311')
    ap.add_argument('--camera-ready-timeout', type=float, default=40.0)
    ap.add_argument('--camera-tries', type=int, default=2,
                    help='RealSense USB enumeration can fail once and then '
                         'succeed; this retries rather than failing the edge')
    ap.add_argument('--plane-warmup', type=float, default=4.0,
                    help='s to wait after launching before polling --plane-url')
    ap.add_argument('--plane-ready-timeout', type=float, default=30.0,
                    help='s to wait for the detector to answer before giving '
                         'up on that edge')
    ap.add_argument('--ignore-no-camera', action='store_true',
                    help='start even if the plane detector is unreachable')
    # The per-side motion now comes from edge_servo.py, the code validated
    # on the robot by edge_approach_test.py. These MUST match how nav_bridge
    # was launched: a 0.25 s burst at the default 0.35 m/s^2 only reaches
    # 0.0875 m/s and the gait ignores it, so the robot twitches and never
    # walks. Launch it with -p max_lin_acc:=2.0 -p max_ang_acc:=2.0
    # The lidar registry's candidates are good but not certain, so a human
    # looks at each one before the robot commits to walking around it.
    ap.add_argument('--confirm', choices=['on', 'off'], default='on',
                    help='ask before visiting each table')
    ap.add_argument('--confirm-timeout', type=float, default=0.0,
                    help='s to wait for an answer; 0 = wait forever')
    ap.add_argument('--accel', type=float, default=2.0)
    ap.add_argument('--ang-accel', type=float, default=2.0)
    ap.add_argument('--step-dur', type=float, default=0.4)
    ap.add_argument('--cam-delay', type=float, default=0.5)
    ap.add_argument('--refine', type=int, default=0)
    ap.add_argument('--side-travel', type=float, default=1.20,
                    help='m of forward motion allowed per side')
    ap.add_argument('--nav-timeout', type=float, default=120.0)
    a = ap.parse_args()
    a.target = ((a.cam_h - a.table_h) * math.sin(math.radians(a.pitch))
                + a.standoff * math.cos(math.radians(a.pitch)))
    sides = [s.strip() for s in a.sides.split(',') if s.strip()]

    require_ros2()
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from geometry_msgs.msg import Twist, PoseStamped
    from std_msgs.msg import String
    from nav2_msgs.action import ComputePathToPose, FollowPath
    import tf2_ros
    import edge_servo

    class Mission(Node):
        def __init__(self):
            super().__init__('table_mission')
            self.reg = None
            self.plane = Plane(a.plane_url, a.plane_max_age)
            self.sp = edge_servo.Params(
                v_fwd=a.v_fwd, w_turn=a.w_turn, accel=a.accel,
                ang_accel=a.ang_accel, tol=a.dist_tol, ang_tol=a.ang_tol,
                step_dur=a.step_dur, cam_delay=a.cam_delay,
                refine=a.refine, max_travel=a.side_travel,
                settle=a.settle, max_steps=a.max_iters)
            self.servo = edge_servo.EdgeServo(
                send=lambda vx, wz: self.send(vx, wz),
                spin=lambda dt: rclpy.spin_once(self, timeout_sec=dt),
                plane=self.plane,
                log=lambda m: self.get_logger().info(m),
                p=self.sp,
                abort=lambda: self.stop_flag or self.abort,
                dry_run=bool(a.dry_run))
            self.buf = tf2_ros.Buffer()
            self.lis = tf2_ros.TransformListener(self.buf, self)
            self.create_subscription(String, '/table_registry', self.on_reg, 10)
            self.create_subscription(String, a.cmd_topic, self.on_cmd, 10)
            self.want = None          # table id asked for, consumed by run()
            self.abort = False
            self.ev = self.create_publisher(String, '/table_event', 10)
            # what the executive is doing right now, for the viewer
            self.status = self.create_publisher(String, '/mission_status', 10)
            self.st = dict(phase='idle', table=None, side=None, target=None,
                           note='', dry=bool(a.dry_run))
            self.cam = None          # mission-scope camera stack, if any
            self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
            self.plan = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
            self.follow = ActionClient(self, FollowPath, 'follow_path')
            self.stop_flag = False
            # /table_registry only refreshes every --period, so without this the
            # executive re-selects the SAME table several times inside one
            # window - seen live: table_2 reported twice, 6 s apart
            self.handled = {}
            self.answer = None    # 'yes'/'no' from stdin or the topic

        # ---- plumbing
        def on_cmd(self, m):
            txt = (m.data or '').strip()
            if not txt:
                return
            low = txt.lower()
            if low in ('stop', 'abort', 'halt', 'cancel'):
                self.abort = True
                self.want = None
                self.get_logger().warn('command: %s - stopping motion' % low)
                return
            if low in ('y', 'yes', 'ok', 'confirm', 'go'):
                self.answer = 'yes'
                self.get_logger().info('command: confirmed')
                return
            if low in ('n', 'no', 'skip'):
                self.answer = 'no'
                self.get_logger().info('command: rejected')
                return
            if low in ('list', 'ls', '?'):
                self.get_logger().info('known tables: %s'
                                       % (', '.join(self.known_ids()) or 'none'))
                return
            self.want = norm_id(txt)
            self.get_logger().info('command: visit %s (from %r)'
                                   % (self.want, txt))

        def start_stdin(self):
            """Called from run(), once the node is fully constructed."""
            if a.confirm == 'on':
                threading.Thread(target=self._stdin_loop,
                                 daemon=True).start()

        def _stdin_loop(self):
            """Read answers from the terminal WITHOUT blocking the executor.

            A plain input() would stop rclpy spinning, so the node would go
            deaf to /table_registry and /table_command while it waited.
            """
            while not self.stop_flag:
                try:
                    line = sys.stdin.readline()
                except Exception:
                    return
                if not line:
                    return
                t = line.strip().lower()
                if t in ('y', 'yes', 'ok', 'confirm', 'go'):
                    self.answer = 'yes'
                elif t in ('n', 'no', 'skip'):
                    self.answer = 'no'
                elif t in ('stop', 'abort', 'cancel', 'halt'):
                    self.abort = True
                    self.answer = 'no'
                elif t:
                    print('  (type y / n, or stop)')
                    sys.stdout.flush()

        def ask_confirm(self, en):
            """Show what was found and wait for y / n before committing."""
            print()
            print('=' * 64)
            print('  CONFIRM: is %s really a table?' % en['id'])
            print('    class      %s' % en['cls'])
            print('    size       %.2f x %.2f m   (bare table is %.2f)'
                  % (en['L'], en['W'], a.side_len))
            print('    fill       %.0f%%   cells %d   seen %d times'
                  % (100 * en.get('fill', 0.0), en['n'], en.get('n_obs', 0)))
            print('    centre     (%.2f, %.2f)   %.1f m away'
                  % (en['cx'], en['cy'], en.get('range', -1)))
            print('    state      %s' % en['state'])
            if en['L'] > a.side_len + 0.40:
                print('    NOTE: %.2f m across - that is a table PLUS its '
                      'chairs;' % en['L'])
                print('          the %.2f m standoff pose lands inside the '
                      'chair ring.' % a.standoff)
            print()
            print('    y = visit it     n = not a table, skip it')
            print('=' * 64)
            sys.stdout.flush()
            self.get_logger().info(
                'CONFIRM %s: %s %.2fx%.2f m at (%.2f, %.2f) - answer y or n'
                % (en['id'], en['cls'], en['L'], en['W'], en['cx'], en['cy']))
            self.say(phase='confirm', table=en['id'], side=None,
                     note='waiting for confirmation of %s' % en['id'])
            self.answer = None
            t0 = time.time()
            while self.answer is None and not self.stop_flag:
                self.spin(0.2)
                if a.confirm_timeout and time.time() - t0 > a.confirm_timeout:
                    self.get_logger().warn('no answer in %.0fs - skipping %s'
                                           % (a.confirm_timeout, en['id']))
                    return False
            ans, self.answer = self.answer, None
            if ans == 'yes':
                self.get_logger().info('operator confirmed %s' % en['id'])
                return True
            self.get_logger().info('operator says %s is NOT a table'
                                   % en['id'])
            # The operator looked and said no - that is what REJECTED means,
            # and it stops the entry being offered again this run.
            self.report(en['id'], state='REJECTED')
            self.say(phase='idle', table=None, side=None,
                     note='%s rejected by operator' % en['id'])
            return False

        def on_reg(self, m):
            try:
                self.reg = json.loads(m.data)
            except Exception:
                pass

        def say(self, **kw):
            self.st.update(kw)
            self.st['t'] = time.time()
            try:
                self.status.publish(String(data=json.dumps(self.st, default=float)))
            except Exception:
                pass

        def spin(self, sec):
            t0 = time.time()
            while time.time() - t0 < sec and not self.stop_flag:
                rclpy.spin_once(self, timeout_sec=0.05)

        def send(self, vx, wz, vy=0.0):
            if a.dry_run:
                return
            t = Twist()
            t.linear.x = float(vx)
            t.linear.y = float(vy)
            t.angular.z = float(wz)
            self.cmd.publish(t)

        def halt(self):
            for _ in range(10):
                self.send(0.0, 0.0)
                time.sleep(0.02)

        def burst(self, vx, wz, dur, why, vy=0.0, settle=None):
            """Short commanded motion, then a full stop and a settle."""
            dur = max(0.0, min(dur, max(a.burst_max, dur)))
            self.get_logger().info(
                '    burst %s  vx %+.2f vy %+.2f wz %+.2f for %.2f s'
                % (why, vx, vy, wz, dur))
            if a.dry_run:
                self.spin(0.1)
                return
            t0 = time.time()
            while time.time() - t0 < dur and not self.stop_flag:
                self.send(vx, wz, vy)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.03)
            self.halt()
            self.spin(a.settle if settle is None else settle)

        def robot_heading_deg(self):
            """Map-frame heading in DEGREES, in the frame side_poses uses."""
            for base in ('body', 'base_link'):
                try:
                    tr = self.buf.lookup_transform('map', base,
                                                   rclpy.time.Time())
                except Exception:
                    continue
                r = tr.transform.rotation
                return math.degrees(math.atan2(
                    2.0 * (r.w * r.z + r.x * r.y),
                    1.0 - 2.0 * (r.y * r.y + r.z * r.z)))
            return None

        def edge_cos(self, en, sd):
            """cos of the angle between the robot's heading and the inward
            normal of this edge. +1 facing the table, -1 facing away."""
            P = self.poses_for(en)
            key = (sd, 'stage')
            if key not in P:
                return None
            h = self.robot_heading_deg()
            if h is None:
                return None
            return math.cos(math.radians(h) - P[key][2])

        def lateral_error(self, en, sd):
            """Offset from the edge midpoint along the ROBOT'S OWN left axis.

            THE AXIS MATTERS, and getting it wrong was a real bug.

            This used to project onto the STAGE POSE's left axis,
            (-sin(pyaw), cos(pyaw)). But `vy` moves the robot along the
            ROBOT's left axis. Those agree only while the robot's heading
            matches pyaw, and nothing enforced that.

            MEASURED 2026-10-30: the robot sat at side C's stage pose while
            its heading matched side A's normal - facing OUTWARD, with
            cos(h - pyaw) = -0.94. The two left axes were antiparallel, so a
            correctly-aimed "step right" INCREASED the reported offset:
            observed +0.309 -> +0.385 m, one pulse's worth the wrong way.

            Projecting onto the robot's own left axis makes the sign right
            whichever way the robot faces, because it is the axis vy actually
            moves along. It is still the same line through the edge midpoint,
            so reducing it still centres the robot.

            Positive = the midpoint is to the robot's RIGHT... no: positive
            means the robot is displaced to its own LEFT of the stage pose, so
            it must step RIGHT. `side_step` does that with
            vy = -copysign(side_vy, e), and +vy is LEFT.

            CALLER MUST CHECK edge_cos() FIRST. If the robot is not roughly
            square to the edge (|cos| small), its left axis points along the
            edge NORMAL instead of along the edge, and a vy pulse would drive
            it toward or away from the table rather than sideways.

            MEASURED CAVEAT: the registry's table centre wanders by ~0.25 m
            and this error derives from it, so an error smaller than that is
            not distinguishable from a bad centre.
            """
            P = self.poses_for(en)
            key = (sd, 'stage')
            if key not in P:
                return None
            px, py, _pyaw = P[key]
            xy = self.robot_xy()
            if xy is None:
                return None
            h = self.robot_heading_deg()
            if h is None:
                return None
            hr = math.radians(h)
            ex, ey = -math.sin(hr), math.cos(hr)    # the ROBOT's left
            return (xy[0] - px) * ex + (xy[1] - py) * ey

        def side_step(self, en, sd):
            """Lateral correction to the middle of the edge, by vy pulses.

            NOT a sign-flip search. MEASURED 2026-10-29: 26 of 30 pulses
            drifted CLOCKWISE regardless of step direction - a bias, not a
            direction-coupled effect - so overshooting and coming back
            ACCUMULATES heading error instead of cancelling it. Every reversal
            costs again. So this steps toward the target and stops; it never
            deliberately crosses it.

            Pulse sizes from the same measurement, vy 0.20:
                0.40 s -> 0.056 m, yaw 0.84 deg   (the workhorse; tightest
                                                   yaw spread, 15 deg/m)
                0.30 s -> 0.028 m, yaw 0.73 deg   (final trim only)
                0.20 s -> DEAD, the gait does not commit to a step
                0.80 s -> strictly dominated: 0.085 m for 2.78 deg
            Yaw cost per metre IMPROVES with longer pulses (a fixed ~0.8 deg
            goes into initiating any step), so take the FEWEST pulses that do
            not overshoot, and never fine-step a large error.
            """
            if a.side_step != 'on':
                # was a SILENT return: the one path that left no trace in the
                # log, so "the lateral phase never ran" was indistinguishable
                # from "it ran and did nothing".
                self.get_logger().info(
                    '    side step: DISABLED (--side-step off)')
                return True
            self.get_logger().info('    --- lateral correction (side %s) ---'
                                   % sd)
            c = self.edge_cos(en, sd)
            if c is None:
                self.get_logger().warn('    side step: no heading - skipping')
                return True
            if abs(c) < a.side_min_cos:
                self.get_logger().error(
                    '    side step: the robot is %.0f deg off this edge\'s '
                    'normal (cos %+.2f, need |cos| >= %.2f). Its left axis '
                    'points along the edge NORMAL, so a vy pulse would drive '
                    'it at the table, not sideways. NOT stepping - re-square '
                    'the heading first.'
                    % (math.degrees(math.acos(max(-1.0, min(1.0, abs(c))))),
                       c, a.side_min_cos))
                return True
            if c < 0:
                self.get_logger().warn(
                    '    side step: facing AWAY from the table (cos %+.2f). '
                    'The lateral correction still works - the error is '
                    'measured on the robot\'s own left axis - but something '
                    'upstream aimed it backwards, and the camera will see '
                    'nothing from here.' % c)
            e = self.lateral_error(en, sd)
            if e is None:
                self.get_logger().warn('    side step: no lateral error '
                                       '(registry or TF missing) - skipping')
                return True
            self.get_logger().info(
                '    side step: offset %+.3f m along the edge (tol %.3f, '
                'centre estimate is only good to ~0.25 m)'
                % (e, a.side_tol))
            if abs(e) > a.side_max_err:
                self.get_logger().warn(
                    '    offset %+.3f m exceeds --side-max-err %.2f - that is '
                    'a bad centre estimate, not drift. NOT stepping.'
                    % (e, a.side_max_err))
                return True
            s0 = 1.0 if e > 0 else -1.0     # the side we started on
            prev = e
            for i in range(a.side_max_pulses):
                if self.stop_flag or self.abort:
                    return False
                e = self.lateral_error(en, sd)
                if e is None:
                    return True

                # ---- stop condition
                if a.side_stop == 'cross':
                    if (1.0 if e > 0 else -1.0) != s0:
                        # Confirm before believing it: this error is derived
                        # from the registry centre, which wanders ~0.25 m, so
                        # a single reading can flip on noise alone.
                        self.spin(1.0)
                        c = self.lateral_error(en, sd)
                        if c is not None and (1.0 if c > 0 else -1.0) != s0:
                            self.get_logger().info(
                                '    side step: CROSSED centre  %+.3f -> %+.3f'
                                ' m (confirmed %+.3f) after %d pulse(s)'
                                % (prev, e, c, i))
                            return True
                        self.get_logger().info(
                            '    side step: apparent crossing at %+.3f not '
                            'confirmed (%s) - centre estimate noise, '
                            'continuing' % (e, 'None' if c is None
                                            else '%+.3f' % c))
                        e = c if c is not None else e
                elif abs(e) <= a.side_tol:
                    self.get_logger().info(
                        '    side step %d: offset %+.3f m within --side-tol '
                        '%.3f - done' % (i + 1, e, a.side_tol))
                    return True

                # The one-shot sign flip that used to live here is gone:
                # it existed to paper over the frame bug described in
                # lateral_error. With the error measured on the robot's own
                # left axis the convention cannot be inverted, and two
                # competing correction mechanisms would fight each other.

                coarse = abs(e) >= a.side_coarse_m
                dur = a.side_dur_coarse if coarse else a.side_dur_fine
                vy = -math.copysign(a.side_vy, e)   # +vy is LEFT (measured)
                self.get_logger().info(
                    '    side step %d/%d: offset %+.3f -> vy %+.2f for %.2f s '
                    '(%s, stop-on-%s)'
                    % (i + 1, a.side_max_pulses, e, vy, dur,
                       'coarse' if coarse else 'fine', a.side_stop))
                prev = e
                self.burst(0.0, 0.0, dur, 'side step', vy=vy,
                           settle=a.side_settle)
            e = self.lateral_error(en, sd)
            self.get_logger().warn(
                '    side step: used all %d pulses WITHOUT crossing centre, '
                'offset now %s (started %+.3f). If the offset never shrank, '
                'the robot is probably not moving laterally at all - check '
                'with nav/vy_probe.py, which measures displacement against a '
                'FROZEN origin instead of the wandering registry centre.'
                % (a.side_max_pulses,
                   'unknown' if e is None else '%+.3f m' % e, s0 * abs(prev)))
            return True

        def heading_recheck(self, yaw_target_deg, label):
            """Re-square the heading after the side step.

            MEASURED: a 0.40 s side-step pulse drifts yaw 0.84 deg on average
            and exceeded ang_tol 1.0 deg on 2 of 5 pulses, with an 87%
            clockwise bias that accumulates across pulses. So the heading the
            nav stage set with --finish-yaw does NOT survive a side step.
            """
            if a.recheck_yaw != 'on':
                return True

            def read():
                h = self.robot_heading_deg()
                if h is None:
                    return None
                return (yaw_target_deg - h + 180.0) % 360.0 - 180.0

            e0 = read()
            if e0 is None:
                self.get_logger().warn('    yaw recheck: no heading - skipped')
                return True
            if abs(e0) <= a.ang_tol:
                self.get_logger().info(
                    '    yaw recheck (%s): %+.2f deg, within %.2f - no turn'
                    % (label, e0, a.ang_tol))
                return True
            self.get_logger().info('    yaw recheck (%s): %+.2f deg -> turning'
                                   % (label, e0))
            return edge_servo.step_to_zero(
                read=read,
                turn=lambda d, dur: self.burst(0.0, d * a.w_turn, dur,
                                               'yaw recheck'),
                tol=a.ang_tol, step_dur=a.step_dur, settle=0.0,
                max_steps=a.max_iters,
                log=lambda m: self.get_logger().info(m),
                abort=lambda: self.stop_flag or self.abort,
                wait=self.spin,
                # settle=0.0 is correct HERE: burst() already settles
                # --settle 1.2 s internally, which is above the pose lag's
                # measured 0.647 s maximum.
                #
                # stop_on_cross=False: the default True returns success the
                # moment the sign flips, so this used to finish up to one
                # step (~4 deg) off square and report OK. Same bug as
                # send_goal.finish_yaw had.
                stop_on_cross=False)

        def robot_xy(self):
            try:
                tr = self.buf.lookup_transform('map', 'body', rclpy.time.Time())
                return (tr.transform.translation.x, tr.transform.translation.y)
            except Exception:
                return None

        # ---- Nav2, through send_goal.py
        def goto(self, x, y, yaw, label):
            """Reach the pose with send_goal.py, which owns BOTH position and
            heading.

            Not the raw Nav2 actions any more. Two reasons:

            * HEADING. goal_checker.yaw_goal_tolerance is 3.00 rad (172 deg) on
              purpose, so Nav2 does not turn at the end at all - a pose's yaw
              is effectively ignored. Driving the actions directly therefore
              arrived at the stage pose pointing anywhere, and the camera then
              had to find a tabletop it was not facing. send_goal --finish-yaw
              steps the heading to a zero crossing after position is reached,
              which IS the first alignment of the edge sequence.
            * ROBUSTNESS. send_goal replans every --replan seconds, switches
              path only on a real gain, locks the path inside --final-lock, and
              survives the ABORTED that a preempted FollowPath returns. The
              inline version here planned once and followed once.

            A subprocess, not an import: send_goal owns its own rclpy node and
            publishes /cmd_vel during the yaw finish. Running it in-process
            would put two nodes and two /cmd_vel publishers in one executor.
            It blocks, so the two never command at the same time.
            """
            yaw_deg = math.degrees(yaw)
            self.say(phase='nav', target=[float(x), float(y), float(yaw)],
                     note=label)
            self.get_logger().info('  NAV -> (%.2f, %.2f) yaw %+.0f deg  [%s]'
                                   % (x, y, yaw_deg, label))
            cmd = ['/usr/bin/python3', SEND_GOAL,
                   '%.3f' % x, '%.3f' % y, '%.1f' % yaw_deg,
                   '--finish-yaw', '--timeout', '%.0f' % a.nav_timeout]
            if a.dry_run:
                self.get_logger().info('    [dry run] would run: %s'
                                       % ' '.join(cmd))
                return True
            try:
                pr = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True,
                                      bufsize=1)
            except OSError as exc:
                self.get_logger().error('  cannot run send_goal.py: %s' % exc)
                return False
            reached = False
            t0 = time.time()
            try:
                for line in pr.stdout:
                    line = line.rstrip()
                    if line:
                        self.get_logger().info('    | %s' % line)
                    if 'REACHED' in line:
                        reached = True
                    if self.stop_flag or self.abort:
                        self.get_logger().warn('  stop requested - killing nav')
                        pr.terminate()
                        break
                    if time.time() - t0 > a.nav_timeout + 30.0:
                        self.get_logger().error('  send_goal overran its own '
                                                'timeout - killing it')
                        pr.terminate()
                        break
            finally:
                try:
                    rc = pr.wait(timeout=10.0)
                except Exception:
                    pr.kill()
                    rc = -9
            # exit code is authoritative; the REACHED line is the fallback for
            # an older send_goal.py that always exited 0
            ok = (rc == 0) or reached
            if not ok:
                self.get_logger().error('  nav FAILED (rc %s) [%s]' % (rc, label))
            return ok

        # ---- fine motion, closed on the camera
        def new_camera_stack(self):
            """RealSense + plane_detect, brought up in dependency order.

            The camera first: plane_detect subscribes to it, and starting the
            detector against a camera that is not yet publishing means the
            forward search reads "no plane" while it waits.
            """
            log = lambda m: self.get_logger().info(m.rstrip())
            cam = PLANE_PROC.RealSense(
                depth_topic=a.depth_topic, master=a.ros1_master,
                ready_timeout=a.camera_ready_timeout, tries=a.camera_tries,
                log=log)
            return PLANE_PROC.CameraStack(
                manage_camera=(a.camera_manage == 'on'), camera=cam, log=log,
                table_h=a.table_h, cam_h=a.cam_h, pitch=a.pitch,
                warmup=a.plane_warmup,
                ready=lambda: plane_answers(a.plane_url),
                ready_timeout=a.plane_ready_timeout)

        def camera_window(self):
            """The camera stack for ONE edge's camera phase.

            A context manager and not a start/stop pair because the camera
            phase has a `continue` on failure, and an early exit that skipped
            the stop would leave a 251%% detector and a 97%% nodelet running
            for the rest of the mission.

            Under --camera-scope mission the stack is already open, so this
            hands back a passthrough that must NOT tear it down on exit.
            """
            if a.plane_manage != 'on' or PLANE_PROC is None:
                return _NullPlane()
            if a.camera_scope == 'mission':
                return _Borrowed(self.cam)
            return self.new_camera_stack()

        def close_camera(self):
            """Tear down a mission-scope stack. Safe to call twice."""
            if getattr(self, 'cam', None) is not None:
                self.get_logger().info('closing the mission-scope camera stack')
                try:
                    self.cam.stop()
                except Exception as e:
                    self.get_logger().warn('camera teardown: %s' % e)
                self.cam = None

        def do_side_motion(self, sd):
            """search -> align -> approach, from edge_servo.

            Replaces the old burst-based search_forward/approach/align. Those
            commanded 0.25 s bursts which the bridge slew-limited to 0.09 m/s,
            under the gait's ~0.20 m/s threshold - the robot twitched on every
            burst and never moved, while the script counted phantom travel.
            """
            self.servo.reset()
            self.say(phase='side', side=sd, note='search/align/approach')
            ok = self.servo.do_side(
                a.standoff if a.trim == 'on' else None)
            self.get_logger().info(
                '    side %s: %s   (commanded %.2f m, camera saw %.2f m)'
                % (sd, 'OK' if ok else 'FAILED', self.servo.travel,
                   self.servo.seen))
            return ok

        def back_off(self, metres):
            """Reverse clear of the costmap's no-go band so Nav2 can plan."""
            self.burst(-a.v_fwd, 0.0, metres / a.v_fwd, 'back off %.2f m' % metres)

        # ---- the mission
        def poses_for(self, en):
            return side_poses(en['cx'], en['cy'], en['yaw'], a.side_len,
                              a.robot_radius,
                              {'stage': a.stage, 'stand': a.standoff})

        def report(self, tid, **kw):
            """Tell the registry. A DRY RUN must never do this - otherwise
            rehearsing the mission permanently marks real tables VISITED or
            REJECTED, and the only way back is restarting the registry."""
            msg = json.dumps(dict(id=tid, **kw))
            if a.dry_run:
                self.get_logger().info('    [dry run] would report %s' % msg)
                return
            self.ev.publish(String(data=msg))

        def resolve(self, want):
            """Find the registry entry a command names. None if absent."""
            if self.reg is None:
                return None
            for e in self.reg.get('entries', []):
                if norm_id(e['id']) == want:
                    return e
            return None

        def known_ids(self):
            if self.reg is None:
                return []
            return [e['id'] for e in self.reg.get('entries', [])]

        def visit(self, en):
            """Walk all four edges of one table. Returns True if any confirmed."""
            self.get_logger().info(
                '=== %s  %s  %.2fx%.2f m at (%.2f, %.2f)  %.1f m away'
                % (en['id'], en['cls'], en['L'], en['W'],
                   en['cx'], en['cy'], en.get('range', -1)))
            # The registry's TABLE chunk is the table PLUS its ring of chairs
            # (measured 2.1-2.9 m across for a 1.40 m table). The standoff pose
            # is computed from --side-len, i.e. the bare table, so when the
            # chairs are in, that pose sits INSIDE the chair ring: Nav2 cannot
            # reach it and the camera cannot see the tabletop from there.
            if en['L'] > a.side_len + 0.40:
                self.get_logger().warn(
                    '  chunk is %.2f m across but --side-len is %.2f - the '
                    'chairs are in. The %.2f m standoff pose lands inside the '
                    'chair ring; pull the chairs out or raise --side-len.'
                    % (en['L'], a.side_len, a.standoff))
            if a.confirm == 'on' and not self.ask_confirm(en):
                return False
            self.say(phase='select', table=en['id'], side=None,
                     note='%s %s at (%.2f, %.2f)'
                          % (en['id'], en['cls'], en['cx'], en['cy']))
            P = self.poses_for(en)
            confirmed = False
            saw_camera = False
            for sd in sides:
                if self.stop_flag or self.abort:
                    break
                self.say(phase='side', table=en['id'], side=sd,
                         note='starting side %s' % sd)
                self.get_logger().info('  -- side %s' % sd)
                x, y, yaw = P[(sd, 'stage')]
                if not self.goto(x, y, yaw, '%s stage' % sd):
                    self.get_logger().error('  cannot reach side %s' % sd)
                    continue
                # --- lateral correction, then re-square the heading ---------
                # goto() has already done orientation correction 1 via
                # send_goal --finish-yaw. The side step absorbs the lateral
                # drift that correction leaves, and because it costs ~0.84 deg
                # of yaw per pulse the heading is re-checked afterwards.
                self.say(phase='side', table=en['id'], side=sd,
                         note='lateral correction')
                if not self.side_step(en, sd):
                    self.get_logger().warn('  side %s aborted in side step' % sd)
                    continue
                self.heading_recheck(math.degrees(yaw), 'after side step')
                # the detector runs ONLY from here to the end of the camera
                # work. back_off below does not need it.
                with self.camera_window() as pw:
                    if not getattr(pw, 'ok', True):
                        self.get_logger().error(
                            '  side %s: plane detector would not start' % sd)
                        self.back_off(0.35)
                        continue
                    if not self.do_side_motion(sd):
                        self.get_logger().warn('  side %s did not complete'
                                               % sd)
                        self.back_off(0.35)
                        continue
                    d = self.plane.fresh_ok()
                    if self.plane.read() is not None:
                        saw_camera = True
                    if d:
                        confirmed = True
                        self.get_logger().info(
                            '  side %s DONE   edge %.3f m  angle %+.1f  '
                            'rms %.0f mm'
                            % (sd, d['edge_depth'],
                               d.get('edge_angle', float('nan')),
                               1000 * d['rms']))
                        self.report(en['id'], side=sd)
                self.back_off(a.stage - a.standoff + 0.1)
            # REJECTED means "the camera looked and there is no table-height
            # surface". If the camera never answered, or navigation never got
            # there, that is an INFRASTRUCTURE failure and says nothing about
            # the table - leave it open, or one bad run burns every candidate.
            if confirmed:
                self.report(en['id'], state='VISITED')
                self.get_logger().info('=== %s -> VISITED' % en['id'])
            elif saw_camera:
                self.report(en['id'], state='REJECTED')
                self.get_logger().info('=== %s -> REJECTED (camera saw no '
                                       'tabletop)' % en['id'])
            else:
                self.get_logger().error(
                    '=== %s -> left OPEN: the plane detector never answered. '
                    'Is the camera terminal running? %s'
                    % (en['id'], a.plane_url))
            return confirmed

        def preflight(self):
            if a.plane_manage == 'on':
                # The detector is SUPPOSED to be down now, so "is it
                # answering" is the wrong question. Start it once here instead:
                # it proves the env crossing works and the camera is alive
                # BEFORE the robot walks anywhere, rather than failing at the
                # first edge.
                if PLANE_PROC is None:
                    self.get_logger().error(
                        'camview/plane_proc.py is missing, so the detector '
                        'cannot be managed. Use --plane-manage off and start '
                        'it by hand.')
                    return bool(a.ignore_no_camera)
                self.get_logger().info(
                    'preflight: bringing the camera stack up once to check it '
                    '(camera %s, scope %s)'
                    % (a.camera_manage, a.camera_scope))
                stack = self.new_camera_stack()
                ok = stack.start()
                if not ok:
                    self.get_logger().error(
                        'the camera stack would not come up. Checked: ROS 1 '
                        'master at %s, %s, and the detector at %s'
                        % (a.ros1_master, a.depth_topic, a.plane_url))
                    self.get_logger().error(
                        'rc 97 means camview/env.sh refused because a ROS 2 '
                        'variable leaked into its environment.')
                    stack.stop()
                    if not a.ignore_no_camera:
                        self.get_logger().error(
                            'refusing to start '
                            '(--ignore-no-camera to override)')
                        return False
                    return True
                if a.camera_scope == 'mission':
                    # keep it: one start for the whole run
                    self.cam = stack
                    self.get_logger().info(
                        'camera stack UP and held for the whole mission '
                        '(--camera-scope mission)')
                else:
                    self.get_logger().info(
                        'camera stack starts and answers. Stopping it again '
                        'until the first camera phase.')
                    stack.stop()
                return True
            if self.plane.read() is None:
                self.get_logger().error(
                    'the plane detector at %s is not answering.' % a.plane_url)
                self.get_logger().error(
                    'Without it every table fails its approach. Start it:')
                self.get_logger().error(
                    '    source ~/.scuba/camview/env.sh')
                self.get_logger().error(
                    '    python3 ~/.scuba/camview/plane_detect.py '
                    '--cam-h 1.262 --table-h 0.76')
                if not a.ignore_no_camera:
                    self.get_logger().error('refusing to start '
                                            '(--ignore-no-camera to override)')
                    return False
            else:
                self.get_logger().info('plane detector: reachable')
            return True

        def run(self):
            self.start_stdin()
            self.get_logger().info(
                'mission%s   target edge depth %.3f m  standoff %.2f m '
                '(body to table edge)  sides %s'
                % ('  [DRY RUN - no motion]' if a.dry_run else '',
                   a.target, a.standoff, sides))
            if not self.preflight():
                return
            if a.auto:
                self.get_logger().info('AUTO mode: visiting every open table, '
                                       'nearest first')
                return self.run_auto()
            self.get_logger().info(
                'WAITING for a table command. The robot will not move until '
                'one arrives. Send one with:')
            self.get_logger().info(
                "    ros2 topic pub --once %s std_msgs/String "
                '"{data: \'table_01\'}"' % a.cmd_topic)
            self.get_logger().info(
                '    or:  /usr/bin/python3 ~/.scuba/livemap/visit_table.py '
                'table_01')
            said = None
            while not self.stop_flag:
                self.spin(0.5)
                if self.abort:
                    self.abort = False
                    self.halt()
                    self.get_logger().warn('abort acknowledged, holding')
                want, self.want = self.want, None
                if want is None:
                    note = ('waiting for a table command  (known: %s)'
                            % (', '.join(self.known_ids()) or 'none yet'))
                    if note != said:
                        self.say(phase='idle', table=None, side=None, note=note)
                        said = note
                    continue
                said = None
                if self.reg is None:
                    self.get_logger().warn(
                        'no /table_registry yet - is the registry running?')
                    continue
                en = self.resolve(want)
                if en is None:
                    self.get_logger().error(
                        'no such table %r. Known right now: %s'
                        % (want, ', '.join(self.known_ids()) or 'none'))
                    self.say(phase='idle', table=None, side=None,
                             note='unknown table %s' % want)
                    continue
                # An EXPLICIT command overrides state and the attempt counter:
                # the operator asked for this table, so go, even if a previous
                # run marked it VISITED or REJECTED.
                if en['state'] in ('VISITED', 'REJECTED'):
                    self.get_logger().warn(
                        '%s is %s - revisiting because it was asked for '
                        'explicitly' % (en['id'], en['state']))
                self.visit(en)
                self.halt()
                self.say(phase='idle', table=None, side=None,
                         note='done with %s, waiting for the next command'
                              % en['id'])
                self.get_logger().info(
                    '--- %s finished, back to waiting for a command ---'
                    % en['id'])

        def run_auto(self):
            done = 0
            while not self.stop_flag:
                self.spin(1.0)
                if self.reg is None:
                    self.get_logger().warn('waiting for /table_registry ...')
                    continue
                open_ = [e for e in self.reg.get('entries', [])
                         if e['state'] not in ('VISITED', 'REJECTED')]
                if a.only:
                    open_ = [e for e in open_ if norm_id(e['id']) == norm_id(a.only)]
                open_ = [e for e in open_
                         if self.handled.get(e['id'], 0) < a.max_attempts]
                if not open_:
                    self.say(phase='complete', table=None, side=None,
                             note='no open candidates')
                    self.get_logger().info('no open candidates - complete')
                    return
                en = open_[0]                      # registry ranks nearest first
                self.handled[en['id']] = self.handled.get(en['id'], 0) + 1
                self.visit(en)
                done += 1
                if a.max_tables and done >= a.max_tables:
                    self.get_logger().info('reached --max-tables')
                    return

    rclpy.init()
    n = Mission()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        n.stop_flag = True
        n.halt()
        n.get_logger().info('stopped, zero velocity sent')
        # before destroying the node: a mission-scope stack is otherwise left
        # running, which is the exact 97%+251% leak this feature exists to stop
        try:
            n.close_camera()
        except Exception:
            pass
        # destroy the action clients BEFORE the node: in Foxy, ActionClient
        # __del__ touches the node's handle and throws during interpreter
        # teardown if the node is already gone (send_goal.py hit the same thing)
        for c in ('plan', 'follow'):
            try:
                getattr(n, c).destroy()
            except Exception:
                pass
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
