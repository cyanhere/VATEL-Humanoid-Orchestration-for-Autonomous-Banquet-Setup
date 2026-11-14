#!/usr/bin/env python3
"""Verify the send_goal workflow: distance to goal vs WHO is driving /cmd_vel.

The question this answers is the handoff:

    DWB (controller_server) drives  ->  goal completes  ->  DWB DROPS
      ->  send_goal's finish_yaw drives  ->  heading sign flips  ->  done

and whether anything drives when nothing should. Publishers are identified by
NODE NAME, from rclpy's get_publishers_info_by_topic, so "someone else" is
never a guess (that is what whodrives.py got wrong: it inferred from rate).

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/flowlog.py

Start it BEFORE send_goal. Ctrl-C to stop; it prints a phase timeline and
writes a CSV. Read-only: it publishes nothing and cannot move the robot.
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       HistoryPolicy)
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--goal-topic', default='/nav_goal')
    ap.add_argument('--goal', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'),
                    help='use this goal instead of waiting for --goal-topic')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    ap.add_argument('--hz', type=float, default=5.0, help='console/CSV rate')
    ap.add_argument('--xy-tol', type=float, default=0.30,
                    help='goal_checker.xy_goal_tolerance, for annotation')
    ap.add_argument('--rotate-tol', type=float, default=0.25,
                    help='FollowPath.xy_goal_tolerance - where RotateToGoal engages')
    ap.add_argument('--out', default='')
    a = ap.parse_args()

    rclpy.init()
    n = Node('flowlog')
    buf = Buffer()
    TransformListener(buf, n)

    cmd = {'t': 0.0, 'v': (0.0, 0.0, 0.0), 'n': 0}

    def on_cmd(m):
        cmd['t'] = time.time()
        cmd['v'] = (m.linear.x, m.linear.y, m.angular.z)
        cmd['n'] += 1

    n.create_subscription(Twist, a.cmd_topic, on_cmd, 10)

    goal = {'xy': None, 'yaw': None}
    if a.goal:
        goal['xy'] = (a.goal[0], a.goal[1])
        goal['yaw'] = a.goal[2]
        print('goal fixed from the command line: %.2f, %.2f  yaw %.1f deg'
              % (a.goal[0], a.goal[1], a.goal[2]))
    else:
        # resolve the goal topic's type at runtime: it may be PoseStamped or
        # PoseWithCovarianceStamped depending on who publishes it
        gt = None
        t0 = time.time()
        while time.time() - t0 < 3.0 and gt is None:
            for name, types in n.get_topic_names_and_types():
                if name == a.goal_topic and types:
                    gt = types[0]
            rclpy.spin_once(n, timeout_sec=0.1)
        if gt is None:
            print('!! %s not advertised yet; it will be picked up when it '
                  'appears, or pass --goal X Y YAW_DEG' % a.goal_topic)
        else:
            mod, _, cls = gt.replace('/msg/', '/').rpartition('/')
            M = getattr(__import__(mod + '.msg', fromlist=[cls]), cls)
            print('goal topic %s is %s' % (a.goal_topic, gt))

            def on_goal(m):
                p = m.pose.pose if hasattr(m.pose, 'pose') else m.pose
                goal['xy'] = (p.position.x, p.position.y)
                goal['yaw'] = math.degrees(yaw_of(p.orientation))

            q = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
            n.create_subscription(M, a.goal_topic, on_goal, q)

    def publishers():
        """Node names currently publishing cmd_topic."""
        try:
            info = n.get_publishers_info_by_topic(a.cmd_topic)
        except Exception:
            return ['?x%d' % n.count_publishers(a.cmd_topic)]
        out = []
        for e in info:
            nm = getattr(e, 'node_name', '') or '?'
            ns = getattr(e, 'node_namespace', '') or ''
            out.append((ns.rstrip('/') + '/' + nm) if ns not in ('', '/') else nm)
        return sorted(out)

    def pose():
        try:
            tr = buf.lookup_transform(a.map_frame, a.base_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tr.transform.translation
        return (t.x, t.y, math.degrees(yaw_of(tr.transform.rotation)))

    out = os.path.expanduser(
        a.out or '~/.scuba/logs/flow_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fh = open(out, 'w', newline='')
    w = csv.writer(fh)
    w.writerow(['t', 'x', 'y', 'heading_deg', 'goal_x', 'goal_y', 'goal_yaw_deg',
                'dist', 'yaw_err_deg', 'cmd_vx', 'cmd_vy', 'cmd_vyaw',
                'cmd_hz', 'publishers', 'phase'])

    print()
    print('  %7s %7s %8s  %7s %8s %5s  %-26s %s'
          % ('t', 'dist', 'yaw err', 'cmd vx', 'cmd vyaw', 'Hz', 'publisher(s)', 'phase'))
    print('  ' + '-' * 104)

    t0 = time.time()
    last_n, last_t = 0, t0
    prev_phase, prev_pubs = None, None
    prev_dist = None
    timeline = []
    try:
        while rclpy.ok():
            tick = time.time()
            while time.time() - tick < 1.0 / a.hz:
                rclpy.spin_once(n, timeout_sec=0.01)

            now = time.time()
            hz = (cmd['n'] - last_n) / max(now - last_t, 1e-6)
            last_n, last_t = cmd['n'], now
            fresh = (now - cmd['t']) < 0.5
            vx, vy, vw = cmd['v'] if fresh else (0.0, 0.0, 0.0)
            pubs = publishers()
            p = pose()

            dist = yerr = None
            if p and goal['xy']:
                dist = math.hypot(p[0] - goal['xy'][0], p[1] - goal['xy'][1])
                yerr = (goal['yaw'] - p[2] + 180.0) % 360.0 - 180.0

            dwb = any('controller_server' in s for s in pubs)
            other = [s for s in pubs if 'controller_server' not in s]
            if dwb and fresh:
                phase = 'DWB driving'
            elif other and fresh:
                phase = 'OUR ALGORITHM driving (%s)' % ','.join(other)
            elif dwb and not fresh:
                phase = 'DWB present but silent'
            elif not pubs:
                phase = 'nothing publishing'
            else:
                phase = 'publisher idle'

            if dist is not None:
                if dist < a.xy_tol:
                    phase += '  [inside xy_tol %.2f]' % a.xy_tol
                elif dist < a.rotate_tol:
                    phase += '  [inside RotateToGoal %.2f]' % a.rotate_tol

            print('  %6.1fs %7s %8s  %7.3f %+8.3f %5.1f  %-26s %s'
                  % (now - t0,
                     '  --' if dist is None else '%6.2f' % dist,
                     '  --' if yerr is None else '%+7.1f' % yerr,
                     vx, vw, hz, ','.join(pubs) if pubs else '(none)', phase))

            w.writerow(['%.3f' % (now - t0),
                        '' if not p else '%.3f' % p[0],
                        '' if not p else '%.3f' % p[1],
                        '' if not p else '%.2f' % p[2],
                        '' if not goal['xy'] else '%.3f' % goal['xy'][0],
                        '' if not goal['xy'] else '%.3f' % goal['xy'][1],
                        '' if goal['yaw'] is None else '%.2f' % goal['yaw'],
                        '' if dist is None else '%.3f' % dist,
                        '' if yerr is None else '%.2f' % yerr,
                        '%.3f' % vx, '%.3f' % vy, '%.3f' % vw,
                        '%.1f' % hz, '|'.join(pubs), phase])

            base = phase.split('  [')[0]
            if base != prev_phase:
                timeline.append((now - t0, dist, base))
                print('      ^^ PHASE -> %s' % base)
                prev_phase = base
            if prev_pubs is not None and pubs != prev_pubs:
                print('      ^^ /cmd_vel publishers changed: %s -> %s'
                      % (','.join(prev_pubs) or '(none)', ','.join(pubs) or '(none)'))
            prev_pubs = pubs
            for thr, nm in ((a.xy_tol, 'xy_goal_tolerance'),
                            (a.rotate_tol, 'RotateToGoal radius')):
                if prev_dist is not None and dist is not None:
                    if prev_dist >= thr > dist:
                        print('      ^^ crossed INTO %s (%.2f m) at t=%.1fs'
                              % (nm, thr, now - t0))
            if dist is not None:
                prev_dist = dist
    except KeyboardInterrupt:
        pass
    finally:
        fh.close()
        print('\n  CSV -> %s' % out)
        if timeline:
            print('\n  PHASE TIMELINE')
            for t, d, ph in timeline:
                print('    %6.1fs  dist %s  %s'
                      % (t, '  --' if d is None else '%5.2f' % d, ph))
            print('\n  expected order: DWB driving -> nothing publishing'
                  ' -> OUR ALGORITHM driving')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
