#!/usr/bin/env python3
"""Send a navigation goal, REPLANNING as the world changes. Foxy field names.

    /usr/bin/python3 send_goal.py X Y [YAW_DEG]
    /usr/bin/python3 send_goal.py X Y --replan 1.5      # replan period, seconds
    /usr/bin/python3 send_goal.py X Y --once            # old one-shot behaviour

WHY REPLAN
A* plans once over the global costmap. With an obstacle layer on that costmap,
a box put down after the goal was sent only changes the plan if somebody asks
for a new one - and DWB cannot go around it alone: it only simulates
sim_time * max_vel_x (0.75 m) ahead, while going around takes metres. So this
recomputes the path every --replan seconds.

WHY IT DOES NOT SIMPLY FOLLOW THE NEWEST PLAN
At a junction the two ways round cost almost the same, and costmap noise flips
which one wins from second to second. Handing over every new plan makes the
robot chase each answer and loop on the spot. So a fresh plan is adopted only
when there is a REASON to:

  * there is no path yet, or the controller stopped
  * the path being followed is now BLOCKED (its cells went lethal in the
    global costmap - checked directly, not guessed)
  * the new path is at least --switch-margin shorter than what is left of the
    current one
  * nothing has been reconsidered for --max-commit seconds (so real changes
    still win eventually)

Otherwise the current path is kept, even though a newer one exists. That is
the point: commitment is what stops the oscillation.

NEAR THE GOAL, REPLANNING STOPS ALTOGETHER (--final-lock). Every switch
preempts the controller, and preemption resets the STATEFUL goal checker that
has already accepted the position and is only waiting on the heading - so the
robot kept re-aligning to each new path stub and never finished. A percentage
margin is also meaningless at that range: with 0.25 m left, 15% is 4 cm, i.e.
noise. Switching therefore needs an absolute gain (--switch-min-gain) as well.

A preempted FollowPath returns ABORTED. That is normal here and is NOT counted
as a failure: only an abort of the path we are currently following is, and a
few of those in a row (--max-aborts) end the run.

DISTANCE
"to goal" is measured HERE, from the robot's own map -> body transform to the
goal - the same pose the path viewer draws. FollowPath's distance_to_goal
feedback is shown too, in brackets, but is not trusted: it disagreed with
reality by over a metre on this robot, and Foxy's nav2_util does not even ship
the path-length helper later releases use, so what it measures is unclear.
The switch-margin test below uses the measured remaining path length for the
same reason.

Foxy's ComputePathToPose goal field is `pose` (later versions renamed it to
`goal`). FollowPath publishes distance_to_goal and speed as feedback.
"""
import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
try:
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
except ImportError:                                    # older Foxy names
    from rclpy.qos import QoSDurabilityPolicy as DurabilityPolicy
    from rclpy.qos import QoSReliabilityPolicy as ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, FollowPath
import tf2_ros


def make_pose(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def path_len(path):
    return sum(math.dist((path.poses[i].pose.position.x, path.poses[i].pose.position.y),
                         (path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y))
               for i in range(len(path.poses) - 1))


class RobotPose:
    """Where the robot is, from TF - the same source the viewer draws."""

    def __init__(self, node, map_frame='map', base_frame='body'):
        self.node, self.map_frame, self.base_frame = node, map_frame, base_frame
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, node)

    def xy(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame,
                                          rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            return None

    def heading_deg(self):
        """Robot heading in the map frame, degrees."""
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame,
                                          rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def remaining_along(path, xy):
    """Path length still ahead: from the pose nearest the robot to the end."""
    if path is None or not path.poses or xy is None:
        return None
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    i = min(range(len(pts)), key=lambda k: math.dist(pts[k], xy))
    return sum(math.dist(pts[k], pts[k + 1]) for k in range(i, len(pts) - 1))


class Costmap:
    """Latest global costmap, for asking whether a path is still passable."""

    def __init__(self, node, topic='/global_costmap/costmap'):
        self.g = None
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE
        node.create_subscription(OccupancyGrid, topic, self._cb, qos)

    def _cb(self, m):
        self.g = m

    def blocked(self, path, lethal=99):
        """True if any pose of the path sits in a lethal / inscribed cell.

        OccupancyGrid carries costmap values rescaled to 0..100: 100 is lethal
        and 99 is the inscribed ring, i.e. the robot centre cannot be there.
        Unknown (-1) is not treated as blocked - the map has unknown space the
        planner is already allowed to route through.
        """
        m = self.g
        if m is None or not path.poses:
            return False
        r, w, h = m.info.resolution, m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        for p in path.poses:
            cx = int((p.pose.position.x - ox) / r)
            cy = int((p.pose.position.y - oy) / r)
            if 0 <= cx < w and 0 <= cy < h:
                v = m.data[cy * w + cx]
                if v >= lethal:
                    return True
        return False


def plan_once(n, plan, goal_pose):
    """Ask A* for a path. Returns the Path, or None."""
    g = ComputePathToPose.Goal()
    g.pose = goal_pose                       # Foxy: 'pose', not 'goal'
    g.planner_id = 'GridBased'
    f = plan.send_goal_async(g)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    gh = f.result()
    if gh is None or not gh.accepted:
        return None
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=15)
    res = rf.result()
    if res is None or not res.result.path.poses:
        return None
    return res.result.path


def finish_yaw(node, robot, a):
    """Turn to the goal heading with the step-to-zero-crossing used for the
    table edges.

    DWB's RotateToGoal was turning the wrong way here. This does not need the
    sign convention to be right: it reads the heading error from TF, takes a
    fixed-size step, and if the error GREW it flips direction and carries on.
    Turn-per-command varies 40-60% with gait phase on this robot, so nothing
    tries to size the correction - it just steps until the error changes sign.
    """
    sys.path.insert(0, os.path.expanduser('~/.scuba/livemap'))
    try:
        from edge_servo import step_to_zero, adaptive_step
    except Exception as exc:
        print('  !! cannot import edge_servo (%s) - skipping yaw finish' % exc)
        return
    from geometry_msgs.msg import Twist

    pub = node.create_publisher(Twist, a.cmd_topic, 10)
    for _ in range(40):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('  !! nothing subscribed to %s - is nav_bridge running? '
              'skipping yaw finish' % a.cmd_topic)
        return

    def spin_for(sec):
        """Service callbacks while waiting, or TF never updates."""
        t1 = time.time()
        while time.time() - t1 < sec:
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.005)

    def err():
        spin_for(0.15)                 # drain TF before believing it
        h = robot.heading_deg()
        if h is None:
            return None
        return (a.yaw_deg - h + 180.0) % 360.0 - 180.0

    def turn(d, dur, fine=False):
        # `fine` is the correct-back step after an overshoot: smallest rate the
        # gait still commits to, so it cannot jump back out of the window.
        t = Twist()
        t.angular.z = float(d * (a.yaw_w_fine if fine else a.yaw_w))
        t1 = time.time()
        while time.time() - t1 < dur:
            pub.publish(t)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.02)
        z = Twist()
        for _ in range(12):
            pub.publish(z)
            rclpy.spin_once(node, timeout_sec=0.005)
            time.sleep(0.02)

    e0 = err()
    if e0 is None:
        print('  !! no TF heading - skipping yaw finish')
        return
    print('  finishing heading: want %+.1f deg, off by %+.1f' % (a.yaw_deg, e0))
    sched = (a.yaw_step if a.yaw_step > 0 else
             adaptive_step(a.yaw_rate, a.yaw_min_step, a.yaw_max_step))
    # Stop when the error is INSIDE +/-yaw-final-tol, not merely when it
    # crosses zero. step_to_zero's crossing short-circuit used to return the
    # moment the sign flipped, so the robot walked away from wherever it
    # happened to land - up to one minimum step past zero, 3.6 deg at the old
    # 0.35 s. That is the tilted approach.
    #
    # It converges because THE STEP IS SMALLER THAN THE WINDOW: +/-2 deg is a
    # 4 deg window, and a 0.25 s step is ~2.5 deg, so a step cannot jump it.
    # Worst case it oscillates once and lands inside.
    #
    # Caveat worth knowing: step size is not fixed. Measured over 22 steps,
    # mean 4.08 deg, sd 1.43, max 5.98 - and a 5.98 deg step CAN jump a 4 deg
    # window. That is why this keeps iterating instead of trusting one pass.
    ok = step_to_zero(err, turn, tol=a.yaw_tol, step_dur=sched,
                      settle=a.yaw_settle, max_steps=a.yaw_max_steps,
                      log=lambda m: print(m), wait=spin_for,
                      autoflip=not a.no_yaw_flip, stop_on_cross=False,
                      fine_after_cross=True, fine_dur=a.yaw_min_step)
    e1 = err()
    inside = (e1 is not None and abs(e1) <= a.yaw_tol)
    print('  heading %s: %+.1f -> %+.1f deg   %s (target +/-%.1f)'
          % ('OK' if ok else 'NOT CONVERGED', e0,
             e1 if e1 is not None else float('nan'),
             'WITHIN' if inside else 'OUTSIDE', a.yaw_tol))
    if not inside and e1 is not None:
        print('  !! walking forward from %+.1f deg off square will track at an'
              ' angle.' % e1)
    z = Twist()
    for _ in range(12):
        pub.publish(z)
        rclpy.spin_once(node, timeout_sec=0.005)
        time.sleep(0.02)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('x', type=float)
    ap.add_argument('y', type=float)
    ap.add_argument('yaw_deg', type=float, nargs='?', default=0.0)
    ap.add_argument('--replan', type=float, default=1.5, help='seconds between replans')
    ap.add_argument('--once', action='store_true', help='plan once, never replan')
    ap.add_argument('--timeout', type=float, default=180.0, help='give up after this long')
    ap.add_argument('--max-aborts', type=int, default=4,
                    help='consecutive controller aborts before giving up')
    ap.add_argument('--switch-margin', type=float, default=0.15,
                    help='adopt a new path only if it is this much shorter (0.15 = 15%%)')
    ap.add_argument('--max-commit', type=float, default=10.0,
                    help='seconds before the current path is reconsidered regardless')
    ap.add_argument('--switch-min-gain', type=float, default=0.30,
                    help='a new path must also save at least this many metres')
    ap.add_argument('--final-lock', type=float, default=1.0,
                    help='stop replanning within this distance of the goal (0 = never)')
    ap.add_argument('--always-switch', action='store_true',
                    help='old behaviour: follow the newest plan every time')
    # DWB's RotateToGoal was observed turning the WRONG WAY at the end of a
    # goal. This finishes the heading with the same step-to-zero-crossing
    # used for the table edges, which does not need the sign convention to be
    # right - it detects a wrong direction from the first step and flips.
    # Measured residual with the camera: 0.67 deg.
    ap.add_argument('--finish-yaw', dest='finish_yaw', action='store_true',
                    default=True,
                    help='step the final heading after Nav2 reaches position')
    ap.add_argument('--no-finish-yaw', dest='finish_yaw', action='store_false')
    ap.add_argument('--yaw-tol', type=float, default=2.0,
                    help='deg; the smallest reliable turn measured is ~4')
    ap.add_argument('--yaw-step', type=float, default=0.0,
                    help='s per step; 0 = adaptive from --yaw-rate')
    ap.add_argument('--yaw-rate', type=float, default=10.2,
                    help='deg/s actually achieved (measured 10.21)')
    ap.add_argument('--yaw-min-step', type=float, default=0.25,
                    help='s. Was 0.35, which costs 3.6 deg of overshoot on the '
                         'last step. 0.25 is ~2.5 deg. Below ~0.20 the gait '
                         'does not commit: 0.15 s measured 0.12 deg.')
    ap.add_argument('--yaw-max-step', type=float, default=1.2)
    ap.add_argument('--yaw-w', type=float, default=0.40, help='rad/s')
    ap.add_argument('--yaw-w-fine', type=float, default=0.30,
                    help='rad/s for the correct-back step after an overshoot. '
                         '0.30 is the MEASURED lowest yaw that moves at all '
                         '(standstill). Degrees per step at this rate are NOT '
                         'measured - if it fails to commit, step_to_zero says '
                         'so and reverts to --yaw-w.')
    ap.add_argument('--yaw-settle', type=float, default=0.6,
                    help='s to let the robot stop before believing TF')
    ap.add_argument('--yaw-max-steps', type=int, default=14)
    ap.add_argument('--no-yaw-flip', action='store_true',
                    help='never reverse direction: turn one way only until '
                         'the sign changes')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--goal-topic', default='/nav_goal',
                    help='where to advertise the goal so the viewer can '
                         'draw it')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--base-frame', default='body')
    a = ap.parse_args()
    goal_pose = make_pose(a.x, a.y, math.radians(a.yaw_deg))

    rclpy.init()
    n = Node('send_goal')
    # Advertise the goal so the viewer can draw it. TRANSIENT_LOCAL so a
    # viewer started later still receives it; a VOLATILE publisher would not
    # match that subscription at all.
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy)
    gq = QoSProfile(depth=1)
    gq.durability = DurabilityPolicy.TRANSIENT_LOCAL
    gq.reliability = ReliabilityPolicy.RELIABLE
    goal_pub = n.create_publisher(PoseStamped, a.goal_topic, gq)
    goal_pose.header.stamp = n.get_clock().now().to_msg()
    goal_pub.publish(goal_pose)
    n.get_logger().info('goal advertised on %s for the viewer'
                        % a.goal_topic)
    plan = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    follow = ActionClient(n, FollowPath, 'follow_path')
    if not plan.wait_for_server(timeout_sec=10.0):
        sys.exit('!! planner_server not available (is nav_dwb.launch.py running?)')
    if not follow.wait_for_server(timeout_sec=10.0):
        sys.exit('!! controller_server not available')

    state = {'d': None, 'v': None}

    def on_fb(msg):
        state['d'] = msg.feedback.distance_to_goal
        state['v'] = msg.feedback.speed

    costmap = Costmap(n)
    robot = RobotPose(n, a.map_frame, a.base_frame)
    handle = None            # goal handle we are currently following
    result_f = None          # its result future
    cur_path = None          # the path it is following
    t_plan = t_switch = 0.0
    replans = aborts = switches = kept = 0
    t0 = time.time()
    status = 'running'
    print('goal (%.2f, %.2f) yaw %.0f deg   replan every %s' %
          (a.x, a.y, a.yaw_deg, 'never (--once)' if a.once else '%.1f s' % a.replan))
    try:
        while rclpy.ok():
            if time.time() - t0 > a.timeout:
                status = 'TIMEOUT after %.0f s' % a.timeout
                break

            # inside the final approach: let the controller finish undisturbed,
            # unless the path it is on has actually become blocked
            xy_now = robot.xy()
            near_goal = (xy_now is not None and a.final_lock > 0 and cur_path is not None
                         and math.dist(xy_now, (a.x, a.y)) <= a.final_lock
                         and not costmap.blocked(cur_path))
            due = (handle is None) or (not a.once and not near_goal
                                       and time.time() - t_plan >= a.replan)
            if due:
                path = plan_once(n, plan, goal_pose)
                t_plan = time.time()
                if path is None:
                    print('\n  !! no path (unreachable, inside an obstacle, or not localized)')
                    if handle is None:
                        status = 'no path'
                        break
                else:
                    replans += 1
                    new_len = path_len(path)
                    # what is LEFT of the current path: the controller's own
                    # distance_to_goal, falling back to its full length
                    # measured from OUR pose along the current path, not from
                    # the controller's distance_to_goal (see DISTANCE above)
                    remaining = remaining_along(cur_path, robot.xy())
                    if remaining is None:
                        remaining = state['d'] if state['d'] is not None else (
                            path_len(cur_path) if cur_path is not None else None)
                    if handle is None or cur_path is None:
                        why = 'first path'
                    elif a.always_switch:
                        why = 'always-switch'
                    elif costmap.blocked(cur_path):
                        why = 'current path BLOCKED'
                    elif (remaining is not None
                          and new_len < remaining * (1.0 - a.switch_margin)
                          and new_len < remaining - a.switch_min_gain):
                        why = '%.0f%% shorter (%.2f < %.2f m)' % (
                            100 * (1 - new_len / remaining), new_len, remaining)
                    elif time.time() - t_switch >= a.max_commit:
                        why = 'committed %.0fs, reconsidering' % a.max_commit
                    else:
                        why = None
                        kept += 1
                    if why is not None:
                        fg = FollowPath.Goal()
                        fg.path = path
                        fg.controller_id = 'FollowPath'
                        ff = follow.send_goal_async(fg, feedback_callback=on_fb)
                        rclpy.spin_until_future_complete(n, ff, timeout_sec=10)
                        fh = ff.result()
                        if fh is None or not fh.accepted:
                            print('\n  !! controller rejected the path')
                        else:
                            # replacing the handle drops the old future: its
                            # ABORTED (from preemption) is never a failure here
                            handle, result_f = fh, fh.get_result_async()
                            cur_path = path
                            t_switch = time.time()
                            switches += 1
                            print('\n  new path (%d poses, %.2f m): %s'
                                  % (len(path.poses), new_len, why))

            rclpy.spin_once(n, timeout_sec=0.1)

            if result_f is not None and result_f.done():
                st = result_f.result().status
                if st == GoalStatus.STATUS_SUCCEEDED:
                    xy = robot.xy()
                    status = 'REACHED' + ('' if xy is None else
                                          ' (%.2f m from the goal)' % math.dist(xy, (a.x, a.y)))
                    break
                aborts += 1
                print('\n  controller stopped (status %d) - replanning [%d/%d]'
                      % (st, aborts, a.max_aborts))
                handle, result_f, cur_path = None, None, None
                if aborts >= a.max_aborts:
                    status = 'gave up after %d aborts' % aborts
                    break
                t_plan = 0.0                 # replan immediately
            elif result_f is not None:
                aborts = 0 if state['d'] is not None else aborts

            xy = robot.xy()
            d = '  --' if xy is None else '%5.2f m' % math.dist(xy, (a.x, a.y))
            fb = '' if state['d'] is None else ' (fb %.2f)' % state['d']
            v = '  --' if state['v'] is None else '%5.2f m/s' % state['v']
            print('\r  t=%5.1fs  to goal %s%s  speed %s  plans %d  switched %d  kept %d%s   '
                  % (time.time() - t0, d, fb, v, replans, switches, kept,
                     '  [final approach]' if near_goal else ''), end='', flush=True)
    except KeyboardInterrupt:
        status = 'aborted by user'
        if handle is not None:
            handle.cancel_goal_async()
            time.sleep(1.0)

    print('\n%s   (%.0f s, %d plans, %d switches, %d kept, %d aborts)'
          % (status, time.time() - t0, replans, switches, kept, aborts))

    if a.finish_yaw and status.startswith('REACHED'):
        # DWB must not be commanding while we step the heading, or the two
        # fight and the robot never settles. The action has succeeded by
        # now, but cancel explicitly and let the bridge go stale (0.4 s) so
        # the only thing on /cmd_vel is ours.
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        t_wait = time.time()
        while time.time() - t_wait < 1.0:
            rclpy.spin_once(n, timeout_sec=0.02)
        finish_yaw(n, robot, a)
    # destroy action clients BEFORE the node: ActionClient.__del__ touches the
    # node handle; if the node dies first it raises InvalidHandle at exit.
    for _c in (follow, plan):
        try:
            _c.destroy()
        except Exception:
            pass
    n.destroy_node()
    rclpy.shutdown()
    # An exit code, so a caller can tell success from failure. Previously this
    # always exited 0 and printed the outcome, which is fine interactively but
    # useless to table_mission, which now drives navigation through this
    # script. 0 = position reached (and the yaw finished, if asked).
    return 0 if status.startswith('REACHED') else 1


if __name__ == '__main__':
    sys.exit(main())
