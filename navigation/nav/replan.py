#!/usr/bin/env python3
"""Call the A* planner continuously, so /plan updates live. Nothing moves.

    # replan to one goal at 2 Hz until Ctrl-C
    /usr/bin/python3 ~/.scuba/nav/replan.py --goal 2.0 -2.0 --rate 2

    # cycle between goals, 6 s on each - the path jumps between routes
    /usr/bin/python3 ~/.scuba/nav/replan.py --goal 2.0 -2.0 --goal -1.5 1.0 \\
        --hold 6 --rate 2

WHY NOT JUST LOOP `ros2 action send_goal`
That spawns a process per call - about a second of startup each time, so the
path updates in visible lurches. This holds one action client open and
re-sends, which is what makes it look live on camera.

WHY NOT NavigateToPose
Nav2's own continuous replanning lives in the BT navigator, which is not
running in this stack (`ros2 action list` shows only /compute_path_to_pose and
/follow_path). It would also hand the path to the controller, which is exactly
what we do not want here.

IT CANNOT MOVE THE ROBOT
It calls ComputePathToPose only: the planner computes, publishes /plan, and
returns. No FollowPath goal is ever sent and no /cmd_vel is ever published. It
warns if nav_bridge is running, because that is the only route to the legs, but
planning is safe either way.

WHAT MAKES GOOD FOOTAGE
The path redraws every cycle, so anything that changes the costmap changes the
route in real time - someone walking across the robot's view is the obvious
one, since costmap_decay admits the new returns and A* immediately routes
around them, then forgets them again once the rays clear the cells.
"""
import argparse
import math
import sys
import time


def main():
    ap = argparse.ArgumentParser(
        description='Continuous A* replanning for filming. Plans only; '
                    'publishes no velocity and sends no FollowPath goal.')
    ap.add_argument('--goal', action='append', nargs=2, type=float,
                    metavar=('X', 'Y'), required=True,
                    help='map-frame goal; repeat the flag to cycle goals')
    ap.add_argument('--rate', type=float, default=2.0,
                    help='replans per second (default 2). The planner is '
                         'configured with expected_planner_frequency 1.0, so '
                         'above ~2 Hz Nav2 may log that planning is slow - '
                         'informational, not a failure')
    ap.add_argument('--hold', type=float, default=0.0,
                    help='s to stay on each goal before moving to the next; '
                         '0 = never switch (single goal)')
    ap.add_argument('--planner-id', default='GridBased')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='s to wait for one planning result')
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose

    rclpy.init()
    n = Node('replan')

    cli = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
    print('waiting for the planner action ...', flush=True)
    if not cli.wait_for_server(timeout_sec=20.0):
        print('!! /compute_path_to_pose never appeared. Is planner_server up?')
        n.destroy_node(); rclpy.shutdown()
        return 1

    # Not a refusal - planning is harmless - but worth knowing out loud.
    try:
        import subprocess
        rc = subprocess.call(['/usr/bin/python3',
                              '%s/.scuba/nav/dupcheck.py' % __import__('os').path.expanduser('~'),
                              'nav_bridge'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if rc != 0:
            print('\n!! nav_bridge IS RUNNING. This script still cannot move the')
            print('   robot - it never publishes /cmd_vel - but if anything else')
            print('   sends a FollowPath goal, the robot WILL walk.\n')
        else:
            print('nav_bridge is down: the robot cannot move.')
    except Exception:
        pass

    goals = [tuple(g) for g in a.goal]
    print('replanning to %s at %.1f Hz. Ctrl-C to stop.\n'
          % (' -> '.join('(%.2f, %.2f)' % g for g in goals), a.rate))

    period = 1.0 / max(a.rate, 0.05)
    gi = 0
    t_goal = time.time()
    cyc = 0
    ok = 0
    fail = 0
    last_len = None
    t_start = time.time()

    try:
        while rclpy.ok():
            if a.hold > 0 and len(goals) > 1 and time.time() - t_goal >= a.hold:
                gi = (gi + 1) % len(goals)
                t_goal = time.time()
                last_len = None
                print('--- goal -> (%.2f, %.2f)' % goals[gi], flush=True)

            gx, gy = goals[gi]
            g = ComputePathToPose.Goal()
            ps = PoseStamped()
            ps.header.frame_id = a.frame
            ps.header.stamp = n.get_clock().now().to_msg()
            ps.pose.position.x = gx
            ps.pose.position.y = gy
            ps.pose.orientation.w = 1.0
            g.pose = ps
            g.planner_id = a.planner_id

            t0 = time.time()
            send = cli.send_goal_async(g)
            rclpy.spin_until_future_complete(n, send, timeout_sec=a.timeout)
            gh = send.result() if send.done() else None
            if gh is None or not gh.accepted:
                fail += 1
                print('   [%4d] goal REJECTED or timed out' % cyc, flush=True)
            else:
                rf = gh.get_result_async()
                rclpy.spin_until_future_complete(n, rf, timeout_sec=a.timeout)
                if not rf.done():
                    fail += 1
                    print('   [%4d] no result in %.0fs' % (cyc, a.timeout),
                          flush=True)
                else:
                    res = rf.result().result
                    pts = [(p.pose.position.x, p.pose.position.y)
                           for p in res.path.poses]
                    if len(pts) < 2:
                        fail += 1
                        print('   [%4d] EMPTY path - goal unreachable from here'
                              % cyc, flush=True)
                    else:
                        ok += 1
                        L = sum(math.hypot(pts[i + 1][0] - pts[i][0],
                                           pts[i + 1][1] - pts[i][1])
                                for i in range(len(pts) - 1))
                        direct = math.hypot(pts[-1][0] - pts[0][0],
                                            pts[-1][1] - pts[0][1])
                        pt = res.planning_time
                        ms = pt.sec * 1000.0 + pt.nanosec / 1e6
                        delta = ''
                        if last_len is not None and abs(L - last_len) > 0.05:
                            delta = '   REROUTED %+.2f m' % (L - last_len)
                        last_len = L
                        if not a.quiet:
                            print('   [%4d] %3d poses  %5.2f m  (direct %.2f m, '
                                  '%.2fx)  plan %5.1f ms%s'
                                  % (cyc, len(pts), L, direct,
                                     L / direct if direct > 0.01 else 0.0,
                                     ms, delta), flush=True)
            cyc += 1
            dt = period - (time.time() - t0)
            if dt > 0:
                end = time.time() + dt
                while time.time() < end and rclpy.ok():
                    rclpy.spin_once(n, timeout_sec=min(0.05, dt))
    except KeyboardInterrupt:
        pass

    el = time.time() - t_start
    print('\n%d cycles in %.0f s (%.2f Hz): %d planned, %d failed'
          % (cyc, el, cyc / el if el > 0 else 0.0, ok, fail))
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
