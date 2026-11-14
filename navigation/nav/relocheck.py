#!/usr/bin/env python3
"""Is the relocalizer CORRECTING, or only holding its seed?

    /usr/bin/python3 ~/.scuba/nav/relocheck.py

READ-ONLY. Subscribes and reads TF; publishes nothing.

Run it before each drift run: run A must say ICP ON, run B must say ICP OFF.
A mislabelled run is a wasted 60 m walk.

THREE CHECKS, WEAKEST TO STRONGEST

1. The `seed_only` parameter. Authoritative about INTENT, but it only says
   what was asked for, not what is happening.
2. /relocalizer/inliers and /fitness. With seed_only the ICP loop never runs,
   so these are silent. Behavioural, and quick.
3. Whether map->camera_init actually MOVES. This is the real test, because it
   is the correction itself. MEASURED with ICP on and the robot stationary:
   net 0.0022 m but a path length of 0.0785 m over 20 s - it jitters
   continuously. With ICP off it is frozen and the path is ~0.000 m.

Check 3 is the one to trust. 1 can be stale if the node was restarted with
different args than you remember; 2 can mislead if a topic is simply not being
received. Only 3 observes the correction directly.
"""
import argparse
import math
import sys
import time


def main():
    ap = argparse.ArgumentParser(
        description='Report whether the relocalizer is correcting. Read-only.')
    ap.add_argument('--secs', type=float, default=20.0,
                    help='how long to watch map->camera_init (default 20)')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='camera_init')
    ap.add_argument('--move-thresh', type=float, default=0.005,
                    help='m of path over the window below which the origin is '
                         'considered FROZEN. ICP on measures ~0.08 m per 20 s, '
                         'so this is a wide margin')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import Float32
    import tf2_ros

    rclpy.init()
    n = Node('relocheck')
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    inl, fit = [], []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    n.create_subscription(Float32, '/relocalizer/inliers',
                          lambda m: inl.append(m.data), qos)
    n.create_subscription(Float32, '/relocalizer/fitness',
                          lambda m: fit.append(m.data), qos)

    # ---- 1. the parameter
    seed = None
    try:
        from rcl_interfaces.srv import GetParameters
        cli = n.create_client(GetParameters, '/relocalizer/get_parameters')
        if cli.wait_for_service(timeout_sec=5.0):
            req = GetParameters.Request()
            req.names = ['seed_only']
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
            if fut.done() and fut.result() and fut.result().values:
                v = fut.result().values[0]
                if getattr(v, 'type', 0) == 1:
                    seed = bool(v.bool_value)
    except Exception:
        pass

    print('\n1. seed_only parameter : %s'
          % ('<could not read - is /relocalizer running?>' if seed is None
             else ('True  (ICP disabled by request)' if seed
                   else 'False (ICP enabled by request)')))

    # ---- 3. watch the correction
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    pts = []
    print('\n3. watching %s -> %s for %.0f s ...'
          % (a.map_frame, a.odom_frame, a.secs), flush=True)
    t0 = time.time()
    while time.time() - t0 < a.secs:
        rclpy.spin_once(n, timeout_sec=0.02)
        try:
            tr = buf.lookup_transform(a.map_frame, a.odom_frame,
                                      rclpy.time.Time())
        except Exception:
            time.sleep(0.1)
            continue
        pts.append((tr.transform.translation.x, tr.transform.translation.y))
        time.sleep(0.1)

    el = time.time() - t0
    print('\n2. ICP diagnostics over %.0f s' % el)
    if inl:
        inl.sort()
        print('   inliers : %d msgs, %.3f .. %.3f   -> ICP IS cycling'
              % (len(inl), inl[0], inl[-1]))
    else:
        print('   inliers : SILENT                    -> ICP is NOT cycling')
    if fit:
        fit.sort()
        print('   rmse    : %d msgs, %.3f .. %.3f' % (len(fit), fit[0], fit[-1]))
    else:
        print('   rmse    : SILENT')

    if len(pts) < 5:
        print('\n   !! no %s -> %s TF at all. The relocalizer is not publishing;'
              % (a.map_frame, a.odom_frame))
        print('      that is a DIFFERENT problem from seed_only - nothing has')
        print('      a pose, and nav will refuse to start.')
        n.destroy_node(); rclpy.shutdown()
        return 2

    net = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    path = sum(math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
               for i in range(len(pts) - 1))
    jumps = sorted(math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
                   for i in range(len(pts) - 1))
    print('   origin  : net %.4f m, path %.4f m, max step %.4f m (%d samples)'
          % (net, path, jumps[-1], len(pts)))

    frozen = path < a.move_thresh
    print('\n' + '=' * 58)
    if frozen and not inl:
        print('VERDICT: ICP IS OFF. The origin is frozen (path %.4f m) and the'
              % path)
        print('         diagnostics are silent. This is a valid RUN B.')
        rc = 0
    elif not frozen and inl:
        print('VERDICT: ICP IS ON and correcting (path %.4f m over %.0f s,'
              % (path, el))
        print('         %d inlier reports). This is a valid RUN A.' % len(inl))
        rc = 0
    else:
        print('VERDICT: INCONSISTENT - do not trust this run.')
        print('         origin %s but diagnostics %s.'
              % ('frozen' if frozen else 'moving',
                 'silent' if not inl else 'reporting'))
        print('         A frozen origin with live inliers can mean ICP is')
        print('         running but finding no correction to make; moving with')
        print('         silent diagnostics means something else is publishing')
        print('         that transform. Check for a second relocalizer:')
        print('             ros2 node list | sort | uniq -c')
        rc = 1
    if seed is not None:
        want_off = seed
        actually_off = frozen and not inl
        if want_off != actually_off:
            print('\n!! the parameter says seed_only=%s but the behaviour says'
                  % seed)
            print('   ICP is %s. Believe the BEHAVIOUR.'
                  % ('OFF' if actually_off else 'ON'))
            rc = 1
    print('=' * 58)
    n.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
