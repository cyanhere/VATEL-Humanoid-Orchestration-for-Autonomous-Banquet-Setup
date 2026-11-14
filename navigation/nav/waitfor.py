#!/usr/bin/env python3
"""Block until a topic actually publishes, or a TF actually resolves.

    waitfor.py --topic /Odometry --count 3 --timeout 30
    waitfor.py --tf map body --timeout 40
    waitfor.py --node /relocalizer --timeout 20
    waitfor.py --topic /memory_grid --count 1 --timeout 60 --quiet

Exit 0 when the condition is met, 1 on timeout, 2 on a usage problem.

WHY THIS EXISTS INSTEAD OF `ros2 topic hz`

Measured 2026-10-29 at load 16: `ros2 topic hz` printed "NO MESSAGES in 12s"
for /tf, /Odometry and /cloud_registered while all three were publishing at 26,
9 and 9 Hz. Its own process was not scheduled often enough to see its own
subscription. A startup script that trusts it will either stall forever or,
worse, conclude a healthy stack is dead and carry on half-started.

This spins one node, counts real messages, and says what it saw.

It subscribes BEST_EFFORT + VOLATILE, which is the maximally compatible
subscriber: it can read a RELIABLE publisher and a TRANSIENT_LOCAL one, while
the reverse pairings silently receive nothing.
"""
import argparse
import sys
import time


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--topic', help='wait until this topic publishes')
    g.add_argument('--tf', nargs=2, metavar=('PARENT', 'CHILD'),
                   help='wait until this transform resolves')
    g.add_argument('--node', help='wait until this node appears in the graph')
    ap.add_argument('--count', type=int, default=1,
                    help='messages required (default 1). Use 3+ when you care '
                         'that it is STREAMING, not that one latched message '
                         'arrived')
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    import rclpy
    from rclpy.node import Node

    def say(m):
        if not a.quiet:
            print(m, flush=True)

    rclpy.init()
    n = Node('waitfor_%d' % (int(time.time() * 1000) % 100000))
    t0 = time.time()
    rc = 1

    try:
        if a.node:
            want = '/' + a.node.lstrip('/')
            while time.time() - t0 < a.timeout:
                rclpy.spin_once(n, timeout_sec=0.1)
                # get_node_names() returns bare names, no leading slash
                if want.lstrip('/') in n.get_node_names():
                    say('  %s up after %.1fs' % (want, time.time() - t0))
                    rc = 0
                    break
            else:
                say('  !! %s did not appear in %.0fs' % (want, a.timeout))

        elif a.tf:
            import tf2_ros
            buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(buf, n)
            parent, child = a.tf
            while time.time() - t0 < a.timeout:
                rclpy.spin_once(n, timeout_sec=0.05)
                try:
                    buf.lookup_transform(parent, child, rclpy.time.Time())
                except Exception:
                    continue
                say('  %s -> %s resolved after %.1fs'
                    % (parent, child, time.time() - t0))
                rc = 0
                break
            else:
                say('  !! %s -> %s never resolved in %.0fs'
                    % (parent, child, a.timeout))

        else:
            from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                                   HistoryPolicy, DurabilityPolicy)
            from rosidl_runtime_py.utilities import get_message
            qos = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
            got = [0]
            sub = [None]

            def ensure_sub():
                """The topic may not exist yet, so discover then subscribe."""
                if sub[0] is not None:
                    return True
                for name, types in n.get_topic_names_and_types():
                    if name == a.topic:
                        try:
                            cls = get_message(types[0])
                        except Exception as e:
                            say('  !! %s has type %s which will not import: %s'
                                % (a.topic, types[0], e))
                            return False
                        sub[0] = n.create_subscription(
                            cls, a.topic,
                            lambda m: got.__setitem__(0, got[0] + 1), qos)
                        return True
                return True

            while time.time() - t0 < a.timeout:
                if not ensure_sub():
                    break
                rclpy.spin_once(n, timeout_sec=0.05)
                if got[0] >= a.count:
                    say('  %s publishing after %.1fs (%d msgs)'
                        % (a.topic, time.time() - t0, got[0]))
                    rc = 0
                    break
            else:
                if sub[0] is None:
                    say('  !! %s is not in the graph after %.0fs'
                        % (a.topic, a.timeout))
                else:
                    say('  !! %s exists but published %d/%d msgs in %.0fs'
                        % (a.topic, got[0], a.count, a.timeout))
    finally:
        try:
            n.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return rc


if __name__ == '__main__':
    sys.exit(main())
