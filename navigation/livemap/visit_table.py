#!/usr/bin/env python3
"""Send one table command to the mission executive.

    /usr/bin/python3 ~/.scuba/livemap/visit_table.py table_01
    /usr/bin/python3 ~/.scuba/livemap/visit_table.py stop
    /usr/bin/python3 ~/.scuba/livemap/visit_table.py list

Needs a foxy shell:  source ~/.scuba/lio_ws/env.sh
"""
import os
import sys


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    what = sys.argv[1]
    topic = sys.argv[2] if len(sys.argv) > 2 else '/table_command'
    d = os.environ.get('ROS_DISTRO', '')
    if d and d != 'foxy':
        raise SystemExit('!! ROS_DISTRO is %r, not foxy. Open a new terminal '
                         'and source ~/.scuba/lio_ws/env.sh' % d)
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    n = Node('visit_table')
    pub = n.create_publisher(String, topic, 10)
    # a fresh publisher needs a moment to match the subscriber, or the very
    # first message goes nowhere and the robot never moves
    for _ in range(40):
        if pub.get_subscription_count() > 0:
            break
        rclpy.spin_once(n, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('!! nobody is subscribed to %s - is table_mission.py running?'
              % topic)
    pub.publish(String(data=what))
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.02)
    print('sent %r to %s' % (what, topic))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
