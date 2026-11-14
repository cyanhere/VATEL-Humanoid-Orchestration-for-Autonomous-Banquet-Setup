#!/usr/bin/env python3
"""Report the robot's current FSM mode.

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/fsm.py            # ask once
    /usr/bin/python3 ~/.scuba/nav/fsm.py --watch    # keep polling

Read-only: sends api_id 7001 (GetFsmId) on /api/sport/request and prints the
reply.  It never sets a mode, so it is safe to run beside nav_bridge and Nav2.

The 7001 payload is {"data": <id>} - the id has to be unwrapped; using the dict
directly makes every comparison silently false.
"""
import argparse
import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from unitree_api.msg import Request, Response

LOCO_GET_FSM_ID = 7001

NAMES = {                      # only the ids this codebase has actually verified
    0: 'Zero torque',
    1: 'Damping',
    4: 'Lock stand',
    500: 'Walk (default)',
    801: 'Walk (ai controller)',
}


class FsmQuery(Node):
    def __init__(self):
        super().__init__('fsm_query')
        self._cbg = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._resp, self._ev = {}, {}
        self.pub = self.create_publisher(Request, '/api/sport/request', 10)
        self.create_subscription(Response, '/api/sport/response',
                                 self._on_response, 10, callback_group=self._cbg)

    def _on_response(self, msg):
        rid = msg.header.identity.id
        with self._lock:
            self._resp[rid] = msg
            ev = self._ev.get(rid)
        if ev is not None:
            ev.set()

    def get_fsm(self, timeout=2.0):
        req = Request()
        req.header.identity.api_id = LOCO_GET_FSM_ID
        req.header.identity.id = time.time_ns()
        req.header.lease.id = 0
        req.header.policy.priority = 0
        req.header.policy.noreply = False
        req.parameter = json.dumps({})
        rid = req.header.identity.id
        ev = threading.Event()
        with self._lock:
            self._ev[rid] = ev
        try:
            self.pub.publish(req)
            if not ev.wait(timeout):
                return None, 'no response (is the robot powered and on domain 0?)'
            with self._lock:
                r = self._resp.pop(rid, None)
            if r is None:
                return None, 'response lost'
            if r.header.status.code != 0:
                return None, 'status code %d' % r.header.status.code
            try:
                data = json.loads(r.data)
            except (json.JSONDecodeError, TypeError):
                return None, 'unparseable payload %r' % (r.data,)
            if isinstance(data, dict):
                return data.get('data'), None
            return data, None
        finally:
            with self._lock:
                self._ev.pop(rid, None)
                self._resp.pop(rid, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--watch', action='store_true', help='poll until Ctrl-C')
    ap.add_argument('--interval', type=float, default=1.0)
    a = ap.parse_args()

    rclpy.init()
    node = FsmQuery()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    time.sleep(0.5)                       # let the publisher match subscribers

    try:
        last = object()
        while True:
            fsm, err = node.get_fsm()
            if err:
                line = 'FSM: -- (%s)' % err
            else:
                line = 'FSM: %s  %s' % (fsm, NAMES.get(fsm, '(unrecognised id)'))
            if not a.watch:
                print(line)
                break
            if fsm != last:
                print('%s  %s' % (time.strftime('%H:%M:%S'), line))
                last = fsm
            time.sleep(a.interval)
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
