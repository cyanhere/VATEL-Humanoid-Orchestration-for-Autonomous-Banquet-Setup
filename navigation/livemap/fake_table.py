#!/usr/bin/env python3
"""Closed-loop stand-in for the table + plane detector, for testing ONLY.

Subscribes /cmd_vel, integrates the forward motion it sees, and serves a
/stat payload on :8093 shaped exactly like plane_detect.py - so the real
edge_approach_test.py can be driven end to end with no robot and no camera.
"""
import argparse, json, math, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

S = dict(x=1.10, yaw=0.0, t=time.time())   # x = camera to edge, metres
LOCK = threading.Lock()
A = None

def payload():
    with LOCK:
        x, yaw = S['x'], S['yaw']
    drop = A.cam_h - A.table_h
    th = math.radians(A.pitch)
    # the tabletop only enters the frame within ~1.12 * drop horizontally
    if x > A.visible:
        return {'ok': False, 'why': 'nothing in the band (simulated: edge at '
                                    '%.2f m, visible from %.2f m)' % (x, A.visible)}
    depth = drop * math.sin(th) + x * math.cos(th)
    return {'ok': True, 'drop': drop, 'rms': 0.004, 'cover': 0.09,
            'tilt': 0.6, 'edge_depth': depth, 'edge_x': x,
            'edge_angle': yaw, 'edge_rms': 0.006, 'edge_bins': 14,
            'aligned': abs(yaw) <= 3.0, 'trigger': True}

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def do_GET(self):
        b = json.dumps(payload()).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(b)))
        self.end_headers()
        self.wfile.write(b)

class Sim(Node):
    """Mirror of nav_bridge._tick, so the sim fails where the robot fails.

    The bridge runs a FIXED-RATE tick that slews its held velocity toward the
    last /cmd_vel target, and treats a command older than cmd_timeout as a
    zero target decelerated stale_decel_factor times harder. Modelling this
    only on message arrival was wrong: between bursts the real bridge RAMPS
    DOWN to zero, while the first version of this sim froze the velocity and
    let the next burst resume from it - which hid the very bug being chased.

    On top of that the gait ignores forward commands below --deadband, so a
    slew-limited short burst twitches without translating.
    """

    def __init__(self):
        super().__init__('fake_table')
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.tgt = (0.0, 0.0)
        self.tgt_t = 0.0
        self.cur = [0.0, 0.0]
        self.last_tick = time.time()
        self.peak = 0.0
        self.create_timer(1.0 / A.rate, self.tick)

    def on_cmd(self, m):
        self.tgt = (float(m.linear.x), float(m.angular.z))
        self.tgt_t = time.time()

    def tick(self):
        now = time.time()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        stale = (now - self.tgt_t) > A.cmd_timeout
        tx, tw = (0.0, 0.0) if stale else self.tgt
        k = A.stale_decel if stale else 1.0
        for i, (t, acc) in enumerate(((tx, A.accel), (tw, A.ang_accel))):
            step = k * acc * dt
            self.cur[i] += max(-step, min(step, t - self.cur[i]))
        v = self.cur[0] if abs(self.cur[0]) >= A.deadband else 0.0
        w = self.cur[1] if abs(self.cur[1]) >= A.ang_deadband else 0.0
        self.peak = max(self.peak, abs(self.cur[0]))
        if v or w:
            with LOCK:
                S['x'] -= v * dt * A.gain
                S['yaw'] -= math.degrees(w * dt)


def main():
    global A
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, default=1.10)
    ap.add_argument('--yaw', type=float, default=0.0)
    ap.add_argument('--visible', type=float, default=0.45,
                    help='edge_x beyond which the tabletop is out of frame')
    ap.add_argument('--cam-h', type=float, default=1.25)
    ap.add_argument('--table-h', type=float, default=0.85)
    ap.add_argument('--pitch', type=float, default=63.6)
    ap.add_argument('--rate', type=float, default=20.0,
                    help='bridge tick rate')
    ap.add_argument('--cmd-timeout', type=float, default=0.4)
    ap.add_argument('--stale-decel', type=float, default=3.0)
    ap.add_argument('--accel', type=float, default=0.35,
                    help="nav_bridge max_lin_acc; THIS is what starved the "
                         "0.25 s bursts")
    ap.add_argument('--ang-accel', type=float, default=0.70)
    ap.add_argument('--deadband', type=float, default=0.20,
                    help='gait ignores forward commands below this (measured)')
    ap.add_argument('--ang-deadband', type=float, default=0.10)
    ap.add_argument('--gain', type=float, default=1.0,
                    help='actual travel / commanded; the real gait overshoots')
    ap.add_argument('--port', type=int, default=8093)
    A = ap.parse_args()
    S['x'], S['yaw'] = A.start, A.yaw
    srv = ThreadingHTTPServer(('127.0.0.1', A.port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('fake table: edge at %.2f m, visible within %.2f m, port %d'
          % (A.start, A.visible, A.port))
    rclpy.init()
    n = Sim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        with LOCK:
            print('\nfinal simulated edge_x = %.3f m   yaw = %+.1f deg'
                  % (S['x'], S['yaw']))
        print('peak slew-limited speed reached: %.3f m/s  (gait deadband %.2f)'
              % (n.peak, A.deadband))
        if n.peak < A.deadband:
            print('  -> NEVER crossed the deadband: the robot would only '
                  'twitch, exactly as observed')
        srv.shutdown(); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
