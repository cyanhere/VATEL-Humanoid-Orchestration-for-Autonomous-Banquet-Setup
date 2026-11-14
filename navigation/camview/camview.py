#!/usr/bin/env python3
"""Stream the head RealSense to a web page, so you can see what it actually sees.

    source /opt/ros/noetic/setup.bash
    python3 ~/.scuba/camview/camview.py          # then open :8092

ROS 1, because that is where the camera lives on this robot: realsense2_camera
is installed under Noetic and pyrealsense2 is not installed at all. Subscribes
to the same topics plate_perception.py uses.

READ-ONLY: subscribes to two image topics, publishes nothing, commands nothing.

THE OVERLAY IS THE POINT
The camera is bolted to the torso looking 63.6 deg below horizontal (measured,
not from the URDF - T_TORSO_CAMERA_CALIBRATED in plate_perception/urdf_fk.py).
That is a manipulation pose, aimed at whatever is directly in front of the
robot. The green lines mark where each image row hits the FLOOR, and the amber
line marks where a 0.75 m TABLE TOP would appear. If the amber line is off the
top of the frame, a table at that range is simply not visible, and no amount of
model training changes that.
"""
import argparse
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PITCH_DEG = 63.6          # optical axis below horizontal, from the calibration
STATE = {'color': None, 'depth': None, 'info': None, 't': 0.0}
LOCK = threading.Lock()


def depression(v, cy, fy, pitch_deg):
    """Angle below horizontal of the ray through image row v."""
    return pitch_deg + math.degrees(math.atan((v - cy) / fy))


def row_for_depression(ang_deg, cy, fy, pitch_deg):
    return cy + fy * math.tan(math.radians(ang_deg - pitch_deg))


def annotate(color, depth, K, h_cam, pitch_deg):
    img = color.copy()
    H, W = img.shape[:2]
    fx, fy, cx, cy = K
    # where image rows hit the floor
    for dist in (0.3, 0.5, 0.8, 1.2, 2.0):
        ang = math.degrees(math.atan2(h_cam, dist))
        v = row_for_depression(ang, cy, fy, pitch_deg)
        if 0 <= v < H:
            cv2.line(img, (0, int(v)), (W, int(v)), (0, 190, 0), 1)
            cv2.putText(img, 'floor %.1f m' % dist, (6, int(v) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 190, 0), 1)
    # where a 0.75 m table top would appear
    for dist in (0.5, 1.0, 1.5, 2.0):
        dz = h_cam - 0.75
        if dz <= 0:
            continue
        ang = math.degrees(math.atan2(dz, dist))
        v = row_for_depression(ang, cy, fy, pitch_deg)
        lbl = 'TABLE TOP %.1f m' % dist
        if 0 <= v < H:
            cv2.line(img, (0, int(v)), (W, int(v)), (0, 170, 255), 2)
            cv2.putText(img, lbl, (W - 190, int(v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 170, 255), 1)
        elif v < 0:
            cv2.putText(img, lbl + '  ^ ABOVE FRAME', (W - 300, 16 + 16 * int(dist * 2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 120, 220), 1)
    # centre crosshair + depth probes
    probes = [(cx, cy), (cx, cy * 0.5), (cx, cy * 1.5), (cx * 0.5, cy), (cx * 1.5, cy)]
    for (px, py) in probes:
        px, py = int(px), int(py)
        if not (0 <= px < W and 0 <= py < H):
            continue
        cv2.drawMarker(img, (px, py), (255, 60, 60), cv2.MARKER_CROSS, 12, 1)
        if depth is not None and py < depth.shape[0] and px < depth.shape[1]:
            win = depth[max(0, py-3):py+4, max(0, px-3):px+4].astype(np.float32)
            win = win[win > 0]
            if win.size:
                cv2.putText(img, '%.2fm' % (np.median(win) / 1000.0), (px + 8, py - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 60, 60), 1)
    top = depression(0, cy, fy, pitch_deg)
    bot = depression(H - 1, cy, fy, pitch_deg)
    cv2.rectangle(img, (0, 0), (W, 22), (0, 0, 0), -1)
    cv2.putText(img, 'axis %.1f deg down | frame spans %.1f-%.1f deg below horizontal'
                % (pitch_deg, top, bot), (6, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img


PAGE = b"""<!doctype html><meta charset=utf-8><title>Head camera</title>
<style>body{background:#111;color:#ddd;font:14px/1.5 system-ui,sans-serif;margin:0;padding:16px}
h1{font-size:16px;margin:0 0 4px}p{margin:4px 0 12px;color:#9aa}
.row{display:flex;gap:16px;flex-wrap:wrap}figure{margin:0}
img{max-width:100%;border:1px solid #333;background:#000}
figcaption{color:#9aa;padding:4px 0}</style>
<h1>G1 head RealSense</h1>
<p>green = where that image row meets the floor &middot; amber = where a 0.75 m table top
would appear &middot; blue crosses = live depth</p>
<div class=row>
<figure><img src="/color.mjpg"><figcaption>colour + overlay</figcaption></figure>
<figure><img src="/depth.mjpg"><figcaption>aligned depth</figcaption></figure>
</div>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8092)
    ap.add_argument('--color', default='/camera/color/image_raw')
    ap.add_argument('--depth', default='/camera/aligned_depth_to_color/image_raw')
    ap.add_argument('--info', default='/camera/color/camera_info')
    ap.add_argument('--cam-height', type=float, default=1.25,
                    help='camera height above the floor, metres')
    ap.add_argument('--pitch', type=float, default=PITCH_DEG)
    ap.add_argument('--quality', type=int, default=70)
    a = ap.parse_args()

    import rospy
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    br = CvBridge()

    def on_color(m):
        with LOCK:
            STATE['color'] = br.imgmsg_to_cv2(m, 'bgr8'); STATE['t'] = time.time()

    def on_depth(m):
        with LOCK:
            STATE['depth'] = br.imgmsg_to_cv2(m, 'passthrough')

    def on_info(m):
        with LOCK:
            STATE['info'] = (m.K[0], m.K[4], m.K[2], m.K[5])

    rospy.init_node('camview', anonymous=True, disable_signals=True)
    rospy.Subscriber(a.color, Image, on_color, queue_size=1, buff_size=2**24)
    rospy.Subscriber(a.depth, Image, on_depth, queue_size=1, buff_size=2**24)
    rospy.Subscriber(a.info, CameraInfo, on_info, queue_size=1)

    def frame(kind):
        with LOCK:
            c, d, K = STATE['color'], STATE['depth'], STATE['info']
        if c is None:
            img = np.zeros((360, 640, 3), np.uint8)
            cv2.putText(img, 'waiting for %s' % a.color, (20, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 255), 1)
            return img
        if kind == 'depth':
            if d is None:
                img = np.zeros_like(c)
                cv2.putText(img, 'no depth', (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (80, 80, 255), 2)
                return img
            dm = d.astype(np.float32)
            dm[dm == 0] = np.nan
            lo, hi = 300.0, 3000.0
            n = np.clip((dm - lo) / (hi - lo), 0, 1)
            n = np.nan_to_num(n, nan=1.0)
            return cv2.applyColorMap((255 * (1 - n)).astype(np.uint8), cv2.COLORMAP_TURBO)
        if K is None:
            K = (604.28, 603.67, 323.63, 244.33)     # from last_live_scene.json
        return annotate(c, d, K, a.cam_height, a.pitch)

    class H(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *_):
            pass

        def do_GET(self):
            p = self.path.split('?')[0]
            if p == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
                return
            if p in ('/color.mjpg', '/depth.mjpg'):
                kind = 'depth' if 'depth' in p else 'color'
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                try:
                    while True:
                        ok, buf = cv2.imencode(
                            '.jpg', frame(kind),
                            [int(cv2.IMWRITE_JPEG_QUALITY), a.quality])
                        if ok:
                            self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                             b'Content-Length: %d\r\n\r\n' % len(buf))
                            self.wfile.write(buf.tobytes())
                            self.wfile.write(b'\r\n')
                        time.sleep(0.1)
                except Exception:
                    return
            if p == '/shot.jpg':
                ok, buf = cv2.imencode('.jpg', frame('color'))
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(buf)))
                self.end_headers()
                self.wfile.write(buf.tobytes())
                return
            self.send_error(404)

    srv = ThreadingHTTPServer(('0.0.0.0', a.port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('camview on http://0.0.0.0:%d   (Ctrl-C to stop)' % a.port)
    print('  colour %s' % a.color)
    print('  depth  %s' % a.depth)
    print('  overlay assumes camera %.2f m up, axis %.1f deg down'
          % (a.cam_height, a.pitch))
    try:
        while not rospy.is_shutdown():
            time.sleep(0.5)
            with LOCK:
                age = time.time() - STATE['t'] if STATE['t'] else None
            if age is not None and age > 5:
                print('  !! no colour frame for %.0f s - is the driver running?' % age)
                STATE['t'] = time.time()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
