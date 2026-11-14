#!/usr/bin/env python3
"""Find the tabletop plane in the RealSense depth, and report the servo variable.

    source /opt/ros/noetic/setup.bash
    python3 ~/.scuba/camview/plane_detect.py          # live page on :8093
    python3 ~/.scuba/camview/plane_detect.py --once   # one reading, then exit

ROS 1, because that is where the camera lives on this robot.
READ-ONLY: subscribes to depth + colour, publishes nothing, commands nothing.

WHY NOT PLAIN RANSAC
A tabletop is HORIZONTAL and we know roughly where "down" is from the mount
calibration, so there is no need to search blindly. Projecting every point onto
the down axis turns the problem into a 1-D histogram: the tabletop is a sharp
peak at camera_height - table_height, the floor is a second peak at
camera_height. Picking the peak is O(n), robust to clutter, and cannot latch
onto a wall the way an unconstrained RANSAC can. The plane is then least-
squares refined on that peak's points, and the REFINED normal - not the
assumed one - is what gets reported, so a leaning robot shows up rather than
being silently absorbed.

WHAT IT REPORTS
  drop        perpendicular distance from camera down to the plane.
              tabletop ~0.40 m (1.25 - 0.85), floor ~1.25 m. The gap is the
              whole discriminator - no ambiguity between the two.
  edge_depth  the SERVO VARIABLE: depth of the nearest point on the plane,
              i.e. the table edge. Target 0.500 m for a 0.30 m standoff.
  edge_x      that converted to horizontal distance, sqrt(depth^2 - drop^2).
  rms         plane flatness. 5 mm on a real tabletop; >2 cm means it is not
              one surface.
  cover       fraction of the frame, which grows as the robot approaches.
"""
import argparse
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# --- Thread caps. These must precede EVERY third-party import below.
#
# `import cv2` pulls numpy in with it, so OpenBLAS initialises at the cv2 line;
# a cap placed after it is read too late and does nothing. MEASURED: capping
# after the cv2 import gave 23 threads and 221% CPU - barely better than the
# 30 threads and 224% of no cap at all. Capping before it, 16 threads and 15%.
#
# Those ~209 points were worker threads SPINNING, not computation: the number
# is identical whether a plane is being detected or not. The decompositions
# here are 3x3 covariance SVDs, where threading cannot ever pay for itself.
#
# Uncapped, this one process starved FAST-LIO until map->body ran 2.0 s stale,
# which looked like the map drifting and silently broke every TF-closed servo.
import os as _os
for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
           'NUMEXPR_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS',
           'OPENCV_FOR_THREADS_NUM'):
    _os.environ.setdefault(_v, '1')   # setdefault: an explicit override wins

import cv2
cv2.setNumThreads(1)   # pthreads pool: 8 workers by default here
import numpy as np

S = {'color': None, 'depth': None, 'K': None, 't': 0.0}
LOCK = threading.Lock()
OUT = {'ok': False}


def down_axis(pitch_deg):
    """Unit 'down' in the camera OPTICAL frame (X right, Y down, Z forward)."""
    t = math.radians(pitch_deg)
    return np.array([0.0, math.cos(t), math.sin(t)])


def horiz_axes(pitch_deg):
    """(right, forward) - the horizontal plane, in optical coords.

    Together with down_axis these form a right-handed frame, so a point's
    position on the TABLE's surface is just (P.right, P.forward) and the edge
    becomes a straight line in those two numbers.
    """
    t = math.radians(pitch_deg)
    return (np.array([1.0, 0.0, 0.0]),
            np.array([0.0, -math.sin(t), math.cos(t)]))


def fit_edge(Q, vrow, imgh, pitch_deg, bin_w, min_bins, pct):
    """Direction of the table's NEAR edge, from the plane's inliers.

    BIN LATERALLY, then take the nearest point in each bin. A fixed forward
    threshold does NOT work: for an edge slanted by phi, selecting
    `forward < min + band` keeps only a sliver at one end of the table (at 20
    deg with a 6 cm band, just the outer 16 cm), and a line fitted through
    that sliver reads about HALF the true angle. Taking the nearest point per
    lateral bin follows the edge whatever its slant.

    Bins CLIPPED BY THE FRAME are rejected. The visible tabletop window is only
    ~0.41 m deep, but an edge slanted by 20 deg spans 2*1.2*tan20 = 0.87 m of
    forward distance across the table - twice the window. Where the real edge
    falls outside the frame, the nearest point in that bin is the IMAGE
    BOUNDARY, which runs at angle 0 and drags the fit flat (measured: 20 deg
    read as 14.6). A bin whose nearest point sits on the bottom row is
    therefore dropped rather than fitted.

    PCA on those per-bin minima, not a least-squares slope, because an edge
    seen nearly end-on is vertical in these coordinates.

    Returns (angle_deg, rms, n_bins). The angle is measured from the camera's
    RIGHT axis: 0 means the edge runs straight across the view and the robot
    is square to the table. Positive means the edge's right-hand end is
    FURTHER from the camera.
    """
    r, f = horiz_axes(pitch_deg)
    a = Q @ r
    b = Q @ f
    if len(a) < 20:
        return None, None, 0
    lo, hi = a.min(), a.max()
    nb = max(1, int(math.ceil((hi - lo) / bin_w)))
    idx = np.clip(((a - lo) / bin_w).astype(int), 0, nb - 1)
    pts = []
    clipped = 0
    for k in range(nb):
        m = idx == k
        if int(m.sum()) < 4:
            continue
        # nearer points are LOWER in a downward-looking image, so the bin is
        # clipped when its nearest point sits on the bottom row
        if vrow is not None and int(vrow[m].max()) >= imgh - 3:
            clipped += 1
            continue
        # a low percentile, not the raw minimum, so one noisy point cannot
        # drag the edge toward the camera
        pts.append((lo + (k + 0.5) * bin_w, float(np.percentile(b[m], pct))))
    if len(pts) < min_bins:
        return None, None, len(pts)
    A = np.array(pts)
    C = A - A.mean(0)
    _, _, Vt = np.linalg.svd(C, full_matrices=False)
    d = Vt[0]
    if d[0] < 0:
        d = -d
    ang = math.degrees(math.atan2(d[1], d[0]))
    if ang > 90:
        ang -= 180
    if ang < -90:
        ang += 180
    perp = C @ np.array([-d[1], d[0]])
    rms = float(np.sqrt((perp ** 2).mean()))
    # one robust refit: a bin whose nearest point is an occluding object on the
    # table, or a straggler, should not define the edge
    keep = np.abs(perp) < max(0.015, 2.0 * rms)
    if int(keep.sum()) >= min_bins and int(keep.sum()) < len(pts):
        C2 = A[keep] - A[keep].mean(0)
        _, _, Vt2 = np.linalg.svd(C2, full_matrices=False)
        d = Vt2[0]
        if d[0] < 0:
            d = -d
        ang = math.degrees(math.atan2(d[1], d[0]))
        if ang > 90:
            ang -= 180
        if ang < -90:
            ang += 180
        perp = C2 @ np.array([-d[1], d[0]])
        rms = float(np.sqrt((perp ** 2).mean()))
        pts = [p for p, k_ in zip(pts, keep) if k_]
    return ang, rms, len(pts)


def analyse(depth, K, a):
    fx, fy, cx, cy = K
    H, W = depth.shape
    step = a.step
    d = depth[::step, ::step].astype(np.float32) / 1000.0
    v, u = np.mgrid[0:H:step, 0:W:step]
    ok = (d > a.zmin) & (d < a.zmax)
    if ok.sum() < 500:
        return {'ok': False, 'why': 'not enough valid depth (%d)' % int(ok.sum())}
    Z = d[ok]
    X = (u[ok] - cx) / fx * Z
    Y = (v[ok] - cy) / fy * Z
    P = np.stack([X, Y, Z], 1)
    Vrow = v[ok] * 1.0          # image row of each point, for clip detection

    n0 = down_axis(a.pitch)
    drop = P @ n0                                   # distance below the camera

    lo, hi = a.drop_band
    sel = (drop > lo) & (drop < hi)
    if sel.sum() < 200:
        return {'ok': False, 'why': 'nothing in the %.2f-%.2f m band (%d pts)'
                % (lo, hi, int(sel.sum())), 'drop_hist': hist_of(drop)}
    # the tabletop is the dominant layer inside the band
    h, e = np.histogram(drop[sel], bins=int((hi - lo) / 0.01))
    k = int(np.argmax(h))
    peak = 0.5 * float(e[k] + e[k + 1])
    near = np.abs(drop - peak) < a.slab
    Vnear = Vrow[near]
    if near.sum() < 200:
        return {'ok': False, 'why': 'peak too thin', 'drop_hist': hist_of(drop)}

    # least-squares refine on the peak's points: centroid + smallest eigenvector
    Q = P[near]
    c = Q.mean(0)
    _, _, Vt = np.linalg.svd(Q - c, full_matrices=False)
    n = Vt[-1]
    if n @ n0 < 0:
        n = -n                                      # orient downward
    resid = (Q - c) @ n
    keep = np.abs(resid) < a.inlier
    Q, resid, Vnear = Q[keep], resid[keep], Vnear[keep]
    if len(Q) < 200:
        return {'ok': False, 'why': 'plane too small after refit'}
    rms = float(np.sqrt((resid ** 2).mean()))
    drop_ref = float(c @ n)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ n0)))))
    zq = Q[:, 2]
    edge_depth = float(np.percentile(zq, a.edge_pct))
    # A depth image reports Z - the perpendicular distance to the image plane -
    # NOT the Euclidean range, so this is not Pythagoras. Along the camera's
    # vertical plane:  Z = drop*sin(pitch) + x*cos(pitch).
    # At x=0.30, drop=0.40, pitch=63.6 that gives Z=0.492, not 0.500; the 0.500
    # is the true straight-line distance, which the sensor does not report.
    t = math.radians(a.pitch)
    edge_x = float((edge_depth - drop_ref * math.sin(t)) / math.cos(t))
    cover = float(len(Q)) * step * step / (H * W)
    e_ang, e_rms, e_n = fit_edge(Q, Vnear, H, a.pitch, a.edge_bin,
                                 a.edge_min_bins, a.edge_pct_lo)

    trig = (a.trig_drop[0] <= drop_ref <= a.trig_drop[1]
            and rms < a.trig_rms and cover >= a.trig_cover)
    aligned = (e_ang is not None and abs(e_ang) <= a.align_tol
               and e_rms is not None and e_rms < a.edge_rms)
    return {'ok': True, 'drop': drop_ref, 'rms': rms, 'cover': cover,
            'tilt': tilt, 'edge_depth': edge_depth, 'edge_x': edge_x,
            'edge_angle': e_ang, 'edge_rms': e_rms, 'edge_bins': e_n,
            'aligned': bool(aligned),
            'n': n.tolist(), 'npts': int(len(Q)), 'peak': peak,
            'trigger': bool(trig), 'drop_hist': hist_of(drop),
            'mask_sel': (near, ok, step, H, W)}


def hist_of(drop):
    h, e = np.histogram(drop, bins=np.arange(0.0, 2.0, 0.05))
    return [[float(e[i]), int(h[i])] for i in range(len(h)) if h[i]]


def overlay(color, res, a):
    img = color.copy()
    H, W = img.shape[:2]
    if res.get('ok') and 'mask_sel' in res:
        near, ok, step, h0, w0 = res['mask_sel']
        m = np.zeros(ok.shape, bool)
        m[ok] = near
        big = cv2.resize(m.astype(np.uint8), (W, H), interpolation=cv2.INTER_NEAREST)
        tint = img.copy()
        tint[big > 0] = (0, 200, 0)
        img = cv2.addWeighted(img, 0.6, tint, 0.4, 0)
    bar = 96
    bar = 108
    cv2.rectangle(img, (0, 0), (W, bar), (0, 0, 0), -1)
    if not res.get('ok'):
        cv2.putText(img, 'NO PLANE: %s' % res.get('why', ''), (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 80, 255), 1)
        return img
    col = (0, 255, 0) if res['trigger'] else (0, 170, 255)
    cv2.putText(img, 'edge depth %.3f m   -> x %.3f m   (target %.3f)'
                % (res['edge_depth'], res['edge_x'], a.target),
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62, col, 2)
    err = res['edge_depth'] - a.target
    cv2.putText(img, 'error %+.3f m  -> %s' % (err, move_hint(err, a)),
                (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 1)
    cv2.putText(img, 'drop %.3f m  rms %.1f mm  cover %.0f%%  tilt %.1f deg  %s'
                % (res['drop'], 1000 * res['rms'], 100 * res['cover'],
                   res['tilt'], 'TRIGGER' if res['trigger'] else 'no trigger'),
                (8, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
    ea = res.get('edge_angle')
    if ea is None:
        cv2.putText(img, 'edge: no straight boundary found', (8, 92),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 255), 1)
    else:
        acol = (0, 255, 0) if res.get('aligned') else (0, 170, 255)
        turn = 'SQUARE' if res.get('aligned') else (
            'turn %s %.1f deg' % ('LEFT' if ea > 0 else 'RIGHT', abs(ea)))
        cv2.putText(img, 'edge angle %+.1f deg  rms %.0f mm  %d bins -> %s'
                    % (ea, 1000 * res['edge_rms'], res['edge_bins'], turn),
                    (8, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55, acol, 2)
    return img


def move_hint(err, a):
    """dZ/dx = cos(pitch), so a depth error converts to a ground move by /cos."""
    if abs(err) <= a.tol:
        return 'HOLD'
    move = abs(err) / math.cos(math.radians(a.pitch))
    return ('BACK OFF %.0f cm' % (move * 100)) if err < 0 \
        else ('ADVANCE %.0f cm' % (move * 100))


PAGE = b"""<!doctype html><meta charset=utf-8><title>plane detector</title>
<style>body{background:#111;color:#ddd;font:14px/1.5 system-ui,sans-serif;margin:0;padding:16px}
h1{font-size:16px;margin:0 0 8px}img{max-width:100%;border:1px solid #333}
pre{background:#191919;padding:10px;border:1px solid #333;white-space:pre-wrap}</style>
<h1>tabletop plane detector</h1>
<img src="/view.mjpg"><pre id=j>...</pre>
<script>setInterval(async()=>{try{
const r=await fetch('/stat');document.getElementById('j').textContent=
JSON.stringify(await r.json(),null,1)}catch(e){}},500)</script>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8093)
    ap.add_argument('--depth-topic', default='/camera/aligned_depth_to_color/image_raw')
    ap.add_argument('--color-topic', default='/camera/color/image_raw')
    ap.add_argument('--info-topic', default='/camera/color/camera_info')
    ap.add_argument('--pitch', type=float, default=63.6,
                    help='camera axis below horizontal, from the calibration')
    ap.add_argument('--cam-h', type=float, default=1.25)
    ap.add_argument('--table-h', type=float, default=0.76)
    ap.add_argument('--drop-band', type=float, nargs=2, default=[0.25, 0.70],
                    help='where to look for the tabletop layer')
    ap.add_argument('--slab', type=float, default=0.04)
    ap.add_argument('--inlier', type=float, default=0.02)
    ap.add_argument('--edge-bin', type=float, default=0.04,
                    help='m; lateral bin width for picking the nearest point')
    ap.add_argument('--edge-min-bins', type=int, default=8)
    ap.add_argument('--edge-pct-lo', type=float, default=10.0,
                    help='percentile of forward-distance taken as the edge '
                         'within each bin (not the raw minimum, for noise)')
    ap.add_argument('--edge-rms', type=float, default=0.02,
                    help='m; above this the near boundary is not a straight edge')
    ap.add_argument('--align-tol', type=float, default=3.0,
                    help='deg; |edge angle| under this counts as parallel')
    ap.add_argument('--edge-pct', type=float, default=2.0,
                    help='percentile of depth taken as the edge (2 = robust min)')
    ap.add_argument('--step', type=int, default=3)
    ap.add_argument('--zmin', type=float, default=0.15)
    ap.add_argument('--zmax', type=float, default=3.0)
    # The trigger checks the plane is at TABLE HEIGHT, which is a direct
    # physical test. Measured on synthetic ground truth: across x = 0.10-0.42 m
    # the fitted drop stays 0.391-0.399 and position is accurate to <1.3 cm,
    # but right at the FOV limit (x = 0.44) only a sliver is visible, drop
    # collapses to 0.325 and the estimate is 14.5 cm out. A +-0.05 m window on
    # drop rejects exactly that case and keeps the rest; a cover threshold
    # cannot - at 5% it also threw away the accurate 0.35-0.42 m range.
    ap.add_argument('--trig-drop-tol', type=float, default=0.05)
    ap.add_argument('--trig-drop', type=float, nargs=2, default=None,
                    help='absolute override; default is expected drop +- tol')
    ap.add_argument('--trig-rms', type=float, default=0.02)
    ap.add_argument('--trig-cover', type=float, default=0.015)
    ap.add_argument('--target', type=float, default=-1.0,
                    help='edge depth to servo to; default = sqrt(x^2+drop^2) at 0.30 m')
    ap.add_argument('--tol', type=float, default=0.02)
    ap.add_argument('--once', action='store_true')
    a = ap.parse_args()
    if a.trig_drop is None:
        d0 = a.cam_h - a.table_h
        a.trig_drop = [d0 - a.trig_drop_tol, d0 + a.trig_drop_tol]
    if a.target < 0:
        # the Z the camera will REPORT with the edge 0.30 m away horizontally
        t = math.radians(a.pitch)
        a.target = (a.cam_h - a.table_h) * math.sin(t) + 0.30 * math.cos(t)

    import rospy
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    br = CvBridge()

    def on_c(m):
        with LOCK:
            S['color'] = br.imgmsg_to_cv2(m, 'bgr8')

    def on_d(m):
        with LOCK:
            S['depth'] = br.imgmsg_to_cv2(m, 'passthrough'); S['t'] = time.time()

    def on_i(m):
        with LOCK:
            S['K'] = (m.K[0], m.K[4], m.K[2], m.K[5])

    rospy.init_node('plane_detect', anonymous=True, disable_signals=True)
    rospy.Subscriber(a.color_topic, Image, on_c, queue_size=1, buff_size=2**24)
    rospy.Subscriber(a.depth_topic, Image, on_d, queue_size=1, buff_size=2**24)
    rospy.Subscriber(a.info_topic, CameraInfo, on_i, queue_size=1)

    print('expected drop for a %.2f m table from a %.2f m camera: %.3f m'
          % (a.table_h, a.cam_h, a.cam_h - a.table_h))
    print('trigger: drop %.3f-%.3f, rms < %.0f mm, cover >= %.1f%%'
          % (a.trig_drop[0], a.trig_drop[1], 1000*a.trig_rms, 100*a.trig_cover))
    print('servo target (edge DEPTH at 0.30 m horizontal): %.3f m' % a.target)
    print('  (the straight-line distance there is %.3f m - a depth image '
          'reports Z, not range)' % math.hypot(0.30, a.cam_h - a.table_h))

    t0 = time.time()
    while time.time() - t0 < 20:
        with LOCK:
            have = S['depth'] is not None and S['K'] is not None
        if have:
            break
        time.sleep(0.2)
    with LOCK:
        if S['depth'] is None:
            raise SystemExit('!! no depth on %s - is the driver running?'
                             % a.depth_topic)
        if S['K'] is None:
            S['K'] = (604.28, 603.67, 323.63, 244.33)
            print('  (no camera_info; using the stored intrinsics)')

    def step_once():
        with LOCK:
            d, c, K = S['depth'], S['color'], S['K']
        if d is None:
            return {'ok': False, 'why': 'no depth'}, None
        res = analyse(d, K, a)
        img = overlay(c if c is not None else np.zeros((480, 640, 3), np.uint8),
                      res, a)
        pub = {k: v for k, v in res.items() if k != 'mask_sel'}
        with LOCK:
            OUT.clear(); OUT.update(pub)
        return res, img

    if a.once:
        res, img = step_once()
        cv2.imwrite('/tmp/plane_detect.jpg', img)
        if res.get('ok'):
            print('\n  drop        %.3f m   (expected %.3f)'
                  % (res['drop'], a.cam_h - a.table_h))
            print('  rms         %.1f mm' % (1000 * res['rms']))
            print('  cover       %.0f%% of frame  (%d pts)' % (100*res['cover'], res['npts']))
            print('  tilt vs assumed down: %.1f deg' % res['tilt'])
            print('  EDGE depth  %.3f m   -> horizontal %.3f m'
                  % (res['edge_depth'], res['edge_x']))
            print('  target      %.3f m   error %+.3f m  -> %s'
                  % (a.target, res['edge_depth'] - a.target,
                     move_hint(res['edge_depth'] - a.target, a)))
            print('  TRIGGER     %s' % ('YES' if res['trigger'] else 'no'))
            if res.get('edge_angle') is None:
                print('  edge        no straight boundary (%d pts)' % res['edge_bins'])
            else:
                print('  EDGE angle  %+.1f deg  rms %.0f mm  %d pts   %s'
                      % (res['edge_angle'], 1000 * res['edge_rms'],
                         res['edge_bins'],
                         'SQUARE' if res['aligned'] else
                         ('turn %s %.1f deg' % ('LEFT' if res['edge_angle'] > 0
                                                else 'RIGHT',
                                                abs(res['edge_angle'])))))
        else:
            print('\n  NO PLANE: %s' % res.get('why'))
            for lo, n in res.get('drop_hist', [])[:14]:
                print('     drop %4.2f m  %s %d' % (lo, '#'*int(min(40, n/40)), n))
        print('\n  wrote /tmp/plane_detect.jpg')
        return

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
                self.end_headers(); self.wfile.write(PAGE); return
            if p == '/stat':
                import json
                b = json.dumps(OUT, default=float).encode()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(b)))
                self.end_headers(); self.wfile.write(b); return
            if p == '/view.mjpg':
                self.send_response(200)
                self.send_header('Content-Type',
                                 'multipart/x-mixed-replace; boundary=f')
                self.end_headers()
                try:
                    while True:
                        _, img = step_once()
                        ok, buf = cv2.imencode('.jpg', img,
                                               [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                        if ok:
                            self.wfile.write(b'--f\r\nContent-Type: image/jpeg\r\n'
                                             b'Content-Length: %d\r\n\r\n' % len(buf))
                            self.wfile.write(buf.tobytes()); self.wfile.write(b'\r\n')
                        time.sleep(0.15)
                except Exception:
                    return
            self.send_error(404)

    srv = ThreadingHTTPServer(('0.0.0.0', a.port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print('plane detector on http://0.0.0.0:%d   (Ctrl-C to stop)' % a.port)
    last = 0.0
    try:
        while not rospy.is_shutdown():
            res, _ = step_once()
            if time.time() - last > 1.0:
                last = time.time()
                if res.get('ok'):
                    ea = res.get('edge_angle')
                    print('  drop %.3f  rms %4.1fmm  cover %3.0f%%  edge %.3f m '
                          '(x %.3f)  err %+.3f  ang %s  %s'
                          % (res['drop'], 1000*res['rms'], 100*res['cover'],
                             res['edge_depth'], res['edge_x'],
                             res['edge_depth'] - a.target,
                             '  --  ' if ea is None else '%+5.1f' % ea,
                             ('TRIGGER' if res['trigger'] else '') +
                             (' SQUARE' if res.get('aligned') else '')))
                else:
                    print('  no plane: %s' % res.get('why'))
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
