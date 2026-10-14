"""
The hall camera's side of VATEL: grab CCTV frames, find the tables, serve them to the planner.

    python cctv_bridge.py --demo                          # a simulated hall, no camera
    python cctv_bridge.py --source rtsp://user:pw@10.0.0.20/stream1
    python cctv_bridge.py --source 0                      # a USB camera
    python cctv_bridge.py --source hall.mp4               # a recording, looped

Then open the planner at http://localhost:8765/ -- this serves it too, so the page and the
camera share an origin -- and in Settings, "Calibrate on the live view" once; close
Settings and press "Go live". The tables the camera sees appear on the plan where they
stand, and follow it. (The planner opened straight from disk works as well; it reads the
camera across origins, which this allows.)

It serves, with CORS open so a page opened straight from disk can read it:

    GET /status       source, frame size, tables in view, and the calibration if known
    GET /detections   banquet-detections/0.1 in pixels, with a `seq` that changes per frame
    GET /frame.jpg    the latest frame, for calibrating and for the plan's camera view

WHERE THE MAPPING HAPPENS
    Not here. This sends pixels -- rim outlines, not centres -- and the planner maps them
    onto the floor through four points clicked once on the live view, then corrects for
    the 0.762 m the tabletop stands above that floor. So the camera can be mounted in a
    corner, re-aimed, or swapped without touching this file.

THE DETECTOR
    --demo knows exactly where its tables are, so it reports them (rims projected from
    the simulated hall, plus a pixel of noise). It exercises everything downstream: the
    calibration, the rim fit, the parallax correction, the live updates on the plan.

    On a real camera, `hough` is the placeholder from detect_tables.py and will find
    chair backs and light fittings. Replace detect_frame() with a segmentation model --
    Grounded-SAM zero-shot to start, YOLO-seg once there are labelled frames -- and send
    each mask's outline as outline_px. See the README's camera section.
"""

import argparse
import datetime
import json
import math
import random
import sys
import threading
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

try:
    import cv2
except ImportError:                       # pragma: no cover - told to the user, not hidden
    sys.exit("cctv_bridge.py needs OpenCV:  pip install opencv-python numpy")

SCHEMA = "banquet-detections/0.1"
# The planner page, served at /: beside this script, or where the VATEL repo publishes it.
HERE = Path(__file__).resolve().parent
PAGE = next((p for p in (HERE / "index.html", HERE.parent / "docs" / "planner" / "index.html")
             if p.exists()), HERE / "index.html")
TABLE_H = 0.762            # the standard banquet table height, as in the planner
FT = 0.3048


# ====================================================================== simulated hall
class DemoHall:
    """A banquet hall seen by a CCTV camera high in one corner, being set up.

    Frame: x east, y south from the room's north-west corner, z up -- the planner's own.
    Tables are rolled in one at a time until the room is full, then one is nudged now
    and then, so the plan has something to follow."""

    def __init__(self, room=(20.0, 21.0), size=(1280, 720), cam=(0.25, 0.25, 4.5),
                 target=(11.5, 12.5), hfov_deg=100.0, period=4.0, static=False, seed=7):
        self.W, self.H = room
        self.w, self.h = size
        self.C = np.array(cam, float)
        self.period, self.static = period, static
        self.rng = random.Random(seed)
        self.t0 = time.time()

        f = np.array([target[0], target[1], 0.0]) - self.C
        f /= np.linalg.norm(f)
        x = np.cross([0.0, 0.0, 1.0], f)
        x /= np.linalg.norm(x)
        y = np.cross(f, x)                      # image down
        self.R = np.stack([x, -y, f]) if y[2] > 0 else np.stack([x, y, f])
        self.fpx = (self.w / 2) / math.tan(math.radians(hfov_deg) / 2)

        self.stage = (self.W / 2, 1.9, 8.0, 3.0)        # centre x, y, width, depth
        self.dance = (self.W / 2, 6.6, 6.0, 4.0)
        self.tables = self._layout()
        self.order = list(range(len(self.tables)))
        self.rng.shuffle(self.order)
        self.start_n = max(3, int(len(self.tables) * 0.55))
        self.offsets = {}
        self.background = self._background()

    # -------------------------------------------------------------- geometry
    def _layout(self):
        """Six-foot rounds on a staggered grid, clear of the stage and the dance floor."""
        d = 6 * FT
        pitch = d + 1.55
        out = []
        y, row = 5.0, 0
        while y <= self.H - 1.4:
            xs = np.arange(1.5 + (pitch / 2 if row % 2 else 0), self.W - 1.3, pitch)
            for x in xs:
                if self._clear(x, y, d / 2 + 0.6):
                    out.append([float(x), float(y), d])
            y += pitch * 0.87
            row += 1
        return out

    def _clear(self, x, y, r):
        for cx, cy, w, h in (self.stage, self.dance):
            if abs(x - cx) < w / 2 + r and abs(y - cy) < h / 2 + r:
                return False
        return True

    def project(self, P):
        """World points (N,3) -> pixels (N,2) and depth (N,)."""
        q = (np.asarray(P, float) - self.C) @ self.R.T
        z = q[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.w / 2 + self.fpx * q[:, 0] / z
            v = self.h / 2 + self.fpx * q[:, 1] / z
        return np.stack([u, v], 1), z

    def floor_point(self, u, v):
        ray = np.array([(u - self.w / 2) / self.fpx, (v - self.h / 2) / self.fpx, 1.0]) @ self.R
        t = -self.C[2] / ray[2]
        return self.C + t * ray

    def visible(self, pts, margin=12):
        uv, z = self.project(pts)
        return bool(np.all(z > 0.2) and np.all((uv[:, 0] > margin) & (uv[:, 0] < self.w - margin) &
                                               (uv[:, 1] > margin) & (uv[:, 1] < self.h - margin)))

    # -------------------------------------------------------------- drawing
    def _background(self):
        """Floor, walls and ceiling, ray-cast once: the room does not move."""
        u, v = np.meshgrid(np.arange(self.w) + 0.5, np.arange(self.h) + 0.5)
        rays = np.stack([(u - self.w / 2) / self.fpx, (v - self.h / 2) / self.fpx,
                         np.ones_like(u)], -1) @ self.R
        img = np.zeros((self.h, self.w, 3), np.float32)
        img[:] = (62, 66, 70)                                     # ceiling
        # walls: the nearest vertical plane each ray meets, up to 4.2 m
        best = np.full(u.shape, np.inf)
        for axis, val in ((0, 0.0), (0, self.W), (1, 0.0), (1, self.H)):
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (val - self.C[axis]) / rays[..., axis]
            p = self.C + t[..., None] * rays
            other = 1 - axis
            ok = (t > 0) & (p[..., 2] >= 0) & (p[..., 2] <= 4.2) & \
                 (p[..., other] >= -0.01) & (p[..., other] <= (self.H if other else self.W) + 0.01)
            take = ok & (t < best)
            best[take] = t[take]
            shade = 150 + 25 * axis
            img[take] = (shade - 12, shade - 4, shade)
            stripe = take & (np.abs(p[..., 2] - 1.0) < 0.04)     # a dado rail
            img[stripe] = (110, 118, 124)
        # floor: carpet with a faint metre pattern, where the ray lands inside the room
        with np.errstate(divide="ignore", invalid="ignore"):
            t = -self.C[2] / rays[..., 2]
        p = self.C + t[..., None] * rays
        on = (t > 0) & (p[..., 0] >= 0) & (p[..., 0] <= self.W) & (p[..., 1] >= 0) & \
             (p[..., 1] <= self.H) & (t < best)
        carpet = np.array((48, 58, 120), np.float32)              # BGR: a red banquet carpet
        pat = (((np.floor(p[..., 0] * 2) + np.floor(p[..., 1] * 2)) % 2) * 9)[..., None]
        img[on] = (carpet + pat)[on]
        for (cx, cy, w, h), col in ((self.stage, (70, 92, 112)), (self.dance, (88, 132, 168))):
            m = on & (np.abs(p[..., 0] - cx) <= w / 2) & (np.abs(p[..., 1] - cy) <= h / 2)
            img[m] = col
        # light falls off away from the camera
        dist = np.clip(np.where(np.isfinite(best), best, 30), 0, 40)
        dist = np.where(on, np.minimum(t, dist), dist)
        img *= np.clip(1.15 - dist[..., None] / 45, 0.55, 1.15)
        return np.clip(img, 0, 255).astype(np.uint8)

    def _present(self, now):
        el = now - self.t0
        n = len(self.tables) if self.static else min(len(self.tables),
                                                     self.start_n + int(el / self.period))
        out = []
        for i in self.order[:n]:
            x, y, d = self.tables[i]
            ox, oy = self.offsets.get(i, (0.0, 0.0))
            out.append((i, x + ox, y + oy, d))
        # once the room is full, staff move a table now and then
        if not self.static and n == len(self.tables):
            k = int(el / (self.period * 2))
            if k != getattr(self, "_nudged", None):
                self._nudged = k
                i = self.rng.choice(self.order)
                self.offsets[i] = (self.rng.uniform(-0.35, 0.35), self.rng.uniform(-0.35, 0.35))
        return out

    def _rim(self, x, y, d, z, n=64):
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        return np.stack([x + d / 2 * np.cos(a), y + d / 2 * np.sin(a), np.full(n, z)], 1)

    def render(self, now=None):
        now = time.time() if now is None else now
        img = self.background.copy()
        present = self._present(now)
        # far first, so nearer tables hide the ones behind them
        present.sort(key=lambda t: -math.hypot(t[1] - self.C[0], t[2] - self.C[1]))
        dets = []
        for i, x, y, d in present:
            top, z = self.project(self._rim(x, y, d, TABLE_H))
            if np.any(z < 0.2):
                continue
            # chairs: ten, each a seat and a back, set round the table
            for k in range(10):
                a = 2 * np.pi * k / 10
                ca, sa = math.cos(a), math.sin(a)
                cx, cy = x + (d / 2 + 0.32) * ca, y + (d / 2 + 0.32) * sa
                sq = np.array([[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]])
                seat = np.array([[cx + u * ca - v * sa, cy + u * sa + v * ca, 0.46] for u, v in sq])
                bx, by = cx + 0.2 * ca, cy + 0.2 * sa
                back = np.array([[bx - 0.2 * sa, by + 0.2 * ca, 0.46], [bx + 0.2 * sa, by - 0.2 * ca, 0.46],
                                 [bx + 0.2 * sa, by - 0.2 * ca, 0.95], [bx - 0.2 * sa, by + 0.2 * ca, 0.95]])
                (ps, zs), (pb, zb) = self.project(seat), self.project(back)
                if np.all(zs > 0.2) and np.all(zb > 0.2):
                    cv2.fillConvexPoly(img, ps.astype(np.int32), (58, 60, 70), cv2.LINE_AA)
                    cv2.fillConvexPoly(img, pb.astype(np.int32), (40, 42, 50), cv2.LINE_AA)
            base, _ = self.project(self._rim(x, y, d * 0.98, 0.02))
            skirt = cv2.convexHull(np.vstack([top, base]).astype(np.int32))
            cv2.fillConvexPoly(img, skirt, (196, 200, 204), cv2.LINE_AA)
            cv2.fillConvexPoly(img, top.astype(np.int32), (238, 240, 242), cv2.LINE_AA)
            cv2.polylines(img, [top.astype(np.int32)], True, (170, 176, 182), 1, cv2.LINE_AA)
            centre, _ = self.project(np.array([[x, y, TABLE_H + 0.05]]))
            cv2.circle(img, tuple(centre[0].astype(int)), max(2, int(0.1 * self.fpx / z.mean())),
                       (120, 150, 200), -1, cv2.LINE_AA)          # a centrepiece
            inside = np.mean((top[:, 0] > 0) & (top[:, 0] < self.w) &
                             (top[:, 1] > 0) & (top[:, 1] < self.h))
            if inside < 0.6:
                continue
            rim48, _ = self.project(self._rim(x, y, d, TABLE_H, n=48))
            rim48 = rim48 + np.random.normal(0, 0.8, (48, 2))
            span = top.max(0) - top.min(0)
            dets.append({
                "id": "t%02d" % (i + 1), "shape": "round",
                "centre_px": [round(float(v), 1) for v in top.mean(0)],
                "diameter_px": round(float(span.max()), 1),
                "outline_px": [[round(float(a), 1), round(float(b), 1)] for a, b in rim48],
                "confidence": round(0.9 + 0.08 * self.rng.random(), 2),
            })
        # CCTV furniture: noise and a burned-in clock
        img = np.clip(img.astype(np.int16) + np.random.normal(0, 3, img.shape), 0, 255).astype(np.uint8)
        stamp = "CAM 01  HALL NW  " + datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        (tw, th), _ = cv2.getTextSize(stamp, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(img, (10, 10), (22 + tw, 20 + th), (0, 0, 0), -1)
        cv2.putText(img, stamp, (16, 15 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1, cv2.LINE_AA)
        truth = [{"id": "t%02d" % (i + 1), "centre_m": [round(x, 3), round(y, 3)], "diameter_m": round(d, 3)}
                 for i, x, y, d in present]
        return img, dets, truth

    def calibration(self):
        """Four floor points the camera can see, with their plan positions. What a person
        would click on the live view; the simulation simply knows them."""
        W, H = self.W, self.H
        cands = [(W, H), (W, 0.0), (0.0, H), (W - 2, 2.0), (2.0, H - 2), (4.0, 4.0), (6.0, 3.0),
                 (3.0, 6.0), (W / 2, H / 2), (W - 3, H / 2), (W / 2, H - 3)]
        pts = []
        for x, y in cands:
            if self.visible(np.array([[x, y, 0.0]]), margin=25):
                pts.append((x, y))
        best, area = None, 0
        for a in range(len(pts)):
            for b in range(a + 1, len(pts)):
                for c in range(b + 1, len(pts)):
                    for e in range(c + 1, len(pts)):
                        quad = np.array([pts[a], pts[b], pts[c], pts[e]], np.float32)
                        hull = cv2.convexHull(quad)
                        if len(hull) < 4:
                            continue
                        ar = cv2.contourArea(hull)
                        if ar > area:
                            best, area = [tuple(p) for p in hull.reshape(-1, 2)], ar
        if not best:
            return None
        uv, _ = self.project(np.array([[x, y, 0.0] for x, y in best]))
        return {"frame": [self.w, self.h],
                "points": [{"px": [round(float(u), 1), round(float(v), 1)], "m": [float(x), float(y)]}
                           for (x, y), (u, v) in zip(best, uv)],
                "camera_height_m": float(self.C[2]),
                "nadir_m": [float(self.C[0]), float(self.C[1])]}


# ====================================================================== a real camera
class Capture:
    """Any source OpenCV opens: an RTSP/HTTP stream, a device index, a file (looped)."""

    def __init__(self, source):
        self.source = int(source) if str(source).isdigit() else source
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            sys.exit("cannot open camera source %r" % (source,))
        self.lock = threading.Lock()
        self.frame = None
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        # Read continuously: an RTSP stream buffers, and detecting on a frame from ten
        # seconds ago is worse than detecting less often.
        while True:
            ok, f = self.cap.read()
            if not ok:
                if isinstance(self.source, str) and not self.source.startswith(("rtsp", "http")):
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)      # a file: loop it
                time.sleep(0.05)
                continue
            with self.lock:
                self.frame = f

    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def detect_frame(img, detector):
    """Tables in one BGR frame, as banquet-detections entries in pixels.

    The seam for the real segmenter: return one dict per table with outline_px (the mask's
    rim, ~48 points), centre_px, diameter_px and a confidence."""
    if detector == "hough":
        import detect_tables
        return detect_tables.detect_image(img)["tables"]
    raise SystemExit("unknown detector %r" % detector)


# ====================================================================== the service
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.seq = 0
        self.jpg = b""
        self.doc = {"schema": SCHEMA, "units": "pixels", "seq": 0, "tables": []}
        self.truth = []


def run_worker(state, args, hall, cap):
    while True:
        t0 = time.time()
        if hall:
            img, dets, truth = hall.render()
        else:
            img = cap.read()
            if img is None:
                time.sleep(0.2)
                continue
            dets, truth = detect_frame(img, args.detector), []
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 82])
        with state.lock:
            state.seq += 1
            state.jpg = jpg.tobytes() if ok else state.jpg
            state.truth = truth
            state.doc = {
                "schema": SCHEMA, "source": args.name, "seq": state.seq,
                "captured_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
                "units": "pixels",
                "frame": {"width": int(img.shape[1]), "height": int(img.shape[0])},
                "tables": dets,
            }
        time.sleep(max(0.0, args.every - (time.time() - t0)))


def make_handler(state, hall, args):
    class Handler(BaseHTTPRequestHandler):
        def _head(self, code, ctype, n):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(n))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.end_headers()

        def _json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self._head(code, "application/json", len(body))
            self.wfile.write(body)

        def do_OPTIONS(self):
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Allow-Private-Network", "true")
            self.end_headers()

        def do_GET(self):
            path = self.path.split("?")[0]
            with state.lock:
                doc, jpg, truth = state.doc, state.jpg, state.truth
            if path == "/detections":
                self._json(doc)
            elif path == "/frame.jpg":
                if not jpg:
                    return self._json({"error": "no frame yet"}, 503)
                self._head(200, "image/jpeg", len(jpg))
                self.wfile.write(jpg)
            elif path in ("/", "/index.html"):
                page = PAGE.read_bytes() if PAGE.exists() else b"index.html not found"
                self._head(200, "text/html; charset=utf-8", len(page))
                self.wfile.write(page)
            elif path == "/status":
                st = {"service": "vatel-cctv-bridge", "source": args.name, "seq": doc["seq"],
                      "frame": doc.get("frame"), "tables_seen": len(doc["tables"]),
                      "detector": "simulated hall (exact)" if hall else args.detector,
                      "calibration": hall.calibration() if hall else None}
                if hall:
                    st["demo_truth"] = truth
                self._json(st)
            else:
                self._json({"error": "not found",
                            "try": ["/status", "/detections", "/frame.jpg"]}, 404)

        def log_message(self, fmt, *a):
            if args.verbose:
                sys.stderr.write("%s  %s\n" % (self.address_string(), fmt % a))

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("--source", help="rtsp://..., a device index, or a video file")
    ap.add_argument("--demo", action="store_true", help="a simulated hall instead of a camera")
    ap.add_argument("--detector", default="hough", help="detector for a real source (default: hough)")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to serve other machines")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between detections")
    ap.add_argument("--name", default=None, help="what to call this camera")
    ap.add_argument("--room", nargs=2, type=float, default=(20.0, 21.0), metavar=("W", "H"),
                    help="demo: room size in metres (default: the 20 x 21 m Grand Ballroom preset)")
    ap.add_argument("--period", type=float, default=4.0, help="demo: seconds between tables arriving")
    ap.add_argument("--static", action="store_true", help="demo: a finished room that does not change")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv)
    if not args.demo and args.source is None:
        ap.error("give --source, or --demo")

    hall = DemoHall(room=tuple(args.room), period=args.period, static=args.static) if args.demo else None
    cap = None if args.demo else Capture(args.source)
    args.name = args.name or ("simulated hall, NW corner camera" if args.demo else str(args.source))

    state = State()
    threading.Thread(target=run_worker, args=(state, args, hall, cap), daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), make_handler(state, hall, args))
    print("VATEL camera bridge: %s -> http://%s:%d  (status, detections, frame.jpg)"
          % (args.name, args.host, args.port), flush=True)
    print("planner: http://%s:%d/" % ("localhost" if args.host in ("127.0.0.1", "0.0.0.0") else args.host,
                                      args.port), flush=True)
    if hall:
        print("demo: %d tables, %d at the start, one more every %.0f s"
              % (len(hall.tables), len(hall.tables) if args.static else hall.start_n, args.period),
              flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
