"""A simple plate stack for offline tests of the stages' logic (not physics), used with pk.robot_io.MockBackend."""
import json
import socket
import threading
import time

import numpy as np

from pk import config
from pk.model import PINKY_END

K = 3000.0          # N/m
C = 60.0            # N s/m, contact damping


# proximity of the little finger above porcelain on the stack, R1 (lowest point's height -> raw counts)
PROX_GAP_MM = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.5, 10.0]
PROX_COUNTS = [96700, 96700, 90000, 88400, 76000, 44000, 16300, 6500, 4700, 3300, 2500, 200, 0]


class MockStack(object):
    def __init__(self, center=(0.365, 0.062), rim_z=0.060, R=0.1275, slide_N=4.0, pitch=0.0068, finger_r=0.006, profile=False,
                 prox_noise=0.03, seed=1):
        """profile: the top plate's top surface as the measured cross-section (pk.plate.TASK: the slope rising to the
        rim) instead of flat near the edge with a 12 mm step inside."""
        self.c = np.array(center, float)
        self.c0 = self.c.copy()
        self.rim_z, self.R, self.slide_N, self.pitch, self.fr = rim_z, R, slide_N, pitch, finger_r
        self.profile = profile
        self.prox_noise = prox_noise
        self.rng = np.random.RandomState(seed)
        self.log = []
        self.lock = threading.Lock()

    def surface(self, r, top):
        """Height of the top plate's top surface at radius r (r < R), its rim's top at `top`."""
        from pk.plate import TASK
        if self.profile:
            return top - TASK.H + TASK.z_top(min(r, TASK.R))
        return top if r > self.R - 0.03 else top - 0.012

    def proximity(self, p):
        """Raw proximity counts of a fingertip point p (the little finger's lowest point): its distance to the top
        plate (straight down onto the top surface over it; to the rim's corner beyond the edge), through the R1's curve."""
        with self.lock:
            r = float(np.linalg.norm(p[:2] - self.c))
            slid = float(np.linalg.norm(self.c - self.c0)) * 1000
        top = self.rim_z - float(np.interp(slid, [0.0, 8.0, 45.0, 80.0], [0.0, 0.0, 4.5, 4.6])) / 1000
        if r <= self.R:
            gap = p[2] - self.surface(r, top)
        else:
            gap = float(np.hypot(r - self.R, max(0.0, p[2] - top)))
        n = float(np.interp(1000 * max(gap, 0.0), PROX_GAP_MM, PROX_COUNTS))
        return max(0.0, n * (1.0 + self.prox_noise * self.rng.randn()))

    def contact(self, p, v):
        F = np.zeros(3)
        with self.lock:
            d = p[:2] - self.c
            r = float(np.linalg.norm(d))
            u = d / max(r, 1e-9)
            slid = float(np.linalg.norm(self.c - self.c0)) * 1000
            drop = float(np.interp(slid, [0.0, 8.0, 45.0, 80.0], [0.0, 0.0, 4.5, 4.6])) / 1000   # the pushed edge sinks
            top, under = self.rim_z - drop, self.rim_z - 0.0055 - drop
            in_band = under - 0.001 < p[2] < top
            if in_band and self.R - 0.003 < r < self.R + self.fr:             # at the edge face: a wall that slides
                pen = self.R + self.fr - r
                if K * pen > self.slide_N:                                  # push beats friction: the plate moves on
                    move = pen - self.slide_N / K
                    self.c -= u * move
                    pen = self.slide_N / K
                F[:2] += u * max(0.0, K * pen - C * float(v[:2] @ u))
            elif r < self.R and p[2] < top:                                 # on (or into) the top plate's surface
                pen = self.surface(r, top) - p[2]
                if pen > 0:
                    F[2] += max(0.0, K * pen - C * float(v[2]))
            # the plate below (fixed, at the original place)
            d0 = p[:2] - self.c0
            r0 = float(np.linalg.norm(d0))
            if r0 < self.R + self.fr and p[2] < self.rim_z - self.pitch:
                F[2] += max(0.0, K * (self.rim_z - self.pitch - p[2]) - C * float(v[2]))
            self.log.append((time.time(), p.copy(), F.copy(), self.c.copy()))
        return F


class FakePerception(object):
    """Plays perception.py's part: plate estimates to the server, answers `locate` / `head_lock`."""

    def __init__(self, stack, port=config.SERVER_PORT, noise_mm=0.5, rim_err_mm=0.0, xy_err_mm=(0.0, 0.0)):
        """rim_err_mm / xy_err_mm: the camera's error against the arm (as the R1's x +20, y -42 mm)."""
        self.stack = stack
        self.addr = (config.HOST, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((config.HOST, 0))
        self.sock.settimeout(0.05)
        self.noise = noise_mm / 1000
        self.rim_err = rim_err_mm / 1000
        self.xy_err = np.asarray(xy_err_mm, float) / 1000
        self.stop = False
        self.located = None
        self.last_req = None
        self.start_c = None
        self.extra = None                      # fn() -> more fields for the plate message (a test marker)
        threading.Thread(target=self.loop, daemon=True).start()

    def locate(self):
        with self.stack.lock:
            c = self.stack.c.copy()
        self.start_c = c.copy()
        c = c + self.xy_err
        self.located = dict(center=[float(c[0]), float(c[1]), self.stack.rim_z + self.rim_err], top_height=0.034,
                            table_z=self.stack.rim_z - 0.034, diameter=0.255, n_plates=getattr(self.stack, "n_plates", 3), t=time.time())

    def loop(self):
        self.sock.sendto(json.dumps(dict(op="hello", role="head", name="fake perception")).encode(), self.addr)
        self.locate()
        seq = 0
        while not self.stop:
            try:
                data, _ = self.sock.recvfrom(65536)
                m = json.loads(data.decode())
                if m.get("op") == "state":
                    req = m.get("req")
                    if req and req != self.last_req:
                        self.last_req = req
                        if req.get("what") == "locate":
                            time.sleep(0.3)
                            self.locate()
            except (socket.timeout, ValueError):
                pass
            seq += 1
            with self.stack.lock:
                c = self.stack.c.copy()
            c = c + np.random.normal(0, self.noise, 2)
            moved = c - self.start_c
            plate = dict(seq=seq, t=time.time(), status="tracking", ok=True, located=self.located,
                         center=[float(self.located["center"][0] + moved[0]), float(self.located["center"][1] + moved[1]),
                                 self.located["center"][2]], moved_mm=float(1000 * np.linalg.norm(moved)), n=60, coverage=0.8, rms_mm=1.0)
            if self.extra:
                plate.update(self.extra())
            self.sock.sendto(json.dumps(dict(op="plate", plate=plate)).encode(), self.addr)
            self.sock.sendto(json.dumps(dict(op="head", target=[0.55, 0.3])).encode(), self.addr)
            time.sleep(1 / 30.0)
