"""perception.py's main loop with a fake camera, head and tracker.

    python3 tests/test_perception_loop.py
"""
import contextlib
import os
import sys
import tempfile
import threading
import time
import urllib.request

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import perception as P  # noqa: E402

SAG = np.array([0.045, 0.003])                  # measured - commanded head angles at rest, the robot's
CENTRE = [0.371, 0.104, 0.071]
OK = []
LINK = []


def check(cond, what):
    OK.append((bool(cond), what))
    print(("PASS " if cond else "FAIL ") + what, flush=True)


class Stop(Exception):
    pass


class FakeLink(object):
    """The server as perception sees it: the head command ramps to the target at 0.35 rad/s; the head rests SAG off."""
    can_aim = True

    def __init__(self, *a, **k):
        self.cmd = np.array([0.012, 0.004])
        self.target = self.cmd.copy()
        self.extra = np.zeros(2)                # a real head motion, added by the camera below
        self.t = time.time()
        self.sent = []
        self.heads = []                         # (time, target) of every head command
        self.req = None
        LINK.append(self)

    def state(self):
        now = time.time()
        self.cmd += np.clip(self.target - self.cmd, -0.35 * (now - self.t), 0.35 * (now - self.t))
        self.t = now
        return dict(head=list(self.cmd + SAG + self.extra), head_cmd=list(self.cmd), req=self.req)

    def head(self, target):
        self.target = np.array(target, float)
        self.heads.append((time.time(), self.target.copy()))

    def publish(self, p):
        self.sent.append((time.time(), p))

    def busy(self, s):
        return contextlib.suppress()


class FakeKin(object):
    correction = None

    def __init__(self, *a, **kw):
        pass

    def angles(self, st):
        return {"head_pitch_joint": st["head"][0], "head_yaw_joint": st["head"][1]}

    def aim(self, look, ang):
        return np.array([0.575, 0.161]), False

    def T_po(self, ang):
        T = np.eye(4)
        T[:3, 3] = np.array(CENTRE) - np.array([0.0, 0.0, 0.5])      # the plate straight ahead, mid-picture
        return T

    def spheres(self, st):
        return []


class FakeCam(object):
    intr_d = dict(fx=600.0, fy=600.0, cx=424.0, cy=240.0, width=848, height=480)
    intr_c = intr_d
    scale = 0.001
    T_cd = np.eye(4)
    N = 120                                     # frames, then the run ends
    LOCK_AT = LOST_AT = None                    # frame numbers: a test locks the head / the plate is lost

    def __init__(self):
        self.n = 0

    def frames(self):
        self.n += 1
        FakeTracker.frame = self.n
        if self.n > self.N:
            raise Stop()
        if 60 <= self.n < 70 and self.LOST_AT is None:
            LINK[-1].extra += np.array([0.01, 0.0])                  # the head really moves for 10 frames
        if self.LOCK_AT is not None and self.n == self.LOCK_AT:
            LINK[-1].req = dict(seq=1, what="head_lock", args={"on": True}, t=time.time())
        time.sleep(0.02)
        return None, np.zeros((480, 848), np.uint16), np.full((480, 848, 3), 90, np.uint8), time.time()


class FakeTracker(object):
    frame = 0
    r_band, R, band_r = 0.125, 0.1275, 0.018

    def __init__(self, c, z, table_z, plate_r=0.1275):
        self.start = np.array(c[:2], float)
        self.c, self.rim_z = self.start.copy(), float(z)
        a = np.linspace(-np.pi, np.pi, 72, endpoint=False)
        ring = np.c_[np.cos(a), np.sin(a)]
        self.dbg = dict(Q=np.c_[self.c + 0.11 * ring, np.full(72, z)], E=self.c + self.r_band * ring, inl=np.arange(72) % 9 != 0)

    def init(self, *a):
        lost = FakeCam.LOST_AT is not None and FakeTracker.frame >= FakeCam.LOST_AT
        return dict(ok=not lost, center=list(CENTRE), n=71, coverage=0.99, rms_mm=1.2, why="only 3 rim points" if lost else None)

    update = init


LOCATES = []


def fake_locate(cam, kin, link, a):
    LOCATES.append(time.time())
    st = link.state()
    loc = dict(center=list(CENTRE), top_height=0.0346, table_z=0.0367, diameter=0.254, n_plates=3, t=time.time(),
               head=st["head"], table_above_floor=0.751, top_above_floor=0.785)
    return loc, "", (None, np.zeros((480, 848)), FakeCam.intr_d, kin.angles(st))


def watch(port, got):
    """A browser: the page, then pictures from the stream."""
    time.sleep(2.5)
    try:
        got["page"] = urllib.request.urlopen("http://127.0.0.1:%d/" % port, timeout=3).read()
        r = urllib.request.urlopen("http://127.0.0.1:%d/stream.mjpg" % port, timeout=5)
        got["type"] = r.headers.get("Content-Type")
        buf = b""
        while buf.count(JPEG_START) < 3 and len(buf) < 5000000:
            buf += r.read(4096)
        got["stream"] = buf
    except Exception as ex:                                  # noqa: BLE001
        got["error"] = repr(ex)


JPEG_START, JPEG_END = bytes([0xFF, 0xD8]), bytes([0xFF, 0xD9])


def main():
    P.ServerLink, P.Kin, P.Camera, P.RimTracker, P.locate = FakeLink, FakeKin, FakeCam, FakeTracker, fake_locate
    P.OUT = tempfile.mkdtemp(prefix="perception_loop_")
    port = 8097
    sys.argv = ["perception.py", "--expect", "0.36", "0.06", "--view-port", str(port)]
    got = {}
    threading.Thread(target=watch, args=(port, got), daemon=True).start()
    t0 = time.time()
    try:
        P.main()
    except Stop:
        pass
    sent = LINK[0].sent
    t_loc = next((t for t, p in sent if "locating" in str(p.get("why"))), None)
    check(t_loc is not None and t_loc - t0 < 4.0, "aiming ends once the head stops, though it rests 0.045 rad low (%.1f s; "
          "the time-out is 6 s)" % ((t_loc or 0) - t0))
    tracking = [p for _, p in sent if p.get("status") == "tracking"]
    with_c = [p for p in tracking if p.get("center")]
    moving = [p for p in tracking if p.get("why") == "head moving"]
    check(len(with_c) > 80, "the plate reaches the server while tracking: %d of %d messages carry its centre" % (len(with_c), len(tracking)))
    check(5 <= len(moving) <= 14, "frames taken while the head really moves are skipped (%d)" % len(moving))
    last = [p for p in tracking[-20:] if p.get("center")]
    check(len(last) >= 18, "after that motion the plate comes back (%d of the last 20)" % len(last))
    st = got.get("stream", b"")
    n_jpg = st.count(JPEG_START)
    check(b"stream.mjpg" in got.get("page", b"") and "multipart/x-mixed-replace" in str(got.get("type")) and n_jpg >= 3,
          "live view: the page and %d pictures streamed%s" % (n_jpg, (" (" + got["error"] + ")") if "error" in got else ""))
    if n_jpg >= 1:
        import cv2
        i0 = st.index(JPEG_START)
        i1 = st.index(JPEG_END, i0) + 2
        img = cv2.imdecode(np.frombuffer(st[i0:i1], np.uint8), cv2.IMREAD_COLOR)
        check(img is not None and img.shape[:2] == (480, 848 + 480), "a streamed picture: camera + top view, %s" % (
            img.shape if img is not None else None,))
    # a test locks the head, then the plate is lost: located again from here, the head does not move
    del LINK[:]
    del LOCATES[:]
    FakeCam.N, FakeCam.LOCK_AT, FakeCam.LOST_AT = 200, 40, 70
    sys.argv = ["perception.py", "--expect", "0.36", "0.06", "--view-port", "0"]
    try:
        P.main()
    except Stop:
        pass
    lk = LINK[0]
    t_lock = next((t for t, p in lk.sent if p.get("head_lock")), None)
    after = [h for t, h in lk.heads if t_lock is not None and t > t_lock]
    check(len(LOCATES) >= 2 and t_lock is not None and not after,
          "plate lost while the head is locked: located again (%d locates) without moving the head (%d head commands after the lock)" % (
              len(LOCATES), len(after)))
    n_bad = sum(1 for ok, _ in OK if not ok)
    print("\n%d checks, %d failed" % (len(OK), n_bad))
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())
