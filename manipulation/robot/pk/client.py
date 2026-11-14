"""Client for the arm server (server.py), used by tests, perception and tuning programs."""
import contextlib
import json
import socket
import threading
import time

import numpy as np

from . import config


class Denied(RuntimeError):
    pass


class ServerGone(RuntimeError):
    pass


class Client(object):
    def __init__(self, name, role="arms", host=config.HOST, port=config.SERVER_PORT, hang_s=1.0, wait_s=3.0, wait_home_s=90.0):
        self.name, self.role = name, role
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, 0))
        self.sock.settimeout(0.2)
        if hasattr(socket, "SIO_UDP_CONNRESET"):                # Windows only (see server_core.udp_no_reset)
            try:
                self.sock.ioctl(socket.SIO_UDP_CONNRESET, False)
            except (OSError, ValueError):
                pass
        self._lock = threading.Lock()
        self._state, self._state_t = None, 0.0
        self._welcome = None
        self._denied = None
        self._q = self._h = None
        self._extra = {}
        self._head = None
        self._seq = 0
        self._stop = threading.Event()
        self._last_main = time.time()
        self._hang_s = hang_s
        self._busy_until = 0.0
        self._rx = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx.start()
        self._send(dict(op="hello", role=role, name=name))
        t0 = time.time()
        while self._welcome is None:
            if time.time() - t0 > wait_s:
                raise ServerGone("no answer from the arm server on port %d: is server.py running?" % port)
            time.sleep(0.01)
            if time.time() - t0 > 0.5 and int((time.time() - t0) * 10) % 5 == 0:
                self._send(dict(op="hello", role=role, name=name))
        # the arms still on their way home (the last test's rewind): wait for them instead of failing
        t_wait = time.time()
        said = False
        while (not self._welcome.get("ok") and role == "arms" and self._welcome.get("mode") in ("REWIND", "RETURN", "GO_HOME")
               and time.time() - t_wait < wait_home_s):
            if not said:
                print("waiting for the arms to get back to HOME (%s) ..." % self._welcome.get("mode"), flush=True)
                said = True
            time.sleep(0.5)
            self._welcome = None
            self._send(dict(op="hello", role=role, name=name))
            t1 = time.time()
            while self._welcome is None and time.time() - t1 < wait_s:
                time.sleep(0.01)
            if self._welcome is None:
                raise ServerGone("the arm server stopped answering")
        if not self._welcome.get("ok"):
            raise Denied(self._welcome.get("reason", "refused"))
        self.home = dict(q=np.array(self._welcome["home"]["q"]), hands=np.array(self._welcome["home"]["hands"]))
        self.head_look = np.array(self._welcome.get("head_look", config.HEAD_LOOK))
        self.wait_state()
        if role in ("arms",):
            t1 = time.time()
            while time.time() - t1 < 0.3:                    # a state sent before the welcome still names no owner: not "taken back"
                with self._lock:
                    if self._state.get("owner") == name:
                        break
                time.sleep(0.005)
            st = self.state()
            self._q, self._h = np.array(st["des"]), np.array(st["hands_cmd"])
        if role in ("arms", "head", "monitor"):
            self._tx = threading.Thread(target=self._send_loop, daemon=True)
            self._tx.start()

    # ------------------------------------------------------------------ plumbing
    def _send(self, obj):
        try:
            self.sock.sendto(json.dumps(obj).encode(), self.addr)
        except OSError:
            pass

    def _recv_loop(self):
        while not self._stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65536)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue
            except OSError:
                return
            try:
                m = json.loads(data.decode())
            except ValueError:
                continue
            op = m.get("op")
            if op == "state":
                with self._lock:
                    self._state, self._state_t = m, time.time()
            elif op == "welcome":
                self._welcome = m
            elif op == "denied":
                self._denied = m.get("reason", "")

    def _send_loop(self):
        while not self._stop.is_set():
            now = time.time()
            alive = (now - self._last_main < self._hang_s) or now < self._busy_until
            if alive:
                if self.role == "arms":
                    with self._lock:
                        msg = dict(op="cmd", seq=self._seq, q=None if self._q is None else [float(v) for v in self._q],
                                   hands=None if self._h is None else [float(v) for v in self._h], **self._extra)
                    self._send(msg)
                elif self.role == "head" and self._head is not None:
                    self._send(dict(op="head", target=[float(v) for v in self._head]))
                elif self.role in ("head", "monitor"):
                    # keep-alive: the server drops a client that is silent for 5 s
                    self._send(dict(op="sub"))
            time.sleep(0.02)

    def alive(self):
        """Call from the main loop: proves the program is not stuck."""
        self._last_main = time.time()

    @contextlib.contextmanager
    def busy(self, seconds):
        """A known long computation: the server holds the arms (no time-out) for up to `seconds` (max 60), even if
        this process cannot send meanwhile; if it dies inside, the arms hold until then and then go home."""
        self._busy_until = time.time() + seconds
        self._send(dict(op="busy", seconds=float(seconds)))
        try:
            yield
        finally:
            self._busy_until = 0.0
            self._send(dict(op="busy", seconds=0.0))
            self.alive()

    # ------------------------------------------------------------------ state
    def wait_state(self, timeout=2.0):
        t0 = time.time()
        while True:
            with self._lock:
                if self._state is not None and time.time() - self._state_t < 0.5:
                    return
            if time.time() - t0 > timeout:
                raise ServerGone("the arm server stopped sending state")
            time.sleep(0.005)

    def state(self, max_age=0.5):
        self.alive()
        with self._lock:
            st, t = self._state, self._state_t
        if st is None or time.time() - t > max_age:
            self.wait_state()
            with self._lock:
                st = self._state
        if self.role == "arms" and st.get("owner") != self.name and self._welcome is not None:
            raise Denied("the server took the arms back (mode %s%s)" % (st.get("mode"), (": " + st["freeze"]) if st.get("freeze") else ""))
        return st

    def fresh_state(self, after):
        """The first state received after time `after`."""
        while True:
            with self._lock:
                st, t = self._state, self._state_t
            if st is not None and st["t"] > after:
                return self.state()
            time.sleep(0.002)

    # ------------------------------------------------------------------ commands
    def set(self, q=None, hands=None, vmax=None, hand_lead=None, tag=None, payload=None):
        self.alive()
        with self._lock:
            if payload is not None:
                self._extra["payload"] = payload if payload else {"side": "right", "mass": 0}
            if q is not None:
                self._q = np.array(q, float)
            if hands is not None:
                self._h = np.clip(np.array(hands, float), 0, 1)
            if vmax is not None:
                self._extra["vmax"] = float(vmax)
            if hand_lead is not None:
                self._extra["hand_lead"] = float(hand_lead)
            if tag is not None:
                self._extra["tag"] = tag
            self._seq += 1
            msg = dict(op="cmd", seq=self._seq, q=None if self._q is None else [float(v) for v in self._q],
                       hands=None if self._h is None else [float(v) for v in self._h], **self._extra)
        self._send(msg)

    def head(self, target):
        self.alive()
        self._head = config.clamp_head(np.array(target, float))
        self._send(dict(op="head", target=[float(v) for v in self._head]))

    def request(self, what, **args):
        """Ask perception (through the server) for something: 'locate', 'aim' (target=[x,y,z]), 'head_lock' (on=bool)."""
        seq = int(time.time() * 1000) % 1000000000          # increasing, unique across restarts
        self._send(dict(op="req", seq=seq, what=what, args=args))
        return seq

    def mark(self):
        """Remember the current pose on the server's recorded path (see trim)."""
        self.alive()
        self._send(dict(op="mark"))

    def trim(self):
        """Back at the marked pose: the server forgets the path recorded after the mark, so the rewind home at bye()
        retraces only the way to the mark. Refused (server's note) unless the arms are within 0.01 rad of it."""
        self.alive()
        self._send(dict(op="trim"))

    def publish_plate(self, plate):
        self._send(dict(op="plate", plate=plate))

    def target(self):
        return None if self._q is None else self._q.copy(), None if self._h is None else self._h.copy()

    def move_to(self, q, hands=None, v=0.3, tag=None, tol=0.004, timeout=None, check=None):
        """Smooth joint-space move (smoothstep, fastest joint at <= v rad/s) from the current target to q. Blocks until
        the server's planned ARM pose is within tol of q (q's waist is ignored: the server never turns it).
        check(state) -> str: a non-empty reason aborts (returns it)."""
        q = np.array(q, float)
        q0, h0 = self.target()
        h1 = h0 if hands is None else np.array(hands, float)
        dist = float(np.max(np.abs(q - q0))) if q0 is not None else 0.0
        T = max(0.3, 1.5 * dist / max(v, 1e-3))
        dh = float(np.max(np.abs(h1 - h0))) if h0 is not None else 0.0
        T = max(T, dh / 1.0)
        t0 = time.time()
        self.set(tag=tag, vmax=max(v * 1.6, 0.05))
        while True:
            u = min(1.0, (time.time() - t0) / T)
            s = u * u * (3 - 2 * u)
            self.set(q=q0 + s * (q - q0), hands=h0 + s * (h1 - h0))
            st = self.state()
            if check is not None:
                why = check(st)
                if why:
                    return why
            off = np.abs(np.array(st["des"]) - q)
            off[config.WAIST] = 0.0             # the server holds the waist where the robot had it, whatever q says
            if u >= 1.0 and np.max(off) < tol:
                return ""
            if timeout is not None and time.time() - t0 > timeout:
                return "timeout"
            if u >= 1.0 and time.time() - t0 > T + 10.0:
                return "server did not get there in 10 s (mode %s)" % st["mode"]
            time.sleep(0.01)

    def to_pregrip(self, v=0.3, tag="to the pre-grip pose"):
        """Move to the pre-grip pose (params/premotion.json: arms and hands; the waist stays): where the older stages and
        tools (push, grip1.., armid, armwide, handid, nearforce, calib_cam, table_check, touch_watch, tune) were planned
        from. The server's HOME is the neutral pose; a server started with --home pregrip is already there.
        Straight in joint space (checked in MuJoCo from the neutral HOME: tests/home_paths_check.py). Returns "" or why not."""
        h = json.load(open(config.PREGRIP_FILE))
        q = np.array(h["q"], float)
        st = self.state()
        q[config.WAIST] = st["des"][config.WAIST]
        hands = np.array(h["hands"], float)
        if float(np.max(np.abs(q - np.array(st["des"], float))[:10])) < 0.004 and float(np.max(np.abs(hands - np.array(st["hands_cmd"], float)))) < 0.01:
            return ""
        return self.move_to(q, hands, v=v, tag=tag)

    def hold(self, seconds, check=None):
        t0 = time.time()
        while time.time() - t0 < seconds:
            st = self.state()
            if check is not None:
                why = check(st)
                if why:
                    return why
            time.sleep(0.01)
        return ""

    def bye(self, then="home", why="done"):
        self._send(dict(op="bye", then=then, why=why))
        time.sleep(0.05)
        self._stop.set()

    def close(self):
        self._stop.set()
        try:
            self.sock.close()
        except OSError:
            pass


def operator(cmd, host=config.HOST, port=config.SERVER_PORT, timeout=1.0, said=None):
    """Send an operator command (freeze, home, status, take, release) to the server. Returns its mode or None; the
    server's answer lines go into the list `said` if one is given."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(dict(op="operator", cmd=cmd)).encode(), (host, port))
        while True:
            data, _ = s.recvfrom(65536)
            m = json.loads(data.decode())
            if m.get("op") == "ack":
                if said is not None:
                    said.extend(m.get("said") or [])
                return m.get("mode")
    except (socket.timeout, OSError, ValueError):
        return None
    finally:
        s.close()
