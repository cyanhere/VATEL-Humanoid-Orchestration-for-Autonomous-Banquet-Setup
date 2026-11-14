"""The arm server: the only program that commands the R1's arms, head and hands.

Every target passes a safety filter (joint, speed, acceleration and jump limits, finger lead limit) and gravity
compensation. Watchdogs on force, joint error, torque and state age freeze the arms where they are. When a test ends
or fails, the server retraces its commanded path back to the home pose.
"""
import json
import os
import socket
import time

import numpy as np

from . import config
from .fit import force_pinv
from .model import GravityComp


MAX_TARGET_JUMP = 0.25         # rad: a client target further than this from the current pose is refused
REWIND_LOOK = 0.025            # rad: the rewind aims this far ahead along the recorded path


def _r(a, n=5):
    return [round(float(x), n) for x in a]


def arm_off(a, b):
    """max |a - b| over the arm joints. The waist is not ours: the server holds it wherever the robot had it when the
    arms were taken, so it must never count in an 'arrived' test."""
    d = np.abs(np.asarray(a, float) - np.asarray(b, float))
    d[config.WAIST] = 0.0
    return float(np.max(d))


class Server(object):
    def __init__(self, io, home=None, port=config.SERVER_PORT, log_dir=config.LOG_DIR, kp_scale=1.0, comp=True,
                 v_max=config.V_MAX, verbose=True, gravity_path=None, kd_scale=1.0, fric=0.0, dq_ff=False):
        self.io = io
        self.verbose = verbose
        home = home or load_home()
        self.home_q = config.clamp_arm(np.array(home["q"], float))
        self.home_h = np.clip(np.array(home["hands"], float), 0, 1)
        self.head_look = config.clamp_head(np.array(home.get("head", config.HEAD_LOOK), float))
        self.kp_scale = float(np.clip(kp_scale, *config.GAIN_SCALE_KP))
        self.kd_scale = float(np.clip(kd_scale, *config.GAIN_SCALE_KD))
        self.kp, self.kd = config.arm_gains(self.kp_scale, self.kd_scale)
        self.v_max = min(v_max, config.V_MAX)
        self.comp = GravityComp(path=gravity_path, enabled=comp)
        self.fric_alpha = float(np.clip(fric, 0.0, config.FRIC_MAX))
        self.fric_gate = True                    # the lead only while a joint lags behind its plan (`fric A raw`: always)
        self.fric = np.zeros(config.N_ARM)       # friction lead in the command now (rad), low-passed
        self.dq_ff = bool(dq_ff)                 # send the planned joint speed as the motors' dq target (else 0)
        self.home_name = home.get("name", os.path.basename(str(home.get("_path", "HOME"))))
        self.ret_from = self.ret_h = None
        self.ret_T = 0.0
        self.mode = "WAIT_TAKE"
        self.mode_t = time.time()
        self.weight = 0.0
        self.des = None                      # planned arm pose after the safety filter (before gravity offset)
        self.vel = np.zeros(config.N_ARM)
        self.sent = None
        self.grav = np.zeros(config.N_ARM)
        self.hands = self.home_h.copy()      # hand command actually sent
        self.head_cmd = None
        self.head_target = None
        self.head_goal = self.head_via = None   # the target being reached, and the point below it in yaw to come from
        self.waist = 0.0
        self.target = None                   # where the current mode wants the arms
        self.target_h = None
        self.v_lim = self.v_max
        self.owner = None                    # dict(addr, name, t, seq, tag)
        self.head_owner = None
        self.history = []                    # [(q11, hands12)] commanded since the owner took the arms at HOME
        self.history_mark = None             # an index into history the owner marked (op mark / trim)
        self.freeze_reason = ""
        self.start_rpy = None
        self.err_since = None
        self.plate = None
        self.plate_t = 0.0
        self.req = None
        self.payload = None                  # extra mass carried by a hand (the client says), for the gravity offset
        self.last_head_addr, self.last_head_name = None, ""
        self.f_hand = {"left": 0.0, "right": 0.0}
        self.f_since = None
        self.subs = {}                       # addr -> last seen
        self.cycle = 0
        self.go_from = None
        self.release_done = False
        self.release_t = 0.0
        self.release_w0 = 0.0
        self.release_h0 = None
        self.take_w0 = 0.0
        self.hands_init = False
        self.notes = []
        self.lead_cap = config.HAND_LEAD_MAX
        self.tag = ""
        self.hand_lead_client = None
        self.last_tgt = None
        self.tgt_t = 0.0
        self.v_tgt = np.zeros(config.N_ARM)
        self.t_last = 0.0
        self.cyc_dt = []
        self.been_home = False
        self.hand_still = 0
        self.jump_warn_t = 0.0
        self.ctrl_c_t = 0.0                  # the last Ctrl-C in the server's terminal
        self.late_said = False
        self.said = 0                        # messages so far (every operator command must answer)
        self.quit = False                    # server.py stops (only when nothing is taken)
        self.limp, self.limp_since = False, None
        # UDP. Exactly one server: two would both command the arms. No address reuse (Windows would let a second
        # process bind the same port), and anything already answering on the port stops this one.
        if another_server(port):
            raise RuntimeError("another arm server already answers on port %d: stop it first (only ONE may command the arms)" % port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        try:
            self.sock.bind((config.HOST, port))
        except OSError as ex:
            raise RuntimeError("port %d is taken (%s): is another arm server running? Only ONE may command the arms" % (port, ex))
        self.sock.setblocking(False)
        udp_no_reset(self.sock)
        # log
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, time.strftime("server_%Y%m%d_%H%M%S.csv"))
        self.log = open(self.log_path, "w")
        self._log_header()

    # ------------------------------------------------------------------ messages
    def say(self, txt):
        line = time.strftime("%H:%M:%S ") + txt
        self.said += 1
        self.notes.append(line)
        self.notes = self.notes[-20:]
        if self.verbose:
            print(line, flush=True)

    def _send(self, addr, obj):
        try:
            self.sock.sendto(json.dumps(obj).encode(), addr)
        except OSError:
            pass

    def poll(self, now):
        for _ in range(200):
            try:
                data, addr = self.sock.recvfrom(65536)
            except BlockingIOError:
                return
            except ConnectionResetError:          # Windows: an earlier send went to a closed client port; not ours
                continue
            except OSError:
                return
            try:
                m = json.loads(data.decode())
            except ValueError:
                continue
            self.handle(m, addr, now)

    def handle(self, m, addr, now):
        op = m.get("op")
        self.subs[addr] = now
        if op == "hello":
            role = m.get("role", "arms")
            ok, why = self.claim(role, addr, m.get("name", "?"), now)
            self._send(addr, dict(op="welcome", ok=ok, reason=why, role=role, home=dict(q=_r(self.home_q), hands=_r(self.home_h)),
                                  head_look=_r(self.head_look), mode=self.mode))
        elif op == "cmd":
            o = self.owner
            if o is None or o["addr"] != addr:
                self._send(addr, dict(op="denied", reason="you do not own the arms (mode %s)" % self.mode))
                return
            o["t"] = now
            o["seq"] = m.get("seq", 0)
            if m.get("q") is not None:
                q = np.array(m["q"], float)
                if q.shape == (config.N_ARM,) and np.all(np.isfinite(q)):
                    jump = np.abs(config.clamp_arm(q) - self.des)
                    jump[config.WAIST] = 0.0
                    if np.max(jump) > MAX_TARGET_JUMP:
                        # a client must stream a path, not teleport: refuse, keep the last target
                        if now - self.jump_warn_t > 1.0:
                            self.jump_warn_t = now
                            j = int(np.argmax(jump))
                            self.say("refused a target %.2f rad away (%s) from '%s': send a path" % (jump[j], config.ARM_NAMES[j], o["name"]))
                            self._send(addr, dict(op="denied", reason="target %.2f rad from the current pose" % jump[j]))
                    else:
                        self.target = q
            if m.get("hands") is not None:
                h = np.array(m["hands"], float)
                if h.shape == (12,) and np.all(np.isfinite(h)):
                    self.target_h = np.clip(h, 0, 1)
            v = m.get("vmax")
            self.v_lim = min(self.v_max, float(v)) if v else self.v_max
            hl = m.get("hand_lead")
            self.hand_lead_client = float(hl) if hl is not None else None
            pl = m.get("payload")
            if pl is not None:
                ok = isinstance(pl, dict) and pl.get("side") in ("left", "right") and 0.0 <= float(pl.get("mass", 0)) <= 1.5
                self.payload = pl if ok and float(pl.get("mass", 0)) > 0 else None
            self.tag = str(m.get("tag", self.tag))[:40]
        elif op in ("mark", "trim") and self.owner is not None and self.owner["addr"] == addr and self.mode == "CLIENT":
            # mark: remember the position on the recorded path. trim: the client is back at the marked pose (within 0.01 rad),
            # so everything recorded after the mark is dropped and the rewind only retraces the way to the mark
            if op == "mark":
                self.history_mark = len(self.history) - 1
                self.say("path marked at waypoint %d" % self.history_mark)
            elif self.history_mark is not None and self.history_mark < len(self.history):
                q_m = self.history[self.history_mark][0]
                if arm_off(self.des, q_m) < 0.01:
                    n = len(self.history)
                    del self.history[self.history_mark + 1:]
                    self.say("path trimmed to the mark: %d -> %d waypoints" % (n, len(self.history)))
                else:
                    self.say("trim refused: the arms are %.3f rad from the marked pose" % arm_off(self.des, q_m))
        elif op == "busy":                                   # the owner computes for a while: hold, do not time it out
            for o in (self.owner, self.head_owner):
                if o is not None and o["addr"] == addr:
                    o["t"] = now
                    o["lease"] = now + min(max(float(m.get("seconds", 0)), 0.0), 60.0)
        elif op == "head":
            o = self.head_owner
            if o is None and self.last_head_addr == addr:      # it went quiet (busy) and came back: its head again
                self.head_owner = o = dict(addr=addr, name=self.last_head_name, t=now)
                self.say("head given back to '%s'" % o["name"])
            if o is None or o["addr"] != addr:
                return
            o["t"] = now
            h = m.get("target")
            if h is not None and len(h) == 2 and np.all(np.isfinite(h)):
                self.head_target = config.clamp_head(np.array(h, float))
        elif op == "plate":
            self.plate = m.get("plate")
            self.plate_t = now
        elif op == "req":                                   # a test asks perception for something (relayed in the state)
            self.req = dict(seq=int(m.get("seq", 0)), what=str(m.get("what", "")), args=m.get("args"), t=round(now, 3),
                            frm=self.owner["name"] if self.owner and self.owner["addr"] == addr else "?")
        elif op == "bye":
            if self.owner is not None and self.owner["addr"] == addr:
                then = m.get("then", "home")
                self.say("client '%s' finished (%s)" % (self.owner["name"], m.get("why", "done")))
                self.owner = None
                if then == "hold":
                    self.freeze("client asked to hold here", at_measured=False)
                elif then == "path":
                    self.start_rewind("client finished (%s)" % m.get("why", "done"))
                else:
                    self.start_return("client finished")
            if self.head_owner is not None and self.head_owner["addr"] == addr:
                self.head_owner = None
        elif op == "operator":
            n = self.said
            self.operator(str(m.get("cmd", "")), now)
            k = min(self.said - n, len(self.notes))
            self._send(addr, dict(op="ack", mode=self.mode, said=self.notes[-k:] if k > 0 else []))
        elif op == "sub":
            pass

    def claim(self, role, addr, name, now):
        if role == "head":
            if self.head_owner is not None and now - self.head_owner["t"] < config.CLIENT_TIMEOUT_S and self.head_owner["addr"] != addr:
                return False, "the head belongs to '%s'" % self.head_owner["name"]
            self.head_owner = dict(addr=addr, name=name, t=now)
            self.last_head_addr, self.last_head_name = addr, name
            self.say("head given to '%s'" % name)
            return True, ""
        if role == "monitor":
            return True, ""
        if self.owner is not None and self.owner["addr"] != addr:
            return False, "the arms belong to '%s'" % self.owner["name"]
        if self.mode != "HOME":
            return False, "the arms are not resting at HOME (mode %s)" % self.mode
        self.owner = dict(addr=addr, name=name, t=now, seq=0, t0=now)
        self.mode, self.mode_t = "CLIENT", now
        self.target, self.target_h = self.des.copy(), self.hands.copy()
        self.history = [(self.des.copy(), self.hands.copy())]
        self.v_lim, self.hand_lead_client, self.tag = self.v_max, None, ""
        self.say("arms given to '%s'" % name)
        return True, ""

    # ------------------------------------------------------------------ operator
    def operator(self, cmd, now=None):
        now = now or time.time()
        c = cmd.strip().lower()
        taken = self.weight >= 0.999
        if c in ("take",):
            if self.mode == "WAIT_TAKE" or (self.mode == "FREEZE" and not taken):
                self.start_take(now)
            elif self.mode == "RELEASE" and not self.release_done:
                self.say("release STOPPED: taking the arms back where they are")
                self.start_take(now)
            elif self.mode == "RELEASE":
                self.say("too late: released, the server is stopping (start it again)")
            else:
                self.say("already taken (weight %.2f, mode %s)" % (self.weight, self.mode))
        elif c in ("home", "h", "home path", "hp"):
            path = c in ("home path", "hp")
            if not taken:
                # never move the arms while the robot is not following our commands: what we "move" would only be
                # waiting there for the weight to come up
                self.say("REFUSED: the arms are not taken (weight %.2f, mode %s). Type `take` first" % (self.weight, self.mode))
            elif self.mode in ("HELD",) or (self.mode == "FREEZE" and not self.been_home):
                self.mode, self.mode_t = "GO_HOME", now
                self.go_from = self.des.copy()
                self.vel[:] = 0.0
                self.late_said = False
                self.say("moving to HOME (%s) in joint space at %.2f rad/s -- the TABLE must be out of reach" % (self.home_name, config.V_HOME))
            elif self.mode in ("CLIENT", "FREEZE"):
                if self.owner is not None:
                    self.say("operator: taking the arms back from '%s'" % self.owner["name"])
                    self.owner = None
                if path:
                    self.start_rewind("operator asked")
                else:
                    self.start_return("operator asked")
            elif self.mode == "RETURN" and path:
                self.start_rewind("operator asked: along the path instead")
            elif self.mode in ("HOME", "REWIND", "GO_HOME", "RETURN"):
                self.say("already %s" % self.mode)
            else:
                self.say("cannot go home from %s" % self.mode)
        elif c in ("f", "freeze", "stop", "s"):
            if self.mode in ("CLIENT", "REWIND", "RETURN", "HOME", "GO_HOME", "HELD", "TAKE"):
                self.freeze("operator")
            elif self.mode == "FREEZE":
                self.say("already frozen (%s). `home path` = back along the path; `home` = straight to HOME; `release` = to the robot" % self.freeze_reason)
            elif self.mode == "RELEASE":
                self.say("releasing (weight %.2f): `take` stops the release and holds the arms where they are" % self.weight)
            else:
                self.say("nothing to freeze: the arms are not taken (mode %s)" % self.mode)
        elif c in ("release",):
            self.release(now)
        elif c in ("quit", "exit", "q"):
            if self.mode == "WAIT_TAKE":
                self.say("stopping (the arms were never taken)")
                self.quit = True
            else:
                self.say("refused: the arms are held (mode %s). `release` hands them to the robot, then the server stops" % self.mode)
        elif c in ("status", "st"):
            self.say(self.status_line())
        elif c in ("ping", ""):
            pass
        elif c.startswith("comp"):
            self.comp.enabled = not c.endswith("off")
            self.say("gravity compensation %s" % ("on" if self.comp.enabled else "OFF"))
        elif c.split()[0] in ("fric", "gains"):
            self.set_tuning(c.split())
        elif c.split()[0] == "dqff":
            if self.mode == "CLIENT":
                self.say("refused: '%s' owns the arms; change it between tests" % (self.owner["name"] if self.owner else "a test"))
            elif c.split()[1:] in (["on"], ["off"]):
                self.dq_ff = c.split()[1] == "on"
                self.say("planned joint speed to the motors (dq target): %s" % ("ON" if self.dq_ff else "off (0, as before 2026-11-06)"))
            else:
                self.say("usage: dqff on | dqff off   (now %s)" % ("on" if self.dq_ff else "off"))
        else:
            self.say("commands: take | home (straight) | home path (back along the path) | f (freeze) | status | comp on/off | "
                     "fric A | gains KP KD | dqff on/off | release | quit")

    def set_tuning(self, words):
        """`fric A` (friction lead, fraction of the fitted friction, 0 = off) and `gains KP KD` (scales of Unitree's arm
        gains): between tests only, so one test runs with one setting; every log row records them."""
        if self.mode == "CLIENT":
            self.say("refused: '%s' owns the arms; change it between tests" % (self.owner["name"] if self.owner else "a test"))
            return
        raw = words[0] == "fric" and len(words) == 3 and words[2] == "raw"
        try:
            vals = [float(w) for w in words[1:(2 if raw else None)]]
        except ValueError:
            vals = []
        if words[0] == "fric" and len(vals) == 1:
            self.fric_alpha = float(np.clip(vals[0], 0.0, config.FRIC_MAX))
            self.fric_gate = not raw
            self.say("friction lead x%.2f of the fitted friction, %s%s" % (
                self.fric_alpha, "always (raw)" if raw else "only while a joint lags behind its plan",
                "" if self.comp.friction.any() else " -- but the gravity fit has no friction: nothing changes"))
        elif words[0] == "gains" and len(vals) == 2:
            self.kp_scale = float(np.clip(vals[0], *config.GAIN_SCALE_KP))
            self.kd_scale = float(np.clip(vals[1], *config.GAIN_SCALE_KD))
            self.kp, self.kd = config.arm_gains(self.kp_scale, self.kd_scale)
            self.say("arm gains: kp x%.2f, kd x%.2f of Unitree's (the waist unchanged)" % (self.kp_scale, self.kd_scale))
        else:
            self.say("usage: fric A [raw] (0..%.1f; 0 = off)  |  gains KP_SCALE KD_SCALE (kp %.1f..%.1f, kd %.1f..%.1f)" % (
                config.FRIC_MAX, config.GAIN_SCALE_KP[0], config.GAIN_SCALE_KP[1], config.GAIN_SCALE_KD[0], config.GAIN_SCALE_KD[1]))

    def release(self, now=None, why="operator"):
        """Hand the arms back to the robot from ANY mode: the operator's word is final. A test that owns the arms loses them. `take` within the 3 s ramp undoes it."""
        now = now or time.time()
        if self.mode == "WAIT_TAKE":
            self.say("nothing to release: the arms were never taken (`quit` or Ctrl-C stops the server)")
            return
        if self.mode == "RELEASE":
            self.say("already releasing (weight %.2f)" % self.weight)
            return
        if self.owner is not None:
            self.say("'%s' loses the arms (%s)" % (self.owner["name"], why))
            self.owner = None
        self.start_release(now)

    def ctrl_c(self, now=None):
        """Ctrl-C in the server's terminal: freeze; a second one within 3 s releases. Every press answers. Returns True when the server should stop (nothing taken)."""
        now = now or time.time()
        prev, self.ctrl_c_t = self.ctrl_c_t, now
        if self.mode == "WAIT_TAKE":
            return True
        if self.mode == "RELEASE":
            self.say("releasing (weight %.2f): the server stops by itself at weight 0; `take` stops the release" % self.weight)
        elif self.mode == "FREEZE" and now - prev < 3.0:
            self.release(now, "Ctrl-C twice")
        elif self.mode == "FREEZE":
            self.say("frozen (%s). Ctrl-C again within 3 s = RELEASE; or type `home` / `release`" % self.freeze_reason)
        else:
            self.freeze("operator Ctrl-C")
            self.say("Ctrl-C again within 3 s RELEASES the arms (the robot lowers them); `home` goes back along the path")
        return False

    def status_line(self):
        s = self.io.s
        err = arm_off(self.des, s.q) if self.des is not None else 0.0
        cyc = ("cycle %.1f ms mean, %.1f max | " % (1000 * np.mean(self.cyc_dt), 1000 * np.max(self.cyc_dt))) if self.cyc_dt else ""
        return cyc + ("mode %s%s, owner %s, head %s, weight %.2f, max |plan - measured| %.3f rad, hands %s, touch %s, plate %s, comp %s"
                      " | kp x%.2f kd x%.2f, friction lead x%.2f, dq target %s, HOME %s" % (
                          self.mode, (" (%s)" % self.freeze_reason) if self.mode == "FREEZE" else "",
                          self.owner["name"] if self.owner else "-", self.head_owner["name"] if self.head_owner else "-", self.weight, err,
                          "ok" if s.hands_ok() else "NO STATE", "ok" if s.touch_ok() else "none",
                          "%.1f s old" % (time.time() - self.plate_t) if self.plate is not None else "none",
                          self.comp.source if self.comp.enabled else "OFF", self.kp_scale, self.kd_scale, self.fric_alpha,
                          "planned" if self.dq_ff else "0", self.home_name))

    # ------------------------------------------------------------------ modes
    def start_take(self, now):
        """Weight ramps from where it is to 1 while the command is the MEASURED pose: the arms do not move."""
        s = self.io.s
        if self.des is not None:
            q = s.q.copy()
            q[config.WAIST] = self.waist
            self.des = config.clamp_arm(q)
            self.target = self.des.copy()
            self.vel[:] = 0.0
        self.take_w0 = self.weight
        self.mode, self.mode_t = "TAKE", now
        self.say("taking the arms where they are (weight %.2f -> 1 over %.1f s, no motion)" % (self.weight, 2.0 * (1.0 - self.weight)))

    def start_release(self, now):
        """Weight ramps from where it is (never up) to 0 over 3 s; the hands open over the same 3 s."""
        self.release_w0 = self.weight
        self.release_h0 = self.hands.copy()
        self.release_done = False
        self.vel[:] = 0.0
        if self.des is not None:
            self.target = self.des.copy()
        self.mode, self.mode_t = "RELEASE", now
        self.say("RELEASING (weight %.2f -> 0 over 3 s, hands open): the robot's own controller lowers the arms" % self.weight)

    def freeze(self, why, at_measured=True):
        s = self.io.s
        if self.des is None:
            return
        if at_measured:
            q = s.q.copy()
            q[config.WAIST] = self.waist
            self.des = config.clamp_arm(q)              # hold where the arm IS: no pressing force left
        self.vel[:] = 0.0
        self.target = self.des.copy()
        self.target_h = self.hands.copy()
        if self.owner is not None:
            self.say("owner '%s' dropped" % self.owner["name"])
            self.owner = None
        self.mode, self.mode_t = "FREEZE", time.time()
        self.freeze_reason = why
        self.say("!" * 78)
        self.say("!!! FROZEN: %s" % why)
        if self.weight >= 0.999:
            self.say("!!! the arms hold where they are. `home path` = back along the path (a hand among the plates); `home` = straight "
                     "to HOME; `release` = to the robot (TABLE AWAY)")
        else:
            self.say("!!! the arms are NOT taken (weight %.2f). `take` = take them where they are; `release` = let go" % self.weight)
        self.say("!" * 78)

    def start_rewind(self, why):
        self.mode, self.mode_t = "REWIND", time.time()
        # keep any payload compensation during the rewind: the plate may still be in the hand
        self.vel[:] = 0.0
        n = len(self.history)
        self.say("going back to HOME along the commanded path (%d waypoints, %.2f rad/s): %s" % (n, config.V_REWIND, why))

    def start_return(self, why):
        """Straight to HOME in joint space: every joint arrives together, the fastest at <= V_RETURN; the hands keep their
        shape until the arms are home (opening on the way would drop anything held), then open."""
        self.mode, self.mode_t = "RETURN", time.time()
        self.vel[:] = 0.0
        self.ret_from = self.des.copy()
        self.ret_h = self.hands.copy()
        self.ret_T = max(1.0, 1.5 * arm_off(self.home_q, self.ret_from) / config.V_RETURN)
        self.history = [(self.des.copy(), self.hands.copy())]
        self.say("going straight to HOME (%.1f s): %s" % (self.ret_T, why))

    def check_owner(self, now):
        o = self.owner
        if o is not None and now - o["t"] > config.CLIENT_TIMEOUT_S and now > o.get("lease", 0.0):
            self.say("client '%s' silent for %.1f s (killed?)" % (o["name"], now - o["t"]))
            self.owner = None
            if self.mode == "CLIENT":
                self.start_rewind("client gone")
        h = self.head_owner
        if h is not None and now - h["t"] > 2.0 and now > h.get("lease", 0.0):
            self.say("head client '%s' gone: head stays where it is" % h["name"])
            self.head_owner = None

    def head_step(self, dt):
        """The head's next command toward head_target. A yaw move that would not END turning + (left) by
        config.HEAD_YAW_APPROACH first goes that far below the target's yaw (the camera's calibration holds only for a
        head that arrives this way: ~1 deg of yaw play). A target whose yaw does not change moves straight."""
        if self.head_goal is None or np.max(np.abs(self.head_target - self.head_goal)) > 1e-4:
            self.head_goal = self.head_target.copy()
            self.head_via = None
            below = self.head_goal[1] - config.HEAD_YAW_APPROACH
            if abs(self.head_goal[1] - self.head_cmd[1]) > 0.002 and self.head_cmd[1] > below + 1e-4:
                self.head_via = config.clamp_head(np.array([self.head_goal[0], below]))
        tgt = self.head_via if self.head_via is not None else self.head_goal
        cmd = config.clamp_head(self.head_cmd + np.clip(tgt - self.head_cmd, -config.V_HEAD * dt, config.V_HEAD * dt))
        if self.head_via is not None and np.max(np.abs(cmd - self.head_via)) < 1e-4:
            self.head_via = None
        return cmd

    def watchdog(self, s, now):
        if self.mode in ("WAIT_TAKE", "RELEASE", "FREEZE"):
            return
        if s.age(now) > config.STATE_STALE_S:
            self.freeze("robot state %.0f ms old" % (1000 * s.age(now)), at_measured=False)
            return
        if s.motor_err:
            self.freeze("an arm motor reports an error code")
            return
        if self.start_rpy is not None and np.max(np.abs(s.rpy[:2] - self.start_rpy[:2])) > config.LEAN_TRIP:
            self.freeze("the robot leans (IMU %.2f %.2f rad)" % tuple(s.rpy[:2] - self.start_rpy[:2]))
            return
        if self.weight < 0.99:
            return
        t_bad = np.abs(s.tau) > config.TAU_TRIP
        if np.any(t_bad):
            j = int(np.argmax(np.abs(s.tau)))
            self.freeze("%s torque %.1f N m" % (config.ARM_NAMES[j], s.tau[j]))
            return
        # force on each hand: a compliant arm pressing on something spreads the error over its joints, so neither
        # one joint's error nor one motor's torque need be large -- the force at the hand is what porcelain feels
        if self.cycle % 5 == 0:                                # 20 Hz
            self.f_hand = self.hand_forces(s)
        if self.mode in ("CLIENT", "REWIND", "RETURN", "HOME") and max(self.f_hand.values()) > config.F_TRIP:
            self.f_since = self.f_since or now
            if now - self.f_since > config.F_TRIP_S:
                side = max(self.f_hand, key=self.f_hand.get)
                self.freeze("force on the %s hand ~%.0f N (pressing on something)" % (side, self.f_hand[side]))
                return
        else:
            self.f_since = None
        e = np.abs(self.des - s.q)
        e[config.WAIST] = 0.0
        # the server's own moves are in free air: the arms must follow closely (not following = not obeying)
        trip = config.ERR_TRIP_OWN if self.mode in ("GO_HOME", "REWIND", "RETURN", "HOME", "HELD") else config.ERR_TRIP
        if np.max(e) > trip:
            self.err_since = self.err_since or now
            if now - self.err_since > config.ERR_TRIP_S:
                j = int(np.argmax(e))
                if np.max(np.abs(s.tau[:10])) < config.LIMP_TAU:
                    self.freeze("the arms do not respond: NO motor torque -- the robot's mode ignores arm commands (zero "
                                "torque / damping?); %s %.2f rad off" % (config.ARM_NAMES[j], e[j]))
                else:
                    self.freeze("%s is %.2f rad from where it should be (blocked?)" % (config.ARM_NAMES[j], e[j]))
        else:
            self.err_since = None

    def check_limp(self, s, now):
        """At full weight a held arm needs torque on some motor; none at all = the robot's mode ignores arm_sdk."""
        if np.max(np.abs(s.tau[:10])) < config.LIMP_TAU:
            self.limp_since = self.limp_since or now
            if now - self.limp_since > 0.5 and not self.limp:
                self.limp = True
                self.say("!!! the arms are LIMP: no motor torque at full weight -- the robot's mode ignores arm commands "
                         "(zero torque / damping?). Put it in its running mode; `home` would only freeze")
        else:
            self.limp_since = None
            if self.limp:
                self.limp = False
                self.say("the motors hold the arms now")

    # ------------------------------------------------------------------ the cycle
    def step(self, now=None):
        now = now or time.time()
        if self.t_last:
            self.cyc_dt.append(now - self.t_last)
            self.cyc_dt = self.cyc_dt[-500:]
        # the real time since the last cycle (a slow cycle must not slow the motion down, or speed it up past limits)
        dt = min(max(now - self.t_last, 0.002), 0.05) if self.t_last else 1.0 / config.RATE_HZ
        self.t_last = now
        s = self.io.read()
        self.poll(now)
        if self.mode == "WAIT_TAKE":
            self._broadcast(s, now)
            return
        if self.des is None:                               # first cycle after `take`
            self.io.open_arm()                              # the SDK pauses ~0.2 s per publisher it creates...
            s = self.io.read()                              #... so everything below starts from a FRESH reading
            now = time.time()
            self.des = s.q.copy()
            self.waist = float(s.q[config.WAIST])
            self.head_cmd = s.head.copy()
            self.head_target = s.head.copy()
            self.start_rpy = s.rpy.copy()
            self.hands_init = bool(s.hands_ok(now))
            self.hands = s.hand_q.copy() if self.hands_init else self.home_h.copy()
            self.target, self.target_h = self.des.copy(), self.hands.copy()
            self.take_w0 = 0.0
            self.mode_t = now                               # the weight ramp starts now, from 0
            self.t_last = now
        if not self.hands_init and s.hands_ok(now):         # the hand server came up later: start from where they are
            self.hands_init = True
            self.hands = s.hand_q.copy()
            if self.target_h is None or self.mode in ("TAKE", "HELD", "FREEZE"):
                self.target_h = self.hands.copy()
        self.check_owner(now)
        self.watchdog(s, now)
        if self.mode == "HELD" and self.weight >= 0.999:
            self.check_limp(s, now)
        m = self.mode
        if m in ("GO_HOME", "HOME", "CLIENT", "REWIND", "RETURN") and self.weight < 0.999:
            # the arms must never be "moved" while the robot is not following our commands (a home move
            # at weight 0, then a release that jumped to it)
            self.freeze("internal: %s with weight %.2f -- not moving" % (m, self.weight), at_measured=True)
            m = self.mode
        if m == "TAKE":
            self.weight = min(1.0, self.take_w0 + (now - self.mode_t) / 2.0)
            if self.weight >= 1.0:
                self.mode, self.mode_t = "HELD", now
                self.say("arms taken and held where they were. Table away from the arms? Then type `home`")
        elif m == "GO_HOME":
            T = max(3.0, arm_off(self.home_q, self.go_from) / config.V_HOME)
            u = min(1.0, (now - self.mode_t) / T)
            u = u * u * (3 - 2 * u)
            self.target = self.go_from + u * (self.home_q - self.go_from)
            self.target_h = self.home_h.copy()
            self.head_target = self.head_look.copy()
            if u >= 1.0 and arm_off(self.des, self.home_q) < 0.002:
                self.mode, self.mode_t = "HOME", now
                self.been_home = True
                self.say("at HOME (%s): holding. Tests can start now" % self.home_name)
            elif u >= 1.0 and now - self.mode_t > T + 3.0 and not self.late_said:
                self.late_said = True
                d = np.abs(self.des - self.home_q)
                d[config.WAIST] = 0.0
                j = int(np.argmax(d))
                self.say("the home move does not end: %s planned %.3f rad from HOME. `f` or `release` work" % (
                    config.ARM_NAMES[j], d[j]))
        elif m == "HOME":
            self.target, self.target_h = self.home_q.copy(), self.home_h.copy()
        elif m == "CLIENT":
            last = self.history[-1]
            if (np.max(np.abs(self.des - last[0])) > 0.004 or np.max(np.abs(self.hands - last[1])) > 0.02):
                self.history.append((self.des.copy(), self.hands.copy()))
        elif m == "RETURN":
            u = min(1.0, (now - self.mode_t) / self.ret_T)
            u = u * u * (3 - 2 * u)
            self.target = self.ret_from + u * (self.home_q - self.ret_from)
            self.target_h = self.ret_h
            if u >= 1.0 and arm_off(self.des, self.home_q) < 0.003:
                self.target_h = self.home_h.copy()                 # the arms are home: now open the hands
                if (np.max(np.abs(self.hands - self.home_h)) < 0.03) or self.hand_still > 50:
                    self.mode, self.mode_t = "HOME", now
                    self.been_home = True
                    self.history = [(self.des.copy(), self.hands.copy())]
                    self.say("back at HOME")
        elif m == "REWIND":
            # chase a point REWIND_LOOK ahead on the recorded path (continuous motion, within a few mm of the path);
            # a finger that cannot reach its waypoint (blocked; the lead cap holds it) must not stall the rewind
            hands_there = lambda h_w, tol=0.03: (np.max(np.abs(self.hands - h_w)) < tol) or self.hand_still > 50   # noqa: E731
            while self.history:
                q_w, h_w = self.history[-1]
                if arm_off(self.des, q_w) < REWIND_LOOK and hands_there(h_w, 0.06):
                    self.history.pop()
                    continue
                self.target, self.target_h = q_w, h_w
                break
            if not self.history:
                self.target, self.target_h = self.home_q.copy(), self.home_h.copy()
                if arm_off(self.des, self.home_q) < 0.003 and hands_there(self.home_h):
                    self.mode, self.mode_t = "HOME", now
                    self.been_home = True
                    self.say("back at HOME (along the path)")
        elif m == "RELEASE":
            # from the weight it had, never up; the hands open over the same 3 s
            u = min(1.0, max(0.0, (now - self.mode_t) / 3.0))
            self.weight = self.release_w0 * (1.0 - u)
            self.target_h = self.release_h0 * (1.0 - u)
            if u >= 1.0 and not self.release_done:
                self.release_done = True
                self.release_t = now
                self.say("released (arms to the robot, hands open)")
        # -- safety filter: limits, speed, acceleration
        if self.mode in ("CLIENT",):
            v_lim = self.v_lim
        elif self.mode == "REWIND":
            v_lim = config.V_REWIND
        elif self.mode == "RETURN":
            v_lim = config.V_RETURN
        elif self.mode == "GO_HOME":
            v_lim = config.V_HOME
        else:
            v_lim = self.v_max
        if self.mode in ("FREEZE", "TAKE", "HELD", "RELEASE", "WAIT_TAKE"):
            self.vel[:] = 0.0
            tgt = self.des
        else:
            tgt = config.clamp_arm(self.target)
        tgt = tgt.copy()
        tgt[config.WAIST] = self.waist                     # the waist is not ours to turn (yet)
        self.des = self.track(tgt, v_lim, dt, now)
        # hands: speed limit, and closing fingers kept within the lead of where they are
        th = self.target_h if self.target_h is not None else self.hands
        if self.mode in ("FREEZE", "TAKE", "HELD"):
            th = self.hands
        dh = np.clip(th - self.hands, -config.V_HAND * dt, config.V_HAND * dt)
        h_new = np.clip(self.hands + dh, 0, 1)
        if s.hands_ok(now):
            lead = self.lead_cap if self.hand_lead_client is None or self.mode != "CLIENT" else min(self.lead_cap, self.hand_lead_client)
            closing = h_new > self.hands
            cap = s.hand_q + lead
            h_new = np.where(closing & (h_new > cap), np.maximum(cap, np.minimum(self.hands, h_new)), h_new)
        self.hand_still = self.hand_still + 1 if np.max(np.abs(h_new - self.hands)) < 1e-4 else 0
        self.hands = h_new
        # head
        if self.head_target is not None:
            self.head_cmd = self.head_step(dt)
        # gravity: what the PD must be told so the arm ends up at des
        self.grav = self.comp.offset(self.des, self.hands, self.kp, self.payload)
        # friction (only when asked): lead along the planned velocity, low-passed; decays to 0 when the plan stops
        lead = self.comp.friction_lead(self.vel, self.kp, self.fric_alpha)
        if self.fric_gate and self.fric_alpha > 0:
            # only while the joint LAGS its plan in the planned direction: all of it once it lags a friction dead band
            # (f / kp), none when on or past the plan -- so the lead cannot carry a joint past the plan as a motion ends
            e0 = np.maximum(self.comp.friction / self.kp, 1e-4)
            lead = lead * np.clip((self.des - s.q) * np.sign(self.vel) / e0, 0.0, 1.0)
        self.fric += (lead - self.fric) * min(1.0, dt / config.FRIC_TAU)
        self.sent = config.clamp_arm(self.des + self.grav + self.fric)
        if self.mode != "WAIT_TAKE":
            # the planned speed as the motors' speed target (dqff on): only while the arms are ours and moving on a plan
            dq = self.vel.copy() if (self.dq_ff and self.mode in ("CLIENT", "REWIND", "RETURN", "GO_HOME", "HOME")) else None
            if dq is not None:
                dq[config.WAIST] = 0.0
            self.io.send_arm(self.sent, self.head_cmd, self.weight, self.kp, self.kd, dq)
            if self.cycle % config.HAND_EVERY == 0 and self.hands_init:
                self.io.send_hands(self.hands[:6], self.hands[6:])
        self._broadcast(s, now)
        self._log(s, now)
        self.cycle += 1

    def hand_forces(self, s):
        """|force| (N) on each hand's palm from the joint lag minus gravity (the fitted model), static estimate."""
        if self.sent is None:
            return {"left": 0.0, "right": 0.0}
        rob = self.comp.rob
        T = rob.fk(s.q, s.hand_q)
        tau = self.kp * (self.sent - s.q) - self.kd * s.dq
        g = self.comp.scale * rob.gravity11(s.q, s.hand_q, T=T) + self.comp.bias_nm
        if self.payload:
            pl = self.payload
            M = T.get(pl.get("link"))
            if M is not None:
                c = M[:3, :3] @ np.asarray(pl.get("p", (0, 0, 0)), float) + M[:3, 3]
                sl_ = config.LEFT if pl["side"] == "left" else config.RIGHT
                g[sl_] += rob.gravity(T, pl["side"], extra=[(float(pl["mass"]), c)]) - rob.gravity(T, pl["side"])
        out = {}
        for side, sl in (("left", config.LEFT), ("right", config.RIGHT)):
            M = T["%s_base_link" % side]
            p = M[:3, :3] @ np.array([0.0, 0.0, 0.08]) + M[:3, 3]          # middle of the hand
            J = rob.jacobian(T, side, p)
            out[side] = float(np.linalg.norm(force_pinv(J, tau[sl] - g[sl])))
        return out

    def track(self, tgt, v_lim, dt, now=None):
        """Move the planned pose toward a (possibly moving) target: speed <= v_lim, acceleration <= A_MAX, stops at a
        still target without overshoot. A streamed path arrives in steps every 10-30 ms: its speed is estimated from
        when the target CHANGES, fed forward, and the target is carried on at that speed between updates (at most
        60 ms), so the path is followed without lag; a target that jumps is approached at the limits."""
        now = time.time() if now is None else now
        if self.last_tgt is None or self.mode in ("FREEZE", "TAKE", "HELD", "RELEASE"):
            self.last_tgt, self.tgt_t, self.v_tgt = tgt.copy(), now, np.zeros(config.N_ARM)
        if np.max(np.abs(tgt - self.last_tgt)) > 1e-7:
            v = (tgt - self.last_tgt) / max(now - self.tgt_t, dt)
            jump = np.abs(v) > 2 * v_lim                   # a jump, not a motion: no feed-forward
            v[jump] = 0.0
            self.v_tgt = np.clip(0.5 * self.v_tgt + 0.5 * v, -v_lim, v_lim)
            self.v_tgt[jump] = 0.0
            self.last_tgt, self.tgt_t = tgt.copy(), now
        elif now - self.tgt_t > 0.06:
            self.v_tgt = np.zeros(config.N_ARM)
        v_tgt = self.v_tgt
        tgt = tgt + v_tgt * min(now - self.tgt_t, 0.06)  # carried on between updates
        d = tgt - self.des
        v_stop = np.sqrt(2 * config.A_MAX * np.abs(d)) + np.abs(v_tgt)
        v_ref = np.clip(v_tgt + 10.0 * d, -np.minimum(v_lim, v_stop), np.minimum(v_lim, v_stop))
        self.vel = self.vel + np.clip(v_ref - self.vel, -config.A_MAX * dt, config.A_MAX * dt)
        step_ = self.vel * dt
        over = (np.abs(step_) > np.abs(d)) & (np.abs(v_tgt) < 1e-6)
        step_[over] = d[over]
        self.vel[over] = 0.0
        return config.clamp_arm(self.des + step_)

    # ------------------------------------------------------------------ out
    def state_msg(self, s, now):
        return dict(op="state", t=round(now, 4), mode=self.mode, owner=self.owner["name"] if self.owner else None,
                    head_owner=self.head_owner["name"] if self.head_owner else None,
                    freeze=self.freeze_reason if self.mode == "FREEZE" else "", tag=self.tag,
                    q=_r(s.q), dq=_r(s.dq, 4), tau=_r(s.tau, 3), des=_r(self.des if self.des is not None else s.q),
                    sent=_r(self.sent if self.sent is not None else s.q), grav=_r(self.grav),
                    kp=_r(self.kp, 2), kd=_r(self.kd, 2), fric=_r(self.fric), weight=round(self.weight, 3),
                    head=_r(s.head), head_cmd=_r(self.head_cmd if self.head_cmd is not None else s.head), waist_roll=round(float(s.waist_roll), 5),
                    legs=_r(s.legs, 4),
                    rpy=_r(s.rpy), hands=_r(s.hand_q, 4), hands_cmd=_r(self.hands, 4), hands_ok=bool(s.hands_ok(now)),
                    touch=[[_r(f, 1) for f in side] for side in s.touch] if s.touch_ok(now) else None,
                    plate=self.plate, plate_age=round(now - self.plate_t, 3) if self.plate is not None else None,
                    age=round(s.age(now), 4), history=len(self.history), req=self.req,
                    f_hand={k: round(v, 1) for k, v in self.f_hand.items()},
                    home=_r(self.home_q), home_hands=_r(self.home_h))

    def _broadcast(self, s, now):
        if not self.subs:
            return
        msg = json.dumps(self.state_msg(s, now)).encode()
        for addr, t in list(self.subs.items()):
            if now - t > 5.0:                               # a client that has not spoken for 5 s is dropped
                del self.subs[addr]
                continue
            try:
                self.sock.sendto(msg, addr)
            except OSError:
                pass

    def _log_header(self):
        cols = ["wall", "traj_t", "stage", "phase", "weight"]
        cols += ["cmd_" + j for j in config.ARM_NAMES] + ["q_" + j for j in config.ARM_NAMES] + ["tau_" + j for j in config.ARM_NAMES]
        cols += ["des_" + j for j in config.ARM_NAMES]
        cols += ["%s_%s_%s" % (w, s, f) for s in "LR" for w in ("hcmd", "hq", "hcur") for f in config.FINGERS]
        cols += ["touch_%s_%s" % (s, f) for s in "LR" for f in config.TOUCH_FINGERS]
        cols += ["imu_roll", "imu_pitch", "cmd_head_pitch", "cmd_head_yaw", "q_head_pitch", "q_head_yaw", "plate_x", "plate_y", "plate_z"]
        cols += ["kp_scale", "kd_scale"] + ["fric_" + j for j in config.ARM_NAMES]    # cmd = des + gravity + fric
        cols += ["prox_%s_%s" % (s, f) for s in "LR" for f in config.TOUCH_FINGERS]    # fingertip proximity, raw counts
        cols += ["dq_ff"]                                                               # 1: the planned speed went to the motors as dq
        self.log.write(",".join(cols) + "\n")
        self.t0 = time.time()

    def _log(self, s, now):
        if self.mode == "HOME" and self.cycle % 50:          # 2 Hz while idle at home, 100 Hz otherwise
            return
        p = self.plate.get("center") if isinstance(self.plate, dict) else None
        row = ["%.3f" % (now - self.t0), "%.3f" % (now - self.owner["t0"]) if self.owner and "t0" in self.owner else "0", self.mode, self.tag or "-",
               "%.2f" % self.weight]
        row += ["%.5f" % v for v in self.sent] + ["%.5f" % v for v in s.q] + ["%.3f" % v for v in s.tau] + ["%.5f" % v for v in self.des]
        for k in range(2):
            row += ["%.3f" % v for v in self.hands[6 * k:6 * k + 6]] + ["%.3f" % v for v in s.hand_q[6 * k:6 * k + 6]] + \
                   ["%.3f" % v for v in s.hand_cur[6 * k:6 * k + 6]]
        row += ["%.0f" % v for v in s.touch[:, :, 0].ravel()]
        row += ["%.4f" % s.rpy[0], "%.4f" % s.rpy[1], "%.4f" % self.head_cmd[0], "%.4f" % self.head_cmd[1], "%.4f" % s.head[0], "%.4f" % s.head[1]]
        row += ["%.4f" % v for v in (p if p else (float("nan"),) * 3)]
        row += ["%.3f" % self.kp_scale, "%.3f" % self.kd_scale] + ["%.5f" % v for v in self.fric]
        row += ["%.0f" % v for v in s.touch[:, :, 3].ravel()]
        row += ["1" if self.dq_ff else "0"]
        self.log.write(",".join(row) + "\n")
        if self.cycle % 100 == 0:
            self.log.flush()

    def close(self):
        try:
            self.log.close()
        except OSError:
            pass
        self.sock.close()


def udp_no_reset(sock):
    """Windows reports 'connection reset' on a UDP socket after sending to a closed port; Linux does not. Off."""
    if hasattr(socket, "SIO_UDP_CONNRESET"):
        try:
            sock.ioctl(socket.SIO_UDP_CONNRESET, False)
        except (OSError, ValueError):
            pass


def another_server(port, timeout=0.4):
    """True when something answers an operator `status` on the port."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(dict(op="operator", cmd="ping")).encode(), (config.HOST, port))
        s.recvfrom(65536)
        return True
    except (socket.timeout, OSError):
        return False
    finally:
        s.close()


def load_home(path=None):
    """HOME: config.HOME_FILE unless a path (or "pregrip" / "neutral") is given."""
    path = {"pregrip": config.PREGRIP_FILE, "neutral": config.HOME_FILE, None: config.HOME_FILE}.get(path, path)
    h = json.load(open(path))
    h["name"] = os.path.splitext(os.path.basename(path))[0]
    return h
