"""End-to-end test of the arm server with the simulated robot (no DDS).

    python3 tests/test_server_mock.py            # laptop or robot NX; ~1-2 min; writes logs/test_server_mock.png
"""
import os
import queue
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from pk import config  # noqa: E402
from pk.client import Client, Denied, ServerGone, operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server, arm_off, load_home  # noqa: E402

PORT = 9711
OK = []


def check(cond, what):
    OK.append((bool(cond), what))
    print(("PASS " if cond else "FAIL ") + what, flush=True)


class Run(object):
    def __init__(self, home=None):
        """home: the server's HOME; these tests were written for the pre-grip pose (HOME until), which stays the
        default here (neutral_returns() checks the neutral HOME)."""
        self.io = MockBackend(grav_scale=1.2, friction=0.3)
        self.srv = Server(self.io, home=home or load_home("pregrip"), port=PORT, verbose=False,
                          log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
        self.trace = []                      # (t, mode, des, q, hands, hand_q)
        self.stop = False
        self.calls = queue.Queue()
        self.th = threading.Thread(target=self.loop, daemon=True)
        self.th.start()

    def call(self, fn, *args):
        """fn(*args) in the server's own thread between two cycles, as server.py handles keys and signals."""
        box = []
        self.calls.put((fn, args, box))
        t0 = time.time()
        while not box and time.time() - t0 < 2.0:
            time.sleep(0.005)
        return box[0] if box else None

    def loop(self):
        nxt = time.time()
        while not self.stop:
            while not self.calls.empty():
                fn, args, box = self.calls.get()
                box.append(fn(*args))
            self.srv.step()
            s = self.io.s
            if self.srv.des is not None:
                # the server's own clock for this cycle (a thread's time.time() after the step is off by the step's length)
                self.trace.append((self.srv.t_last, self.srv.mode, self.srv.des.copy(), s.q.copy(), self.srv.hands.copy(), s.hand_q.copy()))
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))

    def wait_mode(self, mode, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.srv.mode == mode:
                return True
            time.sleep(0.01)
        return False


def weight_safety():
    """ on the robot: `take` froze at weight 0 (state looked stale after the SDK's publisher pause), `home`
    then 'moved' at weight 0, and `release` ramped from 1 -> the arms jumped to the pre-grip pose. Never again."""
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    q0 = R.io.s.q.copy()
    operator("take", port=PORT)
    ok = R.wait_mode("HELD", 6)
    check(ok and srv.weight >= 0.999, "take reaches full weight despite the 0.6 s publisher pause (mode %s, weight %.2f)" % (srv.mode, srv.weight))
    # freeze half way through a second take: release from there must only go DOWN and must not move the arms
    operator("release", port=PORT)
    t0 = time.time()
    while srv.mode == "RELEASE" and time.time() - t0 < 5:
        time.sleep(0.05)
    R.stop = True
    time.sleep(0.1)
    srv.close()
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    time.sleep(0.6 + 0.5)                       # pause + a quarter of the ramp
    operator("f", port=PORT)
    w_frozen = srv.weight
    time.sleep(0.3)
    operator("home", port=PORT)
    time.sleep(0.5)
    check(srv.mode == "FREEZE", "home refused while the arms are not taken (weight %.2f, mode %s)" % (w_frozen, srv.mode))
    q_before = R.io.s.q.copy()
    ws = []
    operator("release", port=PORT)
    t0 = time.time()
    while time.time() - t0 < 3.5:
        ws.append(srv.weight)
        time.sleep(0.02)
    rise = max(b - a for a, b in zip(ws, ws[1:])) if len(ws) > 1 else 0.0
    check(rise <= 1e-6 and max(ws) <= w_frozen + 1e-6, "release from weight %.2f never raises it (max %.2f)" % (w_frozen, max(ws)))
    moved = float(np.max(np.abs(R.io.s.q - q_before)[:10]))
    check(moved < 0.03, "release from a frozen take moves nothing (max %.3f rad)" % moved)
    R.stop = True
    time.sleep(0.1)
    srv.close()


def operator_modes():
    """ on the robot: `home` reached the pose but never ended (the waist, 0.010 rad where the arms were
    taken, counted in the 'arrived' test); `release` typed meanwhile did nothing and said nothing; Ctrl-C pressed more
    than 3 s after the previous one did nothing at all. Now every command answers, release works from every mode,
    `take` stops a release, and Ctrl-C is freeze, then release on a 2nd press within 3 s, whatever came before."""
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    for cmd in ("f", "release", "home", "status", "nonsense"):
        n = srv.said
        operator(cmd, port=PORT)
        check(srv.said > n and srv.mode == "WAIT_TAKE", "nothing taken: `%s` answers (%s)" % (cmd, srv.notes[-1][9:70]))
    w = float(R.io.s.q[config.WAIST])
    check(abs(w - srv.home_q[config.WAIST]) > 0.005, "the simulated waist reads %.4f rad, the pose file %.4f (as the robot)" % (
        w, srv.home_q[config.WAIST]))
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    operator("home", port=PORT)
    time.sleep(1.0)
    check(srv.mode == "GO_HOME", "home move under way")
    operator("release", port=PORT)
    des_r = srv.des.copy()
    check(srv.mode == "RELEASE", "release typed DURING the home move -> RELEASE (mode %s)" % srv.mode)
    ws = []
    t0 = time.time()
    while time.time() - t0 < 3.5:
        ws.append(srv.weight)
        time.sleep(0.02)
    check(all(b <= a + 1e-9 for a, b in zip(ws, ws[1:])) and ws[-1] == 0.0, "... the weight ramps down only, to 0")
    check(arm_off(srv.des, des_r) < 1e-9, "... and no arm motion is planned meanwhile")
    for cmd in ("f", "release", "home", "take", "quit", "nonsense"):
        n = srv.said
        operator(cmd, port=PORT)
        check(srv.said > n and srv.mode == "RELEASE", "released: `%s` answers (%s)" % (cmd, srv.notes[-1][9:70]))
    R.stop = True
    time.sleep(0.1)
    srv.close()

    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    t_h = time.time()
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 30), "home ENDS (HOME after %.1f s) although the waist is %.4f rad off the pose file" % (
        time.time() - t_h, abs(srv.des[config.WAIST] - srv.home_q[config.WAIST])))
    for cmd in ("home", "take"):
        n = srv.said
        operator(cmd, port=PORT)
        check(srv.said > n and srv.mode == "HOME", "at home: `%s` answers (%s)" % (cmd, srv.notes[-1][9:70]))
    # Ctrl-C: freeze; one 4 s later only answers; one within 3 s of that releases
    now = time.time()
    R.call(srv.ctrl_c, now - 6.0)
    check(srv.mode == "FREEZE", "Ctrl-C at home -> FREEZE")
    n = srv.said
    R.call(srv.ctrl_c, now - 2.0)
    check(srv.mode == "FREEZE" and srv.said > n, "Ctrl-C 4 s later: still frozen, and says what the next press does")
    n = srv.said
    operator("f", port=PORT)
    check(srv.said > n and srv.mode == "FREEZE", "frozen: `f` answers (%s)" % srv.notes[-1][9:70])
    R.call(srv.ctrl_c, now)
    check(srv.mode == "RELEASE", "a 2nd Ctrl-C within 3 s -> RELEASE")
    time.sleep(1.0)
    w_mid = srv.weight
    operator("take", port=PORT)
    check(srv.mode in ("TAKE", "HELD"), "`take` during the release stops it (weight was %.2f)" % w_mid)
    check(R.wait_mode("HELD", 5) and srv.weight >= 0.999 and not srv.release_done, "... and takes the arms back (HELD, weight %.2f)" % srv.weight)
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 30), "home again from there")
    # a test owns the arms: release takes them from it
    c = Client("release test", port=PORT)
    q1 = srv.home_q.copy()
    q1[0] += 0.1
    c.move_to(q1, v=0.3)
    operator("release", port=PORT)
    check(srv.mode == "RELEASE" and srv.owner is None, "release while a test owns the arms -> RELEASE, the test loses them")
    c.close()
    time.sleep(0.5)
    operator("take", port=PORT)
    R.wait_mode("HELD", 5)
    operator("home", port=PORT)
    R.wait_mode("HOME", 30)
    # a killed test: release during the rewind
    c = Client("rewind test", port=PORT)
    q2 = srv.home_q.copy()
    q2[3] += 0.2
    c.move_to(q2, v=0.3)
    c._stop.set()
    check(R.wait_mode("REWIND", 2.0), "killed test -> REWIND")
    time.sleep(0.3)
    operator("release", port=PORT)
    check(srv.mode == "RELEASE", "release during the rewind -> RELEASE (mode %s)" % srv.mode)
    R.stop = True
    time.sleep(0.1)
    srv.close()
    # quit: only while nothing is taken
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    stop = R.call(srv.ctrl_c, time.time())
    check(stop is True and srv.mode == "WAIT_TAKE", "Ctrl-C with nothing taken stops the server")
    operator("quit", port=PORT)
    check(srv.quit, "`quit` with nothing taken stops the server")
    R.stop = True
    time.sleep(0.1)
    srv.close()


def limp():
    """ on the robot: its mode ignored arm_sdk (every motor <= 0.11 N m), and `home` froze 1.7 s in
    saying only 'blocked?'. Now a warning once the arms are taken, and the freeze says why."""
    R = Run()
    srv = R.srv
    R.io.limp = True
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    time.sleep(1.0)
    check(srv.limp and any("LIMP" in n for n in srv.notes), "limp arms after take: warned (%s)" % srv.notes[-1][9:80])
    operator("home", port=PORT)
    check(R.wait_mode("FREEZE", 5) and "NO motor torque" in srv.freeze_reason, "home with limp arms -> FREEZE saying why (%s)" % srv.freeze_reason[:60])
    R.io.limp = False                       # the robot put in its running mode
    time.sleep(0.5)
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 30), "running mode again: home works from that freeze")
    R.stop = True
    time.sleep(0.1)
    srv.close()
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    time.sleep(1.0)
    check(not srv.limp, "arms really held (motor torque %.1f N m): no limp warning" % np.max(np.abs(R.io.s.tau[:10])))
    R.stop = True
    time.sleep(0.1)
    srv.close()


def tuning():
    """: `fric A` (friction lead along the planned motion) and `gains P D` (arm kp / kd scales), between tests
    only; the gravity offset stays right at a scaled kp; every log row says what the PD had, and the fit reads it."""
    from pk import fit as F
    from pk import logs as L
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    operator("home", port=PORT)
    R.wait_mode("HOME", 40)
    check(srv.fric_alpha == 0.0 and np.all(srv.fric == 0.0) and np.allclose(srv.kp, config.KP), "default: no friction lead, Unitree's gains")
    for cmd in ("fric x", "gains 1.5", "fric"):
        n = srv.said
        operator(cmd, port=PORT)
        check(srv.said > n and "usage" in srv.notes[-1] and srv.fric_alpha == 0.0 and srv.kp_scale == 1.0, "`%s` -> usage, nothing changed" % cmd)
    operator("gains 5 5", port=PORT)
    check(srv.kp_scale == config.GAIN_SCALE_KP[1] and srv.kd_scale == config.GAIN_SCALE_KD[1], "`gains 5 5` clipped to kp x%.1f, kd x%.1f" % (srv.kp_scale, srv.kd_scale))
    operator("gains 1.5 1.22", port=PORT)
    operator("fric 0.5 raw", port=PORT)
    arm = np.arange(config.N_ARM) != config.WAIST
    check(np.allclose(srv.kp[arm], 1.5 * config.KP[arm]) and np.allclose(srv.kd[arm], 1.22 * config.KD[arm])
          and srv.kp[config.WAIST] == config.KP[config.WAIST] and srv.kd[config.WAIST] == config.KD[config.WAIST],
          "`gains 1.5 1.22`: arm kp x1.5, kd x1.22, the waist unchanged")
    check(srv.fric_alpha == 0.5 and not srv.fric_gate and "friction lead x0.50" in srv.status_line(), "`fric 0.5 raw` set, and in the status line")
    time.sleep(1.5)
    err = np.max(np.abs(R.io.s.q - srv.home_q)[:10])
    check(err < 0.03, "kp x1.5: the gravity offset still holds the pre-grip pose (max |q - plan| %.4f rad)" % err)
    c = Client("tuning test", port=PORT)
    n = srv.said
    operator("fric 0", port=PORT)
    operator("gains 1 1", port=PORT)
    check(srv.said >= n + 2 and srv.fric_alpha == 0.5 and srv.kp_scale == 1.5 and "refused" in srv.notes[-1],
          "while a test owns the arms: `fric` / `gains` refused (%s)" % srv.notes[-1][9:70])
    q1 = srv.home_q.copy()
    q1[0] += 0.2
    leads, gaps = [], []

    def watch(st):
        if abs(srv.vel[0]) > 0.09:                     # cruising at 0.1 rad/s
            leads.append(srv.fric[0])
            # read in the server's thread between two cycles (from here a cycle can land between the four reads)
            gaps.append(R.call(lambda: float(np.max(np.abs(srv.sent - srv.des - srv.grav - srv.fric)[:10]))))
        return ""
    c.move_to(q1, v=0.1, check=watch)
    want = 0.5 * srv.comp.friction[0] * np.tanh(0.1 / config.FRIC_V0) / srv.kp[0]
    check(len(leads) > 20 and abs(np.median(leads) - want) < 0.15 * want,
          "moving L_sh_pitch at 0.1 rad/s: friction lead %.4f rad (0.5 x %.2f N m / kp %.0f = %.4f), in the direction of motion" % (
              np.median(leads) if leads else float("nan"), srv.comp.friction[0], srv.kp[0], want))
    gaps = [g for g in gaps if g is not None]
    check(len(gaps) > 10 and max(gaps) < 1e-9, "sent = plan + gravity + friction lead, nothing else (%d cycles, max gap %.1e rad)" % (
        len(gaps), max(gaps) if gaps else float("nan")))
    c.hold(0.4)
    check(np.max(np.abs(srv.fric)) < 1e-4, "the plan stopped: the lead decayed to %.1e rad within 0.4 s" % np.max(np.abs(srv.fric)))
    # gated (`fric A`): the lead only while the joint lags its plan, never past it
    srv.fric_gate = True
    gated, lags = [], []

    def watch2(st):
        if abs(srv.vel[0]) > 0.09:
            gated.append(abs(srv.fric[0]))
            lags.append(float((srv.des[0] - R.io.s.q[0]) * np.sign(srv.vel[0])))
        return ""
    c.move_to(srv.home_q, v=0.1, check=watch2)                 # back: L_sh_pitch -0.2 rad at 0.1 rad/s
    e0 = srv.comp.friction[0] / srv.kp[0]
    check(gated and 0.0 <= np.median(gated) <= want * 1.05 and abs(np.median(gated) - want * min(1.0, max(0.0, np.median(lags) / e0))) < 0.25 * want,
          "gated lead %.4f rad = the raw %.4f x how far the joint lags (%.4f of a dead band %.4f rad)" % (
              np.median(gated) if gated else float("nan"), want, np.median(lags) if lags else float("nan"), e0))
    c.move_to(srv.home_q, v=0.1)
    c.move_to(srv.home_q, v=0.1)
    c.bye()
    R.wait_mode("HOME", 20)
    operator("fric 0", port=PORT)
    operator("gains 1 1", port=PORT)
    check(srv.fric_alpha == 0.0 and np.allclose(srv.kp, config.KP) and np.allclose(srv.kd, config.KD), "between tests: back to off and Unitree's gains")
    R.stop = True
    time.sleep(0.1)
    srv.close()
    lg = L.load(srv.log_path)
    hi = lg["kp"][:, 0] > 1.4 * config.KP[0]
    check(hi.any() and np.allclose(lg["kp"][hi][:, :10], 1.5 * config.KP[:10]) and np.allclose(lg["kp"][hi][:, 10], config.KP[10]),
          "the log records kp x1.5 on the arm joints only (%d rows)" % hi.sum())
    check(np.max(np.abs(lg["fric"][:, 0])) > 0.8 * want, "the log records the friction lead (max %.4f rad)" % np.max(np.abs(lg["fric"][:, 0])))
    i = np.nonzero(hi)[0][:50]
    tau = F.measured_torque(lg, i)
    tau_hand = 1.5 * config.KP * (lg["cmd"][i] - lg["q"][i]) - 1.22 * config.KD * lg["dq"][i]
    check(np.allclose(tau[:, :10], tau_hand[:, :10]), "the gravity fit takes the torque with the gains the log says (kp x1.5, kd x1.22)")


def head_approach():
    """: every head yaw move ends turning + (left) by >= config.HEAD_YAW_APPROACH (the camera's calibration
    was captured that way); a move up in yaw from far below goes straight; a pitch-only move does not touch the yaw."""
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    operator("home", port=PORT)
    R.wait_mode("HOME", 40)
    hc = Client("head test", role="head", port=PORT)
    log = []

    def go(target, timeout=8.0):
        del log[:]
        t0 = time.time()
        while time.time() - t0 < timeout:
            hc.head(target)
            log.append(srv.head_cmd.copy())
            if np.max(np.abs(srv.head_cmd - config.clamp_head(np.array(target)))) < 1e-4 and srv.head_via is None:
                break
            time.sleep(0.02)
        return np.array(log)
    a = config.HEAD_YAW_APPROACH
    go([0.5, 0.30])
    L = go([0.5, 0.0])                                       # down in yaw: past 0 by the approach, then up
    check(L[:, 1].min() <= -a + 0.005 and abs(L[-1, 1]) < 1e-3, "yaw 0.30 -> 0.00: dips to %.3f (<= -%.3f), ends at %.3f" % (L[:, 1].min(), a, L[-1, 1]))
    L = go([0.5, 0.30])                                       # up from far below: straight, ends moving +
    check(L[:, 1].min() >= -1e-3 and abs(L[-1, 1] - 0.30) < 1e-3, "yaw 0.00 -> 0.30: straight up (lowest %.3f)" % L[:, 1].min())
    L = go([0.5, 0.28])                                       # a small step down: still from below
    check(L[:, 1].min() <= 0.28 - a + 0.005 and abs(L[-1, 1] - 0.28) < 1e-3, "yaw 0.30 -> 0.28: via %.3f, ends 0.28" % L[:, 1].min())
    L = go([0.3, 0.28])                                       # pitch only
    check(np.ptp(L[:, 1]) < 1e-3 and abs(L[-1, 0] - 0.3) < 1e-3, "pitch-only move leaves the yaw alone (yaw range %.4f)" % np.ptp(L[:, 1]))
    hc.close()
    R.stop = True
    time.sleep(0.1)
    srv.close()


def path_trim():
    """: a client marks a pose, makes a loop, comes back, trims: the rewind retraces only the way to the mark;
    a trim away from the marked pose is refused."""
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    operator("home", port=PORT)
    R.wait_mode("HOME", 40)
    c = Client("trim test", port=PORT)
    q1 = srv.home_q.copy()
    q1[0] += 0.15
    c.move_to(q1, v=0.3)
    c.mark()
    time.sleep(0.1)
    m = srv.history_mark
    q2 = q1.copy()
    q2[3] += 0.2
    c.move_to(q2, v=0.3)
    n_loop = len(srv.history)
    c.trim()
    time.sleep(0.1)
    check(len(srv.history) == n_loop and "refused" in srv.notes[-1], "trim away from the mark refused (%s)" % srv.notes[-1][9:60])
    c.move_to(q1, v=0.3)
    c.trim()
    time.sleep(0.1)
    check(m is not None and len(srv.history) == m + 1, "back at the mark: path %d -> %d waypoints" % (n_loop, len(srv.history)))
    c.bye()
    ok = R.wait_mode("HOME", 30)
    tr = [x for x in R.trace if x[1] == "REWIND"]
    worst = max(x[2][3] - srv.home_q[3] for x in tr) if tr else 0.0
    check(ok and worst < 0.05, "the rewind went home without the loop (elbow at most %+.3f rad from home)" % worst)
    R.stop = True
    time.sleep(0.1)
    srv.close()


def neutral_returns():
    """: HOME = the neutral pose; a completed client -> RETURN (straight, every joint arriving together, the hands
    keeping their shape until the arms are home, then open); bye(then="path") and `home path` -> REWIND; `home` after a freeze ->
    RETURN; `dqff on` sends the planned speed as the motors' dq target."""
    R = Run(home=load_home())
    srv = R.srv
    time.sleep(0.3)
    check(srv.home_name == "home_neutral", "default HOME is the neutral pose (%s)" % srv.home_name)
    operator("take", port=PORT)
    R.wait_mode("HELD", 6)
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 40), "first home -> the neutral pose")
    check(max(srv.hands) < 0.01, "hands open at HOME")
    # a client: to the pre-grip pose and a closed hand, then bye -> RETURN
    c = Client("neutral test", port=PORT)
    why = c.to_pregrip(v=0.4)
    check(why == "" and arm_off(srv.des, load_home("pregrip")["q"]) < 0.005, "Client.to_pregrip reaches the pre-grip pose (%s)" % (why or "ok"))
    q, h = c.target()
    h = h.copy()
    h[2] = h[8] = 0.6
    c.set(hands=h)
    c.hold(1.0)
    q_from = srv.des.copy()
    t_b = time.time()
    c.bye()
    check(R.wait_mode("RETURN", 1.0), "completed client (bye) -> RETURN")
    check(R.wait_mode("HOME", 20), "RETURN ends at HOME after %.1f s" % (time.time() - t_b))
    tr = [x for x in R.trace if x[0] >= t_b and x[1] == "RETURN"]
    line = srv.home_q - q_from
    j = int(np.argmax(np.abs(line[:10])))
    off = 0.0
    for x in tr:
        u = (x[2][j] - q_from[j]) / line[j]
        off = max(off, float(np.max(np.abs((x[2] - (q_from + u * line))[:10]))))
    check(off < 0.01, "RETURN is a straight line in joint space (max %.4f rad off it)" % off)
    v = max(np.max(np.abs(b[2] - a[2])[:10]) / max(b[0] - a[0], 1e-3) for a, b in zip(tr, tr[5:]))
    check(v <= config.V_RETURN * 1.15, "RETURN speed %.2f rad/s <= %.2f" % (v, config.V_RETURN))
    moving = [x for x in tr if arm_off(x[2], srv.home_q) > 0.01]
    check(moving and min(x[4][2] for x in moving) > 0.55, "the hands keep their shape while the arms move (index >= %.2f)" % (
        min(x[4][2] for x in moving) if moving else -1))
    t_h = time.time()                                       # HOME is declared with the hands within 0.03; they finish opening there
    while max(srv.hands) >= 0.01 and time.time() - t_h < 1.5:
        time.sleep(0.05)
    check(max(srv.hands) < 0.01, "... and are open at HOME (%.2f s after)" % (time.time() - t_h))
    # bye(then="path") -> REWIND along the path
    c = Client("neutral test 2", port=PORT)
    c.to_pregrip(v=0.4)
    t_b = time.time()
    c.bye(then="path")
    check(R.wait_mode("REWIND", 1.0), "bye(then='path') -> REWIND")
    check(R.wait_mode("HOME", 30), "REWIND ends at HOME (%.1f s)" % (time.time() - t_b))
    # freeze, then `home path` -> REWIND; again, `home` -> RETURN
    for cmd, mode in (("home path", "REWIND"), ("home", "RETURN")):
        c = Client("neutral test 3", port=PORT)
        c.to_pregrip(v=0.4)
        operator("f", port=PORT)
        R.wait_mode("FREEZE", 1.0)
        operator(cmd, port=PORT)
        check(R.wait_mode(mode, 1.0), "after a freeze `%s` -> %s" % (cmd, mode))
        check(R.wait_mode("HOME", 30), "... and HOME")
        c.close()
    # dqff: the planned speed reaches the (simulated) motors as dq
    operator("dqff on", port=PORT)
    c = Client("neutral test 4", port=PORT)
    dqs = []
    th = threading.Thread(target=lambda: c.to_pregrip(v=0.4), daemon=True)
    th.start()
    t0 = time.time()
    while th.is_alive() and time.time() - t0 < 15:
        dqs.append(float(np.max(np.abs(getattr(R.io, "cmd_dq", np.zeros(11))))))
        time.sleep(0.02)
    check(max(dqs) > 0.1, "dqff on: the motors get the planned speed as dq (max %.2f rad/s)" % max(dqs))
    c.bye()
    R.wait_mode("HOME", 20)
    operator("dqff off", port=PORT)
    c = Client("neutral test 5", port=PORT)
    dqs = []
    th = threading.Thread(target=lambda: c.to_pregrip(v=0.4), daemon=True)
    th.start()
    t0 = time.time()
    while th.is_alive() and time.time() - t0 < 15:
        dqs.append(float(np.max(np.abs(getattr(R.io, "cmd_dq", np.zeros(11))))))
        time.sleep(0.02)
    check(max(dqs) == 0.0, "dqff off: dq 0 (max %.2f)" % max(dqs))
    c.bye()
    R.wait_mode("HOME", 20)
    R.stop = True
    time.sleep(0.1)
    srv.close()


def main():
    neutral_returns()
    path_trim()
    head_approach()
    weight_safety()
    operator_modes()
    limp()
    tuning()
    R = Run()
    srv = R.srv
    time.sleep(0.3)
    q_hang = R.io.s.q.copy()
    operator("take", port=PORT)
    check(R.wait_mode("HELD", 5), "take: weight ramp done, arms HELD")
    moved = np.max(np.abs(R.io.s.q - q_hang))
    check(moved < 0.03, "take moves nothing (max %.3f rad)" % moved)
    t_home = time.time()
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 40), "home: reached the pre-grip pose")
    tr = [x for x in R.trace if x[0] >= t_home]
    v = max(np.max(np.abs(b[2] - a[2])) / max(b[0] - a[0], 1e-3) for a, b in zip(tr, tr[5:]))      # over ~50 ms
    check(v <= config.V_HOME * 1.15, "home move speed %.3f rad/s <= %.2f" % (v, config.V_HOME))
    time.sleep(1.0)
    err = np.max(np.abs(R.io.s.q - srv.home_q)[:10])
    check(err < 0.03, "gravity-compensated hold at the pre-grip pose: max |q - plan| %.4f rad" % err)

    # ---- a client takes the arms and moves along a path
    c = Client("test path", port=PORT)
    check(srv.owner is not None and srv.mode == "CLIENT", "client got the arms")
    try:
        Client("intruder", port=PORT)
        check(False, "second client refused")
    except Denied as ex:
        check(True, "second client refused (%s)" % ex)
    q0 = srv.home_q.copy()
    q1 = q0.copy()
    q1[0] += 0.25
    q1[3] += 0.2
    q1[6] -= 0.15
    t_path = time.time()
    why = c.move_to(q1, v=0.3, tag="out")
    check(why == "", "client move done")
    q2 = q1.copy()
    q2[1] -= 0.2
    c.move_to(q2, v=0.3, tag="out2")
    fwd = [x for x in R.trace if x[0] >= t_path]
    # jump refused
    big = q2.copy()
    big[2] += 0.8
    c.set(q=big)
    c.hold(0.3)
    check(arm_off(srv.des, q2) < 0.01, "a 0.8 rad jump is refused (planned pose stayed)")
    c.set(q=q2)
    # joint limit: walk R_sh_roll (upper limit 0.227 rad, near its pre-grip value) past its limit slowly
    lim = config.ARM_HI[6] - config.LIMIT_MARGIN
    q3 = q2.copy()
    for k in range(120):
        q3[6] = min(q3[6] + 0.004, lim + 0.15)
        c.set(q=q3)
        c.hold(0.02)
    c.hold(0.8)
    check(srv.des[6] <= lim + 1e-6 and srv.des[6] > lim - 0.01, "joint limit held (R_sh_roll planned %.4f, limit %.4f)" % (srv.des[6], lim))
    c.move_to(q2, v=0.3)
    # speed limit: a client asking 2 rad/s
    t_fast = time.time()
    q4 = q2.copy()
    q4[5] += 0.2
    c.set(vmax=5.0)
    for k in range(10):
        q4f = q2.copy()
        q4f[5] += 0.02 * (k + 1)
        c.set(q=q4f)
        c.hold(0.01)
    c.hold(1.0)
    tr = [x for x in R.trace if x[0] >= t_fast]
    v = max(np.max(np.abs(b[2] - a[2])) / max(b[0] - a[0], 1e-3) for a, b in zip(tr, tr[5:]))      # over ~50 ms
    check(v <= config.V_MAX * 1.1, "speed limit: %.2f rad/s <= %.2f even when the client asks for more" % (v, config.V_MAX))
    c.move_to(q2, v=0.3)
    # hand lead cap: an obstacle stops the right index at 0.3; command it fully closed
    R.io.finger_stop[6 + 2] = 0.3
    h = np.array(c.target()[1])
    h[6 + 2] = 1.0
    c.set(hands=h)
    c.hold(2.0)
    lead = srv.hands[8] - R.io.s.hand_q[8]
    check(lead <= config.HAND_LEAD_MAX + 0.01, "closing finger stays within %.2f of where it is (lead %.3f, cmd %.3f, at %.3f)" % (
        config.HAND_LEAD_MAX, lead, srv.hands[8], R.io.s.hand_q[8]))
    h[6 + 2] = 0.0
    c.set(hands=h)
    c.hold(0.8)
    R.io.finger_stop[:] = 2.0

    # ---- kill the client mid-motion: the server must retrace the path home
    q5 = q2.copy()
    q5[0] -= 0.2
    t_kill = time.time()
    def move_until_killed():
        try:
            c.move_to(q5, v=0.25)
        except (ServerGone, Denied):         # the 'killed' client's own thread notices the silence: expected
            pass
    th = threading.Thread(target=move_until_killed, daemon=True)
    th.start()
    time.sleep(0.6)
    c._stop.set()                         # what a kill looks like from the server: silence
    # (a client whose main thread stops calling it for > 1 s goes silent the same way: hang guard)
    check(R.wait_mode("REWIND", 2.0), "killed client -> REWIND within %.2f s" % config.CLIENT_TIMEOUT_S)
    t_rw = time.time()
    check(R.wait_mode("HOME", 60), "rewind ended at the pre-grip pose")
    rew = [x for x in R.trace if x[0] >= t_rw and x[1] == "REWIND"]
    # every rewind pose lies near the forward path (it retraces it)
    fwd_all = [x[2] for x in R.trace if t_path <= x[0] <= t_rw]
    dmax = max(min(np.max(np.abs(p[2] - f)) for f in fwd_all[::3]) for p in rew[::5])
    check(dmax < 0.03, "rewind stays on the forward path (max %.3f rad off)" % dmax)
    check(arm_off(srv.des, srv.home_q) < 0.004, "back home exactly")

    # ---- blocked arm -> freeze at the measured pose
    c2 = Client("block test", port=PORT)
    # a hard surface 15 mm under the pushing fingertip: pushes back only while the fingertip is in it
    from pk.model import PINKY_END
    T0 = R.io.rob.fk(R.io.s.q, R.io.s.hand_q)
    z_wall = R.io.rob.point(T0, "left_pinky_distal_link", PINKY_END["left"])[2] - 0.015

    def wall(p, v):
        pen = z_wall - p[2]
        return np.array([0.0, 0.0, max(0.0, 4000.0 * pen - 60.0 * v[2])]) if pen > 0 else np.zeros(3)
    R.io.contact = wall
    q6 = srv.home_q.copy()
    q6[0] += 0.8                          # far into the surface: ~50 N, more than the server's 45 N hand limit
    try:
        why = c2.move_to(q6, v=0.3, timeout=20)
    except Denied as ex:
        why = str(ex)
    check("FREEZE" in why or srv.mode == "FREEZE", "the client is told: %s" % why)
    check((R.wait_mode("FREEZE", 10) or srv.mode == "FREEZE") and ("force" in srv.freeze_reason or "should be" in srv.freeze_reason), "pressing arm -> FREEZE (hand force or tracking error: %s)" % srv.freeze_reason)
    time.sleep(0.5)
    gap = np.max(np.abs(srv.des - R.io.s.q)[:10])
    check(gap < 0.06, "frozen at the measured pose (plan - measured %.3f rad)" % gap)
    R.io.contact = None
    operator("home", port=PORT)
    check(R.wait_mode("HOME", 40), "operator home after a freeze -> back at the pre-grip pose")
    h_before = srv.hands.copy()
    operator("release", port=PORT)
    t0 = time.time()
    ws = []
    while time.time() - t0 < 4.0:
        ws.append(srv.weight)
        time.sleep(0.02)
    check(max(h_before) > 0.3 and max(srv.hands) < 0.01, "release opens the hands (max %.2f -> %.2f)" % (max(h_before), max(srv.hands)))
    check(all(b <= a + 1e-6 for a, b in zip(ws, ws[1:])) and ws[-1] == 0.0, "release ramps the weight down only, to 0")
    R.stop = True
    time.sleep(0.1)
    srv.close()
    plot(R, os.path.join(os.path.dirname(HERE), "logs", "test_server_mock.png"), t_home)
    n_bad = sum(1 for ok, _ in OK if not ok)
    print("\n%d checks, %d failed" % (len(OK), n_bad))
    return 1 if n_bad else 0


def plot(R, path, t0):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    tr = [x for x in R.trace if x[0] >= t0]
    t = np.array([x[0] for x in tr]) - t0
    des = np.array([x[2] for x in tr])
    q = np.array([x[3] for x in tr])
    modes = [x[1] for x in tr]
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for j, c in zip((0, 1, 3, 5, 6), ("C0", "C1", "C2", "C3", "C4")):
        ax[0].plot(t, des[:, j], c, lw=1.5, label="planned " + config.ARM_NAMES[j])
        ax[0].plot(t, q[:, j], c, lw=0.8, ls="--")
    ax[0].set_ylabel("rad (solid planned, dashed measured)")
    ax[0].legend(fontsize=7, ncol=5)
    cols = {"GO_HOME": "#ddeeff", "CLIENT": "#e8f5e0", "REWIND": "#fff0d0", "FREEZE": "#ffd8d8", "HOME": "#f4f4f4"}
    for a in ax:
        start = 0
        for i in range(1, len(modes) + 1):
            if i == len(modes) or modes[i] != modes[start]:
                a.axvspan(t[start], t[i - 1], color=cols.get(modes[start], "white"), lw=0)
                if a is ax[1]:
                    a.text((t[start] + t[i - 1]) / 2, 0.02, modes[start], ha="center", fontsize=7, transform=a.get_xaxis_transform())
                start = i
    ax[1].plot(t, np.max(np.abs(des - q)[:, :10], axis=1), "k", lw=0.8)
    ax[1].set_ylabel("max |planned - measured| rad")
    ax[1].set_xlabel("s")
    fig.suptitle("arm server with the simulated robot (pk.robot_io.MockBackend, not MuJoCo): home move, a client's path, "
                 "client killed -> rewind along the path, blocked arm -> freeze", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("wrote", path)


if __name__ == "__main__":
    sys.exit(main())
