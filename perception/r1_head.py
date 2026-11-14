"""Read the R1's head and waist angles, and tilt its head, over the Unitree SDK.

    python3 r1_head.py eth0                       # read the head and waist, once (moves nothing)
    python3 r1_head.py eth0 --watch               # ...keep reading
    python3 r1_head.py eth0 --down 20             # PLAN a tilt to 20 deg down: prints, moves nothing
    python3 r1_head.py eth0 --down 20 --go        # tilt, hold until Enter, then hand the head back
    python3 r1_head.py eth0 --go --sequence       # read "PITCH YAW" lines, go to each (sweeps)

`eth0` is the robot computer's network interface on 192.168.123.x (see `ip -4 addr`).

WHY THE HEAD HAS TO MOVE
    At the CAD head angle the camera looks 34 deg down and sees the table from ~0.6 m out --
    past the arm's reach (about 0.36 m from the shoulder). Tilted ~30 deg further down, it
    sees the stack the hand can actually get to. And once it moves, the perception has to
    know by how much, every frame: that is what the reading half of this file is for.

HOW THE HEAD IS MOVED
    Through rt/arm_sdk, the same channel Unitree's own R1 teleoperation uses. The robot's own
    controller keeps balancing; arm_sdk blends our command into the upper body by a weight,
    msg.mode_pr, 0..100 % (on the R1 -- the G1 uses slot 29 for this, which on the R1 is the
    head pitch motor; see r1_config). While the weight is up, EVERY joint arm_sdk covers
    (both arms, the waist, the head) follows our command, so every one of them is commanded
    to stay exactly where it was measured when we engaged. Only the head target moves.

    - the weight ramps up over 1 s, and down over 1 s at the end;
    - the head moves at 15 deg/s at most, inside its URDF limits less 2 deg;
    - before handing back it returns the head to where it found it, so nothing jumps;
    - if the robot's reports pause (>0.3 s) the head is frozen where it is -- still held,
      no further motion -- and only after 2 s of silence is it handed back, with the
      normal 1 s ramp. A pause is far more often this computer being busy than the robot
      being gone, and letting go at once snaps the head back 30 deg;
    - if the head does not follow (>8 deg behind for 1.5 s), it lets go at once: the
      robot is ignoring the command anyway;
    - every yaw move ENDS travelling toward + (the robot's left), by at least
      YAW_APPROACH_DEG. The yaw gearing has play: turned in from the other side the camera
      sat ~13 mm sideways and 0.9 deg rolled differently (22 Oct). Arriving the same way
      every time takes the play up the same way, so the calibration holds at every pose.
      Anything else that aims this head (the manipulation side) must do the same.

    Run it in its OWN process (check_stack.py --head-down does): Python runs one thread at
    a time, and a depth frame being fitted in the same process can starve this loop.

ONE PUBLISHER AT A TIME
    Anything else publishing rt/arm_sdk -- the arm controller on the manipulation side --
    fights this. Tilt, detect, hand back, THEN move the arm. The plate's position is in
    pelvis_link, so it stays right after the head goes back, as long as the feet do not move.

Needs unitree_sdk2py (on the robot computer). Reading needs only the Ethernet link.
"""
import argparse
import math
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import r1_config as cfg

RATE_HZ = 50.0
MAX_SPEED_DEG_S = 15.0
RAMP_S = 1.0
LIMIT_MARGIN_DEG = 2.0
STALE_S = 0.3            # reports this old: freeze the head where it is
GONE_S = 2.0             # this old: hand it back
FOLLOW_TOL_DEG = 8.0
# Where the head comes to rest, against where it was told. The helmet makes the head
# front-heavy, and at Unitree's gentle head gains (kp 15) it settles ~2-3 deg low of any
# target. That is fine: perception reads the head's real angle every frame. So "there"
# means STOPPED within this of the target, and the real angle is what gets reported.
SETTLE_TOL_DEG = 5.0
STILL_DEG, STILL_S = 0.3, 0.3          # stopped: moved less than this over that long
FOLLOW_GRACE_S = 1.5
YAW_APPROACH_DEG = 4.0                 # every yaw move ends moving + (left) by at least this

# every joint arm_sdk covers on the R1: both arms, the waist, the head
HELD_JOINTS = (cfg.arm_joints("left") + cfg.arm_joints("right") + cfg.TRUNK_JOINTS
               + cfg.HEAD_JOINTS)


def gains(name):
    """(kp, kd) Unitree's R1 teleoperation uses for this joint over arm_sdk."""
    key = name.replace("left_", "").replace("right_", "")
    return cfg.ARM_SDK_GAINS[key]


# ==========================================================================
# the link: state in, commands out
# ==========================================================================
_DDS_UP = False


def init_dds(interface=None):
    """Once per process."""
    global _DDS_UP
    if _DDS_UP:
        return
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    if interface:
        ChannelFactoryInitialize(0, interface)
    else:
        ChannelFactoryInitialize(0)
    _DDS_UP = True


class R1Link:
    """The latest rt/lowstate, kept current in the background."""

    def __init__(self, interface=None, subscribe=True):
        self._lock = threading.Lock()
        self._msg, self._t = None, 0.0
        if subscribe:
            init_dds(interface)
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
            self._sub = ChannelSubscriber(cfg.LOWSTATE_TOPIC, LowState_)
            self._sub.Init(self.feed, 10)

    def feed(self, msg):
        with self._lock:
            self._msg, self._t = msg, time.time()

    def wait(self, timeout=5.0):
        t0 = time.time()
        while self.age() is None:
            if time.time() - t0 > timeout:
                raise RuntimeError(
                    "No %s within %.0f s. Check the Ethernet cable, that this computer has a "
                    "192.168.123.x address, and the interface name." % (cfg.LOWSTATE_TOPIC, timeout))
            time.sleep(0.02)
        return self

    def age(self):
        with self._lock:
            return None if self._msg is None else time.time() - self._t

    def latest(self):
        with self._lock:
            return self._msg

    def joints(self):
        """{joint name: rad} for all 26, from the latest message."""
        msg = self.latest()
        if msg is None:
            raise RuntimeError("no robot state yet")
        return {n: float(msg.motor_state[s].q) for n, s in cfg.MOTOR_SLOT.items()}

    def state(self):
        """{"joints": all 26, "imu_rpy": [r, p, y] (rad) or None}, the latest report."""
        msg = self.latest()
        if msg is None:
            raise RuntimeError("no robot state yet")
        import r1_state
        return r1_state.decode(msg)

    def angles(self, max_age=0.5, wait_s=1.0):
        """The waist and head angles the camera's position depends on, rad. Waits up to
        `wait_s` for a fresh report (a busy process may simply not have heard the latest
        one yet), then refuses: an old angle is a confidently wrong plate position."""
        t0 = time.time()
        while True:
            a = self.age()
            if a is not None and a <= max_age:
                break
            if time.time() - t0 > wait_s:
                raise RuntimeError("robot state is %s old"
                                   % ("never" if a is None else "%.1f s" % a))
            time.sleep(0.01)
        j = self.joints()
        return {n: j[n] for n in cfg.CAMERA_CHAIN_JOINTS}


# ==========================================================================
# the head
# ==========================================================================
class HeadNotFollowing(RuntimeError):
    pass


def clamp_head(name, rad):
    lo, hi = cfg.joint_limits()[name]
    m = math.radians(LIMIT_MARGIN_DEG)
    return float(min(max(rad, lo + m), hi - m))


class HeadController:
    """Tilt the head over rt/arm_sdk and always give it back. Use as a context manager:

        with HeadController(link) as head:
            head.move_to(pitch_deg=25)        # blocks until it is there
            ...detect...
        # head back where it was, weight at 0, robot's own controller in charge

    `publisher`, `new_cmd` and `crc` are injectable so the command logic is testable
    without the SDK (crc=False: no checksum)."""

    def __init__(self, link, publisher=None, new_cmd=None, crc=None, rate_hz=RATE_HZ,
                 speed_deg_s=MAX_SPEED_DEG_S, ramp_s=RAMP_S, log=print):
        self.link, self.rate, self.ramp_s = link, rate_hz, ramp_s
        self.speed = math.radians(speed_deg_s)
        self.log = log
        if publisher is None:
            from unitree_sdk2py.core.channel import ChannelPublisher
            from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
            publisher = ChannelPublisher(cfg.ARM_SDK_TOPIC, LowCmd_)
            publisher.Init()
        if new_cmd is None:
            from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
            new_cmd = unitree_hg_msg_dds__LowCmd_
        if crc is None:
            from unitree_sdk2py.utils.crc import CRC
            crc = CRC().Crc
        self.pub, self.new_cmd, self.crc = publisher, new_cmd, crc
        self.hold, self.start_head = None, None
        self.goal = {}                    # head joint -> target rad (the setpoint walks to it)
        self.weight, self.weight_goal = 0.0, 0.0
        self.error = None
        self._stop = threading.Event()
        self._thread = None
        self._behind_since = None
        self._quiet = False

    # ---------------------------------------------------------------- one message
    def build(self, weight, targets, mode_machine=0):
        """The arm_sdk message for blend `weight` (0..1) and {joint: q} targets."""
        cmd = self.new_cmd()
        cmd.mode_pr = int(round(100 * min(max(weight, 0.0), 1.0)))
        try:
            cmd.mode_machine = int(mode_machine)
        except AttributeError:
            pass
        for name in HELD_JOINTS:
            m = cmd.motor_cmd[cfg.MOTOR_SLOT[name]]
            kp, kd = gains(name)
            m.mode = 1
            m.q = float(targets[name])
            m.dq = 0.0
            m.kp = kp
            m.kd = kd
            m.tau = 0.0
        if self.crc:                       # False: no checksum (tests)
            cmd.crc = self.crc(cmd)
        return cmd

    # ---------------------------------------------------------------- the loop
    def _step(self, dt):
        """Advance the weight and head setpoints one tick, check safety, publish."""
        age = self.link.age()
        if age is None or age > GONE_S:
            raise RuntimeError("no robot state for %.1f s" % (age or -1))
        quiet = age > STALE_S
        if quiet and not self._quiet:
            self.log("r1_head: robot reports paused -- holding the head still")
        self._quiet = quiet
        # weight
        dw = dt / max(self.ramp_s, 1e-3)
        self.weight += max(-dw, min(dw, self.weight_goal - self.weight))
        # head setpoints walk toward their goals at the speed limit -- not while blind
        if not quiet:
            for n, g in self.goal.items():
                q = self.hold[n]
                step = self.speed * dt
                self.hold[n] = q + max(-step, min(step, g - q))
        # does the head follow? (only meaningful with the weight fully up)
        meas = self.link.joints()
        lag = max(abs(meas[n] - self.hold[n]) for n in cfg.HEAD_JOINTS)
        if self.weight > 0.99 and not quiet and lag > math.radians(FOLLOW_TOL_DEG):
            self._behind_since = self._behind_since or time.time()
            if time.time() - self._behind_since > FOLLOW_GRACE_S:
                raise HeadNotFollowing(
                    "the head is %.0f deg from where it is told to be -- the robot is not taking "
                    "arm_sdk commands in this mode, or something is in the way" % math.degrees(lag))
        else:
            self._behind_since = None if not quiet else self._behind_since
        msg = self.link.latest()
        self.pub.Write(self.build(self.weight, self.hold, getattr(msg, "mode_machine", 0)))

    def _run(self):
        period = 1.0 / self.rate
        nxt = time.time()
        try:
            while not self._stop.is_set():
                self._step(period)
                if self.weight_goal == 0.0 and self.weight <= 0.0:
                    break
                nxt += period
                time.sleep(max(0.0, nxt - time.time()))
        except Exception as e:                       # let go and say why
            self.error = e
            self.log("r1_head: letting go -- %s" % e)
            try:
                # a head that is not following can go at once; otherwise the normal ramp
                self.ramp_s = 0.3 if isinstance(e, HeadNotFollowing) else self.ramp_s
                self.weight_goal = 0.0
                while self.weight > 0.0:
                    self.weight = max(0.0, self.weight - period / self.ramp_s)
                    msg = self.link.latest()
                    self.pub.Write(self.build(self.weight, self.hold,
                                              getattr(msg, "mode_machine", 0)))
                    time.sleep(period)
            except Exception:
                pass

    # ---------------------------------------------------------------- public
    def engage(self):
        """Take the upper body at exactly its measured pose, weight ramping up."""
        self.link.wait()
        j = self.link.joints()
        self.hold = {n: j[n] for n in HELD_JOINTS}
        self.start_head = {n: j[n] for n in cfg.HEAD_JOINTS}
        self.weight, self.weight_goal = 0.0, 1.0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def move_to(self, pitch_deg=None, yaw_deg=None, timeout=None):
        """Walk the head to these angles (degrees; pitch + is down) and wait until it has
        stopped, within SETTLE_TOL_DEG of them. Returns where it actually is, in degrees --
        under the helmet's weight that is a few degrees lower than asked. Angles are
        clamped inside the limits. A yaw is always arrived at moving + (see the top)."""
        if self._thread is None:
            raise RuntimeError("engage() first")
        if yaw_deg is not None:
            ty = clamp_head("head_yaw_joint", math.radians(yaw_deg))
            pre = clamp_head("head_yaw_joint", ty - math.radians(YAW_APPROACH_DEG))
            if ty - self.hold["head_yaw_joint"] < math.radians(YAW_APPROACH_DEG) - 1e-6                     and pre < ty - math.radians(1.0):
                self._move(pitch_deg, math.degrees(pre), timeout)   # come in from the right
        return self._move(pitch_deg, yaw_deg, timeout)

    def _move(self, pitch_deg=None, yaw_deg=None, timeout=None):
        if pitch_deg is not None:
            self.goal["head_pitch_joint"] = clamp_head("head_pitch_joint", math.radians(pitch_deg))
        if yaw_deg is not None:
            self.goal["head_yaw_joint"] = clamp_head("head_yaw_joint", math.radians(yaw_deg))
        far = max([abs(g - self.hold[n]) for n, g in self.goal.items()] + [0.0])
        timeout = timeout or (self.ramp_s + far / self.speed + 3.0)
        t0 = time.time()
        recent = []                                  # (time, head angles) for the stillness test
        while True:
            if self.error:
                raise self.error
            if not self._thread.is_alive():
                raise RuntimeError("the head loop stopped")
            meas = self.link.joints()
            now = time.time()
            age = self.link.age()
            if age is None or age > STALE_S:         # a frozen report looks perfectly still
                recent = []
                if now - t0 > timeout:
                    raise HeadNotFollowing("no fresh robot state to tell where the head is")
                time.sleep(0.05)
                continue
            recent = [(t, q) for t, q in recent if now - t <= 1.0]
            recent.append((now, {n: meas[n] for n in cfg.HEAD_JOINTS}))
            there = all(abs(self.hold[n] - g) < 1e-4 for n, g in self.goal.items())
            # still: no more than STILL_DEG of movement since a sample at least STILL_S old
            older = [x for x in recent if now - x[0] >= STILL_S]
            win = ([older[-1]] + [x for x in recent if now - x[0] < STILL_S]) if older else []
            still = bool(win) and all(max(q[n] for _, q in win) - min(q[n] for _, q in win)
                                      < math.radians(STILL_DEG) for n in cfg.HEAD_JOINTS)
            close = all(abs(meas[n] - g) < math.radians(SETTLE_TOL_DEG)
                        for n, g in self.goal.items())
            if there and still and self.weight > 0.99:
                if close:
                    return {n: math.degrees(meas[n]) for n in cfg.HEAD_JOINTS}
                raise HeadNotFollowing(
                    "the head stopped at %s, more than %.0f deg from %s"
                    % ({n: round(math.degrees(meas[n]), 1) for n in self.goal}, SETTLE_TOL_DEG,
                       {n: round(math.degrees(g), 1) for n, g in self.goal.items()}))
            if now - t0 > timeout:
                raise HeadNotFollowing(
                    "the head did not settle near %s in %.0f s (it is at %s)"
                    % ({n: round(math.degrees(g), 1) for n, g in self.goal.items()}, timeout,
                       {n: round(math.degrees(meas[n]), 1) for n in self.goal}))
            time.sleep(0.05)

    def release(self):
        """Head back to where it was found, then the weight down to 0: the robot's own
        controller has it again with nothing to jump to."""
        if self._thread is None:
            return
        if self._thread.is_alive() and not self.error and self.start_head:
            try:
                self.goal = dict(self.start_head)
                self.move_to(timeout=10.0)
            except Exception as e:
                self.log("r1_head: could not return the head first (%s); letting go here" % e)
        self.weight_goal = 0.0
        self._thread.join(timeout=self.ramp_s + 2.0)
        self._stop.set()
        self._thread = None

    def __enter__(self):
        return self.engage()

    def __exit__(self, *exc):
        self.release()
        return False


# ==========================================================================
# command line
# ==========================================================================
def describe(link):
    import camera_calibration as cc
    j = link.joints()
    D = math.degrees
    print("head    pitch %+6.2f deg (+ is down)   yaw %+6.2f deg"
          % (D(j["head_pitch_joint"]), D(j["head_yaw_joint"])))
    print("waist   roll  %+6.2f deg               yaw %+6.2f deg"
          % (D(j["waist_roll_joint"]), D(j["waist_yaw_joint"])))
    cal = cc.load()
    if cal:
        T = cc.T_pelvis_optical(cal, angles={n: j[n] for n in cfg.CAMERA_CHAIN_JOINTS})
        t, r = cc._tilt_roll(T[:3, :3].T @ np.array([0.0, 0.0, 1.0]))
        print("camera  %.1f deg below horizontal, %.2f m above the pelvis, %.2f m forward"
              % (t, T[2, 3], T[0, 3]))


def run_sequence(link, go):
    """Poses from stdin, one per line -- for a program driving the head from another
    process (calibrate_sweep.py), so its own heavy work cannot starve this loop."""
    if not go:
        print("nothing moved. --sequence needs --go")
        return 1
    with HeadController(link) as head:
        print("ready", flush=True)
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            if line == "done":
                break
            try:
                p, y = (float(v) for v in line.split())
            except ValueError:
                print("error: expected 'PITCH YAW', got %r" % line, flush=True)
                continue
            got = head.move_to(p, y)
            print("there: pitch %+.2f deg, yaw %+.2f deg" % (got["head_pitch_joint"],
                                                           got["head_yaw_joint"]), flush=True)
    print("handed back", flush=True)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", help="network interface on 192.168.123.x, e.g. eth0")
    ap.add_argument("--watch", action="store_true", help="keep reading, twice a second")
    ap.add_argument("--down", type=float, metavar="DEG",
                    help="tilt the head to this pitch, degrees down (head limit is 36)")
    ap.add_argument("--yaw", type=float, metavar="DEG", help="and turn it to this yaw")
    ap.add_argument("--go", action="store_true", help="actually move (without it: a plan)")
    ap.add_argument("--hold", type=float, metavar="S",
                    help="hold this many seconds, then hand back (default: until Enter)")
    ap.add_argument("--sequence", action="store_true",
                    help="with --go: read 'PITCH YAW' lines (degrees) from stdin, move to "
                         "each and print 'there: ...'; 'done' or end of input hands back")
    args = ap.parse_args(argv)

    link = R1Link(args.interface).wait()
    describe(link)
    if args.sequence:
        return run_sequence(link, args.go)
    if args.down is None and args.yaw is None:
        while args.watch:
            time.sleep(0.5)
            print("-" * 60)
            describe(link)
        return 0

    j = link.joints()
    tp = clamp_head("head_pitch_joint", math.radians(args.down)) if args.down is not None \
        else j["head_pitch_joint"]
    ty = clamp_head("head_yaw_joint", math.radians(args.yaw)) if args.yaw is not None \
        else j["head_yaw_joint"]
    far = max(abs(tp - j["head_pitch_joint"]), abs(ty - j["head_yaw_joint"]))
    print("\nplan: head pitch %+.1f -> %+.1f deg, yaw %+.1f -> %+.1f deg, %.1f s at %.0f deg/s"
          % (math.degrees(j["head_pitch_joint"]), math.degrees(tp),
             math.degrees(j["head_yaw_joint"]), math.degrees(ty),
             far / math.radians(MAX_SPEED_DEG_S), MAX_SPEED_DEG_S))
    print("      over rt/arm_sdk, weight (mode_pr) 0 -> 100 %% over %.0f s; arms and waist held "
          "where they are now" % RAMP_S)
    if not args.go:
        print("\nnothing moved. Add --go to do it -- with someone at the remote, ready to stop.")
        return 0

    print("\nmoving in 3 s -- Ctrl-C to stop")
    time.sleep(3.0)
    with HeadController(link) as head:
        got = head.move_to(math.degrees(tp), math.degrees(ty))
        print("there: pitch %+.1f deg, yaw %+.1f deg  (asked %+.1f, %+.1f; the helmet's weight "
              "accounts for the difference)" % (got["head_pitch_joint"], got["head_yaw_joint"],
                                               math.degrees(tp), math.degrees(ty)), flush=True)
        describe(link)
        if args.hold is not None:
            time.sleep(args.hold)
        else:
            input("holding. Enter to hand the head back ")
    print("handed back")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
