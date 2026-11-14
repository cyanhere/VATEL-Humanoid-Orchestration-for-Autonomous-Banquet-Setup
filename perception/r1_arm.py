"""Move an R1 arm over rt/arm_sdk, slowly, and never let it drop by surprise.

The same channel and the same rules as r1_head.py (which tilts the head on this robot):
the robot's own controller keeps balancing, our command is blended in by msg.mode_pr,
and while it is up EVERY upper-body joint follows us -- so every one is commanded to stay
exactly where it was measured, and only the joints being moved get new targets.

What differs for an arm:

    - joints move in straight lines in joint space at MAX_SPEED_RAD_S (0.3 rad/s, the G1's
      figure), all arriving together;
    - a move asking any joint for more than MAX_DELTA_RAD (1.5 rad) is refused: a solver
      asking for that is far more likely wrong than meant;
    - trouble FREEZES the arm where it is and keeps holding it. Letting go of an arm is
      not safe the way letting go of a head is: the robot's own controller would swing it
      back to its rest pose, through whatever is in front of it. Only release() lets go,
      and it ramps the weight down over RELEASE_S;
    - "trouble": the robot's reports stop (frozen at once; nothing moves without them), or
      a joint falls more than LAG_RAD behind for LAG_S (something is in the way: the
      target becomes where the arm is, so it stops pushing).

Run it in its own process from anything heavy -- see r1_head.py.
"""
import math
import threading
import time

import numpy as np

import r1_config as cfg
import r1_head as rh

MAX_SPEED_RAD_S = 0.30
MAX_DELTA_RAD = 1.5
LAG_RAD = 0.35
LAG_S = 1.5
SETTLE_RAD = 0.04
RELEASE_S = 2.0


class ArmFault(RuntimeError):
    pass


class ArmController:
    """Hold the upper body, move arm joints. Use engage(), move(), release()."""

    def __init__(self, link, publisher=None, new_cmd=None, crc=None, rate_hz=rh.RATE_HZ,
                 speed=MAX_SPEED_RAD_S, ramp_s=rh.RAMP_S, log=print):
        # the head controller already builds and publishes these messages correctly
        self._hc = rh.HeadController(link, publisher=publisher, new_cmd=new_cmd, crc=crc,
                                     rate_hz=rate_hz, ramp_s=ramp_s, log=log)
        self.link, self.rate, self.ramp_s, self.speed, self.log = link, rate_hz, ramp_s, speed, log
        self.hold = None                 # joint -> setpoint (rad), every held joint
        self.goal = {}                   # joint -> where it is walking to
        self.weight, self.weight_goal = 0.0, 0.0
        self.fault = None
        self._stop = threading.Event()
        self._thread = None
        self._lag_since = None
        self._lock = threading.Lock()

    # ---------------------------------------------------------------- loop
    def _publish(self):
        msg = self.link.latest()
        self._hc.pub.Write(self._hc.build(self.weight, self.hold, getattr(msg, "mode_machine", 0)))

    def _step(self, dt):
        age = self.link.age()
        quiet = age is None or age > rh.STALE_S
        dw = dt / max(self.ramp_s, 1e-3)
        self.weight += max(-dw, min(dw, self.weight_goal - self.weight))
        with self._lock:
            if quiet:
                if self.goal:
                    self.fault = self.fault or ArmFault(
                        "robot state stopped arriving (%s) -- holding the arm where it is"
                        % ("never" if age is None else "%.2f s" % age))
                    self.goal = {}
            else:
                step = self.speed * dt
                # all moving joints arrive together: scale each step by its share of the move
                far = max([abs(g - self.hold[n]) for n, g in self.goal.items()] + [0.0])
                for n, g in list(self.goal.items()):
                    q = self.hold[n]
                    share = abs(g - q) / far if far > 1e-9 else 1.0
                    s = step * share
                    self.hold[n] = q + max(-s, min(s, g - q))
                meas = self.link.joints()
                lag = max([abs(meas[n] - self.hold[n]) for n in self.goal] + [0.0])
                if self.weight > 0.99 and self.goal and lag > LAG_RAD:
                    self._lag_since = self._lag_since or time.time()
                    if time.time() - self._lag_since > LAG_S:
                        for n in self.goal:                 # stop pushing: hold where it is
                            self.hold[n] = meas[n]
                        self.goal = {}
                        self.fault = ArmFault("a joint fell %.0f deg behind its command -- "
                                              "something is in the way; holding here"
                                              % math.degrees(lag))
                else:
                    self._lag_since = None
        self._publish()

    def _run(self):
        period = 1.0 / self.rate
        nxt = time.time()
        while not self._stop.is_set():
            try:
                self._step(period)
            except Exception as e:                  # keep holding whatever happens
                self.fault = self.fault or e
                try:
                    self._publish()
                except Exception:
                    pass
            if self.weight_goal == 0.0 and self.weight <= 0.0:
                break
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))

    # ---------------------------------------------------------------- public
    def engage(self):
        self.link.wait()
        j = self.link.joints()
        self.hold = {n: j[n] for n in rh.HELD_JOINTS}
        self.start = dict(self.hold)
        self.weight, self.weight_goal = 0.0, 1.0
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        t0 = time.time()
        while self.weight < 0.99:
            if time.time() - t0 > self.ramp_s + 3:
                raise ArmFault("the hold did not come up")
            time.sleep(0.02)
        return self

    def check(self, target):
        """Why a move to `target` ({joint: rad}) would be refused, or None."""
        lim = cfg.joint_limits()
        for n, g in target.items():
            if n not in self.hold:
                return "%s is not a joint this controller holds" % n
            lo, hi = lim[n]
            if not (lo - 1e-6 <= g <= hi + 1e-6):
                return "%s = %.1f deg is outside its limits" % (n, math.degrees(g))
            if abs(g - self.hold[n]) > MAX_DELTA_RAD:
                return "%s would move %.0f deg in one go (limit %.0f)" % (
                    n, math.degrees(abs(g - self.hold[n])), math.degrees(MAX_DELTA_RAD))
        return None

    def move(self, target, timeout=None, interrupt=None):
        """Walk the given joints to `target` and wait until they get there (or a fault).
        `interrupt`: a threading.Event that stops the move where it is when set."""
        why = self.check(target)
        if why:
            raise ArmFault("refused: " + why)
        if self.fault:
            raise self.fault
        with self._lock:
            far = max([abs(g - self.hold[n]) for n, g in target.items()] + [0.0])
            self.goal = dict(target)
        timeout = timeout or far / self.speed + 6.0
        t0 = time.time()
        while True:
            if self.fault:
                raise self.fault
            if interrupt is not None and interrupt.is_set():
                self.stop_here()
                raise KeyboardInterrupt
            with self._lock:
                done = not self.goal or all(abs(self.hold[n] - g) < 1e-6 for n, g in self.goal.items())
            meas = self.link.joints()
            if done and all(abs(meas[n] - g) < SETTLE_RAD for n, g in target.items()):
                with self._lock:
                    self.goal = {}
                return {n: meas[n] for n in target}
            if time.time() - t0 > timeout:
                self.stop_here()
                raise ArmFault("did not arrive in %.0f s; holding where it is" % timeout)
            time.sleep(0.05)

    def stop_here(self):
        """Stop walking; hold the setpoints where they are now."""
        with self._lock:
            self.goal = {}

    def release(self):
        """Hand the upper body back to the robot's own controller, slowly. The arm will
        go to wherever the robot keeps it -- clear the space first."""
        if self._thread is None:
            return
        with self._lock:
            self.goal = {}
        self.ramp_s = RELEASE_S
        self.weight_goal = 0.0
        self._thread.join(timeout=RELEASE_S + 3.0)
        self._stop.set()
        self._thread = None
