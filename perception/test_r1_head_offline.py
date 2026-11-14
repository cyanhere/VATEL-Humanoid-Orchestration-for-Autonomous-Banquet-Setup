"""Off-robot test for r1_head.py: a simulated R1 answers the arm_sdk commands.

    python3 test_r1_head_offline.py

No SDK, no robot. Checks what goes out on rt/arm_sdk (every held joint in its own slot,
held where it was, the weight in mode_pr and never in slot 29), that the head walks at its
speed limit inside its limits, that it is handed back where it was found, that it lets go
when the robot ignores it or goes quiet -- and that a tilted head changes the plate's
position in the robot frame by exactly what the joints say.
"""
import math
import os
import sys
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import r1_config as cfg
import r1_head as rh

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


def new_cmd():
    return types.SimpleNamespace(
        mode_pr=0, mode_machine=0, crc=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
                   for _ in range(cfg.NUM_SLOTS)])


class FakeR1:
    """Joints that follow arm_sdk commands in proportion to the weight; publishes state."""

    def __init__(self, link, start_deg, follows=True, quiet_after=None, pause=None, sag_deg=0.0):
        self.link, self.follows, self.quiet_after = link, follows, quiet_after
        self.sag = math.radians(sag_deg)     # a front-heavy head settles this far below target
        self.pause = pause                   # (from message n, for s seconds): reports stop
        self.q = {n: 0.0 for n in cfg.MOTOR_SLOT}
        self.q.update({n: math.radians(v) for n, v in start_deg.items()})
        self.sent, self.lock = [], threading.Lock()
        self.publish_state()

    def publish_state(self):
        ms = [types.SimpleNamespace(q=0.0) for _ in range(cfg.NUM_SLOTS)]
        for n, s in cfg.MOTOR_SLOT.items():
            ms[s].q = self.q[n]
        self.link.feed(types.SimpleNamespace(motor_state=ms, mode_machine=5))

    def Write(self, cmd):
        with self.lock:
            self.sent.append(cmd)
            w = cmd.mode_pr / 100.0
            if self.follows:
                for n in cfg.HEAD_JOINTS:
                    target = cmd.motor_cmd[cfg.MOTOR_SLOT[n]].q
                    if n == "head_pitch_joint":
                        target += self.sag * w
                    self.q[n] += 0.5 * w * (target - self.q[n])
            if self.pause and len(self.sent) == self.pause[0]:
                self.pause_until = time.time() + self.pause[1]
            if time.time() < getattr(self, "pause_until", 0.0):
                return
            if self.quiet_after is None or len(self.sent) < self.quiet_after:
                self.publish_state()


def controller(link, robot, **kw):
    kw.setdefault("rate_hz", 200.0)
    kw.setdefault("speed_deg_s", 60.0)
    kw.setdefault("ramp_s", 0.2)
    return rh.HeadController(link, publisher=robot, new_cmd=new_cmd, crc=False,
                             log=lambda *a: None, **kw)


START = {"head_pitch_joint": 3.0, "head_yaw_joint": -1.0, "waist_roll_joint": 0.4,
         "left_elbow_joint": 20.0, "right_shoulder_pitch_joint": -10.0}

print("\n  ONE MESSAGE")
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START)
hc = controller(link, robot)
targets = link.joints()
cmd = hc.build(0.37, targets, mode_machine=5)
check("weight goes in mode_pr, as a percent", cmd.mode_pr == 37)
check("mode_machine copied from the robot's state", cmd.mode_machine == 5)
slots = {cfg.MOTOR_SLOT[n] for n in rh.HELD_JOINTS}
check("every arm, waist and head joint is commanded (14 of them)",
      len(rh.HELD_JOINTS) == 14 and all(cmd.motor_cmd[s].mode == 1 for s in slots))
check("each held at its measured angle, in its own slot",
      all(abs(cmd.motor_cmd[cfg.MOTOR_SLOT[n]].q - targets[n]) < 1e-12 for n in rh.HELD_JOINTS))
check("slot 29 carries the head pitch, not a G1-style weight",
      abs(cmd.motor_cmd[29].q - math.radians(3.0)) < 1e-12)
legs = [s for n, s in cfg.MOTOR_SLOT.items() if "hip" in n or "knee" in n or "ankle" in n]
check("the legs are left alone (no gains, no mode)",
      all(cmd.motor_cmd[s].kp == 0 and cmd.motor_cmd[s].mode == 0 for s in legs))
check("the head gets Unitree's teleop gains (15, 1)",
      (cmd.motor_cmd[29].kp, cmd.motor_cmd[29].kd) == (15.0, 1.0))

print("\n  TILT AND HAND BACK")
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START)
hc = controller(link, robot)
with hc:
    got = hc.move_to(pitch_deg=25.0)
    sent_at_target = len(robot.sent)
check("the head reaches 25 deg down", abs(got["head_pitch_joint"] - 25.0) < 1.5,
      "%.2f deg" % got["head_pitch_joint"])
w = [c.mode_pr for c in robot.sent]
check("the weight ramps up from 0, never jumps", w[0] <= 10 and max(np.diff(w[:sent_at_target])) <= 10)
hp = [c.motor_cmd[29].q for c in robot.sent]
step = max(abs(np.diff(hp)))
check("the head setpoint never moves faster than the speed limit",
      step <= math.radians(60.0) / 200.0 * 1.01, "%.3f deg per tick" % math.degrees(step))
held_ok = all(abs(c.motor_cmd[cfg.MOTOR_SLOT[n]].q - math.radians(START.get(n, 0.0))) < 1e-12
              for c in robot.sent for n in rh.HELD_JOINTS if n not in cfg.HEAD_JOINTS)
check("arms and waist stay commanded exactly where they were, every message", held_ok)
check("handed back: the last message has weight 0", robot.sent[-1].mode_pr == 0)
check("and the head was returned to where it was found first",
      abs(math.degrees(robot.q["head_pitch_joint"]) - 3.0) < 1.5,
      "%.2f deg" % math.degrees(robot.q["head_pitch_joint"]))

print("\n  A FRONT-HEAVY HEAD (the helmet)")
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START, sag_deg=2.5)
hc = controller(link, robot)
with hc:
    got = hc.move_to(pitch_deg=30.0)
check("sagging 2.5 deg past 30, it is accepted once it stops",
      abs(got["head_pitch_joint"] - 32.5) < 0.5, "%.2f deg reported" % got["head_pitch_joint"])
check("and it reports where the head really is, not 30", got["head_pitch_joint"] > 31.5)
check("the hand-back still ends at weight 0", robot.sent[-1].mode_pr == 0)
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START, sag_deg=9.0)
hc = controller(link, robot)
hc.engage()
try:
    hc.move_to(pitch_deg=20.0, timeout=8.0)
    far = None
except rh.HeadNotFollowing as e:
    far = e
hc.release()
check("but a head that stops 9 deg off is refused", far is not None, str(far)[:60])

print("\n  LIMITS")
check("50 deg down is clamped to the 36 deg limit less 2",
      abs(math.degrees(rh.clamp_head("head_pitch_joint", math.radians(50))) - 34.0) < 1e-3)
check("and 50 deg up likewise", abs(math.degrees(rh.clamp_head("head_pitch_joint",
                                                               math.radians(-50))) + 34.0) < 1e-3)

print("\n  LETTING GO")
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START, follows=False)
hc = controller(link, robot)
hc.engage()
try:
    hc.move_to(pitch_deg=25.0, timeout=6.0)
    raised = None
except rh.HeadNotFollowing as e:
    raised = e
hc.release()
check("a head that does not follow is noticed", raised is not None, str(raised)[:60])
check("and let go (last weight 0)", robot.sent[-1].mode_pr == 0)

link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START, quiet_after=40)
hc = controller(link, robot)
hc.engage()
try:
    hc.move_to(pitch_deg=25.0, timeout=4.0)
    raised = None
except Exception as e:
    raised = e
hc.release()
check("a robot that goes quiet is noticed", raised is not None, str(raised)[:60])
check("and let go (last weight 0)", robot.sent[-1].mode_pr == 0)
link = rh.R1Link(subscribe=False)
robot = FakeR1(link, START, pause=(60, 0.8))
hc = controller(link, robot)
hc.engage()
try:
    got = hc.move_to(pitch_deg=25.0, timeout=8.0)
    err = None
except Exception as e:
    err = e
pitch_during = [c.motor_cmd[29].q for c in robot.sent]
hc.release()
check("a 0.8 s pause in the reports is ridden out: the head still gets there",
      err is None and abs(got["head_pitch_joint"] - 25.0) < 1.5, str(err)[:60] if err else "")
check("and the weight never dropped during the pause",
      min(c.mode_pr for c in robot.sent[60:200]) == 100 or min(c.mode_pr for c in robot.sent[40:200]) > 0)

link = rh.R1Link(subscribe=False)
try:
    link.angles(wait_s=0.2)
    stale = False
except RuntimeError:
    stale = True
check("no state, no angles: refuses rather than guessing zero", stale)

print("\n  THE PLATE FOLLOWS THE HEAD")
import camera_calibration as cc
import plate_stack as ps
import urdf_fk as fk

J = fk.load_urdf(cfg.URDF_PATH)
cal = cc.load()
truth = np.array([0.40, 0.05, 0.08])            # a stack top, pelvis_link
for deg in (0.0, 25.0):
    ang = {"head_pitch_joint": math.radians(deg)}
    T = cc.T_pelvis_optical(cal, J, ang)
    in_cam = np.linalg.inv(T) @ np.r_[truth, 1.0]
    stack = {"center_m": in_cam[:3], "normal": np.linalg.inv(T)[:3, :3] @ [0, 0, 1.0],
             "top_height_m": 0.04, "diameter_m": 0.254, "n_plates": 4}
    r = ps.to_robot_frame(stack, joints=J, angles=ang, calibration=cal)
    check("head %2.0f deg down: the stack comes back where it is" % deg,
          np.linalg.norm(r["center_m"] - truth) < 1e-9)
    wrong = ps.to_robot_frame(stack, joints=J, angles={}, calibration=cal)
    if deg:
        check("ignoring a 25 deg head tilt would be badly wrong",
              np.linalg.norm(wrong["center_m"] - truth) > 0.10,
              "%.0f mm out" % (1000 * np.linalg.norm(wrong["center_m"] - truth)))

print("\n%s" % ("ALL PASS" if ok else "SOME FAILED"))
sys.exit(0 if ok else 1)
