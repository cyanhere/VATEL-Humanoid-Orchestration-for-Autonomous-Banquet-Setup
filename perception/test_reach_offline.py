"""Off-robot test for the reach: IK, the plan, the hand measurement, the arm controller and
the hand messages -- against a simulated R1.

    python3 test_reach_offline.py
"""
import json
import math
import os
import sys
import tempfile
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import r1_config as cfg  # noqa: E402
import urdf_fk as fk  # noqa: E402
import r1_ik as ik  # noqa: E402
import reach_plate as rp  # noqa: E402
import r1_arm  # noqa: E402
import r1_head as rh  # noqa: E402
import r1_hand  # noqa: E402

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-66s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-66s      %s" % ("", detail))


J = fk.load_urdf(cfg.URDF_PATH)
TOOL = np.array([0.15, 0.0, 0.0])          # a stand-in Revo 2 pinch point and palm
PALM = np.array([0.0, 0.0, -1.0])

# the stack the robot measured on 19 Oct (tape-verified): 36 cm forward, 3.4 cm right
STACK = {"center": np.array([0.362, -0.034, 0.073]), "radius": 0.127, "top_above_table": 0.044,
         "table_z": 0.073 - 0.044, "n_plates": 4, "captured_at": None}

print("\n  THE HAND, MEASURED")
pose = {n: 0.0 for n in cfg.MOTOR_SLOT}
Tw = fk.T_pelvis_wrist(J, pose, "right")
true_tool = np.array([0.13, -0.01, 0.02])
pinch = Tw[:3, :3] @ true_tool + Tw[:3, 3]
pelvis_h = 0.714
tool, palm, _ = rp.hand_frame_from_measurement(J, pose, "right", 100 * pinch[0], 100 * pinch[1],
                                               100 * (pinch[2] + pelvis_h), pelvis_h, "down")
check("tape measurements give the pinch point back in the wrist frame",
      np.linalg.norm(tool - true_tool) < 1e-9, str(np.round(tool, 4)))
check("'palm faces down' becomes a wrist-frame axis that points down in this pose",
      np.allclose(Tw[:3, :3] @ palm, [0, 0, -1]))

print("\n  THE PLAN, ON THE MEASURED STACK")
wps, fore, P, u = rp.waypoints(J, STACK, "right")
names = [n for n, _ in wps]
check("five waypoints: approach, beside, grasp, lift, set down",
      names == ["approach", "beside", "grasp", "lift", "set down"])
check("the rim point faces the right shoulder", u[1] < 0 and abs(np.linalg.norm(P[:2] - STACK["center"][:2]) - 0.127) < 1e-9)
d = dict(wps)
check("beside is 4 cm outside the rim and 2 cm above its top",
      np.allclose(d["beside"] - P, 0.04 * u + [0, 0, 0.02]))
check("grasp is 1.2 cm inside the rim, 5 mm below its top",
      np.allclose(d["grasp"] - P, -0.012 * u + [0, 0, -0.005]))
REST = rp.rest_pose()                     # where the robot really keeps its arms
start = {n: REST[n] for n in cfg.arm_joints("right")}
fixed = {n: v for n, v in REST.items() if n not in start}
t0 = time.time()
steps, problems = rp.plan(J, STACK, "right", start, fixed, table_edge_x=0.18, tool=TOOL, palm=PALM)
check("every waypoint reached to within 5 mm", all(s["pos_err_m"] < 0.005 for s in steps),
      " ".join("%s %.1fmm" % (s["name"], 1000 * s["pos_err_m"]) for s in steps))
check("the plan is safe to run (no problems)", not problems, "; ".join(problems))
check("elbow and wrist stay 3 cm+ above the table wherever over it",
      all(s["sweep_low_arm_m"] >= 0.03 for s in steps))
check("solved in reasonable time (< 20 s)", time.time() - t0 < 20, "%.1f s" % (time.time() - t0))
low = {n: 0.0 for n in start}            # forearm level with the table top
_, problems_low = rp.plan(J, STACK, "right", low, fixed, table_edge_x=0.18, tool=TOOL, palm=PALM)
check("a start with the hand already down at the table is flagged", bool(problems_low),
      problems_low[0] if problems_low else "")
check("the rest pose's mirror is the left arm's (roll, yaw, wrist flip sign)",
      REST["left_shoulder_roll_joint"] == -REST["right_shoulder_roll_joint"]
      and REST["left_elbow_joint"] == REST["right_elbow_joint"])
check("the hand stays 2 cm+ above the stack on the way in",
      all(s["over_stack_m"] >= 0.02 for s in steps[:2]),
      " ".join("%s %s" % (s["name"], "never over it" if s["over_stack_m"] > 8 else "%.1fcm" % (100 * s["over_stack_m"])) for s in steps[:2]))
tall = dict(STACK, center=STACK["center"] + [0, 0, 0.06])   # the same move, 6 cm more plates
over = rp.stack_clearance(J, "right", dict(fixed, **start), dict(fixed, **steps[0]["angles"]),
                          tall, tool=TOOL)
check("...and a stack 6 cm taller under the same move would be hit (it sees that)",
      over < 0.02, "%.1f cm" % (100 * over))
ik.FORCE_NUMPY = True                     # the robot computer may not have scipy
t0 = time.time()
steps_np, problems_np = rp.plan(J, STACK, "right", start, fixed, table_edge_x=0.18, tool=TOOL,
                                palm=PALM)
ik.FORCE_NUMPY = False
check("without scipy: the same plan, every waypoint within 5 mm, no problems",
      not problems_np and all(s["pos_err_m"] < 0.005 for s in steps_np),
      "%.1f s; " % (time.time() - t0) + " ".join("%s %.1fmm" % (s["name"], 1000 * s["pos_err_m"])
                                                for s in steps_np))
check("...and it lands within 1 cm of scipy's hand position at every waypoint",
      all(np.linalg.norm(a["tool_m"] - b["tool_m"]) < 0.01 for a, b in zip(steps, steps_np)))
far = dict(STACK, center=np.array([0.70, -0.034, 0.073]))
_, problems_far = rp.plan(J, far, "right", start, fixed, table_edge_x=0.18, tool=TOOL, palm=PALM)
check("a stack 70 cm out is refused as unreachable",
      any("cannot reach" in p for p in problems_far))

print("\n  THE ARM CONTROLLER, ON A SIMULATED ROBOT")


def new_cmd():
    return types.SimpleNamespace(
        mode_pr=0, mode_machine=0, crc=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
                   for _ in range(cfg.NUM_SLOTS)])


class FakeR1:
    def __init__(self, link, stuck=None, quiet_after=None):
        self.link, self.stuck, self.quiet_after = link, stuck, quiet_after
        self.q = {n: 0.0 for n in cfg.MOTOR_SLOT}
        self.q["right_elbow_joint"] = 0.3
        self.sent = []
        self.publish()

    def publish(self):
        ms = [types.SimpleNamespace(q=0.0) for _ in range(cfg.NUM_SLOTS)]
        for n, s in cfg.MOTOR_SLOT.items():
            ms[s].q = self.q[n]
        self.link.feed(types.SimpleNamespace(motor_state=ms, mode_machine=5))

    def Write(self, cmd):
        self.sent.append(cmd)
        w = cmd.mode_pr / 100.0
        for n in rh.HELD_JOINTS:
            if n == self.stuck:
                continue
            self.q[n] += 0.5 * w * (cmd.motor_cmd[cfg.MOTOR_SLOT[n]].q - self.q[n])
        if self.quiet_after is None or len(self.sent) < self.quiet_after:
            self.publish()


def controller(link, robot):
    return r1_arm.ArmController(link, publisher=robot, new_cmd=new_cmd, crc=False, rate_hz=200,
                                speed=1.5, ramp_s=0.2, log=lambda *a: None)


link = rh.R1Link(subscribe=False)
robot = FakeR1(link)
ctl = controller(link, robot).engage()
target = {"right_shoulder_pitch_joint": -0.6, "right_elbow_joint": -0.3}
got = ctl.move(target)
check("the arm reaches its target", all(abs(got[n] - v) < 0.05 for n, v in target.items()))
sp = np.array([c.motor_cmd[cfg.MOTOR_SLOT["right_shoulder_pitch_joint"]].q for c in robot.sent])
el = np.array([c.motor_cmd[cfg.MOTOR_SLOT["right_elbow_joint"]].q for c in robot.sent])
moving = np.nonzero(np.abs(np.diff(sp)) > 1e-9)[0]
check("no joint setpoint moves faster than the speed limit",
      max(np.max(np.abs(np.diff(sp))), np.max(np.abs(np.diff(el)))) <= 1.5 / 200 * 1.01)
check("both joints arrive together (they stop changing on the same tick)",
      abs(np.nonzero(np.abs(np.diff(sp)) > 1e-9)[0][-1] - np.nonzero(np.abs(np.diff(el)) > 1e-9)[0][-1]) <= 1)
others = [n for n in rh.HELD_JOINTS if n not in target]
check("every other upper-body joint is held where it was, in every message",
      all(abs(c.motor_cmd[cfg.MOTOR_SLOT[n]].q - (0.3 if n == "right_elbow_joint" else 0.0)) < 1e-12
          for c in robot.sent for n in others))
try:
    ctl.move({"right_shoulder_pitch_joint": 1.4})
    refused = False
except r1_arm.ArmFault as e:
    refused = "refused" in str(e)
check("a 116-deg swing in one move is refused", refused)
try:
    ctl.move({"right_elbow_joint": 3.0})
    refused = False
except r1_arm.ArmFault as e:
    refused = "outside its limits" in str(e)
check("a target past the joint's limit is refused", refused)
ctl.release()
check("release ramps the weight to 0", robot.sent[-1].mode_pr == 0
      and max(c.mode_pr for c in robot.sent[-20:]) < 100)

link = rh.R1Link(subscribe=False)
robot = FakeR1(link, stuck="right_elbow_joint")
ctl = controller(link, robot).engage()
try:
    ctl.move({"right_elbow_joint": -0.5}, timeout=10)
    fault = None
except r1_arm.ArmFault as e:
    fault = e
check("an arm that cannot move is noticed", fault is not None, str(fault)[:70])
held = robot.sent[-1].motor_cmd[cfg.MOTOR_SLOT["right_elbow_joint"]].q
check("...and it stops pushing: the command becomes where the joint is",
      abs(held - robot.q["right_elbow_joint"]) < 1e-6)
check("...and it does NOT let go (weight still 100)", robot.sent[-1].mode_pr == 100)
ctl.release()

link = rh.R1Link(subscribe=False)
robot = FakeR1(link, quiet_after=80)
ctl = controller(link, robot).engage()
try:
    ctl.move({"right_shoulder_pitch_joint": -1.0}, timeout=6)
    fault = None
except r1_arm.ArmFault as e:
    fault = e
check("a robot that goes quiet is noticed", fault is not None, str(fault)[:70])
time.sleep(0.2)
check("...the arm freezes and keeps being held (weight 100)", robot.sent[-1].mode_pr == 100)
ctl.release()

print("\n  THE HAND MESSAGES")
sent = []
fake_pub = types.SimpleNamespace(Write=lambda m: sent.append(m))
hand = r1_hand.Hand("right", publisher=fake_pub, subscribe=False,
                    new_cmds=lambda: types.SimpleNamespace(
                        cmds=[types.SimpleNamespace(q=0.0, dq=0.0) for _ in range(6)]))
m = hand.build([0.5, 1.2, -0.1, 0.7, 0.7, 0.7], speed=0.6)
check("six fingers, positions clamped to 0..1",
      [c.q for c in m.cmds] == [0.5, 1.0, 0.0, 0.7, 0.7, 0.7])
check("speed set on every finger", all(c.dq == 0.6 for c in m.cmds))
hand.open()
check("open sends all-zero, a few times", len(sent) == 5 and all(c.q == 0.0 for c in sent[-1].cmds))
check("the topic is Unitree's brainco service's", r1_hand.topic("right", "cmd") == "rt/brainco/right/cmd")

print("\n  THE OFFLINE RUN (plan printed, nothing moves)")
tmp = tempfile.mkdtemp()
hf = os.path.join(tmp, "hand_frame.json")
with open(hf, "w") as f:
    json.dump({"right": {"tool_m": TOOL.tolist(), "palm_axis": PALM.tolist()}}, f)
sj = os.path.join(tmp, "stack.json")
with open(sj, "w") as f:
    json.dump([{"center_m": STACK["center"].tolist(), "diameter_m": 0.254,
                "top_height_m": 0.044, "n_plates": 4}], f)
cfg.HAND_FRAME_PATH = hf
import contextlib
import io
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    code = rp.main(["offline", "run", sj, "--table-edge", "18"])
out = buf.getvalue()
check("`run` offline prints the plan and moves nothing", code == 0 and "beside" in out
      and "nothing moved" in out, out.strip().splitlines()[-1] if out else "")

print("\n%s" % ("ALL PASS" if ok else "SOME FAILED"))
sys.exit(0 if ok else 1)
