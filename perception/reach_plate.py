"""Reach for the plate the camera found, beside its rim, and pick it up -- one step at a time.

The point is to find out where a miss comes from. The camera's plate position was checked
against a tape measure (correct to ~1 cm); this checks the rest of the chain -- arm
kinematics, the hand's size, the controller -- by putting the hand where the numbers say
the plate is, stopping beside it, and letting you measure the gap.

STAGES, IN ORDER (as on the G1; do not skip). Nothing moves without --go.

    python3 r1_hand.py eth10 right open --go         # H: the hand alone: open, close, look
    python3 reach_plate.py eth10 hold --go           # A: hold the upper body, change nothing
    python3 reach_plate.py eth10 nudge right_elbow_joint -10 --go    # B: one joint, 10 deg
    python3 reach_plate.py eth10 measure-hand right --go             # once: the hand's size
    python3 check_stack.py --live --robot eth10 --head-down 30 --json stack.json   # see it
    python3 reach_plate.py eth10 run stack.json --table-edge 18      # C: the plan, no motion
    python3 reach_plate.py eth10 run stack.json --table-edge 18 --go # C: do it

`run` steps through, waiting for Enter before each:

    approach   6 cm outside the rim, 7 cm above it (on the side facing the arm's shoulder)
    beside     4 cm outside the rim, 2 cm above its top          <- STOP AND MEASURE HERE
    grasp      the pinch point 1.2 cm inside the rim, at its top; then the hand closes
    lift       4 cm up -- hold, look
    set down   back down; the hand opens
    clear      back out to beside, approach, and the pose it started from; then let go

At `beside` it asks where the hand's pinch point really is -- forward from the ankle
midpoint, left of centre, height off the floor, the same tape measurements as the plate
check -- and prints the arm's error in the robot's frame next to it, twice: against where
the arm was TOLD to go, and against where its joints SAY it went (the right arm was seen
hanging 8-12 mm above its command). The first minus the second is the arm not following;
the second alone is the arm/hand model. Everything goes into reach_log.json.

Tape numbers become robot coordinates through floor_frame.py, which reads the robot's lean
(~3 deg forward standing) instead of assuming the pelvis is level.

If anything looks wrong: Ctrl+C. The arm stops where it is and keeps holding, and you choose
r (retrace back to the start and let go), h (keep holding), or x (let go here -- the arm
swings back to the robot's rest pose, so clear the space first).

NEEDS
    --table-edge CM   how far the table's near edge is in front of the floor point midway
                      between the ankles. The arm is kept 5 cm above the table only over it.
    hand_frame.json   from measure-hand (where the pinch point is; which way the palm faces)
    the hand service  sudo ~/brainco_hand_service/bin/brainco_hand_server --network eth10
    one stack.json    from check_stack.py, with the robot's feet not moved since.
"""
import argparse
import datetime
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import r1_config as cfg  # noqa: E402
import urdf_fk as fk  # noqa: E402

APPROACH_OUT_M, APPROACH_UP_M = 0.06, 0.07
BESIDE_OUT_M, BESIDE_UP_M = 0.04, 0.02
GRASP_IN_M, GRASP_DOWN_M = 0.012, 0.005
LIFT_M = 0.04
STACK_MARGIN_M = 0.03          # the hand keeps this far outside the stack, or 2 cm above it,
STACK_ABOVE_M = 0.02           # on every move but the grasp itself
FOREARM_TILT_DEG = 25.0
LOG = os.path.join(HERE, "reach_log.json")
D = math.degrees
# where the robot holds its right arm when standing (read off it on 19 Oct); the left
# mirrors it. Only used to plan offline -- on the robot the plan starts from the live pose.
REST_RIGHT = {"right_shoulder_pitch_joint": -0.4216, "right_shoulder_roll_joint": -0.0868,
              "right_shoulder_yaw_joint": 0.0064, "right_elbow_joint": -0.2581,
              "right_wrist_roll_joint": -0.1814}
MIRROR = {"shoulder_pitch": 1, "shoulder_roll": -1, "shoulder_yaw": -1, "elbow": 1, "wrist_roll": -1}


def rest_pose():
    """Every joint at 0 but the arms, which are where the robot keeps them."""
    j = {n: 0.0 for n in cfg.MOTOR_SLOT}
    for n, v in REST_RIGHT.items():
        j[n] = v
        k = n[len("right_"):-len("_joint")]
        j["left_" + n[len("right_"):]] = MIRROR[k] * v
    return j


# ==========================================================================
# the plan (no robot needed)
# ==========================================================================
def load_stack(path, which=0):
    with open(path) as f:
        doc = json.load(f)
    stacks = doc if isinstance(doc, list) else doc.get("stacks", [doc])
    if not stacks:
        raise SystemExit("%s has no stack in it" % path)
    s = stacks[which]
    c = np.asarray(s["center_m"], float)
    out = {"center": c, "radius": s["diameter_m"] / 2, "top_above_table": s["top_height_m"],
           "table_z": float(c[2] - s["top_height_m"]), "n_plates": s.get("n_plates"),
           "captured_at": s.get("captured_at"), "raw": s}
    return out


def waypoints(joints, stack, arm):
    """Pinch-point targets in pelvis frame, and the direction the forearm should come in."""
    sh = fk.shoulder_positions(joints, {})[arm]
    C, r = stack["center"], stack["radius"]
    u = sh - C
    u[2] = 0.0
    u /= np.linalg.norm(u)                     # from the plate's centre toward the shoulder
    P = C + r * u                              # the rim point facing the arm
    z = np.array([0.0, 0.0, 1.0])
    t = math.radians(FOREARM_TILT_DEG)
    fore = -u * math.cos(t) - z * math.sin(t)
    grasp = P - GRASP_IN_M * u - GRASP_DOWN_M * z
    wps = [("approach", P + APPROACH_OUT_M * u + APPROACH_UP_M * z),
           ("beside", P + BESIDE_OUT_M * u + BESIDE_UP_M * z),
           ("grasp", grasp),
           ("lift", grasp + LIFT_M * z),
           ("set down", grasp + 0.003 * z)]
    return wps, fore, P, u


def stack_clearance(joints, arm, a0, a1, stack, tool=None, n=12):
    """Lowest height above the stack's top met by the pinch point or the wrist, moving in
    a straight line in joint space from a0 to a1, while it is over the stack (within
    STACK_MARGIN_M of its rim). 9.0 if nothing passes over it."""
    import r1_ik as ik
    C, r = stack["center"], stack["radius"] + STACK_MARGIN_M
    names = cfg.arm_joints(arm)
    low = 9.0
    for s in np.linspace(0.0, 1.0, n):
        a = dict(a0)
        a.update({k: (1 - s) * a0[k] + s * a1[k] for k in names})
        tip = ik.tool_point(joints, a, arm, tool)[0]
        wrist = fk.T_pelvis_wrist(joints, a, arm)[:3, 3]
        for p in (tip, wrist):
            if np.hypot(p[0] - C[0], p[1] - C[1]) < r:
                low = min(low, float(p[2] - C[2]))
    return low


def plan(joints, stack, arm, start, fixed, table_edge_x, tool=None, palm=None, log=print):
    """IK for every waypoint from the live pose. Returns (steps, problems)."""
    import r1_ik as ik
    wps, fore, P, u = waypoints(joints, stack, arm)
    steps, problems, prev = [], [], dict(start)
    for name, tgt in wps:
        s = ik.solve(joints, arm, tgt, prev, fixed=fixed, forearm_dir=fore,
                     table_z=stack["table_z"], table_edge_x=table_edge_x, tool=tool, palm=palm)
        low_kp, low_tool = ik.sweep_clearance(joints, arm, dict(fixed, **prev),
                                              dict(fixed, **s["arm"]), stack["table_z"],
                                              tool=tool, table_edge_x=table_edge_x)
        over = stack_clearance(joints, arm, dict(fixed, **prev), dict(fixed, **s["arm"]),
                               stack, tool=tool)
        big = max(abs(s["arm"][n] - prev[n]) for n in s["arm"])
        st = {"name": name, "target_m": tgt, "angles": s["arm"], "tool_m": s["tool_m"],
              "pos_err_m": s["pos_err_m"], "forearm_err_deg": s["forearm_err_deg"],
              "palm_err_deg": s["palm_err_deg"], "sweep_low_arm_m": low_kp,
              "sweep_low_hand_m": low_tool, "over_stack_m": over, "largest_move_deg": D(big)}
        if not s["reached"]:
            problems.append("%s: the arm cannot reach it (misses by %.0f mm)" % (name, 1000 * s["pos_err_m"]))
        if low_kp < 0.03:
            problems.append("%s: the elbow or wrist passes %.0f cm above the table" % (name, 100 * low_kp))
        if low_tool < (-0.002 if name in ("grasp", "set down") else 0.01):
            problems.append("%s: the hand passes %.1f cm from the table top" % (name, 100 * low_tool))
        if name in ("approach", "beside") and over < STACK_ABOVE_M:
            problems.append("%s: the hand passes %.1f cm %s the top of the stack on the way"
                            % (name, 100 * abs(over), "above" if over >= 0 else "below"))
        if D(big) > 85:
            problems.append("%s: a joint would swing %.0f deg" % (name, D(big)))
        steps.append(st)
        prev = dict(s["arm"])
    return steps, problems


def print_plan(stack, arm, steps, problems, table_edge_x):
    c = stack["center"]
    print("plate stack  centre %.0f cm forward, %.0f cm %s, rim %.1f cm above the hips; "
          "%s plates, %.1f mm tall; table top %.1f cm %s the hips"
          % (100 * c[0], 100 * abs(c[1]), "left" if c[1] >= 0 else "right", 100 * c[2],
             stack["n_plates"] or "?", 1000 * stack["top_above_table"],
             100 * abs(stack["table_z"]), "above" if stack["table_z"] >= 0 else "below"))
    print("arm          %s;  table edge %s" % (arm, "%.0f cm in front" % (100 * table_edge_x)
                                                if table_edge_x is not None else "not given"))
    print("\n%-9s %-26s %8s %9s %7s %13s %8s" % ("step", "pinch point (cm fwd/left/up)", "miss",
                                               "forearm", "palm", "lowest arm/hand", "move"))
    for s in steps:
        t = s["target_m"]
        print("%-9s %7.1f %7.1f %7.1f      %5.1f mm %6.0f deg %4s deg %7s / %-5s %5.0f deg"
              % (s["name"], 100 * t[0], 100 * t[1], 100 * t[2], 1000 * s["pos_err_m"],
                 s["forearm_err_deg"], "-" if s["palm_err_deg"] is None else "%.0f" % s["palm_err_deg"],
                 "-" if s["sweep_low_arm_m"] > 8 else "%.0fcm" % (100 * s["sweep_low_arm_m"]),
                 "-" if s["sweep_low_hand_m"] > 8 else "%.1fcm" % (100 * s["sweep_low_hand_m"]),
                 s["largest_move_deg"]))
    print("(forearm/palm: how far from the ideal hand angle the arm's five joints allow -- "
          "a wish, not a requirement)")
    if problems:
        print("\nNOT SAFE TO RUN:")
        for p in problems:
            print("  - " + p)


# ==========================================================================
# the robot
# ==========================================================================
def _connect(iface):
    import r1_head as rh
    link = rh.R1Link(iface).wait()
    return link


def _controller(link):
    import r1_arm
    return r1_arm.ArmController(link)


def _ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "a"


def _safe_menu(arm_ctl, retrace):
    """After a Ctrl+C or a fault: stop the arm where it is (it keeps holding), then ask."""
    arm_ctl.stop_here()
    while True:
        c = _ask("\nThe arm is holding still.  r = retrace to the start and let go,  "
                 "h = keep holding,  x = let go here (it swings to rest -- clear the space): ")
        if c == "r":
            for name, target in retrace:
                print("  back to %s" % name)
                arm_ctl.move(target)
            arm_ctl.release()
            return
        if c == "x":
            arm_ctl.release()
            return
        if c == "h":
            time.sleep(1.0)


def cmd_hold(args):
    link = _connect(args.interface)
    if not args.go:
        print("would hold the whole upper body where it is now, until Enter. Add --go.")
        return 0
    ctl = _controller(link).engage()
    _ask("holding (nothing should have moved). Enter to let go ")
    ctl.release()
    print("let go")
    return 0


def cmd_nudge(args):
    link = _connect(args.interface)
    j = link.joints()
    if args.joint not in j:
        raise SystemExit("no joint %r" % args.joint)
    tgt = j[args.joint] + math.radians(args.degrees)
    print("%s: %.1f -> %.1f deg, then back" % (args.joint, D(j[args.joint]), D(tgt)))
    if not args.go:
        print("nothing moved -- add --go")
        return 0
    ctl = _controller(link).engage()
    try:
        ctl.move({args.joint: tgt})
        _ask("there. Enter to go back ")
        ctl.move({args.joint: j[args.joint]})
    except (KeyboardInterrupt, Exception) as e:
        print("stopped: %s" % (e or "Ctrl+C"))
        _safe_menu(ctl, [("start", {args.joint: j[args.joint]})])
        return 1
    ctl.release()
    return 0


MEASURE_POSE = {"shoulder_pitch_joint": 0.0, "shoulder_roll_joint": 0.0,
                "shoulder_yaw_joint": 0.0, "elbow_joint": 0.0, "wrist_roll_joint": 0.0}
PALM_WORDS = {"down": [0, 0, -1], "up": [0, 0, 1], "left": [0, 1, 0], "right": [0, -1, 0],
              "forward": [1, 0, 0], "back": [-1, 0, 0]}


def hand_frame_from_measurement(joints, angles, arm, fwd_cm, left_cm, height_cm,
                                floor, palm_word):
    """The pinch point and palm direction in the wrist frame, from tape measurements
    taken with the arm at `angles`. floor: a floor_frame.FloorFrame (or, the old way, the
    pelvis' height above the floor, the pelvis taken as level)."""
    import floor_frame as ff
    if not isinstance(floor, ff.FloorFrame):
        floor = ff.FloorFrame([0.0, 0.0, 1.0], floor)
    Tw = fk.T_pelvis_wrist(joints, angles, arm)
    p = floor.to_pelvis((fwd_cm / 100.0, left_cm / 100.0, height_cm / 100.0))
    tool = Tw[:3, :3].T @ (p - Tw[:3, 3])
    palm = Tw[:3, :3].T @ floor.direction_to_pelvis(PALM_WORDS[palm_word])
    return tool, palm, p


def cmd_measure_hand(args):
    import camera_calibration as cc
    cal = cc.load()
    if not cal or cal.get("pelvis_above_floor_m") is None:
        raise SystemExit("needs camera_calibration.json with the pelvis height: run "
                         "check_mount.py --calibrate --robot eth10 first")
    link = _connect(args.interface)
    j = link.joints()
    floor = _floor(link)
    names = cfg.arm_joints(args.arm)
    target = {"%s_%s" % (args.arm, k): v for k, v in MEASURE_POSE.items()}
    start = {n: j[n] for n in names}
    w = floor.to_tape(fk.T_pelvis_wrist(fk.load_urdf(cfg.URDF_PATH), dict(j, **target),
                                        args.arm)[:3, 3])
    print("measure pose: all five %s arm joints at 0 -- upper arm down, forearm straight forward;"
          " the wrist joint will be %.0f cm forward, %.0f cm %s, %.0f cm off the floor"
          % (args.arm, 100 * w[0], 100 * abs(w[1]), "left" if w[1] >= 0 else "right",
             100 * w[2]))
    print("That is about table height: NOTHING may be within 50 cm in front of the robot "
          "(do this before it stands at the table).")
    if not args.go:
        print("nothing moved -- add --go")
        return 0
    hand = _hand(args.arm)
    ctl = _controller(link).engage()
    try:
        ctl.move(target)
        if hand:
            hand.close()                         # measure the pinch with the pinch made
            time.sleep(1.5)
        print("\nNow measure the hand's PINCH POINT -- where the thumb tip meets the index"
              "\nfingertip (closed now, if the hand service is running) -- with the tape:")
        fwd = float(_ask("  forward from the floor point midway between the ankles (cm): "))
        left = float(_ask("  sideways from the robot's centre line, + = its LEFT, - = its right (cm): "))
        up = float(_ask("  height above the floor (cm): "))
        palm = ""
        while palm not in PALM_WORDS:
            palm = _ask("  which way does the PALM face? down/up/left/right/forward/back: ")
        if hand:
            hand.open()
            time.sleep(1.0)
        ctl.move(start)
    except (KeyboardInterrupt, Exception) as e:
        print("stopped: %s" % (e or "Ctrl+C"))
        _safe_menu(ctl, [("start", start)])
        return 1
    ctl.release()
    J = fk.load_urdf(cfg.URDF_PATH)
    tool, palm_axis, p = hand_frame_from_measurement(J, dict(j, **target), args.arm, fwd, left,
                                                     up, floor, palm)
    L = float(np.linalg.norm(tool))
    print("\npinch point: %.1f cm from the wrist joint, at %s in the wrist frame; palm %s"
          % (100 * L, np.round(tool, 3).tolist(), np.round(palm_axis, 2).tolist()))
    if not 0.05 <= L <= 0.30:
        print("THAT LENGTH IS UNLIKELY for a Revo 2 (expect ~10-20 cm). Not saved -- measure again.")
        return 1
    doc = {}
    if os.path.exists(cfg.HAND_FRAME_PATH):
        with open(cfg.HAND_FRAME_PATH) as f:
            doc = json.load(f)
    doc[args.arm] = {"tool_m": [round(float(v), 4) for v in tool],
                     "palm_axis": [round(float(v), 4) for v in palm_axis],
                     "measured": {"forward_cm": fwd, "left_cm": left, "height_cm": up, "palm": palm,
                                  "pelvis_up": np.round(floor.up, 5).tolist(),
                                  "date": datetime.date.today().isoformat()}}
    with open(cfg.HAND_FRAME_PATH, "w") as f:
        json.dump(doc, f, indent=2)
    print("saved to %s" % cfg.HAND_FRAME_PATH)
    return 0


def cmd_run(args):
    J = fk.load_urdf(cfg.URDF_PATH)
    stack = load_stack(args.stack, args.which)
    arm = args.arm if args.arm != "auto" else ("left" if stack["center"][1] >= 0 else "right")
    edge = None if args.table_edge is None else args.table_edge / 100.0
    if edge is None:
        raise SystemExit("--table-edge CM is needed: the distance from the floor point midway "
                         "between the ankles forward to the table's near edge")
    try:
        cfg.tool_offset(arm)      # refuses, with instructions, if the hand is not measured
    except NotImplementedError as e:
        raise SystemExit(str(e))
    link = _connect(args.interface) if args.interface != "offline" else None
    j = link.joints() if link else rest_pose()
    floor = _floor(link)
    names = cfg.arm_joints(arm)
    start = {n: j[n] for n in names}
    fixed = {n: v for n, v in j.items() if n not in names}
    steps, problems = plan(J, stack, arm, start, fixed, edge)
    print_plan(stack, arm, steps, problems, edge)
    if problems or not args.go or link is None:
        if not problems:
            print("\nnothing moved -- add --go to do it (with the hand service running)")
        return 1 if problems else 0

    hand = None if args.no_hand else _hand(arm)
    log = {"date": datetime.datetime.now().isoformat(timespec="seconds"), "arm": arm,
           "stack": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                     for k, v in stack.items() if k != "raw"},
           "steps": [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in s.items()}
                     for s in steps], "measured": {}}
    ctl = _controller(link).engage()
    done = [("start", start)]
    try:
        if hand:
            hand.open()
        for s in steps:
            if _ask("\nEnter = go to %-9s (a = abort) " % s["name"]) == "a":
                raise KeyboardInterrupt
            got = ctl.move(s["angles"])
            done.append((s["name"], s["angles"]))
            t = s["tool_m"]
            tt = floor.to_tape(t)
            print("  at %s: the pinch point should be %.1f cm forward, %.1f cm %s, %.1f cm off "
                  "the floor" % (s["name"], 100 * tt[0], 100 * abs(tt[1]),
                                 "left" if tt[1] >= 0 else "right", 100 * tt[2]))
            if s["name"] == "beside":
                import r1_ik as ik
                now = dict(link.joints(), **got)          # where the joints SAY the arm is
                said = ik.tool_point(J, now, arm)[0]
                if hand:                         # make the pinch, in the air, to measure it
                    hand.close()
                    time.sleep(1.5)
                log["measured"]["beside"] = _measure_gap(t, floor, said)
                if hand:
                    hand.open()
                    time.sleep(1.0)
            if s["name"] == "grasp" and hand:
                _ask("  Enter to CLOSE the hand ")
                hand.close()
                time.sleep(1.5)
            if s["name"] == "set down" and hand:
                hand.open()
                time.sleep(1.0)
        for name, target in reversed(done[:-1]):
            if name in ("grasp", "lift"):
                continue
            _ask("\nEnter = back to %s " % name)
            ctl.move(target)
        ctl.release()
        print("done -- let go")
    except (KeyboardInterrupt, Exception) as e:
        print("\nstopped: %s" % (e or "Ctrl+C"))
        retrace = [(n, a) for n, a in reversed(done[:-1]) if n not in ("grasp", "lift")]
        _safe_menu(ctl, retrace)
    finally:
        with open(LOG, "a") as f:
            f.write(json.dumps(log) + "\n")
        print("logged to %s" % LOG)
    return 0


def _hand(arm):
    """The arm's Revo 2, or None (with a warning) if the hand service is not answering."""
    try:
        import r1_hand
        h = r1_hand.Hand(arm)
    except Exception as e:                       # pragma: no cover - robot only
        print("no hand: %s -- carrying on with the arm alone" % e)
        return None
    t0 = time.time()
    while h.state() is None and time.time() - t0 < 2.0:
        time.sleep(0.05)
    if h.state() is None:
        print("the %s hand's service is not answering (rt/brainco/%s/state) -- the arm will "
              "move, the hand will not. Start it with:\n"
              "    sudo ~/brainco_hand_service/bin/brainco_hand_server --network eth10"
              % (arm, arm))
        return None
    return h


def _floor(link=None):
    """The tape frame (floor_frame.FloorFrame): how tape numbers become robot coordinates,
    with the robot's lean read off it. Offline: level, at the calibration's pelvis height."""
    import camera_calibration as cc
    import floor_frame as ff
    cal = cc.load() or {}
    if link is None:
        h = cal.get("pelvis_above_floor_m")
        return ff.FloorFrame([0.0, 0.0, 1.0], 0.71 if h is None else h)
    frame, info = ff.FloorFrame.from_state(link.state(), cal)
    print(ff.describe(info))
    if frame is None:
        raise SystemExit("needs camera_calibration.json with the floor's height: run "
                         "check_mount.py --calibrate --robot eth10 (or calibrate_sweep.py)")
    return frame


def _say(err):
    return "%.1f cm %s, %.1f cm %s, %.1f cm %s" % (
        100 * abs(err[0]), "forward" if err[0] >= 0 else "back",
        100 * abs(err[1]), "left" if err[1] >= 0 else "right",
        100 * abs(err[2]), "high" if err[2] >= 0 else "low")


def _measure_gap(planned_tool, floor, said_tool=None):
    """Tape-measure the hand at `beside`; print the arm's error in the robot's frame --
    against the command, and against what the joints report."""
    print("\n  MEASURE NOW. Where is the hand's pinch point really? (Enter to skip)")
    try:
        f = _ask("    forward from the ankle midpoint (cm): ")
        if not f:
            return None
        l = float(_ask("    sideways, + = robot's LEFT (cm): "))
        h = float(_ask("    height above the floor (cm): "))
        real = floor.to_pelvis((float(f) / 100, l / 100, h / 100))
    except ValueError:
        print("    not a number -- skipped")
        return None
    err = real - planned_tool
    print("    the hand is %s of where the arm was TOLD to put it" % _say(err))
    out = {"real_m": real.tolist(), "planned_m": np.asarray(planned_tool).tolist(),
           "error_m": err.tolist(), "pelvis_up": floor.up.tolist()}
    if said_tool is not None:
        e2 = real - said_tool
        lag = np.asarray(said_tool) - planned_tool
        print("    the hand is %s of where its JOINTS say it is" % _say(e2))
        print("    (the joints themselves are %s of the command: the arm not following)"
              % _say(lag))
        out.update({"joints_say_m": np.asarray(said_tool).tolist(),
                    "error_vs_joints_m": e2.tolist()})
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("interface", help="eth10 on the robot computer; 'offline' to plan only")
    sub = ap.add_subparsers(dest="cmd", required=True)
    h = sub.add_parser("hold")
    h.add_argument("--go", action="store_true")
    n = sub.add_parser("nudge")
    n.add_argument("joint")
    n.add_argument("degrees", type=float)
    n.add_argument("--go", action="store_true")
    m = sub.add_parser("measure-hand")
    m.add_argument("arm", choices=("left", "right"))
    m.add_argument("--go", action="store_true")
    r = sub.add_parser("run")
    r.add_argument("stack", help="stack.json from check_stack.py --json")
    r.add_argument("--which", type=int, default=0, help="which stack in the file (default 0)")
    r.add_argument("--arm", choices=("auto", "left", "right"), default="auto")
    r.add_argument("--table-edge", type=float, metavar="CM",
                   help="table's near edge, cm in front of the ankle midpoint")
    r.add_argument("--no-hand", action="store_true", help="move the arm only")
    r.add_argument("--go", action="store_true")
    args = ap.parse_args(argv)
    return {"hold": cmd_hold, "nudge": cmd_nudge, "measure-hand": cmd_measure_hand,
            "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
