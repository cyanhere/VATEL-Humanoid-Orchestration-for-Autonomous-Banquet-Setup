"""Off-robot test for the camera fixes of 23 Oct: the robot's lean, the head-sweep
calibration and the yaw play -- what the manipulation side measured on 21-22 Oct -- against
a simulated R1 whose camera mount, yaw axis and stance are off by known amounts.

    python3 test_sweep_offline.py
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

import calibrate_sweep as cs  # noqa: E402
import camera_calibration as cc  # noqa: E402
import floor_frame as ff  # noqa: E402
import r1_config as cfg  # noqa: E402
import r1_head as rh  # noqa: E402
import reach_plate as rp  # noqa: E402
import urdf_fk as fk  # noqa: E402

ok = True
R, D = math.radians, math.degrees


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-66s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-66s      %s" % ("", detail))


J = fk.load_urdf(cfg.URDF_PATH)
rng = np.random.default_rng(1)

# ---------------------------------------------------------------- the simulated robot
LEAN = (R(0.2), R(3.0), 0.0)                 # IMU roll, pitch, yaw: as read on 22 Oct
UP = ff.up_from_imu(LEAN)
M_CAD = fk.T(J, "head_yaw_link", cfg.CAMERA_OPTICAL_LINK, {})
TRUE_MOUNT = fk.homogeneous(cc.rot([R(0.8), R(-1.5), R(1.0)]) @ M_CAD[:3, :3],
                            M_CAD[:3, 3] + [0.0, 0.0, 0.004])
TRUE_NECK = (R(1.6), R(0.7))                 # the yaw axis tipped / a head-pitch zero
FLOOR_H = 0.711                              # pelvis origin above the floor, along up
TABLE_H = -0.029                             # the table top is 2.9 cm ABOVE the pelvis
TABLE_ABOVE_FLOOR = FLOOR_H - TABLE_H
INTR = {"fx": 213.0, "fy": 213.0, "cx": 212.0, "cy": 120.0, "width": 424, "height": 240}


def pose(pitch_deg, yaw_deg, lean=LEAN):
    """All joints: head as asked; legs bent so both soles are flat on a level floor while
    the pelvis leans by `lean` (knee + ankle pitch take the pitch, ankle roll the roll)."""
    a = {n: 0.0 for n in cfg.MOTOR_SLOT}
    a.update(head_pitch_joint=R(pitch_deg), head_yaw_joint=R(yaw_deg))
    for s in ("left", "right"):
        a["%s_knee_joint" % s] = 0.3
        a["%s_ankle_pitch_joint" % s] = -0.3 - lean[1]
        a["%s_ankle_roll_joint" % s] = -lean[0]
    return a


def state(a, rpy=LEAN):
    return {"joints": dict(a), "imu_rpy": list(rpy)}


def true_T(a, mount=TRUE_MOUNT, neck=TRUE_NECK):
    return cc.T_pelvis_headyaw(J, a, neck) @ mount


def ray_hit(T_po, H, px=None):
    """Where the camera ray through pixel px (default: centre) meets the level plane H below
    the pelvis: (point in optical coords, point in pelvis_link)."""
    u, v = px or (INTR["cx"], INTR["cy"])
    d = np.array([(u - INTR["cx"]) / INTR["fx"], (v - INTR["cy"]) / INTR["fy"], 1.0])
    n, c = cc.predict_plane(T_po, UP, H)
    p_opt = d * (c / (n @ d))
    return p_opt, T_po[:3, :3] @ p_opt + T_po[:3, 3]


def plate_error_mm(T_model, T_true, H=TABLE_H):
    """How far the model puts table points the camera sees (centre and two corners)."""
    errs = []
    for px in [None, (60, 40), (360, 200)]:
        p_opt, p_true = ray_hit(T_true, H, px)
        p_model = T_model[:3, :3] @ p_opt + T_model[:3, 3]
        errs.append(1000 * np.linalg.norm(p_model - p_true))
    return max(errs)


def obs_from_truth(pitches, yaws, H=TABLE_H, noise=True, surface="table", session=0):
    out = []
    for p in pitches:
        for y in yaws:
            a = pose(p, y)
            n, c = cc.predict_plane(true_T(a), UP, H)
            if noise:
                n = n + rng.normal(0.0, R(0.05), 3)
                n /= np.linalg.norm(n)
                c += rng.normal(0.0, 0.0007)
            out.append({"session": session, "surface": surface, "angles": a, "up": UP,
                        "lean": ff.pelvis_up(state(a), J)[1], "n": n, "c": c, "spread": 0.0,
                        "name": "p%+03.0f_y%+03.0f" % (p, y), "imu_rpy": list(LEAN),
                        "pitch": p, "yaw": y})
    return out


print("\n  OLD CALIBRATIONS STILL MEAN WHAT THEY MEANT")
worst = 0.0
for _ in range(5):
    a = {n: rng.uniform(-0.4, 0.4) for n in cfg.CAMERA_CHAIN_JOINTS}
    worst = max(worst, np.abs(cc.T_pelvis_headyaw(J, a) - fk.T(J, cfg.ROOT_LINK, "head_yaw_link", a)).max())
check("no neck tilt: exactly the URDF's head chain", worst < 1e-12, "%.1e" % worst)
old = {"T_headyaw_optical": TRUE_MOUNT.tolist()}
a = pose(20, 10)
check("a calibration file without the new fields gives the same camera pose",
      np.allclose(cc.T_pelvis_optical(old, J, a),
                  fk.T(J, cfg.ROOT_LINK, "head_yaw_link", a) @ TRUE_MOUNT, atol=1e-12))

print("\n  THE LEAN, TWO WAYS")
fwd, left = ff.lean_deg(ff.up_from_imu((0.0, R(3.0), 0.0)))
check("IMU pitch +3 deg reads as leaning 3 deg forward", abs(fwd - 3.0) < 1e-9 and abs(left) < 1e-9)
legs_up, feet = ff.up_from_legs(J, pose(0, 0))
check("legs (feet flat) give the same up as the IMU", ff.angle_deg(legs_up, UP) < 0.02,
      "%.3f deg apart, feet agree to %.3f deg" % (ff.angle_deg(legs_up, UP), feet))
up, info = ff.pelvis_up(state(pose(0, 0)), J)
check("...so no warning, IMU used", info["source"] == "imu" and not info["warning"])
bad = pose(0, 0)
bad["left_ankle_pitch_joint"] += R(2.0)
bad["right_ankle_pitch_joint"] += R(2.0)
_, info = ff.pelvis_up(state(bad), J)
check("IMU and legs 2 deg apart: it says so", info["warning"] is not None, info["warning"] or "")
_, info = ff.pelvis_up({"joints": {}, "imu_rpy": None}, J)
check("neither: level assumed, and it says so", info["source"] == "level" and info["warning"])
fl = ff.FloorFrame([0, 0, 1], FLOOR_H, [0.0, 0.0, -0.69])
check("level, ankles under the pelvis: tape = (x, y, z + pelvis height), as before",
      np.allclose(fl.to_tape([0.36, -0.03, 0.07]), [0.36, -0.03, 0.07 + FLOOR_H]))
fl = ff.FloorFrame(UP, FLOOR_H, ff.ankle_midpoint(J, pose(0, 0)))
p = np.array([0.36, -0.03, 0.07])
check("leaning: tape -> pelvis -> tape round trip", np.allclose(fl.to_pelvis(fl.to_tape(p)), p))
q1, q2 = fl.to_pelvis((0.30, 0.0, 0.74)), fl.to_pelvis((0.60, 0.2, 0.74))
check("two points at the same tape height lie on one level plane (in the lean)",
      abs(UP @ (q1 - q2)) < 1e-12)
lvl = ff.FloorFrame([0, 0, 1], FLOOR_H)
e = 1000 * np.linalg.norm(lvl.to_tape(p) - fl.to_tape(p))
check("assuming level under a 3 deg lean misplaces a plate's tape numbers by cm", e > 20,
      "%.0f mm" % e)

print("\n  ONE POSE: TAKING THE LEAN FROM THE ROBOT, NOT ASSUMING LEVEL")
mount_xy = fk.homogeneous(cc.rot([R(0.8), R(-1.5), 0.0]) @ M_CAD[:3, :3], M_CAD[:3, 3])
a0 = pose(0, 0)
n, c = cc.predict_plane(true_T(a0, mount_xy, (0, 0)), UP, FLOOR_H)
chain = {k: a0[k] for k in cfg.CAMERA_CHAIN_JOINTS}
cal_level = cc.from_level_surface(n, -c, J, chain)
cal_imu = cc.from_level_surface(n, -c, J, chain, up_pelvis=UP, body=state(a0))
a30 = pose(30, 0)
e_imu = plate_error_mm(cc.T_pelvis_optical(cal_imu, J, a30), true_T(a30, mount_xy, (0, 0)))
e_lvl = plate_error_mm(cc.T_pelvis_optical(cal_level, J, a30), true_T(a30, mount_xy, (0, 0)))
check("lean from the robot: the plate where it is, in the arm's frame", e_imu < 1.0,
      "%.2f mm" % e_imu)
check("lean assumed level (the old way): the plate off by cm in the arm's frame", e_lvl > 15,
      "%.0f mm -- the 3 deg lean, folded into the camera" % e_lvl)
check("the pelvis' height above the floor comes out right",
      abs(cal_imu["pelvis_above_floor_m"] - FLOOR_H) < 1e-6)
check("...and the ankles' height is kept, so the floor follows the stance later",
      cal_imu.get("ankle_above_floor_m") is not None)

print("\n  THE SWEEP: WHAT THE OLD CALIBRATION GETS WRONG AWAY FROM YAW 0")
n0, c0 = cc.predict_plane(true_T(a0), UP, FLOOR_H)
base = cc.from_level_surface(n0, -c0, J, chain)       # what the robot has: one pose, level
pitches, yaws = cs.GRIDS["table"]
obs = obs_from_truth(pitches, yaws)
rows = cs.evaluate(base, obs, J, "  the one-pose calibration on a table sweep:")
dh = np.array([r[2] for r in rows])
s = np.sin(np.radians([o["yaw"] for o in obs]))
slope = np.polyfit(s, dh, 1)[0]
check("its surface height drifts with sin(yaw), as measured on the robot", abs(slope) > 5,
      "%+.1f mm x sin(yaw) here (robot, 22 Oct: -27.6)" % slope)
push = pose(20, 17)                                   # where the push looks from
e_base = plate_error_mm(cc.T_pelvis_optical(base, J, push), true_T(push))
check("at the push's head pose (17 deg yaw) the plate is off by cm", e_base > 10,
      "%.0f mm" % e_base)

print("\n  THE SWEEP FIT")
t0 = time.time()
x, model, rep = cs.fit(obs, J, base, 1, log=lambda *a: None)
dt = time.time() - t0
check("held-out poses (left out of the fit) agree to under 2 mm", rep["held_out_rms_mm"] < 2.0,
      "%.2f mm rms (fitted ones %.2f); %.1f s" % (rep["held_out_rms_mm"], rep["train_rms_mm"], dt))
neck_err = max(abs(D(x[3]) - D(TRUE_NECK[0])), abs(D(x[4]) - D(TRUE_NECK[1])))
check("the yaw axis' tip is found", neck_err < 0.3,
      "%s deg, truly %s" % (np.round(D(1) * x[3:5], 2).tolist(), [1.6, 0.7]))
cal = cs.calibration(x, model, obs, [({"surface": "table", "table_height_m": TABLE_ABOVE_FLOOR},
                                      None)], base, rep, "imu", ["synthetic"])
errs = [plate_error_mm(cc.T_pelvis_optical(cal, J, pose(p, y)), true_T(pose(p, y)))
        for p in (15, 20, 25, 33) for y in (-30, -17, -5, 0, 9, 17, 30)]
check("plates placed right at every head pose, inside the sweep and beyond it",
      max(errs) < 3.0, "worst %.2f mm over 28 poses (yaw to 30 deg)" % max(errs))
e_new = plate_error_mm(cc.T_pelvis_optical(cal, J, push), true_T(push))
check("...the push's head pose included", e_new < 3.0, "%.2f mm (was %.0f mm)" % (e_new, e_base))
check("the floor's height comes from the table's taped height",
      abs(cal["pelvis_above_floor_m"] - FLOOR_H) < 0.003,
      "%.4f m, truly %.4f" % (cal["pelvis_above_floor_m"], FLOOR_H))
check("the saved file is JSON", bool(json.dumps(cal)))

print("\n  PLANES IN A DEPTH FRAME (rendered: a table, the floor beyond it)")


def render(T_po, table_x=(0.18, 0.95), noise_mm=1.0):
    h, w = INTR["height"], INTR["width"]
    vv, uu = np.mgrid[0:h, 0:w]
    d = np.stack([(uu - INTR["cx"]) / INTR["fx"], (vv - INTR["cy"]) / INTR["fy"],
                  np.ones((h, w))], axis=-1)
    out = np.zeros((h, w))
    for H, is_table in ((FLOOR_H, False), (TABLE_H, True)):
        n, c = cc.predict_plane(T_po, UP, H)
        t = c / (d @ n)
        p = d * t[..., None]
        pel = p @ T_po[:3, :3].T + T_po[:3, 3]
        ok_ = t > 0
        if is_table:
            ok_ &= (pel[..., 0] > table_x[0]) & (pel[..., 0] < table_x[1]) & (np.abs(pel[..., 1]) < 0.9)
        out = np.where(ok_, t, out)                  # the table, where it is, covers the floor
    out = out + rng.normal(0.0, noise_mm / 1000.0, out.shape)
    out[(out < 0.1) | (out > 6.0)] = 0
    return np.round(out * 1000).astype(np.uint16)


a = pose(15, 0)
T = true_T(a)
dep = render(T)
e_up = T[:3, :3].T @ UP
planes = cs.level_planes(dep, INTR, 1000.0, e_up, cs.MAX_RANGE_M["table"])
tab = cs.pick_plane(planes, "table")
nt, ct = cc.predict_plane(T, UP, TABLE_H)
check("the table is found, not the floor beyond it",
      tab is not None and ff.angle_deg(tab[0], nt) < 0.1 and abs(tab[1] - ct) < 0.002,
      "%.3f deg, %.1f mm off" % (ff.angle_deg(tab[0], nt), 1000 * abs(tab[1] - ct)) if tab else "")
fl_ = cs.pick_plane(cs.level_planes(dep, INTR, 1000.0, e_up, 6.0), "floor")
nf, cf = cc.predict_plane(T, UP, FLOOR_H)
check("...and asked for the floor, the floor", fl_ is not None and abs(fl_[1] - cf) < 0.005,
      "%.1f mm off" % (1000 * abs(fl_[1] - cf)) if fl_ else "")

print("\n  END TO END: a sweep folder -> calibrate_sweep.py fit --save")
tmp = tempfile.mkdtemp()
folder = os.path.join(tmp, "sweep_table")
os.makedirs(folder)
for i, (p, y) in enumerate([(p, y) for p in pitches for y in yaws]):
    a = pose(p, y)
    np.savez_compressed(os.path.join(folder, "pose_%02d.npz" % i),
                        depth=np.stack([render(true_T(a))]), intr=json.dumps(INTR),
                        depth_scale=1000.0, state=json.dumps(state(a)), asked=json.dumps([p, y]))
with open(os.path.join(folder, "sweep.json"), "w") as f:
    json.dump({"surface": "table", "table_height_m": TABLE_ABOVE_FLOOR}, f)
cal_path = os.path.join(tmp, "camera_calibration.json")
cc.save(base, cal_path)
import contextlib
import io
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    code = cs.main(["fit", folder, "--cal", cal_path, "--save"])
saved = cc.load(cal_path)
check("fit --save writes a new calibration and keeps the old one",
      code == 0 and saved.get("version") == 2 and
      any(n.startswith("camera_calibration.json.before_") for n in os.listdir(tmp)))
e2e = [plate_error_mm(cc.T_pelvis_optical(saved, J, pose(p, y)), true_T(pose(p, y)))
       for p, y in ((20, 17), (25, -17), (33, 24), (15, 0))]
check("from rendered depth frames: plates right to a few mm at any head pose",
      max(e2e) < 4.0, "worst %.2f mm (old: %.0f mm at the push pose)" % (max(e2e), e_base))
held = [l for l in buf.getvalue().splitlines() if "held-out poses: rms" in l]
check("it reports the held-out test", bool(held), held[0].strip() if held else "")

print("\n  THE YAW PLAY: EVERY YAW ARRIVED AT MOVING +")


def new_cmd():
    return types.SimpleNamespace(
        mode_pr=0, mode_machine=0, crc=0,
        motor_cmd=[types.SimpleNamespace(mode=0, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
                   for _ in range(cfg.NUM_SLOTS)])


class FakeHead:
    def __init__(self, link):
        self.link, self.sent = link, []
        self.q = {n: 0.0 for n in cfg.MOTOR_SLOT}
        self.publish()

    def publish(self):
        ms = [types.SimpleNamespace(q=0.0) for _ in range(cfg.NUM_SLOTS)]
        for n, s_ in cfg.MOTOR_SLOT.items():
            ms[s_].q = self.q[n]
        self.link.feed(types.SimpleNamespace(motor_state=ms, mode_machine=5))

    def Write(self, cmd):
        self.sent.append(cmd)
        w = cmd.mode_pr / 100.0
        for n in cfg.HEAD_JOINTS:
            self.q[n] += 0.5 * w * (cmd.motor_cmd[cfg.MOTOR_SLOT[n]].q - self.q[n])
        self.publish()


link = rh.R1Link(subscribe=False)
robot = FakeHead(link)
head = rh.HeadController(link, publisher=robot, new_cmd=new_cmd, crc=False, rate_hz=200,
                         speed_deg_s=60, ramp_s=0.2, log=lambda *a: None).engage()
ys = lambda: [D(c.motor_cmd[cfg.MOTOR_SLOT["head_yaw_joint"]].q) for c in robot.sent]  # noqa
head.move_to(10, 12)
seq = ys()
check("12 deg to the left of where it is: straight there", min(seq) > -0.01 and
      all(b >= a - 1e-9 for a, b in zip(seq, seq[1:])))
n0 = len(robot.sent)
head.move_to(10, -6)
seq = ys()[n0:]
check("back to the right: it goes 4 deg past, then comes in moving +",
      abs(min(seq) - (-10.0)) < 0.05 and abs(seq[-1] - (-6.0)) < 0.05,
      "lowest %.2f, ends %.2f" % (min(seq), seq[-1]))
last = seq[seq.index(min(seq)):]
check("...its last stretch only increases", all(b >= a - 1e-9 for a, b in zip(last, last[1:])))
n0 = len(robot.sent)
head.move_to(10, -4)
seq = ys()[n0:]
check("2 deg left (less than the play margin): still comes in from 4 deg right",
      abs(min(seq) - (-8.0)) < 0.05, "lowest %.2f" % min(seq))
head.release()

print("\n  TAPE NUMBERS UNDER THE LEAN (the hand measurement, the plate report)")
pose_m = rest = rp.rest_pose()
pose_m.update({n: 0.0 for n in cfg.arm_joints("right")})
pose_m.update({k: v for k, v in pose(0, 0).items() if "hip" in k or "knee" in k or "ankle" in k})
Tw = fk.T_pelvis_wrist(J, pose_m, "right")
true_tool = np.array([0.13, -0.01, 0.02])
pinch = Tw[:3, :3] @ true_tool + Tw[:3, 3]
fl = ff.FloorFrame(UP, FLOOR_H, ff.ankle_midpoint(J, pose_m))
f_, l_, h_ = 100 * fl.to_tape(pinch)
tool, palm, _ = rp.hand_frame_from_measurement(J, pose_m, "right", f_, l_, h_, fl, "down")
check("measure-hand with the lean known: the pinch point exactly", np.allclose(tool, true_tool),
      str(np.round(tool, 4)))
tool_lvl, _, _ = rp.hand_frame_from_measurement(J, pose_m, "right", f_, l_, h_, FLOOR_H, "down")
check("the same tape numbers read as if level: off by cm",
      1000 * np.linalg.norm(tool_lvl - true_tool) > 15,
      "%.0f mm" % (1000 * np.linalg.norm(tool_lvl - true_tool)))

import check_stack  # noqa: E402
a = pose(30, 0)
T = true_T(a)
p_opt, p_true = ray_hit(T, TABLE_H - 0.044)          # the top plate's centre, 4.4 cm up
nt, _ = cc.predict_plane(T, UP, TABLE_H)
cal_imu_sweep = dict(cal)
cal_imu_sweep["ankle_above_floor_m"] = FLOOR_H + UP @ ff.ankle_midpoint(J, a)
orig = cc.load
cc.load = lambda *a_, **k_: cal_imu_sweep
try:
    with contextlib.redirect_stdout(io.StringIO()):
        out = check_stack.robot_frame_report(
            [{"center_m": p_opt, "normal": -nt, "top_height_m": 0.044, "diameter_m": 0.254,
              "n_plates": 4}], (nt, 0.0), angles={k: a[k] for k in cfg.CAMERA_CHAIN_JOINTS},
            body=state(a))
finally:
    cc.load = orig
tape_true = ff.FloorFrame(UP, FLOOR_H, ff.ankle_midpoint(J, a)).to_tape(p_true)
e = 1000 * np.linalg.norm(np.array(out[0]["tape_m"]) - tape_true)
check("check_stack's tape numbers for a plate, the robot leaning, match the truth", e < 3.0,
      "%.2f mm" % e)
check("...and its centre_m is the pelvis-frame position (what the arm needs)",
      1000 * np.linalg.norm(np.array(out[0]["center_m"]) - p_true) < 3.0)

print("\n%s" % ("ALL PASS" if ok else "SOME FAILED"))
sys.exit(0 if ok else 1)
