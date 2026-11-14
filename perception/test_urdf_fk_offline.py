"""Off-robot test for the R1 forward kinematics and the helmet sensor frames.

    python3 test_urdf_fk_offline.py

No robot, no camera. Everything here is R1.urdf plus arithmetic.

These tests exist because the port from the G1 is exactly the kind of change that looks
finished and is not: the arm chain has the same five joint names on both robots, so code
written for the G1 imports, runs, and returns plausible numbers for the WRONG robot. Each
check below pins down one thing that differs, so a silent regression to G1 assumptions fails
here instead of on hardware.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import r1_config as cfg
import urdf_fk as fk

ok = True


def check(label, cond, detail=""):
    global ok
    print("  %-62s %s%s" % (label, "PASS" if cond else "FAIL",
                            ("   " + detail) if detail and not cond else ""))
    if detail and cond:
        print("  %-62s      %s" % ("", detail))
    if not cond:
        ok = False


joints = fk.load_urdf(cfg.URDF_PATH)
D = np.degrees


def max_reach(joints, arm="left", refine=6):
    """Largest shoulder-to-wrist distance the joint limits allow.

    Coarse grid then hill-climb. Only four joints matter: wrist_roll spins the wrist
    link about its own axis and cannot move its origin.
    """
    import itertools

    lim = cfg.joint_limits(joints)
    names = ["%s_%s" % (arm, k) for k in
             ("shoulder_pitch_joint", "shoulder_roll_joint",
              "shoulder_yaw_joint", "elbow_joint")]
    sh = fk.shoulder_positions(joints)[arm]

    def dist(vals):
        a = dict(zip(names, vals))
        return float(np.linalg.norm(fk.T_pelvis_wrist(joints, a, arm=arm)[:3, 3] - sh))

    best = (0.0, None)
    for combo in itertools.product(*[np.linspace(lim[n][0], lim[n][1], 9) for n in names]):
        d = dist(combo)
        if d > best[0]:
            best = (d, combo)
    for _ in range(refine):
        step = [(lim[n][1] - lim[n][0]) / 20.0 for n in names]
        for i in range(len(names)):
            for sgn in (-1, 1):
                t = list(best[1])
                t[i] = float(np.clip(t[i] + sgn * step[i], *lim[names[i]]))
                d = dist(t)
                if d > best[0]:
                    best = (d, tuple(t))
    return best[0]


def axis_elevation(T):
    """Degrees the camera's optical axis points BELOW horizontal."""
    a = T[:3, :3] @ np.array([1.0, 0.0, 0.0])
    return float(D(np.arcsin(-a[2] / np.linalg.norm(a))))


print("\n  THE URDF IS THE R1'S, WITH THE HELMET ADDED")
check("it loads", len(joints) > 0, "%d joints" % len(joints))
check("the root link is pelvis_link, not the G1's 'pelvis'",
      "pelvis_link" in {j["parent"] for j in joints.values()})
try:
    fk.find_chain(joints, "pelvis", "d435_link")
    leaked = True
except ValueError:
    leaked = False
check("a chain rooted at 'pelvis' is refused, not silently found", not leaked,
      "G1 code pasted in unchanged must fail loudly")
check("26 movable joints",
      sum(1 for j in joints.values() if j["type"] != "fixed") == 26)
for link in (cfg.CAMERA_LINK, cfg.CAMERA_OPTICAL_LINK, cfg.LIDAR_LINK):
    check("%s exists" % link, link in {j["child"] for j in joints.values()})
check("the arm chain ends at the wrist, with no hand link",
      not any("hand" in n or "palm" in n or "finger" in n
              for j in joints.values() for n in (j["child"],)))

print("\n  THE SENSOR FRAMES MATCH THE HELMET CAD")
cam_j, lid_j = joints["d435_joint"], joints["jt128_joint"]
check("camera origin is the CAD's",
      np.allclose(cam_j["xyz"], cfg.NOMINAL_CAMERA_IN_HEAD_YAW, atol=1e-9),
      "%s m" % np.array2string(cam_j["xyz"], precision=6))
check("camera is a pure pitch, no roll or yaw",
      abs(cam_j["rpy"][0]) < 1e-12 and abs(cam_j["rpy"][2]) < 1e-12)
check("camera pitch is 30 deg",
      abs(cam_j["rpy"][1] - cfg.NOMINAL_CAMERA_PITCH_RAD) < 1e-6,
      "%.4f deg" % D(cam_j["rpy"][1]))
check("lidar origin is the CAD's",
      np.allclose(lid_j["xyz"], cfg.NOMINAL_LIDAR_IN_HEAD_YAW, atol=1e-9),
      "%s m" % np.array2string(lid_j["xyz"], precision=6))
check("lidar pitch is 24 deg",
      abs(lid_j["rpy"][1] - cfg.NOMINAL_LIDAR_PITCH_RAD) < 1e-6,
      "%.4f deg" % D(lid_j["rpy"][1]))
check("both hang off head_yaw_link, so the head moves them",
      cam_j["parent"] == "head_yaw_link" and lid_j["parent"] == "head_yaw_link")

print("\n  THE OPTICAL FRAME IS THE CONVENTION IT CLAIMS TO BE")
R = fk.T(joints, cfg.CAMERA_LINK, cfg.CAMERA_OPTICAL_LINK)[:3, :3]
check("optical z is the body's x (forward)", np.allclose(R[:, 2], [1, 0, 0], atol=1e-6))
check("optical x is the body's -y (right)", np.allclose(R[:, 0], [0, -1, 0], atol=1e-6))
check("optical y is the body's -z (down)", np.allclose(R[:, 1], [0, 0, -1], atol=1e-6))
check("it is a proper rotation", abs(np.linalg.det(R) - 1.0) < 1e-9)

print("\n  THE CAMERA IS WHERE IT SHOULD BE AT ZERO POSE")
T0 = fk.T_pelvis_camera(joints)
check("camera position in pelvis_link",
      np.allclose(T0[:3, 3], [0.1225, 0.0, 0.4705], atol=1e-4),
      "%s m" % np.array2string(T0[:3, 3], precision=4))
check("optical axis is 30 deg below horizontal",
      abs(axis_elevation(T0) - 30.0) < 1e-3, "%.4f deg" % axis_elevation(T0))
check("camera sits forward of the shoulders",
      T0[0, 3] > fk.shoulder_positions(joints)["left"][0],
      "camera x %.4f vs shoulder x %.4f"
      % (T0[0, 3], fk.shoulder_positions(joints)["left"][0]))

print("\n  BOTH WAIST JOINTS MOVE THE CAMERA -- NOT JUST YAW LIKE THE G1")
T_yaw = fk.T_pelvis_camera(joints, {"waist_yaw_joint": 0.4})
T_roll = fk.T_pelvis_camera(joints, {"waist_roll_joint": 0.4})
check("waist_yaw moves it", not np.allclose(T_yaw[:3, 3], T0[:3, 3], atol=1e-6),
      "moved %.1f mm" % (1000 * np.linalg.norm(T_yaw[:3, 3] - T0[:3, 3])))
check("waist_roll moves it too", not np.allclose(T_roll[:3, 3], T0[:3, 3], atol=1e-6),
      "moved %.1f mm -- code that only passes waist_yaw loses this"
      % (1000 * np.linalg.norm(T_roll[:3, 3] - T0[:3, 3])))
check("and they are not the same motion",
      not np.allclose(T_yaw[:3, 3], T_roll[:3, 3], atol=1e-6))

print("\n  THE HEAD AIMS THE CAMERA, WHICH THE G1 COULD NOT DO AT ALL")
lim = cfg.joint_limits(joints)
p_lo, p_hi = lim["head_pitch_joint"]
e_lo = axis_elevation(fk.T_pelvis_camera(joints, {"head_pitch_joint": p_lo}))
e_hi = axis_elevation(fk.T_pelvis_camera(joints, {"head_pitch_joint": p_hi}))
check("the pitch limits are exactly +/-36 deg",
      abs(D(p_hi) - 36.0) < 1e-3 and abs(D(p_lo) + 36.0) < 1e-3,
      "%.3f to %.3f deg" % (D(p_lo), D(p_hi)))
check("full-up pitch aims 6 deg ABOVE horizontal", abs(e_lo + 6.0) < 0.02,
      "%.2f deg (negative = above)" % e_lo)
check("full-down pitch aims 66 deg below", abs(e_hi - 66.0) < 0.02, "%.2f deg" % e_hi)
check("the G1's fixed 47.6 deg is inside that range", e_lo < 47.6 < e_hi,
      "so any G1 viewing geometry can be reproduced")
check("head_yaw swings the camera off the sagittal plane",
      abs(fk.T_pelvis_camera(joints, {"head_yaw_joint": 0.8})[1, 3]) > 0.02)

print("\n  AIMING SOLVES, CLAMPS, AND ADMITS WHEN IT CANNOT")
for tgt in ([0.35, 0.0, 0.30], [0.25, 0.20, 0.30], [0.30, -0.25, 0.28],
            [0.20, 0.30, 0.30]):
    r = fk.aim_head_at(joints, tgt)
    check("aims at %s" % np.array2string(np.array(tgt), precision=2),
          r["residual_deg"] < 1e-3 and not r["clamped"],
          "residual %.6f deg" % r["residual_deg"])
r_far = fk.aim_head_at(joints, [0.9, 0.0, 0.9])
check("a target above the neck's travel reports clamped", r_far["clamped"],
      "residual %.1f deg, so the caller can tell it is not in frame"
      % r_far["residual_deg"])
r_lim = fk.aim_head_at(joints, [0.05, 0.0, -0.5])
check("clamped angles still obey the joint limits",
      p_lo - 1e-9 <= r_lim["head_pitch_joint"] <= p_hi + 1e-9 and
      lim["head_yaw_joint"][0] - 1e-9 <= r_lim["head_yaw_joint"]
      <= lim["head_yaw_joint"][1] + 1e-9)
r_trunk = fk.aim_head_at(joints, [0.30, 0.10, 0.30], {"waist_yaw_joint": 0.3})
check("it still solves with the trunk turned", r_trunk["residual_deg"] < 1e-3,
      "residual %.6f deg" % r_trunk["residual_deg"])

print("\n  THE ARMS ARE SHORT, AND SYMMETRIC")
wl = fk.T_pelvis_wrist(joints, arm="left")[:3, 3]
wr = fk.T_pelvis_wrist(joints, arm="right")[:3, 3]
check("left and right wrists mirror in y",
      np.allclose(wl * np.array([1, -1, 1]), wr, atol=1e-9))
sh = fk.shoulder_positions(joints)
reach = np.linalg.norm(wl - sh["left"])
check("shoulder to wrist at the zero pose is 0.2564 m", abs(reach - 0.2564) < 1e-3,
      "%.4f m" % reach)
check("the geometric maximum over the joint limits is 0.3587 m",
      abs(max_reach(joints) - 0.3587) < 2e-3,
      "%.4f m -- a contorted pose, so usable reach is less, and the "
      "dexterous hand adds an amount nobody has measured" % max_reach(joints))
check("an elbow bend moves the wrist",
      np.linalg.norm(fk.T_pelvis_wrist(joints, {"left_elbow_joint": 0.8},
                                       arm="left")[:3, 3] - wl) > 0.05)
kp = fk.arm_keypoints(joints, {"left_elbow_joint": 0.5}, arm="left")
check("arm keypoints give shoulder, elbow and wrist",
      set(kp) == {"shoulder_yaw_joint", "elbow_joint", "wrist_roll_joint"})

print("\n  WHAT IS NOT KNOWN REFUSES TO GUESS")
for label, fn in (
    ("tool_offset raises until the dexterous hand is measured",
     lambda: cfg.tool_offset("left")),
    ("T_pelvis_tool raises for the same reason",
     lambda: fk.T_pelvis_tool(joints, arm="left")),
):
    try:
        fn()
        check(label, False, "it returned a value instead")
    except NotImplementedError:
        check(label, True)

print("\n  THE MOTOR MAP IS THE R1'S, FROM UNITREE'S R1 CODE")
slots = cfg.motor_index()
movable = {n for n, j in joints.items() if j["type"] != "fixed"}
check("every movable URDF joint has a slot, and nothing else does",
      set(slots) == movable, "%d slots for %d joints" % (len(slots), len(movable)))
check("no two joints share a slot", len(set(slots.values())) == len(slots))
check("all slots fit the 35-slot message", all(0 <= s < cfg.NUM_SLOTS for s in slots.values()))
check("left arm is 15-19 and right arm 22-26, in chain order",
      [slots[n] for n in cfg.arm_joints("left")] == [15, 16, 17, 18, 19]
      and [slots[n] for n in cfg.arm_joints("right")] == [22, 23, 24, 25, 26])
check("slot 29 is HEAD PITCH -- where the G1 kept its arm_sdk weight",
      slots["head_pitch_joint"] == 29,
      "G1 arm code unchanged would drive the head to 57 deg on every engage")
try:
    fk.T_pelvis_optical(joints)
    check("T_pelvis_optical refuses an uncalibrated chain by default", False)
except RuntimeError:
    check("T_pelvis_optical refuses an uncalibrated chain by default", True)
T_opt = fk.T_pelvis_optical(joints, allow_nominal=True)
check("...but allow_nominal=True gives the CAD nominal for geometry work",
      np.allclose(T_opt[:3, 3], T0[:3, 3], atol=1e-9),
      "same origin as the body frame, rotated into optical axes")

print("\n  THE SHARED ENGINE HAS NOT DRIFTED FROM THE G1 COPY")
# G1_Plate_Perception sits outside this repository, and how far outside has already
# changed once. Search upward rather than assume a depth -- this check degrades to SKIP
# when it cannot find the file, so a wrong path here would hide the drift it exists to catch.
g1 = None
_p = HERE
for _ in range(5):
    _c = os.path.join(_p, "G1_Plate_Perception", "urdf_fk.py")
    if os.path.exists(_c):
        g1 = _c
        break
    _up = os.path.dirname(_p)
    if _up == _p:
        break
    _p = _up
g1 = g1 or ""
if not os.path.exists(g1):
    print("  %-62s SKIP   (G1 module not present)" % "engine comparison")
else:
    import io

    SENTINEL = "# --- END OF SHARED URDF ENGINE"

    def engine_of(path):
        s = io.open(path, encoding="utf-8").read()
        cut = s.index(SENTINEL) if SENTINEL in s else s.index("def T_pelvis_camera(")
        body = s[:cut]
        return body[body.index('"""', body.index('"""') + 3) + 3:].strip()

    check("load_urdf/find_chain/fk_chain are byte-identical on both robots",
          engine_of(g1) == engine_of(os.path.join(HERE, "urdf_fk.py")),
          "a fix to the URDF engine must land in both, or they diverge")

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
