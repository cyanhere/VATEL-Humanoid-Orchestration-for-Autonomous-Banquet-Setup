"""Is the camera actually where the helmet CAD says it is?

    python3 check_mount.py --imu                  # fastest, if the D435i's IMU is alive
    python3 check_mount.py --live                 # from a view of a flat horizontal floor
    python3 check_mount.py floor.npz
    python3 check_mount.py --pair faceN.npz faceS.npz   # cancels a sloping floor
    python3 check_mount.py --watch 20 --interval 30     # thermal drift over 10 minutes

WHY THIS IS THE FIRST THING TO RUN
----------------------------------
The whole perception chain inherits the camera's mounting angle. On the G1 the URDF said the
camera sat 47.6 deg below horizontal and the real bracket was at ~71 deg; that 23 deg error put
the plate 230 mm from where it actually was, and it took a hand-touch calibration to find.

The helmet CAD predicts, for this robot, that with the torso upright and the head at zero the
camera's optical axis sits **30.0 deg below horizontal** with no roll. That is a falsifiable
prediction about a part that has just been printed and bolted on, and it costs one frame to
check.

HOW BIG AN ERROR MATTERS
------------------------
At a 0.35 m working range, a tilt error of

    1 deg ->   6 mm        3 deg ->  18 mm       10 deg ->  62 mm
    2 deg ->  12 mm        5 deg ->  31 mm       23 deg -> 149 mm

and plates nest at about 10 mm. So **2 deg already costs more than one plate pitch**: the hand
would close on the wrong plate. Aim to be inside 1-2 deg, or plan on solving the extrinsic by
calibration instead of trusting the CAD.

WHAT THIS MEASURES, HONESTLY
----------------------------
The number it reports is the camera's tilt relative to gravity. That is the sum of

    the helmet mount's error  +  the head joint's zero offset  +  the torso's lean

and this tool cannot separate them -- that needs the head encoders and the pelvis IMU, which
means the SDK. But separating them is not the point: the perception chain inherits the TOTAL,
so if the total is inside a couple of degrees you are fine no matter which part contributes,
and if it is not, you know before you waste a day on detections that were never going to land.

Two confounds you can remove without any robot software:
  - a SLOPING FLOOR cancels if you measure facing one way, then rotate the robot 180 deg and
    measure again, and average. Use --pair. Mount error does not cancel; floor slope does.
  - a LEANING TORSO does not cancel that way, because it is body-fixed. Put a spirit level or
    a plumb line against the torso before believing a small residual.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plate_detectors as pd
import r1_config as cfg
import urdf_fk as fk

NOMINAL_TILT_DEG = 30.0        # the CAD's camera nose pitch, head at zero
WORK_RANGE_M = 0.35


# --------------------------------------------------------------------------
# what the CAD predicts
# --------------------------------------------------------------------------

def predicted(head_pitch_rad=0.0, head_yaw_rad=0.0):
    """(tilt below horizontal in deg, floor normal in optical coords) from the URDF."""
    joints = fk.load_urdf(cfg.URDF_PATH)
    T = fk.T_pelvis_optical(joints, {"head_pitch_joint": head_pitch_rad,
                                     "head_yaw_joint": head_yaw_rad},
                            allow_nominal=True)
    up_opt = T[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    tilt = float(np.degrees(np.arcsin(-float(np.array([0.0, 0.0, 1.0]) @ up_opt))))
    return tilt, up_opt


def readout(up_opt):
    """From an upward unit vector in optical coords: (tilt below horizontal, roll), deg.

    The optical frame is x right, y down, z forward, so a vector pointing 'up' in the world
    should be almost exactly -y. Any x component is the helmet rolled on the head.
    """
    n = np.asarray(up_opt, dtype=float)
    n = n / np.linalg.norm(n)
    tilt = float(np.degrees(np.arcsin(-float(n[2]))))
    roll = float(np.degrees(np.arctan2(n[0], -n[1])))
    return tilt, roll


def cost_mm(err_deg, rng=WORK_RANGE_M):
    return 1000.0 * rng * np.tan(np.radians(abs(err_deg)))


# --------------------------------------------------------------------------
# measuring it: from the IMU, or from a flat floor
# --------------------------------------------------------------------------

def up_from_imu(samples=200):
    """Gravity in the camera optical frame, from the D435i's accelerometer.

    Better than the floor method when it works: no flat surface needed, no floor slope to
    cancel, and it takes a second. librealsense reports accel in the IMU's own frame, which
    on the D4xx is aligned with the DEPTH optical frame up to sign conventions -- so this is
    cross-checked against the floor method rather than trusted alone. Run both.
    """
    import pyrealsense2 as rs

    pipe = rs.pipeline()
    cfg_rs = rs.config()
    cfg_rs.enable_stream(rs.stream.accel)
    pipe.start(cfg_rs)
    try:
        acc = []
        for _ in range(samples):
            f = pipe.wait_for_frames()
            m = f.first_or_default(rs.stream.accel)
            if m:
                d = m.as_motion_frame().get_motion_data()
                acc.append([d.x, d.y, d.z])
        if len(acc) < 10:
            raise RuntimeError("no accelerometer samples -- is this a D435i/D435if?")
        a = np.mean(np.asarray(acc), axis=0)
    finally:
        pipe.stop()
    g = np.linalg.norm(a)
    # the accelerometer reads the reaction to gravity, so 'up' is +a
    return a / g, g, len(acc)


class NotFloor(RuntimeError):
    pass


def _points(depth, intr, scale, stride, max_range):
    d = np.asarray(depth)[::stride, ::stride].astype(np.float64) / scale
    h, w = d.shape
    yy, xx = np.mgrid[0:h, 0:w]
    ok = (d > 0.1) & (d < max_range)
    fx, fy, cx, cy = (intr[k] for k in ("fx", "fy", "cx", "cy"))
    z = d[ok]
    u, v = xx[ok] * stride, yy[ok] * stride
    return np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)


def _why_not_floor(depth, intr, scale, up, max_dev_deg):
    msg = ("no level surface found -- nothing flat in view is within %.0f deg of where the "
           "floor should be." % max_dev_deg)
    try:
        n, c = pd.dominant_plane(depth, intr, scale)
        off = float(np.degrees(np.arccos(np.clip(abs(float(n @ up)), 0.0, 1.0))))
        if off > 60.0:
            msg += ("\n             The biggest flat thing in view is %.1f m away and "
                    "upright: a WALL.\n             Turn the robot to face open floor, "
                    "nothing within ~3 m." % abs(c))
        else:
            msg += ("\n             The biggest flat thing in view is %.0f deg off level. "
                    "If that\n             really is the floor, the mount is grossly off."
                    % off)
    except RuntimeError:
        pass
    return msg


def up_from_floor(depth, intr, scale, expect_up=None, max_dev_deg=25.0, stride=4,
                  max_range=3.0, tol=0.015, iters=500, min_frac=0.05, seed=0):
    """The FLOOR's upward normal in optical coords, as (n, c, inlier_fraction).

    Not simply the biggest plane in view. With the head level, the top of the image looks out
    almost horizontally, so a wall a couple of metres away can outnumber the floor. The first
    run on the real robot fitted exactly that -- a wall 2.2 m away, 9% of pixels -- and
    reported it as a 40 deg mount error.

    So a candidate plane only counts if its normal is within max_dev_deg of where the CAD says
    the floor's normal is. That cannot hide a mount error we care about (the budget is 1-2
    deg); a mount off by more than 25 deg is refused outright, which is the right answer,
    since nothing downstream would work at that size anyway. Any LEVEL surface gives the same
    tilt -- a table top is as good as the floor for this.
    """
    if expect_up is None:
        expect_up = predicted(0.0)[1]
    e = np.asarray(expect_up, float)
    e = e / np.linalg.norm(e)
    pts = _points(depth, intr, scale, stride, max_range)
    if len(pts) < 200:
        raise NotFloor("almost no depth within %.1f m of the camera" % max_range)

    cos_lim = np.cos(np.radians(max_dev_deg))
    rng = np.random.default_rng(seed)
    best = (0, None, None)
    for _ in range(iters):
        p0, p1, p2 = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        if n @ e < 0:
            n = -n
        if n @ e < cos_lim:
            continue                       # not level: a wall, a box side, a person
        c = float(n @ p0)
        k = int((np.abs(pts @ n - c) < tol).sum())
        if k > best[0]:
            best = (k, n, c)
    if best[1] is None or best[0] < min_frac * len(pts):
        raise NotFloor(_why_not_floor(depth, intr, scale, e, max_dev_deg))

    n, c = best[1], best[2]
    # Refine on the inliers with a band that tightens each pass. Where only a thin strip of
    # floor shows below a wall, the foot of the wall sits inside a 15 mm band of the floor
    # and tilts the fit by ~0.4 deg; shrinking the band sheds it.
    for band in (tol, 0.6 * tol, 0.4 * tol):
        keep = np.abs(pts @ n - c) < max(band, 0.005)
        if keep.sum() < 50:
            break
        q = pts[keep]
        m = q.mean(axis=0)
        n = np.linalg.svd(q - m, full_matrices=False)[2][-1]
        if n @ e < 0:
            n = -n
        c = float(n @ m)
    return n, c, float(np.mean(np.abs(pts @ n - c) < tol))


def helmet_in_view(depth, scale, near_m=0.20, border=40):
    """Fraction of the frame returning something closer than near_m, and where.

    The CAD checked that no printed part enters the depth frustum. This is that check made
    against the real thing: a printed brow or the roll hoop shows up as a band of very near
    returns hugging one edge.
    """
    d = np.asarray(depth).astype(float) / scale
    near = (d > 0.01) & (d < near_m)
    if not near.any():
        return 0.0, {}
    h, w = d.shape
    where = {"top": float(near[:border, :].mean()),
             "bottom": float(near[-border:, :].mean()),
             "left": float(near[:, :border].mean()),
             "right": float(near[:, -border:].mean()),
             "middle": float(near[border:-border, border:-border].mean())}
    return float(near.mean()), where


# --------------------------------------------------------------------------
# frames
# --------------------------------------------------------------------------

def capture(warmup=30):
    """One depth frame, aligned to colour, straight off a RealSense (any USB link)."""
    import realsense_capture
    return realsense_capture.capture(warmup)


def save(name, color, depth, intr, s):
    if not name.endswith(".npz"):
        name += ".npz"
    extra = {} if color is None else {"color": color}     # depth-only on a weak USB link
    np.savez_compressed(name, depth=depth, intr=json.dumps(intr), depth_scale=s, **extra)
    return name


def load(path):
    z = np.load(path, allow_pickle=False)
    return (z["color"] if "color" in z else None, z["depth"],
            json.loads(str(z["intr"])), float(z["depth_scale"]))


# --------------------------------------------------------------------------
# the report
# --------------------------------------------------------------------------

def calibrate_from(frames, args, source=""):
    """Average the floor over several frames and turn it into a saved camera calibration.

    frames: list of (depth, intrinsics, depth_scale). Refuses if the frames disagree by more
    than half a degree -- that means the robot moved, or something walked through the view,
    and an average of a moving target is not a calibration.
    """
    import camera_calibration as cc

    _, exp_up = predicted(np.radians(args.head_pitch), np.radians(args.head_yaw))
    ns, cs_ = [], []
    for depth, intr, scale in frames:
        n, c, _ = up_from_floor(depth, intr, scale, expect_up=exp_up)
        ns.append(n)
        cs_.append(c)
    ns = np.asarray(ns)
    mean = ns.mean(axis=0)
    mean /= np.linalg.norm(mean)
    spread = max(float(np.degrees(np.arccos(np.clip(n @ mean, -1, 1)))) for n in ns)
    if spread > 0.5:
        raise NotFloor("the %d floor readings disagree by up to %.2f deg -- hold the robot "
                       "still and keep people out of view" % (len(ns), spread))
    height = -float(np.median(cs_))
    angles = {"head_pitch_joint": float(np.radians(args.head_pitch)),
              "head_yaw_joint": float(np.radians(args.head_yaw))}
    angles.update(getattr(args, "waist", None) or {})      # read off the robot with --robot
    body = getattr(args, "body", None)                      # ...and its lean
    up_p = None
    if body is not None:
        import floor_frame as ff
        up_p, info = ff.pelvis_up(body)
        print(ff.describe(info))
    cal = cc.from_level_surface(mean, camera_height_m=height, angles=angles,
                                source=source or ("check_mount.py --calibrate, %d frames, "
                                                  "spread %.2f deg" % (len(ns), spread)),
                                surface="floor", up_pelvis=up_p, body=body)
    return cal, spread


def verdict(err_deg, rng):
    mm = cost_mm(err_deg, rng)
    if abs(err_deg) <= 1.0:
        return "GOOD -- the CAD nominal is usable as the extrinsic (%.0f mm at %.2f m)" % (mm, rng)
    if abs(err_deg) <= 2.0:
        return ("MARGINAL -- %.0f mm at %.2f m, about one plate pitch. Usable to find a\n"
                "            stack, not to pick off it. Calibrate before grasping."
                % (mm, rng))
    return ("TOO BIG -- %.0f mm at %.2f m. Do not trust the CAD extrinsic; either reseat\n"
            "           the mount or solve the camera pose by calibration." % (mm, rng))


def report_one(depth, intr, scale, args, label=""):
    exp_tilt, exp_up = predicted(np.radians(args.head_pitch), np.radians(args.head_yaw))
    if label:
        print("\n--- %s" % label)
    try:
        n, c, inl = up_from_floor(depth, intr, scale, expect_up=exp_up)
    except NotFloor as e:
        print("  NOT THE FLOOR  %s" % e)
        return None
    tilt, roll = readout(n)

    print("  floor      found: %.0f%% of the nearby picture, camera %.2f m above it"
          % (100 * inl, -c))
    print("  expected   tilt %.1f deg below horizontal, roll 0" % exp_tilt)
    print("  measured   tilt %.2f deg below horizontal, roll %+.2f deg" % (tilt, roll))
    print("  error      tilt %+.2f deg, roll %+.2f deg" % (tilt - exp_tilt, roll))
    print("  %s" % verdict(max(abs(tilt - exp_tilt), abs(roll)), args.range))

    frac, where = helmet_in_view(depth, scale, args.near)
    if frac > 0.001:
        hot = sorted(where.items(), key=lambda kv: -kv[1])[:2]
        print("  OCCLUSION  %.2f%% of the frame is closer than %.0f mm, mostly %s."
              % (100 * frac, 1000 * args.near,
                 ", ".join("%s %.1f%%" % (k, 100 * v) for k, v in hot)))
        print("             If that hugs one edge it is a printed part in the depth\n"
              "             frustum -- the CAD said none should be. Check the brow bumper\n"
              "             and the roll hoop.")
    else:
        print("  occlusion  clear: nothing within %.0f mm of the camera" % (1000 * args.near))
    return tilt, roll


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", nargs="?", help="a saved .npz")
    ap.add_argument("--capture", metavar="NAME", help="save a frame and stop")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--imu", action="store_true",
                    help="use the D435i accelerometer instead of a floor plane")
    ap.add_argument("--pair", nargs=2, metavar=("A.npz", "B.npz"),
                    help="two frames taken 180 deg apart; averaging cancels floor slope")
    ap.add_argument("--watch", type=int, metavar="N",
                    help="take N measurements and report the drift between them")
    ap.add_argument("--interval", type=float, default=30.0, help="seconds, with --watch")
    ap.add_argument("--calibrate", nargs="?", const=15, type=int, metavar="N",
                    help="measure the floor N times (default 15) and SAVE the result as the "
                         "camera calibration that check_stack.py uses for robot coordinates")
    ap.add_argument("--calibrate-from", nargs="+", metavar="FLOOR.npz",
                    help="the same, from saved frames of the floor")
    ap.add_argument("--robot", metavar="IFACE",
                    help="read the head angles from the robot (eth0) instead of --head-pitch/--head-yaw")
    ap.add_argument("--head-pitch", type=float, default=0.0,
                    help="head_pitch in DEGREES if the head is not at zero")
    ap.add_argument("--head-yaw", type=float, default=0.0, help="head_yaw in degrees")
    ap.add_argument("--range", type=float, default=WORK_RANGE_M,
                    help="working range the error is costed at, m")
    ap.add_argument("--near", type=float, default=0.20,
                    help="anything closer than this counts as occlusion, m")
    args = ap.parse_args(argv)

    if args.robot:
        import r1_head
        a = r1_head.R1Link(args.robot).wait().angles()
        args.head_pitch = float(np.degrees(a["head_pitch_joint"]))
        args.head_yaw = float(np.degrees(a["head_yaw_joint"]))
        print("from the robot: head pitch %+.2f deg, yaw %+.2f deg; waist roll %+.2f, yaw %+.2f"
              % (args.head_pitch, args.head_yaw, np.degrees(a["waist_roll_joint"]),
                 np.degrees(a["waist_yaw_joint"])))
        # stored with the calibration, so a waist slightly off zero is accounted for, not assumed
        args.waist = {k: float(a[k]) for k in ("waist_roll_joint", "waist_yaw_joint")}
        # and the lean: the pelvis is NOT level standing (IMU: 3 deg forward, 22 Oct), and a
        # calibration that assumes it is puts the lean into the camera
        args.body = r1_head.R1Link(args.robot).wait().state()

    if args.capture:
        color, depth, intr, s = capture()
        print("saved %s" % save(args.capture, color, depth, intr, s))
        return 0

    if args.calibrate or args.calibrate_from:
        import camera_calibration as cc
        if args.calibrate_from:
            frames = [load(p)[1:] for p in args.calibrate_from]
        else:
            print("measuring the floor %d times -- keep the robot still and the view clear"
                  % args.calibrate)
            frames = []
            for i in range(args.calibrate):
                _, depth, intr, s = capture(warmup=30 if i == 0 else 4)
                frames.append((depth, intr, s))
        try:
            cal, spread = calibrate_from(frames, args)
        except NotFloor as e:
            print("NOT SAVED: %s" % e)
            return 1
        path = cc.save(cal)
        print("saved %s" % path)
        print("  camera tilt   %.2f deg below horizontal (CAD %.1f, so %+.2f)"
              % (cal["measured_tilt_deg"], cal["cad_tilt_deg"], cal["correction_tilt_deg"]))
        print("  camera roll   %+.2f deg" % cal["measured_roll_deg"])
        print("  camera        %.3f m above the floor; robot frame origin %.3f m above it"
              % (cal["camera_above_floor_m"], cal["pelvis_above_floor_m"]))
        print("  frames agree  to within %.2f deg" % spread)
        if getattr(args, "body", None) is None:
            print("  pelvis        taken as LEVEL -- run with --robot to use its real lean")
        print("  (one head pose: right where it was taken. Over head yaw, calibrate_sweep.py)")
        print("\ncheck_stack.py will now report plate positions in the robot frame.")
        return 0

    if args.imu:
        exp_tilt, exp_up = predicted(np.radians(args.head_pitch), np.radians(args.head_yaw))
        up, g, n = up_from_imu()
        tilt, roll = readout(up)
        print("accelerometer: %d samples, |g| = %.3f m/s^2 (expect ~9.81)" % (n, g))
        if abs(g - 9.81) > 0.5:
            print("  |g| is off: the robot is moving, or the IMU needs calibrating.")
        print("  up vector  [% .4f % .4f % .4f]" % (up[0], up[1], up[2]))
        print("  expected   [% .4f % .4f % .4f]   tilt %.1f deg"
              % (exp_up[0], exp_up[1], exp_up[2], exp_tilt))
        print("  measured   tilt %.2f deg below horizontal, roll %+.2f deg" % (tilt, roll))
        print("  error      tilt %+.2f deg" % (tilt - exp_tilt))
        print("  %s" % verdict(tilt - exp_tilt, args.range))
        print("\nNow run the floor-plane check too. Two independent routes to the same\n"
              "angle is the only way to tell a real mount error from a sign convention.")
        return 0

    if args.pair:
        a = report_one(*load(args.pair[0])[1:], args=args, label="frame A")
        b = report_one(*load(args.pair[1])[1:], args=args, label="frame B (rotated 180 deg)")
        if a is None or b is None:
            print("\nCannot combine: one of the two frames did not show the floor.")
            return 1
        exp_tilt, _ = predicted(np.radians(args.head_pitch), np.radians(args.head_yaw))
        mean_tilt = 0.5 * (a[0] + b[0])
        slope = 0.5 * (a[0] - b[0])
        print("\n--- combined")
        print("  mean tilt        %.2f deg  (floor slope cancels in the mean)" % mean_tilt)
        print("  half-difference  %.2f deg  <- this IS the floor's slope" % slope)
        print("  mount error      %+.2f deg" % (mean_tilt - exp_tilt))
        print("  %s" % verdict(mean_tilt - exp_tilt, args.range))
        if abs(slope) > 1.0:
            print("  That floor is not flat. Good thing you measured both ways.")
        return 0

    if args.watch:
        print("thermal drift: %d measurements, %.0f s apart\n" % (args.watch, args.interval))
        rows = []
        _, exp_up = predicted(np.radians(args.head_pitch), np.radians(args.head_yaw))
        for i in range(args.watch):
            _, depth, intr, s = capture(warmup=8)
            try:
                n, c, _ = up_from_floor(depth, intr, s, expect_up=exp_up)
            except NotFloor as e:
                print("  NOT THE FLOOR  %s" % e)
                return 1
            t, r = readout(n)
            rows.append((time.time(), t, r, c))
            print("  %5.0f s   tilt %.2f deg   roll %+.2f deg   plane offset %.4f m"
                  % (rows[-1][0] - rows[0][0], t, r, c))
            if i + 1 < args.watch:
                time.sleep(args.interval)
        tilts = [r[1] for r in rows]
        offs = [r[3] for r in rows]
        print("\n  tilt drifted %.2f deg over %.0f s (%.2f .. %.2f)"
              % (max(tilts) - min(tilts), rows[-1][0] - rows[0][0], min(tilts), max(tilts)))
        print("  plane offset drifted %.1f mm (%.4f .. %.4f m)"
              % (1000 * (max(offs) - min(offs)), min(offs), max(offs)))
        print("\n  A D435 warms for 10-20 minutes and its depth creeps while it does. If the\n"
              "  offset drift is more than a few mm, let it warm up BEFORE calibrating or\n"
              "  measuring stack heights, or the whole session inherits the creep.")
        return 0

    if args.live:
        try:
            while True:
                _, depth, intr, s = capture(warmup=8)
                print("=" * 72)
                report_one(depth, intr, s, args)
        except KeyboardInterrupt:
            return 0

    if not args.frame:
        ap.error("give a frame, or --capture, --live, --imu, --pair or --watch")
    report_one(*load(args.frame)[1:], args=args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
