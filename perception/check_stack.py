"""Read-only: show what the stack detector sees, and why it accepted or rejected it.

    python3 check_stack.py --synthetic              # no camera at all: prove the install
    python3 check_stack.py --capture lab01          # save one frame, detect nothing
    python3 check_stack.py lab01.npz                # run on a saved frame
    python3 check_stack.py --live                   # keep running on a live camera
    python3 check_stack.py lab01.npz --png look.png # ...and write a diagnostic image
    python3 check_stack.py lab01.npz --plane p.json # use the segmentation side's plane

  on the robot computer, with the robot's joints:
    python3 check_stack.py --live --robot eth0 --json stack.json
    python3 check_stack.py --live --robot eth0 --head-down 30 --json stack.json
    python3 check_stack.py --live --robot eth0 --head-down 30 --head-left 20 --json stack.json
        --robot     read the head and waist angles at every frame, so the plate's
                    position is right wherever the head points
        --head-down tilt the head this far down first (over rt/arm_sdk, see r1_head.py),
                    hold it while detecting, give it back on exit
        --head-left, --head-right  and/or turn it this far to the ROBOT's left or right

  photos: with --json stack.json, every stack.json written comes with stack.jpg -- the
  frame behind those numbers -- and stack_view.jpg is the latest frame, accepted or not.
  Green ring: accepted, with its robot-frame position. Red ring: rejected, with why. A red
  bar marks a picture edge a stack runs off. The yellow cross is the picture's centre.

This changes nothing and commands nothing. It exists because "it found no stack" is not a
useful answer on its own: every candidate blob is listed with the reason it passed or failed,
so a bad frame can be told apart from a confidently wrong one.

NEEDS NO ROBOT. A RealSense, a table and a stack of plates is the whole rig.

THE PLANE
---------
Without --plane, the table is fitted here by RANSAC (plate_detectors.dominant_plane), so this
works before the segmentation side is wired up. With --plane, a JSON file supplies it:

    {"normal": [nx, ny, nz], "c": 0.41}        # n . X = c
    {"normal": [nx, ny, nz], "d": -0.41}       # nx X + ny Y + nz Z + d = 0
    {"normal": [nx, ny, nz], "point": [x,y,z]} # a plane through a known point

all in the CAMERA OPTICAL frame, metres. Give both and the two are compared, which is the
quickest way to find out whether the supplied plane agrees with the one in the picture.

THE ACCEPTANCE TEST
-------------------
Put a ruler against the stack. Run this. Remove ONE plate. Run it again. `top height` must
drop by one plate pitch and by nothing else. That single check exercises the whole chain and
needs no robot, no calibration and no nav stack.
"""
import argparse
import json
import os
import struct
import sys
import time
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import plate_detectors as pd
import plate_stack as ps


# --------------------------------------------------------------------------
# frames
# --------------------------------------------------------------------------

def capture_realsense(warmup=30):
    """One depth frame, aligned to colour, straight off a RealSense (any USB link)."""
    import realsense_capture
    return realsense_capture.capture(warmup)


def save_frame(name, color, depth, intr, scale):
    if not name.endswith(".npz"):
        name += ".npz"
    extra = {} if color is None else {"color": color}     # depth-only on a weak USB link
    np.savez_compressed(name, depth=depth, intr=json.dumps(intr), depth_scale=scale, **extra)
    return name


def load_frame(path):
    z = np.load(path, allow_pickle=False)
    return (z["color"] if "color" in z else None, z["depth"],
            json.loads(str(z["intr"])), float(z["depth_scale"]))


def read_plane(path):
    with open(path) as f:
        return json.load(f)


# --------------------------------------------------------------------------
# diagnostic image -- zlib only, so it works with no cv2 and no PIL
# --------------------------------------------------------------------------

def write_png(path, rgb):
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data
                + struct.pack(">I", zlib.crc32(typ + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def height_png(path, height, stacks, intr, top_band):
    """Height above the table as colour, with each accepted rim ringed in white.

    Grey = no depth. Blue = the table. Green through red = rising above it. The point of
    looking at this is to see whether the bright ring sits on the TOP plate's rim or has
    slipped down the stack.
    """
    h = np.where(np.isfinite(height), height, np.nan)
    img = np.zeros(h.shape + (3,), dtype=np.uint8)
    img[...] = 70                                        # grey: no data
    fin = np.isfinite(h)
    lo, hi = 0.0, max(0.05, float(np.nanpercentile(h[fin], 99.5)) if fin.any() else 0.05)
    t = np.clip((h - lo) / max(hi - lo, 1e-6), 0, 1)
    near = fin & (h <= 0.008)
    img[near] = np.array([40, 70, 150])                  # table
    up = fin & (h > 0.008)
    tt = t[up]
    img[up] = np.stack([(255 * np.clip(2 * tt, 0, 1)).astype(np.uint8),
                        (255 * np.clip(2 - 2 * tt, 0, 1)).astype(np.uint8),
                        np.full(tt.shape, 40, np.uint8)], axis=1)

    H, W = h.shape
    for s in stacks:
        e = s.get("ellipse")
        if not e:
            continue
        a = np.linspace(0, 2 * np.pi, 720)
        ct, st = np.cos(e["theta"]), np.sin(e["theta"])
        xs = e["cx"] + e["major"] * np.cos(a) * ct - e["minor"] * np.sin(a) * st
        ys = e["cy"] + e["major"] * np.cos(a) * st + e["minor"] * np.sin(a) * ct
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                xi = np.clip(np.round(xs + dx).astype(int), 0, W - 1)
                yi = np.clip(np.round(ys + dy).astype(int), 0, H - 1)
                img[yi, xi] = 255
    write_png(path, img)
    return path



# --------------------------------------------------------------------------
# photo: what the camera saw, and what was found in it
# --------------------------------------------------------------------------

def _ellipse_pts(e, n=360):
    a = np.linspace(0, 2 * np.pi, n)
    ct, st = np.cos(e["theta"]), np.sin(e["theta"])
    xs = e["cx"] + e["major"] * np.cos(a) * ct - e["minor"] * np.sin(a) * st
    ys = e["cy"] + e["major"] * np.cos(a) * st + e["minor"] * np.sin(a) * ct
    return np.stack([xs, ys], axis=1)


def photo(base_path, color, depth, cands, robot=None, angles=None, plate_d=0.254):
    """Save the camera's picture with each candidate stack drawn on it.

    Green ring = accepted (its robot-frame position written beside it); red ring = rejected,
    with the reason; a red bar along any picture edge a stack runs off; a small cross at the
    picture's centre, to aim by. `base_path` has no extension: .jpg with OpenCV, else .png
    (no text then -- the zlib writer has no font). Returns the path written."""
    import time as _time
    if color is not None:
        img = np.ascontiguousarray(color[..., :3]).copy()            # BGR
    else:                                                            # depth only (weak USB)
        d = depth.astype(np.float32)
        v = d[d > 0]
        hi = float(np.percentile(v, 98)) if v.size else 1.0
        g = np.clip(255 - 255 * d / max(hi, 1.0), 0, 255).astype(np.uint8)
        g[d == 0] = 40
        img = np.stack([g, g, g], axis=-1)
    H, W = img.shape[:2]
    GREEN, RED, YELLOW = (60, 200, 60), (40, 40, 230), (0, 220, 255)     # BGR
    try:
        import cv2
    except ImportError:
        cv2 = None

    # Plate-sized things get the full treatment; wires, chargers and walls a thin ring only.
    # A clipped plate reads small (its outline is a chord), hence the generous lower bound.
    def platey(c):
        return (0.6 * plate_d <= c["diameter_m"] <= 1.4 * plate_d
                and c["axis_ratio"] >= ps.MIN_AXIS_RATIO)

    # edges a plate-sized stack runs off
    bar = max(4, W // 120)
    for c in cands:
        if not platey(c):
            continue
        for e in (c.get("clipped_edges") or []):
            sl = {"top": np.s_[:bar, :], "bottom": np.s_[-bar:, :],
                  "left": np.s_[:, :bar], "right": np.s_[:, -bar:]}.get(e)
            if sl is not None:
                img[sl] = RED
    # centre cross
    cx, cy, k = W // 2, H // 2, max(8, W // 60)
    img[cy - 1:cy + 2, cx - k:cx + k] = YELLOW
    img[cy - k:cy + k, cx - 1:cx + 2] = YELLOW

    rows = {id(r_s): r for r_s, r in (robot or [])}
    for i, c in enumerate(cands):
        e = c.get("ellipse")
        if not e:
            continue
        ok = c.get("_verdict") == "PLATE STACK"
        main = ok or platey(c)
        col = GREEN if ok else RED
        pts = _ellipse_pts(e)
        if cv2 is not None:
            cv2.polylines(img, [np.round(pts).astype(np.int32)], True, col, 3 if main else 1,
                          cv2.LINE_AA)
            if not main:
                continue
        else:
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    xi = np.clip(np.round(pts[:, 0] + dx).astype(int), 0, W - 1)
                    yi = np.clip(np.round(pts[:, 1] + dy).astype(int), 0, H - 1)
                    img[yi, xi] = col
        if cv2 is not None:
            n = c.get("n_plates")
            lines = ["#%d %s" % (i, c.get("_verdict", "")),
                     "top %.1f mm, %s plate%s" % (1000 * c["top_height_m"], n or "?",
                                                  "" if n == 1 else "s")]
            r = rows.get(id(c))
            if r is not None:
                x, y, z = r["center_m"]
                lines.append("robot x %+.3f  y %+.3f  z %+.3f m" % (x, y, z))
            # on a dark translucent box, so the text reads over any background
            fs, pad = 0.55, 6
            sz = [cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)[0] for t in lines]
            bw, lh = max(s[0] for s in sz) + 2 * pad, max(s[1] for s in sz) + 8
            bh = lh * len(lines) + pad
            tx = int(np.clip(e["cx"] - bw / 2, 4, W - bw - 4))
            ty = int(np.clip(e["cy"] - e["major"] - bh - 8, 4, H - bh - 4))
            over = img.copy()
            cv2.rectangle(over, (tx, ty), (tx + bw, ty + bh), (20, 20, 20), -1)
            img[:] = cv2.addWeighted(over, 0.6, img, 0.4, 0)
            cv2.rectangle(img, (tx, ty), (tx + bw, ty + bh), col, 1)
            for j, t in enumerate(lines):
                cv2.putText(img, t, (tx + pad, ty + pad + lh * (j + 1) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, fs, col if j == 0 else (255, 255, 255),
                            1, cv2.LINE_AA)
    if cv2 is not None:
        head = _time.strftime("%H:%M:%S")
        if angles:
            head += "   head pitch %+.1f  yaw %+.1f deg" % (
                np.degrees(angles.get("head_pitch_joint", 0.0)),
                np.degrees(angles.get("head_yaw_joint", 0.0)))
        if not cands:
            head += "   no stack found"
        cv2.putText(img, head, (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, head, (8, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        path = base_path + ".jpg"
        cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 90])
    else:
        path = base_path + ".png"
        write_png(path, img[..., ::-1])
    return path


def photo_bases(args):
    """(every-frame view, the photo that goes with stack.json): paths without extension."""
    if args.photo:
        view = os.path.splitext(args.photo)[0]
    elif args.json:
        view = os.path.splitext(args.json)[0] + "_view"
    else:
        view = None
    paired = os.path.splitext(args.json)[0] if args.json else None
    return view, paired


# --------------------------------------------------------------------------
# the report
# --------------------------------------------------------------------------

def report(depth, intr, scale, plane, args, color=None, angles=None, body=None):
    print("frame     %d x %d   depth_scale %.0f (units per metre)"
          % (depth.shape[1], depth.shape[0], scale))
    valid = (depth > 0)
    print("depth     %.1f%% of pixels have a reading" % (100.0 * valid.mean()))

    fitted = None
    try:
        fitted = pd.dominant_plane(depth, intr, scale)
    except RuntimeError as e:
        print("RANSAC    could not fit a table here: %s" % e)

    if plane is None:
        if fitted is None:
            print("\nNo plane to work from. Supply one with --plane.")
            return 2
        plane = fitted
        print("plane     fitted here by RANSAC  n=%s c=%.4f"
              % (np.array2string(fitted[0], precision=4), fitted[1]))
    else:
        n, c = ps._as_plane(plane)
        print("plane     supplied             n=%s c=%.4f"
              % (np.array2string(n, precision=4), c))
        if fitted is not None:
            ang = np.degrees(np.arccos(np.clip(abs(float(n @ fitted[0])), -1, 1)))
            off = abs(abs(c) - abs(fitted[1]))
            agree = "agrees" if (ang < 5 and off < 0.03) else "DISAGREES"
            print("          vs the RANSAC fit: %s  (%.1f deg apart, %.0f mm offset)"
                  % (agree, ang, 1000 * off))
            if agree == "DISAGREES":
                print("          One of the two is wrong. Everything below inherits it.")
        plane = (n, c)

    try:
        H, _ = ps.heights(depth, intr, plane, scale)
    except ps.NoStackFound as e:
        print("\nREFUSED: %s" % e)
        if photo_bases(args)[0]:
            photo(photo_bases(args)[0], color, depth, [], None, angles)
        return 2
    fin = np.isfinite(H)
    band = fin & (H > args.min_height) & (H < args.max_height)
    print("standing  %d px between %.0f and %.0f mm above the table"
          % (band.sum(), 1000 * args.min_height, 1000 * args.max_height))

    try:
        # allow_clipped and require_plate_size are both off here on purpose: this tool
        # exists to show what was rejected and why, so it asks for everything and applies
        # the verdicts itself below.
        cands = ps.locate_stacks(depth, intr, plane, depth_scale=scale,
                                 plate_diameter_m=args.diameter,
                                 plate_pitch_m=args.pitch,
                                 plate_rim_height_m=args.rim_height,
                                 min_height_m=args.min_height,
                                 max_height_m=args.max_height,
                                 require_plate_size=False,
                                 allow_clipped=True)
    except ps.NoStackFound as e:
        print("\nREFUSED: %s" % e)
        if photo_bases(args)[0]:
            photo(photo_bases(args)[0], color, depth, [], None, angles)
        if args.png:
            print("wrote %s" % height_png(args.png, H, [], intr, args.top_band))
        return 2

    print("\n%-3s %-11s %-11s %-9s %-8s %-7s %-6s %s"
          % ("#", "top height", "diameter", "size err", "rim px", "ratio", "count", "verdict"))
    accepted = []
    for i, s in enumerate(cands):
        bad = []
        if s["clipped"]:
            bad.append("CLIPPED at the %s edge" % "+".join(s.get("clipped_edges") or ["frame"]))
        elif s["size_error"] > ps.DIAMETER_TOL:
            bad.append("wrong size")
        elif s["axis_ratio"] < ps.MIN_AXIS_RATIO:
            bad.append("too flat to be a rim")
        if s["clipped"] and args.allow_clipped:
            bad = []
        verdict = "PLATE STACK" if not bad else ("rejected: " + ", ".join(bad))
        s["_verdict"] = verdict
        if not bad:
            accepted.append(s)
        print("%-3d %7.1f mm  %7.1f mm  %7.1f%%  %7d  %6.2f  %-6s %s"
              % (i, 1000 * s["top_height_m"], 1000 * s["diameter_m"],
                 100 * s["size_error"], s["rim_px"], s["axis_ratio"],
                 ("%d" % s["n_plates"]) if s["n_plates"] else "-", verdict))

    if not accepted:
        if cands and all(c["clipped"] for c in cands):
            # the camera faces forward: the picture's top is far, its left is the robot's left
            way = {"top": "closer to the robot", "bottom": "further from the robot",
                   "left": "toward the robot's right", "right": "toward the robot's left"}
            moves = [way[e] for e in (cands[0].get("clipped_edges") or []) if e in way]
            print("\nThe stack runs off the %s edge of the picture, so its size cannot be\n"
                  "measured and it cannot be confirmed as a plate.%s The heights above\n"
                  "are probably already right -- pass --allow-clipped to use them."
                  % ("+".join(cands[0].get("clipped_edges") or ["frame"]),
                     (" Move it " + " and ".join(moves) + ", ~10 cm at a time.") if moves else ""))
        else:
            print("\nNothing here is plate-shaped. The rows above say why; a size error near\n"
                  "100% usually means --diameter is wrong for these plates.")
    for i, s in enumerate(accepted):
        print("\nstack %d" % i)
        print("  top plate rim   %.1f mm above the table" % (1000 * s["top_height_m"]))
        print("  centre          %s m  (camera optical frame)"
              % np.array2string(s["center_m"], precision=4))
        print("  diameter        %.1f mm   rim fit rms %.1f mm"
              % (1000 * s["diameter_m"], 1000 * s["rim_rms_m"]))
        if s["ledges_m"]:
            print("  ledges          %s mm"
                  % ", ".join("%.0f" % (1000 * x) for x in s["ledges_m"][:14]))
        if s["clipped"]:
            print("  CLIPPED         the rim runs off the frame, so diameter and centre are\n"
                  "                  a chord's, not a plate's. The height above is still good.")
        if s["n_plates"]:
            print("  plates          %d, counted from the %s"
                  % (s["n_plates"], s["count_from"]))
        elif args.rim_height is None:
            print("  plates          not counted. Give --pitch AND --rim-height (a single\n"
                  "                  plate's rim height) and it follows from the height.")
        if s["pitch_m"]:
            print("  ledge pitch     %.1f mm from %d ledges"
                  % (1000 * s["pitch_m"], len(s["ledges_m"])))
        else:
            print("  pitch           not measurable from this view (the stack's side is\n"
                  "                  barely visible). Height is still good; the count is not.")

    if args.png:
        print("\nwrote %s" % height_png(args.png, H, accepted, intr, args.top_band))
        print("  grey = no depth, blue = table, green->red = rising, white ring = the rim\n"
              "  it measured. Check the ring is on the TOP plate, not partway down.")

    robot = robot_frame_report(accepted, plane, level=args.level, angles=angles, body=body)
    view, paired = photo_bases(args)
    pairs = list(zip(accepted, robot)) if robot else []
    if args.json and accepted:
        with open(args.json, "w") as f:
            json.dump(robot, f, indent=2)
        print("\nwrote %s  (hand this to the manipulation side)" % args.json)
        print("wrote %s  (what the camera saw for it)"
              % photo(paired, color, depth, cands, pairs, angles, args.diameter))
    if view:
        photo(view, color, depth, cands, pairs, angles, args.diameter)

    if accepted and not robot:
        print("\nNow remove ONE plate and run this again. 'top plate rim' should fall by one\n"
              "pitch (%s) and by nothing else. That is the test that matters."
              % (("%.0f mm" % (1000 * args.pitch)) if args.pitch else "one plate"))
    return 0 if accepted else 1


def robot_frame_report(accepted, plane, level="floor", angles=None, body=None):
    """Print each accepted stack in the robot frame, if a camera calibration is saved.

    body: the robot's state at this frame ({"joints", "imu_rpy"}, R1Link.state()). With it
    the robot's lean is known (floor_frame.py): the table check compares against the real
    level, and the tape numbers (forward of the ankles, height off the floor) are right
    even though the pelvis is not level. Without it the pelvis is taken as level.

    level="floor": tilt and roll from the saved floor calibration.
    level="table": tilt and roll from the table in THIS frame, treated as level. Always
    current -- no stale calibration if the robot has moved -- but only as good as the
    table's levelness. Heights above the floor still come from the saved calibration.

    angles: the waist and head joint angles (rad) at this frame, from the robot. Without
    them the robot is taken to be in the pose it was calibrated in.

    Returns a JSON-ready list, or [] when there is no calibration.
    """
    import camera_calibration as cc
    import floor_frame as ff
    import r1_config as rcfg
    import urdf_fk as fk

    cal = cc.load()
    if not accepted:
        return []
    up_p, lean = ff.pelvis_up(body) if body else (np.array([0.0, 0.0, 1.0]), None)
    if level == "table":
        n_t = np.asarray(plane[0], float)
        n_t = -n_t if n_t[2] > 0 else n_t
        saved = cal
        cal = cc.from_level_surface(n_t, surface="table", angles=angles,
                                    up_pelvis=up_p if body else None, body=body,
                                    source="the table in this frame, taken as level")
        if saved and saved.get("pelvis_above_floor_m") is not None:
            cal["pelvis_above_floor_m"] = saved["pelvis_above_floor_m"]
            cal["measured_tilt_deg_floor"] = saved["measured_tilt_deg"]
    if cal is None:
        print("\nrobot frame     not available: no camera calibration saved yet.\n"
              "                Face the robot at open floor and run:\n"
              "                  python check_mount.py --calibrate\n"
              "                or level from the table in this frame:  --level table")
        return []

    # Is the table in THIS frame tipped the way the calibration predicts for the head's
    # angle now? If not, the robot has moved or is leaning differently: the calibration is stale.
    if angles is None:
        angles = {k: v for k, v in cal.get("angles_at_calibration", {}).items()}
    # Compared in the ROBOT's frame, as how far the table looks tipped front-back and
    # left-right. Not as camera tilt and roll: with the head down, the camera's "roll"
    # exaggerates a sideways tip (1 deg reads as 2.3 deg with the camera 64 deg down).
    n = np.asarray(plane[0], float)
    n = -n if n[2] > 0 else n
    T_pred = cc.T_pelvis_optical(cal, fk.load_urdf(rcfg.URDF_PATH), angles)
    u = T_pred[:3, :3] @ (n / np.linalg.norm(n))       # the table's up, in pelvis_link
    # against the real level (the IMU's, when the robot is read): a leaning robot sees a
    # level table tipped by its lean, and that is right, not an error
    d_fwd = float(np.degrees(np.arctan2(u[0], u[2]) - np.arctan2(up_p[0], up_p[2])))
    d_side = float(np.degrees(np.arctan2(u[1], u[2]) - np.arctan2(up_p[1], up_p[2])))
    floor = None
    if body is not None:
        floor, _ = ff.FloorFrame.from_state(body, cal)
    elif cal.get("pelvis_above_floor_m") is not None:
        floor = ff.FloorFrame([0.0, 0.0, 1.0], cal["pelvis_above_floor_m"])

    out = []
    print("\nROBOT FRAME  (pelvis_link: x forward, y left, z up -- origin directly above the\n"
          "              floor point midway between the ankles)")
    for i, s in enumerate(accepted):
        r = ps.to_robot_frame(s, calibration=cal, angles=angles)
        r["joint_angles_deg"] = {k: round(float(np.degrees(v)), 2) for k, v in angles.items()}
        r["captured_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        c = r["center_m"]
        print("  stack %d  top plate centre  x %+.3f m   y %+.3f m   z %+.3f m"
              % (i, c[0], c[1], c[2]))
        if floor is not None:                # what a tape measures, lean accounted for
            t = floor.to_tape(c)
            r["tape_m"] = t.tolist()
            r["top_above_floor_m"] = float(t[2])
            r["table_above_floor_m"] = float(t[2] - r["top_height_m"])
            print("           tape: %.1f cm in front of the ankle midpoint, %.1f cm to the %s,"
                  "\n                 top plate %.1f cm off the floor (table %.1f cm)"
                  % (100 * t[0], 100 * abs(t[1]), "left" if t[1] >= 0 else "right",
                     100 * t[2], 100 * r["table_above_floor_m"]))
        else:
            print("           that is %.0f cm in front, %.0f cm to the %s"
                  % (100 * c[0], 100 * abs(c[1]), "left" if c[1] >= 0 else "right"))
        if body is not None:
            r["imu_rpy"] = body.get("imu_rpy")
            r["pelvis_up"] = up_p.tolist()
        out.append({k: (v.tolist() if isinstance(v, np.ndarray) else v)
                    for k, v in r.items()})
    print("  calibration  %s (%s)" % (cal.get("method", "?"), cal.get("date", "?")))
    if lean is not None:
        print("  " + ff.describe(lean).replace("\n", "\n  "))
    elif cal.get("version") == 2 or cal.get("pelvis_up"):
        print("  pelvis       taken as LEVEL (no robot state): run with --robot for the lean")
    if angles:
        print("  joints       head pitch %+.1f deg, yaw %+.1f; waist roll %+.1f, yaw %+.1f"
              % tuple(np.degrees(angles.get(k, 0.0)) for k in
                      ("head_pitch_joint", "head_yaw_joint", "waist_roll_joint",
                       "waist_yaw_joint")))
    if level != "table":
        worst = max(abs(d_fwd), abs(d_side))
        # what that does to a plate's position: the camera's height over it times the angle
        h = float(T_pred[2, 3] - np.mean([o["center_m"][2] for o in out])) if out else 0.4
        cm = 100 * h * np.tan(np.radians(worst))
        flag = "" if worst < 2.0 else (
            "\n                 <-- a level table should read under 2 deg. The robot leans"
            "\n                 differently from when it was calibrated, or the table is not"
            "\n                 level: positions may be off by ~%.0f cm. Re-run"
            "\n                 check_mount.py --calibrate --robot, or use --level table." % cm)
        print("  table, as the robot sees it: tipped %+.1f deg front-back, %+.1f deg "
              "left-right (~%.1f cm at the plate)%s" % (d_fwd, d_side, cm, flag))
    print("  sideways (y) is only as good as the camera's yaw, which a level surface cannot\n"
          "  measure: put the stack on the robot's centreline once and check y is near 0.")
    return out


class HeadProcess:
    """r1_head.py holding the head down, in a process of its own.

    Not a thread here: Python runs one thread at a time, and fitting a depth frame starves
    a head loop in the same process for long enough to trip its safety check. Its output is
    shown prefixed [head]; Enter on its stdin hands the head back (as when run by hand)."""

    def __init__(self, interface, down_deg=None, right_deg=None, timeout=30.0):
        import subprocess
        import threading
        cmd = [sys.executable, "-u", os.path.join(HERE, "r1_head.py"), interface, "--go"]
        if down_deg is not None:
            cmd += ["--down", str(down_deg)]
        if right_deg is not None:            # r1_head's yaw is + to the LEFT
            cmd += ["--yaw", str(-right_deg)]
        self.p = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self.there = threading.Event()

        def pump():
            for line in self.p.stdout:
                print("[head] " + line.rstrip(), flush=True)
                if line.startswith("there:"):
                    self.there.set()
        threading.Thread(target=pump, daemon=True).start()
        t0 = time.time()
        while not self.there.wait(0.2):
            if self.p.poll() is not None:
                raise SystemExit("the head did not tilt (r1_head.py exited, see [head] above)")
            if time.time() - t0 > timeout:
                self.release()
                raise SystemExit("the head did not get there in %.0f s" % timeout)

    def alive(self):
        return self.p.poll() is None

    def release(self):
        if self.p.poll() is None:
            try:
                self.p.stdin.write("\n")
                self.p.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                pass
        try:
            self.p.wait(timeout=20)
        except Exception:
            self.p.terminate()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", nargs="?", help="a .npz saved by --capture")
    ap.add_argument("--capture", metavar="NAME", help="save one frame and stop")
    ap.add_argument("--live", action="store_true", help="loop on a live camera")
    ap.add_argument("--synthetic", action="store_true",
                    help="a ray-marched stack, no camera needed")
    ap.add_argument("--synthetic-plates", type=int, default=8)
    ap.add_argument("--plane", metavar="FILE.json",
                    help="use this table plane instead of fitting one")
    # Defaults are the lab plates, measured from the first on-robot frames (Oct 2026):
    # 254 mm (10 in) across -- measured 254.7 mm by this tool, 254 mm by ruler -- nesting
    # at ~9 mm, a single plate's rim 15.6 mm above the table.
    # Different crockery needs these re-measured.
    ap.add_argument("--diameter", type=float, default=0.254, help="plate diameter, m")
    ap.add_argument("--pitch", type=float, default=0.009,
                    help="nesting pitch, m -- measure it, it sets the whole tolerance")
    ap.add_argument("--rim-height", type=float, default=0.0156,
                    help="a SINGLE plate's rim height above the table, m. With --pitch "
                         "this counts the stack from its height, which beats counting "
                         "ledges by a wide margin.")
    ap.add_argument("--allow-clipped", action="store_true",
                    help="accept a stack that runs off the frame edge. Its height is "
                         "still good; its diameter and centre are not.")
    ap.add_argument("--min-height", type=float, default=ps.MIN_HEIGHT_M)
    ap.add_argument("--max-height", type=float, default=ps.MAX_HEIGHT_M)
    ap.add_argument("--top-band", type=float, default=ps.TOP_BAND_M)
    ap.add_argument("--png", metavar="OUT.png", help="write a diagnostic image")
    ap.add_argument("--level", choices=("floor", "table"), default="floor",
                    help="what to take as level for the robot frame: the saved floor "
                         "calibration (default), or the table in this frame")
    ap.add_argument("--robot", metavar="IFACE",
                    help="read the head/waist angles from the robot over this interface (eth0)")
    ap.add_argument("--head-down", type=float, metavar="DEG",
                    help="with --robot: tilt the head this far down first, hand it back on exit")
    ap.add_argument("--head-right", type=float, metavar="DEG",
                    help="with --robot: also turn the head this far to the ROBOT's right "
                         "(as seen standing behind it), to follow a stack moved sideways")
    ap.add_argument("--head-left", type=float, metavar="DEG",
                    help="with --robot: the same, to the robot's left")
    ap.add_argument("--photo", metavar="VIEW.jpg",
                    help="save the latest annotated camera view here every frame "
                         "(default with --json: <json>_view.jpg; the frame behind each "
                         "stack.json is always saved as <json>.jpg)")
    ap.add_argument("--json", metavar="OUT.json",
                    help="write the stacks' robot-frame positions (needs a calibration)")
    args = ap.parse_args(argv)
    np.seterr(invalid="ignore")          # NaN depth compares as False, which is what is meant
    if args.head_left is not None:
        if args.head_right is not None:
            ap.error("give --head-left or --head-right, not both")
        args.head_right = -args.head_left

    plane = read_plane(args.plane) if args.plane else None

    if args.capture:
        color, depth, intr, scale = capture_realsense()
        print("saved %s" % save_frame(args.capture, color, depth, intr, scale))
        return 0

    if args.synthetic:
        from stack_scene import render, INTR
        depth, truth = render(n_plates=args.synthetic_plates)
        print("synthetic: %d plates, top rim truly %.1f mm above the table\n"
              % (args.synthetic_plates, 1000 * truth["stacks"][0]["top_height_m"]))
        if plane is None:
            plane = truth["plane"]
            print("plane     taken from the synthetic scene (exact)")
        return report(depth, INTR, 1000.0, plane, args)

    if args.live:
        link = head = None
        if (args.head_down is not None or args.head_right is not None) and not args.robot:
            ap.error("--head-down and --head-right need --robot")
        try:
            if args.robot:
                import r1_head
                link = r1_head.R1Link(args.robot).wait()
                if args.head_down is not None or args.head_right is not None:
                    head = HeadProcess(args.robot, args.head_down, args.head_right)
            while True:
                angles = None
                if head is not None and not head.alive():
                    print("the head process has stopped -- stopping too")
                    return 1
                body = None
                try:
                    if link is not None:
                        before = link.angles()
                    color, depth, intr, scale = capture_realsense(warmup=8)
                    if link is not None:
                        after = link.angles()
                        moved = max(abs(after[k] - before[k]) for k in before)
                        angles = after
                        body = link.state()
                except RuntimeError as e:
                    print("=" * 72)
                    print("skipped: %s" % e)
                    continue
                print("=" * 72)
                if link is not None and moved > np.radians(0.5):
                    print("the head or waist moved %.1f deg during the capture: skipped"
                          % np.degrees(moved))
                    continue
                report(depth, intr, scale, plane, args, color, angles=angles, body=body)
                print()
        except KeyboardInterrupt:
            return 0
        finally:
            if head is not None:
                head.release()

    if not args.frame:
        ap.error("give a frame, or --capture, --live or --synthetic")
    color, depth, intr, scale = load_frame(args.frame)
    angles = body = None
    if args.robot:                       # a saved frame, the robot's joints NOW: only if unmoved
        import r1_head
        link = r1_head.R1Link(args.robot).wait()
        angles, body = link.angles(), link.state()
    return report(depth, intr, scale, plane, args, color, angles=angles, body=body)


if __name__ == "__main__":
    sys.exit(main())
