"""
Find banquet tables with a depth camera, in the lab, before any ceiling camera exists.

    python3 table_segmentation.py --live                  # capture from a RealSense and detect
    python3 table_segmentation.py --capture lab01         # just save a frame
    python3 table_segmentation.py lab01.npz               # detect on a saved frame
    python3 table_segmentation.py --synthetic             # no camera at all

Writes `<name>_detections.json` in the planner's own format, so the result goes straight
into Camera -> Import detections -> Review, and `<name>_overlay.png` if cv2 is installed.

WHY THIS EXISTS, AND WHAT IT IS NOT
    The ceiling camera in the hall will be RGB only. This is a DEPTH detector, so it is
    not the thing that ships. It is here for two jobs that matter more right now:

      1. It exercises the whole path end to end -- detect, JSON, corner calibration,
         review, place -- with real hardware, today, in a room you can actually get into.
      2. It hands you GROUND TRUTH for free. A depth camera measures where the tables
         really are, to a centimetre or so, with no hand-labelling. Point it at the same
         scene as the RGB detector you are evaluating and you can score that detector
         automatically, on every frame, instead of squinting at overlays.

    So: the RealSense is not the test subject. It is the ruler you test the real detector
    against.

HOW IT WORKS
    Exactly like the plate detector next door, one storey up. There, a plate stands on a
    table; here, a table stands on the floor. The geometry is identical and the code is
    the same code -- fit the dominant plane, take everything standing on it at the right
    height, fill the holes, label the blobs, and fit each one as a circle IN ITS OWN
    PLANE rather than as a circle in the image.

    That last step matters for the same reason it mattered for the plate: a round table
    seen from anywhere but straight overhead projects as an ELLIPSE, and a circle fitted
    to an ellipse is wrong in both size and centre. Backprojecting the outline onto the
    tabletop's own plane removes both errors. A ceiling camera will have the same problem
    and the planner's four-corner homography is what solves it there.
"""

import argparse
import datetime
import json
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# The geometry is the plate detector's, unchanged. Importing it rather than
# copying it is deliberate: fit_ellipse, height_above_plane and circle_on_plane
# each took a subtle bug to get right (a singular scatter matrix, a sign taken
# from the median of a frame that is mostly plane, a centre that is not the
# projection of the centre). A second copy would drift away from the tested one.
# ---------------------------------------------------------------------------
def _load_detectors():
    here = os.path.dirname(os.path.abspath(__file__))
    for cand in (here,
                 os.path.join(here, "..", "G1_Plate_Perception"),
                 os.path.join(here, "..", "..", "G1_Plate_Perception"),
                 os.path.join(here, "..", "..", "..", "G1_Plate_Perception"),
                 os.environ.get("PLATE_PERCEPTION_DIR", "")):
        if cand and os.path.exists(os.path.join(cand, "plate_detectors.py")):
            sys.path.insert(0, os.path.abspath(cand))
            import plate_detectors
            return plate_detectors
    raise SystemExit(
        "cannot find plate_detectors.py.\n"
        "Copy it next to this script, or set PLATE_PERCEPTION_DIR to the folder\n"
        "holding it (on the robot that is ~/.scuba/plate_perception).")


pd = _load_detectors()

# ---------------------------------------------------------------------------
# what a banquet table looks like to a depth camera
# ---------------------------------------------------------------------------
TABLE_HEIGHT_M = 0.762   # 2.5 ft, the standard banquet table height
BAND_M = 0.09            # how far off that a tabletop may read and still count
MIN_D_M = 1.40           # a 5 ft round is 1.52 m; allow for a squashed fit
MAX_D_M = 2.30           # a 7 ft round is 2.13 m
MIN_AXIS_RATIO = 0.40    # flatter than this is a chair back or a wall, not a top
MIN_COMPONENT_PX = 400   # at stride 2 this is a blob you can actually fit
STOCK_M = {"5 ft round": 1.524, "5.5 ft round": 1.6764,
           "6 ft round": 1.8288, "7 ft round": 2.1336}


def plane_from_pixels(depth, intrinsics, mask, depth_scale=1000.0):
    """Least-squares plane through the 3D points of a mask. (n, c) with n . X = c."""
    d = np.asarray(depth).astype(np.float64) / depth_scale
    ys, xs = np.nonzero(np.asarray(mask, dtype=bool) & (d > 0))
    if len(ys) < 50:
        return None
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    z = d[ys, xs]
    P = np.stack([(xs - cx) * z / fx, (ys - cy) * z / fy, z], axis=1)
    centroid = P.mean(axis=0)
    n = np.linalg.svd(P - centroid, full_matrices=False)[2][-1]
    return n, float(n @ centroid)


def floor_plane(depth, intrinsics, depth_scale=1000.0, stride=4, seed=0):
    """The floor, which is not reliably the biggest flat thing in view.

    In a hall it is. In a lab it is not: put a camera two metres above one 6 ft round and
    the TABLE fills the frame, so dominant_plane() returns the tabletop, every table then
    measures 0.00 m above "the floor", and the detector finds nothing at all. This is the
    single most likely way a first lab test fails.

    So fit the dominant plane, then look underneath it. If a large slab of the frame sits
    well below, that slab is the floor and the first answer was furniture.

    Returns (n, c, corrected).
    """
    n, c = pd.dominant_plane(depth, intrinsics, depth_scale, stride, seed=seed)
    h = pd.height_above_plane(depth, intrinsics, (n, c), depth_scale)
    ok = np.isfinite(h)
    if ok.sum() < 200:
        return n, c, False
    below = ok & (h < -0.25)
    if below.sum() > 0.15 * ok.sum():
        masked = np.where(below, np.asarray(depth), 0)
        try:
            n2, c2 = pd.dominant_plane(masked, intrinsics, depth_scale, stride, seed=seed)
            return n2, c2, True
        except RuntimeError:
            pass
    return n, c, False


def nearest_stock(diameter_m):
    return min(STOCK_M.items(), key=lambda kv: abs(kv[1] - diameter_m))


def detect_tables(depth, intrinsics, depth_scale=1000.0, stride=2,
                  table_height_m=TABLE_HEIGHT_M, band_m=BAND_M, verbose=False):
    """Every banquet table standing on the floor, from one depth frame."""
    n, c, corrected = floor_plane(depth, intrinsics, depth_scale)
    hgt = pd.height_above_plane(depth, intrinsics, (n, c), depth_scale)

    band = np.isfinite(hgt) & (np.abs(hgt - table_height_m) <= band_m)
    # Centrepieces, glasses and cutlery sit ABOVE the band and punch holes in the top.
    # Same fix as the glaze dropout on the plate: a hole is background that cannot
    # reach the border.
    sub = pd.fill_holes(band[::stride, ::stride])
    labels, count = pd.label_mask(sub)

    found, rejected = [], []
    for k in range(1, count + 1):
        comp = labels == k
        px = int(comp.sum())
        if px < MIN_COMPONENT_PX:
            rejected.append(("too small", px * stride * stride, None))
            continue
        edge = pd.outline(comp) * stride
        ell = pd.fit_ellipse(edge)
        if ell is None:
            rejected.append(("no ellipse fit", px * stride * stride, None))
            continue
        ratio = ell["minor"] / max(ell["major"], 1e-9)
        if ratio < MIN_AXIS_RATIO:
            rejected.append(("too flat", px * stride * stride, ratio))
            continue

        full = np.zeros(np.asarray(depth).shape, dtype=bool)
        full[::stride, ::stride] = comp
        top = plane_from_pixels(depth, intrinsics, full, depth_scale)
        if top is None:
            rejected.append(("no tabletop plane", px * stride * stride, ratio))
            continue

        fit = pd.circle_on_plane(edge, intrinsics, top)
        if fit is None:
            rejected.append(("no circle on the tabletop", px * stride * stride, ratio))
            continue
        centre_cam, radius, rms = fit
        diameter = 2.0 * radius
        if not (MIN_D_M <= diameter <= MAX_D_M):
            rejected.append(("%.2f m across" % diameter, px * stride * stride, ratio))
            continue

        # How round it actually came out: the share of the outline that lands on the
        # fitted circle. An honest stand-in for confidence, and the thing that separates
        # a table from a trolley the band happened to catch.
        on = _roundness(edge, intrinsics, top, centre_cam, radius)
        label, nominal = nearest_stock(diameter)
        found.append({
            "centre_px": [float(ell["cx"]), float(ell["cy"])],
            "diameter_px": float(2.0 * ell["major"]),
            "outline_px": _decimate(edge, ell),
            "axis_ratio": float(ratio),
            "confidence": round(float(on), 2),
            "measured": {
                "centre_camera_m": [round(float(v), 4) for v in centre_cam],
                "diameter_m": round(float(diameter), 4),
                "height_above_floor_m": round(float(np.median(hgt[full])), 4),
                "circle_rms_mm": round(float(rms * 1000.0), 1),
                "nearest_stock": label,
                "stock_error_mm": round(float((diameter - nominal) * 1000.0), 1),
            },
        })

    found.sort(key=lambda t: -t["confidence"])
    for i, t in enumerate(found):
        t["id"] = "t%d" % (i + 1)
        t["shape"] = "round"
    if verbose:
        print("  floor plane %s" % ("re-fitted: the dominant plane was a tabletop"
                                    if corrected else "= the dominant plane"))
        print("  %d blobs in the %.2f-%.2f m band, %d kept"
              % (count, table_height_m - band_m, table_height_m + band_m, len(found)))
        for why, area, ratio in rejected[:8]:
            print("    dropped: %-26s %6d px%s"
                  % (why, area, "" if ratio is None else "  ratio %.2f" % ratio))
    return found, (n, c), hgt


def _decimate(edge_px, ell, n=48):
    """The rim as `n` points, evenly spaced by angle. Sent so the PLANNER can fit the
    circle in the room's own plane.

    Handing over a centre and a diameter is not enough. A round table seen from anywhere
    but straight overhead projects as an ellipse, and the centre of that ellipse is not
    the projection of the table's centre -- the same bias that cost the plate detector
    9-14 mm, except a banquet table is eight times bigger, so it costs about 10 cm. That
    is the planner's whole stated accuracy. With the rim, the planner can map the points
    onto the floor, where a table is a circle again, and fit it there.
    """
    e = np.asarray(edge_px, dtype=float)
    ang = np.arctan2(e[:, 1] - ell["cy"], e[:, 0] - ell["cx"])
    out, taken = [], set()
    for k in range(n):
        want = -np.pi + 2 * np.pi * k / n
        i = int(np.argmin(np.abs(np.angle(np.exp(1j * (ang - want))))))
        if i not in taken:
            taken.add(i)
            out.append([round(float(e[i, 0]), 1), round(float(e[i, 1]), 1)])
    return out


def _roundness(edge_px, intrinsics, plane, centre_cam, radius, tol_m=0.03):
    """Share of the outline lying within tol of the fitted circle, on the tabletop."""
    n, c = plane
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    e = np.asarray(edge_px, dtype=float)
    dirs = np.stack([(e[:, 0] - cx) / fx, (e[:, 1] - cy) / fy, np.ones(len(e))], axis=1)
    den = dirs @ n
    keep = np.abs(den) > 1e-6
    if keep.sum() < 6:
        return 0.0
    P = dirs[keep] * (c / den[keep])[:, None]
    r = np.linalg.norm(P - np.asarray(centre_cam), axis=1)
    return float(np.mean(np.abs(r - radius) < tol_m))


# ---------------------------------------------------------------------------
# the planner's format
# ---------------------------------------------------------------------------
def to_document(tables, width, height, source="realsense-d435-lab"):
    """The shape the planner's Import detections reads.

    Pixels, not metres, on purpose: that is the path the ceiling camera will use, so the
    lab test exercises the homography too. The metres this camera actually measured ride
    along under `measured` -- the planner ignores fields it does not know, and they are
    what you check the homography against.
    """
    return {
        "schema": "banquet-detections/0.1",
        "source": source,
        "captured_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "units": "pixels",
        "frame": {"width": int(width), "height": int(height)},
        "tables": tables,
    }


def draw_overlay(color, tables, path):
    try:
        import cv2
    except ImportError:
        return None
    vis = color.copy() if color is not None else None
    if vis is None:
        return None
    for t in tables:
        u, v = int(t["centre_px"][0]), int(t["centre_px"][1])
        r = int(t["diameter_px"] / 2)
        cv2.circle(vis, (u, v), r, (0, 190, 255), 2)
        cv2.drawMarker(vis, (u, v), (0, 190, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(vis, "%s %.2fm %.0f%%" % (t["id"], t["measured"]["diameter_m"],
                                              100 * t["confidence"]),
                    (u - r, v - r - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 190, 255), 1)
    cv2.imwrite(path, vis)
    return path


# ---------------------------------------------------------------------------
# frames in and out
# ---------------------------------------------------------------------------
def capture_realsense(warmup=30):
    """One aligned colour+depth frame straight off a RealSense."""
    import pyrealsense2 as rs
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
    profile = pipe.start(cfg)
    try:
        align = rs.align(rs.stream.color)
        for _ in range(warmup):            # let auto-exposure and the depth engine settle
            frames = align.process(pipe.wait_for_frames())
        depth_f, color_f = frames.get_depth_frame(), frames.get_color_frame()
        i = color_f.profile.as_video_stream_profile().intrinsics
        intr = {"fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy,
                "width": i.width, "height": i.height}
        scale = 1.0 / profile.get_device().first_depth_sensor().get_depth_scale()
        return (np.asanyarray(color_f.get_data()),
                np.asanyarray(depth_f.get_data()), intr, scale)
    finally:
        pipe.stop()


def save_frame(name, color, depth, intr, scale):
    np.savez_compressed(name + ".npz", color=color, depth=depth,
                        intr=json.dumps(intr), depth_scale=scale)
    return name + ".npz"


def load_frame(path):
    z = np.load(path, allow_pickle=False)
    return (z["color"] if "color" in z else None, z["depth"],
            json.loads(str(z["intr"])), float(z["depth_scale"]))


def view_at(intr, camera_height_m, table_height_m=TABLE_HEIGHT_M):
    """How much floor-at-table-height one frame covers, in metres. (width, height)."""
    z = camera_height_m - table_height_m
    return (intr["width"] / intr["fx"]) * z, (intr["height"] / intr["fy"]) * z


def synthetic(camera_height=3.2, gap=0.8, chairs=True, seed=0,
              diameters=(1.8288, 1.524), tilt_deg=0.0):
    """A ray-cast room: a floor, round tables 0.762 m above it, and chairs to reject.

    Rendered from real projective geometry rather than drawn, so a table lands as the
    ellipse a real camera would see, foreshortened by exactly what the viewing angle
    demands -- which is the whole point, because fitting a circle to that ellipse is the
    mistake this detector exists to avoid.

    The layout is built to FIT the frame, because a D435 does not see much: at 3.2 m it
    covers about 4.9 x 2.8 m, which is two 6 ft rounds and no more. Overlapping the
    tables, as an earlier version of this did, merges them into one blob that the size
    gate then rightly throws away.

    `tilt_deg` leans the camera off vertical. `camera_height` is measured along its axis.
    """
    intr = {"fx": 425.0, "fy": 425.0, "cx": 424.0, "cy": 240.0,
            "width": 848, "height": 480}
    h, w = intr["height"], intr["width"]
    yy, xx = np.mgrid[0:h, 0:w]
    dirs = np.stack([(xx - intr["cx"]) / intr["fx"],
                     (yy - intr["cy"]) / intr["fy"],
                     np.ones_like(xx, dtype=float)], axis=2)

    t = np.radians(tilt_deg)
    n = np.array([0.0, -np.sin(t), -np.cos(t)])          # floor normal, facing the camera
    c_floor = float(n @ np.array([0.0, 0.0, camera_height]))
    c_top = c_floor + TABLE_HEIGHT_M                      # the tabletop, parallel to it

    e1 = np.array([1.0, 0.0, 0.0]) - n * n[0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    nd = dirs @ n
    nd = np.where(np.abs(nd) < 1e-9, 1e-9, nd)

    def surface(c):
        return dirs * (c / nd)[:, :, None]

    depth = np.where(nd != 0, c_floor / nd, 0.0)          # the floor, everywhere
    top = surface(c_top)
    origin_top = np.array([0.0, 0.0, camera_height]) + n * (c_top - c_floor) / (n @ n)

    if not diameters:
        offsets = []
    elif len(diameters) == 1:
        offsets = [0.0]
    else:
        pitch = (diameters[0] + diameters[1]) / 2.0 + gap
        offsets = [-pitch / 2.0, pitch / 2.0]
    truth = []
    for d, ox in zip(diameters, offsets):
        centre = origin_top + ox * e1
        on = np.linalg.norm(top - centre, axis=2) <= d / 2
        depth = np.where(on, c_top / nd, depth)
        truth.append({"centre_cam": centre, "offset": ox, "diameter": d})

    if chairs:
        # A banquet chair: a seat at 0.45 m and a back reaching 0.95 m. The back passes
        # straight THROUGH the table band, so it is the false positive to beat.
        for d, ox in zip(diameters, offsets):
            for k in range(6):
                a = 2 * np.pi * k / 6
                for height_m, halfu, halfv in ((0.45, 0.22, 0.22), (0.95, 0.21, 0.05)):
                    c_obj = c_floor + height_m
                    S = surface(c_obj)
                    o = origin_top + (ox + (d / 2 + 0.30) * np.cos(a)) * e1                         + ((d / 2 + 0.30) * np.sin(a)) * e2
                    du = (S - o) @ e1
                    dv = (S - o) @ e2
                    hit = (np.abs(du) < halfu) & (np.abs(dv) < halfv)
                    depth = np.where(hit, np.minimum(depth, c_obj / nd), depth)

    depth = np.where(depth > 0, depth, 0.0)
    depth = depth + np.random.default_rng(seed).normal(0, 0.004, depth.shape) * (depth > 0)
    color = np.zeros((h, w, 3), dtype=np.uint8)
    return color, (depth * 1000.0).astype(np.uint16), intr, 1000.0, truth


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("frame", nargs="?", help="a saved .npz frame")
    ap.add_argument("--live", action="store_true", help="capture from a RealSense and detect")
    ap.add_argument("--capture", metavar="NAME", help="capture and save, do not detect")
    ap.add_argument("--synthetic", action="store_true", help="a rendered room, no camera")
    ap.add_argument("--height", type=float, default=TABLE_HEIGHT_M,
                    help="table height in metres (default %.3f)" % TABLE_HEIGHT_M)
    ap.add_argument("--band", type=float, default=BAND_M,
                    help="half-width of the height band (default %.2f m)" % BAND_M)
    ap.add_argument("-o", "--out", help="write the JSON here")
    args = ap.parse_args(argv)

    truth = None
    if args.capture:
        color, depth, intr, scale = capture_realsense()
        print("saved " + save_frame(args.capture, color, depth, intr, scale))
        return 0
    if args.synthetic:
        color, depth, intr, scale, truth = synthetic()
        vw, vh = view_at(intr, 3.2)
        print("synthetic: camera 3.20 m up, sees %.1f x %.1f m at table height"
              % (vw, vh))
        name = "synthetic"
    elif args.live:
        color, depth, intr, scale = capture_realsense()
        name = "live"
        save_frame(name, color, depth, intr, scale)
    elif args.frame:
        color, depth, intr, scale = load_frame(args.frame)
        name = os.path.splitext(os.path.basename(args.frame))[0]
    else:
        ap.error("give a frame, or --live, --capture or --synthetic")

    print("frame %dx%d, depth %.2f-%.2f m"
          % (intr["width"], intr["height"],
             depth[depth > 0].min() / scale, depth.max() / scale))
    tables, plane, _ = detect_tables(depth, intr, scale, table_height_m=args.height,
                                     band_m=args.band, verbose=True)

    print("\n  id   px centre        diameter   nearest stock      vs stock   round")
    print("  " + "-" * 70)
    for t in tables:
        m = t["measured"]
        print("  %-4s %5.0f,%5.0f     %.3f m   %-14s  %+5.0f mm   %3.0f%%"
              % (t["id"], t["centre_px"][0], t["centre_px"][1], m["diameter_m"],
                 m["nearest_stock"], m["stock_error_mm"], 100 * t["confidence"]))
    if not tables:
        print("  nothing found -- see the dropped blobs above")

    if truth:
        print("\n  against the rendered truth:")
        for t in tables:
            m = t["measured"]
            best = min(truth, key=lambda g: abs(g["diameter"] - m["diameter_m"]))
            print("    %s  diameter %+.0f mm" % (t["id"],
                  1000 * (m["diameter_m"] - best["diameter"])))

    doc = to_document(tables, intr["width"], intr["height"])
    path = args.out or (name + "_detections.json")
    with open(path, "w") as f:
        f.write(json.dumps(doc, indent=2) + "\n")
    print("\nwrote %s  -- open the planner, Camera -> Import detections" % path)
    if color is not None and color.any():
        p = draw_overlay(color, tables, name + "_overlay.png")
        if p:
            print("wrote %s" % p)
    return 0


if __name__ == "__main__":
    sys.exit(main())
