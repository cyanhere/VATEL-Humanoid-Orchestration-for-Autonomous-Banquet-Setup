"""
Stage 1, v2: plate pose from RGB-D, robust to glossy-plate depth dropout.

Why this version exists: on real captures, a glossy plate's interior causes
specular stereo-depth dropout (RealSense D435), which skews a depth-driven
circle fit off-center. Fix: detect the plate's boundary in the COLOR image
(the rim is a strong, reliable circular edge), and use depth only where it's
known to be trustworthy -- a band over the rim, not the glare-prone interior.
Table orientation (normal) comes from a plane fit on the surrounding table,
which also has clean depth.

Pipeline:
  1. Detect the plate's circular boundary in the color image (Hough circles
     on a blurred/edge image), constrained to a plausible pixel-radius range
     computed from the known plate diameter + an expected working-distance
     window.
  2. Sample depth in an annulus over the rim band (avoiding the outer edge
     and the glare-prone interior) -> robust distance estimate.
  3. Back-project (u, v, distance) through the intrinsics -> 3D center.
  4. Fit a plane to the table region surrounding the plate (RANSAC) -> normal.

Falls back cleanly (raises with a clear message) if no circle is found, so
the sequence controller's Stage-3-style "no reachable grasp" handling has
something to catch.
"""

import sys
import time

import numpy as np
import cv2

# open3d is only needed by estimate_table_normal(); importing it lazily keeps
# the rest of this module usable where open3d is missing or slow to load.


def capture_live_frame(color_topic="/camera/color/image_raw/compressed",
                        depth_topic="/camera/aligned_depth_to_color/image_raw",
                        info_topic="/camera/color/camera_info",
                        timeout=10.0, slop=0.05):
    """
    Blocks until one synced color+depth pair (and CameraInfo) arrive over
    ROS, then returns (color, depth, intrinsics) in memory -- no files
    written. Requires ROS sourced and the camera driver already running
    (ROS sourced, camera driver up -- see setup.md).
    """
    import rospy
    import message_filters
    from sensor_msgs.msg import CompressedImage, Image, CameraInfo
    from cv_bridge import CvBridge

    bridge = CvBridge()
    result = {"color": None, "depth": None, "intrinsics": None}

    def on_info(msg):
        if result["intrinsics"] is None:
            result["intrinsics"] = {
                "width": msg.width, "height": msg.height,
                "fx": msg.K[0], "fy": msg.K[4], "cx": msg.K[2], "cy": msg.K[5],
            }

    def on_pair(color_msg, depth_msg):
        if result["color"] is not None:
            return
        color_np = np.frombuffer(color_msg.data, dtype=np.uint8)
        color_img = cv2.imdecode(color_np, cv2.IMREAD_COLOR)
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        result["color"] = color_img
        result["depth"] = depth_img.astype(np.uint16)

    # init_node throws if called twice in one process. The end-to-end script
    # can capture more than once (and other modules may have initialised ROS
    # already), so only initialise if this process has no node yet.
    if not rospy.core.is_initialized():
        rospy.init_node("plate_perception_live", anonymous=True, disable_signals=True)

    rospy.Subscriber(info_topic, CameraInfo, on_info)
    color_sub = message_filters.Subscriber(color_topic, CompressedImage)
    depth_sub = message_filters.Subscriber(depth_topic, Image)
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=5, slop=slop)
    ts.registerCallback(on_pair)

    start = time.time()
    while result["color"] is None or result["intrinsics"] is None:
        if time.time() - start > timeout:
            missing = []
            if result["color"] is None:
                missing.append("color/depth pair")
            if result["intrinsics"] is None:
                missing.append("camera intrinsics")
            raise TimeoutError(
                f"Timed out waiting for {', '.join(missing)}. "
                "Check the camera driver (roscore + rs_camera.launch) is running."
            )
        time.sleep(0.05)

    return result["color"], result["depth"], result["intrinsics"]


SIZE_DEPTH_TOLERANCE = 0.20   # fitted radius vs depth may disagree by this fraction


def detect_plate_circle(color: np.ndarray, depth: np.ndarray, intrinsics: dict,
                         known_diameter_m: float,
                         min_distance_m: float = 0.25,
                         max_distance_m: float = 1.0,
                         dp: float = 1.0, param1: float = 100, param2: float = 30,
                         depth_scale: float = 1000.0):
    """
    Find the plate's boundary circle in the color image.
    Returns (u, v, r_px) in pixel coordinates, or raises RuntimeError.

    A decorated plate offers Hough several strong circles -- the outer rim, and
    every band painted inside it. Taking the strongest response picks whichever
    has the crispest edge, which on a plate with a bold inner ring is NOT the rim.
    That failure is quiet and expensive: the circle is confidently wrong, the depth
    annulus then straddles the rim and the table beyond it, and the plate reads a
    few centimetres low and to one side. rim_valid stays high throughout, because
    it only counts valid depth samples and knows nothing about which circle it
    sampled (observed 2026-09-29: 90% valid on a circle sitting on the inner ring).

    The fix uses the one thing a wrong concentric circle still gets right: the CENTRE.
    Find any circle, measure the depth inside it, and let that depth say how big the
    rim must be -- then look only there. A band painted at 78% of the radius is outside
    that window, so it cannot be mistaken for the rim.
    """
    fx = intrinsics["fx"]
    r_min_px = int(fx * (known_diameter_m / 2.0) / max_distance_m * 0.8)
    r_max_px = int(fx * (known_diameter_m / 2.0) / min_distance_m * 1.2)

    gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    def hough(rmin, rmax):
        return cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=dp,
            minDist=gray.shape[0] // 2,   # one plate in frame; see the note below
            param1=param1, param2=param2,
            minRadius=int(max(1, rmin)), maxRadius=int(rmax))

    # Pass 1 finds SOME concentric circle -- rim or painted band, whichever has the
    # crisper edge. Either way its CENTRE is the plate's, and that is all this pass is
    # for. (Asking for both at once does not work: minDist suppresses circles by centre
    # separation, and concentric circles are 0 px apart, so only one ever comes back.)
    first = hough(r_min_px, r_max_px)
    if first is None:
        raise RuntimeError(
            f"No plate circle found (searched radius {r_min_px}-{r_max_px}px). "
            "Check lighting/contrast, or widen min/max_distance_m.")
    u0, v0, r0 = (float(x) for x in first[0][0])

    # How far away is it? Depth over the disc inside that circle -- plate interior
    # whichever band was found, since they share a centre.
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    inside = ((xx - u0) ** 2 + (yy - v0) ** 2 <= (0.9 * r0) ** 2) & (depth > 0)
    if inside.sum() >= 30:
        d_depth = float(np.median(depth[inside])) / depth_scale
    else:
        d_depth = sample_rim_depth(depth, u0, v0, r0, depth_scale=depth_scale)[0]

    # Pass 2 looks only where the depth says the rim must be. A painted ring at 78% of
    # the radius is simply not in this window, so it cannot win.
    r_expect = fx * (known_diameter_m / 2.0) / d_depth
    second = hough(r_expect * (1 - SIZE_DEPTH_TOLERANCE), r_expect * (1 + SIZE_DEPTH_TOLERANCE))
    if second is None:
        # Not fatal any more. The radius returned here is only a hint: locate_plate
        # measures the real one off the depth silhouette and uses nothing from this
        # function but the CENTRE, which pass 1 already has. Raising here would throw
        # away a good centre over a radius nobody reads.
        return u0, v0, r0
    u, v, r_px = (float(x) for x in second[0][0])
    return u, v, r_px


SILHOUETTE_STEP_M = 0.012   # a surface this far above the table plane is the plate
# Anything this much NEARER than the plate is between the camera and the scene -- at the
# ready pose that is the robot's own raised arms, which fill the left and right of the
# frame. They stand above the table too, so without this they are taken for the plate:
# on 2026-09-29 the fitted circle came out 2.5x too big, the table plane 41 deg off
# level, and the tabletop 0.28 m ABOVE the plate.
NEAR_OBJECT_MARGIN_M = 0.03
PLATE_DEPTH_BAND_M = 0.05   # a silhouette point must be at the plate's own distance
# A circle fitted to a one-sided ARC has a centre pulled toward the visible side, and it
# looks perfectly confident doing it. That biases both grasp targets the same way, so one
# hand ends up tight against the rim and the other short of it -- which is what was seen
# on 2026-09-29 with a hand occluding part of the plate. Refuse past this much missing.
MAX_ARC_GAP_DEG = 150.0
DIAMETER_TOLERANCE = 0.12   # the colour circle is believed only this close to the real plate


def _plate_depth_at(depth, u, v, depth_scale=1000.0, radii=(25, 50, 90)):
    """How far away the plate is, from a small patch at the circle's centre.

    Deliberately a fixed patch in PIXELS rather than a fraction of the seed radius:
    the seed's radius is the part that goes wrong, while its centre stays on the
    plate (concentric circles share one). A patch scaled off a bad radius would
    reach out onto whatever made it bad.
    """
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    d2 = (xx - u) ** 2 + (yy - v) ** 2
    for r in radii:
        inside = (d2 <= r * r) & (depth > 0)
        if inside.sum() >= 20:
            return float(np.median(depth[inside])) / depth_scale
    raise RuntimeError("no valid depth at the plate's centre -- glare or a dead frame")


def _table_plane(depth, intrinsics, u, v, r_px, depth_scale=1000.0,
                  margin_frac=1.25, band_px=70, d_plate=None):
    """Least-squares plane through the table ring around the plate, as n . X = c.

    Fitted once, from near the plate. Judging each ray against the depth at its
    own far end instead does not work: rays that run off the end of the table
    take the floor beyond as their reference, and the tabletop then reads as if
    it were standing proud of it.
    """
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    dist = np.hypot(xx - u, yy - v)
    ring = ((dist >= margin_frac * r_px) & (dist <= margin_frac * r_px + band_px)
            & (depth > 0))
    if d_plate is not None:      # the robot's own arms are nearer than the plate
        ring &= depth / depth_scale > d_plate - NEAR_OBJECT_MARGIN_M
    if ring.sum() < 100:
        raise RuntimeError("not enough table visible around the plate to fit a plane")

    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    ys, xs = np.where(ring)
    zs = depth[ys, xs].astype(np.float64) / depth_scale
    pts = np.stack([(xs - cx) * zs / fx, (ys - cy) * zs / fy, zs], axis=1)

    keep = np.ones(len(pts), bool)
    n, c = np.array([0.0, 0.0, 1.0]), float(np.median(zs))
    for _ in range(3):
        p = pts[keep]
        centroid = p.mean(axis=0)
        n = np.linalg.svd(p - centroid, full_matrices=False)[2][-1]
        c = float(n @ centroid)
        resid = np.abs(pts @ n - c)
        nxt = resid < max(0.004, 3.0 * float(np.median(resid[keep])))
        if nxt.sum() < 50:
            break
        keep = nxt
    return n, c


def _circumcircle(p, q, s):
    d = 2 * (p[0] * (q[1] - s[1]) + q[0] * (s[1] - p[1]) + s[0] * (p[1] - q[1]))
    if abs(d) < 1e-9:
        return None
    pp_, qq, ss = p @ p, q @ q, s @ s
    ux = (pp_ * (q[1] - s[1]) + qq * (s[1] - p[1]) + ss * (p[1] - q[1])) / d
    uy = (pp_ * (s[0] - q[0]) + qq * (p[0] - s[0]) + ss * (q[0] - p[0])) / d
    return ux, uy, float(np.hypot(p[0] - ux, p[1] - uy))


def _fit_circle_ransac(pts, r_hint, tol=4.0, iters=800, seed=0,
                        lo=0.5, hi=1.8):
    """RANSAC, because the outliers here are systematic rather than random: the
    support under the plate and a hand resting on the table both stand above it
    too, and a plain least-squares fit gets dragged off the plate by them."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i, j, k = rng.choice(len(pts), 3, replace=False)
        c = _circumcircle(pts[i], pts[j], pts[k])
        if c is None or not (lo * r_hint < c[2] < hi * r_hint):
            continue
        inl = np.abs(np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1]) - c[2]) < tol
        if best is None or inl.sum() > best[0]:
            best = (int(inl.sum()), inl)
    if best is None or best[0] < 12:
        raise RuntimeError("no circle fits the traced silhouette")

    p = pts[best[1]]
    x, y = p[:, 0], p[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    sol = np.linalg.lstsq(A, -(x ** 2 + y ** 2), rcond=None)[0]
    cx, cy = -sol[0] / 2, -sol[1] / 2
    return float(cx), float(cy), float(np.sqrt(max(cx ** 2 + cy ** 2 - sol[2], 1e-9)))


def arc_gap_deg(pts, circle, tol=4.0):
    """The largest stretch of the fitted circle with no traced edge on it, in degrees.

    Small means the edge was seen most of the way round and the centre is pinned by
    points on opposite sides. Large means it was fitted to an arc, and an arc's centre
    slides along its own bisector for very little penalty.
    """
    cx, cy, r = circle
    on = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r) < tol
    if on.sum() < 3:
        return 360.0
    a = np.sort(np.degrees(np.arctan2(pts[on, 1] - cy, pts[on, 0] - cx)) % 360.0)
    return float(np.max(np.diff(np.concatenate([a, a[:1] + 360.0]))))


def refine_circle_by_depth(depth, intrinsics, u0, v0, known_diameter_m,
                            depth_scale=1000.0, step_m=SILHOUETTE_STEP_M, n_rays=96,
                            return_points=False):
    """Re-fit the plate's circle on its silhouette against the table.

    The plate stands above the table, and that step is something no paint can
    imitate -- which is exactly what the colour detector cannot tell apart.

    Each ray keeps the OUTERMOST pixel standing above the plane, so a hole in the
    glossy interior cannot end one early. (Marching out until depth stops matching
    the centre's fails for that reason: on a real capture the interior was mostly
    specular dropout and every ray died well inside the plate.)
    """
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    h, w = depth.shape

    # Scale everything off the distance the plate actually is, not off the seed's
    # radius. The seed comes from colour Hough and its radius is exactly what cannot
    # be trusted; its distance-implied size can be.
    d_plate = _plate_depth_at(depth, u0, v0, depth_scale)
    r_expect = fx * (known_diameter_m / 2.0) / d_plate
    n, c = _table_plane(depth, intrinsics, u0, v0, r_expect, depth_scale,
                        d_plate=d_plate)

    pts = []
    for a in np.linspace(0, 2 * np.pi, n_rays, endpoint=False):
        rs = np.arange(0.35 * r_expect, 1.4 * r_expect, 1.0)
        xs, ys = u0 + rs * np.cos(a), v0 + rs * np.sin(a)
        on = (xs >= 0) & (xs < w) & (ys >= 0) & (ys < h)
        if on.sum() < 10:
            continue
        rs, xs, ys = rs[on], xs[on], ys[on]
        d = depth[ys.astype(int), xs.astype(int)].astype(np.float64) / depth_scale
        denom = n[0] * (xs - cx) / fx + n[1] * (ys - cy) / fy + n[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            z_table = c / denom
        above = ((d > 0) & (np.abs(denom) > 1e-6) & (z_table > 0)
                 & (d < z_table - step_m)
                 & (np.abs(d - d_plate) < PLATE_DEPTH_BAND_M))
        if above.any():
            r_edge = rs[above].max()
            pts.append((u0 + r_edge * np.cos(a), v0 + r_edge * np.sin(a)))

    pts = np.asarray(pts)
    if len(pts) < 20:
        raise RuntimeError(
            "only %d points on the plate's depth silhouette -- is the plate "
            "standing on a support? lying flat there is no step to find" % len(pts))
    circle = _fit_circle_ransac(pts, r_expect, lo=0.75, hi=1.25)
    gap = arc_gap_deg(pts, circle)
    if gap > MAX_ARC_GAP_DEG:
        raise RuntimeError(
            "the plate's edge was only traced over part of its circumference -- %.0f deg "
            "of it shows no edge at all. Something is covering that side, and a circle "
            "fitted to an arc puts its centre off toward the part it can see."
            % gap)
    return (circle + (pts,)) if return_points else circle


def locate_plate(depth, intrinsics, u, v, r_px, known_diameter_m, depth_scale=1000.0):
    """The circle the grasp should use: depth if it can be had, colour if it is at
    least the right SIZE, and otherwise nothing.

    Only the colour circle's CENTRE is used, never its radius -- the radius is the
    part that goes wrong, and the plate's distance gives a better one. A decorated plate offers Hough
    several concentric circles and it takes whichever has the crispest edge. On
    2026-09-29 that was a circle implying a 0.272 m plate for a 0.225 m one, and
    nothing downstream noticed: rim_valid stayed at 91% throughout, because it
    only counts valid depth samples and cannot tell which circle they came from.
    Hence the size gate -- a circle that is confidently wrong is worse than none.
    """
    try:
        cu, cv, cr = refine_circle_by_depth(depth, intrinsics, u, v,
                                             known_diameter_m, depth_scale=depth_scale)
        d, _ = sample_rim_depth(depth, cu, cv, cr, depth_scale=depth_scale)
        implied = 2 * cr * d / intrinsics["fx"]
        if abs(implied - known_diameter_m) > DIAMETER_TOLERANCE * known_diameter_m:
            raise RuntimeError(
                "the depth silhouette implies a %.3f m plate, not %.3f m -- it has "
                "traced something other than the plate" % (implied, known_diameter_m))
        return cu, cv, cr, "depth silhouette"
    except RuntimeError as depth_failed:
        d, _ = sample_rim_depth(depth, u, v, r_px, depth_scale=depth_scale)
        implied = 2 * r_px * d / intrinsics["fx"]
        wrong_by = abs(implied - known_diameter_m) / known_diameter_m
        if wrong_by > DIAMETER_TOLERANCE:
            raise RuntimeError(
                "Could not find the plate's edge in depth (%s), and the colour "
                "circle is not believable either -- it implies a %.3f m plate "
                "where yours is %.3f m, %.0f%% out. Refusing rather than grasping "
                "at the wrong circle." % (depth_failed, implied, known_diameter_m,
                                          wrong_by * 100))
        return u, v, r_px, "colour only (%s)" % depth_failed


def sample_rim_depth(depth: np.ndarray, u: float, v: float, r_px: float,
                      inner_frac: float = 0.75, outer_frac: float = 0.93,
                      n_samples: int = 64, depth_scale: float = 1000.0):
    """
    Sample depth in an annulus over the rim band (inner_frac*r to
    outer_frac*r), which per the validity check sits reliably inside valid
    depth while avoiding the glare-prone interior and the blurry outer edge.
    Returns (median depth in metres, valid_frac), or raises RuntimeError if
    too few valid samples are found (signals the rim itself may have depth
    trouble too).
    """
    h, w = depth.shape
    angles = np.linspace(0, 2 * np.pi, n_samples, endpoint=False)
    radii = np.array([inner_frac, (inner_frac + outer_frac) / 2, outer_frac]) * r_px

    samples = []
    for r in radii:
        for a in angles:
            px = int(round(u + r * np.cos(a)))
            py = int(round(v + r * np.sin(a)))
            if 0 <= px < w and 0 <= py < h:
                d = depth[py, px]
                if d > 0:
                    samples.append(d)

    samples = np.asarray(samples, dtype=np.float64)
    valid_frac = samples.size / (n_samples * len(radii))
    if samples.size < 10:
        raise RuntimeError(
            f"Only {samples.size} valid depth samples on the rim band "
            f"({valid_frac:.0%} valid) -- rim depth may be unreliable in this frame."
        )

    return float(np.median(samples)) / depth_scale, valid_frac


def estimate_table_normal(depth: np.ndarray, intrinsics: dict,
                           u: float, v: float, r_px: float,
                           margin_frac: float = 1.15, band_px: float = 60,
                           depth_scale: float = 1000.0,
                           distance_threshold: float = 0.005,
                           plate_distance_m=None):
    """
    Fit a plane to the table region just outside the plate's detected
    circle (an annulus from margin_frac*r_px out to +band_px), which has
    clean depth per the validity check. Returns (normal in camera frame,
    mean table distance) for a sanity check against the rim-band distance.
    """
    h, w = depth.shape
    yy, xx = np.mgrid[0:h, 0:w]
    dist_from_center = np.sqrt((xx - u) ** 2 + (yy - v) ** 2)

    r_inner = margin_frac * r_px
    r_outer = r_inner + band_px
    ring_mask = (dist_from_center >= r_inner) & (dist_from_center <= r_outer) & (depth > 0)
    if plate_distance_m is not None:   # drop the robot's own arms, nearer than the plate
        ring_mask &= depth / depth_scale > plate_distance_m - NEAR_OBJECT_MARGIN_M

    if ring_mask.sum() < 100:
        raise RuntimeError(
            "Not enough valid table points around the plate to fit a plane -- "
            "widen band_px or check the plate isn't near the frame edge."
        )

    import open3d as o3d

    fx, fy, cx, cy = intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]
    ys, xs = np.where(ring_mask)
    zs = depth[ys, xs].astype(np.float64) / depth_scale
    xs3d = (xs - cx) * zs / fx
    ys3d = (ys - cy) * zs / fy
    pts = np.stack([xs3d, ys3d, zs], axis=1)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold, ransac_n=3, num_iterations=1000)

    normal = np.array(plane_model[:3])
    normal = normal / np.linalg.norm(normal)
    mean_table_z = float(pts[inliers][:, 2].mean())
    return normal, mean_table_z


def get_plate_pose(color: np.ndarray, depth: np.ndarray, intrinsics: dict,
                    known_plate_diameter_m: float,
                    min_distance_m: float = 0.25, max_distance_m: float = 1.0,
                    depth_scale: float = 1000.0):
    """
    Full v2 pipeline. Returns dict with:
      center: (x, y, z) in camera frame, metres -- distance = center[2]
      distance_m: rim-band median depth (robust to interior glare)
      normal: table plane normal, camera frame
      pixel_circle: (u, v, r_px) -- useful for overlay/debug visualization
      rim_depth_valid_frac: fraction of rim-band samples that had valid depth
      circle_source: which measurement the circle came from
    """
    u, v, r_px = detect_plate_circle(
        color, depth, intrinsics, known_plate_diameter_m, min_distance_m,
        max_distance_m, depth_scale=depth_scale)
    u, v, r_px, source = locate_plate(depth, intrinsics, u, v, r_px,
                                       known_plate_diameter_m, depth_scale)

    Z, valid_frac = sample_rim_depth(depth, u, v, r_px, depth_scale=depth_scale)

    fx, fy, cx, cy = intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"]
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    center = np.array([X, Y, Z])

    normal, table_z = estimate_table_normal(depth, intrinsics, u, v, r_px,
                                             depth_scale=depth_scale,
                                             plate_distance_m=Z)
    if np.dot(normal, -center) < 0:
        normal = -normal

    return {
        "center": center.tolist(),
        "distance_m": Z,
        "normal": normal.tolist(),
        "pixel_circle": (u, v, r_px),
        "rim_depth_valid_frac": valid_frac,
        "circle_source": source,
        "table_plane_distance_m": table_z,
    }


def run_on_capture(idx: str, intrinsics: dict, known_plate_diameter_m: float):
    """
    Run the pipeline on one captured pair (e.g. idx='0000') and save its
    overlay. Returns a result dict on success, or a dict with an 'error' key
    on failure -- callers should keep going rather than aborting the batch.
    """
    color_path = f"{idx}_color.jpg"
    depth_path = f"{idx}_depth.png"
    color = cv2.imread(color_path)
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)

    if color is None or depth is None:
        return {"idx": idx, "error": f"could not read {color_path} / {depth_path}"}

    try:
        pose = get_plate_pose(color, depth, intrinsics, known_plate_diameter_m)
    except RuntimeError as e:
        return {"idx": idx, "error": str(e)}

    u, v, r_px = pose["pixel_circle"]
    vis = color.copy()
    cv2.circle(vis, (int(u), int(v)), 5, (0, 0, 255), -1)
    cv2.circle(vis, (int(u), int(v)), int(r_px), (0, 255, 0), 2)
    overlay_path = f"{idx}_overlay_v2.png"
    cv2.imwrite(overlay_path, vis)

    pose["idx"] = idx
    pose["overlay_path"] = overlay_path
    return pose


def batch(prefixes, intrinsics, known_plate_diameter_m, out="batch_results.json"):
    """Replay saved <prefix>_color.jpg / _depth.png pairs into a results file
    that `bimanual_pickup.py --offline` can read. No camera, no robot."""
    import json
    results = [run_on_capture(p, intrinsics, known_plate_diameter_m) for p in prefixes]
    for r in results:
        print("  %-16s %s" % (r.get("idx"), r.get("error") or
                              "%.3f m, rim_valid %.0f%%" % (r["distance_m"],
                                                            r["rim_depth_valid_frac"] * 100)))
    with open(out, "w") as f:
        json.dump(results, f, indent=2, default=float)
    print("wrote %s (%d ok, %d failed)"
          % (out, sum("error" not in r for r in results), sum("error" in r for r in results)))
    return results


if __name__ == "__main__":
    KNOWN_DIAMETER_M = 0.225  # tape-measured 2026-09-29

    if len(sys.argv) > 1 and sys.argv[1] == "batch":
        import glob
        import json
        prefixes = sorted(p[:-len("_color.jpg")] for p in glob.glob("*_color.jpg"))
        if not prefixes:
            raise SystemExit("no *_color.jpg here -- copy recordings off the robot first")
        scene = sorted(glob.glob("*_scene.json"))
        if not scene:
            raise SystemExit("no *_scene.json -- intrinsics unknown, re-record with a "
                             "current capture_scene()")
        with open(scene[0]) as f:
            intrinsics = json.load(f)["intrinsics"]
        batch(prefixes, intrinsics, KNOWN_DIAMETER_M)
        raise SystemExit(0)

    print("Waiting for a live color/depth frame...")
    color, depth, intrinsics = capture_live_frame()
    print("Got a frame. Running perception...")

    try:
        pose = get_plate_pose(color, depth, intrinsics, KNOWN_DIAMETER_M)
    except RuntimeError as e:
        print(f"FAILED: {e}")
        raise SystemExit(1)

    print(f"distance={pose['distance_m']:.4f}m  "
          f"center=({pose['center'][0]:.3f}, {pose['center'][1]:.3f}, {pose['center'][2]:.3f})  "
          f"rim_valid={pose['rim_depth_valid_frac']:.0%}")

    u, v, r_px = pose["pixel_circle"]
    vis = color.copy()
    cv2.circle(vis, (int(u), int(v)), 5, (0, 0, 255), -1)
    cv2.circle(vis, (int(u), int(v)), int(r_px), (0, 255, 0), 2)
    cv2.imwrite("live_overlay.png", vis)
    print("saved live_overlay.png")