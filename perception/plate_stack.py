"""Find plate stacks on an already-segmented table, and measure the top plate.

WHAT THIS IS FOR
----------------
The table plane comes from outside this module -- the nav/segmentation side produces it from
the table's known height and the robot's stance, and streams it with the depth frames. That is
the fragile half of the problem and it is already solved, so nothing here re-fits the table.
Given that plane, this answers:

    where are the plate stacks, which plate is on top, and how high is it above the table?

The last number is the one that matters. Plates nest at roughly a centimetre of pitch, so if
the top height is wrong by one pitch the hand closes on the second plate, or on air above the
first. Everything here is arranged around measuring it well.

HOW THE HEIGHT IS MEASURED WELL
-------------------------------
Not from one pixel. The top plate's rim is an annulus of several hundred pixels, all at the
same height by construction, so the median over the annulus averages the depth noise down by
roughly sqrt(N). A D435 at half a metre has a few millimetres of per-pixel noise; over a few
hundred rim pixels that becomes a fraction of a millimetre, which is what makes a ~10 mm pitch
safely resolvable.

THE TRAP: A PLATE IS NOT FLAT
-----------------------------
A dinner plate's rim stands proud of its well by 10-20 mm, so the TOP PLATE ALONE produces two
height clusters. Treating the well as a second plate would overcount every stack and -- worse
-- a well mistaken for the top surface reads ~15 mm low, which is more than a pitch.

This module never looks for "clusters" generically. It takes the TOPMOST cluster only, which is
the rim, and the rim is also the part a finger closes on, so the number it reports is the number
the grasp wants. The well falls outside the band and is ignored.

FRAME
-----
Everything returned is in the CAMERA OPTICAL frame, because that is the frame the depth and the
supplied plane live in. Converting to the robot frame needs the hand-eye calibration, which this
robot does not have yet -- see to_robot_frame(), which refuses rather than guessing.
"""

import numpy as np

import plate_detectors as pd

# --------------------------------------------------------------------------
# tuning. All in metres.
# --------------------------------------------------------------------------

MIN_HEIGHT_M = 0.012      # below this is the table itself, plus its noise
MAX_HEIGHT_M = 0.45       # a stack taller than this is not a stack
TOP_BAND_M = 0.006        # how thick a slice counts as "the top surface".
                          # Must be comfortably under a plate's rim-to-well drop
                          # (10-20 mm) or the well leaks into the top cluster, and
                          # comfortably over the depth noise on the rim.
MIN_COMPONENT_PX = 400    # at full resolution
MIN_RIM_PX = 60           # fewer rim pixels than this and the circle fit is noise
MIN_AXIS_RATIO = 0.35     # the rim seen from anywhere the robot can stand
DIAMETER_TOL = 0.30       # fractional; a blob this far off plate-size is not a plate


class NoStackFound(RuntimeError):
    pass


def _as_plane(plane):
    """Accept (n, c) with n . X = c, or a dict, or a 4-vector [a,b,c,d] meaning
    a x + b y + c z + d = 0. Normalises the normal and returns (n, c)."""
    if isinstance(plane, dict):
        if "normal" in plane:
            n = np.asarray(plane["normal"], dtype=float)
            if "c" in plane:
                c = float(plane["c"])
            elif "d" in plane:
                c = -float(plane["d"])
            elif "point" in plane:
                c = float(n @ np.asarray(plane["point"], dtype=float))
            else:
                raise ValueError("plane dict needs one of 'c', 'd' or 'point' with 'normal'")
        else:
            raise ValueError("plane dict needs a 'normal'")
    else:
        arr = np.asarray(plane, dtype=object)
        if arr.shape == (4,):
            v = np.asarray(plane, dtype=float)
            n, c = v[:3], -v[3]
        elif len(plane) == 2:
            n = np.asarray(plane[0], dtype=float)
            c = float(plane[1])
        else:
            raise ValueError("plane must be (n, c), [a,b,c,d] or a dict")
    ln = np.linalg.norm(n)
    if ln < 1e-9:
        raise ValueError("plane normal is zero-length")
    return n / ln, c / ln


def heights(depth, intrinsics, plane, depth_scale=1000.0):
    """Per-pixel height above the supplied table plane, in metres. NaN where no depth.

    Refuses a plane that puts almost the whole frame underground. That is the signature of
    a plane handed over in the wrong convention, and it is worth catching loudly: the
    failure mode otherwise is an empty stack list, which reads like "no plates here".
    """
    n, c = _as_plane(plane)
    h = pd.height_above_plane(depth, intrinsics, (n, c), depth_scale)
    fin = h[np.isfinite(h)]
    if fin.size < 200:
        raise NoStackFound("almost no valid depth in this frame")
    if np.percentile(fin, 90) < -0.05:
        raise NoStackFound(
            "this plane puts 90%% of the frame more than 50 mm BELOW the table "
            "(90th percentile height %.3f m). The plane is probably inverted or is not "
            "the surface the plates stand on." % float(np.percentile(fin, 90)))
    return h, (n, c)


def _largest_piece(mask):
    """The biggest 4-connected piece of a boolean mask (all False if it is empty)."""
    labels, n = pd.label_mask(mask)
    if n <= 1:
        return mask.copy()
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    return labels == int(np.argmax(sizes))


def _outer_edge(edge_px, bins=180):
    """From an outline, the outermost point in each direction from its centre.

    A plate's rim seen from anywhere is star-shaped about its centre, so the outermost
    point per direction is its outer edge -- whatever inner edges, gaps or well boundary
    the outline also happened to trace.
    """
    e = np.asarray(edge_px, dtype=float)
    if len(e) < 12:
        return e
    c = e.mean(axis=0)
    d = e - c
    r = np.hypot(d[:, 0], d[:, 1])
    a = np.arctan2(d[:, 1], d[:, 0])
    b = np.floor((a + np.pi) / (2 * np.pi) * bins).astype(int) % bins
    keep = []
    for k in range(bins):
        idx = np.nonzero(b == k)[0]
        if idx.size:
            keep.append(idx[np.argmax(r[idx])])
    return e[np.asarray(keep, dtype=int)]


def rim_heights(hs, pitch_hint=None, bin_m=0.002, min_frac=0.02):
    """The heights at which this blob has rim-like ledges, tallest first.

    A nested stack shows one ledge per plate on its side, spaced by the plate pitch, so the
    count of ledges is the count of plates and their spacing is the pitch. Returned as a
    plain list so the caller can decide how much to trust it -- on a stack seen from almost
    straight above, the side is barely visible and there is nothing to count.
    """
    hs = np.asarray(hs, dtype=float)
    hs = hs[np.isfinite(hs)]
    if hs.size < 50:
        return []
    lo, hi = float(hs.min()), float(hs.max())
    if hi - lo < bin_m:
        return [float(np.median(hs))]
    nb = max(4, int(np.ceil((hi - lo) / bin_m)))
    cnt, edges = np.histogram(hs, bins=nb, range=(lo, hi))
    centres = 0.5 * (edges[:-1] + edges[1:])
    thresh = max(min_frac * cnt.max(), 3.0)
    peaks = []
    for i in range(len(cnt)):
        if cnt[i] < thresh:
            continue
        left = cnt[i - 1] if i > 0 else 0
        right = cnt[i + 1] if i + 1 < len(cnt) else 0
        if cnt[i] >= left and cnt[i] >= right:
            peaks.append((float(centres[i]), int(cnt[i])))
    # merge peaks closer together than a plausible pitch; keep the taller
    if pitch_hint:
        merged = []
        for hgt, n in sorted(peaks, key=lambda p: -p[1]):
            if all(abs(hgt - m[0]) > 0.5 * pitch_hint for m in merged):
                merged.append((hgt, n))
        peaks = merged
    return [h for h, _ in sorted(peaks, key=lambda p: -p[0])]


def locate_stacks(depth, intrinsics, plane, *, depth_scale=1000.0, stride=2,
                  plate_diameter_m=0.225, plate_pitch_m=None,
                  plate_rim_height_m=None,
                  min_height_m=MIN_HEIGHT_M, max_height_m=MAX_HEIGHT_M,
                  top_band_m=TOP_BAND_M, min_component_px=MIN_COMPONENT_PX,
                  diameter_tol=DIAMETER_TOL, require_plate_size=True,
                  allow_clipped=False):
    """Every plate stack standing on the supplied table plane, best first.

    Returns a list of dicts, each with:

        top_height_m   the top plate's RIM, above the table plane          <- the headline
        center_m       (3,) the top rim's centre, camera optical frame
        normal         (3,) the table normal; the plate lies parallel to it
        diameter_m     measured, from the rim fitted in its own plane
        rim_rms_m      circle-fit residual; a sanity figure, not a guess
        rim_px         how many pixels the height came from
        axis_ratio     how foreshortened the rim is in the image
        ledges_m       detected rim ledges down the stack's side, tallest first
        n_plates       how many plates, or None when it cannot be told
        count_from     "height" (needs plate_pitch_m AND plate_rim_height_m, and is the
                       reliable one), "ledges" (a lower bound), or None
        pitch_m        median ledge spacing, else None
        size_error     |measured - expected| / expected
        clipped        the blob touches the frame edge, so diameter and centre are a
                       chord's, not a plate's. Rejected unless allow_clipped=True, which
                       also skips the size gate, since gating on a chord is meaningless.
        diameter_reliable  False exactly when clipped. Check it before trusting diameter
                       or centre; top_height_m is unaffected by clipping.
        rejected       why this candidate was dropped (only when it was)

    `require_plate_size=False` returns blobs that are not plate-sized too, with their
    size_error, which is the way to see WHY a frame found nothing.
    """
    depth = np.asarray(depth)
    H, plane_nc = heights(depth, intrinsics, plane, depth_scale)
    n, _ = plane_nc

    sub = H[::stride, ::stride]
    standing = np.isfinite(sub) & (sub > min_height_m) & (sub < max_height_m)
    if standing.sum() * stride * stride < min_component_px:
        raise NoStackFound("nothing is standing on this table plane")

    labels, ncomp = pd.label_mask(pd.fill_holes(standing))
    fx, fy = intrinsics["fx"], intrinsics["fy"]
    cx, cy = intrinsics["cx"], intrinsics["cy"]
    d_sub = depth[::stride, ::stride].astype(np.float64) / depth_scale

    out = []
    clipped_any = []
    for k in range(1, ncomp + 1):
        comp = labels == k
        if comp.sum() * stride * stride < min_component_px:
            continue
        hs = sub[comp]
        hs = hs[np.isfinite(hs)]
        if hs.size < 30:
            continue

        # ---- the TOP surface: the rim, never the well.
        #
        # A percentile seeds this, not the max, so one spiking pixel cannot define the top.
        # But a percentile of a NOISY distribution sits high -- the 99th is about 2.3 sigma
        # up, which at 8 mm of depth noise is ~18 mm, nearly two plate pitches, biased the
        # same way on every frame. That was the sweep's "wrong plate" failure.
        #
        # So the band is re-centred on its own median and re-cut, a few times. The tail that
        # noise pulled in falls outside the re-cut band and stops dragging the answer up.
        top_est = float(np.percentile(hs, 97.0))
        top_mask = comp & np.isfinite(sub) & (sub >= top_est - top_band_m)
        if top_mask.sum() * stride * stride < MIN_RIM_PX:
            continue
        top_height = float(np.median(sub[top_mask]))
        for _ in range(3):
            cut = comp & np.isfinite(sub) & (np.abs(sub - top_height) <= top_band_m)
            if cut.sum() * stride * stride < MIN_RIM_PX:
                break
            h_new = float(np.median(sub[cut]))
            top_mask = cut
            if abs(h_new - top_height) < 1e-5:
                top_height = h_new
                break
            top_height = h_new

        # ---- the rim's OUTER edge, and only that.
        # Found on the first real frames: a real rim's top band is a ring with small gaps,
        # so filling it does not close the well, and the outline traced BOTH edges of the
        # ring. The circle then landed halfway between the plate's edge and the well's
        # edge -- ~200 mm on a bigger plate, with a 35 mm fit residual. So keep only the
        # biggest piece of the band (the ring, not specks of noise), and from its outline
        # only the outermost point in each direction from its centre.
        ring = _largest_piece(top_mask)
        filled = pd.fill_holes(ring)
        edge = _outer_edge(pd.outline(filled))
        if len(edge) < 12:
            continue

        # Does this blob run off the frame? If so its outline is a chord, not a rim, and
        # the circle fitted to it measures nothing. Height survives -- a partial rim still
        # sits at the rim's height -- but diameter and centre do not.
        hh_, ww_ = comp.shape
        edges = [name for name, hit in (("top", comp[0, :].any()), ("bottom", comp[-1, :].any()),
                                        ("left", comp[:, 0].any()), ("right", comp[:, -1].any()))
                 if hit]
        clipped = bool(edges)
        ell = pd.fit_ellipse(edge * stride)
        if ell is None or ell["major"] <= 0:
            continue
        ratio = float(ell["minor"] / ell["major"])

        # ---- fit the rim as a circle in the top plate's own plane.
        # That plane is the table's normal offset to the rim; taking the offset from the
        # rim's own 3-D points avoids reasoning about which way height_above_plane chose
        # to call "up", which is decided per frame from the geometry.
        # Offset from the RING's points only: the filled disc includes the well, which sits
        # lower, and would drag the rim's plane down.
        ys, xs = np.nonzero(ring)
        zs = d_sub[ring]
        good = zs > 0
        if good.sum() < 10:
            continue
        P = np.stack([(xs[good] * stride - cx) * zs[good] / fx,
                      (ys[good] * stride - cy) * zs[good] / fy,
                      zs[good]], axis=1)
        c_top = float(np.median(P @ n))
        fitted = pd.circle_on_plane(edge * stride, intrinsics, (n, c_top))
        if fitted is None:
            continue
        centre_m, radius_m, rms_m = fitted
        diameter_m = 2.0 * radius_m
        size_error = abs(diameter_m - plate_diameter_m) / plate_diameter_m

        ledges = rim_heights(hs, pitch_hint=plate_pitch_m)
        pitch = None
        n_plates = None
        if len(ledges) >= 2:
            gaps = np.diff(sorted(ledges))
            gaps = gaps[gaps > 1e-4]
            if gaps.size:
                pitch = float(np.median(gaps))
                # Two ledges are evenly spaced by definition, so the spacing test below
                # passes on them for free -- which is how this came to report "2 plates"
                # for a stack of ten. Three is the smallest number that can disagree.
                even = len(ledges) >= 3 and float(np.std(gaps)) < 0.4 * pitch
                # and if the real pitch is known, the measured one has to look like it
                if plate_pitch_m:
                    even = even and abs(pitch - plate_pitch_m) < 0.25 * plate_pitch_m
                if even:
                    n_plates = len(ledges)
        count_from = "ledges" if n_plates else None

        # Counting ledges can only ever give a LOWER bound: from near the table normal the
        # stack's side barely shows, so a 16-stack presented 5 evenly spaced ledges and was
        # reported as 5 plates. The top height counts far better, because one plate added is
        # exactly one pitch of height -- but it needs a single plate's rim height, which is
        # a property of the crockery, measured once.
        if plate_pitch_m and plate_rim_height_m is not None:
            n_h = int(round((top_height - plate_rim_height_m) / plate_pitch_m)) + 1
            n_h = max(1, n_h)
            if n_plates is not None and n_plates != n_h:
                # the height is the more trustworthy of the two; say so rather than
                # quietly averaging them
                count_from = "height (ledges said %d)" % n_plates
            else:
                count_from = "height"
            n_plates = n_h

        rec = {
            "top_height_m": top_height,
            "center_m": np.asarray(centre_m, dtype=float),
            "normal": np.asarray(n, dtype=float),
            "diameter_m": diameter_m,
            "rim_rms_m": float(rms_m),
            "rim_px": int(top_mask.sum() * stride * stride),
            "pixels": int(comp.sum() * stride * stride),
            "axis_ratio": ratio,
            "ledges_m": ledges,
            "n_plates": n_plates,
            "count_from": count_from,
            "pitch_m": pitch,
            "size_error": float(size_error),
            "clipped": clipped,
            "clipped_edges": edges,
            "diameter_reliable": not clipped,
            "ellipse": ell,
        }
        if clipped and not allow_clipped:
            rec["rejected"] = "clipped by the frame edge"
            clipped_any.append(rec)
            continue
        # A clipped blob's outline is a chord, so its diameter and axis ratio are both
        # measurements of nothing -- gating on them would be incoherent once the caller has
        # said, by passing allow_clipped, that it wants the height and knows the rest is
        # unusable. So the size gate is skipped here rather than applied to a figure this
        # function has already declared meaningless.
        if clipped:
            out.append(rec)
            continue
        if require_plate_size and (size_error > diameter_tol or ratio < MIN_AXIS_RATIO):
            rec["rejected"] = ("wrong size" if size_error > diameter_tol
                               else "too flat to be a rim")
            continue
        out.append(rec)

    if not out:
        if clipped_any:
            raise NoStackFound(
                "the only plate-sized blob here runs off the frame edge, so its rim is a "
                "chord and its diameter means nothing (measured %.0f mm). Frame the whole "
                "stack -- stand further back or tilt the head up. Its HEIGHT is probably "
                "still good (%.0f mm); pass allow_clipped=True to use it anyway."
                % (1000 * clipped_any[0]["diameter_m"],
                   1000 * clipped_any[0]["top_height_m"]))
        raise NoStackFound(
            "found standing blobs but none was plate-shaped. Re-run with "
            "require_plate_size=False to see what was rejected and why.")
    out.sort(key=lambda r: (r["size_error"], -r["rim_px"]))
    return out


def top_plate(depth, intrinsics, plane, **kw):
    """The single best stack's top plate. Convenience over locate_stacks."""
    return locate_stacks(depth, intrinsics, plane, **kw)[0]


def to_robot_frame(stack, joints=None, angles=None, allow_nominal=False,
                   calibration="auto"):
    """Re-express a stack in pelvis_link: x forward, y left, z up, metres.

    Uses the saved camera calibration (camera_calibration.json, written by
    `check_mount.py --calibrate`), which corrects the camera's tilt and roll from a measured
    level surface. With calibration=None or no file saved it refuses, because the only
    other option is the CAD nominal -- which on this robot is 4 deg out, ~40 mm at the
    plate. allow_nominal=True accepts that anyway, for geometry work only.

    Returns a dict with:
        center_m        top plate's rim centre, pelvis_link
        normal          the plate's up direction, pelvis_link
        top_height_m    rim above the table (unchanged; frame-independent)
        diameter_m, n_plates
        top_above_floor_m, table_above_floor_m   when the calibration knows the floor
        frame, calibration                        what produced these numbers
    """
    import camera_calibration as cc
    import urdf_fk as fk
    import r1_config as cfg

    if joints is None:
        joints = fk.load_urdf(cfg.URDF_PATH)
    cal = cc.load() if calibration == "auto" else calibration
    if cal is not None:
        T = cc.T_pelvis_optical(cal, joints, angles)
        how = cal.get("method", "calibrated")
    else:
        T = fk.T_pelvis_optical(joints, angles, allow_nominal=allow_nominal)
        how = "CAD nominal (uncalibrated)"
    c = T[:3, :3] @ np.asarray(stack["center_m"], dtype=float) + T[:3, 3]
    nrm = T[:3, :3] @ np.asarray(stack["normal"], dtype=float)
    if nrm[2] < 0:                      # report the plate's UP direction
        nrm = -nrm
    out = {"center_m": c, "normal": nrm, "top_height_m": stack["top_height_m"],
           "diameter_m": stack["diameter_m"], "n_plates": stack.get("n_plates"),
           "frame": "pelvis_link (x forward, y left, z up)", "calibration": how}
    if cal is not None and cal.get("pelvis_above_floor_m") is not None:
        out["top_above_floor_m"] = float(c[2] + cal["pelvis_above_floor_m"])
        out["table_above_floor_m"] = out["top_above_floor_m"] - stack["top_height_m"]
    return out
