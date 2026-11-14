"""
Ways to find the plate, so they can be compared on the same frame.

WHY THIS EXISTS
---------------
The plate is a circle lying flat and the camera looks at it from 22-41 deg off its
normal, so what appears in the image is an ELLIPSE, squashed by 7-24% at the working
distances actually used. Every detector before this one fitted a CIRCLE to it:

  - Hough circles, which only search for circles at all
  - the RANSAC circle fit on the depth silhouette

A circle fitted to a 15%-squashed ellipse lands somewhere between the semi-minor and
semi-major axis, so the diameter it implies is wrong by up to about 8% in whichever
direction the fit happened to favour, and the centre wanders with it. That is the
0.252-0.261 m readings for a 0.225 m plate, and the refusals that came with them.

The fix is to fit what is actually there. An ellipse's MAJOR axis is the unforeshortened
one -- it is the plate's true diameter, whatever the viewing angle -- so

    diameter = 2 * semi_major * distance / fx

is a measurement rather than an estimate, and the size gate finally means something.

Everything here is pure numpy: no cv2, no scipy. That is deliberate, so the geometry can
be tested on a laptop rather than only on the robot.
"""

import numpy as np


# --------------------------------------------------------------------------
# ellipse fitting
# --------------------------------------------------------------------------

def fit_ellipse(pts):
    """Direct least-squares ellipse fit (Fitzgibbon). Returns a dict or None.

    Keys: cx, cy, major, minor (SEMI-axes), theta (radians, major axis direction).

    Coordinates are centred and scaled first: the design matrix holds x^2 terms, and at
    pixel magnitudes those are ~1e5 while the linear terms are ~1e2, which conditions the
    problem badly enough to matter.
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 6:
        return None
    x, y = pts[:, 0], pts[:, 1]
    mx, my = float(x.mean()), float(y.mean())
    s = float(np.sqrt((((x - mx) ** 2 + (y - my) ** 2)).mean()))
    if not np.isfinite(s) or s < 1e-9:
        return None
    X, Y = (x - mx) / s, (y - my) / s

    # Halir & Flusser's split of Fitzgibbon's method. The textbook single 6x6 version
    # inverts the scatter matrix, which is exactly singular when the points lie on a
    # conic -- i.e. on clean data, where the answer is its null vector. Splitting the
    # quadratic and linear parts reduces it to a 3x3 that stays well conditioned, and
    # the constraint still guarantees an ellipse rather than a hyperbola, which matters
    # on the partial outline an occluded plate gives.
    D1 = np.stack([X * X, X * Y, Y * Y], axis=1)
    D2 = np.stack([X, Y, np.ones_like(X)], axis=1)
    S1, S2, S3 = D1.T @ D1, D1.T @ D2, D2.T @ D2
    try:
        T = -np.linalg.solve(S3, S2.T)
    except np.linalg.LinAlgError:
        return None
    red = S1 + S2 @ T
    red = np.array([red[2] / 2.0, -red[1], red[0] / 2.0])   # premultiplied by inv(C1)
    try:
        w, v = np.linalg.eig(red)
    except np.linalg.LinAlgError:
        return None
    v = np.real(v)
    cond = 4.0 * v[0] * v[2] - v[1] ** 2                    # >0 picks the ellipse
    idx = np.where(np.isfinite(cond) & (cond > 0))[0]
    if idx.size == 0:
        return None
    a1 = v[:, idx[0]]
    a = np.concatenate([a1, T @ a1])

    A, B, Cc, Dd, E, F = a
    # Quadratic form, so the axes come out of an eigendecomposition rather than the
    # usual closed-form soup -- fewer sign conventions to get wrong.
    M = np.array([[A, B / 2.0], [B / 2.0, Cc]])
    try:
        centre = np.linalg.solve(M, [-Dd / 2.0, -E / 2.0])
    except np.linalg.LinAlgError:
        return None
    const = F + (Dd / 2.0) * centre[0] + (E / 2.0) * centre[1]
    evals, evecs = np.linalg.eigh(M)
    if np.any(np.abs(evals) < 1e-12):
        return None
    axes2 = -const / evals
    if np.any(axes2 <= 0):
        return None                      # the conic is a hyperbola, not an ellipse
    axes = np.sqrt(axes2)
    order = np.argsort(axes)[::-1]       # major first
    major, minor = axes[order[0]], axes[order[1]]
    vmaj = evecs[:, order[0]]
    return {"cx": mx + s * centre[0], "cy": my + s * centre[1],
            "major": s * major, "minor": s * minor,
            "theta": float(np.arctan2(vmaj[1], vmaj[0]))}


def ellipse_residual(pts, e):
    """Mean distance from the points to the fitted ellipse, in pixels (approximate --
    the radial distance along each point's own direction from the centre)."""
    pts = np.asarray(pts, dtype=float)
    dx, dy = pts[:, 0] - e["cx"], pts[:, 1] - e["cy"]
    ct, st = np.cos(-e["theta"]), np.sin(-e["theta"])
    u, v = dx * ct - dy * st, dx * st + dy * ct
    r = np.hypot(u, v)
    ang = np.arctan2(v, u)
    on = np.hypot(e["major"] * np.cos(ang), e["minor"] * np.sin(ang))
    return float(np.mean(np.abs(r - on)))


# --------------------------------------------------------------------------
# connected components, without cv2 or scipy
# --------------------------------------------------------------------------

def label_mask(mask):
    """4-connected labelling of a boolean mask. Returns (labels, count).

    Union-find over the set pixels. Pure Python on purpose: it keeps the whole detector
    testable where cv2 and scipy are not installed, and at the sizes here it costs
    fractions of a second.
    """
    mask = np.asarray(mask, dtype=bool)
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    parent = [0]

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    ys, xs = np.nonzero(mask)
    for y, x in zip(ys.tolist(), xs.tolist()):
        up = labels[y - 1, x] if y > 0 else 0
        left = labels[y, x - 1] if x > 0 else 0
        if up and left:
            labels[y, x] = min(up, left)
            union(up, left)
        elif up or left:
            labels[y, x] = up or left
        else:
            parent.append(len(parent))
            labels[y, x] = len(parent) - 1

    roots = {}
    out = np.zeros_like(labels)
    for y, x in zip(ys.tolist(), xs.tolist()):
        r = find(labels[y, x])
        if r not in roots:
            roots[r] = len(roots) + 1
        out[y, x] = roots[r]
    return out, len(roots)


def fill_holes(mask):
    """Close gaps that do not reach the frame edge.

    The glaze drops out in the MIDDLE of the plate, so without this one disc arrives as
    dozens of fragments and every fit is on an arc. A hole is any background region that
    cannot reach the border.
    """
    mask = np.asarray(mask, dtype=bool)
    lab, n = label_mask(~mask)
    if n == 0:
        return mask.copy()
    touching = set(np.unique(np.concatenate(
        [lab[0], lab[-1], lab[:, 0], lab[:, -1]])).tolist())
    out = mask.copy()
    for k in range(1, n + 1):
        if k not in touching:
            out[lab == k] = True
    return out


def outline(component):
    """Boundary pixels of a boolean component, as an (N, 2) array of (x, y)."""
    c = np.asarray(component, dtype=bool)
    inner = np.ones_like(c)
    inner[1:, :] &= c[:-1, :]
    inner[:-1, :] &= c[1:, :]
    inner[:, 1:] &= c[:, :-1]
    inner[:, :-1] &= c[:, 1:]
    ys, xs = np.nonzero(c & ~inner)
    return np.stack([xs, ys], axis=1).astype(float)


# --------------------------------------------------------------------------
# the table plane, fitted over the whole frame rather than around a seed
# --------------------------------------------------------------------------

def dominant_plane(depth, intrinsics, depth_scale=1000.0, stride=4,
                   iters=200, tol=0.01, seed=0):
    """The biggest flat surface in view, as (n, c) with n . X = c.

    RANSAC over the whole frame, so it needs no seed circle -- which is the point: every
    earlier plane fit was anchored on a circle that might itself be wrong.
    """
    d = np.asarray(depth)[::stride, ::stride].astype(np.float64) / depth_scale
    h, w = d.shape
    yy, xx = np.mgrid[0:h, 0:w]
    ok = d > 0
    if ok.sum() < 200:
        raise RuntimeError("almost no valid depth in this frame")
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    zs = d[ok]
    us, vs = xx[ok] * stride, yy[ok] * stride
    pts = np.stack([(us - cx) * zs / fx, (vs - cy) * zs / fy, zs], axis=1)

    rng = np.random.default_rng(seed)
    best = (0, None, None)
    for _ in range(iters):
        i = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[i]
        n = np.cross(p1 - p0, p2 - p0)
        ln = np.linalg.norm(n)
        if ln < 1e-9:
            continue
        n = n / ln
        c = float(n @ p0)
        inl = int((np.abs(pts @ n - c) < tol).sum())
        if inl > best[0]:
            best = (inl, n, c)
    if best[1] is None:
        raise RuntimeError("no plane found in this frame")

    n, c = best[1], best[2]
    for _ in range(3):                       # refine on the inliers
        keep = np.abs(pts @ n - c) < tol
        if keep.sum() < 50:
            break
        q = pts[keep]
        centroid = q.mean(axis=0)
        n = np.linalg.svd(q - centroid, full_matrices=False)[2][-1]
        c = float(n @ centroid)
    return n, c


def height_above_plane(depth, intrinsics, plane, depth_scale=1000.0):
    """Perpendicular height of every pixel above the plane, in metres. NaN where the
    depth is missing."""
    d = np.asarray(depth).astype(np.float64) / depth_scale
    h, w = d.shape
    yy, xx = np.mgrid[0:h, 0:w]
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    n, c = plane
    # A pixel's ray is dir = ((u-cx)/fx, (v-cy)/fy, 1) and the point at depth z is z*dir,
    # so n . P - c is just z*(n . dir) - c. A point NEARER than the plane along the same
    # ray has a smaller z, which makes that quantity -sign(n . dir) -- hence the flip,
    # taken from the geometry rather than from the median of the image (which is ~0,
    # since most of the frame IS the plane, and so decides nothing).
    nd = n[0] * (xx - cx) / fx + n[1] * (yy - cy) / fy + n[2]
    sign = 1.0 if np.median(nd) > 0 else -1.0
    return np.where(d > 0, -(d * nd - c) * sign, np.nan)


def circle_on_plane(edge_px, intrinsics, plane):
    """Fit the plate as a circle IN ITS OWN PLANE, from the outline's pixels.

    Returns (centre_xyz, radius_m, rms_m).

    This is the measurement; the 2D ellipse is only for drawing. Two things go wrong if
    the ellipse is used directly:

      - its RADIUS is foreshortened in one direction, so only the major axis means
        anything, and only if the fit found the right axis
      - its CENTRE is not the projection of the circle's centre. For a tilted circle
        those are different points, by 9-14 mm at the distances and angles this robot
        works at -- a systematic bias toward the far side of the plate, every frame.

    Back-projecting the outline onto the plate's plane removes both: on the plane it is
    a circle again, with no foreshortening and no projective offset.
    """
    n, c = plane
    fx, fy, cx, cy = (intrinsics[k] for k in ("fx", "fy", "cx", "cy"))
    e = np.asarray(edge_px, dtype=float)
    dirs = np.stack([(e[:, 0] - cx) / fx, (e[:, 1] - cy) / fy, np.ones(len(e))], axis=1)
    denom = dirs @ n
    keep = np.abs(denom) > 1e-6
    if keep.sum() < 6:
        return None
    P = dirs[keep] * (c / denom[keep])[:, None]

    origin = P.mean(axis=0)
    e1 = np.array([1.0, 0.0, 0.0]) - n * n[0]
    if np.linalg.norm(e1) < 1e-6:
        e1 = np.array([0.0, 1.0, 0.0]) - n * n[1]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    Q = np.stack([(P - origin) @ e1, (P - origin) @ e2], axis=1)

    x, y = Q[:, 0], Q[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    sol = np.linalg.lstsq(A, -(x ** 2 + y ** 2), rcond=None)[0]
    a, b = -sol[0] / 2, -sol[1] / 2
    r2 = a * a + b * b - sol[2]
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    rms = float(np.sqrt(np.mean((np.hypot(x - a, y - b) - r) ** 2)))
    return origin + a * e1 + b * e2, r, rms


# --------------------------------------------------------------------------
# the detector
# --------------------------------------------------------------------------

MIN_STAND_M = 0.010   # a surface this far above the table is standing on it
MAX_STAND_M = 0.30    # taller than this is furniture or an arm, not a plate on a box
MIN_AXIS_RATIO = 0.45 # an ellipse flatter than this is not a plate seen from anywhere
MIN_COMPONENT_PX = 200


def detect_plate_ellipse(depth, intrinsics, known_diameter_m, depth_scale=1000.0,
                          stride=2, return_candidates=False):
    """Find the plate with no colour and no seed circle.

    Fit the table, take everything standing on it, and pick the blob whose ELLIPSE is
    the right size to be the plate. Nothing here can be fooled by paint, and nothing
    depends on a starting guess that might already be wrong.

    Returns a dict with cx, cy, major, minor, theta (pixels/radians), distance_m and
    diameter_m -- or raises RuntimeError.
    """
    depth = np.asarray(depth)
    plane = dominant_plane(depth, intrinsics, depth_scale)
    height = height_above_plane(depth, intrinsics, plane, depth_scale)

    sub = height[::stride, ::stride]
    standing = np.isfinite(sub) & (sub > MIN_STAND_M) & (sub < MAX_STAND_M)
    if standing.sum() < MIN_COMPONENT_PX // (stride * stride):
        raise RuntimeError("nothing is standing on the table in this frame")

    standing = fill_holes(standing)
    labels, n = label_mask(standing)
    fx = intrinsics["fx"]
    d_sub = np.asarray(depth)[::stride, ::stride].astype(np.float64) / depth_scale

    cands = []
    for k in range(1, n + 1):
        comp = labels == k
        if comp.sum() * stride * stride < MIN_COMPONENT_PX:
            continue
        edge = outline(comp)
        if len(edge) < 12:
            continue
        e = fit_ellipse(edge * stride)
        if e is None or e["major"] <= 0:
            continue
        ratio = e["minor"] / e["major"]
        if ratio < MIN_AXIS_RATIO:
            continue
        dvals = d_sub[comp]
        dvals = dvals[dvals > 0]
        if dvals.size < 10:
            continue
        dist = float(np.median(dvals))

        # The plate's own surface, not the table's: fitted from this blob's points, so a
        # plate on a box is measured in the plane it is actually lying in.
        ys, xs = np.nonzero(comp)
        zs = d_sub[comp]
        good = zs > 0
        pts3 = np.stack([(xs[good] * stride - intrinsics["cx"]) * zs[good] / fx,
                         (ys[good] * stride - intrinsics["cy"]) * zs[good] / intrinsics["fy"],
                         zs[good]], axis=1)
        top = pts3.mean(axis=0)
        nrm = np.linalg.svd(pts3 - top, full_matrices=False)[2][-1]
        fitted = circle_on_plane(edge * stride, intrinsics, (nrm, float(nrm @ top)))

        diam = 2.0 * fitted[1] if fitted else 2.0 * e["major"] * dist / fx
        e.update({"pixels": int(comp.sum() * stride * stride), "distance_m": dist,
                  "diameter_m": diam, "axis_ratio": ratio,
                  "centre_m": (fitted[0].tolist() if fitted else None),
                  "plane_rms_m": (fitted[2] if fitted else None),
                  "size_error": abs(diam - known_diameter_m) / known_diameter_m,
                  "residual_px": ellipse_residual(edge * stride, e)})
        cands.append(e)

    if not cands:
        raise RuntimeError("nothing on the table is plate-shaped in this frame")
    cands.sort(key=lambda e: e["size_error"])
    best = cands[0]
    if return_candidates:
        return best, cands
    return best
