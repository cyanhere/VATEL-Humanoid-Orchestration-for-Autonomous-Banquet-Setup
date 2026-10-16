"""A hall CCTV camera as geometry: from a pixel to a spot on the floor, and back.

    python cctv_camera.py cameras/cam_a.json              # solve it, print the fit
    python cctv_camera.py cameras/cam_a.json --overlay frame.png out.jpg

WHY A FULL CAMERA MODEL AND NOT FOUR CLICKED CORNERS
    The planner's built-in calibration is a four-point homography: exact for a pinhole
    camera looking at a flat floor, and fine for a demo. Real CCTV is neither:

    - the lenses are wide (~100 deg) and bend straight lines near the edges, which a
      homography cannot represent, so tables near the frame edge land a metre out;
    - a table's top is ~0.75 m above the floor, and its sides and legs reach the floor, so
      one plane is the wrong model for any of it;
    - a room may need two cameras, and their maps must agree.

    So each camera gets intrinsics (focal length, radial distortion) and a pose (where it
    hangs, which way it looks), solved once from a handful of landmarks whose position on
    the floor plan is known. After that any pixel can be put on ANY horizontal plane -- the
    floor, a tabletop, a chair seat -- and a table's footprint can be recovered properly.

LANDMARKS (the per-camera JSON)
    "points":  a pixel and the 3-D point it shows: a column's foot, a door frame's bottom.
    "vlines":  a pixel ANYWHERE on a vertical edge (a column corner, a door jamb) and that
               edge's x, y. No height needed -- which matters, because in a furnished room
               column feet are usually behind a table.
    Coordinates are metres in the venue's frame: x east, y south, height up.

WHAT IT CANNOT KNOW
    Only what the landmarks pin down. The fit's own residual is printed, and --overlay draws
    the plan back onto the frame: if the walls and columns do not sit on the walls and
    columns in the picture, the calibration is wrong, whatever the number says.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    sys.exit("cctv_camera.py needs OpenCV:  pip install opencv-python")


# ---------------------------------------------------------------- frames
# The venue frame is the planner's: x east, y south, both on the floor, with height UP.
# x-east, y-south, z-up is LEFT-handed, which no rotation matrix can express, so inside
# this file 3-D points are (x, y, -height): x east, y south, z DOWN, right-handed.
def w3(x, y, h=0.0):
    return np.array([x, y, -h], float)


class Camera:
    """Intrinsics + pose. Pixels in the camera's native frame (e.g. 2560 x 1440)."""

    def __init__(self, width, height, f, k1=0.0, k2=0.0, rvec=None, centre=None, name=""):
        self.w, self.h = int(width), int(height)
        self.f, self.k1, self.k2 = float(f), float(k1), float(k2)
        self.rvec = np.zeros(3) if rvec is None else np.asarray(rvec, float)
        self.C = np.zeros(3) if centre is None else np.asarray(centre, float)   # venue 3-D
        self.name = name

    # -------------------------------------------------------------- basics
    @property
    def K(self):
        return np.array([[self.f, 0, (self.w - 1) / 2], [0, self.f, (self.h - 1) / 2], [0, 0, 1]])

    @property
    def dist(self):
        return np.array([self.k1, self.k2, 0.0, 0.0, 0.0])

    @property
    def R(self):
        return cv2.Rodrigues(self.rvec)[0]

    @property
    def tvec(self):
        return -self.R @ self.C

    @property
    def height_m(self):
        return -self.C[2]

    @property
    def nadir(self):
        return self.C[:2].copy()

    def project(self, P):
        """(N, 3) venue points -> (N, 2) pixels, and a mask of the ones in front of the lens."""
        P = np.asarray(P, float).reshape(-1, 3)
        cam = (P - self.C) @ self.R.T
        front = cam[:, 2] > 0.05
        # Past the radius where r(1 + k1 r^2 + k2 r^4) stops growing, the lens model folds
        # back and a point far outside the view lands inside the frame. Call those unseen.
        with np.errstate(divide="ignore", invalid="ignore"):
            r2 = (cam[:, 0] ** 2 + cam[:, 1] ** 2) / cam[:, 2] ** 2
        front &= r2 < self.r2_max
        uv, _ = cv2.projectPoints(P.reshape(-1, 1, 3), self.rvec, self.tvec, self.K, self.dist)
        return uv.reshape(-1, 2), front

    def rays(self, uv):
        """Pixels -> unit ray directions in the venue frame."""
        uv = np.asarray(uv, float).reshape(-1, 1, 2)
        n = cv2.undistortPoints(uv, self.K, self.dist).reshape(-1, 2)
        d = np.column_stack([n, np.ones(len(n))]) @ self.R            # camera -> venue
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def on_plane(self, uv, height=0.0):
        """Pixels -> (N, 2) floor positions where each ray meets the plane `height` m up.
        NaN for rays that never come down to it (pointing at the ceiling)."""
        d = self.rays(uv)
        zt = -float(height)
        with np.errstate(divide="ignore", invalid="ignore"):
            s = (zt - self.C[2]) / d[:, 2]
        P = self.C + s[:, None] * d
        P[~(s > 0)] = np.nan
        return P[:, :2]

    @property
    def r2_max(self):
        """Largest squared normalised radius at which the distortion is still monotonic."""
        rr = np.linspace(0.0, 4.0, 4001) ** 2
        grow = 1 + 3 * self.k1 * rr + 5 * self.k2 * rr ** 2
        bad = np.nonzero(grow <= 0.05)[0]
        return float(rr[bad[0]]) if len(bad) else 16.0

    # -------------------------------------------------------------- persistence
    def to_dict(self):
        return {"width": self.w, "height": self.h, "f": self.f, "k1": self.k1, "k2": self.k2,
                "rvec": self.rvec.tolist(), "centre_m": [float(self.C[0]), float(self.C[1])],
                "height_m": float(self.height_m)}

    @classmethod
    def from_dict(cls, d, name=""):
        c = d.get("centre_m", [0, 0])
        return cls(d["width"], d["height"], d["f"], d.get("k1", 0), d.get("k2", 0), d["rvec"],
                   w3(c[0], c[1], d.get("height_m", 3.0)), name=name)


# ---------------------------------------------------------------- solving a camera
def look_at(C, target):
    """rvec for a camera at C looking at `target`, with the image's up roughly up."""
    f = np.asarray(target, float) - C
    f /= np.linalg.norm(f)
    down = np.array([0.0, 0.0, 1.0])                         # venue z is DOWN
    x = np.cross(down, f)                                    # image right
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    x /= np.linalg.norm(x)
    y = np.cross(f, x)                                       # image down
    R = np.stack([x, y, f])                                  # rows: camera axes in venue
    return cv2.Rodrigues(R)[0].ravel()


def _residuals(params, cam, points, vlines, priors, fix_k2, rects=(), verticals=()):
    f, k1, k2 = params[0], params[1], (0.0 if fix_k2 else params[2])
    o = 2 if fix_k2 else 3
    cam.f, cam.k1, cam.k2 = f, k1, k2
    cam.rvec = params[o:o + 3]
    cam.C = params[o + 3:o + 6]
    res = []
    if rects:
        # Tabletops as a calibration target. Four corners of one tabletop, put on the plane
        # of the tops, must make a RECTANGLE: opposite sides equal, diagonals equal. That says
        # nothing about where the table is, only about how the floor is tilted and how the
        # lens bends -- which is exactly what a few scattered landmarks leave loose.
        h_tab = params[o + 6]
        for rc in rects:
            P = cam.on_plane(rc["px"], h_tab)
            if not np.isfinite(P).all():                 # keep the residual count fixed
                res.append([500.0] * (3 + bool(rc.get("square")) + 4 * bool(rc.get("size_m"))))
                continue
            A, B, C_, D = P
            ab, bc, cd, da = (np.linalg.norm(B - A), np.linalg.norm(C_ - B),
                              np.linalg.norm(D - C_), np.linalg.norm(A - D))
            # Shape misfit relative to the table's size, so shrinking every table to a dot
            # (tops at the camera's own height) earns nothing. 1 % misfit ~ 3 px.
            k = 300.0 * rc.get("w", 1.0) / max((ab + bc + cd + da) / 4, 1e-3)
            res.append([k * (ab - cd), k * (bc - da),
                        k * (np.linalg.norm(C_ - A) - np.linalg.norm(D - B))])
            if rc.get("square"):
                res.append([k * (ab + cd - bc - da) / 2])
            if rc.get("size_m"):                         # a table of known size fixes scale
                sx, sy = rc["size_m"]
                res.append([k * (ab - sx), k * (cd - sx), k * (bc - sy), k * (da - sy)])
        hp, hsd = priors.get("table_h_m", (0.75, 0.15))
        res.append([(h_tab - hp) / hsd * 2.0,
                    200.0 * max(0.0, h_tab - (-cam.C[2] - 1.0)),   # tops well below the lens
                    200.0 * max(0.0, 0.3 - h_tab)])
    if len(verticals):
        # Any edge that is plumb in the room -- a door frame, a column corner, a shelf
        # upright -- with no idea where it is. Its two pixels and the lens centre span a
        # plane, which must contain the vertical. Pins pitch, roll and the lens bend.
        d = cam.rays(np.array([vt["px"] for vt in verticals], float).reshape(-1, 2))
        n = np.cross(d[0::2], d[1::2])
        n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
        res.append(n[:, 2] * cam.f * np.array([vt.get("w", 1.0) for vt in verticals]))
    if points:
        P = np.array([p["P"] for p in points])
        uv, front = cam.project(P)
        obs = np.array([p["px"] for p in points])
        r = (uv - obs) * np.array([p.get("w", 1.0) for p in points])[:, None]
        r[~front] = 500.0
        res.append(r.ravel())
    for vl in vlines:
        A, B = vl["seg"]
        ts = np.linspace(0.0, 1.0, 60)
        uv, front = cam.project(A[None, :] + ts[:, None] * (B - A)[None, :])
        uv = uv[front]
        if len(uv) < 2:
            res.append([500.0])
            continue
        q = np.asarray(vl["px"], float)
        a, b = uv[:-1], uv[1:]
        ab = b - a
        t = np.clip(((q - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
        dist = np.linalg.norm(a + t[:, None] * ab - q, axis=1).min()
        res.append([dist * vl.get("w", 1.0)])
    # soft priors keep an under-constrained fit physical, not the fit's answer
    pr = []
    hlo, hhi = priors.get("height_m", (2.2, 6.0))
    H = -cam.C[2]
    pr.append(20.0 * max(0.0, hlo - H) + 20.0 * max(0.0, H - hhi))
    pr.append(40.0 * max(0.0, abs(k1) - 0.6) + 40.0 * max(0.0, k1 - 0.05))
    if "centre_m" in priors:
        cx, cy, sd = priors["centre_m"]
        pr.append((cam.C[0] - cx) / sd * 2.0)
        pr.append((cam.C[1] - cy) / sd * 2.0)
    if "f" in priors:
        fp, sd = priors["f"]
        pr.append((f - fp) / sd * 2.0)
    res.append(pr)
    return np.concatenate([np.ravel(r) for r in res])


def solve(cfg, verbose=True, start=None):
    """Fit a camera to a calibration config. Returns (Camera, report dict).
    `start`: a Camera to refine from, instead of the multi-start search."""
    from scipy.optimize import least_squares

    W, H = cfg["frame"]
    init = cfg.get("init", {})
    C0 = w3(*init.get("centre_m", (0.0, 0.0)), init.get("height_m", 3.2))
    target = w3(*init.get("look_at_m", (5.0, 5.0)), 0.0)
    f0 = init.get("f", W / 2 / math.tan(math.radians(init.get("hfov_deg", 100) / 2)))
    cam = Camera(W, H, f0, init.get("k1", -0.15), 0.0, look_at(C0, target), C0, cfg.get("name", ""))

    points = [{"P": w3(*p["m"], p.get("h", 0.0)), "px": p["px"], "w": p.get("w", 1.0),
               "label": p.get("label", "")} for p in cfg.get("points", [])]
    # A vertical edge is a segment from the floor up; a "line" is any straight edge with a
    # known 3-D position -- a counter's front edge, a wall's foot -- observed at one pixel.
    vlines = [{"seg": (w3(*v["m"], v.get("h_from", 0.0)), w3(*v["m"], v.get("h_to", 3.2))),
               "px": v["px"], "w": v.get("w", 1.0), "label": v.get("label", "")}
              for v in cfg.get("vlines", [])]
    vlines += [{"seg": (w3(*l["from"]), w3(*l["to"])), "px": l["px"], "w": l.get("w", 1.0),
                "label": l.get("label", "")} for l in cfg.get("lines", [])]
    priors = cfg.get("priors", {})
    # A tabletop's four corners, in order round the edge. "square": the sides are equal;
    # "size_m": [first side, second side] when the table's size is known.
    rects = [{"px": r["px"], "w": r.get("w", 1.0), "label": r.get("label", ""),
              "square": r.get("square", False), "size_m": r.get("size_m")}
             for r in cfg.get("rects", [])]
    verticals = [{"px": v["px"], "w": v.get("w", 1.0), "label": v.get("label", "")}
                 for v in cfg.get("verticals", [])]
    fix_k2 = not cfg.get("solve_k2", False)
    p0 = [cam.f, cam.k1] + ([] if fix_k2 else [0.0]) + list(cam.rvec) + list(cam.C)
    if rects:
        p0.append(priors.get("table_h_m", (0.75, 0.15))[0])
    o0 = 2 if fix_k2 else 3
    best = None
    # Several starts. Focal length and radial distortion fold into each other on a wide
    # lens: started at the wrong field of view, the fit will bend the lens the wrong way to
    # make the scene fit (the hub's SB-3 camera did exactly that from a 100-deg guess, landing
    # on k1 = +0.6 at 25 px rms). CCTV lenses run from ~2.8 mm (~105 deg) to ~6 mm
    # (~55 deg), so try that range and keep the best.
    hfovs = init.get("hfov_try", [55, 70, 85, 100, 110])
    starts = []
    for hf in hfovs:
        for k1s in (-0.25, -0.05):
            q = list(p0)
            q[0] = W / 2 / math.tan(math.radians(hf) / 2)
            q[1] = k1s
            q[o0:o0 + 3] = list(look_at(C0, target))
            starts.append(q)
    if start is not None:
        q = list(p0)
        q[0], q[1] = start.f, start.k1
        q[o0:o0 + 6] = list(start.rvec) + list(start.C)
        starts = [q]
    for q in starts:
        r = least_squares(_residuals, q, args=(cam, points, vlines, priors, fix_k2, rects, verticals),
                          loss="soft_l1", f_scale=8.0, max_nfev=4000, x_scale="jac")
        if best is None or r.cost < best.cost:
            best = r
    _residuals(best.x, cam, points, vlines, priors, fix_k2, rects, verticals)  # leave cam here
    table_h = float(best.x[o0 + 6]) if rects else None

    rep = {"points": [], "vlines": []}
    if points:
        uv, _ = cam.project(np.array([p["P"] for p in points]))
        for p, q in zip(points, uv):
            rep["points"].append((p["label"], float(np.linalg.norm(q - np.array(p["px"])))))
    for vl in vlines:
        r = _residuals(best.x, cam, [], [vl], {}, fix_k2)[0]
        rep["vlines"].append((vl["label"], float(r / vl["w"])))
    rep["verticals"] = []
    for vt in verticals:
        d = cam.rays(vt["px"])
        n = np.cross(d[0], d[1])
        rep["verticals"].append((vt["label"], float(math.degrees(math.asin(
            abs(n[2]) / max(np.linalg.norm(n), 1e-12))))))
    if rects:
        rep["table_h_m"] = table_h
        rep["rects"] = []
        for rc in rects:
            A, B, C_, D = cam.on_plane(rc["px"], table_h)
            sides = [np.linalg.norm(B - A), np.linalg.norm(C_ - B), np.linalg.norm(D - C_),
                     np.linalg.norm(A - D)]
            rep["rects"].append((rc["label"], sides, float(abs(np.linalg.norm(C_ - A) -
                                                                np.linalg.norm(D - B)))))
    errs = [e for _, e in rep["points"]] + [e for _, e in rep["vlines"]]
    rep["rms_px"] = float(np.sqrt(np.mean(np.square(errs)))) if errs else float("nan")
    if verbose:
        print("%s: f %.0f px (hfov %.0f deg), k1 %+.3f, at (%.2f, %.2f) m, %.2f m up, "
              "rms %.1f px over %d landmarks"
              % (cam.name or "camera", cam.f, math.degrees(2 * math.atan(W / 2 / cam.f)), cam.k1,
                 cam.C[0], cam.C[1], cam.height_m, rep["rms_px"], len(errs)))
        for lab, e in rep["points"]:
            print("   point  %-28s %6.1f px" % (lab, e))
        for lab, e in rep["vlines"]:
            print("   vline  %-28s %6.1f px" % (lab, e))
        if rep["verticals"]:
            v = [e for _, e in rep["verticals"]]
            print("   verticals: %d edges, lean rms %.2f deg, worst %.2f deg (%s)"
                  % (len(v), float(np.sqrt(np.mean(np.square(v)))), max(v),
                     rep["verticals"][int(np.argmax(v))][0]))
        for lab, sides, dd in rep.get("rects", []):
            print("   rect   %-28s sides %s m, diagonals differ %.2f m"
                  % (lab, " / ".join("%.2f" % x for x in sides), dd))
        if rects:
            print("   tabletop height %.2f m" % table_h)
    return cam, rep


# ---------------------------------------------------------------- footprints
def _clip(subject, clipper):
    """Sutherland-Hodgman: convex polygon `subject` clipped by convex polygon `clipper`."""
    def inside(p, a, b):
        return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0

    def cross(p1, p2, a, b):
        dc = a - b
        dp = p1 - p2
        n1 = a[0] * b[1] - a[1] * b[0]
        n2 = p1[0] * p2[1] - p1[1] * p2[0]
        den = dc[0] * dp[1] - dc[1] * dp[0]
        if abs(den) < 1e-12:
            return p2
        return np.array([(n1 * dp[0] - n2 * dc[0]) / den, (n1 * dp[1] - n2 * dc[1]) / den])

    out = [np.asarray(p, float) for p in subject]
    clip = [np.asarray(p, float) for p in clipper]
    # both counter-clockwise
    if _area(np.array(clip)) < 0:
        clip = clip[::-1]
    for i in range(len(clip)):
        a, b = clip[i], clip[(i + 1) % len(clip)]
        inp, out = out, []
        if not inp:
            break
        s = inp[-1]
        for e in inp:
            if inside(e, a, b):
                if not inside(s, a, b):
                    out.append(cross(s, e, a, b))
                out.append(e)
            elif inside(s, a, b):
                out.append(cross(s, e, a, b))
            s = e
    return np.array(out)


def _area(poly):
    p = np.asarray(poly, float)
    if len(p) < 3:
        return 0.0
    x, y = p[:, 0], p[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y))


def _hull(pts):
    pts = np.asarray(pts, float)
    pts = pts[np.isfinite(pts).all(1)]
    if len(pts) < 3:
        return pts
    h = cv2.convexHull(pts.astype(np.float32)).reshape(-1, 2).astype(float)
    return h if _area(h) > 0 else h[::-1]


def far_envelope(cam, outline_px, top_h, bin_deg=0.3, border_px=2.0):
    """The part of an object's outline that is the true edge of its top.

    Put every outline pixel on the plane of the top. The top's own edge lands where it is.
    Everything lower -- the legs, a skirt, a pedestal, a chair in front -- lands SHORT, on
    the line from where it is back toward the point under the camera: a ray reaches the
    top's height before it reaches them. So seen from the point under the camera, in every
    direction the FARTHEST outline point is the top's edge, and nothing below the top can
    ever push past it. That holds for any table on any legs, which is why this is used and
    not the legs or the whole mask: those depend on what the table stands on.

    Except where the outline runs along the edge of the frame: that is where the picture
    stops, not where the table does, and past it the camera knows nothing. A direction
    whose farthest point is on the frame's edge is dropped altogether -- not answered with
    the next point in, which would be a chair or a leg standing in for the hidden top.

    Returns the (N, 2) far points in angular order around the camera's nadir."""
    uv = np.asarray(outline_px, float).reshape(-1, 2)
    on_edge = ((uv[:, 0] <= border_px) | (uv[:, 1] <= border_px) |
               (uv[:, 0] >= cam.w - 1 - border_px) | (uv[:, 1] >= cam.h - 1 - border_px))
    P = cam.on_plane(uv, top_h)
    ok = np.isfinite(P).all(1)
    P, on_edge = P[ok], on_edge[ok]
    if len(P) < 3:
        return P
    d = P - cam.nadir
    a = np.arctan2(d[:, 1], d[:, 0])
    m = math.atan2(np.sin(a).mean(), np.cos(a).mean())
    a = (np.degrees(a - m) + 180.0) % 360.0      # one object's span, away from the wrap-around
    r = np.hypot(d[:, 0], d[:, 1])
    b = np.floor(a / bin_deg).astype(int)
    order = np.lexsort((-r, b))                  # by bin, farthest first within each
    first = np.ones(len(order), bool)
    first[1:] = b[order][1:] != b[order][:-1]
    keep = order[first]
    return P[keep[~on_edge[keep]]]


def _edge_angle(E, step=3):
    """Dominant edge direction of a chain of points, mod 90 deg (a rectangle's two axes),
    weighting each piece by its length. Radians."""
    if len(E) <= step:
        return 0.0
    d = E[step:] - E[:-step]
    L = np.hypot(d[:, 0], d[:, 1])
    ok = L > 1e-3
    if not ok.any():
        return 0.0
    t = np.degrees(np.arctan2(d[ok, 1], d[ok, 0])) % 90.0
    L = L[ok]
    hist = np.zeros(90)
    np.add.at(hist, t.astype(int) % 90, L)
    kern = np.exp(-0.5 * (np.arange(-6, 7) / 2.5) ** 2)
    sm = np.array([np.sum(hist[(np.arange(-6, 7) + j) % 90] * kern) for j in range(90)])
    j = int(np.argmax(sm))
    off = (t - j + 45.0) % 90.0 - 45.0
    near = np.abs(off) <= 6
    return math.radians(j + float(np.sum(off[near] * L[near]) / max(float(np.sum(L[near])), 1e-9)))


def fit_top(cam, outline_px, top_h, axes_deg=None, snap_deg=12.0):
    """An object's outline in one camera -> its top as a rectangle or a circle, in metres.

    Fitted to the far envelope only (see far_envelope), so the result is the TOP, not the
    top plus its legs smeared toward the camera. A rectangle: the envelope's dominant edge
    direction, then its extent along that and the perpendicular -- the envelope always
    holds the far corner and the two corners the camera sees past, so those extents are
    the whole table. A circle: a fit to the far arc. Whichever explains the envelope better.

    `axes_deg`: the room's wall directions. Rectangular tables are set square to the walls
    far more often than not, so an angle within `snap_deg` of one is taken as square to it.
    """
    E = far_envelope(cam, outline_px, top_h)
    if len(E) < 6:
        return None
    t = _edge_angle(E)
    if axes_deg is not None:
        for ax in np.atleast_1d(axes_deg):
            dd = (math.degrees(t) - ax + 45.0) % 90.0 - 45.0
            if abs(dd) <= snap_deg:
                t -= math.radians(dd)
                break
    R = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
    Q = E @ R.T
    (u0, v0), (u1, v1) = Q.min(0), Q.max(0)
    corners = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]]) @ R
    # how well a rectangle explains the far points: distance to its nearest side
    du = np.minimum(Q[:, 0] - u0, u1 - Q[:, 0])
    dv = np.minimum(Q[:, 1] - v0, v1 - Q[:, 1])
    rect_err = float(np.mean(np.minimum(du, dv)))
    # ... and a circle (Kasa fit)
    A = np.column_stack([2 * E, np.ones(len(E))])
    sol, *_ = np.linalg.lstsq(A, (E ** 2).sum(1), rcond=None)
    cxy, rr = sol[:2], math.sqrt(max(sol[2] + sol[:2] @ sol[:2], 0.0))
    circ_err = float(np.mean(np.abs(np.hypot(*(E - cxy).T) - rr)))
    ang = np.sort(np.degrees(np.arctan2(E[:, 1] - cxy[1], E[:, 0] - cxy[0])))
    span = 360.0 - float(np.max(np.diff(np.concatenate([ang, ang[:1] + 360.0]))))
    w, h = u1 - u0, v1 - v0
    out = {"far_points": int(len(E)), "rect_err_m": rect_err, "circle_err_m": circ_err}
    # Round only when the far points really are an arc: a ragged outline fits a circle
    # about as badly as a rectangle, and a straight far edge fits a huge one.
    if (0.3 < rr < 1.3 and span > 120 and circ_err < 0.035 and circ_err < 0.5 * rect_err
            and min(w, h) > 0.7 * max(w, h)):
        out.update(shape="round", centre_m=[float(cxy[0]), float(cxy[1])],
                   diameter_m=float(2 * rr))
        return out
    rot = math.degrees(t)
    if w < h:                                     # report the long side first
        w, h, rot = h, w, rot + 90.0
    rot = (rot + 90.0) % 180.0 - 90.0
    c = corners.mean(0)
    out.update(shape="rect", centre_m=[float(c[0]), float(c[1])], size_m=[float(w), float(h)],
               rotation_deg=float(rot), corners_m=corners.tolist())
    return out


# ---------------------------------------------------------------- plumb edges, found
def find_verticals(img, cam=None, tol_deg=4.0, min_len_frac=0.035, max_tilt_deg=25.0):
    """Straight edges in a clean frame that are (probably) plumb in the room.

    Every venue is full of them -- door frames, columns, cabinet doors, table pedestals --
    and together they say how the camera is tilted and how the lens bends, with nothing
    measured. Without a camera: long segments within `max_tilt_deg` of upright in the
    image. With one: segments whose lean under that camera is under `tol_deg`, which
    throws out the floor lines that merely look upright. Re-solve and call again with a
    tighter tolerance to converge.
    """
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    segs = cv2.createLineSegmentDetector().detect(g)[0]
    if segs is None:
        return []
    segs = segs.reshape(-1, 4)
    L = np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1])
    tilt = np.degrees(np.arctan2(np.abs(segs[:, 2] - segs[:, 0]), np.abs(segs[:, 3] - segs[:, 1])))
    keep = (L >= min_len_frac * g.shape[0]) & (tilt <= max_tilt_deg)
    out = []
    for x0, y0, x1, y1 in segs[keep]:
        px = [[float(x0), float(y0)], [float(x1), float(y1)]]
        if cam is not None:
            d = cam.rays(px)
            n = np.cross(d[0], d[1])
            if math.degrees(math.asin(min(1.0, abs(n[2]) / max(np.linalg.norm(n), 1e-12)))) > tol_deg:
                continue
        out.append({"px": px, "label": "auto"})
    return out


# ---------------------------------------------------------------- overlay
def draw_plan(img, cam, segments, colour=(0, 0, 255), heights=(0.0,), thick=2):
    """Draw floor-plan line segments [(x0, y0, x1, y1), ...] at the given heights."""
    for h in heights:
        for x0, y0, x1, y1 in segments:
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.1))
            P = np.array([w3(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, h)
                          for t in np.linspace(0, 1, n)])
            uv, front = cam.project(P)
            ok = front & np.isfinite(uv).all(1) & (np.abs(uv) < 1e5).all(1)
            pts = uv[ok].astype(np.int32)
            if len(pts) >= 2:
                cv2.polylines(img, [pts], False, colour, thick, cv2.LINE_AA)
    return img


def load_config(path):
    with open(path) as f:
        cfg = json.load(f)
    cfg.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    return cfg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("config")
    ap.add_argument("--overlay", nargs=2, metavar=("FRAME", "OUT"))
    ap.add_argument("--save", action="store_true", help="write the solved camera into the config")
    ap.add_argument("--auto-verticals", metavar="FRAME",
                    help="also use the plumb edges found in this clean frame (doors, columns...)")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    auto = []
    if args.auto_verticals:
        frame = cv2.imread(args.auto_verticals)
        manual = list(cfg.get("verticals", []))
        # The landmarks alone first, then only the edges that camera calls plumb, tighter
        # each pass: each one throws out the floor lines and chair legs the last let in.
        # (or from the last saved fit, which is usually closer)
        cam, rep = solve(cfg, verbose=False)
        if "solved" in cfg:
            cam = Camera.from_dict(cfg["solved"], cfg.get("name", ""))
        for tol in (4.0, 2.0, 1.0):
            auto = find_verticals(frame, cam, tol)
            auto = sorted(auto, key=lambda v: -np.hypot(*np.subtract(*v["px"])))[:200]
            for v in auto:
                v["w"] = 0.2                                # ~5 px per degree of lean, each
            cam, rep = solve(dict(cfg, verticals=manual + auto), verbose=False, start=cam)
        print("auto verticals: %d plumb edges used" % len(auto))
        cam, rep = solve(dict(cfg, verticals=manual + auto), start=cam)
    else:
        cam, rep = solve(cfg)
    if args.save:
        cfg["solved"] = cam.to_dict()
        cfg["solved"]["rms_px"] = rep["rms_px"]
        if rep.get("table_h_m") is not None:
            cfg["solved"]["table_h_m"] = rep["table_h_m"]
        if auto:
            cfg["solved"]["auto_verticals"] = len(auto)
        with open(args.config, "w") as f:
            json.dump(cfg, f, indent=2)
        print("saved the solved camera into", args.config)
    if args.overlay:
        img = cv2.imread(args.overlay[0])
        segs = [tuple(s) for s in cfg.get("plan_segments", [])]
        draw_plan(img, cam, segs, (0, 0, 255), (0.0,), 3)
        for c in cfg.get("plan_columns", []):
            x0, y0, x1, y1 = c
            box = [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]
            draw_plan(img, cam, box, (0, 255, 255), (0.0, 2.5), 2)
            for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                draw_plan(img, cam, [(x, y, x, y)], (0, 255, 255), (0.0,), 2)
        for p in cfg.get("points", []):
            q, _ = cam.project(w3(*p["m"], p.get("h", 0.0)))
            cv2.circle(img, tuple(np.int32(p["px"])), 9, (0, 255, 0), 2)
            cv2.line(img, tuple(np.int32(p["px"])), tuple(np.int32(q[0])), (255, 0, 255), 2)
        segs = [(w3(*v["m"], 0.0), w3(*v["m"], 3.2), v["px"]) for v in cfg.get("vlines", [])]
        segs += [(w3(*l["from"]), w3(*l["to"]), l["px"]) for l in cfg.get("lines", [])]
        for A, B, px in segs:
            cv2.drawMarker(img, tuple(np.int32(px)), (0, 255, 0), cv2.MARKER_CROSS, 22, 2)
            P = A[None, :] + np.linspace(0, 1, 30)[:, None] * (B - A)[None, :]
            uv, fr = cam.project(P)
            cv2.polylines(img, [uv[fr].astype(np.int32)], False, (255, 0, 255), 2, cv2.LINE_AA)
        for v in auto:
            cv2.line(img, tuple(np.int32(v["px"][0])), tuple(np.int32(v["px"][1])),
                     (255, 255, 0), 2, cv2.LINE_AA)
        cv2.imwrite(args.overlay[1], img, [cv2.IMWRITE_JPEG_QUALITY, 88])
        print("wrote", args.overlay[1])


if __name__ == "__main__":
    main()
