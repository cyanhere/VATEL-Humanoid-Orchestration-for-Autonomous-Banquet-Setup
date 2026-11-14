"""Fast tracking of the top plate's rim between full detections. Numpy only."""
import math

import numpy as np

PLATE_R = 0.1265                 # 253 mm diameter


def deproject(depth, intr, scale, roi, stride=2):
    """Points (N x 3, optical frame of `intr`) of a depth crop roi=(u0, u1, v0, v1)."""
    u0, u1, v0, v1 = roi
    d = depth[v0:v1:stride, u0:u1:stride].astype(np.float64) / scale
    if d.size == 0:
        return np.zeros((0, 3))
    vv, uu = np.mgrid[v0:v1:stride, u0:u1:stride]
    ok = d > 0
    z = d[ok]
    x = (uu[ok] - intr["cx"]) * z / intr["fx"]
    y = (vv[ok] - intr["cy"]) * z / intr["fy"]
    return np.stack([x, y, z], axis=1)


def project(P_o, intr):
    z = np.maximum(P_o[:, 2], 1e-6)
    return np.stack([intr["fx"] * P_o[:, 0] / z + intr["cx"], intr["fy"] * P_o[:, 1] / z + intr["cy"]], axis=1)


def fit_circle_known_r(xy, c0, r, iters=8, huber=0.003):
    """Robust centre of a circle of radius r through points xy (N x 2), from c0."""
    c = np.array(c0, float)
    w = np.ones(len(xy))
    for _ in range(iters):
        d = xy - c
        dist = np.sqrt((d ** 2).sum(axis=1)) + 1e-9
        e = dist - r
        a = np.abs(e)
        w = np.where(a <= huber, 1.0, huber / a)
        J = -d / dist[:, None]
        H = (J * w[:, None]).T @ J + 1e-9 * np.eye(2)
        g = (J * w[:, None]).T @ e
        step = -np.linalg.solve(H, g)
        c = c + step
        if np.linalg.norm(step) < 1e-6:
            break
    d = xy - c
    e = np.sqrt((d ** 2).sum(axis=1)) - r
    return c, e


def sector_angles(n_bins=72):
    return (np.arange(n_bins) + 0.5) / n_bins * 2 * math.pi - math.pi


def excluded(ang, exclude):
    """Which angles (rad, around the plate's centre) fall in one of the excluded (centre, half width) ranges."""
    out = np.zeros(np.shape(ang), bool)
    for th, half in exclude:
        out |= np.abs((np.asarray(ang) - th + math.pi) % (2 * math.pi) - math.pi) < half
    return out


def edge_points(Q, c, n_bins=72, pct=0.92, min_pts=4, exclude=()):
    """The rim's OUTER edge: in each angular sector around c, the radius below which `pct` of the rim-top points lie.
    A wide flat rim fills an annulus 20-30 mm wide; its outer boundary is sharp, its middle is not a circle to fit.
    Sectors in `exclude` ((centre angle, half width) rad) are left out: where the robot's hand is."""
    d = Q[:, :2] - c
    r = np.sqrt((d ** 2).sum(axis=1))
    th = np.arctan2(d[:, 1], d[:, 0])
    b = np.minimum(((th + math.pi) / (2 * math.pi) * n_bins).astype(int), n_bins - 1)
    order = np.lexsort((r, b))
    bs, rs = b[order], r[order]
    counts = np.bincount(bs, minlength=n_bins)
    starts = np.r_[0, np.cumsum(counts)[:-1]]
    good = counts >= min_pts
    if len(exclude):
        good &= ~excluded(sector_angles(n_bins), exclude)
    idx = starts[good] + np.minimum((counts[good] * pct).astype(int), counts[good] - 1)
    r_e = rs[idx]
    ang = (np.nonzero(good)[0] + 0.5) / n_bins * 2 * math.pi - math.pi
    return c + np.stack([r_e * np.cos(ang), r_e * np.sin(ang)], axis=1), float(good.mean())


def fit_circle_free(xy):
    """Algebraic (Kasa) circle: centre, radius."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x ** 2 + y ** 2
    s, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = np.array([s[0] / 2, s[1] / 2])
    return c, math.sqrt(max(s[2] + c @ c, 1e-12))


N_SEC = 72


def sector_heights(P, c, R, rim_z, excl, band=(-0.015, 0.025), ring=(0.012, 0.003), min_pts=6):
    """Rim-top height per 5 deg sector: the median z of the points on the flat rim top (R - ring[0].. R + ring[1] from c,
    R the rim's outer edge) within band of the tracked rim height. NaN where too few points or a hand is there."""
    d = P[:, :2] - c
    rr = np.hypot(d[:, 0], d[:, 1])
    dz = P[:, 2] - rim_z
    m = (rr > R - ring[0]) & (rr < R + ring[1]) & (dz > band[0]) & (dz < band[1])
    Q = P[m]
    ang = np.arctan2(Q[:, 1] - c[1], Q[:, 0] - c[0])
    if excl:
        keep = ~excluded(ang, excl)
        Q, ang = Q[keep], ang[keep]
    k = ((ang + math.pi) / (2 * math.pi) * N_SEC).astype(int) % N_SEC
    z = np.full(N_SEC, np.nan)
    n = np.bincount(k, minlength=N_SEC)
    if len(Q):
        order = np.argsort(k, kind="stable")
        ks, zs = k[order], Q[order, 2]
        starts = np.searchsorted(ks, np.arange(N_SEC))
        ends = np.searchsorted(ks, np.arange(N_SEC), side="right")
        for i in range(N_SEC):
            if ends[i] - starts[i] >= min_pts:
                z[i] = np.median(zs[starts[i]:ends[i]])
    return z, n


def fit_plane(z, R, huber=0.002, iters=10):
    """z_i = z0 + gx R cos a_i + gy R sin a_i over the sectors that have a height (Huber IRLS). (z0, gx, gy, rms m) or None.
    The rim height at angle a (from the centre, pelvis frame): z0 + R (gx cos a + gy sin a)."""
    a = -math.pi + (np.arange(N_SEC) + 0.5) * 2 * math.pi / N_SEC
    ok = np.isfinite(z)
    if ok.sum() < 12:
        return None
    A = np.stack([np.ones(ok.sum()), R * np.cos(a[ok]), R * np.sin(a[ok])], axis=1)
    b = z[ok]
    w = np.ones(len(b))
    for _ in range(iters):
        sw = np.sqrt(w)
        p = np.linalg.lstsq(A * sw[:, None], b * sw, rcond=None)[0]
        e = np.abs(A @ p - b)
        w = np.where(e < huber, 1.0, huber / np.maximum(e, 1e-9))
    e = A @ p - b
    rms = float(np.sqrt(np.mean(e[w > 0.5] ** 2))) if (w > 0.5).any() else float("nan")
    return float(p[0]), float(p[1]), float(p[2]), rms


class RimTracker(object):
    def __init__(self, center_xy, rim_z, table_z, plate_r=PLATE_R, band_z=0.004, band_r=0.018, stride=2, rise_z=0.025):
        self.c = np.array(center_xy[:2], float)
        self.rim_z = float(rim_z)
        self.rim_z0 = float(rim_z)
        self.table_z = float(table_z)
        self.R = plate_r
        self.r_band = plate_r - 0.002          # radius of the rim-top's outer edge; refined by init()
        self.band_z, self.band_r = band_z, band_r
        self.rise_z = rise_z                    # points up to this far ABOVE the rim count: a pushed plate rises
        self.stride = stride
        self.n_ok = 0
        self.last = None
        self.start = self.c.copy()
        self.dbg = {}                           # the last frame's rim points and edge points, for the live view

    # ------------------------------------------------------------------ geometry
    def roi(self, intr, T_po, margin=0.035, T_cd=None):
        ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        P = np.stack([self.c[0] + (self.R + margin) * np.cos(ang), self.c[1] + (self.R + margin) * np.sin(ang),
                      np.full(24, self.rim_z)], axis=1)
        R, t = T_po[:3, :3], T_po[:3, 3]
        P_o = (P - t) @ R                       # pelvis -> (colour) optical
        if T_cd is not None:                    # -> the depth camera, whose intrinsics `intr` are
            P_o = (P_o - T_cd[:3, 3]) @ T_cd[:3, :3]
        if np.any(P_o[:, 2] < 0.05):
            return None
        uv = project(P_o, intr)
        W, H = int(intr["width"]), int(intr["height"])
        u0, v0 = int(max(0, uv[:, 0].min())), int(max(0, uv[:, 1].min()))
        u1, v1 = int(min(W, uv[:, 0].max() + 1)), int(min(H, uv[:, 1].max() + 1))
        if u1 - u0 < 20 or v1 - v0 < 20:
            return None
        clipped = uv[:, 0].min() < 0 or uv[:, 1].min() < 0 or uv[:, 0].max() > W or uv[:, 1].max() > H
        return (u0, u1, v0, v1), clipped

    def points(self, depth, intr, scale, T_po, spheres=(), T_cd=None):
        r = self.roi(intr, T_po, T_cd=T_cd)
        if r is None:
            return None, None
        roi, clipped = r
        P = deproject(depth, intr, scale, roi, self.stride)
        if T_cd is not None:                    # depth optical -> colour optical (the calibrated frame)
            P = P @ T_cd[:3, :3].T + T_cd[:3, 3]
        P = P @ T_po[:3, :3].T + T_po[:3, 3]    # -> pelvis
        keep = np.ones(len(P), bool)
        for c, rad in spheres:
            keep &= ((P - np.asarray(c)) ** 2).sum(axis=1) > rad * rad
        return P[keep], clipped

    def hand_sectors(self, spheres, near=0.06, margin=0.02):
        """(angle, half width) around the plate's centre of every hand / finger sphere near the rim: those sectors are
        not used. the curled little finger at the rim's height just outside its edge passed the height and
        annulus filters and pulled the tracked circle 5-9 mm toward the hand, against the push."""
        out = []
        for c, r in spheres or ():
            c = np.asarray(c, float)
            d = c[:2] - self.c
            dist = float(np.linalg.norm(d))
            if abs(c[2] - self.rim_z) > near + r or abs(dist - self.R) > near + r:
                continue
            half = math.asin(min(1.0, (r + margin) / max(dist, r + margin)))
            out.append((math.atan2(d[1], d[0]), half))
        return out

    # ------------------------------------------------------------------ the estimate
    def update(self, depth, intr, scale, T_po, spheres=(), T_cd=None, refine_radius=False):
        self.dbg = {}
        P, clipped = self.points(depth, intr, scale, T_po, spheres, T_cd)
        out = dict(ok=False, center=[float(self.c[0]), float(self.c[1]), self.rim_z], n=0, coverage=0.0, rms_mm=None,
                   moved_mm=float(np.linalg.norm(self.c - self.start) * 1000), clipped=bool(clipped) if clipped is not None else None)
        if P is None or len(P) < 50:
            out["why"] = "plate out of view" if P is None else "no depth on the rim"
            return out
        # the top rim: from band_z below the rim height to rise_z above it (a pushed plate rises onto the one below), in an
        # annulus around the last centre (wide inward: flat rims are wide)
        dz = P[:, 2] - self.rim_z
        rr = np.sqrt(((P[:, :2] - self.c) ** 2).sum(axis=1))
        sel = (dz > -self.band_z) & (dz < self.rise_z) & (rr > self.R - 0.05) & (rr < self.R + self.band_r)
        Q = P[sel]
        excl = self.hand_sectors(spheres)
        if excl:                                # points where the hand is do not count, for the edge nor the height
            dq = Q[:, :2] - self.c
            Q = Q[~excluded(np.arctan2(dq[:, 1], dq[:, 0]), excl)]
        self.dbg = dict(Q=Q, excl=excl)
        if len(Q) < 60:
            out["why"] = "only %d rim points" % len(Q)
            out["n"] = int(len(Q))
            return out
        # outer edge per 5 deg sector, re-binned around each new centre
        c = self.c.copy()
        for it in range(4):
            E, cov = edge_points(Q, c, exclude=excl)
            if len(E) < 12:
                break
            if refine_radius and it >= 1:
                c_f, r_f = fit_circle_free(E)
                if abs(r_f - self.R) < 0.012:
                    self.r_band = r_f
            c, e = fit_circle_known_r(E, c, self.r_band, huber=0.002)
        if len(E) < 12:
            out["why"] = "rim edge seen in only %d sectors" % len(E)
            return out
        inl = np.abs(e) < 0.004
        self.dbg.update(E=E, inl=inl, fit_c=c)
        n_in = int(inl.sum())
        n_used = 72 - int(excluded(sector_angles(), excl).sum()) if excl else 72
        cov = float(n_in) / max(n_used, 1)
        rms = float(np.sqrt(np.mean(e[inl] ** 2))) if n_in else 1.0
        jump = float(np.linalg.norm(c - self.c))
        z_new = float(np.median(Q[:, 2]))
        ok = n_in >= 18 and cov >= 0.25 and rms < 0.003 and jump < 0.03
        out.update(n=n_in, coverage=round(cov, 3), rms_mm=round(1000 * rms, 2), jump_mm=round(1000 * jump, 1), points=int(len(Q)),
                   hand_sectors=72 - n_used if excl else 0)
        if ok:
            # the rim plane: rim-top height per 5 deg sector, plane fitted to them. A pushed plate tilts ~5 deg
            # and its pushed-out rim stands ~15 mm above the single height below
            zs, _ = sector_heights(P, c, self.r_band, self.rim_z, excl)
            f = fit_plane(zs, self.R)
            if f is not None:
                out["plane"] = dict(z0=float(f[0]), gx=float(f[1]), gy=float(f[2]), rms_mm=round(1000 * f[3], 2),
                                    n_sec=int(np.isfinite(zs).sum()))
            self.c = c
            self.rim_z = 0.9 * self.rim_z + 0.1 * z_new
            self.n_ok += 1
            out.update(ok=True, center=[float(c[0]), float(c[1]), float(self.rim_z)],
                       moved_mm=float(np.linalg.norm(self.c - self.start) * 1000),
                       move_xy_mm=[float(1000 * v) for v in (self.c - self.start)],
                       top_height=float(self.rim_z - self.table_z), r_band=float(self.r_band))
        else:
            out["why"] = "fit rejected (n %d, coverage %.2f, rms %.1f mm, jump %.0f mm)" % (n_in, cov, 1000 * rms, 1000 * jump)
        self.last = out
        return out

    def init(self, depth, intr, scale, T_po, spheres=(), T_cd=None):
        """First frames: measure the rim band's radius with a free circle fit, then keep it."""
        est = None
        for _ in range(3):
            est = self.update(depth, intr, scale, T_po, spheres, T_cd, refine_radius=True)
        self.start = self.c.copy()
        return est
