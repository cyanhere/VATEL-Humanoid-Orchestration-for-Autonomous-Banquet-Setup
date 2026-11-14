"""Track a hand-drawn checker on the plate's well floor. Test reference only, to judge the rim tracker."""
import numpy as np


def find(gray, last, r=20, area=(8, 260), bg_min=130.0, dark=45.0):
    """The checker near `last` (u, v): (u, v, long axis deg, dark pixels) or None."""
    import cv2
    x0, y0 = max(int(round(last[0])) - r, 0), max(int(round(last[1])) - r, 0)
    roi = gray[y0:y0 + 2 * r, x0:x0 + 2 * r].astype(float)
    if roi.size == 0:
        return None
    bg = float(np.median(roi))
    if bg < bg_min:                             # not on the white plate: a hand or the arm over it
        return None
    n, lab, st, cen = cv2.connectedComponentsWithStats((roi < bg - dark).astype(np.uint8), connectivity=8)
    rel = np.array([last[0] - x0, last[1] - y0])
    near = [i for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 2 and np.linalg.norm(cen[i] - rel) < r * 0.75]
    if not near:
        return None
    k = min(near, key=lambda i: np.linalg.norm(cen[i] - rel) - 0.3 * st[i, cv2.CC_STAT_AREA])
    keep = [i for i in near if np.linalg.norm(cen[i] - cen[k]) < r * 0.6]
    m = np.isin(lab, keep)
    px = int(m.sum())
    if not area[0] <= px <= area[1]:
        return None
    v, u = np.nonzero(m)
    w = bg - roi[v, u]
    x, y = float((u * w).sum() / w.sum()), float((v * w).sum() / w.sum())
    C = np.cov(np.c_[u, v].T, aweights=w)
    ang = 0.5 * float(np.degrees(np.arctan2(2 * C[0, 1], C[0, 0] - C[1, 1])))
    return x + x0, y + y0, ang, px


def to_plane(u, v, intr, T_po, z):
    """Pixel -> the pelvis-frame point at height z on its ray (T_po: pelvis <- colour optical)."""
    ray = np.array([(u - intr["cx"]) / intr["fx"], (v - intr["cy"]) / intr["fy"], 1.0])
    d = T_po[:3, :3] @ ray
    if abs(d[2]) < 1e-6:
        return None
    return T_po[:3, 3] + d * (z - T_po[2, 3]) / d[2]


class MarkerTrack(object):
    """Follows the checker from a first guess (the located plate centre). update() gives the plate's travel since the first
    good frame, on the plane of the well floor."""

    def __init__(self, seed_xy, z, intr, seek=32, r=20):
        self.seed_xy, self.z, self.intr, self.seek, self.r = np.array(seed_xy, float)[:2], float(z), intr, seek, r
        self.uv = None
        self.start = None

    def _seed_uv(self, T_po):
        P = np.array([self.seed_xy[0], self.seed_xy[1], self.z])
        o = (P - T_po[:3, 3]) @ T_po[:3, :3]
        if o[2] < 0.05:
            return None
        return np.array([self.intr["fx"] * o[0] / o[2] + self.intr["cx"], self.intr["fy"] * o[1] / o[2] + self.intr["cy"]])

    def update(self, colour, T_po):
        """dict(ok, uv, xy, moved_xy_mm, moved_mm, angle_deg, dark_px)."""
        import cv2
        out = dict(ok=False)
        if colour is None:
            return out
        gray = cv2.cvtColor(colour, cv2.COLOR_BGR2GRAY)
        f = None
        if self.uv is not None:
            f = find(gray, self.uv, self.r)
        if f is None and self.start is None:              # not found yet: look around where the plate should be
            uv = self._seed_uv(T_po)
            if uv is not None:
                f = find(gray, uv, self.seek)
        if f is None:
            return out
        self.uv = np.array(f[:2])
        P = to_plane(f[0], f[1], self.intr, T_po, self.z)
        if P is None:
            return out
        if self.start is None:
            self.start = P[:2].copy()
        mv = 1000 * (P[:2] - self.start)
        return dict(ok=True, uv=[round(f[0], 2), round(f[1], 2)], xy=[round(float(P[0]), 5), round(float(P[1]), 5)],
                    moved_xy_mm=[round(float(mv[0]), 2), round(float(mv[1]), 2)], moved_mm=round(float(np.linalg.norm(mv)), 2),
                    angle_deg=round(f[2], 1), dark_px=f[3])
