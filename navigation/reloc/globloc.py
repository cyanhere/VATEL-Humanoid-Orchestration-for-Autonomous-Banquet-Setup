"""Global relocalization core: find a robot pose anywhere in a map.

numpy only - no scipy, so it runs identically on the laptop (numpy 2.x) and on
the robot (numpy 1.17/1.24), and sidesteps the scipy/np.typeDict breakage.

Nearest-neighbour lookups use a voxel-centroid hash instead of a KD-tree.  Each
occupied voxel is collapsed to the centroid of the points inside it; a query
checks the 27 voxels around itself and keeps the closest centroid.  At 10 cm
voxels that is within a few mm of true nearest-neighbour for scan matching,
and it is fully vectorised.
"""
import numpy as np

_OFF = np.array([(dx, dy, dz) for dx in (-1, 0, 1)
                 for dy in (-1, 0, 1) for dz in (-1, 0, 1)], dtype=np.int64)


def voxel_downsample(P, v):
    k = np.floor(P / v).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return P[np.sort(idx)]


def yaw_to_R(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose_to_T(x, y, yaw, z=0.0):
    T = np.eye(4)
    T[:3, :3] = yaw_to_R(yaw)
    T[:3, 3] = (x, y, z)
    return T


def floor_z(P, bin_w=0.05, span=1.5):
    """Height of the dominant floor plane in a cloud.

    The floor is the densest horizontal slice near the bottom of an indoor
    cloud, so take the fullest histogram bin within `span` metres of the
    lowest points (ignoring the sparse tail that stray returns produce)."""
    z = P[:, 2]
    lo = np.percentile(z, 1.0)
    sel = z[z < lo + span]
    if len(sel) < 50:
        return float(lo)
    hist, edges = np.histogram(sel, bins=max(4, int(span / bin_w)))
    k = int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def estimate_z(scan_body, map_pts):
    """Height of the robot's body frame in map coordinates.

    Matching a scan taken at one sensor height against a map built at another
    fails if the search assumes z=0 - every point sits a fixed offset off its
    surface.  Both clouds see the same floor, so line the floors up:

        body_z_in_map = map_floor_z - floor_z_in_body_frame

    (floor_z_in_body_frame is negative - the floor is below the sensor - so
    this comes out positive, and equals the sensor height above the map floor.)
    """
    return floor_z(map_pts) - floor_z(scan_body)


def T_to_pose(T):
    return T[0, 3], T[1, 3], np.arctan2(T[1, 0], T[0, 0])


class VoxelNN:
    """Nearest-neighbour over voxel centroids."""

    _B = np.int64(1 << 21)

    def __init__(self, P, v):
        self.v = float(v)
        ijk = np.floor(P / self.v).astype(np.int64)
        self._o = ijk.min(axis=0) - 2                 # keep keys non-negative
        key = self._key(ijk)
        order = np.argsort(key, kind='stable')
        key, Ps = key[order], P[order]
        first = np.ones(len(key), dtype=bool)
        first[1:] = key[1:] != key[:-1]
        start = np.flatnonzero(first)
        cnt = np.diff(np.append(start, len(key))).astype(np.float64)
        cs = np.cumsum(Ps, axis=0)
        tot = cs[np.append(start[1:], len(key)) - 1]
        tot[1:] -= cs[start[1:] - 1]
        self.keys = key[start]                        # sorted, unique
        self.cent = tot / cnt[:, None]

    def _key(self, ijk):
        d = ijk - self._o
        return (d[:, 0] * self._B + d[:, 1]) * self._B + d[:, 2]

    def query(self, Q):
        """Return (distance, centroid) of the nearest centroid to each row."""
        base = np.floor(Q / self.v).astype(np.int64)
        best_d = np.full(len(Q), np.inf)
        best_i = np.zeros(len(Q), dtype=np.int64)
        for off in _OFF:
            k = self._key(base + off)
            j = np.searchsorted(self.keys, k)
            ok = j < len(self.keys)
            j = np.where(ok, j, 0)
            ok &= self.keys[j] == k
            if not ok.any():
                continue
            d = np.linalg.norm(Q[ok] - self.cent[j[ok]], axis=1)
            w = np.flatnonzero(ok)
            upd = d < best_d[w]
            best_d[w[upd]] = d[upd]
            best_i[w[upd]] = j[ok][upd]
        return best_d, self.cent[best_i]


class MultiNN:
    """One VoxelNN per search radius.

    A VoxelNN only looks at the 27 voxels around a query, so it can only see
    out to ~1.5 voxels.  A single fine grid therefore cannot serve a coarse
    ICP gate - correspondences past ~1.5v come back as inf and the gate starves.
    So build one level per gate, sized to that gate."""

    def __init__(self, P, gates):
        self.levels = {}
        for g in sorted(set(gates)):
            self.levels[g] = VoxelNN(P, max(0.10, g / 1.5))

    def __getitem__(self, gate):
        return self.levels[gate]


def score(nn, S, T, gate):
    """Inlier fraction and inlier RMSE of scan S placed at pose T."""
    Q = S @ T[:3, :3].T + T[:3, 3]
    d, _ = (nn[gate] if isinstance(nn, MultiNN) else nn).query(Q)
    m = d < gate
    if not m.any():
        return 0.0, np.inf
    return m.mean(), float(np.sqrt((d[m] ** 2).mean()))


def _kabsch(A, B):
    """Rigid transform taking A onto B, constrained to yaw + translation."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca)[:, :2].T @ (B - cb)[:, :2]
    yaw = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    R = yaw_to_R(yaw)
    T = np.eye(4)
    T[:3, :3] = R
    t = cb - R @ ca
    t[2] = 0.0                       # planar robot - no vertical slip
    T[:3, 3] = t
    return T


def icp(nn, S, T0, gates=(1.0, 0.5, 0.3), iters=10, min_inl=0.25):
    """Point-to-point ICP, planar (yaw + xy), coarse-to-fine gates."""
    if not isinstance(nn, MultiNN):
        nn = MultiNN(nn.cent, gates)
    T = T0.copy()
    inl = rmse = 0.0
    for gate in gates:
        lvl = nn[gate]
        for _ in range(iters):
            Q = S @ T[:3, :3].T + T[:3, 3]
            d, C = lvl.query(Q)
            m = d < gate
            if m.sum() < max(30, min_inl * len(S) * 0.3):
                break
            dT = _kabsch(Q[m], C[m])
            T = dT @ T
            if np.linalg.norm(dT[:3, 3]) < 1e-3 and abs(
                    np.arctan2(dT[1, 0], dT[0, 0])) < 1e-3:
                break
        inl, rmse = score(nn, S, T, gates[-1])
    return T, rmse, inl


def read_pgm_map(yaml_path):
    """Read a Nav2 map yaml + binary P5 pgm. Returns (occ, res, origin).

    occ is a float array in [0,1] of occupancy probability, row 0 = LOWEST y
    (the pgm is stored top-row-first, so it gets flipped)."""
    import os
    meta = {}
    for line in open(yaml_path):
        line = line.split('#')[0].rstrip()
        if line.startswith('- '):
            meta.setdefault('origin', []).append(float(line[2:]))
        elif ':' in line:
            k, v = line.split(':', 1)
            k, v = k.strip(), v.strip()
            if v.startswith('[') and v.endswith(']'):      # inline flow list
                meta[k] = [float(t) for t in v[1:-1].split(',')]
            elif v == '':                                  # block list follows
                meta[k] = []
            else:
                meta[k] = v
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    if not os.path.exists(pgm):                    # yaml often names a bare file
        base = os.path.basename(yaml_path).replace('.yaml', '.pgm')
        pgm = os.path.join(os.path.dirname(yaml_path), base)
    with open(pgm, 'rb') as f:
        assert f.readline().strip() == b'P5', 'not a binary pgm'
        vals = []
        while len(vals) < 3:
            ln = f.readline().split(b'#')[0].split()
            vals += [int(t) for t in ln]
        w, h, mx = vals
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    p = 1.0 - img.astype(np.float64) / mx          # white=free -> p=0
    if int(meta.get('negate', 0)):
        p = 1.0 - p
    return p[::-1], float(meta['resolution']), np.array(meta['origin'][:2])


def free_positions(occ, res, origin, spacing, clearance=0.35,
                   free_thresh=0.25, bounds=None):
    """Candidate robot positions: free cells with `clearance` m of free around."""
    rad = max(1, int(round(clearance / res)))
    H, W = occ.shape
    free = occ < free_thresh
    known = occ >= 0.0
    safe = free.copy()
    for dy in range(-rad, rad + 1):                # erode by the clearance
        for dx in range(-rad, rad + 1):
            if dx * dx + dy * dy > rad * rad:
                continue
            safe &= np.roll(np.roll(free, dy, 0), dx, 1)
    step = max(1, int(round(spacing / res)))
    ys, xs = np.nonzero(safe[::step, ::step])
    P = np.stack([origin[0] + (xs * step + 0.5) * res,
                  origin[1] + (ys * step + 0.5) * res], axis=1)
    if bounds is not None:
        cx, cy, r = bounds
        P = P[np.hypot(P[:, 0] - cx, P[:, 1] - cy) <= r]
    return P


def sweep(nn, S, positions, n_yaw=24, gates=(1.5, 0.8, 0.4, 0.25),
          coarse_pts=400, refine_pts=2500, top_k=40, rng=None, progress=None,
          z=0.0):
    """Global search: cheap-score every (position, yaw), ICP-refine the best.

    Ranks by INLIER FRACTION, not RMSE.  A wrong pose in a room full of
    parallel walls produces a fine-looking RMSE from the few points that do
    line up; the fraction of the scan that finds any support is what actually
    separates right from wrong."""
    rng = rng or np.random.default_rng(0)
    idx = rng.choice(len(S), min(coarse_pts, len(S)), replace=False)
    Sc = S[idx]
    Sr = S[rng.choice(len(S), min(refine_pts, len(S)), replace=False)]
    yaws = 2 * np.pi * np.arange(n_yaw) / n_yaw
    cg = gates[0]

    cand = []
    for i, (x, y) in enumerate(positions):
        for yw in yaws:
            inl, _ = score(nn, Sc, pose_to_T(x, y, yw, z), cg)
            cand.append((inl, x, y, yw))
        if progress and i % 25 == 0:
            progress(i, len(positions))
    cand.sort(key=lambda c: -c[0])

    out = []
    for _, x, y, yw in cand[:top_k]:
        T, rmse, inl = icp(nn, Sr, pose_to_T(x, y, yw, z), gates=gates)
        out.append((inl, rmse) + T_to_pose(T) + (yw,))
    out.sort(key=lambda r: (-r[0], r[1]))
    return dedupe(out)


def dedupe(res, dx=1.2, dyaw=np.radians(25)):
    """Collapse results that are the same pose, so runners-up are real rivals.

    The top candidates almost always converge onto one another; without this
    the 'runner-up' is a copy of the winner and the ambiguity gap reads 0."""
    keep = []
    for r in res:
        x, y, yw = r[2], r[3], r[4]
        if any(np.hypot(x - k[2], y - k[3]) < dx and
               abs((yw - k[4] + np.pi) % (2 * np.pi) - np.pi) < dyaw for k in keep):
            continue
        keep.append(r)
    return keep


def verdict(res, min_inl=0.58, min_gap=0.04):
    """Judge a sweep result. Returns (ok, message)."""
    if not res:
        return False, 'no candidates scored'
    best = res[0][0]
    if best < min_inl:
        return False, ('LOW CONFIDENCE - best fit is only %.0f%% inliers. The scan '
                       'does not match the map well anywhere; the room has probably '
                       'changed, or the robot is outside the mapped area.' % (100 * best))
    if len(res) > 1 and best - res[1][0] < min_gap:
        return False, ('AMBIGUOUS - two different poses fit almost equally well '
                       '(%.0f%% vs %.0f%%). Move the robot somewhere more '
                       'distinctive and retry.' % (100 * best, 100 * res[1][0]))
    return True, 'CONFIDENT - %.0f%% inliers, next rival %.0f%%' % (
        100 * best, 100 * res[1][0] if len(res) > 1 else 0)
