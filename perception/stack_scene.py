"""A ray-marched nested plate stack on a table, with exact ground truth.

Shared by test_plate_stack_offline.py (which asserts against it) and check_stack.py
(which uses it for --synthetic, so the tool can be exercised with no camera attached).

The geometry is a solid of revolution whose radius returns to the full plate radius at every
rim level and dips between them, so the stack's side carries one ledge per plate. The top
plate has a rim standing proud of a well. Both of those are the features most likely to fool
a detector, so they are modelled rather than assumed away:

  - the rim/well pair means the TOP PLATE ALONE has two height clusters, and reading the well
    as the top surface is wrong by more than a plate pitch
  - the nested side is what makes the plates countable at all

Surfaces are found by marching and then bisecting, so the depth is not quantised to the march
step. Slow (a few seconds a frame) and exact, which is the right trade for a fixture.
"""
import numpy as np

INTR = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}

PLATE_D = 0.225           # 225 mm dinner plate
R = PLATE_D / 2
RI = 0.080                # where the well begins
PITCH = 0.010             # nesting pitch: 10 mm per plate
RIM0 = 0.018              # a single plate's rim height above the table
WELL = 0.015              # rim stands this proud of the well floor
WAVE = 0.004


def render(n_plates=8, tilt_deg=45.0, dist=0.42, offset=(0.0, 0.0), noise=0.002,
           holes=0.35, seed=0, extra=None, step_m=0.003, normal=None, origin=None):
    """Ray-march a nested plate stack standing on a table.

    tilt_deg is the angle between the camera's view and the table normal.
    `extra` is a list of further stacks as (n_plates, (du, dv)) in table coordinates.
    `normal` and `origin`, if given, override tilt_deg and dist: the table's upward normal
    and the base point of the first stack, both in camera optical coords. That is how a
    scene is placed from a known camera pose, roll included.
    Returns (depth_mm, truth) where truth lists each stack's base centre and top height.
    """
    h, w = INTR["height"], INTR["width"]
    yy, xx = np.mgrid[0:h, 0:w]
    dirs = np.stack([(xx - INTR["cx"]) / INTR["fx"],
                     (yy - INTR["cy"]) / INTR["fy"],
                     np.ones_like(xx, dtype=float)], axis=2)

    t = np.radians(tilt_deg)
    n = np.array([0.0, -np.sin(t), -np.cos(t)])        # table normal, toward the camera
    if normal is not None:
        n = np.asarray(normal, dtype=float)
    n = n / np.linalg.norm(n)
    O = np.array([0.0, 0.0, dist]) - n * 0.0           # table point under the first stack
    if origin is not None:
        O = np.asarray(origin, dtype=float)
    c_table = float(n @ O)

    # an orthonormal basis in the table plane
    e1 = np.array([1.0, 0.0, 0.0]) - n * n[0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)

    nd = dirs @ n
    nd = np.where(np.abs(nd) < 1e-9, 1e-9, nd)
    t_table = c_table / nd
    t_table = np.where(t_table > 0, t_table, np.inf)

    stacks = [(k, o) for k, o in ([(n_plates, offset)] + list(extra or [])) if k >= 1]
    truth = []
    for np_k, off in stacks:
        truth.append({"base": O + e1 * off[0] + e2 * off[1],
                      "top_height_m": RIM0 + (np_k - 1) * PITCH,
                      "n_plates": np_k})

    def inside(P):
        """Is this point inside any stack?"""
        rel = P - O
        z = rel @ n
        got = np.zeros(P.shape[:-1], dtype=bool)
        for (np_k, off), tr in zip(stacks, truth):
            htop = tr["top_height_m"]
            u = rel @ e1 - off[0]
            v = rel @ e2 - off[1]
            r = np.hypot(u, v)
            # wavy side: radius returns to R at every rim level, dips WAVE between them
            phase = 2 * np.pi * (z - RIM0) / PITCH
            rad = R - WAVE * 0.5 * (1.0 - np.cos(phase))
            body = (z >= 0.0) & (z <= htop) & (r <= rad)
            # the top plate's well, carved out of the body
            well = (z > htop - WELL) & (r <= RI)
            got |= body & ~well
        return got

    near = float(np.nanmin(np.where(np.isfinite(t_table), t_table, np.nan))) - 0.30
    far = float(np.nanmax(np.where(np.isfinite(t_table), t_table, np.nan))) + 0.02
    near = max(near, 0.05)

    hit = np.full((h, w), np.inf)
    prev_t = None
    for tv in (np.arange(near, far, step_m) if stacks else []):
        todo = ~np.isfinite(hit)
        if not todo.any():
            break
        P = dirs * tv
        got = inside(P) & todo
        if got.any():
            hit[got] = tv if prev_t is None else prev_t
        prev_t = tv

    # one bisection pass, so the surface is not quantised to the march step
    coarse = np.isfinite(hit)
    lo = np.where(coarse, hit, 0.0)
    hi = lo + step_m
    for _ in range(6):
        mid = 0.5 * (lo + hi)
        g = inside(dirs * mid[:, :, None]) & coarse
        hi = np.where(g, mid, hi)
        lo = np.where(g, lo, mid)
    hit = np.where(coarse, 0.5 * (lo + hi), np.inf)

    depth = np.where(np.isfinite(hit), hit, np.where(np.isfinite(t_table), t_table, 0.0))
    depth = np.where(np.isfinite(depth), depth, 0.0)

    rng = np.random.default_rng(seed)
    if holes:                                 # specular dropout on glaze
        rel = dirs * depth[:, :, None] - O
        rr = np.hypot(rel @ e1, rel @ e2)
        depth[(rr < 0.7 * R) & (rng.random((h, w)) < holes)] = 0.0
    depth += rng.normal(0, noise, depth.shape) * (depth > 0)
    return (np.clip(depth, 0, 6) * 1000.0).astype(np.uint16), {
        "plane": (n, c_table), "stacks": truth, "e1": e1, "e2": e2, "O": O}
