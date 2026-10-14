"""
Turn a lidar map (.pgm + .yaml) into an editable layout the planner can open.

    python3 map_to_plan.py ilounge.pgm ilounge.yaml -o ilounge.plan.json

Then in the planner: Export tab -> Import file. The room arrives as real objects --
walls, obstacles -- that drag, rotate and delete like anything else. Lay tables on it,
then export a static map back out.

WHY NOT JUST SHOW THE PICTURE
    A backdrop image is something to trace against by eye. It is not the map. What the
    nav stack eventually receives has to be built out of things the manager can move,
    delete and add to -- if a partition is coming down for the event, it has to come off
    the plan, and no amount of looking at a photograph of it will do that.

    So this vectorises: occupied cells in, rectangles out.

HOW
    1. The room is almost never square to the grid -- this lab sits about four degrees
       off -- so first find the angle its walls actually run at, by rotating the occupied
       cells and asking which angle packs them into the fewest rows and columns. Walls
       are long and straight, so they concentrate hard when you hit the right angle.
    2. Rotate into that frame, where walls ARE axis-aligned.
    3. Split the occupancy into rectangles: take a blob, draw the smallest box round it,
       and if the box is mostly empty, cut it in half the long way and try again. A
       straight run of wall settles immediately; an L-shape splits into two; a scatter of
       desks splits until each piece is a desk.
    4. Rotate the rectangles back, and emit them as planner items.

WHAT IT IS NOT
    A faithful reproduction. It is a tidy approximation of a noisy scan, which is the
    point: the export is a prior for navigation, and a prior made of a hundred clean
    rectangles is easier for everyone downstream than one made of forty thousand speckles.
    Anything it gets wrong, the manager fixes by dragging it.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

import export_ros_map as ex

MIN_AREA_M2 = 0.06      # smaller than a hand's width of wall; noise
FILL_OK = 0.62          # a box this full of cells is a good enough rectangle
MIN_FILL_EMIT = 0.34    # below this, drop it rather than box thin air
MAX_DEPTH = 8           # how many times a blob may be halved
MAX_RECTS = 400
CLOSE_M = 0.15          # gap in a scan of one solid thing, bridged before tracing
WALL_MAX_THICK = 0.45   # thinner than this, and long, and it is a wall
WALL_MIN_ASPECT = 2.5

# A disc fills pi/4 = 0.785 of its bounding box, and that box is square. A solid
# rectangle fills about 1.0. Those two facts are the whole test, and they are what keeps
# a wall from being read as a very large pillar.
CIRCLE_FILL_LO, CIRCLE_FILL_HI = 0.68, 0.88
CIRCLE_ASPECT = 1.18
CIRCLE_IOU = 0.82
CIRCLE_MIN_R = 0.22


def shift(m, dy, dx):
    out = np.zeros_like(m)
    h, w = m.shape
    y0, y1 = max(0, dy), min(h, h + dy)
    x0, x1 = max(0, dx), min(w, w + dx)
    if y0 < y1 and x0 < x1:
        out[y0:y1, x0:x1] = m[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    return out


def _disc(r):
    return [(dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
            if dy * dy + dx * dx <= r * r]


def close(mask, r):
    """Bridge the gaps a lidar leaves across one solid object.

    A desk does not come back as a desk. It comes back as a scatter of returns off its
    edge, its legs and whatever is on it, with holes everywhere the beam passed over or
    under. Boxing that scatter directly gives a box that is mostly air, which is why the
    first version of this drew a large rectangle across an empty corner. Filling the gaps
    first turns a scatter into the object it came from.
    """
    if r < 1:
        return mask
    d = mask.copy()
    for dy, dx in _disc(r):
        d |= shift(mask, dy, dx)
    e = d.copy()
    for dy, dx in _disc(r):
        e &= shift(d, dy, dx)
    return e | mask          # never erode away something that was really there


def label(mask):
    """4-connected components, iteratively."""
    h, w = mask.shape
    lab = np.zeros((h, w), np.int32)
    cur = 0
    for sy, sx in zip(*np.nonzero(mask)):
        if lab[sy, sx]:
            continue
        cur += 1
        stack = [(sy, sx)]
        lab[sy, sx] = cur
        while stack:
            y, x = stack.pop()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    stack.append((ny, nx))
    return lab, cur


def dominant_angle(xy, res, lo=-45.0, hi=45.0, step=0.25):
    """The angle the walls run at.

    Rotate the cells, bin their x and y, and score how concentrated the bins are. Long
    straight walls pile into a few bins when the frame lines up with them and smear
    across many when it does not, so the best angle is simply the peakiest.
    """
    best, best_score = 0.0, -1.0
    c = xy.mean(axis=0)
    q = xy - c
    for deg in np.arange(lo, hi + 1e-9, step):
        a = math.radians(deg)
        ca, sa = math.cos(a), math.sin(a)
        rx = q[:, 0] * ca + q[:, 1] * sa
        ry = -q[:, 0] * sa + q[:, 1] * ca
        score = 0.0
        for v in (rx, ry):
            n = int((v.max() - v.min()) / res) + 1
            if n < 2:
                continue
            hist = np.bincount(((v - v.min()) / res).astype(int), minlength=n)
            score += float((hist.astype(np.float64) ** 2).sum())
        if score > best_score:
            best_score, best = score, float(deg)
    return best


def as_circle(pts, res, x0, y0, x1, y1, fill):
    """The disc this blob is, or None.

    Round tables and pillars are common, and a staircase of boxes is a poor way to say
    so: wrong at every edge, and thirty objects where one would do.
    """
    w, h = x1 - x0, y1 - y0
    if not (CIRCLE_FILL_LO <= fill <= CIRCLE_FILL_HI):
        return None
    if max(w, h) / max(min(w, h), 1e-6) > CIRCLE_ASPECT:
        return None
    r = (w + h) / 4.0
    if r < CIRCLE_MIN_R:
        return None
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    inside = int((np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) <= r).sum())
    disc_cells = math.pi * r * r / (res * res)
    union = len(pts) + disc_cells - inside
    if union <= 0 or inside / union < CIRCLE_IOU:
        return None
    return {"cx": cx, "cy": cy, "w": 2 * r, "h": 2 * r, "fill": fill, "round": True}


def split_rects(pts, res, out, depth=0):
    """Rectangles -- and discs -- covering `pts` (cell centres, axis-aligned frame)."""
    if len(pts) == 0 or len(out) >= MAX_RECTS:
        return
    x0, y0 = pts[:, 0].min() - res / 2, pts[:, 1].min() - res / 2
    x1, y1 = pts[:, 0].max() + res / 2, pts[:, 1].max() + res / 2
    w, h = x1 - x0, y1 - y0
    area = w * h
    covered = len(pts) * res * res
    if area < MIN_AREA_M2:
        return
    circ = as_circle(pts, res, x0, y0, x1, y1, covered / area)
    if circ:
        out.append(circ)
        return
    if covered / area >= FILL_OK or depth >= MAX_DEPTH:
        # At the bottom of the recursion a blob may still be mostly air -- a diagonal
        # smear, a scatter of stray returns. Boxing it would invent an obstacle in a
        # place the robot can actually drive through, which is the worse error.
        if covered / area >= MIN_FILL_EMIT:
            out.append({"cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2, "w": w, "h": h,
                        "fill": covered / area})
        return
    # cut the long way, at the median, so a corner separates into two straight runs
    axis = 0 if w >= h else 1
    cut = float(np.median(pts[:, axis]))
    left = pts[pts[:, axis] <= cut]
    right = pts[pts[:, axis] > cut]
    if len(left) == 0 or len(right) == 0:      # everything on one side; stop splitting
        if covered / area >= MIN_FILL_EMIT:
            out.append({"cx": (x0 + x1) / 2, "cy": (y0 + y1) / 2, "w": w, "h": h,
                        "fill": covered / area})
        return
    split_rects(left, res, out, depth + 1)
    split_rects(right, res, out, depth + 1)


def trace(grid, res, angle=None, min_blob_m2=0.06, close_m=CLOSE_M):
    """Occupied cells in, planner rectangles out (metres, planner frame)."""
    H, W = grid.shape
    occ = close(grid < 100, int(round(close_m / res)))
    ys, xs = np.nonzero(occ)
    if len(ys) == 0:
        return [], 0.0
    # planner frame: x east from the map's west edge, y SOUTH from its north edge, which
    # for a PGM is simply (col, row) scaled -- row 0 already IS the north edge.
    xy = np.stack([(xs + 0.5) * res, (ys + 0.5) * res], axis=1)

    if angle is None:
        sub = xy if len(xy) < 40000 else xy[np.random.default_rng(0).choice(
            len(xy), 40000, replace=False)]
        angle = dominant_angle(sub, res)
    a = math.radians(angle)
    ca, sa = math.cos(a), math.sin(a)
    centre = xy.mean(axis=0)
    q = xy - centre
    rot = np.stack([q[:, 0] * ca + q[:, 1] * sa, -q[:, 0] * sa + q[:, 1] * ca], axis=1)

    # components in the rotated frame, so a wall is one straight run
    gx = ((rot[:, 0] - rot[:, 0].min()) / res).astype(int)
    gy = ((rot[:, 1] - rot[:, 1].min()) / res).astype(int)
    rh, rw = gy.max() + 1, gx.max() + 1
    rmask = np.zeros((rh, rw), bool)
    rmask[gy, gx] = True
    lab, n = label(rmask)

    rects = []
    for k in range(1, n + 1):
        cy, cx = np.nonzero(lab == k)
        if len(cy) * res * res < min_blob_m2:
            continue
        pts = np.stack([cx * res + rot[:, 0].min(), cy * res + rot[:, 1].min()], axis=1)
        split_rects(pts, res, rects)

    out = []
    for r in rects:
        # back to the planner frame
        px = r["cx"] * ca - r["cy"] * sa + centre[0]
        py = r["cx"] * sa + r["cy"] * ca + centre[1]
        if r.get("round"):
            # a disc has no orientation, so the wall angle means nothing for it
            out.append({"type": "obst", "shape": "circle",
                        "x": round(float(px), 3), "y": round(float(py), 3),
                        "w": round(float(r["w"]), 3), "h": round(float(r["w"]), 3),
                        "rot": 0.0, "fill": round(r["fill"], 2)})
            continue
        thin = min(r["w"], r["h"])
        longd = max(r["w"], r["h"])
        kind = ("wall" if thin <= WALL_MAX_THICK and longd / max(thin, 1e-6) >= WALL_MIN_ASPECT
                else "obst")
        out.append({"type": kind, "shape": "rect",
                    "x": round(float(px), 3), "y": round(float(py), 3),
                    "w": round(float(r["w"]), 3), "h": round(float(r["h"]), 3),
                    "rot": round(angle, 2), "fill": round(r["fill"], 2)})
    return out, angle


FLOOR_CELL = 0.25       # the floor mask is coarse on purpose; it only gates placement


def floor_mask(grid, res, cell=FLOOR_CELL):
    """The room's own extent, as a coarse bitmap: where a table may legally stand.

    This is NOT "where the scan saw free floor", which is what it used to be and was
    wrong in a way that only showed up later: a desk's footprint is not free floor, so
    the desk's own cells were marked unplaceable for good. Delete the desk in the planner
    and the space stayed barred -- the mask still said "no floor here", because at scan
    time there had been a desk standing on it.

    So: take the free space, then fill its holes. Furniture sits in the middle of floor,
    which makes it a hole, which fills. Unknown space outside the building reaches the
    border and does not. What comes out is the envelope of the room, and whether any
    particular spot inside it is blocked is a question about the objects on the plan --
    which the planner already answers, and can answer again after you move one.

    Packed LSB-first, row-major, base64. At a quarter metre a whole hall is a couple of
    kilobytes.
    """
    import base64
    H, W = grid.shape
    k = max(int(round(cell / res)), 1)
    w, h = W // k, H // k
    free = (grid[:h * k, :w * k] == 254).reshape(h, k, w, k).mean(axis=(1, 3)) >= 0.5

    # flood the NOT-free cells from the border; whatever that cannot reach is a hole
    outside = np.zeros((h, w), bool)
    stack = []
    def push(r, c):
        if 0 <= r < h and 0 <= c < w and not outside[r, c] and not free[r, c]:
            outside[r, c] = True
            stack.append((r, c))
    for c in range(w):
        push(0, c); push(h - 1, c)
    for r in range(h):
        push(r, 0); push(r, w - 1)
    while stack:
        r, c = stack.pop()
        push(r + 1, c); push(r - 1, c); push(r, c + 1); push(r, c - 1)

    inside = free | ~outside          # free floor, plus every hole in it
    bits = inside.reshape(-1)
    packed = bytearray((len(bits) + 7) // 8)
    for idx, b in enumerate(bits):
        if b:
            packed[idx >> 3] |= 1 << (idx & 7)
    return {"cell": cell, "w": int(w), "h": int(h),
            "bits": base64.b64encode(bytes(packed)).decode("ascii")}


def trace_exact(grid, res):
    """One to one with the map: every occupied cell covered, nothing else, no rotation.

    Not one rectangle per cell -- that is forty thousand objects for a hall. Horizontal
    runs of occupied cells become one rectangle each, and a run merges downward with the
    run above when they start and end on the same columns. A wall six hundred cells long
    and three thick is one rectangle, not eighteen hundred. The cover is still exact.
    """
    H, W = grid.shape
    occ = grid < 100
    out = []
    open_runs = {}

    def emit(c0, c1, r0, r1):
        w, h = (c1 - c0 + 1) * res, (r1 - r0 + 1) * res
        thin, longd = min(w, h), max(w, h)
        kind = ("wall" if thin <= WALL_MAX_THICK and longd / max(thin, 1e-9) >= WALL_MIN_ASPECT
                else "obst")
        out.append({"type": kind, "shape": "rect",
                    "x": round((c0 + (c1 - c0 + 1) / 2.0) * res, 4),
                    "y": round((r0 + (r1 - r0 + 1) / 2.0) * res, 4),
                    "w": round(w, 4), "h": round(h, 4), "rot": 0.0, "fill": 1.0})

    for r in range(H):
        row = occ[r]
        nxt = {}
        c = 0
        while c < W:
            if not row[c]:
                c += 1
                continue
            c0 = c
            while c < W and row[c]:
                c += 1
            key = (c0, c - 1)
            nxt[key] = open_runs.pop(key, r)
        for (c0, c1), r0 in open_runs.items():
            emit(c0, c1, r0, r - 1)
        open_runs = nxt
    for (c0, c1), r0 in open_runs.items():
        emit(c0, c1, r0, H - 1)
    return out, 0.0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("pgm")
    ap.add_argument("yaml")
    ap.add_argument("-o", "--out", default=None, help="the plan json to write")
    ap.add_argument("--angle", type=float, default=None,
                    help="force the wall angle in degrees instead of finding it")
    ap.add_argument("--min-blob", type=float, default=0.06,
                    help="ignore occupied blobs smaller than this, m2")
    ap.add_argument("--close", type=float, default=CLOSE_M,
                    help="bridge scan gaps up to this wide before tracing, m")
    ap.add_argument("--exact", action="store_true",
                    help="cover every occupied cell exactly: no rotation, no gap "
                         "bridging, no approximation. Many more pieces.")
    ap.add_argument("--name", default=None, help="name for the layout")
    args = ap.parse_args(argv)

    meta = ex.read_map_yaml(args.yaml)
    grid = ex.read_pgm(args.pgm)
    res = meta["resolution"]
    H, W = grid.shape
    w_m, h_m = W * res, H * res
    print("map   %d x %d px at %.3f m/px  =  %.2f x %.2f m" % (W, H, res, w_m, h_m))
    print("      origin [%.3f, %.3f]" % (meta["origin"][0], meta["origin"][1]))

    if args.exact:
        rects, angle = trace_exact(grid, res)
    else:
        rects, angle = trace(grid, res, args.angle, args.min_blob, args.close)
    walls = sum(1 for r in rects if r["type"] == "wall")
    discs = sum(1 for r in rects if r.get("shape") == "circle")
    print("walls run at %.2f deg off the grid" % angle)
    print("traced %d pieces: %d wall, %d round, %d box"
          % (len(rects), walls, discs, len(rects) - walls - discs))
    occ_m2 = int((grid < 100).sum()) * res * res
    got_m2 = sum(r["w"] * r["h"] * r["fill"] for r in rects)
    print("       covering %.0f m2 of the %.0f m2 the scan marks occupied (%.0f%%)"
          % (got_m2, occ_m2, 100 * got_m2 / max(occ_m2, 1e-6)))

    items, nid = [], 1
    for r in rects:
        it = {"k": "f", "id": nid, "type": r["type"], "x": r["x"], "y": r["y"],
              "w": r["w"], "h": r["h"], "rot": r["rot"]}
        if r.get("shape") == "circle":
            it["shape"] = "circle"
        items.append(it)
        nid += 1
    plan = {
        "roomId": "custom",
        "custom": {"w": round(w_m, 3), "h": round(h_m, 3)},
        "event": {"name": args.name or os.path.splitext(os.path.basename(args.pgm))[0],
                  "date": "", "pax": 0, "courses": 3},
        "items": items,
        "nextId": nid,
        "floor": floor_mask(grid, res),
        "traced_from": {"image": os.path.basename(args.pgm),
                        "resolution": res, "origin": meta["origin"],
                        "size_px": [W, H], "wall_angle_deg": angle},
    }
    out = args.out or (os.path.splitext(os.path.basename(args.pgm))[0] + ".plan.json")
    with open(out, "w") as f:
        f.write(json.dumps(plan, indent=1) + "\n")
    fm = plan["floor"]
    import base64 as _b64
    _bits = _b64.b64decode(fm["bits"])
    _n = sum(bin(b).count("1") for b in _bits)
    print("room extent %d x %d cells at %.2f m  (%.0f m2 placeable, %.1f kB)"
          % (fm["w"], fm["h"], fm["cell"], _n * fm["cell"] ** 2,
             len(fm["bits"]) / 1024.0))
    print("\nwrote %s  (%d items)" % (out, len(items)))
    print("open the planner -> Export tab -> Import file")
    return 0


if __name__ == "__main__":
    sys.exit(main())
