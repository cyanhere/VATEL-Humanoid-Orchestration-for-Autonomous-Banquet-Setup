"""
Checks for the map tracer: lidar occupancy grid in, editable objects out.

    python3 test_map_to_plan.py

The browser does the same job in index.html so nobody has to run a script, and these
numbers are what it is measured against. When the two were written they disagreed three
times -- a sampling stride that beat against the pixel grid and voted for zero degrees,
a border the erosion refused to eat (which painted straight lines all round the frame
and voted for zero again), and a median that cut at a value so many cells shared that the
split gave up. None of those are visible by looking at the result; all three are obvious
against a reference.
"""

import math
import os
import sys

import numpy as np

import export_ros_map as ex
import map_to_plan as m2p

ok = fails = 0


def check(label, cond, detail=""):
    global ok, fails
    if cond:
        ok += 1
        print("  PASS  %-52s %s" % (label, detail))
    else:
        fails += 1
        print("  FAIL  %-52s %s" % (label, detail))


# ---------------------------------------------------------------------------
# A banquet hall as a lidar leaves it: walls a few degrees off square, two pillars,
# nine round tables, and nearly half the returns missing.
# ---------------------------------------------------------------------------
RES = 0.05
W_M, H_M = 20.0, 21.0
TILT = -3.0
FT = 0.3048
STOCK = {5: 5 * FT, 6: 6 * FT, 7: 7 * FT}
SPOTS = [(-5.5, -6.0, 6), (0.0, -6.0, 6), (5.5, -6.0, 6),
         (-5.5, -1.0, 7), (0.0, -1.0, 5), (5.5, -1.0, 7),
         (-5.5, 4.0, 6), (0.0, 4.0, 6), (5.5, 4.0, 5)]
PILLARS = [(-4.0, -3.0), (4.0, 3.0)]


def build_hall(dropout=0.45, seed=3):
    W, H = int(W_M / RES), int(H_M / RES)
    g = np.full((H, W), ex.FREE, np.uint8)
    ys, xs = np.mgrid[0:H, 0:W]
    px, py = (xs + 0.5) * RES, (ys + 0.5) * RES
    a = math.radians(TILT)
    u = (px - W_M / 2) * math.cos(a) + (py - H_M / 2) * math.sin(a)
    v = -(px - W_M / 2) * math.sin(a) + (py - H_M / 2) * math.cos(a)
    inside = (np.abs(u) <= 9.2) & (np.abs(v) <= 9.7)
    ring = (np.abs(u) <= 9.35) & (np.abs(v) <= 9.85)
    g[ring & ~inside] = ex.OCCUPIED
    g[~ring] = ex.UNKNOWN
    for pc in PILLARS:
        g[np.hypot(u - pc[0], v - pc[1]) <= 0.30] = ex.OCCUPIED
    for (qa, qb, ft) in SPOTS:
        g[np.hypot(u - qa, v - qb) <= STOCK[ft] / 2] = ex.OCCUPIED
    if dropout:
        rng = np.random.default_rng(seed)
        g[(g == ex.OCCUPIED) & (rng.random((H, W)) < dropout)] = ex.FREE
    return g


def truth_centres():
    a = math.radians(TILT)
    ca, sa = math.cos(a), math.sin(a)
    return [(W_M / 2 + (qa * ca - qb * sa), H_M / 2 + (qa * sa + qb * ca), STOCK[ft])
            for (qa, qb, ft) in SPOTS]


grid = build_hall()
rects, angle = m2p.trace(grid, RES)
circles = [r for r in rects if r.get("shape") == "circle"]
walls = [r for r in rects if r["type"] == "wall"]

print("\nA HALL OF ROUND TABLES, HALF THE SCAN MISSING")
check("the wall angle is found", abs(angle - TILT) <= 0.3,
      "%.2f deg, laid out at %.1f" % (angle, TILT))
check("the walls are traced as walls", len(walls) >= 8, "%d wall pieces" % len(walls))
check("every round table comes back round", len(circles) >= 9,
      "%d discs for 9 tables + 2 pillars" % len(circles))

print("\n  EACH TABLE, AGAINST WHERE IT WAS PUT")
worst_pos = worst_dia = 0.0
for (tx, ty, td) in truth_centres():
    best, bd = None, 1e9
    for c in circles:
        d = math.hypot(c["x"] - tx, c["y"] - ty)
        if d < bd:
            bd, best = d, c
    worst_pos = max(worst_pos, bd)
    worst_dia = max(worst_dia, abs(best["w"] - td))
    check("%.2f m table" % td, bd < 0.10 and abs(best["w"] - td) < 0.10,
          "%3.0f mm out of place, %+4.0f mm on size" % (bd * 1000, (best["w"] - td) * 1000))
check("worst position error under 100 mm", worst_pos < 0.10, "%.0f mm" % (worst_pos * 1000))
check("worst diameter error under 100 mm", worst_dia < 0.10, "%.0f mm" % (worst_dia * 1000))

print("\n  A WALL IS NOT A VERY LARGE PILLAR")
# a solid rectangle fills ~1.0 of its box; a disc fills pi/4. That is the whole test.
solid = np.stack(np.meshgrid(np.arange(0, 3.0, RES), np.arange(0, 0.4, RES)),
                 axis=-1).reshape(-1, 2)
check("a long solid slab is refused as a disc",
      m2p.as_circle(solid, RES, 0, 0, 3.0, 0.4, 1.0) is None)
square = np.stack(np.meshgrid(np.arange(0, 2.0, RES), np.arange(0, 2.0, RES)),
                  axis=-1).reshape(-1, 2)
check("a solid square is refused too",
      m2p.as_circle(square, RES, 0, 0, 2.0, 2.0, 1.0) is None,
      "it fills its box, a disc fills 0.785 of it")
th = np.linspace(0, 2 * math.pi, 400)
disc = np.array([[x, y] for x in np.arange(0, 1.8, RES) for y in np.arange(0, 1.8, RES)
                 if math.hypot(x - 0.9, y - 0.9) <= 0.9])
check("an actual disc is accepted",
      m2p.as_circle(disc, RES, 0, 0, 1.8, 1.8, len(disc) * RES * RES / (1.8 * 1.8)) is not None)

print("\n  THE ROOM'S EXTENT INCLUDES WHAT IS STANDING ON IT")
fm = m2p.floor_mask(grid, RES)
import base64
bits = base64.b64decode(fm["bits"])
at = lambda c, r: (bits[(r * fm["w"] + c) >> 3] >> ((r * fm["w"] + c) & 7)) & 1
# the floor under a table has to stay placeable, or deleting the table frees nothing
hits = 0
for (tx, ty, td) in truth_centres():
    if at(int(tx / fm["cell"]), int(ty / fm["cell"])):
        hits += 1
check("the floor under every table is placeable", hits == len(SPOTS),
      "%d of %d -- this is what lets you delete furniture and lay a table there"
      % (hits, len(SPOTS)))
check("space the scan never saw is not placeable",
      not at(1, 1) and not at(fm["w"] - 2, fm["h"] - 2),
      "the corners outside the room stay barred")

print("\n  DETAIL HAS TO ACTUALLY CHANGE SOMETHING")
# fillOK is the point at which a box is called good enough, so RAISING it chases the
# shape harder. The first version lowered it for "fine", which accepted looser boxes --
# fine was coarser than normal, and both traced exactly the same count. The levels have
# to come out strictly ordered or the control is a lie.
LEVELS = [("coarse", 5, 0.15, 0.52, 400),
          ("normal", 8, 0.06, 0.68, 2000),
          ("fine", 13, 0.015, 0.88, 12000)]
counts = []
for name, depth, minarea, fillok, cap in LEVELS:
    m2p.MAX_DEPTH, m2p.MIN_AREA_M2, m2p.FILL_OK, m2p.MAX_RECTS = depth, minarea, fillok, cap
    counts.append((name, len(m2p.trace(grid, RES, angle=TILT)[0])))
m2p.MAX_DEPTH, m2p.MIN_AREA_M2, m2p.FILL_OK, m2p.MAX_RECTS = 8, 0.06, 0.68, 2000
for (na, ca), (nb, cb) in zip(counts, counts[1:]):
    check("%s is strictly finer than %s" % (nb, na), cb > ca,
          "%d vs %d pieces" % (cb, ca))

print("\n  EXACT IS EXACT")
ex_rects, ex_angle = m2p.trace_exact(grid, RES)
cover = np.zeros(grid.shape, bool)
gy, gx = np.mgrid[0:grid.shape[0], 0:grid.shape[1]]
cpx, cpy = (gx + 0.5) * RES, (gy + 0.5) * RES
for r in ex_rects:
    cover |= ((np.abs(cpx - r["x"]) <= r["w"] / 2) &
              (np.abs(cpy - r["y"]) <= r["h"] / 2))
occupied = grid < 100
check("every occupied cell is covered", int((occupied & ~cover).sum()) == 0,
      "%d missed of %d" % (int((occupied & ~cover).sum()), int(occupied.sum())))
check("and nothing else is", int((cover & ~occupied).sum()) == 0,
      "%d invented" % int((cover & ~occupied).sum()))
check("it is not one rectangle per cell", len(ex_rects) < occupied.sum() / 2,
      "%d pieces for %d cells -- runs merge" % (len(ex_rects), int(occupied.sum())))
check("exact beats every approximate level", len(ex_rects) > counts[-1][1],
      "%d vs %d at fine" % (len(ex_rects), counts[-1][1]))
check("and it does not rotate the room", ex_angle == 0.0,
      "rotating would resample the grid and stop it being exact")

print("")
print("=" * 72)
print("%d passed, %d failed" % (ok, fails))
print("=" * 72)
sys.exit(1 if fails else 0)
