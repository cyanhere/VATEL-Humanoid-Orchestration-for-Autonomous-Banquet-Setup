"""
Check the planner's geometry, transcribed from index.html.

The browser pane serves a cached snapshot and reports a zero-size element, so it
cannot be trusted to measure a viewBox. These are the formulas I actually changed:
the staggered arrangement, the view fit, and the camera homography.
"""
import math

FT = 0.3048
ROBOT_RADIUS, STAND_OFF, MIN_AISLE = 0.30, 0.45, 1.50
ROOM = (20.0, 21.0)
STOCK = {"r5": 5 * FT, "r55": 5.5 * FT, "r6": 6 * FT, "r7": 7 * FT}

ok = fails = 0


def check(label, cond, detail=""):
    global ok, fails
    if cond:
        ok += 1
        print("  PASS  %-58s %s" % (label, detail))
    else:
        fails += 1
        print("  FAIL  %-58s %s" % (label, detail))


# ---------------------------------------------------------------- arrangeCells
def arrange_cells(w, aisle, stagger, room=ROOM, fixtures=()):
    pitch_x = w + aisle
    pitch_y = (w + aisle) * 0.87 if stagger else w + aisle
    margin = w / 2 + max(ROBOT_RADIUS, 0.35)
    uw, uh = room[0] - 2 * margin, room[1] - 2 * margin
    if uw < 0 or uh < 0:
        return []
    cols = int(uw / pitch_x) + 1
    rows = int(uh / pitch_y) + 1
    cells = []
    for r in range(rows):
        odd = stagger and r % 2 == 1
        n = cols - 1 if odd else cols
        if n < 1:
            continue
        span = (n - 1) * pitch_x
        x0 = margin + (uw - span) / 2
        y = margin + (uh - (rows - 1) * pitch_y) / 2 + r * pitch_y
        for c in range(n):
            x = x0 + c * pitch_x
            if x < margin - 1e-6 or x > room[0] - margin + 1e-6:
                continue
            if all(abs(x - fx) > w / 2 + 0.4 or abs(y - fy) > w / 2 + 0.4
                   for fx, fy in fixtures):
                cells.append((x, y))
    return cells


def tightest(cells, diameter):
    best = float("inf")
    for i in range(len(cells)):
        for j in range(i + 1, len(cells)):
            d = math.hypot(cells[i][0] - cells[j][0], cells[i][1] - cells[j][1])
            best = min(best, d - diameter)
    return best


print("\nSTAGGERED ARRANGEMENT -- the aisle you ask for is the aisle you get")
for key in ("r5", "r6", "r7"):
    w = STOCK[key]
    for aisle in (0.9, 1.2, 1.5, 2.0):
        cells = arrange_cells(w, aisle, True)
        g = tightest(cells, w)
        check("%-4s staggered at %.1f m" % (key, aisle), g >= aisle - 0.02,
              "%d tables, tightest %.3f m" % (len(cells), g))

print("\n...and a plain grid too")
for key in ("r6", "r7"):
    for aisle in (0.9, 1.5):
        w = STOCK[key]
        cells = arrange_cells(w, aisle, False)
        g = tightest(cells, w)
        check("%-4s grid at %.1f m" % (key, aisle), g >= aisle - 0.02,
              "%d tables, tightest %.3f m" % (len(cells), g))

print("\nStaggering has to earn its keep")
a, b = arrange_cells(STOCK["r6"], 1.5, True), arrange_cells(STOCK["r6"], 1.5, False)
check("stagger fits at least as many as a grid", len(a) >= len(b),
      "%d staggered vs %d square" % (len(a), len(b)))

print("\nEvery station has to be reachable at the trade aisle")
w = STOCK["r6"]
cells = arrange_cells(w, 1.5, True)
need = STAND_OFF + ROBOT_RADIUS          # rim to stand point, plus the footprint
check("a station clears the neighbouring rim", tightest(cells, w) >= need,
      "needs %.2f m, has %.3f m" % (need, tightest(cells, w)))

print("\nNothing lands on the stage")
fx = [(ROOM[0] / 2, 1.9)]
cells = arrange_cells(w, 1.5, True, fixtures=fx)
clear = all(abs(x - fx[0][0]) > w / 2 + 0.4 or abs(y - fx[0][1]) > w / 2 + 0.4
            for x, y in cells)
check("the stage cell is skipped", clear, "%d tables placed" % len(cells))


# -------------------------------------------------------------------- fitView
def fit_view(room, box_w, box_h, pad=1.2):
    want_w, want_h = room[0] + pad * 2, room[1] + pad * 2
    vw = max(want_w, want_h * (box_w / max(box_h, 1)))
    vh = vw * (box_h / box_w)
    return (room[0] / 2 - vw / 2, room[1] / 2 - vh / 2, vw, vh)


print("\nFIT -- the whole room is inside the viewBox at any window shape")
for bw, bh in [(1200, 700), (600, 800), (359, 128), (1600, 400), (900, 900), (320, 1000)]:
    x, y, vw, vh = fit_view(ROOM, bw, bh)
    inside = x <= 1e-9 and y <= 1e-9 and x + vw >= ROOM[0] - 1e-9 and y + vh >= ROOM[1] - 1e-9
    aspect_ok = abs((vw / vh) - (bw / bh)) < 1e-6
    check("%dx%d element" % (bw, bh), inside and aspect_ok,
          "viewBox %.1f x %.1f, aspect %s" % (vw, vh, "matches" if aspect_ok else "WRONG"))


# ----------------------------------------------------------------- homography
def solve(A, b):
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            return None
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r == c:
                continue
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


def homography(corners, room=ROOM):
    dst = [(0, 0), (room[0], 0), (room[0], room[1]), (0, room[1])]
    A, b = [], []
    for (u, v), (x, y) in zip(corners, dst):
        A.append([u, v, 1, 0, 0, 0, -x * u, -x * v]); b.append(x)
        A.append([0, 0, 0, u, v, 1, -y * u, -y * v]); b.append(y)
    h = solve(A, b)
    return h + [1] if h else None


def pix_to_room(h, u, v):
    w = h[6] * u + h[7] * v + 1
    return ((h[0] * u + h[1] * v + h[2]) / w, (h[3] * u + h[4] * v + h[5]) / w)


print("\nCAMERA HOMOGRAPHY")
# A camera looking straight down: pixels are just a scaled, offset copy of the room.
sq = [(100, 50), (1800, 50), (1800, 1000), (100, 1000)]
h = homography(sq)
check("four corners solve", h is not None)
if h:
    for (u, v), want in zip(sq, [(0, 0), (20, 0), (20, 21), (0, 21)]):
        got = pix_to_room(h, u, v)
        check("corner (%d,%d)" % (u, v), math.dist(got, want) < 1e-6,
              "-> %.3f, %.3f m" % got)
    mid = pix_to_room(h, 950, 525)
    check("the centre pixel lands mid-room", math.dist(mid, (10, 10.5)) < 1e-6,
          "-> %.3f, %.3f m" % mid)

# An oblique view: a real trapezoid, the case a corner-mounted camera actually gives.
trap = [(300, 200), (1600, 240), (1850, 950), (60, 900)]
h2 = homography(trap)
check("an oblique view solves", h2 is not None)
if h2:
    good = all(math.dist(pix_to_room(h2, u, v), w) < 1e-6
               for (u, v), w in zip(trap, [(0, 0), (20, 0), (20, 21), (0, 21)]))
    check("all four oblique corners land exactly", good)
    # A point inside the quad must land inside the room.
    inner = pix_to_room(h2, 950, 570)
    check("an interior pixel stays in the room",
          0 < inner[0] < 20 and 0 < inner[1] < 21, "-> %.2f, %.2f m" % inner)

print("\n" + "=" * 74)
print("%d passed, %d failed" % (ok, fails))
print("=" * 74)
raise SystemExit(1 if fails else 0)
