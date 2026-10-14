"""
Offline checks for the depth table detector. No camera, no robot, no model weights.

    python3 test_table_segmentation.py

Scenes are ray-cast from real projective geometry, so a tilted table lands as the ellipse
a real camera would see. That matters: the mistake this detector exists to avoid is
fitting a circle to that ellipse, and a scene drawn with a circle-drawing function cannot
catch it.
"""

import sys

import numpy as np

import table_segmentation as ts

ok = fails = 0


def check(label, cond, detail=""):
    global ok, fails
    if cond:
        ok += 1
        print("  PASS  %-56s %s" % (label, detail))
    else:
        fails += 1
        print("  FAIL  %-56s %s" % (label, detail))


def run(**kw):
    _, depth, intr, scale, truth = ts.synthetic(**kw)
    found, plane, _ = ts.detect_tables(depth, intr, scale)
    return found, truth


print("\nTWO TABLES, CAMERA STRAIGHT DOWN AT 3.2 m")
found, truth = run()
check("both tables found", len(found) == 2, "%d found" % len(found))
if len(found) == 2:
    for t, g in zip(sorted(found, key=lambda x: x["centre_px"][0]),
                    sorted(truth, key=lambda x: x["offset"])):
        err = (t["measured"]["diameter_m"] - g["diameter"]) * 1000
        check("%.2f m table measured within 25 mm" % g["diameter"], abs(err) < 25,
              "%+.0f mm" % err)
        check("  matched to the right stock in the hall",
              abs(ts.STOCK_M[t["measured"]["nearest_stock"]] - g["diameter"]) < 0.01,
              t["measured"]["nearest_stock"])
    check("sits 0.762 m above the floor",
          all(abs(t["measured"]["height_above_floor_m"] - 0.762) < 0.03 for t in found),
          "%.3f m" % found[0]["measured"]["height_above_floor_m"])

print("\nTWELVE CHAIRS ROUND THOSE TABLES ARE NOT TABLES")
check("nothing but the two tops survives", len(found) == 2,
      "a chair back passes straight through the height band")
no_chairs, _ = run(chairs=False)
check("and removing the chairs changes nothing", len(no_chairs) == 2,
      "%d found without them" % len(no_chairs))

print("\nAN OBLIQUE CAMERA -- a round table projects as an ELLIPSE")
for tilt in (12.0, 22.0, 32.0):
    f, g = run(tilt_deg=tilt, chairs=False)
    if not f:
        check("%.0f deg off vertical" % tilt, False, "nothing found")
        continue
    worst = max(abs(t["measured"]["diameter_m"] -
                    min(g, key=lambda q: abs(q["diameter"] - t["measured"]["diameter_m"]))["diameter"])
                for t in f)
    ratio = min(t["axis_ratio"] for t in f)
    check("%.0f deg off vertical, %d found, within 40 mm" % (tilt, len(f)),
          len(f) == 2 and worst < 0.040,
          "worst %+.0f mm, image squashed to %.0f%%" % (worst * 1000, ratio * 100))

print("\nTHE LAB FAILURE: the table IS the biggest plane in view")
# One 7 ft round at 2.2 m fills about three quarters of the frame, so the dominant
# plane is the tabletop and every table then measures 0.00 m above "the floor".
_, depth, intr, scale, truth = ts.synthetic(camera_height=2.2, diameters=(2.1336,),
                                            chairs=False)
n_dom, c_dom = ts.pd.dominant_plane(depth, intr, scale)
h_dom = ts.pd.height_above_plane(depth, intr, (n_dom, c_dom), scale)
frac_below = float(np.mean(h_dom[np.isfinite(h_dom)] < -0.25))
check("the naive dominant plane really is the tabletop", frac_below > 0.15,
      "%.0f%% of the frame lies below it" % (100 * frac_below))
n_f, c_f, corrected = ts.floor_plane(depth, intr, scale)
check("floor_plane() notices and re-fits", corrected)
found1, _ = run(camera_height=2.2, diameters=(2.1336,), chairs=False)
check("and the table is then found", len(found1) == 1, "%d found" % len(found1))
if found1:
    err = (found1[0]["measured"]["diameter_m"] - 2.1336) * 1000
    check("  measured within 30 mm", abs(err) < 30, "%+.0f mm" % err)
    check("  still 0.762 m above the floor",
          abs(found1[0]["measured"]["height_above_floor_m"] - 0.762) < 0.03,
          "%.3f m" % found1[0]["measured"]["height_above_floor_m"])

print("\nAN EMPTY ROOM IS EMPTY")
_, depth, intr, scale, _ = ts.synthetic(diameters=(), chairs=False)
empty, _, _ = ts.detect_tables(depth, intr, scale)
check("no tables invented on bare floor", len(empty) == 0, "%d found" % len(empty))

print("\nWHAT A D435 CAN ACTUALLY SEE")
intr = {"fx": 425.0, "fy": 425.0, "cx": 424.0, "cy": 240.0, "width": 848, "height": 480}
for hgt in (2.0, 2.5, 3.0, 3.5):
    vw, vh = ts.view_at(intr, hgt)
    fits = int(vw // (1.8288 + 0.8))
    print("       %.1f m up -> %.1f x %.1f m of floor, about %d six-foot rounds across"
          % (hgt, vw, vh, fits))
check("one table needs about 2.4 m of height", ts.view_at(intr, 2.4)[0] > 1.83 + 0.6,
      "%.1f m across at 2.4 m" % ts.view_at(intr, 2.4)[0])
check("two need about 3.2 m", ts.view_at(intr, 3.2)[0] > 2 * 1.8288 + 0.8,
      "%.1f m across at 3.2 m" % ts.view_at(intr, 3.2)[0])

print("\nTHE JSON THE PLANNER READS")
found, _ = run()
doc = ts.to_document(found, 848, 480)
check("schema the planner accepts", doc["schema"] == "banquet-detections/0.1")
check("units are pixels, so the homography is exercised", doc["units"] == "pixels")
check("every table has what the planner needs",
      all({"id", "shape", "centre_px", "diameter_px", "confidence"} <= set(t)
          for t in doc["tables"]))
check("and carries the metres this camera measured",
      all("diameter_m" in t["measured"] for t in doc["tables"]),
      "the ground truth an RGB detector gets scored against")

# ---------------------------------------------------------------------------
# The round trip, mirroring what the planner does in JavaScript: four clicked
# corners -> homography -> metres. Written out here rather than imported from
# the browser, because the point is to check that arithmetic independently.
# ---------------------------------------------------------------------------
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


def homography(corners, room):
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


def circle_fit(pts):
    M = [[0.0] * 3 for _ in range(3)]
    v = [0.0] * 3
    for x, y in pts:
        row, rhs = [x, y, 1.0], -(x * x + y * y)
        for i in range(3):
            for j in range(3):
                M[i][j] += row[i] * row[j]
            v[i] += row[i] * rhs
    sol = solve(M, v)
    if not sol:
        return None
    cx, cy = -sol[0] / 2, -sol[1] / 2
    r2 = cx * cx + cy * cy - sol[2]
    return (cx, cy, r2 ** 0.5) if r2 > 0 else None


print("")
print("THE ROUND TRIP -- pixels through the planner's homography, back to metres")
CAM_H = 5.0
_, depth, intr, scale, truth = ts.synthetic(camera_height=CAM_H, chairs=False)
found, _, _ = ts.detect_tables(depth, intr, scale)
# The room is exactly what this camera sees AT FLOOR LEVEL, so the frame's four
# corners ARE the room's four corners -- which is what the manager clicks.
W = intr["width"] / intr["fx"] * CAM_H
H = intr["height"] / intr["fy"] * CAM_H
hom = homography([(0, 0), (intr["width"], 0), (intr["width"], intr["height"]),
                  (0, intr["height"])], (W, H))
k = (CAM_H - ts.TABLE_HEIGHT_M) / CAM_H
nadir = (W / 2, H / 2)
check("both tables still detected at %.1f m" % CAM_H, len(found) == 2)

worst_raw = worst_fixed = worst_dia = 0.0
for t in found:
    g = min(truth, key=lambda q: abs(q["diameter"] - t["measured"]["diameter_m"]))
    want = (W / 2 + g["offset"], H / 2)
    raw = pix_to_room(hom, t["centre_px"][0], t["centre_px"][1])
    worst_raw = max(worst_raw, ((raw[0] - want[0]) ** 2 + (raw[1] - want[1]) ** 2) ** 0.5)
    ring = [pix_to_room(hom, u, v) for u, v in t["outline_px"]]
    ring = [(nadir[0] + (x - nadir[0]) * k, nadir[1] + (y - nadir[1]) * k) for x, y in ring]
    cf = circle_fit(ring)
    worst_fixed = max(worst_fixed, ((cf[0] - want[0]) ** 2 + (cf[1] - want[1]) ** 2) ** 0.5)
    worst_dia = max(worst_dia, abs(2 * cf[2] - g["diameter"]))

check("uncorrected parallax really does wreck it", worst_raw > 0.15,
      "%.2f m out of place" % worst_raw)
check("the rim, corrected, lands within 60 mm", worst_fixed < 0.060,
      "%.0f mm out" % (worst_fixed * 1000))
check("and its diameter within 60 mm", worst_dia < 0.060,
      "%.0f mm out" % (worst_dia * 1000))
check("the correction is the one the planner applies", abs(k - 0.8476) < 0.001,
      "x%.4f about the nadir" % k)

print("")
print("=" * 74)
print("%d passed, %d failed" % (ok, fails))
print("=" * 74)
sys.exit(1 if fails else 0)
