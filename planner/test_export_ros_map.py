"""
Checks for the ROS map export. No ROS needed.

    python3 test_export_ros_map.py

The one that matters is the frame. The planner measures y SOUTH from the north-west
corner; a ROS map measures y NORTH from the south-west. Both produce a map of the right
size with tables of the right size, so a mirrored export looks completely fine and simply
sends the robot to the wrong side of the room. So these tests do not check the image --
they ask the question the nav stack asks: "what is at this world coordinate?"

The composite tests use a base map with a NON-ZERO origin, which is where offset bugs
hide: with origin [0, 0] a forgotten offset is invisible.
"""

import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np

import export_ros_map as ex

ok = fails = 0


def check(label, cond, detail=""):
    global ok, fails
    if cond:
        ok += 1
        print("  PASS  %-54s %s" % (label, detail))
    else:
        fails += 1
        print("  FAIL  %-54s %s" % (label, detail))


def read_yaml(path):
    out = {}
    with open(path) as f:
        for line in f:
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                out[k.strip()] = v.strip()
    return out


def load_goals(path):
    goals, cur = {}, None
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s.startswith("- name:"):
                cur = json.loads(s.split(":", 1)[1].strip())
                goals[cur] = {}
            elif cur and ":" in s and not s.startswith("#"):
                k, v = s.split(":", 1)
                if k in ("x", "y", "yaw_deg"):
                    goals[cur][k] = float(v)
    return goals


def probe(grid, res, origin):
    """world (X, Y) -> pixel value, exactly as map_server would index it."""
    h_px = grid.shape[0]

    def at(X, Y):
        c = int(round((X - origin[0]) / res - 0.5))
        r = int(round(h_px - 0.5 - (Y - origin[1]) / res))
        return int(grid[r, c])
    return at


NAME = {ex.OCCUPIED: "occupied", ex.FREE: "free", ex.UNKNOWN: "unknown"}

# A small plan in the planner's real schema, spread out so nothing is degenerate.
PLAN = {
    "schema": "banquet-static-map/0.1",
    "generated_at": "2026-09-30T06:41:46.177Z",
    "room": {"id": "ballroom", "name": "Grand Ballroom",
             "width_m": 20, "height_m": 21,
             "origin": "north-west corner, x east, y south"},
    "table_height_m": 0.762,
    "obstacles": [
        {"id": "T01", "kind": "table", "shape": "circle",
         "centre_m": [5.0, 4.0], "radius_m": 0.91},
        {"id": "T02", "kind": "table", "shape": "circle",
         "centre_m": [15.0, 17.0], "radius_m": 0.76},
        {"id": "Stage", "kind": "stage", "shape": "rect", "centre_m": [10.0, 1.9],
         "size_m": [8.0, 3.0], "rotation_deg": 0, "solid": True},
    ],
    "zones": [{"id": "Dance floor", "kind": "dance", "shape": "rect",
               "centre_m": [10.0, 8.0], "size_m": [6.0, 4.0], "rotation_deg": 0}],
    "access_points": [
        {"id": "Service door", "kind": "door", "shape": "rect", "centre_m": [0.16, 17.0],
         "size_m": [1.2, 0.25], "rotation_deg": 90, "solid": False},
        {"id": "Dish return", "kind": "dish", "shape": "rect", "centre_m": [1.3, 19.6],
         "size_m": [1.5, 0.8], "rotation_deg": 0, "solid": True},
    ],
    "stations": [
        # north of T01, so it faces south: +90 in the planner's y-south frame
        {"goal": "T01/cover01", "stand_m": [5.0, 2.64], "heading_deg": 90,
         "reachable": True},
        # west of T01, facing east
        {"goal": "T01/cover04", "stand_m": [3.64, 4.0], "heading_deg": 0,
         "reachable": True},
        {"goal": "T02/cover01", "stand_m": [15.0, 15.79], "heading_deg": 90,
         "reachable": False, "blocked_by": "the wall"},
    ],
}

RES = 0.05
OUT = tempfile.mkdtemp(prefix="rosmap")
try:
    with open(os.path.join(OUT, "plan.json"), "w") as f:
        json.dump(PLAN, f)
    rc = ex.main([os.path.join(OUT, "plan.json"), "-o", OUT, "-r", str(RES)])
    print("")
    check("the exporter ran", rc == 0)

    grid = ex.read_pgm(os.path.join(OUT, "map.pgm"))
    y = read_yaml(os.path.join(OUT, "map.yaml"))
    origin = [float(v) for v in y["origin"].strip("[]").replace(",", " ").split()]
    at = probe(grid, RES, origin)
    H_M = PLAN["room"]["height_m"]
    ros = lambda x, yy: (x, H_M - yy)          # planner -> map frame

    check("size matches the room at this resolution", grid.shape == (420, 400),
          "%d x %d px for 20.0 x 21.0 m at %.2f" % (grid.shape[1], grid.shape[0], RES))
    check("yaml points at the image", y["image"] == "map.pgm")
    check("yaml resolution matches", abs(float(y["resolution"]) - RES) < 1e-9)
    check("yaml origin is the south-west corner", origin == [0.0, 0.0, 0.0])
    check("negate 0, so 0 means occupied", y["negate"] == "0")
    check("thresholds are the ROS defaults",
          abs(float(y["occupied_thresh"]) - 0.65) < 1e-9 and
          abs(float(y["free_thresh"]) - 0.196) < 1e-9)

    print("")
    print("  ASKING THE MAP WHAT IS WHERE, IN THE MAP'S OWN FRAME")
    for name, (px_, py_), want in [
        ("T01's centre", (5.0, 4.0), ex.OCCUPIED),
        ("T02's centre", (15.0, 17.0), ex.OCCUPIED),
        ("the middle of the stage", (10.0, 1.9), ex.OCCUPIED),
        ("the dish return", (1.3, 19.6), ex.OCCUPIED),
        ("open floor", (7.0, 12.0), ex.FREE),
        ("the dance floor (crossable)", (10.0, 8.0), ex.FREE),
        ("the doorway (not solid)", (0.16, 17.0), ex.FREE),
        ("just outside T01's rim", (5.0, 4.0 + 0.91 + 0.20), ex.FREE),
    ]:
        X, Y = ros(px_, py_)
        got = at(X, Y)
        check(name, got == want,
              "(%.2f, %.2f) in map frame -> %s" % (X, Y, NAME.get(got, got)))

    check("a mirrored lookup finds nothing there", at(5.0, 4.0) == ex.FREE,
          "no phantom table at the mirror of T01")
    check("the west wall is solid", at(*ros(0.02, 10.0)) == ex.OCCUPIED)
    check("the south wall is solid", at(*ros(10.0, 20.98)) == ex.OCCUPIED)

    goals = load_goals(os.path.join(OUT, "map_goals.yaml"))
    check("unreachable stations are left out", "T02/cover01" not in goals,
          "the planner already knows the robot cannot stand there")
    g = goals["T01/cover01"]
    wx, wy = ros(5.0, 2.64)
    check("a goal's position is in the map frame",
          abs(g["x"] - wx) < 0.01 and abs(g["y"] - wy) < 0.01,
          "%.2f, %.2f (planner had 5.00, 2.64)" % (g["x"], g["y"]))
    check("and its heading is flipped with it", abs(g["yaw_deg"] + 90.0) < 0.01,
          "%.1f deg (planner had +90)" % g["yaw_deg"])
    tx, ty = ros(5.0, 4.0)
    facing = math.degrees(math.atan2(ty - g["y"], tx - g["x"]))
    check("it still faces the table it serves", abs(facing - g["yaw_deg"]) < 0.5,
          "table lies at %.1f deg, robot faces %.1f deg" % (facing, g["yaw_deg"]))
    bad = [n for n, gg in goals.items() if at(gg["x"], gg["y"]) != ex.FREE]
    check("every exported goal stands on free floor", not bad,
          ", ".join(bad) or "all of them")

    # =====================================================================
    # COMPOSITING onto somebody else's map -- a lidar run of the lab
    # =====================================================================
    print("")
    print("  COMPOSITING ONTO A MAP THAT ALREADY EXISTS")
    B_RES = 0.05
    B_ORIGIN = [-1.5, -3.25, 0.0]          # deliberately not the origin
    BW_M, BH_M = 12.0, 8.0
    bw, bh = int(BW_M / B_RES), int(BH_M / B_RES)
    base = np.full((bh, bw), ex.FREE, dtype=np.uint8)
    base[:4, :] = base[-4:, :] = ex.OCCUPIED       # walls, 0.2 m
    base[:, :4] = base[:, -4:] = ex.OCCUPIED
    # a pillar at planner (9.0, 2.0), and an unseen corner
    ys, xs = np.mgrid[0:bh, 0:bw]
    bpx, bpy = (xs + 0.5) * B_RES, (ys + 0.5) * B_RES
    base[np.hypot(bpx - 9.0, bpy - 2.0) <= 0.3] = ex.OCCUPIED
    base[(bpx > 10.5) & (bpy > 6.0)] = ex.UNKNOWN
    ex.write_pgm(os.path.join(OUT, "lab.pgm"), base)
    ex.write_yaml(os.path.join(OUT, "lab.yaml"), "lab.pgm", B_RES, B_ORIGIN)

    LAB = {
        "schema": "banquet-static-map/0.1",
        "room": {"id": "custom", "name": "iLounge", "width_m": BW_M, "height_m": BH_M,
                 "origin": "north-west corner, x east, y south"},
        "obstacles": [{"id": "T01", "kind": "table", "shape": "circle",
                       "centre_m": [3.0, 2.0], "radius_m": 0.91}],
        "zones": [], "access_points": [],
        "stations": [{"goal": "T01/cover03", "stand_m": [3.0, 3.36],
                      "heading_deg": -90, "reachable": True}],
    }
    with open(os.path.join(OUT, "lab_plan.json"), "w") as f:
        json.dump(LAB, f)
    rc = ex.main([os.path.join(OUT, "lab_plan.json"), "-o", OUT, "-n", "out",
                  "--base", os.path.join(OUT, "lab.pgm"),
                  "--base-yaml", os.path.join(OUT, "lab.yaml")])
    print("")
    check("the composite ran", rc == 0)

    out_grid = ex.read_pgm(os.path.join(OUT, "out.pgm"))
    oy = read_yaml(os.path.join(OUT, "out.yaml"))
    o_origin = [float(v) for v in oy["origin"].strip("[]").replace(",", " ").split()]
    bat = probe(out_grid, B_RES, o_origin)
    bros = lambda x, yy: (B_ORIGIN[0] + x, B_ORIGIN[1] + BH_M - yy)

    check("the base map's size is kept exactly", out_grid.shape == base.shape,
          "%d x %d px" % (out_grid.shape[1], out_grid.shape[0]))
    check("its resolution is kept", abs(float(oy["resolution"]) - B_RES) < 1e-9)
    check("its origin is carried through, not reset",
          all(abs(a - b) < 1e-6 for a, b in zip(o_origin, B_ORIGIN)),
          "[%.3f, %.3f]" % (o_origin[0], o_origin[1]))

    check("the lidar's pillar survives", bat(*bros(9.0, 2.0)) == ex.OCCUPIED,
          "we composited onto his map, we did not redraw it")
    check("the lidar's walls survive", bat(*bros(0.1, 4.0)) == ex.OCCUPIED)
    check("unseen floor stays unknown, not invented as free",
          bat(*bros(11.5, 7.0)) == ex.UNKNOWN)
    check("our table lands on it", bat(*bros(3.0, 2.0)) == ex.OCCUPIED)
    check("and only where it should be", bat(*bros(6.0, 4.0)) == ex.FREE)
    check("no phantom table at the un-flipped position",
          bat(B_ORIGIN[0] + 3.0, B_ORIGIN[1] + 2.0) == ex.FREE)

    lg = load_goals(os.path.join(OUT, "out_goals.yaml"))["T01/cover03"]
    gx, gy = bros(3.0, 3.36)
    check("the goal carries the base map's origin offset",
          abs(lg["x"] - gx) < 0.01 and abs(lg["y"] - gy) < 0.01,
          "%.2f, %.2f (planner had 3.00, 3.36)" % (lg["x"], lg["y"]))
    check("the goal stands on free floor", bat(lg["x"], lg["y"]) == ex.FREE)
    tx2, ty2 = bros(3.0, 2.0)
    f2 = math.degrees(math.atan2(ty2 - lg["y"], tx2 - lg["x"]))
    check("and still faces its table", abs(f2 - lg["yaw_deg"]) < 0.5,
          "%.1f deg" % lg["yaw_deg"])

    # a map read back must be identical to what went out
    check("PGM survives a write/read round trip",
          np.array_equal(ex.read_pgm(os.path.join(OUT, "lab.pgm")), base))

    # =====================================================================
    # YAML dialects. Both of these are real; the second is what nav2 writes,
    # and reading it as empty silently puts every goal 30 m out.
    # =====================================================================
    print("")
    print("  MAP YAML, WRITTEN BOTH WAYS")
    INLINE = ("image: m.pgm\nresolution: 0.05\n"
              "origin: [-30.515602111816406, -5.851429462432861, 0.0]\n"
              "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    BLOCK = ("image: m.pgm\nresolution: 0.05\norigin:\n"
             "- -30.515602111816406\n- -5.851429462432861\n- 0.0\n"
             "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    WANT = [-30.515602111816406, -5.851429462432861, 0.0]
    for label, text in (("inline [x, y, yaw]", INLINE), ("a block sequence", BLOCK)):
        fp = os.path.join(OUT, "dialect.yaml")
        with open(fp, "w") as f:
            f.write(text)
        meta = ex.read_map_yaml(fp)
        check(label, meta["origin"] == WANT,
              "origin %.4f, %.4f" % (meta["origin"][0], meta["origin"][1]))
    check("a trailing `mode:` does not confuse it",
          ex.read_map_yaml(fp).get("mode") == "trinary")

    # and the failure that must be loud rather than silent
    fp = os.path.join(OUT, "noorigin.yaml")
    with open(fp, "w") as f:
        f.write("image: m.pgm\nresolution: 0.05\norigin:\nnegate: 0\n")
    try:
        ex.read_map_yaml(fp)
        check("an empty origin is refused, not read as [0, 0, 0]", False,
              "it returned a value")
    except SystemExit as e:
        check("an empty origin is refused, not read as [0, 0, 0]",
              "0, 0, 0" in str(e) or "origin" in str(e).lower(),
              "raises instead of shifting every goal")

    # the real file from the lab, kept beside this test
    here = os.path.dirname(os.path.abspath(__file__))
    real = os.path.join(here, "ilounge_02_dt_map.yaml")
    real = real if os.path.exists(real) else None
    if real:
        meta = ex.read_map_yaml(real)
        check("the iLounge map's own yaml reads correctly",
              abs(meta["origin"][0] + 30.5156) < 1e-3 and
              abs(meta["origin"][1] + 5.8514) < 1e-3,
              "origin [%.4f, %.4f] at %.3f m/px"
              % (meta["origin"][0], meta["origin"][1], meta["resolution"]))

finally:
    shutil.rmtree(OUT, ignore_errors=True)

print("")
print("=" * 74)
print("%d passed, %d failed" % (ok, fails))
print("=" * 74)
sys.exit(1 if fails else 0)
