"""
Turn the planner's static map into a ROS occupancy grid: map.pgm + map.yaml + goals.yaml.

    python3 export_ros_map.py plan.static-map.json
    python3 export_ros_map.py plan.static-map.json --base ilounge.pgm --base-yaml ilounge.yaml

This is what `map_server` / `nav2_map_server` loads directly:

    ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml

WHAT IT IS FOR
    A prior, so auto-nav has somewhere to start before the robot has seen the room. The
    robot localises against it and improves it with its own 3D lidar as it drives.
    Nothing here is surveyed; positions are good to roughly a hand's width.

TWO MODES

    Bare (no --base)
        Draws a rectangular room with walls, from the planner's own dimensions. Fine for
        a ballroom you have measured, useless for a room with any real structure.

    Composited (--base)
        Starts from a map somebody already has -- a lidar SLAM run, a surveyed floor plan
        -- and draws the tables onto it, keeping its resolution, origin and extent
        EXACTLY. The output is a map the robot's stack already agrees with, plus tonight's
        furniture.

        This is the honest split, and it is how it should work in the hall too: walls and
        pillars are permanent and a lidar sees them far better than a ceiling camera ever
        will; tables change every event and are what the camera is for. Redrawing the
        structure from our own guess throws away good data and tests nothing.

THE FRAME, WHICH IS THE PART THAT BITES
    The planner measures from the NORTH-WEST corner, x east, y SOUTH -- the way you read
    a floor plan, and the way an image is rastered. ROS maps are x east, y NORTH, with
    the yaml `origin` giving the pose of the image's LOWER-LEFT pixel. Those two are not
    the same frame; one is a mirror of the other.

        X_ros = origin_x + x_planner
        Y_ros = origin_y + (map_height_m - y_planner)
        theta_ros = -theta_planner

    Image rows are written top-down in planner order, which is what map_server wants
    because it flips the image back on load. Get this wrong and everything still looks
    plausible -- right size, right tables -- but the robot drives to the mirror image of
    every goal. goals.yaml is written in the SAME frame as the map, already converted.

PIXEL VALUES
    The ROS convention, with negate: 0 in the yaml:

        0    occupied   tables, fixtures the robot cannot drive through, walls
        254  free       floor
        205  unknown    never seen, or outside the room

NOT INFLATED
    Obstacles are drawn at their true footprint. Inflating them by the robot radius is
    the local costmap's job, and doing it twice makes a room look impassable.
"""

import argparse
import json
import os
import sys

import numpy as np

OCCUPIED, FREE, UNKNOWN = 0, 254, 205

DEFAULT_RES = 0.05      # metres per pixel; the ROS default, and plenty for a prior
WALL_M = 0.10           # how thick to draw the room's own walls, when we draw them


# ---------------------------------------------------------------------------
# reading somebody else's map
# ---------------------------------------------------------------------------
def read_pgm(path):
    """A PGM, binary (P5) or ASCII (P2), as a uint8 array. Row 0 is the top."""
    with open(path, "rb") as f:
        raw = f.read()
    if raw[:2] not in (b"P5", b"P2"):
        raise SystemExit("%s is not a PGM (expected P5 or P2)" % path)
    binary = raw[:2] == b"P5"
    fields, i = [], 2
    while len(fields) < 3:
        while i < len(raw) and raw[i:i + 1].isspace():
            i += 1
        if raw[i:i + 1] == b"#":                     # comments may sit between fields
            while i < len(raw) and raw[i:i + 1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        fields.append(raw[i:j])
        i = j
    w, h, maxv = (int(x) for x in fields)
    if maxv > 255:
        raise SystemExit("%s uses 16-bit samples; only 8-bit maps are supported" % path)
    if binary:
        body = raw[i + 1:]
        if len(body) < w * h:
            raise SystemExit("%s is truncated: %d bytes for %d pixels"
                             % (path, len(body), w * h))
        return np.frombuffer(body[:w * h], dtype=np.uint8).reshape(h, w).copy()
    vals = np.array(raw[i:].split(), dtype=np.int32)[:w * h]
    return vals.astype(np.uint8).reshape(h, w)


def read_map_yaml(path):
    """Just enough of a map yaml: image, resolution, origin, negate.

    Hand-parsed on purpose, so this runs anywhere without pyyaml -- but it has to read
    BOTH ways a map yaml writes the origin, because tools differ:

        origin: [-30.5156, -5.8514, 0.0]          # inline, what map_saver used to write
        origin:                                   # a block sequence, what nav2 writes
        - -30.5156
        - -5.8514
        - 0.0

    Reading only the first form is worse than failing: `origin:` with nothing after it
    parses as empty, and an empty origin silently becomes [0, 0, 0]. Every goal is then
    out by however far the map's real origin was -- thirty metres, in the first real file
    this met -- and nothing anywhere reports a problem. So a missing or unparseable
    origin raises.
    """
    with open(path) as f:
        lines = [ln.split("#", 1)[0].rstrip() for ln in f]

    out, i = {}, 0
    while i < len(lines):
        line = lines[i]
        if not line.strip() or ":" not in line:
            i += 1
            continue
        k, v = line.split(":", 1)
        k, v = k.strip(), v.strip()
        if v == "":
            seq, j = [], i + 1
            while j < len(lines) and lines[j].strip().startswith("- "):
                seq.append(lines[j].strip()[2:].strip())
                j += 1
            if seq:
                out[k] = seq
                i = j
                continue
        out[k] = v
        i += 1

    if "resolution" not in out:
        raise SystemExit("%s has no resolution -- is it a map yaml?" % path)

    raw = out.get("origin")
    if isinstance(raw, list):
        nums = raw
    elif isinstance(raw, str) and raw.strip():
        nums = raw.strip("[]").replace(",", " ").split()
    else:
        nums = []
    try:
        org = [float(x) for x in nums]
    except ValueError:
        org = []
    if len(org) < 2:
        raise SystemExit(
            "%s has no usable origin. It must be either\n"
            "    origin: [x, y, yaw]\n"
            "or a block sequence of three numbers under `origin:`.\n"
            "Refusing to assume [0, 0, 0]: that would silently shift every goal."
            % path)
    while len(org) < 3:
        org.append(0.0)

    return {"image": out.get("image", ""), "resolution": float(out["resolution"]),
            "origin": org, "negate": int(out.get("negate", 0) or 0),
            "occupied_thresh": float(out.get("occupied_thresh", 0.65) or 0.65),
            "free_thresh": float(out.get("free_thresh", 0.196) or 0.196),
            "mode": out.get("mode", "trinary")}


# ---------------------------------------------------------------------------
# drawing
# ---------------------------------------------------------------------------
def rasterise(plan, resolution=DEFAULT_RES, wall_m=WALL_M, base=None):
    """The occupancy grid, row 0 = the planner's y = 0 (the north edge).

    With `base`, the tables are drawn onto that grid and its size is kept; without it, a
    bare rectangular room is drawn from the plan's own dimensions.
    """
    room = plan["room"]
    # A layout traced from somebody's lidar map already HAS its walls, as objects. Adding
    # a rectangle round the outside would wall off the corridor the real room opens on to.
    traced = any(o.get("kind") == "wall" for o in plan.get("obstacles", []))
    if base is not None:
        grid = base.copy()
        h_px, w_px = grid.shape
        draw_walls = False
    else:
        w_px = int(round(float(room["width_m"]) / resolution))
        h_px = int(round(float(room["height_m"]) / resolution))
        grid = np.full((h_px, w_px), FREE, dtype=np.uint8)
        draw_walls = not traced
    w_m, h_m = w_px * resolution, h_px * resolution

    ys, xs = np.mgrid[0:h_px, 0:w_px]
    px = (xs + 0.5) * resolution          # pixel centres, in planner metres
    py = (ys + 0.5) * resolution

    if draw_walls:
        wall = max(int(round(wall_m / resolution)), 1)
        grid[:wall, :] = OCCUPIED
        grid[-wall:, :] = OCCUPIED
        grid[:, :wall] = OCCUPIED
        grid[:, -wall:] = OCCUPIED

    def stamp(shape):
        cx, cy = shape["centre_m"]
        if shape.get("shape") == "circle":
            grid[np.hypot(px - cx, py - cy) <= float(shape["radius_m"])] = OCCUPIED
        else:
            ow, oh = shape["size_m"]
            a = -np.radians(float(shape.get("rotation_deg", 0)))
            dx, dy = px - cx, py - cy
            u = dx * np.cos(a) - dy * np.sin(a)
            v = dx * np.sin(a) + dy * np.cos(a)
            grid[(np.abs(u) <= ow / 2) & (np.abs(v) <= oh / 2)] = OCCUPIED

    for o in plan.get("obstacles", []):
        stamp(o)
    # An access point that is furniture (the dish return) is solid too; a doorway is not.
    for a in plan.get("access_points", []):
        if a.get("solid"):
            stamp(a)

    return grid, w_m, h_m


def write_pgm(path, grid):
    """Binary (P5) PGM. Row 0 is written first, which map_server reads as the top."""
    h, w = grid.shape
    with open(path, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# generated by the Banquet Hall Planner -- a prior, not a survey\n")
        f.write(("%d %d\n255\n" % (w, h)).encode("ascii"))
        f.write(grid.tobytes())


def write_yaml(path, image_name, resolution, origin):
    with open(path, "w") as f:
        f.write(
            "image: %s\n"
            "resolution: %.6f\n"
            # the pose of the image's LOWER-LEFT pixel
            "origin: [%.6f, %.6f, %.6f]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
            % (image_name, resolution, origin[0], origin[1], origin[2]))


def to_ros(x, y, h_m, origin):
    """Planner metres (y south from the north-west) -> map metres (y north from the
    lower-left pixel named by the yaml's origin)."""
    return float(origin[0] + x), float(origin[1] + h_m - y)


def write_goals(path, plan, h_m, origin):
    """Named service stations, already in the map frame."""
    room = plan.get("room", {})
    lines = [
        "# Service stations from the Banquet Hall Planner.",
        "# Same frame as map.yaml: x east, y north, origin at the map's lower-left",
        "# pixel. Already converted from the planner's own y-south convention.",
        "frame_id: map",
        "room: %s" % json.dumps(room.get("name", "")),
        "generated_at: %s" % json.dumps(plan.get("generated_at", "")),
        "goals:",
    ]
    kept = 0
    for st in plan.get("stations", []):
        if not st.get("reachable", True):
            continue          # the planner already knows the robot cannot stand there
        gx, gy = to_ros(st["stand_m"][0], st["stand_m"][1], h_m, origin)
        lines.append("  - name: %s" % json.dumps(st["goal"]))
        lines.append("    x: %.3f" % gx)
        lines.append("    y: %.3f" % gy)
        lines.append("    yaw_deg: %.1f" % -float(st.get("heading_deg", 0.0)))
        kept += 1
    blocked = sum(1 for s in plan.get("stations", []) if not s.get("reachable", True))
    if blocked:
        lines.append("# %d station(s) left out: the planner says the robot cannot stand"
                     % blocked)
        lines.append("# there. Fix the layout rather than the export.")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return kept


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("plan", help="the planner's static-map JSON")
    ap.add_argument("-o", "--out", default=".", help="directory to write into")
    ap.add_argument("-r", "--resolution", type=float, default=DEFAULT_RES,
                    help="metres per pixel when drawing a bare room (default %.2f)"
                         % DEFAULT_RES)
    ap.add_argument("-n", "--name", default="map", help="base filename (default map)")
    ap.add_argument("--origin", type=float, nargs=2, metavar=("X", "Y"),
                    help="where the map's lower-left pixel sits in the robot's frame. "
                         "Needed when generating a map for a room somebody else has "
                         "already mapped, so both agree on where the origin is.")
    ap.add_argument("--base", help="an existing .pgm to draw the tables onto")
    ap.add_argument("--base-yaml", help="that map's .yaml (required with --base)")
    args = ap.parse_args(argv)

    with open(args.plan) as f:
        plan = json.load(f)
    if not str(plan.get("schema", "")).startswith("banquet-static-map/"):
        print("warning: %s does not look like a planner export" % args.plan,
              file=sys.stderr)

    base, resolution, origin = None, args.resolution, [0.0, 0.0, 0.0]
    if args.origin:
        origin = [float(args.origin[0]), float(args.origin[1]), 0.0]
    if args.base:
        if not args.base_yaml:
            ap.error("--base needs --base-yaml: the resolution and origin live there")
        if args.origin:
            raise SystemExit("--origin and --base disagree by construction: a base map "
                             "already says where its origin is. Drop --origin.")
        meta = read_map_yaml(args.base_yaml)
        base = read_pgm(args.base)
        resolution, origin = meta["resolution"], meta["origin"]
        if meta["negate"]:
            # negate:1 inverts the occupancy reading; rather than guess, refuse.
            raise SystemExit("%s has negate: 1, which this does not handle -- "
                             "re-export it with negate: 0" % args.base_yaml)
        print("base %s  %d x %d px at %.4f m/px  origin [%.3f, %.3f]"
              % (args.base, base.shape[1], base.shape[0], resolution,
                 origin[0], origin[1]))
        room = plan.get("room", {})
        want = (base.shape[1] * resolution, base.shape[0] * resolution)
        have = (float(room.get("width_m", 0)), float(room.get("height_m", 0)))
        if abs(want[0] - have[0]) > 0.05 or abs(want[1] - have[1]) > 0.05:
            print("warning: the plan's room is %.2f x %.2f m but the base map covers "
                  "%.2f x %.2f m.\n"
                  "         Tables are placed relative to the map's north-west corner, "
                  "so set the\n"
                  "         planner's room to the base map's size or they will land in "
                  "the wrong place."
                  % (have[0], have[1], want[0], want[1]), file=sys.stderr)

    os.makedirs(args.out, exist_ok=True)
    grid, w_m, h_m = rasterise(plan, resolution, base=base)
    pgm = os.path.join(args.out, args.name + ".pgm")
    yml = os.path.join(args.out, args.name + ".yaml")
    goals = os.path.join(args.out, args.name + "_goals.yaml")
    write_pgm(pgm, grid)
    write_yaml(yml, args.name + ".pgm", resolution, origin)
    n_goals = write_goals(goals, plan, h_m, origin)

    occ = int((grid == OCCUPIED).sum())
    unk = int((grid == UNKNOWN).sum())
    print("%s  %d x %d px at %.4f m/px  (%.1f x %.1f m)"
          % (pgm, grid.shape[1], grid.shape[0], resolution, w_m, h_m))
    print("%s  origin [%.3f, %.3f, %.3f]" % (yml, origin[0], origin[1], origin[2]))
    print("%s  %d reachable stations, converted to the map frame" % (goals, n_goals))
    print("   %.0f%% occupied, %.0f%% unknown, %.0f%% free"
          % (100.0 * occ / grid.size, 100.0 * unk / grid.size,
             100.0 * (grid.size - occ - unk) / grid.size))
    print("\nros2 run nav2_map_server map_server --ros-args -p yaml_filename:=%s"
          % os.path.abspath(yml))
    return 0


if __name__ == "__main__":
    sys.exit(main())
