"""
Read a lidar map (.pgm + .yaml) and its point cloud (.ply), and find the tables.

    python3 inspect_lidar.py ilounge.pgm ilounge.yaml ilounge.ply
    python3 inspect_lidar.py ilounge.pgm ilounge.yaml ilounge.ply --table-height 0.75

Writes:
    <name>_view.png        the map, with what it thinks are tables drawn on it
    <name>_detections.json the planner's detections format, in METRES

WHY THIS EXISTS
    The ceiling camera is not installed, and one RealSense cannot see a whole room. But
    the robot's own Mid-360 has already been round the lab and its point cloud contains
    the real furniture, at real positions. Segmenting tables out of that gives exactly
    what a perfect ceiling camera would have produced -- so the rest of the chain (import
    -> review -> place -> export -> navigate) can be tested on real furniture in a real
    room, today.

    It is not a stand-in for the camera. It is a way to get truthful table positions
    without one.

HOW IT TELLS A TABLE FROM A WALL
    Height alone cannot: a wall occupies every height band, including the table's. What
    separates them is what is ABOVE. A tabletop has clear air over it; a wall, a cupboard
    or a pillar does not. So a cell counts as a tabletop when it is busy at table height
    and quiet well above it.

    The same trick the plate detector uses one storey down, where a plate is whatever
    stands proud of the table and nothing stands on the plate.

THE ROW ORDER, AGAIN
    A PGM's row 0 is the TOP of the image, and map_server treats the image's LOWER-left
    pixel as the yaml origin. So world y runs UP the image while the row index runs down:

        row = height_px - 1 - (y - origin_y) / resolution

    Rasterising the cloud the other way up produces a map that looks entirely reasonable
    and is a mirror of the truth.
"""

import argparse
import json
import os
import struct
import sys
import zlib

import numpy as np

import export_ros_map as ex

# ---------------------------------------------------------------------------
# what counts as a table
# ---------------------------------------------------------------------------
TABLE_Z = 0.75          # typical worktop/table height in a lab; banquet tops are 0.762
BAND = 0.15             # how far either side of that still counts as the top
CLEAR_LO, CLEAR_HI = 1.25, 2.20     # the air above a table, where a wall would not be
MIN_HITS = 2            # points in a cell before it counts, to shrug off stray returns
MAX_ABOVE = 1           # more than this overhead and it is furniture, not a top
MIN_AREA_M2 = 0.35      # smaller than this is a chair back or a bag
MAX_AREA_M2 = 12.0      # larger is a counter run or a row of desks, not one table


def read_ply_xyz(path):
    """x, y, z from an ASCII ply. Only the vertex block, only the first three fields."""
    with open(path, "r", errors="replace") as f:
        head, line = [], ""
        while True:
            line = f.readline()
            if not line:
                raise SystemExit("%s has no end_header" % path)
            head.append(line.strip())
            if line.strip() == "end_header":
                break
        body = f.read()
    if not any(h.startswith("format ascii") for h in head):
        raise SystemExit("%s is not an ASCII ply; only ASCII is supported" % path)
    props = [h.split()[-1] for h in head if h.startswith("property")]
    ncol = len(props)
    vals = np.array(body.split(), dtype=np.float64)
    usable = (len(vals) // ncol) * ncol
    arr = vals[:usable].reshape(-1, ncol)
    want = [props.index(c) for c in ("x", "y", "z")]
    return arr[:, want]


def write_png(path, rgb):
    """A plain RGB PNG. No dependencies beyond zlib, which is in the standard library."""
    h, w = rgb.shape[:2]
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(typ, data):
        return (struct.pack(">I", len(data)) + typ + data
                + struct.pack(">I", zlib.crc32(typ + data) & 0xFFFFFFFF))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n"
                + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6))
                + chunk(b"IEND", b""))


def label(mask):
    """4-connected components, iteratively (no recursion, no scipy)."""
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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("pgm")
    ap.add_argument("yaml")
    ap.add_argument("ply")
    ap.add_argument("--table-height", type=float, default=TABLE_Z)
    ap.add_argument("--band", type=float, default=BAND)
    ap.add_argument("--floor", type=float, default=None,
                    help="floor z (default: the busiest height near the bottom)")
    ap.add_argument("-o", "--out", default=".")
    args = ap.parse_args(argv)

    meta = ex.read_map_yaml(args.yaml)
    grid = ex.read_pgm(args.pgm)
    res, org = meta["resolution"], meta["origin"]
    H, W = grid.shape
    print("map   %d x %d px at %.3f m/px, origin [%.3f, %.3f]"
          % (W, H, res, org[0], org[1]))
    print("      covering x %.2f..%.2f  y %.2f..%.2f"
          % (org[0], org[0] + W * res, org[1], org[1] + H * res))

    pts = read_ply_xyz(args.ply)
    print("cloud %d points, x %.2f..%.2f  y %.2f..%.2f  z %.2f..%.2f"
          % (len(pts), pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(),
             pts[:, 1].max(), pts[:, 2].min(), pts[:, 2].max()))

    floor = args.floor
    if floor is None:
        low = pts[(pts[:, 2] > -1.0) & (pts[:, 2] < 1.0), 2]
        hist, edges = np.histogram(low, bins=np.arange(-1.0, 1.0, 0.05))
        floor = float(edges[int(np.argmax(hist))] + 0.025)
    print("floor at z = %+.2f m" % floor)

    # ---- rasterise, in the PGM's own row order -------------------------------
    def cells(zlo, zhi):
        m = (pts[:, 2] >= floor + zlo) & (pts[:, 2] < floor + zhi)
        q = pts[m]
        c = np.floor((q[:, 0] - org[0]) / res).astype(int)
        r = H - 1 - np.floor((q[:, 1] - org[1]) / res).astype(int)
        ok = (c >= 0) & (c < W) & (r >= 0) & (r < H)
        g = np.zeros((H, W), np.int32)
        np.add.at(g, (r[ok], c[ok]), 1)
        return g

    top = cells(args.table_height - args.band, args.table_height + args.band)
    above = cells(CLEAR_LO, CLEAR_HI)

    mask = (top >= MIN_HITS) & (above <= MAX_ABOVE)
    print("table band %.2f-%.2f m above the floor: %d cells busy, %d with clear air over"
          % (args.table_height - args.band, args.table_height + args.band,
             int((top >= MIN_HITS).sum()), int(mask.sum())))

    lab, n = label(mask)
    print("%d blobs" % n)

    cell_m2 = res * res
    found = []
    for k in range(1, n + 1):
        ys, xs = np.nonzero(lab == k)
        area = len(ys) * cell_m2
        if area < MIN_AREA_M2 or area > MAX_AREA_M2:
            continue
        # back to metres, undoing the row flip
        wx = org[0] + (xs + 0.5) * res
        wy = org[1] + (H - 1 - ys + 0.5) * res
        cx, cy = float(wx.mean()), float(wy.mean())
        ext_x = float(wx.max() - wx.min() + res)
        ext_y = float(wy.max() - wy.min() + res)
        # how round is it? a disc fills pi/4 of its bounding box, a desk fills ~1
        fill = area / max(ext_x * ext_y, 1e-6)
        found.append({"cx": cx, "cy": cy, "ext": (ext_x, ext_y), "area": area,
                      "fill": fill, "px": len(ys), "k": k})

    found.sort(key=lambda t: -t["area"])
    print("\n  #   centre (map frame)      extent        area    fill   looks like")
    print("  " + "-" * 68)
    for i, t in enumerate(found[:40]):
        shape = ("round" if 0.68 <= t["fill"] <= 0.90 and
                 abs(t["ext"][0] - t["ext"][1]) < 0.35 * max(t["ext"]) else "rectangular")
        print("  %-3d %7.2f, %7.2f   %4.2f x %4.2f m  %5.2f m2  %.2f   %s"
              % (i + 1, t["cx"], t["cy"], t["ext"][0], t["ext"][1], t["area"],
                 t["fill"], shape))
    if len(found) > 40:
        print("  ... and %d more" % (len(found) - 40))

    # ---- a picture, because a list of numbers proves nothing -----------------
    SC = 2
    img = np.zeros((H * SC, W * SC, 3), np.uint8)
    base = np.full((H, W, 3), 255, np.uint8)
    base[grid == ex.UNKNOWN] = (232, 232, 232)
    base[grid < 100] = (40, 48, 54)
    over = mask
    base[over] = (232, 140, 40)
    for t in found:
        base[lab == t["k"]] = (196, 60, 30)
    img = np.repeat(np.repeat(base, SC, axis=0), SC, axis=1)
    name = os.path.splitext(os.path.basename(args.pgm))[0]
    png = os.path.join(args.out, name + "_view.png")
    write_png(png, img)
    print("\nwrote %s  (dark = his map, orange = table height with clear air above,"
          "\n            red = kept as a table candidate)" % png)

    doc = {
        "schema": "banquet-detections/0.1",
        "source": "mid360-pointcloud:" + os.path.basename(args.ply),
        "units": "metres",
        "note": "Segmented from the robot's own lidar cloud, not a camera. Positions "
                "are in the MAP frame; subtract the map origin to place them in the "
                "planner, which measures from the map's north-west corner.",
        "map_origin": org,
        "tables": [
            {"id": "L%02d" % (i + 1), "shape": "round",
             # the planner measures x east from the map's west edge and y SOUTH from its
             # north edge, so the map frame's y has to be turned over on the way in
             "centre_m": [round(t["cx"] - org[0], 3),
                          round(org[1] + H * res - t["cy"], 3)],
             "diameter_m": round(max(t["ext"]), 3),
             "confidence": round(min(t["fill"] / 0.785, 1.0), 2)}
            for i, t in enumerate(found)
        ],
    }
    js = os.path.join(args.out, name + "_detections.json")
    with open(js, "w") as f:
        f.write(json.dumps(doc, indent=2) + "\n")
    print("wrote %s  (%d candidates, in the planner's own frame)" % (js, len(found)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
