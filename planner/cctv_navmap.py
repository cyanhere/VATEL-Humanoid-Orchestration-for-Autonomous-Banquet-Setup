"""Tonight's tables, from the hall CCTV, drawn into the robot's own lidar map.

    # once per venue: where the robot's map sits on the floor plan
    python cctv_navmap.py align venue.json --base robot.pgm --base-yaml robot.yaml

    # every event (or every few minutes): a map nav2 can load
    python cctv_navmap.py build venue.json --room "Ballroom" --base robot.pgm \\
        --base-yaml robot.yaml --bridge http://localhost:8765 -o maps/
    python cctv_navmap.py build ... --detections ballroom.json -o maps/     # from a file

    # the same, as a layout the planner opens (Settings -> This layout -> Open a file)
    python cctv_navmap.py plan venue.json --room "Ballroom" --base robot.pgm \
        --base-yaml robot.yaml -o maps/                    # [--bridge URL | --detections F]

WHO KNOWS WHAT
    The robot localises by matching its lidar against a map, and nothing knows a venue's
    walls and pillars as well as that robot's own SLAM run. Nothing in that map knows
    tonight's tables: it was made on another day. The CCTV knows the tables and little else.
    So: the robot's map, and the camera's tables -- never our guess at its walls.

    - inside the room, where the camera can see, the camera decides: the furniture the
      lidar saw on mapping day is cleared (anything not joined to the walls and not a listed
      fixed structure) and tonight's tables are drawn in at their true footprint;
    - everywhere else -- outside the room, out of the camera's view -- the robot's map is
      left exactly as it was, old furniture and all;
    - resolution, origin and extent are the robot's map's, unchanged: same frame, same yaml.

ALIGN, ONCE
    The camera works in the venue's floor-plan frame; the robot in its map's frame. They
    are one rotation and one shift apart (both are metric, top-down views), found by
    sliding the map's occupied cells over the floor plan's ink: every 2 deg, then a fine
    fit that pulls each lidar point onto the nearest line of the plan. Written to
    <base>.align.json with a picture to check it by: the walls should sit on the walls.
    The venue file needs "plan_image" and "plan_m_per_px".

OUTPUT (build)
    <name>.pgm + <name>.yaml   what map_server loads; same frame as the robot's map
    <name>_tables.json         each table in the map frame: centre, size, yaw, corners
    <name>_preview.png         the map, cleared area tinted, tables outlined
"""
import argparse
import json
import math
import os
import sys
import urllib.request

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    sys.exit("cctv_navmap.py needs OpenCV:  pip install opencv-python")

import cctv_map as cm
import export_ros_map as er

OCC, FREE, UNKNOWN = er.OCCUPIED, er.FREE, er.UNKNOWN


# ================================================================ frames
class MapFrame:
    """A ROS map: pixels <-> map metres (x right, y UP, origin = lower-left pixel)."""

    def __init__(self, grid, resolution, origin):
        self.grid = grid
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])
        self.h, self.w = grid.shape

    def to_px(self, XY):
        XY = np.asarray(XY, float).reshape(-1, 2)
        return np.column_stack([(XY[:, 0] - self.ox) / self.res - 0.5,
                                self.h - (XY[:, 1] - self.oy) / self.res - 0.5])

    def to_m(self, cols, rows):
        return np.column_stack([self.ox + (np.asarray(cols) + 0.5) * self.res,
                                self.oy + (self.h - np.asarray(rows) - 0.5) * self.res])


class Alignment:
    """venue (x east, y south, the floor plan's) = s . Rot(phi) . (X, -Y) + t, for map (X, Y).
    The minus: a ROS map's y points up the page and a floor plan's down it. s is 1 unless
    `align --scale` found the plan's raster and the robot's metres to differ."""

    def __init__(self, phi, tx, ty, s=1.0):
        self.phi, self.t, self.s = float(phi), np.array([tx, ty], float), float(s)

    def _R(self):
        c, s = math.cos(self.phi), math.sin(self.phi)
        return np.array([[c, s], [-s, c]])        # row-vector form: U @ R

    def map_to_venue(self, XY):
        XY = np.asarray(XY, float).reshape(-1, 2)
        return self.s * (np.column_stack([XY[:, 0], -XY[:, 1]]) @ self._R()) + self.t

    def venue_to_map(self, V):
        U = ((np.asarray(V, float).reshape(-1, 2) - self.t) / self.s) @ self._R().T
        return np.column_stack([U[:, 0], -U[:, 1]])

    def yaw_venue_to_map(self, deg):
        """A direction's angle, venue (y down) -> map (y up), degrees."""
        a = math.radians(deg)
        d = self.venue_to_map([[math.cos(a), math.sin(a)]])[0] - self.venue_to_map([[0, 0]])[0]
        return math.degrees(math.atan2(d[1], d[0]))

    def to_dict(self, **extra):
        d = {"convention": "venue = s . Rot(phi) . (X, -Y) + t   (row vectors: s . [X, -Y] @ "
                           "[[cos, sin], [-sin, cos]] + t)",
             "phi_deg": round(math.degrees(self.phi), 4),
             "t_m": [round(float(v), 4) for v in self.t]}
        if self.s != 1.0:
            d["scale"] = round(self.s, 6)
        d.update(extra)
        return d

    @classmethod
    def load(cls, path):
        with open(path) as f:
            d = json.load(f)
        return cls(math.radians(d["phi_deg"]), *d["t_m"], s=d.get("scale", 1.0))


def load_base(pgm, yaml):
    grid = er.read_pgm(pgm)
    meta = er.read_map_yaml(yaml)
    if meta.get("negate"):
        grid = 255 - grid
    return MapFrame(grid, meta["resolution"], meta["origin"]), meta


# ================================================================ align
def _occupied_points(mf, min_px=25):
    occ = (mf.grid < 50).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(occ, 8)
    keep = np.zeros(n, bool)
    keep[1:] = st[1:, cv2.CC_STAT_AREA] >= min_px          # drop speckle
    rows, cols = np.nonzero(keep[lab])
    return mf.to_m(cols, rows), keep[lab]


def align(venue, mf, log=print, fit_scale=False):
    """Find the map's place on the floor plan. Returns (Alignment, report).
    fit_scale: also fit a scale between the plan's raster and the robot's metres (a plan
    printed to fit a page, or a SLAM map a percent long), after the rigid fit."""
    from scipy.ndimage import map_coordinates
    from scipy.optimize import minimize

    plan = cv2.imread(os.path.join(venue["_base"], venue["plan_image"]), cv2.IMREAD_GRAYSCALE)
    if plan is None:
        raise SystemExit("cannot read the venue's plan_image")
    pm = float(venue["plan_m_per_px"])
    ink = (plan < 110).astype(np.uint8)
    # 1. coarse: the map's occupied cells slid over the plan's ink, both at the map's scale
    f = pm / mf.res
    pw = cv2.resize(ink.astype(np.float32), None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
    pw = cv2.GaussianBlur((pw > 0.12).astype(np.float32), (7, 7), 0)
    _, occ = _occupied_points(mf)
    occ = occ.astype(np.float32)
    H, W = occ.shape
    cands = []
    for th in range(0, 360, 2):
        M = cv2.getRotationMatrix2D((W / 2, H / 2), th, 1.0)
        nw = int(H * abs(M[0, 1]) + W * abs(M[0, 0]))
        nh = int(H * abs(M[0, 0]) + W * abs(M[0, 1]))
        M[0, 2] += nw / 2 - W / 2
        M[1, 2] += nh / 2 - H / 2
        rot = cv2.GaussianBlur(cv2.warpAffine(occ, M, (nw, nh)), (7, 7), 0)
        if rot.shape[0] > pw.shape[0] or rot.shape[1] > pw.shape[1]:
            continue
        _, score, _, loc = cv2.minMaxLoc(cv2.matchTemplate(pw, rot, cv2.TM_CCORR_NORMED))
        centre = np.array([(loc[0] + nw / 2) * mf.res, (loc[1] + nh / 2) * mf.res])
        cands.append((score, th, centre))
    cands.sort(key=lambda c: -c[0])

    # 2. fine: each lidar point pulled onto the nearest ink, robustly, from the best few starts
    dist = cv2.distanceTransform((1 - ink).astype(np.uint8), cv2.DIST_L2, 5) * pm
    XY, _ = _occupied_points(mf)
    U = np.column_stack([XY[:, 0], -XY[:, 1]])
    centre_U = np.array([mf.ox + mf.w * mf.res / 2, -(mf.oy + mf.h * mf.res / 2)])

    def venue_of(p):
        c, s = math.cos(p[0]), math.sin(p[0])
        k = p[3] if len(p) > 3 else 1.0
        return k * (U @ np.array([[c, s], [-s, c]])) + p[1:3]

    def cost(p, trunc):
        V = venue_of(p) / pm
        d = map_coordinates(dist, [V[:, 1], V[:, 0]], order=1, mode="constant", cval=trunc)
        return float(np.mean(np.minimum(d, trunc)))

    best = None
    for score, th, centre in cands[:4]:
        for sign in (1, -1):                       # which way the image rotation runs
            phi = math.radians(sign * th)
            c, s = math.cos(phi), math.sin(phi)
            p = np.array([phi, *(centre - centre_U @ np.array([[c, s], [-s, c]]))])
            for trunc in (1.0, 0.5, 0.25):
                p = minimize(lambda q: cost(q, trunc), p, method="Powell",
                             options={"xtol": 1e-4, "ftol": 1e-7}).x
            final = cost(p, 0.25)
            if best is None or final < best[0]:
                best = (final, p, score)
    final, p, score = best
    if fit_scale:
        # turn and scale about the map's centre, not its origin, so the shift does not have
        # to chase them: q = (extra turn, centre's extra shift, scale)
        def rot(a):
            return np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])

        cv = centre_U @ rot(p[0]) + p[1:3]             # where the map's centre lands now

        def scaled(q):
            a = p[0] + q[0]
            return np.array([a, *(cv + q[1:3] - q[3] * (centre_U @ rot(a))), q[3]])

        q = np.array([0.0, 0.0, 0.0, 1.0])
        for trunc in (0.5, 0.25):
            q = minimize(lambda r: cost(scaled(r), trunc), q, method="Powell",
                         options={"xtol": 1e-5, "ftol": 1e-8}).x
        p = scaled(q)
        log("scale: the robot's metres are %.4f of the plan's (%+.2f%%)" % (p[3], 100 * (p[3] - 1)))
    V = venue_of(p)
    d = map_coordinates(dist, [V[:, 1] / pm, V[:, 0] / pm], order=1, mode="constant", cval=9)
    rep = {"coarse_score": round(float(score), 3),
           "points_within_5cm": round(float(np.mean(d < 0.05)), 3),
           "points_within_10cm": round(float(np.mean(d < 0.10)), 3),
           "median_m": round(float(np.median(d)), 4)}
    log("aligned: rotate %.2f deg, shift (%.3f, %.3f) m; %.0f%% of the map's occupied cells "
        "within 5 cm of the plan's lines (furniture is not on the plan, so not all)"
        % (math.degrees(p[0]) % 360, p[1], p[2], 100 * rep["points_within_5cm"]))
    return Alignment(*p[:3], s=p[3] if len(p) > 3 else 1.0), rep, ink


def align_picture(path, mf, al, ink, pm, window=None):
    img = cv2.cvtColor(np.where(ink > 0, 120, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    XY, _ = _occupied_points(mf)
    px = np.round(al.map_to_venue(XY) / pm).astype(int)
    ok = ((px[:, 0] >= 1) & (px[:, 1] >= 1) & (px[:, 0] < img.shape[1] - 1)
          & (px[:, 1] < img.shape[0] - 1))
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            img[px[ok, 1] + dy, px[ok, 0] + dx] = (0, 0, 230)
    if window is None:
        lo, hi = px[ok].min(0), px[ok].max(0)
        pad = int(2 / pm)
        window = (max(0, lo[0] - pad), max(0, lo[1] - pad), hi[0] + pad, hi[1] + pad)
    x0, y0, x1, y1 = window
    crop = img[y0:y1, x0:x1]
    s = 1400 / max(crop.shape[1], 1)
    cv2.imwrite(path, cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_AREA))
    return path


# ================================================================ build
def fetch_detections(url):
    with urllib.request.urlopen(url.rstrip("/") + "/detections", timeout=10) as r:
        return json.loads(r.read().decode("utf-8"))


def table_corners_room(t):
    """A detection's footprint in its room's frame, as a polygon."""
    cx, cy = t["centre_m"]
    if t.get("shape") == "round" or "diameter_m" in t:
        r = t["diameter_m"] / 2
        a = np.linspace(0, 2 * math.pi, 32, endpoint=False)
        return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])
    L, Wd = t["size_m"]
    a = math.radians(t.get("rotation_deg", 0.0))
    u, v = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
    return np.array([[cx, cy] + sx * L / 2 * u + sy * Wd / 2 * v
                     for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def _poly_mask(mf, poly_map):
    m = np.zeros((mf.h, mf.w), np.uint8)
    if len(poly_map) >= 3:
        cv2.fillPoly(m, [np.round(mf.to_px(poly_map) * 8).astype(np.int32)], 1, cv2.LINE_8, 3)
    return m.astype(bool)


def doc_tables_venue(doc, room):
    """A detections doc's tables in the venue frame: (detection, footprint, centre, the long
    side's direction in degrees). The doc says which frame it is in (room.frame): its room's
    own, or a robot map's."""
    frame = (doc.get("room") or {}).get("frame", "room")
    out = []
    for t in doc.get("tables", []):
        foot = room.from_out(table_corners_room(t), frame)
        c = room.from_out([t["centre_m"]], frame)[0]
        q = room.from_out([t["centre_m"], [t["centre_m"][0] + math.cos(math.radians(
            t.get("rotation_deg", 0.0))), t["centre_m"][1] + math.sin(math.radians(
                t.get("rotation_deg", 0.0)))]], frame)
        out.append((t, foot, c, math.degrees(math.atan2(*(q[1] - q[0])[::-1]))))
    return out


def clear_furniture(venue, room, mf, al, clear_margin_m=0.3, log=print):
    """The robot's map with the mapping-day furniture cleared where the room's cameras see.
    -> (grid, mask of what was cleared)"""
    grid = mf.grid.copy()
    # the area the camera decides: the room's floor, inside its walls by a margin, where a
    # camera on this room can see
    zone = _poly_mask(mf, al.venue_to_map(room.floor))
    k = max(1, int(round(clear_margin_m / mf.res)))
    zone = cv2.erode(zone.astype(np.uint8), np.ones((2 * k + 1, 2 * k + 1), np.uint8)).astype(bool)
    seen = np.zeros_like(zone)
    for name, cfg in venue["_cams"].items():
        if room.cameras and name not in room.cameras:
            continue
        view = cm.camera_view_venue(cfg, room)["view"]
        if len(view) >= 3:
            seen |= _poly_mask(mf, al.venue_to_map(view))
    zone &= seen
    # fixed structure inside the room stays: listed fixed_m polygons, and anything occupied
    # that is joined to the walls (a component reaching the zone's edge)
    keep = np.zeros_like(zone)
    for poly in room.fixed:
        keep |= _poly_mask(mf, al.venue_to_map(poly))
    occ = (grid < 50).astype(np.uint8)
    n, lab, st, _ = cv2.connectedComponentsWithStats(occ, 8)
    edge = zone & ~cv2.erode(zone.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)
    touching = np.unique(lab[edge & (occ > 0)])
    outside = np.unique(lab[(~zone) & (occ > 0)])
    joined = np.isin(lab, np.union1d(touching, outside)) & (lab > 0)
    clear = zone & ~keep & ~joined
    grid[clear] = FREE
    log("%s: cleared %.1f m2 of mapping-day furniture where the camera sees"
        % (room.name, clear.sum() * mf.res ** 2))
    return grid, clear


def plan_grid(room, mf, al, wall_m=0.15):
    """The room as the floor plan draws it, in the robot map's frame and extent: the floor
    inside its walls (floor_m) free, a wall's thickness outward from every face occupied,
    fixed structure (fixed_m) occupied, everything else unknown.

    For a map to plan or test navigation on when the robot's own scan is not what is
    wanted underneath -- an empty room to lay tables in. It is not a map to localise
    against: the furniture the lidar would see is not in it."""
    inside = _poly_mask(mf, al.venue_to_map(room.floor))
    k = max(1, int(round(wall_m / mf.res)))
    ring = cv2.dilate(inside.astype(np.uint8),
                      np.ones((2 * k + 1, 2 * k + 1), np.uint8)).astype(bool) & ~inside
    grid = np.full((mf.h, mf.w), UNKNOWN, np.uint8)
    grid[inside] = FREE
    grid[ring] = OCC
    for poly in room.fixed:
        grid[_poly_mask(mf, al.venue_to_map(poly))] = OCC
    return grid


def build(venue, room, mf, al, doc, clear_margin_m=0.3, log=print):
    """-> (grid, tables in map frame, mask of the area the camera decided)."""
    grid, clear = clear_furniture(venue, room, mf, al, clear_margin_m, log)
    tables = []
    for t, foot, c_v, rot_v in doc_tables_venue(doc, room):
        corners = al.venue_to_map(foot)
        cv2.fillPoly(grid, [np.round(mf.to_px(corners) * 8).astype(np.int32)], int(OCC),
                     cv2.LINE_8, 3)
        c = al.venue_to_map([c_v])[0]
        e = {"id": t.get("id"), "shape": t.get("shape", "rect"),
             "centre_m": [round(float(c[0]), 3), round(float(c[1]), 3)],
             "corners_m": [[round(float(x), 3), round(float(y), 3)] for x, y in corners],
             "confidence": t.get("confidence")}
        if "size_m" in t:
            e["size_m"] = t["size_m"]
            yaw = al.yaw_venue_to_map(rot_v)          # a table's yaw is defined mod 180 deg
            e["yaw_deg"] = round(((yaw + 90.0) % 180.0) - 90.0, 1)
        if "diameter_m" in t:
            e["diameter_m"] = t["diameter_m"]
        if t.get("clipped"):
            e["clipped"] = True
        tables.append(e)
    log("drew %d tables" % len(tables))
    return grid, tables, clear


def planner_layout(venue, room, mf, al, meta, doc=None, name=None, style="clean", log=print,
                   base_from="scan"):
    """A layout the planner opens (Settings -> This layout -> Open a file), in the robot
    map's own frame, so the camera's live tables (a bridge on a room with "map") land on
    it and the planner's export comes out in the robot's frame.

    style "clean" (default): the room as the floor plan draws it -- its walls (floor_m)
    and fixed structure (fixed_m), placed by the alignment. Crisp, and what a person
    plans on. The robot's own map, scan-exact, is what `build` writes for the robot.
    style "scan": the robot's scan itself, traced into boxes, mapping-day furniture cleared
    where the camera sees. Every obstacle the lidar met, and every speck of noise.

    With `doc`, its tables are laid too, for a look without the bridge running.

    base_from: what the planner's export draws its tables onto -- "scan", the robot's map
    with mapping-day furniture cleared where the camera sees (default), or "plan", the
    room drawn from the floor plan alone (plan_grid), for an empty room to lay tables in."""
    import map_to_plan as mp
    items, nid, angle, floor = [], 1, 0.0, None
    if style == "scan":
        grid, _ = clear_furniture(venue, room, mf, al, log=log)
        # a whole venue's walls outrun map_to_plan's default 400 pieces; the planner keeps
        # traced structure in one cached layer, so a thousand or so costs nothing
        mp.MAX_RECTS = max(mp.MAX_RECTS, 1500)
        rects, angle = mp.trace(grid, mf.res)
        floor = mp.floor_mask(grid, mf.res)
    else:
        rects = []
        for poly in room.fixed:                       # columns, blocks: as the plan draws them
            q = room.out_pts(poly).astype(np.float32)
            (cx, cy), (w, h), a = cv2.minAreaRect(q)
            rects.append({"type": "obst", "x": round(float(cx), 3), "y": round(float(cy), 3),
                          "w": round(float(w), 3), "h": round(float(h), 3),
                          "rot": round(float(a), 1)})
    for r in rects:
        it = {"k": "f", "id": nid, "type": r["type"], "x": r["x"], "y": r["y"],
              "w": r["w"], "h": r["h"], "rot": r["rot"]}
        if r.get("shape") == "circle":
            it["shape"] = "circle"
        items.append(it)
        nid += 1
    num = 1
    for t, foot, c_v, rot_v in (doc_tables_venue(doc, room) if doc else []):
        q = room.out_pts([c_v])[0]
        r05 = lambda v: round(round(v / 0.05) * 0.05, 2)
        it = {"k": "t", "id": nid, "num": num, "stock": "seen", "src": "cam",
              "x": round(float(q[0]), 3), "y": round(float(q[1]), 3)}
        if "size_m" in t:
            L, Wd = max(t["size_m"]), min(t["size_m"])
            both = Wd >= 0.75
            it.update(dims={"w": r05(L), "h": max(0.3, r05(Wd))},
                      rot=round(((room.out_turn(rot_v) + 90.0) % 180.0) - 90.0, 1),
                      sides="both" if both else "one",
                      covers=max(1, int(L // 0.6)) * (2 if both else 1))
        else:
            d = r05(t["diameter_m"])
            it.update(dims={"d": d}, rot=0, sides=None,
                      covers=max(2, int(math.pi * d // 0.6)))
        items.append(it)
        nid += 1
        num += 1
    w, h = room.out_size()
    origin = [mf.ox, mf.oy, 0.0]
    # the robot's map itself, mapping-day furniture cleared where the camera sees: the
    # planner's export draws its tables onto this instead of boxing the room in
    import base64
    if base_from == "plan":
        cleared = plan_grid(room, mf, al)
    else:
        cleared, _ = clear_furniture(venue, room, mf, al, log=lambda *a: None)
    base = {"w": int(mf.w), "h": int(mf.h), "res": mf.res, "origin": [mf.ox, mf.oy, 0.0],
            "image": meta.get("image"),
            "pgm_b64": base64.b64encode(np.ascontiguousarray(cleared, np.uint8).tobytes())
            .decode("ascii")}
    custom = {"w": round(w, 3), "h": round(h, 3), "name": room.name}
    if room.walls:                 # the same walls, rounded the same way, as the bridge sends
        custom["floor"] = room.out_floor()
    plan = {"roomId": "custom", "custom": custom,
            "event": {"name": name or room.name, "date": "", "pax": 0, "courses": 3},
            "items": items, "nextId": nid,
            "floor": floor, "base": base,
            # the planner sets its robot map frame from this, so its own export is in it
            "mapRef": {"origin": origin, "resolution": mf.res},
            "traced_from": {"image": meta.get("image"), "resolution": mf.res,
                            "origin": origin, "size_px": [mf.w, mf.h],
                            "wall_angle_deg": angle}}
    log("layout (%s): %d pieces of structure, %d tables, %.2f x %.2f m"
        % (style, len(rects), num - 1, w, h))
    return plan


def preview(path, mf, grid, tables, clear):
    img = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    tint = img.copy()
    tint[clear] = (230, 200, 150)
    img = cv2.addWeighted(tint, 0.5, img, 0.5, 0)
    for t in tables:
        pts = np.round(mf.to_px(t["corners_m"])).astype(np.int32)
        cv2.polylines(img, [pts], True, (40, 160, 40), 1, cv2.LINE_AA)
    s = max(1, int(1200 / mf.w))
    cv2.imwrite(path, cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_NEAREST))
    return path


# ================================================================ command line
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("align", help="once per venue: place the robot's map on the floor plan")
    a.add_argument("venue")
    a.add_argument("--base", required=True, help="the robot's map .pgm")
    a.add_argument("--base-yaml", required=True, help="its .yaml")
    a.add_argument("--align", help="alignment file to write (default <base>.align.json)")
    a.add_argument("--scale", action="store_true",
                   help="also fit a scale between the plan and the robot's map (they can "
                        "differ by a percent: a plan printed to fit, a SLAM map's drift)")
    b = sub.add_parser("build", help="the robot's map with tonight's tables")
    b.add_argument("venue")
    b.add_argument("--room", required=True)
    b.add_argument("--base", required=True)
    b.add_argument("--base-yaml", required=True)
    b.add_argument("--align", help="alignment file (default <base>.align.json)")
    src = b.add_mutually_exclusive_group(required=True)
    src.add_argument("--bridge", help="the running cctv_bridge, e.g. http://localhost:8765")
    src.add_argument("--detections", help="a banquet-detections/0.2 file (cctv_map.py --out)")
    b.add_argument("-o", "--out", default=".", help="directory to write into")
    b.add_argument("-n", "--name", default=None, help="base filename (default: the room's)")
    pl = sub.add_parser("plan", help="a layout the planner opens: the robot map's walls")
    pl.add_argument("venue")
    pl.add_argument("--room", required=True)
    pl.add_argument("--base", required=True)
    pl.add_argument("--base-yaml", required=True)
    pl.add_argument("--align", help="alignment file (default <base>.align.json)")
    psrc = pl.add_mutually_exclusive_group()
    psrc.add_argument("--bridge", help="also lay the tables a running bridge sees now")
    psrc.add_argument("--detections", help="also lay the tables in this detections file")
    pl.add_argument("-o", "--out", default=".", help="directory to write into")
    pl.add_argument("-n", "--name", default=None, help="file name (default: the room's)")
    pl.add_argument("--style", choices=("clean", "scan"), default="clean",
                    help="clean: the floor plan's walls and columns (default); "
                         "scan: the robot's lidar scan, traced")
    pl.add_argument("--base-from", choices=("scan", "plan"), default="scan",
                    help="what the planner's export draws tables onto: the robot's scan "
                         "(default), or the room drawn from the floor plan alone -- an "
                         "empty room, also written as <name>_empty.pgm/.yaml")
    args = ap.parse_args(argv)

    venue = cm.load_venue(args.venue) if args.cmd != "align" else _load_venue_light(args.venue)
    mf, meta = load_base(args.base, args.base_yaml)
    align_path = args.align or os.path.splitext(args.base)[0] + ".align.json"

    if args.cmd == "align":
        al, rep, ink = align(venue, mf, fit_scale=args.scale)
        with open(align_path, "w") as f:
            json.dump(al.to_dict(base=os.path.basename(args.base), fit=rep), f, indent=2)
        pic = align_picture(os.path.splitext(align_path)[0] + ".png", mf, al, ink,
                            float(venue["plan_m_per_px"]))
        print("wrote %s and %s -- check the picture: the red walls should sit on the plan's"
              % (align_path, pic))
        return 0

    rooms = [r for r in venue["rooms"] if r["name"] == args.room]
    if not rooms:
        raise SystemExit("no room %r in the venue file" % args.room)
    if not os.path.exists(align_path):
        raise SystemExit("no alignment yet: run  cctv_navmap.py align ...  first")
    al = Alignment.load(align_path)
    room = cm.Room(rooms[0], venue["_base"])
    room.map, room.map_name = (mf, al), os.path.basename(args.base)    # report in this map
    doc = None
    if args.bridge:
        doc = fetch_detections(args.bridge)
    elif args.detections:
        with open(args.detections) as f:
            doc = json.load(f)
    if doc and doc.get("room", {}).get("name") not in (None, room.name):
        raise SystemExit("the detections are for %r, not %r" % (doc["room"]["name"], room.name))
    os.makedirs(args.out, exist_ok=True)
    if args.cmd == "plan":
        plan = planner_layout(venue, room, mf, al, meta, doc, args.name, args.style,
                              base_from=args.base_from)
        name = args.name or room.name.replace(" ", "_").lower()
        out = os.path.join(args.out, name + ".plan.json")
        with open(out, "w") as f:
            f.write(json.dumps(plan, indent=1) + "\n")
        print("wrote %s -- planner: Settings -> This layout -> Open a file" % out)
        if args.base_from == "plan":
            pgm = os.path.join(args.out, name + "_empty.pgm")
            er.write_pgm(pgm, plan_grid(room, mf, al))
            er.write_yaml(os.path.join(args.out, name + "_empty.yaml"), name + "_empty.pgm",
                          mf.res, [mf.ox, mf.oy, 0.0])
            print("wrote %s and %s_empty.yaml -- the empty room, for map_server" % (pgm, name))
        return 0
    grid, tables, clear = build(venue, room, mf, al, doc)
    name = args.name or room.name.replace(" ", "_").lower()
    pgm = os.path.join(args.out, name + ".pgm")
    er.write_pgm(pgm, grid)
    er.write_yaml(os.path.join(args.out, name + ".yaml"), name + ".pgm", mf.res,
                  [mf.ox, mf.oy, float(meta["origin"][2]) if len(meta["origin"]) > 2 else 0.0])
    with open(os.path.join(args.out, name + "_tables.json"), "w") as f:
        json.dump({"frame": "the robot map's (x right, y up, metres), same as %s"
                            % os.path.basename(args.base_yaml),
                   "captured_at": doc.get("captured_at"), "tables": tables}, f, indent=1)
    print("wrote %s, %s.yaml, %s_tables.json, %s"
          % (pgm, name, name, preview(os.path.join(args.out, name + "_preview.png"),
                                      mf, grid, tables, clear)))
    return 0


def _load_venue_light(path):
    """align needs the plan only, not the cameras."""
    with open(path) as f:
        v = json.load(f)
    v["_base"] = os.path.dirname(os.path.abspath(path))
    return v


if __name__ == "__main__":
    sys.exit(main())
