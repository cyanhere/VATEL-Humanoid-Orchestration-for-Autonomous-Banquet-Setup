"""Hall CCTV -> the tables on the planner's floor map, in metres.

    python cctv_map.py venue.json                      # every room, from each camera's recording
    python cctv_map.py venue.json --room "Sandbox" --out sandbox.json --review review/

The pieces, each in its own file:

    cctv_segment.py   frame -> table and chair masks (Grounded-SAM on a people-free plate)
    cctv_camera.py    pixel <-> floor, per camera, solved once from landmarks
    this file         masks -> tables in the venue's metres -> one list per ROOM, with the
                      same table seen by two cameras counted once, in the planner's frame

THE VENUE FILE
    Everything that is about one hall lives here, not in code, so a new hall is a new file:

    {
      "name": "...",
      "table_height_m": 0.75,
      "cameras": ["cameras/cam_a.json", "cameras/cam_b.json"],
      "prompts": {"table": ["table"], "chair": ["chair"]},
      "rooms": [
        {"name": "Ballroom", "origin_m": [0, 0], "rotation_deg": 0, "size_m": [20, 21],
         "cameras": ["cam_a", "cam_b"], "axes_deg": [0]}
      ],
      "ignore_m": [{"label": "bar counter", "polygon": [[x, y], ...]}]
    }

    origin_m / rotation_deg place the room's top-left corner and its x axis in the venue
    frame, so a room can be any rectangle on the venue's plan, square to it or not.
    floor_m (optional): the room's real outline in venue metres, when its walls are not
    that rectangle. Tables outside it are dropped, and the planner draws it as the walls.
    axes_deg: the directions the room's walls run, to square rectangular tables to.
    ignore_m: fixed furniture the detector will call a table (counters, sofas, a bar).
    snap_deg (per room): rectangles within this of the walls' direction are squared to them.
    max_m_per_px: how far one pixel of outline may move a table along the floor before
    its camera is too far away to be believed about it (default 0.07 m). Far off, at a
    grazing angle, a pixel is many centimetres: the table is real but its place is not.
    min_score (per room): the detector score a table needs there (default: anything).

WHAT COMES OUT
    banquet-detections/0.2, in the ROOM's metres (x along its top wall, y down its side):
    tables as {"shape": "rect", "centre_m", "size_m", "rotation_deg"} or
    {"shape": "round", "centre_m", "diameter_m"}, each with a confidence and the cameras
    that saw it; and the cameras themselves -- where each hangs, which way it faces, and
    the floor it can see -- so the plan can show where the picture is taken from. The
    planner snaps each table to the venue's stock.

    Chairs are still looked for, because a chair cut out of a table's mask is what keeps a
    table's outline its own; they are not reported.
"""
import argparse
import datetime
import json
import math
import os
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:                                       # pragma: no cover
    sys.exit("cctv_map.py needs OpenCV:  pip install opencv-python")

import cctv_camera as cc

SCHEMA = "banquet-detections/0.2"


# ================================================================ the venue
def load_venue(path):
    with open(path) as f:
        v = json.load(f)
    base = os.path.dirname(os.path.abspath(path))
    v["_base"] = base
    v["_cams"] = {}
    for rel in v.get("cameras", []):
        p = os.path.join(base, rel)
        cfg = cc.load_config(p)
        if "solved" not in cfg:
            raise SystemExit("%s has no solved camera yet: python cctv_camera.py %s --save" % (rel, rel))
        cfg["_path"] = p
        cfg["_cam"] = cc.Camera.from_dict(cfg["solved"], cfg["name"])
        v["_cams"][cfg["name"]] = cfg
    return v


def camera_source(cfg):
    """Where to read a camera from: its live stream if it has one, else its recording."""
    src = cfg.get("stream") or cfg.get("video")
    if not src:
        return None
    if "://" in str(src) or str(src).isdigit():
        return src
    for base in (os.path.dirname(cfg["_path"]), os.path.dirname(os.path.dirname(cfg["_path"]))):
        p = os.path.join(base, src)
        if os.path.exists(p):
            return p
    return src


class Room:
    """A rectangle on the venue plan, and the planner's frame on it."""

    def __init__(self, d):
        self.name = d["name"]
        self.o = np.asarray(d.get("origin_m", (0.0, 0.0)), float)
        self.rot = float(d.get("rotation_deg", 0.0))
        self.w, self.h = map(float, d["size_m"])
        t = math.radians(self.rot)
        self.ex = np.array([math.cos(t), math.sin(t)])     # room x, in the venue frame
        self.ey = np.array([-math.sin(t), math.cos(t)])    # room y (x east, y south: +90 deg)
        self.cameras = d.get("cameras")
        self.axes = [a for a in d.get("axes_deg", [self.rot])]
        # how far off the walls' direction a rectangle may sit before it is believed:
        # 12 deg by default; a room where every table is square to the walls can say 45
        self.snap = float(d.get("snap_deg", 12.0))
        # the detector's score below which nothing counts as a table here: it depends on
        # the furniture (light tables score high, dark ones low), so it is the venue's call
        self.min_score = float(d.get("min_score", 0.0))
        self.walls = bool(d.get("floor_m"))      # a real outline, not just the rectangle
        self.floor = (np.asarray(d["floor_m"], float) if d.get("floor_m") else
                      np.array([self.to_venue(p) for p in ((0, 0), (self.w, 0), (self.w, self.h),
                                                           (0, self.h))]))

    def to_room(self, p):
        d = np.asarray(p, float) - self.o
        return np.array([d @ self.ex, d @ self.ey])

    def to_venue(self, q):
        q = np.asarray(q, float)
        return self.o + q[0] * self.ex + q[1] * self.ey

    def contains(self, p):
        return cv2.pointPolygonTest(self.floor.astype(np.float32).reshape(-1, 1, 2),
                                    (float(p[0]), float(p[1])), False) >= 0


# ================================================================ one camera
def _contour(mask, scale):
    cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cs:
        return np.zeros((0, 2))
    c = max(cs, key=cv2.contourArea).reshape(-1, 2).astype(float)
    return (c + 0.5) * scale - 0.5


def _touches_edge(mask, margin=2):
    return bool(mask[:margin].any() or mask[-margin:].any() or mask[:, :margin].any()
                or mask[:, -margin:].any())


def map_instances(cfg, instances, venue, axes_deg=None, snap_deg=12.0):
    """One camera's masks -> tables in venue metres, before any filtering."""
    cam = cfg["_cam"]
    top_h = cfg["solved"].get("table_h_m", venue.get("table_height_m", 0.75))
    tables = []
    for inst in instances:
        if inst.cls != "table":
            continue
        fit = cc.fit_top(cam, _contour(inst.mask, inst.scale), top_h, axes_deg=axes_deg,
                         snap_deg=snap_deg)
        if fit is None:
            continue
        r = float(np.hypot(*(np.asarray(fit["centre_m"]) - cam.nadir)))
        # metres along the floor per pixel at that range, on the tabletop's plane
        fit.update(confidence=inst.score, camera=cfg["name"],
                   clipped=_touches_edge(inst.mask), range_m=r,
                   m_per_px=r * r / max((cam.height_m - top_h) * cam.f, 1e-6))
        tables.append(fit)
    return tables


def camera_view(cfg, room, max_range_m=25.0, n=24):
    """Where a camera hangs and what floor it sees, in the room's frame -- so the plan can
    say which way the picture faces. The floor it sees is the frame's border traced onto
    the floor (out to `max_range_m` where the border is above the horizon), cut to the room."""
    cam = cfg["_cam"]
    t = np.linspace(0.0, 1.0, n)
    W, H = cam.w - 1, cam.h - 1
    border = np.concatenate([np.column_stack([t * W, np.zeros(n)]),
                             np.column_stack([np.full(n, W), t * H]),
                             np.column_stack([(1 - t) * W, np.full(n, H)]),
                             np.column_stack([np.zeros(n), (1 - t) * H])])
    pts = []
    for d in cam.rays(border):                       # venue frame, z down
        h = math.hypot(d[0], d[1])
        if d[2] > 1e-6:
            P = cam.C + (-cam.C[2] / d[2]) * d           # where it meets the floor
            if math.hypot(*(P[:2] - cam.C[:2])) <= max_range_m:
                pts.append(P[:2])
                continue
        if h > 1e-9:
            pts.append(cam.C[:2] + max_range_m * d[:2] / h)
    poly = np.array([room.to_room(p) for p in pts])
    # cut to the room's real floor where the venue gives one (walls need not be square to
    # the plan's rectangle), as its hull -- the clip needs a convex edge to cut against
    floor = np.array([room.to_room(p) for p in room.floor])
    rim = cc._hull(floor) if len(floor) >= 3 else np.array(
        [[0.0, 0.0], [room.w, 0.0], [room.w, room.h], [0.0, room.h]])
    seen = cc._clip(poly, rim) if len(poly) >= 3 else np.zeros((0, 2))
    f = cam.R[2][:2]                                  # the optical axis, on the floor
    edge = cam.rays([[0, H / 2], [W, H / 2]])
    return {"name": cfg["name"], "label": cfg.get("short", cfg["name"]),
            "at_m": [round(float(v), 3) for v in room.to_room(cam.nadir)],
            "height_m": round(float(cam.height_m), 2),
            "heading_deg": round(math.degrees(math.atan2(f @ room.ey, f @ room.ex)), 1),
            "hfov_deg": round(math.degrees(math.acos(float(np.clip(edge[0] @ edge[1], -1, 1)))), 1),
            "view_m": [[round(float(x), 2), round(float(y), 2)] for x, y in seen]}


# ================================================================ sanity, then one list
def _poly(t):
    if t["shape"] == "rect":
        return np.asarray(t["corners_m"], float)
    a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    r = t["diameter_m"] / 2
    return np.asarray(t["centre_m"]) + r * np.column_stack([np.cos(a), np.sin(a)])


def _area_of(t):
    return (t["size_m"][0] * t["size_m"][1] if t["shape"] == "rect"
            else math.pi * t["diameter_m"] ** 2 / 4)


def overlap(a, b):
    """Shared area over the smaller of the two."""
    pa, pb = cc._hull(_poly(a)), cc._hull(_poly(b))
    inter = cc._clip(pa, pb)
    ia = abs(cc._area(inter)) if len(inter) >= 3 else 0.0
    return ia / max(min(abs(cc._area(pa)), abs(cc._area(pb))), 1e-9)


def plausible(t, room, venue, min_side=0.3, max_side=9.0, min_area=0.2):
    """Why a detection is not a table in this room, or None if it may be."""
    c = t["centre_m"]
    if not room.contains(c):
        return "outside the room"
    if t["confidence"] < room.min_score:
        return "low score (%.2f)" % t["confidence"]
    if t.get("m_per_px", 0.0) > venue.get("max_m_per_px", 0.07):
        return "too far from %s to place (%.0f cm a pixel)" % (t["camera"], 100 * t["m_per_px"])
    for ig in venue.get("ignore_m", []):
        poly = np.asarray(ig["polygon"], np.float32).reshape(-1, 1, 2)
        if cv2.pointPolygonTest(poly, (float(c[0]), float(c[1])), False) >= 0:
            return "fixed: " + ig.get("label", "ignored area")
    dims = t["size_m"] if t["shape"] == "rect" else [t["diameter_m"]] * 2
    if min(dims) < min_side:
        return "too thin (%.2f m)" % min(dims)
    if max(dims) > max_side:
        return "too long (%.1f m)" % max(dims)
    if _area_of(t) < min_area:
        return "too small"
    return None


def _better(a, b):
    """Of two sightings of one table, the one to trust: whole in frame, then nearer."""
    ka = (a["clipped"], a["range_m"] / max(a["confidence"], 0.05))
    kb = (b["clipped"], b["range_m"] / max(b["confidence"], 0.05))
    return a if ka <= kb else b


def _union(a, b):
    """Two rectangles square to each other -> the one rectangle covering both."""
    t = math.radians(a["rotation_deg"])
    R = np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
    Q = np.vstack([np.asarray(a["corners_m"]), np.asarray(b["corners_m"])]) @ R.T
    (u0, v0), (u1, v1) = Q.min(0), Q.max(0)
    corners = np.array([[u0, v0], [u1, v0], [u1, v1], [u0, v1]]) @ R
    c = corners.mean(0)
    w, h, rot = u1 - u0, v1 - v0, a["rotation_deg"]
    if w < h:
        w, h, rot = h, w, rot + 90.0
    out = dict(_better(a, b))
    out.update(centre_m=[float(c[0]), float(c[1])], size_m=[float(w), float(h)],
               rotation_deg=float((rot + 90.0) % 180.0 - 90.0), corners_m=corners.tolist(),
               confidence=max(a["confidence"], b["confidence"]),
               clipped=a["clipped"] and b["clipped"])
    return out


def _square_to(a, b, tol=3.0):
    if a["shape"] != "rect" or b["shape"] != "rect":
        return False
    d = (a["rotation_deg"] - b["rotation_deg"]) % 90.0
    return min(d, 90.0 - d) <= tol


def fuse(tables, touch_same=0.1, touch_cross=0.3):
    """One entry per table.

    Two real tables cannot overlap, so overlapping footprints are one table seen twice:
    - by ONE camera, in pieces (a row of desks pushed together, cut where a chair hides
      it): the pieces are put back together;
    - by TWO cameras: the better sighting is kept -- whole in frame, nearer the camera,
      more confident -- rather than an average of a good view and a grazing one.
    Two cameras' sightings that only just overlap are often the two halves of a back-to-back
    row, each seen best from its own side, so across cameras it takes more overlap."""
    for t in tables:
        t.setdefault("cameras", [t["camera"]])
    out = sorted(tables, key=lambda t: (t["clipped"], t["range_m"]))
    merged = True
    while merged:
        merged = False
        for i in range(len(out)):
            for j in range(i + 1, len(out)):
                a, b = out[i], out[j]
                if overlap(a, b) < (touch_same if a["camera"] == b["camera"] else touch_cross):
                    continue
                if a["camera"] == b["camera"] and _square_to(a, b):
                    m = _union(a, b)
                else:
                    m = dict(_better(a, b))
                m["cameras"] = sorted(set(a["cameras"]) | set(b["cameras"]))
                m["camera"] = _better(a, b)["camera"]
                out[i] = m
                del out[j]
                merged = True
                break
            if merged:
                break
    return out


def room_doc(room, tables, cameras=(), source="", seq=0):
    """The planner's view: the room's own frame and units."""
    rt = []
    for k, t in enumerate(sorted(tables, key=lambda t: (t["centre_m"][1], t["centre_m"][0]))):
        c = room.to_room(t["centre_m"])
        e = {"id": "t%d" % (k + 1), "shape": t["shape"],
             "centre_m": [round(float(c[0]), 3), round(float(c[1]), 3)],
             "confidence": round(float(t["confidence"]), 3),
             "cameras": t.get("cameras", [t.get("camera")])}
        if t["shape"] == "rect":
            e["size_m"] = [round(float(x), 3) for x in t["size_m"]]
            e["rotation_deg"] = round(((t["rotation_deg"] - room.rot + 90.0) % 180.0) - 90.0, 1)
        else:
            e["diameter_m"] = round(float(t["diameter_m"]), 3)
        if t.get("clipped"):
            e["clipped"] = True
        rt.append(e)
    rd = {"name": room.name, "w": room.w, "h": room.h}
    if room.walls:                                   # the walls, when they are not the box
        rd["floor_m"] = [[round(float(v), 3) for v in room.to_room(p)] for p in room.floor]
    return {"schema": SCHEMA, "source": source, "seq": seq,
            "captured_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
            "units": "metres", "room": rd, "tables": rt, "cameras": list(cameras)}


def map_room(room, venue, per_camera, log=None):
    """per_camera: {camera name: instances}. -> (tables, rejected) in venue metres."""
    tables, rejected = [], []
    for name, instances in per_camera.items():
        cfg = venue["_cams"][name]
        tb = map_instances(cfg, instances, venue, axes_deg=room.axes, snap_deg=room.snap)
        for t in tb:
            why = plausible(t, room, venue)
            if why:
                t["rejected"] = why
                rejected.append(t)
            else:
                tables.append(t)
    tables = fuse(tables)
    if log:
        log("%s: %d tables, %d rejected" % (room.name, len(tables), len(rejected)))
    return tables, rejected


# ================================================================ pictures for review
def draw_on_frame(img, cfg, instances, tables, top_h):
    """The camera's frame, its masks, and each fitted top drawn back into it."""
    import cctv_segment as cs
    out = cs.draw(img, [i for i in instances if i.cls == "table"], label=False)
    cam = cfg["_cam"]
    for t in tables:
        P = _poly(t)
        P = np.vstack([P, P[:1]])
        dense = np.concatenate([P[i] + np.linspace(0, 1, 12)[:, None] * (P[i + 1] - P[i])
                                for i in range(len(P) - 1)])
        uv, ok = cam.project(np.column_stack([dense, np.full(len(dense), -top_h)]))
        col = (0, 0, 255) if not t.get("rejected") else (160, 160, 160)
        if ok.sum() >= 2:
            cv2.polylines(out, [uv[ok].astype(np.int32)], True, col, 4, cv2.LINE_AA)
    return out


def draw_plan(venue, room, tables, rejected, px_per_m=60, plan=None):
    """The room from above: the venue's plan underneath if there is one, tables on top."""
    W, H = int(room.w * px_per_m), int(room.h * px_per_m)
    if plan is not None:
        # resample the venue plan into the room's frame
        s = venue["plan_m_per_px"]
        ys, xs = np.mgrid[0:H, 0:W].astype(np.float32) / px_per_m
        vx = room.o[0] + xs * room.ex[0] + ys * room.ey[0]
        vy = room.o[1] + xs * room.ex[1] + ys * room.ey[1]
        img = cv2.remap(plan, (vx / s).astype(np.float32), (vy / s).astype(np.float32),
                        cv2.INTER_AREA, borderValue=(255, 255, 255))
        img = cv2.addWeighted(img, 0.45, np.full_like(img, 255), 0.55, 0)
    else:
        img = np.full((H, W, 3), 255, np.uint8)

    def rp(p):
        q = room.to_room(p) * px_per_m
        return int(round(q[0])), int(round(q[1]))

    if room.walls:                                   # draw the outline as the walls
        cv2.polylines(img, [np.array([rp(p) for p in room.floor], np.int32)], True,
                      (40, 40, 40), 4, cv2.LINE_AA)
    for t in rejected:
        P = np.array([rp(p) for p in _poly(t)], np.int32)
        cv2.polylines(img, [P], True, (170, 170, 170), 1, cv2.LINE_AA)
    def qp(q):
        return int(round(q[0] * px_per_m)), int(round(q[1] * px_per_m))

    views = [camera_view(cfg, room) for name, cfg in venue["_cams"].items()
             if not room.cameras or name in room.cameras]
    for v in views:                                   # the floor each camera sees
        if len(v["view_m"]) >= 3:
            P = np.array([qp(q) for q in v["view_m"]], np.int32)
            over = img.copy()
            cv2.fillPoly(over, [P], (200, 120, 60))
            img = cv2.addWeighted(over, 0.12, img, 0.88, 0)
            cv2.polylines(img, [P], True, (200, 120, 60), 1, cv2.LINE_AA)
    for k, t in enumerate(tables):
        P = np.array([rp(p) for p in _poly(t)], np.int32)
        cv2.fillPoly(img, [P], (60, 170, 60) if len(t.get("cameras", [])) < 2 else (40, 140, 200))
        cv2.polylines(img, [P], True, (20, 80, 20), 2, cv2.LINE_AA)
        dims = ("%.1fx%.1f" % tuple(t["size_m"]) if t["shape"] == "rect"
                else "%.1f round" % t["diameter_m"])
        cx, cy = rp(t["centre_m"])
        cv2.putText(img, dims, (cx - 30, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, dims, (cx - 30, cy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for v in views:                                   # where it hangs, which way it faces
        x, y = v["at_m"]
        a = math.radians(v["heading_deg"])
        tip = (x + 1.5 * math.cos(a), y + 1.5 * math.sin(a))
        cv2.arrowedLine(img, qp((x, y)), qp(tip), (200, 40, 40), 3, cv2.LINE_AA, tipLength=0.3)
        cv2.circle(img, qp((x, y)), 8, (200, 40, 40), -1, cv2.LINE_AA)
        cv2.putText(img, v["label"], (qp((x, y))[0] + 12, qp((x, y))[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 40, 40), 2, cv2.LINE_AA)
    return img


# ================================================================ command line
class _LazySegmenter:
    """Loads the models only if a frame is not already in the cache."""

    def __init__(self, **kw):
        self.kw, self.seg = kw, None

    def segment(self, img):
        if self.seg is None:
            import cctv_segment as cs
            self.seg = cs.Segmenter(**self.kw)
        return self.seg.segment(img)


def _segment_cached(seg, img, name, cache):
    import hashlib
    import pickle
    path = None
    if cache:
        os.makedirs(cache, exist_ok=True)
        key = hashlib.sha1(img.tobytes()).hexdigest()[:16]
        path = os.path.join(cache, "%s_%s.pkl" % (name, key))
        if os.path.exists(path):
            with open(path, "rb") as f:
                return pickle.load(f)
    t0 = time.time()
    inst = seg.segment(img)
    print("%s: %d masks in %.1f s" % (name, len(inst), time.time() - t0))
    if path:
        with open(path, "wb") as f:
            pickle.dump(inst, f)
    return inst


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument("venue")
    ap.add_argument("--room", help="just this room (default: all)")
    ap.add_argument("--frame", nargs=2, action="append", metavar=("CAMERA", "IMAGE"),
                    help="use this image for a camera instead of a clean plate of its recording")
    ap.add_argument("--out", help="write the detections here (one room), or a directory")
    ap.add_argument("--review", metavar="DIR", help="write per-camera and plan pictures here")
    ap.add_argument("--cache", metavar="DIR", help="keep each frame's masks here and reuse them")
    args = ap.parse_args(argv)

    import cctv_segment as cs
    venue = load_venue(args.venue)
    rooms = [Room(r) for r in venue["rooms"] if not args.room or r["name"] == args.room]
    if not rooms:
        raise SystemExit("no room called %r" % args.room)
    frames = dict(args.frame or [])
    seg = _LazySegmenter(classes=venue.get("prompts"))
    plan = None
    if args.review and venue.get("plan_image"):
        plan = cv2.imread(os.path.join(venue["_base"], venue["plan_image"]))
    if args.review:
        os.makedirs(args.review, exist_ok=True)
    seen = {}
    for room in rooms:
        names = room.cameras or list(venue["_cams"])
        per_cam, imgs = {}, {}
        for name in names:
            cfg = venue["_cams"][name]
            if name not in seen:
                src = frames.get(name)
                img = cv2.imread(src) if src else cs.clean_plate_from_video(camera_source(cfg))
                seen[name] = (img, _segment_cached(seg, img, name, args.cache))
            imgs[name], per_cam[name] = seen[name]
        tables, rejected = map_room(room, venue, per_cam, log=print)
        doc = room_doc(room, tables, [camera_view(venue["_cams"][n], room) for n in names],
                       source=venue.get("name", ""))
        for t in rejected:
            print("   not a table (%s): %s at %.1f, %.1f" % (t["camera"], t["rejected"], *t["centre_m"]))
        if args.out:
            out = (args.out if args.out.endswith(".json") and len(rooms) == 1 else
                   os.path.join(args.out, room.name.replace(" ", "_").lower() + ".json"))
            os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
            with open(out, "w") as f:
                json.dump(doc, f, indent=1)
            print("wrote", out)
        if args.review:
            slug = room.name.replace(" ", "_").lower()
            for name in names:
                cfg = venue["_cams"][name]
                th = cfg["solved"].get("table_h_m", venue.get("table_height_m", 0.75))
                mine = [t for t in tables + rejected if t["camera"] == name or name in t.get("cameras", [])]
                im = draw_on_frame(imgs[name], cfg, per_cam[name], mine, th)
                cv2.imwrite(os.path.join(args.review, "%s_%s.jpg" % (slug, name)), im,
                            [cv2.IMWRITE_JPEG_QUALITY, 85])
            cv2.imwrite(os.path.join(args.review, "%s_plan.png" % slug),
                        draw_plan(venue, room, tables, rejected, plan=plan))
            print("review pictures in", args.review)


if __name__ == "__main__":
    main()
