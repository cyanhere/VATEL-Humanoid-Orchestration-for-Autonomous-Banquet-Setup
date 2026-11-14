#!/usr/bin/env python3
"""Head camera scale check: a checkerboard is slid along a taped line and the camera's depth and PnP readings are compared
with the tape. Reads the head camera only; nothing moves.

    python3 head_ruler.py                         # live; Enter -> type the tape reading (mm) -> Enter; q ends
    python3 head_ruler.py --square 20.5 --pattern 8x5
    python3 head_ruler.py --selftest              # laptop: rendered board + depth, true 1.000, and a 7 % wrong square
"""
import argparse
import json
import os
import select  # noqa: F401 (kept for the laptop tests)
import threading
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pk import config  # noqa: E402
from pk import handeye as he  # noqa: E402


# ----------------------------------------------------------------------------------------------------------- measuring
def corners(gray, pattern):
    import cv2
    from pk.calib_guide import find_board
    ok, c = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if ok:
        return c.reshape(-1, 2).astype(float)
    uv = find_board(gray, pattern, scale=1.0)
    return None if uv is None else uv.astype(float)


def K_of(intr):
    return np.array([[intr["fx"], 0, intr["cx"]], [0, intr["fy"], intr["cy"]], [0, 0, 1.0]])


def pnp(uv, intr, pattern, square):
    """Board in the colour optical frame by PnP: (centre xyz m, R, rms px)."""
    import cv2
    K = K_of(intr)
    X = he.board_points(pattern, square)
    ok, rv, tv = cv2.solvePnP(X, uv.astype(np.float64), K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R = cv2.Rodrigues(rv)[0]
    Pc = X @ R.T + tv.ravel()
    pr = Pc[:, :2] / Pc[:, 2:3] * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    rms = float(np.sqrt(np.mean(np.sum((pr - uv) ** 2, 1))))
    return Pc.mean(0), R, rms


def depth_board(uv, depth, scale, intr, pattern):
    """Board by depth only: a plane fitted (with outlier rejection) to the depth pixels inside the corners' outline,
    each corner's ray cut with it. (centre xyz m, normal, square m, plane rms mm, filled fraction) or None."""
    import cv2
    h, w = depth.shape
    hull = cv2.convexHull(uv.astype(np.float32)).reshape(-1, 2)
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, hull.astype(np.int32), 1)
    mask = cv2.erode(mask, np.ones((7, 7), np.uint8))                 # keep off the board's outline
    vs, us = np.nonzero(mask)
    if len(us) < 200:
        return None
    step = max(1, len(us) // 6000)
    us, vs = us[::step], vs[::step]
    z = depth[vs, us].astype(float) / scale
    fill = float(np.mean(z > 0))
    ok = z > 0
    us, vs, z = us[ok], vs[ok], z[ok]
    if len(z) < 150:
        return None
    P = np.c_[(us - intr["cx"]) / intr["fx"] * z, (vs - intr["cy"]) / intr["fy"] * z, z]
    keep = np.ones(len(P), bool)
    for _ in range(4):
        c = P[keep].mean(0)
        _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
        n = vt[2]
        r = (P - c) @ n
        s = 1.4826 * np.median(np.abs(r[keep] - np.median(r[keep])))
        keep = np.abs(r) < max(3 * s, 0.002)
    rms = float(np.sqrt(np.mean(r[keep] ** 2)))
    rays = np.c_[(uv[:, 0] - intr["cx"]) / intr["fx"], (uv[:, 1] - intr["cy"]) / intr["fy"], np.ones(len(uv))]
    t = (c @ n) / (rays @ n)
    X = rays * t[:, None]
    nx, ny = pattern
    g = X.reshape(ny, nx, 3)
    sp = np.r_[np.linalg.norm(np.diff(g, axis=1), axis=2).ravel(), np.linalg.norm(np.diff(g, axis=0), axis=2).ravel()]
    return X.mean(0), n, float(np.median(sp)), 1000 * rms, fill


def measure(frames, intr, scale, pattern, square):
    """Medians over several (colour, depth) frames. dict or None."""
    import cv2
    rows = []
    last_uv = None
    for col, dep in frames:
        uv = corners(cv2.cvtColor(col, cv2.COLOR_BGR2GRAY), pattern)
        if uv is None:
            continue
        p = pnp(uv, intr, pattern, square)
        d = depth_board(uv, dep, scale, intr, pattern) if dep is not None else None
        if p is None or d is None:
            continue
        rows.append(np.r_[p[0], d[0], d[2], p[2], d[3], d[4], np.linalg.norm(d[0]) - np.linalg.norm(p[0])])
        last_uv, R = uv, p[1]
    if len(rows) < max(2, len(frames) // 3):
        return None
    A = np.array(rows)
    m = np.median(A, 0)
    return dict(pnp_xyz=m[0:3].tolist(), depth_xyz=m[3:6].tolist(), depth_square_mm=1000 * m[6], pnp_rms_px=m[7],
                plane_rms_mm=m[8], depth_fill=m[9], depth_minus_pnp_mm=1000 * m[10], n=len(rows),
                pnp_spread_mm=1000 * float(np.median(np.linalg.norm(A[:, 0:3] - m[0:3], axis=1))),
                depth_spread_mm=1000 * float(np.median(np.linalg.norm(A[:, 3:6] - m[3:6], axis=1))),
                R=R.tolist(), uv=last_uv)


# ------------------------------------------------------------------------------------------------------------ analysis
def line_fit(P, tape):
    """Positions along the best straight line through P (sign: growing with the tape), the scatter off it (mm), and
    tape = slope-free check: s = k * tape + b."""
    P = np.asarray(P, float)
    c = P.mean(0)
    _, _, vt = np.linalg.svd(P - c, full_matrices=False)
    d = vt[0]
    s = (P - c) @ d
    if np.corrcoef(s, tape)[0, 1] < 0:
        s = -s
    off = np.linalg.norm((P - c) - np.outer((P - c) @ d, d), axis=1)
    A = np.c_[tape, np.ones(len(tape))]
    (k, b), *_ = np.linalg.lstsq(A, s, rcond=None)
    res = s - A @ [k, b]
    # slope uncertainty (1 sigma) from the residuals
    sig = float(np.sqrt(np.sum(res ** 2) / max(1, len(s) - 2) / np.sum((tape - tape.mean()) ** 2))) if len(s) > 2 else float("nan")
    return dict(slope=float(k), slope_sigma=sig, rms_mm=1000 * float(np.sqrt(np.mean(res ** 2))),
                off_line_mm=1000 * float(off.max()))


def report(pts, square_mm):
    if len(pts) < 3:
        print("need 3 or more places (have %d)" % len(pts))
        return None
    tape = np.array([p["tape_mm"] for p in pts], float) / 1000.0
    out = dict(n=len(pts), tape_span_mm=1000 * float(tape.max() - tape.min()))
    print("\n%d places, tape %.0f-%.0f mm (span %.0f mm)" % (len(pts), 1000 * tape.min(), 1000 * tape.max(), out["tape_span_mm"]))
    for key, name in (("pnp", "PnP (square %.2f mm)" % square_mm), ("depth", "depth (no square size)")):
        f = line_fit([p[key + "_xyz"] for p in pts], tape)
        out[key] = f
        print("  %-26s move / tape = %.4f +- %.4f   residual %.1f mm   off the line %.1f mm" % (
            name, f["slope"], f["slope_sigma"], f["rms_mm"], f["off_line_mm"]))
    sq = np.array([p["depth_square_mm"] for p in pts])
    out["depth_square_mm"] = float(np.mean(sq))
    out["depth_square_sd_mm"] = float(np.std(sq))
    dm = np.array([p["depth_minus_pnp_mm"] for p in pts])
    out["depth_minus_pnp_mm"] = [float(dm.min()), float(np.median(dm)), float(dm.max())]
    R0 = np.array(pts[0]["R"])
    flip = np.diag([-1.0, -1.0, 1.0])          # the 8x5 board found with its corners in reverse order = turned 180 deg in
    turn = [float(min(np.degrees(np.arccos(np.clip((np.trace(R0.T @ np.array(p["R"]) @ F) - 1) / 2, -1, 1)))
                      for F in (np.eye(3), flip))) for p in pts]   # plane; the centre is the same
    out["turn_deg_max"] = max(turn)
    print("  depth sees a square of %.2f +- %.2f mm (--square %.2f: ratio %.4f)" % (
        out["depth_square_mm"], out["depth_square_sd_mm"], square_mm, out["depth_square_mm"] / square_mm))
    print("  depth - PnP distance: %.1f / %.1f / %.1f mm (min / median / max)" % tuple(out["depth_minus_pnp_mm"]))
    print("  the iPad turned up to %.1f deg between places (keep it under ~2)" % out["turn_deg_max"])
    ok_p = abs(out["pnp"]["slope"] - 1) <= 0.01
    ok_d = abs(out["depth"]["slope"] - 1) <= 0.01
    ok_s = abs(out["depth_square_mm"] / square_mm - 1) <= 0.01
    print("\nVERDICT:")
    print("  PnP   %s (slope %.3f)" % ("a ruler" if ok_p else "NOT a ruler", out["pnp"]["slope"]))
    print("  depth %s (slope %.3f)" % ("a ruler" if ok_d else "NOT a ruler", out["depth"]["slope"]))
    print("  square size %s" % ("agrees with depth" if ok_s else "DISAGREES with depth: PnP scales by %.3f" % (out["depth_square_mm"] / square_mm)))
    if out["tape_span_mm"] < 150:
        print("  (span under 150 mm: slide further for a tighter slope)")
    out["verdict"] = dict(pnp_ruler=ok_p, depth_ruler=ok_d, square_agrees=ok_s)
    return out


# ------------------------------------------------------------------------------------------------------------ live run
def run(a, pattern):
    import cv2
    import perception as per
    cam = per.Camera()
    if not cam.colour:
        print("the head camera gives no colour (USB 2?): stop")
        return 1
    intr, scale = cam.intr_c, cam.scale
    out = os.path.join(config.LOG_DIR, "head_ruler", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    meta = dict(pattern=list(pattern), square_mm=a.square, intr_c=intr, depth_scale=scale, mode=list(cam.mode))
    pts = []
    kin = link = None
    if not a.no_pose:
        try:
            kin = per.Kin(a.perception, correct=True)
            link = per.StandaloneLink(a.iface, a.perception)                # reads rt/lowstate; never commands
        except BaseException as ex:                                          # noqa: BLE001 (SystemExit from Kin too)
            print("no head pose (%s): the view shows camera distances only" % ex)
            kin = link = None
    view = None
    if a.view_port:
        try:
            view = per.LiveView(port=a.view_port, wrist=False)
            print("live view: http://127.0.0.1:%d on the robot" % a.view_port)
        except OSError as ex:
            print("no live view (port %d busy? perception.py running? %s)" % (a.view_port, ex))

    def head_state():
        if link is None:
            return None, None
        try:
            st = link.state()
            return np.asarray(kin.T_po(kin.angles(st)), float), [float(st["head"][0]), float(st["head"][1]), float(st["waist_roll"])]
        except Exception:                                                    # noqa: BLE001
            return None, None

    def where(m, T_po):
        if m is None or T_po is None:
            return None
        return (T_po[:3, :3] @ np.asarray(m["pnp_xyz"]) + T_po[:3, 3]).tolist()

    def draw(col, m, P):
        img = col.copy()
        h, w = img.shape[:2]
        cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 0), cv2.MARKER_CROSS, 24, 1)
        lines = []
        if m is None:
            lines.append(("board NOT found: the whole board must be in the picture", (0, 0, 255)))
        else:
            uv = m["uv"]
            cv2.drawChessboardCorners(img, pattern, uv.astype(np.float32).reshape(-1, 1, 2), True)
            edge = min(uv[:, 0].min(), uv[:, 1].min(), w - uv[:, 0].max(), h - uv[:, 1].max())
            lines.append(("camera: PnP %.0f mm, depth %.0f mm, depth - PnP %+.1f mm" % (
                1000 * np.linalg.norm(m["pnp_xyz"]), 1000 * np.linalg.norm(m["depth_xyz"]), m["depth_minus_pnp_mm"]), (0, 255, 0)))
            lines.append(("depth sees a square of %.2f mm (screen %.0f %% filled)" % (m["depth_square_mm"], 100 * m["depth_fill"]), (0, 255, 0)))
            if P is not None:
                lines.append(("board centre: %.2f m forward of the pelvis, %.2f m %s of the robot's centre line" % (
                    P[0], abs(P[1]), "LEFT" if P[1] > 0 else "RIGHT"), (0, 255, 255)))
            if edge < 20:
                lines.append(("near the picture's edge (%.0f px): move it toward the cross" % edge, (0, 128, 255)))
        if pts:
            lines.append(("%d places saved, tape %.0f-%.0f mm (span %.0f; aim for 200+)" % (
                len(pts), min(q["tape_mm"] for q in pts), max(q["tape_mm"] for q in pts),
                max(q["tape_mm"] for q in pts) - min(q["tape_mm"] for q in pts)), (255, 255, 255)))
            if m is not None:
                lines.append(("from place 0: PnP %.1f mm, depth %.1f mm moved" % (
                    1000 * np.linalg.norm(np.subtract(m["pnp_xyz"], pts[0]["pnp_xyz"])),
                    1000 * np.linalg.norm(np.subtract(m["depth_xyz"], pts[0]["depth_xyz"]))), (255, 255, 255)))
        else:
            lines.append(("no place saved yet: type the tape reading (mm) in the test pane, then Enter", (255, 255, 255)))
        wmax = max(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] for t, _ in lines)
        sub = img[0:12 + 20 * len(lines), 0:min(w, wmax + 16)]
        sub[:] = (sub * 0.35).astype(np.uint8)                              # a dark panel under the text
        for i, (t, c) in enumerate(lines):
            cv2.putText(img, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
        return img
    print("head camera ruler test: nothing moves. Board %dx%d corners, square %.2f mm. Files: %s" % (
        pattern[0], pattern[1], a.square, out))
    print("slide the iPad along the tape (readings in mm: 25.4 cm = 254).")

    lock = threading.Lock()                    # the camera is read by the live-view thread and the main thread

    def grab(n):
        fr = []
        with lock:
            for _ in range(n):
                fs, _, _, _ = cam.frames()
                col, dep, _ = cam.aligned(fs)
                fr.append((col, dep))
        return fr

    def save():
        json.dump(dict(meta, points=[{k: v for k, v in p.items() if k != "uv"} for p in pts], result=res),
                  open(os.path.join(out, "points.json"), "w"), indent=1)

    now = dict(line="(no reading yet)")
    stop = threading.Event()

    def live():                                # live view + the latest status; never prints (the terminal is for typing)
        while not stop.is_set():
            try:
                fr_live = grab(2)
                m = measure(fr_live, intr, scale, pattern, a.square / 1000.0)
                T_po, _ = head_state()
                P = where(m, T_po)
                if view is not None and view.wanted(time.time()):
                    view.put(draw(fr_live[-1][0], m, P), time.time())
                if m is None:
                    now["line"] = "board NOT found (whole board in the picture?)"
                else:
                    now["line"] = "board found: PnP %.0f mm, depth %.0f mm" % (
                        1000 * np.linalg.norm(m["pnp_xyz"]), 1000 * np.linalg.norm(m["depth_xyz"]))
                    if P is not None:
                        now["line"] += ", %.2f m forward, %.2f m %s" % (P[0], abs(P[1]), "LEFT" if P[1] > 0 else "RIGHT")
            except Exception as ex:            # noqa: BLE001
                now["line"] = "live view error: %s" % ex
            stop.wait(0.3)

    th = threading.Thread(target=live, daemon=True)
    th.start()
    res = None
    print("\nFor each place: put the iPad there, let it settle, then type the tape reading in mm and press Enter.")
    print("Enter alone shows what the camera sees now. q + Enter finishes and prints the result.\n")
    try:
        while True:
            time.sleep(0.4)
            print("  camera now: %s" % now["line"])
            s = input("place %d -- tape reading (mm), Enter = refresh, q = finish: " % len(pts)).strip()
            if s.lower() == "q":
                break
            if not s:
                continue
            try:
                tape = float(s)
            except ValueError:
                print("  not a number: nothing saved")
                continue
            if 0 < tape < 100 and pts and min(q["tape_mm"] for q in pts) > 150:
                print("  %.1f looks like cm (earlier places are in mm): nothing saved, type mm" % tape)
                continue
            print("  reading %d frames, keep still ..." % a.frames)
            fr = grab(a.frames)
            m = measure(fr, intr, scale, pattern, a.square / 1000.0)
            if m is None:
                print("\n  board not found in enough of %d frames: nothing saved, try again" % len(fr))
                continue
            k = len(pts)
            col = fr[-1][0].copy()
            if m["uv"] is not None:
                cv2.drawChessboardCorners(col, pattern, m["uv"].astype(np.float32).reshape(-1, 1, 2), True)
            cv2.imwrite(os.path.join(out, "place_%02d.jpg" % k), col, [cv2.IMWRITE_JPEG_QUALITY, 92])
            cv2.imwrite(os.path.join(out, "place_%02d_depth.png" % k), fr[-1][1])
            T_po, head = head_state()
            m.update(tape_mm=tape, file="place_%02d.jpg" % k, pelvis_xyz=where(m, T_po), head=head)
            if head is not None and pts and pts[0].get("head") is not None and \
                    max(abs(x - y) for x, y in zip(head[:2], pts[0]["head"][:2])) > 0.003:   # head pitch, yaw only:
                # waist_roll's reading jitters 0.013 rad in 6 s with the picture still, so it is not checked
                print("\n  !! the head moved since place 0 (%s -> %s rad): the moves are not comparable; start again" % (
                    np.round(pts[0]["head"], 4).tolist(), np.round(head, 4).tolist()))
            pts.append(m)
            print("\n  %d: tape %.0f mm | PnP %.1f mm (+-%.1f) | depth %.1f mm (+-%.1f) | depth square %.2f mm | %d frames" % (
                k, tape, 1000 * np.linalg.norm(m["pnp_xyz"]), m["pnp_spread_mm"], 1000 * np.linalg.norm(m["depth_xyz"]),
                m["depth_spread_mm"], m["depth_square_mm"], m["n"]))
            if len(pts) >= 2:
                d_t = abs(pts[-1]["tape_mm"] - pts[0]["tape_mm"])
                d_p = 1000 * np.linalg.norm(np.subtract(pts[-1]["pnp_xyz"], pts[0]["pnp_xyz"]))
                d_d = 1000 * np.linalg.norm(np.subtract(pts[-1]["depth_xyz"], pts[0]["depth_xyz"]))
                print("     since place 0: tape %.1f mm, PnP %.1f mm, depth %.1f mm" % (d_t, d_p, d_d))
            save()
    except (KeyboardInterrupt, EOFError):
        print("\nstopped")
    finally:
        stop.set()
        th.join(2.0)
        cam.pipe.stop()
    res = report(pts, a.square)
    save()
    print("files in %s -- pull the logs" % out)
    return 0


# ------------------------------------------------------------------------------------------------------------ selftest
def render(pattern, square_true, T_cb, intr, size=(848, 480), noise_px=0.3, depth_noise=0.0015, rs=None):
    """A colour + depth frame of a 9x6-square board (pattern = inner corners) at T_cb (camera <- board), pinhole."""
    import cv2
    w, h = size
    nx, ny = pattern
    cell = 40
    W, H = (nx + 3) * cell, (ny + 3) * cell                                 # one white square of margin round the board
    tex = np.full((H, W), 235, np.uint8)
    for j in range(ny + 1):
        for i in range(nx + 1):
            if (i + j) % 2 == 0:
                tex[(j + 1) * cell:(j + 2) * cell, (i + 1) * cell:(i + 2) * cell] = 20
    # board coordinates of texture pixels: inner corner (0,0) at texture (2*cell, 2*cell)
    s = square_true / cell
    tp = np.float32([[0, 0], [W, 0], [W, H], [0, H]])
    bp = np.c_[(tp - 2 * cell) * s, np.zeros(4)]
    K = K_of(intr)
    Pc = bp @ T_cb[:3, :3].T + T_cb[:3, 3]
    uv = (Pc[:, :2] / Pc[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    M = cv2.getPerspectiveTransform(tp - 0.5, uv.astype(np.float32))      # texture pixel edges at -0.5 (centres at integers)
    ss = 4                                                                 # render 4x finer, then average: a sensor's pixels
    S = np.array([[ss, 0, (ss - 1) / 2.0], [0, ss, (ss - 1) / 2.0], [0, 0, 1]])
    big = cv2.warpPerspective(tex, S @ M, (w * ss, h * ss), borderValue=60, flags=cv2.INTER_LINEAR)
    col = cv2.resize(big, (w, h), interpolation=cv2.INTER_AREA)
    col = cv2.GaussianBlur(col, (3, 3), 0.6)
    if rs is not None:
        col = np.clip(col + rs.normal(0, 2.0, col.shape), 0, 255).astype(np.uint8)
    col = cv2.cvtColor(col, cv2.COLOR_GRAY2BGR)
    # depth: the board's plane where the board is, a wall 1.5 m away elsewhere
    n = T_cb[:3, 2]
    d0 = n @ T_cb[:3, 3]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    rays = np.dstack([(us - K[0, 2]) / K[0, 0], (vs - K[1, 2]) / K[1, 1], np.ones((h, w))])
    t = d0 / (rays @ n)
    z = t.copy()
    inb = cv2.warpPerspective(np.ones((H, W), np.uint8), M, (w, h)) > 0
    z[~inb] = 1.5
    if rs is not None:
        z = z + rs.normal(0, depth_noise, z.shape) * z / 0.6
    scale = 1000.0
    dep = np.clip(z * scale, 0, 65535).astype(np.uint16)
    return col, dep, scale


def selftest():
    pattern = (8, 5)
    intr = dict(fx=605.0, fy=605.0, cx=424.0, cy=240.0)
    rs = np.random.RandomState(0)
    ok_all = True
    for name, sq_true, sq_assumed, want_p in (("true square", 0.0205, 0.0205, 1.0), ("square 7 % larger than assumed", 0.0220, 0.0205, 0.0205 / 0.0220)):
        pts, truth = [], []
        base = np.array([0.02, 0.06, 0.55])
        slide = np.array([0.15, 0.0, 0.99])
        slide /= np.linalg.norm(slide)
        for k, tape in enumerate([300, 330, 360, 390, 420, 450, 480]):
            ang = np.radians(200.0)                                          # board leaning back, turned a little
            Rx = np.array([[1, 0, 0], [0, np.cos(ang), -np.sin(ang)], [0, np.sin(ang), np.cos(ang)]])
            yaw = np.radians(8.0 + rs.normal(0, 0.3))
            Ry = np.array([[np.cos(yaw), 0, np.sin(yaw)], [0, 1, 0], [-np.sin(yaw), 0, np.cos(yaw)]])
            T = np.eye(4)
            T[:3, :3] = Ry @ Rx
            ctr = base + slide * (tape - 300) / 1000.0
            T[:3, 3] = ctr - T[:3, :3] @ np.r_[(pattern[0] - 1) * sq_true / 2, (pattern[1] - 1) * sq_true / 2, 0]
            fr = []
            for _ in range(4):
                col, dep, scale = render(pattern, sq_true, T, intr, rs=rs)
                fr.append((col, dep))
            m = measure(fr, intr, scale, pattern, sq_assumed)
            if m is None:
                print("  %s: board not found at tape %d" % (name, tape))
                ok_all = False
                break
            m["tape_mm"] = tape + rs.normal(0, 0.5)                         # a tape read to +-0.5 mm
            pts.append(m)
            truth.append(ctr)
        print("== %s (true %.1f mm, assumed %.1f mm)" % (name, 1000 * sq_true, 1000 * sq_assumed))
        r = report(pts, 1000 * sq_assumed)
        if r is None:
            ok_all = False
            continue
        # the reference: the TRUE centres against the same (noisy) tape readings, so the tape's own error is not
        # charged to the method (0.5 mm tape noise alone moves the slope by up to ~0.6 % over 180 mm)
        ref = line_fit(truth, np.array([p["tape_mm"] for p in pts]) / 1000.0)["slope"]
        good = (abs(r["pnp"]["slope"] - ref * want_p) < 0.003 and abs(r["depth"]["slope"] - ref) < 0.003
                and abs(r["depth_square_mm"] - 1000 * sq_true) < 0.15)
        print("  %s: PnP slope %.4f (want %.4f), depth slope %.4f (want %.4f), depth square %.2f mm (true %.2f); "
              "the true positions against these tape readings give %.4f" % (
                  "PASS" if good else "FAIL", r["pnp"]["slope"], ref * want_p, r["depth"]["slope"], ref,
                  r["depth_square_mm"], 1000 * sq_true, ref))
        ok_all &= good
    print("selftest %s (rendered pinhole frames + plane depth with noise; not a simulator)" % ("PASS" if ok_all else "FAIL"))
    return 0 if ok_all else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--square", type=float, default=20.5, help="checkerboard square on the iPad, mm (caliper)")
    ap.add_argument("--pattern", default="8x5", help="inner corners, as calib_wrist.py")
    ap.add_argument("--frames", type=int, default=15, help="frames per place (median)")
    ap.add_argument("--view-port", type=int, default=8090, help="live view port on the robot (0: off)")
    ap.add_argument("--no-pose", action="store_true", help="do not read the head joints (no pelvis-frame position)")
    ap.add_argument("--iface", default="eth10")
    ap.add_argument("--perception", default=config.PERCEPTION_DIR)
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    return run(a, he.parse_pattern(a.pattern))


if __name__ == "__main__":
    sys.exit(main())
