#!/usr/bin/env python3
"""Find the plate stack with R1_Plate_Perception, then track the top plate's rim and publish it to the arm server.

    python3 perception.py                         # with server.py running (it moves the head through the server)
    python3 perception.py --expect 0.36 0.06      # where to look first, pelvis frame (x forward, y left), m
    python3 perception.py --standalone            # no server: reads the joints itself, does not move the head
    python3 perception.py --save-frames 5         # also keep a raw frame every 5 s (logs/perception/frames, for offline work)
    python3 perception.py --record-hz 5           # while a test holds the head still: colour + depth + joints, 5 per s, in
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pk import config  # noqa: E402
from pk.model import PINKY_END, Robot  # noqa: E402
from pk.marker import MarkerTrack  # noqa: E402
from pk.tracker import RimTracker, project  # noqa: E402

OUT = os.path.join(config.LOG_DIR, "perception")


# ============================================================================ camera
class Camera(object):
    """A RealSense kept open: depth (+ colour) frames, depth intrinsics, depth->colour extrinsics."""
    MODES = [(848, 480, 30), (848, 480, 15), (640, 480, 30), (640, 480, 15)]

    def __init__(self):
        import pyrealsense2 as rs
        self.rs = rs
        last = None
        for colour in (True, False):
            for w, h, f in self.MODES:
                try:
                    pipe, cfg = rs.pipeline(), rs.config()
                    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, f)
                    if colour:
                        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, f)
                    prof = pipe.start(cfg)
                except RuntimeError as e:
                    last = e
                    continue
                self.pipe, self.colour, self.mode = pipe, colour, (w, h, f)
                dp = prof.get_stream(rs.stream.depth).as_video_stream_profile()
                self.intr_d = self._intr(dp.get_intrinsics())
                self.scale = 1.0 / prof.get_device().first_depth_sensor().get_depth_scale()
                self.T_cd = np.eye(4)
                if colour:
                    cp = prof.get_stream(rs.stream.color).as_video_stream_profile()
                    self.intr_c = self._intr(cp.get_intrinsics())
                    e = dp.get_extrinsics_to(cp)
                    self.T_cd[:3, :3] = np.array(e.rotation).reshape(3, 3).T      # column-major in librealsense
                    self.T_cd[:3, 3] = e.translation
                    self.align = rs.align(rs.stream.color)
                else:
                    self.intr_c, self.align = self.intr_d, None
                for _ in range(15):                                            # auto exposure settles
                    pipe.wait_for_frames(2000)
                print("camera: %d x %d @ %d fps%s" % (w, h, f, "" if colour else ", DEPTH ONLY (USB 2?)"))
                return
        raise RuntimeError("the RealSense would not start (%s)" % last)

    @staticmethod
    def _intr(i):
        return {"fx": i.fx, "fy": i.fy, "cx": i.ppx, "cy": i.ppy, "width": i.width, "height": i.height}

    def frames(self, timeout_ms=1000):
        fs = self.pipe.wait_for_frames(timeout_ms)
        t = time.time()
        d = np.asanyarray(fs.get_depth_frame().get_data()).copy()
        c = np.asanyarray(fs.get_color_frame().get_data()).copy() if self.colour else None
        return fs, d, c, t

    def aligned(self, fs):
        """(colour, depth aligned to colour, colour intrinsics): what R1_Plate_Perception was calibrated with."""
        if self.align is None:
            return None, np.asanyarray(fs.get_depth_frame().get_data()).copy(), self.intr_d
        a = self.align.process(fs)
        return (np.asanyarray(a.get_color_frame().get_data()).copy(), np.asanyarray(a.get_depth_frame().get_data()).copy(),
                self.intr_c)


class Recorder(object):
    """Colour, depth and the joints to disk from its own thread, dropping frames rather than slowing the tracker. For looking at a test
    afterwards, and for replaying the rim tracker on it (frames.csv has T_po; the depth is not aligned, camera.json has intr_d and T_cd)."""

    def __init__(self, cam, hz, out):
        import queue
        import threading
        self.period, self.q, self.t_last, self.dropped, self.n = 1.0 / hz, queue.Queue(maxsize=4), 0.0, 0, 0
        self.dir = None
        self.cam, self.out, self.queue_full = cam, out, queue.Full
        threading.Thread(target=self.loop, daemon=True).start()

    def start_dir(self):
        self.dir = os.path.join(self.out, "rec_" + time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(os.path.join(self.dir, "frames"), exist_ok=True)
        c = self.cam
        json.dump(dict(intr_c=c.intr_c, intr_d=c.intr_d, depth_units_per_m=c.scale, T_cd=np.asarray(c.T_cd).tolist(), mode=list(c.mode),
                       aligned="NOT aligned: depth with intr_d, T_cd to the colour frame"), open(os.path.join(self.dir, "camera.json"), "w"), indent=1)
        self.csv = open(os.path.join(self.dir, "frames.csv"), "w")
        self.csv.write("idx,t,colour,depth,st,T_po\n")
        print("recording the camera for the test: %s" % self.dir)

    def put(self, now, colour, depth, st, T, on):
        if not on:
            self.dir = None
            return
        if now - self.t_last < self.period:
            return
        self.t_last = now
        if self.dir is None:
            self.start_dir()
        try:
            self.q.put_nowait((self.dir, self.csv, now, colour, depth, st, T))
        except self.queue_full:
            self.dropped += 1

    def loop(self):
        import cv2
        while True:
            d, f, now, colour, depth, st, T = self.q.get()
            fc, fd = "frames/c%05d.jpg" % self.n, "frames/d%05d.png" % self.n
            if colour is not None:
                cv2.imwrite(os.path.join(d, fc), colour, [cv2.IMWRITE_JPEG_QUALITY, 92])
            cv2.imwrite(os.path.join(d, fd), depth.astype(np.uint16))
            keep = {k: st.get(k) for k in ("q", "head", "head_cmd", "waist_roll", "hands")}
            f.write("%d,%.4f,%s,%s,\"%s\",\"%s\"\n" % (self.n, now, fc, fd, json.dumps(keep).replace('"', "'"), json.dumps(np.round(T, 6).tolist())))
            f.flush()
            self.n += 1


# ============================================================================ the robot's view of itself
class Kin(object):
    def __init__(self, perception_dir, correct=True):
        """correct: apply model/camera_correction.json (calib_cam.py + calibration/calib_fit.py: where the camera really sits
        on the head) to R1_Plate_Perception's calibration in memory; its files are not changed."""
        sys.path.insert(0, perception_dir)
        import camera_calibration as cc
        import r1_config as rcfg
        import urdf_fk as fk
        self.cc, self.rcfg, self.fk = cc, rcfg, fk
        self.cal = cc.load()
        if self.cal is None:
            raise SystemExit("no camera_calibration.json in %s: run its check_mount.py --calibrate first" % perception_dir)
        self.joints = fk.load_urdf(rcfg.URDF_PATH)
        self.rob = Robot()
        self.head_off = np.zeros(2)
        self.correction = None
        path = os.path.join(config.MODEL_DIR, "camera_correction.json")
        if correct and os.path.exists(path):
            corr = json.load(open(path))
            self.cal = dict(self.cal, T_headyaw_optical=(np.array(corr["T_hy_corr"], float)
                                                         @ np.array(self.cal["T_headyaw_optical"], float)).tolist())
            self.head_off = np.array(corr.get("head_offset", [0.0, 0.0]), float)
            self.correction = corr
        self.shift_mm = config.head_shift_mm()
        if self.shift_mm.any():
            print("head shift ON: %s mm added to what the head camera locates" % np.round(self.shift_mm, 1))

    def angles(self, st):
        q = st["q"]
        return {"head_pitch_joint": st["head"][0] + self.head_off[0], "head_yaw_joint": st["head"][1] + self.head_off[1],
                "waist_roll_joint": st["waist_roll"], "waist_yaw_joint": q[config.WAIST]}

    def T_po(self, ang):
        T = np.array(self.cc.T_pelvis_optical(self.cal, self.joints, ang), float)
        if self.shift_mm.any():
            T[:3, 3] += self.shift_mm / 1000.0
        return T

    def aim(self, target, ang):
        a = self.fk.aim_head_at(self.joints, np.asarray(target, float), {k: ang[k] for k in ("waist_roll_joint", "waist_yaw_joint")})
        return np.array([a["head_pitch_joint"], a["head_yaw_joint"]]), a["clamped"]

    def spheres(self, st):
        """The robot's own arms and hands in view, as spheres (pelvis frame) to cut out of the depth."""
        T = self.rob.fk(np.array(st["q"]), np.array(st["hands"]))
        out = []
        for s in ("left", "right"):
            for link, r in (("%s_elbow_link" % s, 0.06), ("%s_wrist_roll_link" % s, 0.06), ("%s_base_link" % s, 0.07)):
                out.append((T[link][:3, 3].copy(), r))
            for f in ("thumb", "index", "middle", "ring", "pinky"):
                for part in ("proximal", "distal"):
                    out.append((T["%s_%s_%s_link" % (s, f, part)][:3, 3].copy(), 0.028))
            # the finger TIPS (the distal links are alike; the little finger's end point serves for all four): the
            # curled little finger's tip at the rim's height was not cut out
            for f in ("index", "middle", "ring", "pinky"):
                M = T["%s_%s_distal_link" % (s, f)]
                out.append((M[:3, :3] @ PINKY_END[s] + M[:3, 3], 0.022))
            # along the forearm (elbow -> wrist)
            a, b = T["%s_elbow_link" % s][:3, 3], T["%s_wrist_roll_link" % s][:3, 3]
            for u in (0.33, 0.66):
                out.append((a + u * (b - a), 0.055))
        return out


# ============================================================================ the state source
class ServerLink(object):
    def __init__(self):
        from pk.client import Client
        self.c = Client("perception", role="head")
        self.can_aim = True

    def state(self):
        return self.c.state(max_age=0.5)

    def head(self, target):
        self.c.head(target)

    def publish(self, plate):
        self.c.publish_plate(plate)
        self.c.alive()

    def busy(self, s):
        return self.c.busy(s)


class StandaloneLink(object):
    """No server: read the joints straight from rt/lowstate (R1_Plate_Perception's R1Link), never move anything."""

    def __init__(self, iface, perception_dir):
        sys.path.insert(0, perception_dir)
        import r1_head
        self.link = r1_head.R1Link(iface).wait()
        self.can_aim = False

    def state(self):
        j = self.link.joints()
        q = [j["left_" + n] for n in config.URDF_ARM] + [j["right_" + n] for n in config.URDF_ARM] + [j["waist_yaw_joint"]]
        return dict(q=q, head=[j["head_pitch_joint"], j["head_yaw_joint"]], head_cmd=[j["head_pitch_joint"], j["head_yaw_joint"]],
                    waist_roll=j["waist_roll_joint"], hands=[0.0] * 12, req=None)

    def head(self, target):
        pass

    def publish(self, plate):
        pass

    def busy(self, s):
        import contextlib
        return contextlib.nullcontext() if hasattr(contextlib, "nullcontext") else _Null()


class _Null(object):
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


# ============================================================================ locate (R1_Plate_Perception)
def locate(cam, kin, link, args):
    import plate_detectors as pd
    import plate_stack as ps
    fs, _, _, t = cam.frames()
    st = link.state()
    ang = kin.angles(st)
    colour, depth, intr = cam.aligned(fs)
    with link.busy(30):
        try:
            plane = pd.dominant_plane(depth, intr, cam.scale)
            cands = ps.locate_stacks(depth, intr, plane, depth_scale=cam.scale, plate_diameter_m=args.diameter,
                                     plate_pitch_m=args.pitch, plate_rim_height_m=args.rim_height, allow_clipped=False)
        except (ps.NoStackFound, RuntimeError) as e:
            return None, str(e), (colour, depth, intr, ang)
    st2 = link.state()
    moved = max(abs(st2["head"][i] - st["head"][i]) for i in range(2))
    if moved > math.radians(0.5):
        return None, "the head moved during the capture", (colour, depth, intr, ang)
    best = cands[0]
    r = ps.to_robot_frame(best, joints=kin.joints, angles=ang, calibration=kin.cal)
    c = np.asarray(r["center_m"], float)
    out = dict(center=[float(v) for v in c], top_height=float(best["top_height_m"]), table_z=float(c[2] - best["top_height_m"]),
               diameter=float(best["diameter_m"]), n_plates=best.get("n_plates"), t=t,
               normal=[float(v) for v in r["normal"]], head=[float(ang["head_pitch_joint"]), float(ang["head_yaw_joint"])],
               table_above_floor=r.get("table_above_floor_m"), top_above_floor=r.get("top_above_floor_m"))
    return out, "", (colour, depth, intr, ang)


# ============================================================================ main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--expect", type=float, nargs=2, default=list(config.STACK_XY_SIM), help="first place to look (x y, pelvis frame)")
    ap.add_argument("--expect-z", type=float, default=0.07, help="expected rim height in the pelvis frame (m)")
    ap.add_argument("--diameter", type=float, default=0.255)
    ap.add_argument("--pitch", type=float, default=0.0068, help="nesting pitch (m), for the plate count")
    ap.add_argument("--rim-height", type=float, default=0.022, help="a single plate's rim height above the table (m)")
    ap.add_argument("--perception", default=config.PERCEPTION_DIR)
    ap.add_argument("--standalone", action="store_true")
    ap.add_argument("--iface", default="eth10")
    ap.add_argument("--save-frames", type=float, default=0.0, help="keep a raw frame every N s (0: never)")
    ap.add_argument("--record-hz", type=float, default=0.0, help="while a test holds the head still, save colour + depth + joints this many times per s (0: off)")
    ap.add_argument("--no-follow", action="store_true", help="never re-aim the head while tracking")
    ap.add_argument("--view-port", type=int, default=8090, help="live view in a browser (0: off)")
    ap.add_argument("--view-host", default="127.0.0.1", help="127.0.0.1: the robot only (ssh tunnel); 0.0.0.0: the whole network")
    ap.add_argument("--wrist-view", choices=("on", "off"), default="on",
                    help="add the two wrist cameras under the head camera in the live view (opened only while someone watches)")
    ap.add_argument("--no-correction", action="store_true", help="ignore model/camera_correction.json")
    a = ap.parse_args()
    os.makedirs(os.path.join(OUT, "frames"), exist_ok=True)
    np.seterr(invalid="ignore")
    kin = Kin(a.perception, correct=not a.no_correction)
    if kin.correction is not None:
        print("camera correction: %s (model/camera_correction.json, fitted %s)" % (kin.correction.get("summary", ""), kin.correction.get("date", "?")))
    else:
        print("camera correction: NONE (R1_Plate_Perception's calibration as it is)")
    link = StandaloneLink(a.iface, a.perception) if a.standalone else ServerLink()
    cam = Camera()
    seq, last_req, head_lock = 0, None, False
    status, located, trk = "aim", None, None
    look = np.array([a.expect[0], a.expect[1], a.expect_z])
    lost_since, t_snap, t_save, fps, t_prev = None, 0.0, 0.0, 0.0, time.time()
    head_prev = hc_prev = None
    marker, mk = None, None
    rec = Recorder(cam, a.record_hz, OUT) if a.record_hz > 0 else None
    print(__doc__.split("What it does")[0].strip())
    view = None
    if a.view_port:
        try:
            view = LiveView(a.view_host, a.view_port, wrist=(a.wrist_view == "on"))
            print("live view: http://%s:%d" % (a.view_host, a.view_port))
        except OSError as ex:
            print("live view off: port %d: %s" % (a.view_port, ex))

    def publish(est=None, why=""):
        nonlocal seq
        seq += 1
        st = dict(seq=seq, t=round(time.time(), 3), status=status, ok=bool(est and est.get("ok")), why=why or (est or {}).get("why", ""),
                  located=located, fps=round(fps, 1), head_lock=head_lock)
        if est is not None and located is not None:
            moved = np.array(est["center"][:2]) - trk.start
            c = np.array(located["center"][:2]) + moved                 # absolute = located + tracked travel
            st.update(center=[float(c[0]), float(c[1]), float(est["center"][2])], track_center=est["center"],
                      move_xy_mm=[float(1000 * v) for v in moved], moved_mm=float(1000 * np.linalg.norm(moved)),
                      n=est.get("n"), coverage=est.get("coverage"), rms_mm=est.get("rms_mm"),
                      top_height=float(est["center"][2] - located["table_z"]))
            if est.get("ok") and est.get("plane"):
                st["plane"] = est["plane"]                                   # the tilted rim (pk/tracker.py fit_plane)
        if mk is not None:
            st["marker"] = mk
        link.publish(st)
        return st

    def wait_head(target, timeout=6.0):
        """Until the server's head command has reached the target and the head has stopped. The head may rest a few
        degrees off its command under its own weight; the MEASURED angles are what
        the camera pose uses, so that offset does not matter."""
        t0 = time.time()
        still, prev = 0, None
        while time.time() - t0 < timeout:
            link.head(target)
            st = link.state()
            hd = np.array(st["head"], float)
            there = np.max(np.abs(np.array(st["head_cmd"], float) - np.asarray(target, float))) < 0.005
            if there and prev is not None and np.max(np.abs(hd - prev)) < 0.002:
                still += 1
                if still > 15:
                    return True
            else:
                still = 0
            prev = hd
            publish(why="aiming the head")
            time.sleep(0.02)
        return False

    while True:
        try:
            st = link.state()
        except Exception as ex:                               # server restarted or gone: reconnect
            if a.standalone:
                raise
            print("server: %s -- reconnecting" % ex)
            time.sleep(2.0)
            try:
                link = ServerLink()
                status = "aim"
            except Exception as ex2:                          # noqa: BLE001
                print("  not yet: %s" % ex2)
            continue
        req = st.get("req")
        if req and req.get("seq") != (last_req or {}).get("seq"):
            if last_req is not None or time.time() - req.get("t", 0) < 5.0:
                w = req.get("what")
                if w == "locate":
                    status = "aim" if link.can_aim else "locate"
                elif w == "aim" and req.get("args", {}).get("target"):
                    look = np.array(req["args"]["target"], float)
                    status = "aim"
                elif w == "head_lock":
                    head_lock = bool(req.get("args", {}).get("on", True))
                print("request from the test: %s %s" % (w, req.get("args")))
            last_req = req
        if status == "aim":
            if link.can_aim:
                tgt, clamped = kin.aim(look, kin.angles(st))
                print("aiming the head at (%.3f %+.3f %.3f): pitch %.1f deg, yaw %+.1f deg%s" % (
                    look[0], look[1], look[2], math.degrees(tgt[0]), math.degrees(tgt[1]), " (CLAMPED)" if clamped else ""))
                wait_head(tgt)
            status = "locate"
            continue
        if status == "locate":
            publish(why="locating (R1_Plate_Perception, ~5 s)")
            t0 = time.time()
            loc, why, frame = locate(cam, kin, link, a)
            if loc is None:
                print("locate failed (%.1f s): %s" % (time.time() - t0, why))
                publish(why="locate failed: " + why)
                time.sleep(0.5)
                continue
            located = loc
            print("LOCATED in %.1f s: centre (%.3f %+.3f %.3f) m, top %.1f mm above the table (%s plates), diameter %.1f mm, table %.3f m above the floor"
                  % (time.time() - t0, loc["center"][0], loc["center"][1], loc["center"][2], 1000 * loc["top_height"], loc["n_plates"],
                     1000 * loc["diameter"], loc["table_above_floor"] or float("nan")))
            json.dump(loc, open(os.path.join(OUT, "located.json"), "w"), indent=1)
            snapshot(frame[0], frame[2], kin.T_po(frame[3]), loc["center"], a.diameter / 2, os.path.join(OUT, "located.jpg"), "located")
            trk = RimTracker(loc["center"][:2], loc["center"][2], loc["table_z"], plate_r=a.diameter / 2)
            fs, depth, colour, t = cam.frames()
            st = link.state()
            est = trk.init(depth, cam.intr_d, cam.scale, kin.T_po(kin.angles(st)), kin.spheres(st), cam.T_cd)
            print("tracker start: %s" % ({k: est.get(k) for k in ("ok", "n", "coverage", "rms_mm", "why")}))
            sag = np.array(st["head"], float) - np.array(st["head_cmd"], float)
            if np.max(np.abs(sag)) > 0.02:
                print("(the head rests %+.3f %+.3f rad off its command, its own weight: fine, the measured angles are used)" % tuple(sag))
            look = np.array(loc["center"])
            status, lost_since = "tracking", None
            # the marker lies on the well floor: rim top - (22 - 7.35) mm
            marker, mk = MarkerTrack(loc["center"][:2], loc["center"][2] - 0.01465, cam.intr_c), None
            head_prev = hc_prev = None
            continue
        # ---- tracking
        try:
            fs, depth, colour, t = cam.frames()
        except RuntimeError as e:
            publish(why="camera: %s" % e)
            continue
        st = link.state()
        now = time.time()
        fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-3)
        t_prev = now
        # skip frames while the head MOVES (its angle or its command changing), not when it merely rests off its
        # command: it rested 0.045 rad low and every frame was skipped -- no plate reached the tests
        hd, hc = np.array(st["head"], float), np.array(st["head_cmd"], float)
        moving = ((head_prev is not None and np.max(np.abs(hd - head_prev)) > 0.003)
                  or (hc_prev is not None and np.max(np.abs(hc - hc_prev)) > 1e-4))
        head_prev, hc_prev = hd, hc
        if moving:
            publish(why="head moving")
            if view is not None and view.wanted(now):
                view.put(draw_view(colour, cam.intr_c, kin.T_po(kin.angles(st)), trk, dict(ok=False, why="head moving"),
                                   status, fps, head_lock, None), now)
            continue
        T = kin.T_po(kin.angles(st))
        sph = kin.spheres(st)
        est = trk.update(depth, cam.intr_d, cam.scale, T, sph, cam.T_cd)
        mk = marker.update(colour, T) if marker is not None else None
        out = publish(est)
        if rec is not None:
            rec.put(now, colour, depth, st, T, head_lock)
        if view is not None and view.wanted(now):
            view.put(draw_view(colour, cam.intr_c, T, trk, est, status, fps, head_lock, sph, pub=out), now)
        if est["ok"]:
            lost_since = None
            # head follow: keep the plate in the middle 70 % of the picture
            if link.can_aim and not head_lock and not a.no_follow:
                c = np.array(out["center"])
                P_o = (c[None, :] - T[:3, 3]) @ T[:3, :3]
                uv = project(P_o, cam.intr_d)[0]
                W, H = cam.intr_d["width"], cam.intr_d["height"]
                if not (0.15 * W < uv[0] < 0.85 * W and 0.15 * H < uv[1] < 0.85 * H):
                    look = c
                    status = "aim"
                    print("plate near the picture's edge: re-aiming the head")
        else:
            lost_since = lost_since or now
            if now - lost_since > 1.5:
                look = np.array(out.get("center") or located["center"])
                # a test that locked the head keeps it still: locate again from this view
                status = "locate" if head_lock else "aim"
                print("plate lost for 1.5 s (%s): locating again%s" % (est.get("why"), " (head locked: from here)" if head_lock else ""))
        if now - t_snap > 2.0 and colour is not None:
            t_snap = now
            snapshot(colour, cam.intr_c, T, out.get("center", located["center"]),
                     trk.r_band, os.path.join(OUT, "view.jpg"), "%s %s moved %.1f mm" % (status, "ok" if est["ok"] else est.get("why", ""), out.get("moved_mm", 0.0)))
            json.dump(out, open(os.path.join(OUT, "plate.json"), "w"), indent=1)
        if a.save_frames > 0 and now - t_save > a.save_frames:
            t_save = now
            np.savez_compressed(os.path.join(OUT, "frames", time.strftime("f_%H%M%S")), depth=depth, color=colour if colour is not None else np.zeros(1),
                                intr=json.dumps(cam.intr_d), intr_c=json.dumps(cam.intr_c), T_cd=cam.T_cd, depth_scale=cam.scale,
                                state=json.dumps(st), T_po=T)


# ============================================================================ live view (a browser on the VM)
class LiveView(object):
    """The camera picture with what the tracker uses drawn on it, as an MJPEG stream: http://<host>:<port>/.
    Bound to 127.0.0.1 by default (use an ssh tunnel to view it). Frames are drawn only while someone watches."""

    PAGE = (b"<!doctype html><html><head><meta charset='utf-8'><title>R1 plate tracker</title></head>"
            b"<body style='margin:0;background:#111;color:#ccc;font:13px sans-serif'>"
            b"<img src='/stream.mjpg' style='width:100%;max-width:1400px;display:block;margin:auto'>"
            b"<p style='text-align:center'>left: the camera; cyan = rim-top depth points, dots on the rim's outer edge "
            b"(green used, red rejected), green circle = the tracked rim, grey = where it was located, yellow = the robot's "
            b"own arm and hand (cut out), magenta = rim sectors left out because the hand is there, blue = where it looks. "
            b"right: the same from above, robot at the bottom. Below: the LEFT and RIGHT wrist cameras, raw "
            b"(opened only while this page is open; 'busy' when another program, e.g. wristcam.py record, holds them).</p>"
            b"</body></html>")

    def __init__(self, host="127.0.0.1", port=8090, fps=5.0, wrist=True):
        import http.server
        import threading
        self.jpg, self.seq, self.watchers, self.period, self.t_last = None, 0, 0, 1.0 / fps, 0.0
        self.wrist = WristFeed(lambda: self.watchers > 0) if wrist else None
        self.cv = threading.Condition()
        view = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if not self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(view.PAGE)))
                    self.end_headers()
                    self.wfile.write(view.PAGE)
                    return
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with view.cv:
                    view.watchers += 1
                try:
                    seq = -1
                    while True:
                        with view.cv:
                            view.cv.wait_for(lambda: view.seq != seq, timeout=5.0)
                            jpg, seq = view.jpg, view.seq
                        if jpg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg + b"\r\n")
                except OSError:                               # the browser went away
                    pass
                finally:
                    with view.cv:
                        view.watchers -= 1

        self.srv = http.server.ThreadingHTTPServer((host, port), Handler)
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def wanted(self, now):
        return self.watchers > 0 and now - self.t_last >= self.period

    def put(self, img, now):
        import cv2
        self.t_last = now
        if self.wrist is not None:
            img = self.wrist.below(img)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.cv:
                self.jpg, self.seq = buf.tobytes(), self.seq + 1
                self.cv.notify_all()


class WristFeed(object):
    """The two wrist cameras for the live view. A thread opens them (wristcam.py's Grabber, devices from
    params/wrist_cam.json) only while someone watches the page, and releases them 5 s after the last viewer leaves, so
    wristcam.py check / record can use them otherwise. Read-only: it only reads frames."""

    def __init__(self, watched):
        import threading
        self.watched, self.grab, self.err, self.lock = watched, {}, {}, threading.Lock()
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        t_seen, t_try = 0.0, 0.0
        while True:
            now = time.time()
            if self.watched():
                t_seen = now
                # a side that is missing, or whose frames stopped (camera replugged), is (re)opened every 5 s
                if now - t_try > 5.0 and any(self.stale(s_, now) for s_ in ("left", "right")):
                    t_try = now
                    self.open([s_ for s_ in ("left", "right") if self.stale(s_, now)])
            elif self.grab and now - t_seen > 5.0:
                self.close()
            time.sleep(0.5)

    def stale(self, side, now):
        with self.lock:
            g = self.grab.get(side)
        return g is None or now - g.latest()[1] > 3.0

    def open(self, sides):
        try:
            import wristcam as tool                                  # pick/wristcam.py
            from pk import wristcam as wc
            cams = wc.load()
        except Exception as ex:                                      # noqa: BLE001
            with self.lock:
                self.err.update(left=str(ex)[:60], right=str(ex)[:60])
            return
        for side in sides:
            with self.lock:
                old = self.grab.pop(side, None)
            if old is not None:
                old.close()
            try:
                g = tool.Grabber(cams[side])
                with self.lock:
                    self.grab[side] = g
                    self.err.pop(side, None)
            except Exception as ex:                                  # noqa: BLE001
                with self.lock:
                    self.err[side] = "busy or missing: %s" % os.path.basename(str(cams[side].device))

    def close(self):
        with self.lock:
            g, self.grab = self.grab, {}
        for x in g.values():
            x.close()

    def below(self, img):
        """img (the head view) with the left and right wrist frames side by side under it, scaled to its width."""
        import cv2
        W = img.shape[1]
        w = W // 2
        h = w * 3 // 4
        tiles = []
        with self.lock:
            g, err = dict(self.grab), dict(self.err)
        if not g and not err and not self.watched():
            err = {}
        for side in ("left", "right"):
            fr, t = g[side].latest() if side in g else (None, 0.0)
            if fr is None:
                tile = np.full((h, w, 3), 30, np.uint8)
                msg = err.get(side) or ("opening" if self.watched() else "off")
                cv2.putText(tile, "%s wrist: %s" % (side.upper(), msg), (12, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (170, 170, 170), 1, cv2.LINE_AA)
            else:
                tile = cv2.resize(fr, (w, h), interpolation=cv2.INTER_AREA)
                age = time.time() - t
                cv2.putText(tile, "%s wrist  %s" % (side.upper(), "" if age < 1.0 else "STALE %.0f s" % age), (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1, cv2.LINE_AA)
            tiles.append(tile)
        row = np.hstack(tiles)
        if row.shape[1] < W:
            row = np.hstack([row, np.zeros((h, W - row.shape[1], 3), np.uint8)])
        return np.vstack([img, row])


def draw_view(colour, intr, T_po, trk, est, status, fps, head_lock, spheres, size=(480, 848), pub=None):
    """The camera picture with the tracker's points, beside a top view (pelvis frame, robot at the bottom)."""
    import cv2
    img = colour.copy() if colour is not None else np.zeros(size + (3,), np.uint8)
    H, W = img.shape[:2]
    dbg = (getattr(trk, "dbg", None) or {}) if trk is not None else {}
    ok = bool(est and est.get("ok"))
    GREEN, RED, CYAN, GREY, YELLOW, BLUE, ORANGE = (60, 220, 60), (60, 60, 230), (230, 220, 40), (170, 170, 170), (0, 220, 255), (200, 130, 40), (0, 150, 255)
    MAGENTA = (220, 60, 220)

    def proj(P):                                              # pelvis points -> pixels, and which lie in front
        P_o = (np.asarray(P, float).reshape(-1, 3) - T_po[:3, 3]) @ T_po[:3, :3]
        return project(P_o, intr), P_o[:, 2] > 0.05

    def ring(c, r, z, n=120):
        a = np.linspace(0, 2 * math.pi, n)
        return np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a), np.full(n, z)], axis=1)

    def poly(P, col, th=1):
        uv, front = proj(P)
        if front.all():
            cv2.polylines(img, [uv.astype(np.int32)], True, col, th, cv2.LINE_AA)
    if trk is not None and T_po is not None:
        poly(ring(trk.c, trk.R - 0.05, trk.rim_z), BLUE)                       # the annulus it searches
        poly(ring(trk.c, trk.R + trk.band_r, trk.rim_z), BLUE)
        for c, r in spheres or ():                                            # the robot's own arm and hand
            uv, front = proj(c)
            if front[0]:
                z = ((np.asarray(c, float) - T_po[:3, 3]) @ T_po[:3, :3])[2]
                cv2.circle(img, (int(uv[0, 0]), int(uv[0, 1])), max(2, int(intr["fx"] * r / z)), YELLOW, 1, cv2.LINE_AA)
        Q = dbg.get("Q")
        if Q is not None and len(Q):
            uv, front = proj(Q[::max(1, len(Q) // 2500)])
            uv = uv[front].astype(int)
            uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < W - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < H - 1)]
            for du in (0, 1):
                img[uv[:, 1], uv[:, 0] + du] = CYAN
        poly(ring(trk.start, trk.r_band, trk.rim_z), GREY)
        E, inl = dbg.get("E"), dbg.get("inl")
        if E is not None and inl is not None and len(E) == len(inl):
            uv, front = proj(np.c_[E, np.full(len(E), trk.rim_z)])
            for (u, v), f, good in zip(uv, front, inl):
                if f:
                    cv2.circle(img, (int(u), int(v)), 3, GREEN if good else RED, -1, cv2.LINE_AA)
        for th, half in dbg.get("excl") or ():                                # sectors left out: the hand is there
            a_ = np.linspace(th - half, th + half, 16)
            arc = np.stack([trk.c[0] + trk.R * np.cos(a_), trk.c[1] + trk.R * np.sin(a_), np.full(16, trk.rim_z)], axis=1)
            uv, front = proj(arc)
            if front.all():
                cv2.polylines(img, [uv.astype(np.int32)], False, MAGENTA, 4, cv2.LINE_AA)
        fit_c = dbg.get("fit_c", trk.c)
        poly(ring(fit_c, trk.r_band, trk.rim_z), GREEN if ok else ORANGE, 2)
        ends, front = proj(np.array([[trk.start[0], trk.start[1], trk.rim_z], [fit_c[0], fit_c[1], trk.rim_z]]))
        if front.all():
            a, b = ends.astype(int)
            cv2.drawMarker(img, (int(a[0]), int(a[1])), GREY, cv2.MARKER_CROSS, 14, 1)
            cv2.arrowedLine(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), GREEN if ok else ORANGE, 2, cv2.LINE_AA, tipLength=0.3)
    # top view: robot at the bottom, its left on the left, centred where the plate was located
    tv = np.full((H, H, 3), 28, np.uint8)
    if trk is not None:
        s_ = (H - 60) / 0.36
        x0, y0 = trk.start

        def tp(xy):
            xy = np.asarray(xy, float).reshape(-1, 2)
            return np.stack([H / 2 - (xy[:, 1] - y0) * s_, H / 2 - (xy[:, 0] - x0) * s_], axis=1).astype(np.int32)
        a = np.linspace(0, 2 * math.pi, 120)
        for c, r, col, th in ((trk.start, trk.r_band, GREY, 1), (dbg.get("fit_c", trk.c), trk.r_band, GREEN if ok else ORANGE, 2)):
            cv2.polylines(tv, [tp(np.stack([c[0] + r * np.cos(a), c[1] + r * np.sin(a)], axis=1))], True, col, th, cv2.LINE_AA)
        Q = dbg.get("Q")
        if Q is not None and len(Q):
            uv = tp(Q[::max(1, len(Q) // 3000), :2])
            uv = uv[(uv[:, 0] >= 0) & (uv[:, 0] < H) & (uv[:, 1] >= 0) & (uv[:, 1] < H)]
            tv[uv[:, 1], uv[:, 0]] = CYAN
        E, inl = dbg.get("E"), dbg.get("inl")
        if E is not None and inl is not None and len(E) == len(inl):
            for (u, v), good in zip(tp(E), inl):
                cv2.circle(tv, (int(u), int(v)), 3, GREEN if good else RED, -1, cv2.LINE_AA)
        for th, half in dbg.get("excl") or ():
            a_ = np.linspace(th - half, th + half, 16)
            cv2.polylines(tv, [tp(np.stack([trk.c[0] + trk.R * np.cos(a_), trk.c[1] + trk.R * np.sin(a_)], axis=1))], False, MAGENTA, 4, cv2.LINE_AA)
        p0, p1 = tp(trk.start)[0], tp(dbg.get("fit_c", trk.c))[0]
        cv2.arrowedLine(tv, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), GREEN if ok else ORANGE, 2, cv2.LINE_AA, tipLength=0.3)
        L = int(0.1 * s_)
        cv2.line(tv, (20, H - 20), (20 + L, H - 20), GREY, 2)
        cv2.putText(tv, "10 cm", (24, H - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREY, 1, cv2.LINE_AA)
        cv2.putText(tv, "top view: forward = up, robot's left = left", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREY, 1, cv2.LINE_AA)
    # text
    mv = (est or {}).get("move_xy_mm") or [0.0, 0.0]
    lines = ["%s  state: %s   head: %s" % (time.strftime("%H:%M:%S"), status, "LOCKED by the test (does not follow)" if head_lock else "free (follows the plate)"),
             ("TRACKING  the top plate moved %.1f mm since it was located  (x %+.1f forward, y %+.1f left)" % ((est or {}).get("moved_mm", 0.0), mv[0], mv[1]))
             if ok else ("NOT TRACKING: %s" % ((est or {}).get("why") or status))]
    if pub and pub.get("center"):
        c = pub["center"]
        loc = pub.get("located") or {}
        lines.append("top plate, robot frame (pelvis): x %.3f m forward, y %+.3f m left, rim top z %+.3f m" % (c[0], c[1], c[2]))
        if loc:
            lines.append("stack (from the locate): %s plates, top rim %.1f mm above the table (now %.1f mm), diameter %.0f mm" % (
                loc.get("n_plates", "?"), 1000 * float(loc.get("top_height") or 0.0), 1000 * float(pub.get("top_height") or 0.0),
                1000 * float(loc.get("diameter") or 0.0)))
    if est and est.get("n") is not None:
        lines.append("fit quality: rim edge found in %s of 72 sectors (coverage %s), rms %s mm, %d depth points on the rim, %.0f fps" % (
            est.get("n"), est.get("coverage"), est.get("rms_mm"), est.get("points") or 0, fps))
    pl = (est or {}).get("plane")
    if pl:
        a = math.atan2(pl["gy"], pl["gx"])
        lines.append("rim plane: tilt %.1f deg, highest rim toward %+.0f deg (-90 = robot's right), %d sectors, rms %.2f mm" % (
            math.degrees(math.atan(math.hypot(pl["gx"], pl["gy"]))), math.degrees(a), pl["n_sec"], pl["rms_mm"]))
    # the text on its own dark band above the pictures
    row = np.hstack([img, tv])
    band = np.full((14 + 26 * len(lines), row.shape[1], 3), 18, np.uint8)
    for k, ln in enumerate(lines):
        cv2.putText(band, ln, (10, 26 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255) if k else (170, 230, 255), 1, cv2.LINE_AA)
    return np.vstack([band, row])


def snapshot(colour, intr, T_po, center, r, path, label):
    """The camera's picture with the tracked rim drawn on it (green)."""
    if colour is None:
        return
    try:
        import cv2
    except ImportError:
        return
    img = colour.copy()
    ang = np.linspace(0, 2 * math.pi, 120)
    P = np.stack([center[0] + r * np.cos(ang), center[1] + r * np.sin(ang), np.full(120, center[2])], axis=1)
    P_o = (P - T_po[:3, 3]) @ T_po[:3, :3]
    if np.all(P_o[:, 2] > 0.05):
        uv = project(P_o, intr).astype(np.int32)
        cv2.polylines(img, [uv], True, (60, 220, 60), 2, cv2.LINE_AA)
        cu = project(((np.array(center) - T_po[:3, 3]) @ T_po[:3, :3])[None, :], intr)[0].astype(int)
        cv2.drawMarker(img, (int(cu[0]), int(cu[1])), (60, 220, 60), cv2.MARKER_CROSS, 16, 2)
    txt = "%s  %s  centre (%.3f %+.3f %.3f)" % (time.strftime("%H:%M:%S"), label, center[0], center[1], center[2])
    cv2.putText(img, txt, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, txt, (8, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
