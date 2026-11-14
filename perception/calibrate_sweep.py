"""Calibrate the camera over many head poses: a level surface, seen from everywhere the head
will look, against the robot's own sense of level.

ON THE ROBOT (the head moves; nothing else does):
    python3 calibrate_sweep.py capture eth10 --surface table --table-height 74.5 --go
    python3 calibrate_sweep.py capture eth10 --surface floor --go

ANYWHERE (laptop or robot):
    python3 calibrate_sweep.py check sweeps/<folder>             # how the SAVED calibration does
    python3 calibrate_sweep.py fit sweeps/<folder> [...]         # fit, test on held-out poses
    python3 calibrate_sweep.py fit sweeps/<folder> [...] --save  # ...and use it from now on

WHY (from the manipulation side's measurements, 21-22 Oct)
    - the one-pose calibration is right at head yaw 0, where it was taken, but the table's
      height came out off by -27.6 mm x sin(yaw): -8 mm at the 17 deg the push looks from;
    - the pelvis leans ~3 deg forward standing, and the one-pose calibration assumed it level,
      so put the lean into the camera;
    - the head's yaw has play (now always taken up the same way: r1_head.py).

WHAT IS FITTED (camera_calibration.py applies it, everywhere a camera pose is used)
    mount     3 rotations + 3 position offsets of the camera on head_yaw_link. The position
              is held near the CAD by a 1 cm prior: a level plane barely sees it.
    neck      2 rotations at the yaw joint: the yaw axis tipped (or a head-pitch zero).
              At one pose a level surface cannot tell this from the mount; turning the head
              separates them, and an error here grows with sin(yaw) -- what was seen.
    surface   per session, how far the surface is below the pelvis.
Against: each pose's plane (its normal: 2 numbers, its distance: 1), with the floor's up in
pelvis_link from the IMU at that moment (cross-checked against the legs).

WHAT IT CANNOT SEE
    A level surface looks the same however the camera is slid sideways or turned about the
    vertical. Its heading still comes out, because the yaw axis tips with the head's pitch
    (a sweep over pitch sees it; one pose does not) -- but its sideways position barely, so
    that stays near the CAD. Check it with a tape: a stack on the robot's centreline reads
    y ~ 0 (it did, to 1 cm, on 19 Oct). A waist-yaw zero is invisible too, and harmless:
    the arms turn with it.
    And it fits the camera to the IMU's idea of level: an IMU that is itself a degree off
    leaves the camera a degree off. The legs are the check on that; read the warnings.
"""
import argparse
import datetime
import glob
import json
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import camera_calibration as cc  # noqa: E402
import floor_frame as ff  # noqa: E402
import r1_config as cfg  # noqa: E402
import urdf_fk as fk  # noqa: E402

# head poses, degrees (pitch + is down, yaw + is left). Rows of yaw run right to left, so
# each yaw move already ends travelling +; only the jump back between rows needs the extra
# approach move r1_head adds. Pitch stops 2 deg inside the head's 36 deg limit.
GRIDS = {"table": ([15.0, 25.0, 33.0], [-24.0, -12.0, 0.0, 12.0, 24.0]),
         "floor": ([0.0, 12.0, 24.0], [-24.0, -12.0, 0.0, 12.0, 24.0])}
FRAMES_PER_POSE = 3
WARMUP = 15
SETTLE_S = 0.6
MAX_RANGE_M = {"table": 1.5, "floor": 3.5}
MIN_FRACTION = 0.06          # a plane must cover this much of the picture to count
FRAMES_AGREE_DEG = 0.5       # a pose whose frames disagree more than this is left out

SIG_ANG = math.radians(0.1)  # a plane's normal, per pose
SIG_H = 0.0015               # its distance, m
SIG_POS = 0.01               # prior: the camera's position stays within ~1 cm of the CAD
SIG_ROT = math.radians(10)   # prior, weak: only keeps a thin sweep well posed
WORK_RANGE_M = 0.6           # an angle error is costed at this range
WORK_POINT = (0.36, 0.0)     # pelvis x, y where the hand works (the 19 Oct stack): the
                             # surface's height is reported HERE, as a hand touching it would
HOLDOUT = 4                  # every 4th pose is left out of the fit, to test it
D = math.degrees


# ==========================================================================
# planes in a depth frame
# ==========================================================================
def level_planes(depth, intr, scale, expect_up, max_range, k=2, max_dev_deg=25.0,
                 tol=0.012, iters=400, seed=0):
    """Up to k level planes in one depth frame, biggest first: [(up, c, fraction)] in the
    optical frame, n.p = c on the plane (c < 0: the camera is above it)."""
    import check_mount as cm
    pts = cm._points(depth, intr, scale, 4, max_range)
    total = max(len(pts), 1)
    e = np.asarray(expect_up, float) / np.linalg.norm(expect_up)
    cos_lim = math.cos(math.radians(max_dev_deg))
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(k):
        if len(pts) < 200:
            break
        best = (0, None, None)
        for _ in range(iters):
            p0, p1, p2 = pts[rng.choice(len(pts), 3, replace=False)]
            n = np.cross(p1 - p0, p2 - p0)
            ln = np.linalg.norm(n)
            if ln < 1e-9:
                continue
            n = n / ln
            if n @ e < 0:
                n = -n
            if n @ e < cos_lim:
                continue
            c = float(n @ p0)
            cnt = int((np.abs(pts @ n - c) < tol).sum())
            if cnt > best[0]:
                best = (cnt, n, c)
        if best[1] is None or best[0] < MIN_FRACTION * total:
            break
        n, c = best[1], best[2]
        for band in (tol, 0.6 * tol, 0.4 * tol):        # tighten, as check_mount does
            keep = np.abs(pts @ n - c) < max(band, 0.004)
            if keep.sum() < 50:
                break
            q = pts[keep]
            m = q.mean(axis=0)
            n = np.linalg.svd(q - m, full_matrices=False)[2][-1]
            n = -n if n @ e < 0 else n
            c = float(n @ m)
        near = np.abs(pts @ n - c) < tol
        out.append((n, c, float(near.sum()) / total))
        pts = pts[~near]
    return out


def pick_plane(planes, surface):
    """The table is the nearest level plane (it is above the floor); the floor the lowest."""
    if not planes:
        return None
    return max(planes, key=lambda p: p[1]) if surface == "table" else \
        min(planes, key=lambda p: p[1])


def observe(depths, intr, scale, expect_up, surface):
    """One pose's plane from its frames: (up, c, spread_deg, n_frames), or None."""
    ns, cs = [], []
    for d in depths:
        p = pick_plane(level_planes(d, intr, scale, expect_up, MAX_RANGE_M[surface]), surface)
        if p is not None:
            ns.append(p[0])
            cs.append(p[1])
    if not ns:
        return None
    n = np.mean(ns, axis=0)
    n /= np.linalg.norm(n)
    spread = max(ff.angle_deg(m, n) for m in ns)
    return n, float(np.median(cs)), spread, len(ns)


# ==========================================================================
# capture (robot)
# ==========================================================================
class HeadSequence:
    """r1_head.py --sequence in a process of its own (see r1_head.py for why)."""

    def __init__(self, interface, timeout=40.0):
        cmd = [sys.executable, "-u", os.path.join(HERE, "r1_head.py"), interface, "--go",
               "--sequence"]
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT, text=True, bufsize=1)
        self.lines = queue.Queue()
        threading.Thread(target=self._pump, daemon=True).start()
        self._expect("ready", timeout)

    def _pump(self):
        for line in self.p.stdout:
            print("[head] " + line.rstrip(), flush=True)
            self.lines.put(line)
        self.lines.put(None)

    def _expect(self, prefix, timeout):
        t0 = time.time()
        while True:
            try:
                line = self.lines.get(timeout=0.5)
            except queue.Empty:
                if time.time() - t0 > timeout:
                    raise RuntimeError("the head did not answer in %.0f s" % timeout)
                continue
            if line is None:
                raise RuntimeError("the head process stopped (see [head] above)")
            if line.startswith(prefix):
                return line
            if line.startswith("error"):
                raise RuntimeError(line.strip())

    def go(self, pitch_deg, yaw_deg, timeout=40.0):
        self.p.stdin.write("%.3f %.3f\n" % (pitch_deg, yaw_deg))
        self.p.stdin.flush()
        return self._expect("there:", timeout)

    def close(self):
        if self.p.poll() is None:
            try:
                self.p.stdin.write("done\n")
                self.p.stdin.flush()
            except (BrokenPipeError, OSError, ValueError):
                pass
        try:
            self.p.wait(timeout=20)
        except Exception:
            self.p.terminate()


def _mean_state(a, b):
    out = {"joints": {n: 0.5 * (a["joints"][n] + b["joints"][n]) for n in a["joints"]}}
    ra, rb = a.get("imu_rpy"), b.get("imu_rpy")
    out["imu_rpy"] = [0.5 * (x + y) for x, y in zip(ra, rb)] if ra and rb else (ra or rb)
    return out


def _floats(s):
    return [float(v) for v in s.split(",")] if s else None


def cmd_capture(args):
    import r1_head
    import realsense_capture

    pitches, yaws = GRIDS[args.surface]
    pitches, yaws = _floats(args.pitches) or pitches, sorted(_floats(args.yaws) or yaws)
    poses = [(p, y) for p in pitches for y in yaws]
    out = args.out or os.path.join(HERE, "sweeps",
                                   time.strftime("%Y%m%d_%H%M_") + args.surface)
    link = r1_head.R1Link(args.interface).wait()
    up, info = ff.pelvis_up(link.state())
    print(ff.describe(info))
    print("\nsweep: %d head poses (pitch %s, yaw %s deg), %d frames each, of the %s"
          % (len(poses), pitches, yaws, args.frames, args.surface.upper()))
    print("  keep the robot still -- feet unmoved -- and people out of view."
          + ("\n  The table must be CLEAR (plates off): the nearest level plane is taken as it."
             if args.surface == "table" else
             "\n  Face open floor: nothing level within 3.5 m but the floor."))
    if args.surface == "table" and args.table_height is None:
        print("  --table-height CM (tape, floor to table top) lets it also set the floor height")
    if not args.go:
        print("\nnothing moved -- add --go")
        return 0
    os.makedirs(out, exist_ok=False)
    head = HeadSequence(args.interface)
    saved = 0
    try:
        for i, (p, y) in enumerate(poses):
            head.go(p, y)
            time.sleep(SETTLE_S)
            for _ in range(2):
                a = link.state()
                color, depths, intr, scale = realsense_capture.capture_frames(
                    args.frames, WARMUP)
                b = link.state()
                moved = max(abs(b["joints"][n] - a["joints"][n]) for n in cfg.CAMERA_CHAIN_JOINTS)
                if moved < math.radians(0.2):
                    break
                print("  the head moved %.2f deg during the frames -- again" % D(moved))
            st = _mean_state(a, b)
            extra = {} if color is None else {"color": color}
            np.savez_compressed(os.path.join(out, "pose_%02d.npz" % i), depth=np.stack(depths),
                                intr=json.dumps(intr), depth_scale=scale, state=json.dumps(st),
                                asked=json.dumps([p, y]), **extra)
            saved += 1
            print("  pose %2d/%d  asked pitch %+5.1f yaw %+5.1f -> at pitch %+6.2f yaw %+6.2f"
                  % (i + 1, len(poses), p, y, D(st["joints"]["head_pitch_joint"]),
                     D(st["joints"]["head_yaw_joint"])), flush=True)
    finally:
        head.close()
        meta = {"surface": args.surface, "date": datetime.datetime.now().isoformat(
            timespec="seconds"), "poses": saved, "frames_per_pose": args.frames,
            "table_height_m": None if args.table_height is None else args.table_height / 100,
            "lean_at_start": {k: v for k, v in info.items() if k != "warning"},
            "head_yaw_approach_deg": r1_head.YAW_APPROACH_DEG}
        if saved:
            with open(os.path.join(out, "sweep.json"), "w") as f:
                json.dump(meta, f, indent=2)
    print("\nsaved %d poses in %s\nnext:  python3 calibrate_sweep.py fit %s" % (saved, out, out))
    return 0


# ==========================================================================
# the fit
# ==========================================================================
def load_session(folder):
    path = os.path.join(folder, "sweep.json")
    meta = json.load(open(path)) if os.path.exists(path) else {"surface": "table"}
    poses = []
    for p in sorted(glob.glob(os.path.join(folder, "pose_*.npz"))):
        z = np.load(p, allow_pickle=False)
        d = z["depth"]
        poses.append({"name": "%s/%s" % (os.path.basename(os.path.normpath(folder)),
                                         os.path.basename(p)),
                      "depth": d if d.ndim == 3 else d[None],
                      "intr": json.loads(str(z["intr"])), "scale": float(z["depth_scale"]),
                      "state": json.loads(str(z["state"])),
                      "asked": json.loads(str(z["asked"])) if "asked" in z else None})
    if not poses:
        raise SystemExit("no pose_*.npz in %s" % folder)
    return meta, poses


def observations(sessions, J, lean, guess):
    """One level-plane observation per usable pose."""
    obs = []
    for s, (meta, poses) in enumerate(sessions):
        for p in poses:
            ang = p["state"]["joints"]
            up, info = ff.pelvis_up(p["state"], J, lean)
            T = cc.T_pelvis_optical(guess, J, ang) if guess else \
                fk.T_pelvis_optical(J, ang, allow_nominal=True)
            o = observe(p["depth"], p["intr"], p["scale"], T[:3, :3].T @ up, meta["surface"])
            if o is None:
                print("  %s: no level %s found -- left out" % (p["name"], meta["surface"]))
                continue
            if o[2] > FRAMES_AGREE_DEG:
                print("  %s: its frames disagree by %.2f deg -- left out" % (p["name"], o[2]))
                continue
            obs.append({"session": s, "surface": meta["surface"], "angles": ang, "up": up,
                        "lean": info, "n": o[0], "c": o[1], "spread": o[2], "name": p["name"],
                        "imu_rpy": p["state"].get("imu_rpy"),
                        "pitch": D(ang["head_pitch_joint"]), "yaw": D(ang["head_yaw_joint"])})
    return obs


class Model:
    """The camera for parameters x = [mount rotation (3), neck (2), mount shift (3),
    surface height per session (S)]. Mount rotation and shift are relative to `base`."""

    def __init__(self, J, base, n_sessions, obs):
        self.J, self.S = J, n_sessions
        M0 = np.asarray(base["T_headyaw_optical"], float) if base else \
            fk.T(J, "head_yaw_link", cfg.CAMERA_OPTICAL_LINK, {})
        self.R0, self.t0 = M0[:3, :3], M0[:3, 3]
        j = J["head_yaw_joint"]
        self.yaw_rpy, self.yaw_xyz, self.yaw_axis = fk.rpy_to_matrix(j["rpy"]), j["xyz"], j["axis"]
        self.obs = obs
        self.T_hp = [fk.T(J, cfg.ROOT_LINK, "head_pitch_link", o["angles"]) for o in obs]
        self.R_yaw = [fk.axis_angle_matrix(self.yaw_axis, o["angles"].get("head_yaw_joint", 0.0))
                      for o in obs]

    def mount(self, x):
        return fk.homogeneous(cc.rot(x[0:3]) @ self.R0, self.t0 + x[5:8])

    def T(self, x, k, M=None):
        N = fk.homogeneous(self.yaw_rpy @ cc.rot([x[3], x[4], 0.0]) @ self.R_yaw[k], self.yaw_xyz)
        return self.T_hp[k] @ N @ (self.mount(x) if M is None else M)

    def x0(self, base):
        x = np.zeros(8 + self.S)
        if base and base.get("neck_tilt_rad"):
            x[3:5] = base["neck_tilt_rad"]
        for s in range(self.S):
            hs = [-o["c"] - o["up"] @ self.T(x, k)[:3, 3]
                  for k, o in enumerate(self.obs) if o["session"] == s]
            x[8 + s] = float(np.median(hs)) if hs else 0.5
        return x

    def residuals(self, x, idx):
        M = self.mount(x)
        r = []
        for k in idx:
            o = self.obs[k]
            n, c = cc.predict_plane(self.T(x, k, M), o["up"], x[8 + o["session"]])
            r.extend((o["n"] - n) / SIG_ANG)
            r.append((o["c"] - c) / SIG_H)
        r.extend(x[0:3] / SIG_ROT)
        r.extend(x[3:5] / SIG_ROT)
        r.extend(x[5:8] / SIG_POS)
        return np.asarray(r)

    def solve(self, idx, x0):
        import r1_ik
        lsq = r1_ik._least_squares()
        inf = np.full(len(x0), np.inf)
        sol = lsq(lambda x: self.residuals(x, idx), x0, bounds=(-inf, inf), max_nfev=2000)
        return np.asarray(sol.x, float)

    def std(self, x, idx, h=1e-6):
        """1-sigma of each parameter, from the fit's own scatter."""
        r0 = self.residuals(x, idx)
        Jm = np.empty((len(r0), len(x)))
        for i in range(len(x)):
            xh = x.copy()
            xh[i] += h
            Jm[:, i] = (self.residuals(xh, idx) - r0) / h
        dof = max(len(r0) - len(x), 1)
        chi2 = float(r0 @ r0) / dof
        cov = np.linalg.pinv(Jm.T @ Jm) * max(chi2, 1.0)
        return np.sqrt(np.clip(np.diag(cov), 0, None))


def surface_at(T_po, o, xy=WORK_POINT):
    """The measured surface's height (along up; the pelvis origin is 0) straight above or
    below the work point, through this camera pose -- what a hand touching it would check.
    (Not the camera's distance to the plane: a camera tipped about its own centre keeps
    that distance, while the surface under the hand moves by the tip x the reach.)"""
    u = T_po[:3, :3] @ o["n"]
    up = o["up"]
    w = np.array([xy[0], xy[1], 0.0])
    s = (u @ T_po[:3, 3] + o["c"] - u @ w) / (u @ up)
    return float(up @ w + s)


def errors(T_po, o, height):
    """How one pose's plane disagrees with a camera pose: (angle deg, surface height at the
    work point mm, front-back deg, left-right deg, at the plate mm)."""
    n, c = cc.predict_plane(T_po, o["up"], height)
    ang = ff.angle_deg(n, o["n"])
    dh = 1000.0 * (surface_at(T_po, o) + height)     # + : the surface is measured higher
    u = T_po[:3, :3] @ o["n"]                         # the surface's up, by this camera
    up = o["up"]
    fb = D(math.atan2(u[0], u[2]) - math.atan2(up[0], up[2]))
    lr = D(math.atan2(u[1], u[2]) - math.atan2(up[1], up[2]))
    return ang, dh, fb, lr, math.hypot(1000 * WORK_RANGE_M * math.radians(ang), dh)


def evaluate(cal, obs, J, label, idx=None):
    """A saved calibration on these poses, each session's surface height at its best."""
    idx = list(range(len(obs))) if idx is None else idx
    Ts = {k: cc.T_pelvis_optical(cal, J, obs[k]["angles"]) for k in idx}
    heights = {}
    for s in {obs[k]["session"] for k in idx}:
        heights[s] = -float(np.mean([surface_at(Ts[k], obs[k])
                                     for k in idx if obs[k]["session"] == s]))
    rows = [(k,) + errors(Ts[k], obs[k], heights[obs[k]["session"]]) for k in idx]
    _print_rows(label, rows, obs)
    return rows


def _print_rows(label, rows, obs, held=()):
    print("\n%s" % label)
    print("  pose                       head pitch  yaw   tipped f-b  l-r   surface at   at plate\n"
          "                                                               %2.0f cm fwd"
          % (100 * WORK_POINT[0]))
    for k, ang, dh, fb, lr, mm in rows:
        o = obs[k]
        print("  %-26s %+6.1f %+6.1f   %+5.2f %+5.2f deg  %+6.1f mm  %5.1f mm%s"
              % (o["name"][-26:], o["pitch"], o["yaw"], fb, lr, dh, mm,
                 "   (held out)" if k in held else ""))
    mm = np.array([r[5] for r in rows])
    dh = np.array([r[2] for r in rows])
    s = np.sin(np.radians([obs[r[0]]["yaw"] for r in rows]))
    line = "  rms %.1f mm at the plate, worst %.1f mm" % (math.sqrt(np.mean(mm ** 2)), mm.max())
    if np.ptp(s) > 0.2:
        b, a = np.polyfit(s, dh, 1)
        line += ";  surface height %+.1f %+.1f mm x sin(yaw)" % (a, b)
    print(line)
    return mm


def _rms(rows, keep=None):
    mm = np.array([r[5] for r in rows if keep is None or r[0] in keep])
    return float(math.sqrt(np.mean(mm ** 2))) if len(mm) else float("nan")


def fit(obs, J, base, n_sessions, holdout=HOLDOUT, log=print):
    """Fit; test on held-out poses first. -> (x, model, report dict)."""
    model = Model(J, base, n_sessions, obs)
    every = list(range(len(obs)))
    held = [k for k in every if holdout and len(obs) >= 2 * holdout and k % holdout == holdout - 1]
    rep = {"poses": len(obs), "held_out": [obs[k]["name"] for k in held]}
    if held:
        train = [k for k in every if k not in held]
        xt = model.solve(train, model.x0(base))
        rows = [(k,) + errors(model.T(xt, k), obs[k], xt[8 + obs[k]["session"]]) for k in every]
        _print_rows("FITTED WITHOUT the held-out poses, tested on all:", rows, obs, held)
        rep["held_out_rms_mm"] = _rms(rows, held)
        rep["train_rms_mm"] = _rms(rows, train)
        log("  held-out poses: rms %.1f mm (the fitted ones: %.1f mm)"
            % (rep["held_out_rms_mm"], rep["train_rms_mm"]))
    x = model.solve(every, model.x0(base))
    rows = [(k,) + errors(model.T(x, k), obs[k], x[8 + obs[k]["session"]]) for k in every]
    _print_rows("FITTED ON ALL POSES:", rows, obs)
    rep["rms_mm"] = _rms(rows)
    sd = model.std(x, every)
    rep["params"] = {"mount_rotation_deg": D(1) * x[0:3], "mount_rotation_sd_deg": D(1) * sd[0:3],
                     "neck_tilt_deg": D(1) * x[3:5], "neck_tilt_sd_deg": D(1) * sd[3:5],
                     "mount_shift_mm": 1000 * x[5:8], "mount_shift_sd_mm": 1000 * sd[5:8],
                     "surface_below_pelvis_m": x[8:], "surface_sd_mm": 1000 * sd[8:]}
    rep["params"] = {k: np.asarray(v).round(4).tolist() for k, v in rep["params"].items()}
    return x, model, rep


def print_params(p):
    print("\nTHE FIT")
    print("  mount rotation   %s deg  (+- %s)   about head x, y, z"
          % (np.round(p["mount_rotation_deg"], 2).tolist(), np.round(p["mount_rotation_sd_deg"], 2).tolist()))
    print("  yaw axis tipped  %s deg  (+- %s)   about head_pitch x, y (y = a head-pitch zero)"
          % (np.round(p["neck_tilt_deg"], 2).tolist(), np.round(p["neck_tilt_sd_deg"], 2).tolist()))
    print("  camera shifted   %s mm   (+- %s)   from the CAD"
          % (np.round(p["mount_shift_mm"], 1).tolist(), np.round(p["mount_shift_sd_mm"], 1).tolist()))
    loose = [n for n, s in zip(("x", "y", "z"), p["mount_shift_sd_mm"]) if s > 7.0]
    if loose:
        print("  (shift along %s: barely seen by a level surface -- held near the CAD)"
              % ", ".join(loose))


def _yaw_approach_deg():
    import r1_head
    return r1_head.YAW_APPROACH_DEG


def calibration(x, model, obs, sessions, base, rep, lean, folders):
    """The camera_calibration.json for this fit."""
    J = model.J
    M = model.mount(x)
    ref = min(range(len(obs)), key=lambda k: (abs(obs[k]["yaw"]), abs(obs[k]["pitch"] - 25)))
    o = obs[ref]
    T = model.T(x, ref)
    t_meas, r_meas = cc._tilt_roll(T[:3, :3].T @ o["up"])
    Tn = fk.T_pelvis_optical(J, o["angles"], allow_nominal=True)
    t_cad, r_cad = cc._tilt_roll(Tn[:3, :3].T @ o["up"])
    cal = {
        "method": "head sweep: mount (6) and yaw-axis tilt (2) fitted to level %s over %d "
                  "head poses; pelvis lean from the %s"
                  % ("/".join(sorted({m["surface"] for m, _ in sessions})), len(obs), lean),
        "version": 2,
        "source": "calibrate_sweep.py fit " + " ".join(folders),
        "date": datetime.date.today().isoformat(),
        "angles_at_calibration": {n: float(o["angles"][n]) for n in cfg.CAMERA_CHAIN_JOINTS},
        "measured_tilt_deg": t_meas, "measured_roll_deg": r_meas,
        "cad_tilt_deg": t_cad, "correction_tilt_deg": t_meas - t_cad,
        "correction_roll_deg": r_meas - r_cad,
        "T_headyaw_optical": M.tolist(),
        "neck_tilt_rad": [float(x[3]), float(x[4])],
        "pelvis_up": o["up"].tolist(),
        "imu_rpy_at_calibration": o["imu_rpy"],
        "lean_source": lean,
        "head_yaw_approach": {"ends_moving": "+ (left)", "deg": _yaw_approach_deg()},
        "fit": rep,
    }
    # the floor: from a floor session, or a table session with the table's taped height
    for s, (meta, _) in enumerate(sessions):
        H = float(x[8 + s])
        above = 0.0 if meta["surface"] == "floor" else meta.get("table_height_m")
        if above is None:
            continue
        cal["pelvis_above_floor_m"] = H + above
        ankles = [ff.ankle_midpoint(J, ob["angles"]) for ob in obs if ob["session"] == s]
        ankles = [(a, ob["up"]) for a, ob in zip(ankles, [ob for ob in obs if ob["session"] == s])
                  if a is not None]
        if ankles:
            cal["ankle_above_floor_m"] = float(np.mean([H + above + up @ a for a, up in ankles]))
        cal["floor_from"] = "%s session %d" % (meta["surface"], s)
    if "pelvis_above_floor_m" not in cal and base:
        for k in ("pelvis_above_floor_m", "ankle_above_floor_m"):
            if base.get(k) is not None:
                cal[k] = base[k]
        cal["floor_from"] = "carried over from the previous calibration (%s)" % base.get("date")
    if cal.get("pelvis_above_floor_m") is not None:
        cal["camera_above_floor_m"] = float(o["up"] @ T[:3, 3] + cal["pelvis_above_floor_m"])
    return cal


def cmd_fit(args, save=True):
    J = fk.load_urdf(cfg.URDF_PATH)
    base = cc.load(args.cal)
    sessions = [load_session(f) for f in args.folders]
    print("finding the level %s in %d poses..."
          % ("/".join(sorted({m["surface"] for m, _ in sessions})), sum(len(p) for _, p in sessions)))
    obs = observations(sessions, J, args.lean, base)
    if not obs:
        raise SystemExit("no usable poses")
    lean_warn = {o["lean"]["warning"] for o in obs if o["lean"].get("warning")}
    print("\n" + ff.describe(obs[0]["lean"]))
    for w in lean_warn:
        print("  WARNING (some poses): %s" % w)
    if base:
        evaluate(base, obs, J, "THE SAVED CALIBRATION (%s, %s) on these poses:"
                 % (base.get("date"), base.get("method", "?")[:40]))
    if not save:
        return 0
    if len(obs) < 6:
        raise SystemExit("only %d usable poses -- too few to fit 8 numbers honestly" % len(obs))
    x, model, rep = fit(obs, J, base, len(sessions))
    print_params(rep["params"])
    cal = calibration(x, model, obs, sessions, base, rep, args.lean, args.folders)
    if not args.save:
        print("\nnot saved -- add --save to use it")
        return 0
    if base and os.path.exists(args.cal):
        bak = args.cal + ".before_" + time.strftime("%Y%m%d_%H%M")
        shutil.copy(args.cal, bak)
        print("\nthe old calibration is kept as %s" % bak)
    cc.save(cal, args.cal)
    print("saved %s -- check_stack.py and everything else use it now" % args.cal)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="move the head through the poses, save frames + state")
    c.add_argument("interface")
    c.add_argument("--surface", choices=("table", "floor"), required=True)
    c.add_argument("--table-height", type=float, metavar="CM",
                   help="tape, floor to table top: sets the floor's height from a table sweep")
    c.add_argument("--pitches", help="comma-separated degrees, instead of the default grid")
    c.add_argument("--yaws", help="comma-separated degrees")
    c.add_argument("--frames", type=int, default=FRAMES_PER_POSE)
    c.add_argument("--out", help="folder (default sweeps/<date>_<surface>)")
    c.add_argument("--go", action="store_true")
    for name in ("fit", "check"):
        f = sub.add_parser(name)
        f.add_argument("folders", nargs="+")
        f.add_argument("--lean", choices=("imu", "legs", "level"), default="imu",
                       help="where the floor's up comes from (default: the IMU, legs as fallback)")
        f.add_argument("--cal", default=cc.CAL_PATH, help="the calibration file")
        if name == "fit":
            f.add_argument("--save", action="store_true")
    args = ap.parse_args(argv)
    np.seterr(invalid="ignore")
    if args.cmd == "capture":
        return cmd_capture(args)
    return cmd_fit(args, save=(args.cmd == "fit"))


if __name__ == "__main__":
    sys.exit(main())
