"""table_check.py on the simulated arm with a flat table and a fake depth camera with a known pose error.

    python tests/test_table_mock.py                          # table at 0.030; the camera tilted 1 deg about x below the
    python tests/test_table_mock.py --roll 0 --dz 8          # the camera 8 mm too high instead
    python tests/test_table_mock.py --no-sweep
    python tests/test_table_mock.py --roll-mode calibrated # the roll below the yaw joint, but corrected at yaw 0 the way
    python tests/test_table_mock.py --roll 0 --pitch-backlash 1.5 # the camera lags the head pitch encoder by 0.75 deg
"""
import argparse
import glob
import json
import os
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from pk import config  # noqa: E402
from pk.client import operator  # noqa: E402
from pk.robot_io import MockBackend  # noqa: E402
from pk.server_core import Server  # noqa: E402

PORT = 9751
NECK = np.array([0.0265, 0.0316, 0.304])               # head pitch joint, pelvis frame (URDF, waist at 0)
MIRROR = os.path.join(os.path.dirname(HERE), "..", "robot_mirror", "R1_Plate_Perception")


def about(axis, a, point):
    """4x4: a rotation by a about axis through point."""
    u = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = point - R @ point
    return T


class FakeCamera(object):
    """Depth of a flat table at z_true (pelvis), seen from err @ (the camera pose R1_Plate_Perception's calibration gives
    for the mock's head angles): err is the camera's real pose error, in the pelvis frame -- a matrix, or a function of
    (camera, head angles) that may use the head's last direction of motion (self.dirn)."""

    def __init__(self, kin, io, z_true, err, noise_mm=1.0):
        self.kin, self.io, self.z, self.err = kin, io, z_true, err
        self.last, self.dirn = None, np.zeros(2)
        self.intr_d = dict(fx=420.0, fy=420.0, cx=424.0, cy=240.0, width=848, height=480)
        self.intr_c, self.T_cd, self.scale, self.colour = self.intr_d, np.eye(4), 1000.0, False
        self.rs = np.random.RandomState(3)
        self.noise = noise_mm

    def frames(self):
        s = self.io.s
        ang = self.kin.angles(dict(q=s.q.tolist(), head=s.head.tolist(), waist_roll=float(s.waist_roll)))
        hd = np.array(s.head, float)
        if self.last is not None:
            d = hd - self.last
            self.dirn = np.where(np.abs(d) > 0.002, np.sign(d), self.dirn)
        self.last = hd
        err = self.err(self, ang) if callable(self.err) else self.err
        T = err @ self.kin.T_po(ang)
        i = self.intr_d
        vv, uu = np.mgrid[0:i["height"], 0:i["width"]]
        ray = np.stack([(uu - i["cx"]) / i["fx"], (vv - i["cy"]) / i["fy"], np.ones(uu.shape)], axis=-1)
        d = ray @ T[:3, :3].T
        t = (self.z - T[2, 3]) / np.where(np.abs(d[..., 2]) > 1e-6, d[..., 2], 1e-6)
        P = T[:3, 3] + t[..., None] * d
        on = (t > 0.15) & (t < 2.0) & (P[..., 0] > 0.10) & (P[..., 0] < 1.0) & (np.abs(P[..., 1]) < 0.6)
        depth = np.where(on, 1000 * t + self.noise * self.rs.randn(*t.shape), 0.0)
        time.sleep(0.03)
        return None, np.maximum(depth, 0).astype(np.uint16), None, time.time()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--table", type=float, default=0.030, help="the table's true height, pelvis z (m)")
    ap.add_argument("--roll", type=float, default=1.0, help="the camera's real pose: turned this much about x through the neck (deg)")
    ap.add_argument("--dz", type=float, default=0.0, help="... and this much higher than the calibration says (mm)")
    ap.add_argument("--roll-mode", choices=["fixed", "calibrated"], default="fixed",
                    help="fixed: R1_Plate_Perception knows nothing of the roll; calibrated: it took it out at head yaw 0, in head_yaw_link")
    ap.add_argument("--pitch-backlash", type=float, default=0.0, help="deg: the camera lags the pitch encoder by half of it")
    ap.add_argument("--friction", type=float, default=0.3, help="every joint's Coulomb friction, N m")
    ap.add_argument("--friction-fit", type=float, default=None, help="instead: the robot's fitted per-joint friction "
                    "(model/gravity_fit.json; left arm 2-6x the right on the shoulder) times this")
    ap.add_argument("--fric", type=float, default=0.0, help="the server's friction lead (x the fitted friction; 0 off)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--stiction", type=float, default=1.0)
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--grid", action="store_true", help="run `table_check.py grid` instead, then calibration/arm_offsets_fit.py on it")
    ap.add_argument("--fit-out", default=None, help="--grid: also save the fitted offsets here (then rerun with "
                    "DEXPICK_ARM_OFFSETS=<that file> to check them)")
    ap.add_argument("--enc-off", default="", help="the simulated arm's encoder zero errors, deg: e.g. L_elbow=2,L_sh_pitch=-1")
    ap.add_argument("--ghost", type=float, default=None, help="mm above the table: the LEFT hand meets an invisible stop there "
                    "(no proximity) that gives way above --ghost-break N -- the R1's left arm stuck in free air on 2026-10-22 18:21")
    ap.add_argument("--ghost-break", type=float, default=6.0)
    a = ap.parse_args()
    import perception as per
    import table_check as tc
    from pk.model import Robot
    kin = per.Kin(MIRROR, correct=False)
    rob = Robot()
    home = json.load(open(os.path.join(config.ROOT, "params", "premotion.json")))
    q_pose, _, _ = tc.plan(rob, np.array(home["q"], float), np.zeros(12), 0.38, 0.22, 0.12)
    fr = a.friction
    if a.friction_fit is not None:
        fr = a.friction_fit * np.array(json.load(open(os.path.join(config.MODEL_DIR, "gravity_fit.json")))["friction_Nm"], float)
    io = MockBackend(friction=fr)
    io.stiction = a.stiction
    io.table = dict(z=a.table, pts={s: tc.HandPoints(rob, q_pose, np.zeros(12), s, band=0.03).cand for s in tc.SIDES},
                    prox=lambda gap: float(np.interp(gap, [0.0, 1.0, 3.0, 5.0, 9.5, 10.0], [400000, 250000, 60000, 10000, 200, 0])))   # the R1 on the table
    if a.ghost is not None:
        io.table["ghost"] = dict(side="left", z=a.table + a.ghost / 1000, break_N=a.ghost_break, arm_after=time.time() + 25.0)
    srv = Server(io, port=a.port, verbose=False, fric=a.fric, log_dir=os.path.join(os.path.dirname(HERE), "logs", "test"))
    run = dict(stop=False)

    def loop():
        nxt = time.time()
        while not run["stop"]:
            srv.step()
            nxt += 0.01
            time.sleep(max(0.0, nxt - time.time()))
    threading.Thread(target=loop, daemon=True).start()
    time.sleep(0.3)
    operator("take", port=a.port)
    while srv.mode != "HELD":
        time.sleep(0.05)
    operator("home", port=a.port)
    while srv.mode != "HOME":
        time.sleep(0.05)
    th = np.radians(a.roll)

    def err(cam, ang):
        E = about([1, 0, 0], th, NECK)
        E[2, 3] += a.dz / 1000
        if a.roll_mode == "calibrated":                 # R1_Plate_Perception's correction, measured at yaw 0, turns with the head
            T_hy = kin.fk.T(kin.joints, kin.rcfg.ROOT_LINK, "head_yaw_link", ang)
            E = E @ about(T_hy[:3, 0], -th, T_hy[:3, 3])
        if a.pitch_backlash:
            T_hp = kin.fk.T(kin.joints, kin.rcfg.ROOT_LINK, "head_pitch_link", ang)
            E = E @ about(T_hp[:3, 1], -0.5 * np.radians(a.pitch_backlash) * cam.dirn[0], T_hp[:3, 3])
        return E
    cam = FakeCamera(kin, io, a.table, err)
    for item in filter(None, a.enc_off.split(",")):
        k, v = item.split("=")
        io.enc_off[config.ARM_NAMES.index(k.strip())] = np.radians(float(v))
    what = ["grid"] if a.grid else ["touch"] + ([] if a.no_sweep else ["sweep"])
    t0 = time.time()
    # the camera handed over as a slow opener (the RealSense takes seconds): it must be opened before the arms are taken
    rc = tc.main(what + ["--port", str(a.port)], kin=kin, open_camera=lambda: (time.sleep(3.0), cam)[1])
    run["stop"] = True
    if a.ghost is not None:
        print("ghost at the end: %s" % io.table["ghost"])
    out = sorted(glob.glob(os.path.join(config.LOG_DIR, "table", "*")))[-1]
    if a.grid:
        import subprocess
        print("\n==== simulated arm (MockBackend), grid, %.0f s; true encoder offsets (deg): %s" % (
            time.time() - t0, {config.ARM_NAMES[i]: round(float(np.degrees(v)), 2) for i, v in enumerate(io.enc_off) if v}))
        g = json.load(open(os.path.join(out, "grid.json")))
        n_t = {s_: sum(1 for r in g["rows"] if r["side"] == s_ and "touched_z" in r) for s_ in tc.SIDES}
        print("touched spots: %s of %d; offsets in use (deg): %s; friction lead x%.2f, joint friction %s" % (
            n_t, len(tc.SPOTS), np.round(np.degrees(g["arm_offsets_rad"]), 2), a.fric, np.round(np.broadcast_to(fr, (11,)), 2)))
        ev = [json.loads(l_) for l_ in open(os.path.join(out, "events.jsonl"))]
        for s_ in tc.SIDES:
            rows = [r for r in g["rows"] if r["side"] == s_ and "touched_z" in r]
            err = [1000 * (r["touched_z"] - a.table) for r in rows]
            fz = [t_["fz"] for r in rows for t_ in r["touches"]]
            print("  %-5s TRUE touch error (FK of the encoders - table) median %+.1f worst %+.1f mm; stops in free air %d; "
                  "force at touch median %.1f max %.1f N" % (s_, np.median(err), err[int(np.argmax(np.abs(err)))],
                                                              sum(1 for e in ev if e["what"] == "stop in free air" and e.get("side") == s_),
                                                              np.median(fz), np.max(fz)))
        for s_ in tc.SIDES:
            e = [r["camera_minus_touched_mm"] for r in g["rows"] if r["side"] == s_ and "camera_minus_touched_mm" in r]
            print("  %-5s camera - touched (mm): median %+.1f, worst %+.1f" % (s_, np.median(e), e[int(np.argmax(np.abs(e)))]))
        rc2 = subprocess.call([sys.executable, os.path.join(os.path.dirname(HERE), "calibration", "arm_offsets_fit.py"), os.path.join(out, "grid.json"), "--joints", "0123"]
                              + (["--save", "--out", a.fit_out] if a.fit_out else []))
        return 0 if rc == 0 and rc2 == 0 and min(n_t.values()) >= len(tc.SPOTS) - 1 else 1
    res = json.load(open(os.path.join(out, "result.json")))
    print("\n==== simulated arm (MockBackend), %.0f s; table %.4f m; camera error: roll %.1f deg below the yaw joint (%s), %+.0f mm, "
          "pitch backlash %.1f deg" % (time.time() - t0, a.table, a.roll, a.roll_mode, a.dz, a.pitch_backlash))
    ok = rc == 0
    sym = np.abs(res["pose"]["symmetry_deg"])
    print("  pose: command right - mirror(left) max %.3f deg %s" % (sym.max(), "ok" if sym.max() < 0.05 else "FAIL"))
    ok &= sym.max() < 0.05
    for s in tc.SIDES:
        r = res["touch"][s]
        # what the camera should report at this hand: the fake camera's table seen through the model pose
        if "touched_z" not in r:
            print("  %s: NO TOUCH (%s) FAIL" % (s, r.get("why")))
            ok = False
            continue
        e_touch = 1000 * (r["touched_z"] - a.table)
        print("  %-5s camera table %.4f, touched %.4f (true %.4f: %+.1f mm), camera - touched %+.1f mm; phase 2: %s; touches by %s" % (
            s, r["camera_z"], r["touched_z"], a.table, e_touch, r["camera_minus_touched_mm"], r["phase2"], [t_["by"] for t_ in r["touches"]]))
        ok &= abs(e_touch) < 3.0
    if not a.no_sweep:
        for line in res.get("sweep", {}).get("summary", []) if "sweep" in res else json.load(open(os.path.join(out, "sweep.json")))["summary"]:
            print("  sweep: " + line)
    print("RESULT: %s  (logs %s)" % ("PASS" if ok else "FAIL", out))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
