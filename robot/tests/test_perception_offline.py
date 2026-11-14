"""perception.py's locate and track code on R1_Plate_Perception's saved frames (no camera, no robot).

    python3 tests/test_perception_offline.py [--perception ~/R1_Plate_Perception]
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))


class FakeCam(object):
    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        self.depth, self.color = z["depth"], (z["color"] if "color" in z else None)
        self.intr_c = json.loads(str(z["intr"]))
        self.intr_d = self.intr_c                    # the saved depth is already aligned to colour
        self.scale = float(z["depth_scale"])
        self.T_cd = np.eye(4)

    def frames(self, timeout_ms=1000):
        return None, self.depth.copy(), self.color, time.time()

    def aligned(self, fs):
        return self.color, self.depth.copy(), self.intr_c


class FakeLink(object):
    can_aim = False

    def __init__(self, angles, q):
        self.ang, self.q = angles, q

    def state(self):
        return dict(q=list(self.q), head=[self.ang["head_pitch_joint"], self.ang["head_yaw_joint"]],
                    head_cmd=[self.ang["head_pitch_joint"], self.ang["head_yaw_joint"]],
                    waist_roll=self.ang["waist_roll_joint"], hands=[0.0] * 12, req=None)

    def busy(self, s):
        import contextlib
        return contextlib.contextmanager(lambda: (yield))()

    def publish(self, p):
        pass


def main():
    ap = argparse.ArgumentParser()
    default = os.path.expanduser("~/R1_Plate_Perception")
    if not os.path.isdir(default):
        default = os.path.abspath(os.path.join(HERE, "..", "..", "robot_mirror", "R1_Plate_Perception"))
    ap.add_argument("--perception", default=default)
    a = ap.parse_args()
    import perception as P
    from pk.tracker import RimTracker
    kin = P.Kin(a.perception)
    # the head as it was when real_frames/ were saved: at the 19 Oct calibration's angles (~0, 0). From the
    # calibration is captured elsewhere (head 35 deg down), so its own angles no longer say where these frames were taken
    old = os.path.join(a.perception, "camera_calibration.json.before_20261022_1650")
    ang = dict(json.load(open(old))["angles_at_calibration"] if os.path.exists(old) else kin.cal["angles_at_calibration"])
    home = json.load(open(os.path.join(os.path.dirname(HERE), "params", "premotion.json")))
    link = FakeLink(ang, home["q"])
    args = argparse.Namespace(diameter=0.254, pitch=0.009, rim_height=0.0156)
    bad = 0
    for name in ("lab_3plates", "lab_4plates"):
        cam = FakeCam(os.path.join(a.perception, "real_frames", name + ".npz"))
        t0 = time.time()
        loc, why, frame = P.locate(cam, kin, link, args)
        if loc is None:
            print("FAIL %s: locate: %s" % (name, why))
            bad += 1
            continue
        print("%s located in %.1f s: centre %s, top %.1f mm, %s plates, diameter %.1f mm" % (
            name, time.time() - t0, np.round(loc["center"], 3), 1000 * loc["top_height"], loc["n_plates"], 1000 * loc["diameter"]))
        trk = RimTracker(loc["center"][:2], loc["center"][2], loc["table_z"], plate_r=args.diameter / 2)
        st = link.state()
        T = kin.T_po(kin.angles(st))
        est = trk.init(cam.depth, cam.intr_d, cam.scale, T, kin.spheres(st), cam.T_cd)
        t0 = time.time()
        for _ in range(5):
            est = trk.update(cam.depth, cam.intr_d, cam.scale, T, kin.spheres(st), cam.T_cd)
        d = np.linalg.norm(np.array(est["center"][:2]) - np.array(loc["center"][:2])) * 1000
        ok = est["ok"] and d < 6.0
        bad += not ok
        print("%s tracker: ok %s, %.1f mm from the located centre, n %s, coverage %s, %.0f ms/frame" % (
            "PASS" if ok else "FAIL", est["ok"], d, est.get("n"), est.get("coverage"), (time.time() - t0) / 5 * 1000))
        P.snapshot(cam.color, cam.intr_c, T, est["center"], trk.r_band, os.path.join(os.path.dirname(HERE), "logs", "perception_%s.jpg" % name), "offline test")
        # what the live view shows (perception.py --view-port): the tracker's points on the picture, and from above
        import cv2
        img = P.draw_view(cam.color, cam.intr_c, T, trk, est, "offline test", 0.0, False, kin.spheres(st))
        cv2.imwrite(os.path.join(os.path.dirname(HERE), "logs", "perception_view_%s.jpg" % name), img)
    print("%d failed" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
