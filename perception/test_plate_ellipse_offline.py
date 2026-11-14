"""
Off-robot test for the ellipse-based plate detector.

    python3 test_plate_ellipse_offline.py

No camera, no robot, no cv2 -- the detector is pure numpy. The scenes are RAY-CAST from
real geometry rather than drawn, so the plate lands in the image as the ellipse it
actually is (squashed 7-24% at the distances the robot works at), which is the whole
point of the module under test.
"""
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import plate_detectors as pd

INTR = {"fx": 600.0, "fy": 600.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}
PLATE_D = 0.225
R = PLATE_D / 2

ok = True


def check(label, cond):
    global ok
    ok = ok and bool(cond)
    print("  %-66s %s" % (label, "PASS" if cond else "FAIL"))


def render(tilt_deg=32.0, dist=0.46, lift=0.08, holes=0.55, arms=True, seed=0):
    """Ray-cast a plate standing on a table, as the camera really sees it.

    tilt_deg is the angle between the camera's view of the plate and the plate's own
    normal -- 22 deg at the near working distance, 41 deg at the far one.
    """
    h, w = 480, 640
    yy, xx = np.mgrid[0:h, 0:w]
    dirs = np.stack([(xx - INTR["cx"]) / INTR["fx"],
                     (yy - INTR["cy"]) / INTR["fy"],
                     np.ones_like(xx, dtype=float)], axis=2)

    t = np.radians(tilt_deg)
    n = np.array([0.0, -np.sin(t), -np.cos(t)])      # plate/table normal, toward the camera
    centre = np.array([0.0, 0.0, dist])              # plate centre, on the plate plane
    c_plate = float(n @ centre)
    c_table = c_plate - lift                         # the table sits `lift` BEHIND the plate

    nd = dirs @ n
    nd = np.where(np.abs(nd) < 1e-9, 1e-9, nd)
    t_plate = c_plate / nd
    t_table = c_table / nd

    hit = dirs * t_plate[:, :, None]                 # where each ray meets the plate plane
    on_plate = np.linalg.norm(hit - centre, axis=2) <= R

    depth = np.where(on_plate & (t_plate > 0), t_plate, t_table)
    depth = np.where(depth > 0, depth, 0.0)

    rng = np.random.default_rng(seed)
    if arms:                                          # the robot's own hands, much nearer
        depth[(xx < 90) | (xx > 550)] = 0.30
    if holes:                                         # specular dropout on the glaze
        inner = np.linalg.norm(hit - centre, axis=2) <= 0.7 * R
        depth[inner & (rng.random((h, w)) < holes)] = 0.0
    depth += rng.normal(0, 0.0015, depth.shape) * (depth > 0)
    return (depth * 1000.0).astype(np.uint16), centre


print("ellipse fitting:")
th = np.radians(27.0)
ang = np.linspace(0, 2 * np.pi, 90, endpoint=False)
E = {"cx": 311.0, "cy": 236.0, "major": 152.0, "minor": 128.0, "theta": th}
p = np.stack([E["cx"] + E["major"] * np.cos(ang) * np.cos(th) - E["minor"] * np.sin(ang) * np.sin(th),
              E["cy"] + E["major"] * np.cos(ang) * np.sin(th) + E["minor"] * np.sin(ang) * np.cos(th)],
             axis=1)
got = pd.fit_ellipse(p)
check("a known ellipse is recovered to within a pixel",
      got is not None and abs(got["cx"] - E["cx"]) < 1 and abs(got["cy"] - E["cy"]) < 1
      and abs(got["major"] - E["major"]) < 1 and abs(got["minor"] - E["minor"]) < 1)
check("...including which way round the major axis lies",
      abs(((got["theta"] - th + np.pi / 2) % np.pi) - np.pi / 2) < 0.02)

noisy = p + np.random.default_rng(1).normal(0, 1.2, p.shape)
gotn = pd.fit_ellipse(noisy)
check("and it survives a pixel or so of noise on the outline",
      abs(gotn["major"] - E["major"]) < 4 and abs(gotn["minor"] - E["minor"]) < 4)

check("a straight line is refused rather than fitted as a sliver",
      pd.fit_ellipse(np.stack([np.arange(40.0), np.arange(40.0) * 2], axis=1)) is None)

print("\nthe detector, on ray-cast scenes:")
for tilt, label in ((22.0, "near   (22 deg off the normal, 7% squashed)"),
                    (32.0, "middle (32 deg, 15% squashed)"),
                    (41.0, "far    (41 deg, 24% squashed)")):
    depth, centre = render(tilt_deg=tilt)
    r = pd.detect_plate_ellipse(depth, INTR, PLATE_D)
    check("%s -> diameter within 10 mm" % label, abs(r["diameter_m"] - PLATE_D) < 0.010)

# The plate's CENTRE, in metres. Not the ellipse's centre: for a tilted circle those are
# different points by 9-14 mm here, a bias toward the far side on every single frame.
for tilt in (22.0, 32.0, 41.0):
    depth, centre = render(tilt_deg=tilt)
    r = pd.detect_plate_ellipse(depth, INTR, PLATE_D)
    err = np.linalg.norm(np.asarray(r["centre_m"]) - centre)
    check("%.0f deg: the centre is within 8 mm of the real one" % tilt, err < 0.008)
    ell = np.hypot(r["cx"] - (INTR["cx"] + centre[0] * INTR["fx"] / centre[2]),
                   r["cy"] - (INTR["cy"] + centre[1] * INTR["fy"] / centre[2]))
    check("...and beats the ellipse centre, which is %.0f mm out by construction"
          % (1000 * ell * r["distance_m"] / INTR["fx"]),
          err < ell * r["distance_m"] / INTR["fx"])

depth, centre = render(tilt_deg=32.0)
r = pd.detect_plate_ellipse(depth, INTR, PLATE_D)
check("the squash it measures matches the geometry it was rendered from",
      abs(r["axis_ratio"] - np.cos(np.radians(32.0))) < 0.06)

check("the robot's own arms in frame are not mistaken for the plate",
      abs(pd.detect_plate_ellipse(render(arms=True)[0], INTR, PLATE_D)["diameter_m"]
          - PLATE_D) < 0.010)
try:
    pd.detect_plate_ellipse(render(lift=0.0, holes=0, arms=False)[0], INTR, PLATE_D)
    check("...and a frame with nothing standing on the table is refused", False)
except RuntimeError:
    check("...and a frame with nothing standing on the table is refused", True)

print("\nwhy an ellipse and not a circle:")
# The same silhouette, fitted both ways. A circle has no way to represent a squashed
# plate, so its radius lands between the two axes and the diameter it implies is wrong.
sys.modules.setdefault("cv2", types.ModuleType("cv2"))
import plate_perception as pp

depth, centre = render(tilt_deg=41.0, holes=0.0, arms=False)
plane = pd.dominant_plane(depth, INTR)
height = pd.height_above_plane(depth, INTR, plane)
standing = np.isfinite(height) & (height > pd.MIN_STAND_M) & (height < pd.MAX_STAND_M)
edge = pd.outline(standing)
d = float(np.median(np.asarray(depth)[standing]) / 1000.0)

e = pd.fit_ellipse(edge)
d_ellipse = 2 * e["major"] * d / INTR["fx"]
cx, cy, cr = pp._fit_circle_ransac(edge, e["major"], tol=6.0)
d_circle = 2 * cr * d / INTR["fx"]

print("     true %.3f m | ellipse %.3f m (%+.0f mm) | circle %.3f m (%+.0f mm)"
      % (PLATE_D, d_ellipse, 1000 * (d_ellipse - PLATE_D),
         d_circle, 1000 * (d_circle - PLATE_D)))
check("the ellipse gets the diameter right on a squashed plate",
      abs(d_ellipse - PLATE_D) < 0.010)
check("...and a circle fitted to the same outline is further out",
      abs(d_circle - PLATE_D) > abs(d_ellipse - PLATE_D) and abs(d_circle - PLATE_D) > 0.008)

print("\n%s" % ("ALL PASS" if ok else "FAILURES PRESENT"))
sys.exit(0 if ok else 1)
