"""
Off-robot test for detect_plate_circle() -- picking the plate's RIM out of the several
circles a decorated plate offers.

    python3 test_plate_detect_offline.py

Built from the real failure on 2026-09-29: a plate with a bold floral inner ring, where
Hough's strongest response sat on the ring rather than the rim. The circle was wrong by
about 20% of the radius, the depth annulus then straddled the rim and the table beyond
it, and the plate read 57 mm low -- while rim_valid reported 90%, because it only counts
valid depth samples and cannot tell which circle they came from.

The images here are synthetic but the geometry is the real one: a 0.23 m plate at 0.46 m
seen through D435 intrinsics.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

try:
    import cv2
except ImportError:
    print("cv2 is not installed here -- this suite needs it. Skipping.")
    raise SystemExit(0)

import plate_perception as pp

INTR = {"fx": 615.0, "fy": 615.0, "cx": 320.0, "cy": 240.0, "width": 640, "height": 480}
PLATE_D = 0.23
DIST = 0.46
R_TRUE = INTR["fx"] * (PLATE_D / 2) / DIST      # ~154 px
CX, CY = 320.0, 240.0

ok = True
def check(label, cond):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))


def scene(inner_ring=True, ring_frac=0.78):
    """A plate on a table: pale rim, and optionally a hard-edged ring painted inside it.

    The ring is drawn with far more contrast than the rim, which is what makes Hough
    prefer it -- exactly the real plate's floral band on white.
    """
    color = np.full((480, 640, 3), 40, np.uint8)                 # dark table
    cv2.circle(color, (int(CX), int(CY)), int(R_TRUE), (225, 225, 228), -1)   # plate
    if inner_ring:
        cv2.circle(color, (int(CX), int(CY)), int(R_TRUE * ring_frac), (20, 20, 120), 6)

    # Depth: the plate sits DIST away, the table 4 cm further back.
    depth = np.full((480, 640), (DIST + 0.04) * 1000.0, np.float64)
    yy, xx = np.mgrid[0:480, 0:640]
    on_plate = (xx - CX) ** 2 + (yy - CY) ** 2 <= R_TRUE ** 2
    depth[on_plate] = DIST * 1000.0
    return color, depth.astype(np.uint16)


print("detect_plate_circle() tests (synthetic plate, real geometry):")

color, depth = scene(inner_ring=False)
u, v, r = pp.detect_plate_circle(color, depth, INTR, PLATE_D)
check("a plain plate: rim found within 8%% of its true %.0f px radius" % R_TRUE,
      abs(r - R_TRUE) / R_TRUE < 0.08)
check("...and centred", abs(u - CX) < 8 and abs(v - CY) < 8)

# The real failure: a hard inner ring that out-scores the rim.
color, depth = scene(inner_ring=True)
u, v, r = pp.detect_plate_circle(color, depth, INTR, PLATE_D)
check("decorated plate: the RIM wins, not the bold ring inside it",
      abs(r - R_TRUE) / R_TRUE < 0.10)
check("...so the centre is the plate's, not the ring's", abs(u - CX) < 8 and abs(v - CY) < 8)

# Mutation guard: the check must be what rejects the ring. Fitting the ring implies a
# distance ~1/0.78 too far, which has to exceed the tolerance or nothing is enforced.
implied = 1.0 / 0.78 - 1.0
check("a ring at 78%% of the radius is outside the tolerance that rejects it",
      implied > pp.SIZE_DEPTH_TOLERANCE)

# Told the wrong plate size, the whole pipeline must refuse rather than hand back a
# confident wrong circle. The seed finder no longer raises for this -- only its CENTRE
# is used now -- so the check belongs at locate_plate, which has a measured circle to
# judge against.
color, depth = scene(inner_ring=False)
try:
    pp.get_plate_pose(color, depth, INTR, 0.60)           # claim a 60 cm plate
    check("a plate that is not the size it was told -> refused", False)
except RuntimeError as e:
    check("a plate that is not the size it was told -> refused", "0.600" in str(e))

# The whole pose, end to end: distance from depth, centre from the right circle.
color, depth = scene(inner_ring=True)
pose = pp.get_plate_pose(color, depth, INTR, PLATE_D)
check("get_plate_pose: distance comes out at the true %.2f m" % DIST,
      abs(pose["distance_m"] - DIST) < 0.02)
check("get_plate_pose: centre lands on the plate, not offset onto the ring",
      abs(pose["center"][0]) < 0.01 and abs(pose["center"][1]) < 0.01)

print("\n%s" % ("ALL PASS" if ok else "FAILURES PRESENT"))
sys.exit(0 if ok else 1)
