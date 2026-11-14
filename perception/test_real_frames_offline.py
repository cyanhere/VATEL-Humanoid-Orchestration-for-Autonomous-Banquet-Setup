"""Regression test on REAL frames from the R1's head camera.

    python3 test_real_frames_offline.py

Three frames captured on the first day the helmet was on the robot: a stack of lab plates at
4, 3 and 2 plates, taken in that order. They earn a test of their own because they caught a
bug the synthetic scenes never did -- a real rim's top band is a ring with gaps, so the outline
traced both edges of the ring and the plate read ~200 mm with a 35 mm fit residual.

There is no exact ground truth here, so the checks are the things that must hold whatever the
truth is: the count, the step between frames, the same plate measuring the same in two frames,
and the clipped frame being flagged.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import check_stack as cs
import plate_detectors as pd
import plate_stack as ps

PITCH = 0.009          # measured from these very frames
RIM1 = 0.0156          # a lone plate's rim in lab_3plates.npz

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


def measure(name):
    color, depth, intr, scale = cs.load_frame(os.path.join("real_frames", name))
    plane = pd.dominant_plane(depth, intr, scale)
    found = ps.locate_stacks(depth, intr, plane, depth_scale=scale, plate_diameter_m=0.254,
                             plate_pitch_m=PITCH, plate_rim_height_m=RIM1,
                             require_plate_size=False, allow_clipped=True)
    # the background (window sill, wall) also stands above the table plane; the stack is
    # the plate-sized one
    return min(found, key=lambda s: s["size_error"] if not s["clipped"] else
               abs(s["top_height_m"] - 0.04))


frames = {4: measure("lab_4plates_clipped.npz"),
          3: measure("lab_3plates.npz"),
          2: measure("lab_2plates.npz")}

print("\n  THE COUNT")
for n, s in frames.items():
    check("%d plates are counted as %d" % (n, n), s["n_plates"] == n,
          "top %.1f mm, counted %s via %s" % (1000 * s["top_height_m"], s["n_plates"],
                                              s["count_from"]))

print("\n  ONE PLATE OFF IS ONE PITCH DOWN")
for a, b in ((4, 3), (3, 2)):
    step = frames[a]["top_height_m"] - frames[b]["top_height_m"]
    check("%d -> %d plates drops the top by about one pitch" % (a, b),
          0.6 * PITCH < step < 1.4 * PITCH, "%.1f mm" % (1000 * step))

print("\n  THE RIM IS ITS OUTER EDGE, NOT HALFWAY TO THE WELL")
d3, d2 = frames[3]["diameter_m"], frames[2]["diameter_m"]
check("the same plate measures the same in two frames", abs(d3 - d2) < 0.008,
      "%.1f mm and %.1f mm" % (1000 * d3, 1000 * d2))
check("...and it matches the ruler (10 in = 254 mm), not the ~200 mm of the old outline",
      abs(d3 - 0.254) < 0.005, "%.1f mm vs 254.0 mm" % (1000 * d3))
for n in (3, 2):
    check("the %d-plate rim fits a circle to within 15 mm rms" % n,
          frames[n]["rim_rms_m"] < 0.015,
          "%.1f mm (was 35 mm before the outer-edge fix)" % (1000 * frames[n]["rim_rms_m"]))

print("\n  THE FRAME THAT RAN OFF THE EDGE")
check("the 4-plate frame, cut by the bottom of the picture, is flagged clipped",
      frames[4]["clipped"] and not frames[4]["diameter_reliable"])
check("...while its height is still on the line through the others",
      abs(frames[4]["top_height_m"] - (frames[3]["top_height_m"] + PITCH)) < 0.004,
      "%.1f mm vs %.1f expected"
      % (1000 * frames[4]["top_height_m"], 1000 * (frames[3]["top_height_m"] + PITCH)))

print("\n  THE SAME 4-PLATE STACK, FULLY IN FRAME")
f4 = measure("lab_4plates.npz")
check("is accepted, not clipped", not f4["clipped"])
check("is counted as 4 from its height", f4["n_plates"] == 4,
      "top %.1f mm" % (1000 * f4["top_height_m"]))
check("...and the ledges down its side agree", len(f4["ledges_m"]) == 4,
      "%d ledges" % len(f4["ledges_m"]))
check("its height matches the clipped 4-plate frame to within 1 mm",
      abs(f4["top_height_m"] - frames[4]["top_height_m"]) < 0.001,
      "%.1f vs %.1f mm" % (1000 * f4["top_height_m"], 1000 * frames[4]["top_height_m"]))
check("its diameter is within 6 mm of the ruler", abs(f4["diameter_m"] - 0.254) < 0.006,
      "%.1f mm" % (1000 * f4["diameter_m"]))

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
