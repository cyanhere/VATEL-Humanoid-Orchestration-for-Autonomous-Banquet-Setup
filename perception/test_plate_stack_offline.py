"""Off-robot test for the plate-stack detector.

    python3 test_plate_stack_offline.py

No robot, no camera, no cv2. The scenes are RAY-MARCHED from a solid of revolution whose
radius is wavy with the plate pitch, so the stack's side really does have one ledge per plate
and the top plate really does have a rim standing proud of a well. The two things most likely
to break the detector are therefore present in the data rather than assumed away:

  - the rim/well pair, which makes the TOP PLATE ALONE produce two height clusters. Reading
    the well as the top surface puts the answer ~15 mm low, which is more than a plate pitch.
  - the nested side, which makes lower plates' rims visible and countable.

Ground truth is exact because the geometry is generated, so the height error can be compared
against the plate pitch, which is the only tolerance that means anything here.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import plate_stack as ps
from stack_scene import INTR, PLATE_D, R, RI, PITCH, RIM0, WELL, WAVE, render

ok = True


def check(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print("  %-62s %s" % (label, "PASS" if cond else "FAIL"))
    if detail:
        print("  %-62s      %s" % ("", detail))


# ==========================================================================
print("\n  PLANE HANDLING")
# ==========================================================================
depth, T = render(n_plates=8)
for label, form in (("(n, c) tuple", T["plane"]),
                    ("[a,b,c,d] vector", list(T["plane"][0]) + [-T["plane"][1]]),
                    ("dict with normal+c", {"normal": T["plane"][0], "c": T["plane"][1]}),
                    ("dict with normal+point", {"normal": T["plane"][0], "point": T["O"]})):
    try:
        r = ps.top_plate(depth, INTR, form, plate_diameter_m=PLATE_D)
        got = abs(r["top_height_m"] - T["stacks"][0]["top_height_m"]) < 0.004
    except Exception as e:
        got = False
    check("a plane given as a %s works" % label, got)

try:
    ps.top_plate(depth, INTR, (-T["plane"][0], -T["plane"][1]), plate_diameter_m=PLATE_D)
    flipped_ok = True
except ps.NoStackFound as e:
    flipped_ok = "inverted" in str(e) or "BELOW" in str(e)
check("an inverted plane is refused with a reason, not silently empty", flipped_ok)

# ==========================================================================
print("\n  THE TOP PLATE IS THE RIM, NOT THE WELL")
# ==========================================================================
truth = T["stacks"][0]
r = ps.top_plate(depth, INTR, T["plane"], plate_diameter_m=PLATE_D, plate_pitch_m=PITCH)
err = abs(r["top_height_m"] - truth["top_height_m"])
check("top height is within 3 mm of truth", err < 0.003,
      "measured %.4f m, truth %.4f m, error %.1f mm"
      % (r["top_height_m"], truth["top_height_m"], 1000 * err))
check("...which is well inside half a plate pitch", err < 0.5 * PITCH,
      "error %.1f mm vs pitch %.0f mm" % (1000 * err, 1000 * PITCH))
check("it did NOT lock onto the well %.0f mm lower" % (1000 * WELL),
      abs(r["top_height_m"] - (truth["top_height_m"] - WELL)) > 0.5 * WELL,
      "the well would read %.4f m" % (truth["top_height_m"] - WELL))
check("the height came from the whole rim, not a few pixels", r["rim_px"] > 300,
      "%d rim pixels -> depth noise averages down by ~%.0fx"
      % (r["rim_px"], np.sqrt(r["rim_px"])))

# ==========================================================================
print("\n  SIZE AND POSITION")
# ==========================================================================
check("diameter within 8 mm", abs(r["diameter_m"] - PLATE_D) < 0.008,
      "measured %.4f m vs %.3f m" % (r["diameter_m"], PLATE_D))
want = truth["base"] + T["plane"][0] * truth["top_height_m"]
derr = float(np.linalg.norm(r["center_m"] - want))
check("rim centre within 8 mm in 3-D", derr < 0.008, "%.1f mm out" % (1000 * derr))
check("the rim is reported as foreshortened, as it must be at 45 deg",
      0.5 < r["axis_ratio"] < 0.85, "axis ratio %.3f" % r["axis_ratio"])

# ==========================================================================
print("\n  IT TRACKS THE STACK DOWN, ONE PLATE AT A TIME")
# ==========================================================================
meas = []
for npl in (10, 9, 8, 7, 6):
    d2, T2 = render(n_plates=npl, seed=npl)
    rr = ps.top_plate(d2, INTR, T2["plane"], plate_diameter_m=PLATE_D, plate_pitch_m=PITCH)
    meas.append((npl, rr["top_height_m"], T2["stacks"][0]["top_height_m"]))
worst = max(abs(m - t) for _, m, t in meas)
check("every height from 10 plates down to 6 is within 3 mm", worst < 0.003,
      "worst %.1f mm" % (1000 * worst))
steps = [meas[i][1] - meas[i + 1][1] for i in range(len(meas) - 1)]
check("removing one plate drops the measured height by one pitch",
      all(abs(s - PITCH) < 0.0025 for s in steps),
      "steps: " + ", ".join("%.1f" % (1000 * s) for s in steps) + " mm")
check("the heights are strictly decreasing, so a pick is detectable",
      all(meas[i][1] > meas[i + 1][1] for i in range(len(meas) - 1)))

# ==========================================================================
print("\n  TWO STACKS ON ONE TABLE")
# ==========================================================================
# 0.60 m, not the usual 0.42: two 225 mm plates 0.28 m apart span 0.50 m, and the
# frame only covers 0.45 m at 0.42 m range, so the outer one would be clipped --
# which the detector now refuses, correctly. A real rig needs the same standback.
d3, T3 = render(n_plates=8, offset=(-0.14, 0.0), extra=[(4, (0.14, 0.0))],
                dist=0.60, seed=7)
found = ps.locate_stacks(d3, INTR, T3["plane"], plate_diameter_m=PLATE_D,
                         plate_pitch_m=PITCH)
check("both stacks are found", len(found) >= 2, "%d found" % len(found))
if len(found) >= 2:
    hts = sorted(f["top_height_m"] for f in found[:2])
    want_h = sorted(s["top_height_m"] for s in T3["stacks"])
    check("and their two different heights are both right",
          all(abs(a - b) < 0.004 for a, b in zip(hts, want_h)),
          "measured %s vs truth %s"
          % (["%.3f" % x for x in hts], ["%.3f" % x for x in want_h]))
    seps = np.linalg.norm(found[0]["center_m"] - found[1]["center_m"])
    check("they are separated by about the 0.28 m they were placed at",
          abs(seps - 0.28) < 0.02, "%.3f m apart" % seps)

# ==========================================================================
print("\n  COUNTING PLATES FROM THE LEDGES")
# ==========================================================================
# rim_heights on a constructed histogram: exact input, so this isolates the logic
synth = np.concatenate([np.random.default_rng(3).normal(RIM0 + k * PITCH, 0.0008, 400)
                        for k in range(6)])
led = ps.rim_heights(synth, pitch_hint=PITCH)
check("six evenly spaced ledges are recovered from a clean profile",
      len(led) == 6, "%d found: %s" % (len(led), ["%.3f" % x for x in led]))
if len(led) >= 2:
    gaps = np.diff(sorted(led))
    check("...and their spacing is the plate pitch",
          abs(float(np.median(gaps)) - PITCH) < 0.0015,
          "median gap %.1f mm" % (1000 * float(np.median(gaps))))
check("a flat blob yields a single ledge, not a phantom stack",
      len(ps.rim_heights(np.random.default_rng(4).normal(0.02, 0.001, 500),
                         pitch_hint=PITCH)) <= 2)
check("too few points yields nothing rather than a guess",
      ps.rim_heights(np.array([0.02, 0.021]), pitch_hint=PITCH) == [])

r8 = ps.top_plate(depth, INTR, T["plane"], plate_diameter_m=PLATE_D, plate_pitch_m=PITCH)
if r8["n_plates"] is not None:
    check("a rendered 8-stack counts within one plate", abs(r8["n_plates"] - 8) <= 1,
          "counted %d, pitch %.1f mm" % (r8["n_plates"], 1000 * (r8["pitch_m"] or 0)))
else:
    check("when the ledges are not evenly spaced it reports None, not a number",
          r8["n_plates"] is None,
          "%d ledges found but spacing too ragged to trust" % len(r8["ledges_m"]))

# ==========================================================================
print("\n  IT REFUSES RATHER THAN INVENTS")
# ==========================================================================
bare, Tb = render(n_plates=0, holes=0.0)
check("the bare frame really is bare (nothing standing on it)",
      bare.max() > 0 and np.isfinite(Tb["plane"][1]))
try:
    ps.locate_stacks(bare, INTR, Tb["plane"], plate_diameter_m=PLATE_D)
    empty_ok = False
except ps.NoStackFound:
    empty_ok = True
check("an empty table raises instead of returning a stack", empty_ok)

try:
    ps.locate_stacks(depth, INTR, T["plane"], plate_diameter_m=0.60)
    wrong_ok = False
except ps.NoStackFound as e:
    wrong_ok = "plate-shaped" in str(e)
check("a stack of the wrong size is rejected, with a hint how to look",
      wrong_ok)
shown = ps.locate_stacks(depth, INTR, T["plane"], plate_diameter_m=0.60,
                         require_plate_size=False)
check("...and require_plate_size=False shows it with its size_error",
      len(shown) >= 1 and shown[0]["size_error"] > 0.4,
      "size_error %.2f against the wrong expectation" % shown[0]["size_error"])

# ==========================================================================
print("\n  A STACK RUNNING OFF THE FRAME IS CAUGHT, NOT MEASURED")
# ==========================================================================
# Found by the robustness sweep: at a 0.25 m standoff the plate overflows the frame, its
# outline becomes a chord, and the fitted diameter was 67 mm out -- which squeezed just
# inside the 30% size gate and was ACCEPTED. A 20-plate stack failed the same way.
for lbl, kw in (("a plate too close to fit the frame", dict(n_plates=8, dist=0.25)),
                ("a stack too tall to fit the frame", dict(n_plates=20, seed=20))):
    dc, Tc = render(**kw)
    try:
        locate = ps.top_plate(dc, INTR, Tc["plane"], plate_diameter_m=PLATE_D,
                              plate_pitch_m=PITCH)
        caught = False
    except ps.NoStackFound as e:
        caught = "frame edge" in str(e)
    check("%s is refused, by name" % lbl, caught)

    r_c = ps.top_plate(dc, INTR, Tc["plane"], plate_diameter_m=PLATE_D,
                       plate_pitch_m=PITCH, allow_clipped=True)
    want_c = Tc["stacks"][0]["top_height_m"]
    check("...and allow_clipped still gives a good HEIGHT",
          abs(r_c["top_height_m"] - want_c) < 0.002,
          "%.1f mm error -- a partial rim is still at the rim's height"
          % (1000 * abs(r_c["top_height_m"] - want_c)))
    check("...while flagging the diameter as not to be trusted",
          r_c["clipped"] is True and r_c["diameter_reliable"] is False)

# ==========================================================================
print("\n  THE TOP HEIGHT DOES NOT DRIFT UP WITH DEPTH NOISE")
# ==========================================================================
# Also from the sweep. Anchoring the top band on the 99th percentile of a noisy
# distribution put it ~2.3 sigma high -- at 8 mm noise that is nearly two pitches, biased
# upward on every frame. The band is now re-centred on its own median until it settles.
for nz in (0.002, 0.004, 0.008):
    dn, Tn = render(n_plates=8, noise=nz, seed=int(nz * 1e4))
    rn = ps.top_plate(dn, INTR, Tn["plane"], plate_diameter_m=PLATE_D, plate_pitch_m=PITCH)
    en = abs(rn["top_height_m"] - Tn["stacks"][0]["top_height_m"])
    check("at %.0f mm depth noise the height is within a third of a pitch"
          % (1000 * nz), en < PITCH / 3.0,
          "%.1f mm error (%.0f%% of pitch)" % (1000 * en, 100 * en / PITCH))

# ==========================================================================
print("\n  COUNTING PREFERS THE HEIGHT, AND SAYS WHICH IT USED")
# ==========================================================================
# One plate added is exactly one pitch of height, so given the pitch and a single plate's
# rim height the count follows from the height. Ledges cannot do better than a lower bound:
# from near the table normal the stack's side barely shows, and a 16-stack showed 5 ledges
# and was reported as 5 plates.
for npl in (1, 3, 10, 16, 20):
    for tilt in (25.0, 45.0):
        dk, Tk = render(n_plates=npl, tilt_deg=tilt, seed=npl * 100 + int(tilt))
        rk = ps.top_plate(dk, INTR, Tk["plane"], plate_diameter_m=PLATE_D,
                          plate_pitch_m=PITCH, plate_rim_height_m=RIM0,
                          allow_clipped=True)
        check("%2d plates at %.0f deg are counted exactly" % (npl, tilt),
              rk["n_plates"] == npl,
              "got %s via %s" % (rk["n_plates"], rk["count_from"]))

dk, Tk = render(n_plates=10, tilt_deg=25.0, seed=1025)
rk = ps.top_plate(dk, INTR, Tk["plane"], plate_diameter_m=PLATE_D, plate_pitch_m=PITCH)
check("without a rim height, a poor view admits None instead of undercounting",
      rk["n_plates"] is None,
      "ledges alone saw %d of 10" % len(rk["ledges_m"]))

# ==========================================================================
print("\n  THE ROBOT FRAME IS NOT HANDED OUT UNCALIBRATED")
# ==========================================================================
# calibration=None explicitly: these check the no-calibration path, and must not depend on
# whether a camera_calibration.json happens to be sitting in the folder.
try:
    ps.to_robot_frame(r, calibration=None)
    refused = False
except RuntimeError:
    refused = True
check("to_robot_frame refuses without a calibration", refused)
conv = ps.to_robot_frame(r, allow_nominal=True, calibration=None)
check("...but allow_nominal=True converts for geometry work",
      np.isfinite(conv["center_m"]).all() and conv["top_height_m"] == r["top_height_m"])

print()
print("ALL PASS" if ok else "FAILURES ABOVE")
sys.exit(0 if ok else 1)
