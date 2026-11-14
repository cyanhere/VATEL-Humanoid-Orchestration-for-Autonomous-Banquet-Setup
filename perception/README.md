# Perception (Step 4)

Finds the plate stack and reports the top plate's pose in the robot frame, so the robot can take one
plate off the stack. This is the R1 plate-perception package, kept in step with
`R1-Humanoid-Robot/R1_Plate_Perception`. Run `python3 run_tests.py` for the offline tests.

## R1 Plate Pick-Up

Unitree **R1** with the clamp-on sensor helmet (Hesai JT128 + RealSense D435if) perceives a
round dinner plate on a banquet table and picks it up with a dexterous hand.

Status: **kinematics and perception geometry only.** Nothing here has been run on the R1 — the
robot has had no software setup at all yet. Nothing in this directory commands a motor, and the
parts that would need the SDK deliberately raise instead of guessing.

This is a port of `../G1_Plate_Perception`, kept separate on purpose. The two robots have
different trunks, a different neck, different reach and a different hand, but **the same five
arm joint names** — so G1 code imports cleanly, runs, and returns plausible numbers for the
wrong robot. Keeping one directory per robot is what stops that.

---

## What came from where

| File | Origin |
|---|---|
| `plate_detectors.py` | copied **verbatim** — pure numpy, no robot in it |
| `plate_perception.py` | copied **verbatim** — needs cv2, otherwise robot-agnostic |
| `test_plate_ellipse_offline.py`, `test_plate_detect_offline.py` | copied verbatim |
| `urdf_fk.py` | URDF engine copied **byte for byte**; every wrapper rewritten |
| `R1.urdf` | copied from `r1_description`, plus the three helmet frames |
| `r1_config.py`, `test_urdf_fk_offline.py` | new |

A test asserts the shared URDF engine is still byte-identical to the G1's, so a fix to
`find_chain` or `rpy_to_matrix` cannot land in one robot and silently not the other.

### Deliberately not ported yet

`move_arm.py`, `read_joint_angles.py`, `calibrate_camera.py`, `bimanual_pickup.py`,
`solve_ik.py`, `plan_approach.py`, `compute_wrist_target.py`, `pelvis_height.py`.

The first four need the R1's motor table and IDL, which are unverified. The last four hard-code
`"waist_yaw_joint"` as *the* joint above both shoulders, which is a G1 fact — the R1 has two.
That is a real edit in about a dozen places, not a find-and-replace, so they are absent rather
than present and half-right.

---

## The three things that actually differ

**1. Root link is `pelvis_link`, not `pelvis`.** A test asserts a chain rooted at `"pelvis"`
raises, so pasted G1 code fails loudly.

**2. Two trunk joints above both shoulders** — `waist_roll` then `waist_yaw`. Code that passes
only a waist yaw angle loses real motion: at 0.4 rad, waist roll moves the camera **187 mm**.
Hence every FK function takes an angles *dict*.

**3. The camera hangs off a 2-DoF neck.** On the G1 it was bolted to the torso, so one
calibrated constant held forever. Here `head_pitch` and `head_yaw` sit mid-chain, so their
measured zeros are part of the camera calibration — and a head-zero error is
indistinguishable from a mount error until you move the head and watch the plate appear to
move with it.

---

## Numbers established off-robot

**Sensor extrinsics**, from `helmet_geometry.py`, whose hard-coded head joint origins were
cross-checked against `R1.urdf` and agree to 0.0000 mm:

```
d435_joint    parent head_yaw_link   xyz 0.095000 0 0.050000   rpy 0 0.523599 0   (30° down)
jt128_joint   parent head_yaw_link   xyz 0.003835 0 0.140896   rpy 0 0.418879 0   (24° back)
```

Both pure pitch, no roll or yaw. **These are the calibration seed, not the truth.** On the G1
the URDF mount angle was wrong by 23° and put the plate 23 cm from where it was.

**Camera at the zero pose:** `[0.1225, 0, 0.4705]` m in `pelvis_link`, optical axis 30° below
horizontal, 90 mm forward of the shoulder.

**Head travel:** pitch is exactly ±36°, so the camera aims from **6° above horizontal to 66°
below**. The G1's fixed 47.6° is inside that range, so any G1 viewing geometry can be
reproduced. Yaw is ±115°.

**Arm:** shoulder→wrist is 0.2564 m at the zero pose, **0.3587 m** maximum over the joint
limits (a contorted pose, so usable reach is less). The dexterous hand adds an unmeasured
amount — see below.

---

## The standoff is set by the camera, not the arm

Assuming a 0.60 m walking pelvis height, the shoulder sits 0.85 m and the camera 1.07 m above
the floor. For a 0.75 m table:

| standoff | off-normal angle | minor/major | reachable? |
|---|---|---|---|
| 0.25 m | 38.0° | 0.788 | yes |
| 0.35 m | 47.5° | 0.675 | yes |
| 0.45 m | 54.5° | 0.580 | yes, with ≥0.12 m of hand |

Horizontal reach at table height is 0.345 m at the wrist, 0.47–0.53 m with a 0.12–0.18 m hand.
So the G1's `STAND_OFF = 0.45 m` is *reachable* — but it costs a 54.5° view, where a circle
fitted to the plate is out by 21%. **Stand closer than the G1 did**, around 0.25–0.35 m, and
the gain is perceptual, not mechanical.

Two things follow:

- The ellipse fit in `plate_detectors.py` is now a **requirement**, not an improvement. The R1's
  camera is lower than the G1's, so at any given standoff it sees *more* foreshortening, not
  less.
- Pointing the head down does **not** reduce foreshortening. The off-normal angle is fixed by
  where the robot stands relative to the plate. What the neck buys is being able to *see* a
  plate that is close in — and standing close is what reduces the angle. `aim_head_at()` is
  for framing, not for geometry.

---

## Finding the plate stack

`plate_stack.py` takes a depth frame plus an already-segmented table plane and reports, for
every stack standing on it, **which plate is on top and how high its rim is above the table**.
The plane comes from the segmentation side (table height plus the robot's stance), so nothing
here re-fits the table. `check_stack.py` is the read-only tool for looking at what it sees.

```python
import plate_stack as ps
stacks = ps.locate_stacks(depth, intrinsics, plane,
                          plate_diameter_m=0.225, plate_pitch_m=0.010,
                          plate_rim_height_m=0.018)
stacks[0]["top_height_m"]   # the top plate's RIM above the table
stacks[0]["center_m"]       # rim centre, 3-D, camera optical frame
stacks[0]["n_plates"]       # and how many are left
```

The plane may be `(n, c)` with `n . X = c`, a `[a, b, c, d]` vector, or a dict with `normal`
plus `c` / `d` / `point` — all in the camera optical frame, metres.

### Why the height is the number everything hangs on

Plates nest at about a centimetre. Get the top height wrong by one pitch and the hand closes
on the second plate, or on air above the first. So the height is measured as the **median over
the whole top rim** — tens of thousands of pixels, all at the same height by construction —
which averages the depth noise down by roughly sqrt(N). A D435's few millimetres per pixel
becomes a fraction of a millimetre over the annulus.

### The trap: a plate is not flat

A dinner plate's rim stands 10-20 mm proud of its well, so **the top plate alone produces two
height clusters**. Reading the well as the top surface is wrong by more than a plate pitch,
every frame, in the same direction. This module never looks for clusters generically: it takes
the topmost one, which is the rim, and the rim is also what a finger closes on.

### Measured envelope

Ray-marched stacks with exact ground truth, errors quoted against the 10 mm pitch, which is the
only tolerance that means anything here:

| varied | works | breaks |
|---|---|---|
| angle off the table normal | 15-55 deg (0.1-0.8 mm error) | refuses past ~60 deg, where the rim is too foreshortened to confirm |
| standoff | 0.25-0.70 m | — (but see clipping) |
| depth noise | to 8 mm sigma (0.4 mm error) | 15 mm sigma, far beyond a D435 |
| glaze dropout | to 97% of the well lost | — the rim carries the measurement |
| stack height | 1-20 plates | — |

Three failures the sweep found, all now handled:

- **Clipping.** A stack running off the frame edge has a cut outline, so the circle fits a
  chord. At a 0.25 m standoff that gave a 67 mm diameter error and was *accepted*, because
  67 mm squeezed inside the size gate. Now detected by name and refused, with `allow_clipped=True`
  to take the height anyway — a partial rim is still at the rim's height, and `diameter_reliable`
  says the rest is not. Two 225 mm stacks 0.28 m apart need ~0.6 m of standback to both fit.
- **Noise bias.** The top band was anchored on the 99th percentile, and a percentile of a noisy
  distribution sits ~2.3 sigma high: at 8 mm noise that was nearly two pitches, biased upward
  every frame. The band is now re-centred on its own median until it settles, which moved the
  breaking point from 8 mm to 15 mm.
- **Overconfident counting.** Two ledges are evenly spaced by definition, so the evenness test
  passed on them and a stack of ten was reported as two. Counting from **ledges** can only ever
  be a lower bound — from near the table normal the stack's side barely shows. Counting from the
  **height** is exact, because one plate is one pitch, so given `plate_pitch_m` and
  `plate_rim_height_m` that is what it uses, and `count_from` says which. Exact for 1-20 plates
  at both 25 and 45 degrees; without a rim height it returns `None` rather than undercounting.

### Testing it for real

Needs **no robot** — a RealSense, a table and a stack of plates.

```bash
python check_stack.py --synthetic --pitch 0.010 --rim-height 0.018   # no camera at all
python check_stack.py --capture lab01                                # save a frame
python check_stack.py lab01.npz --pitch 0.010 --rim-height 0.018 --png look.png
python check_stack.py lab01.npz --plane his_plane.json --pitch 0.010 # check his plane
```

Measure three things on the real crockery first: diameter, **nesting pitch** (stack ten,
measure, divide by nine) and a single plate's rim height. The pitch sets the whole tolerance
budget; the rim height is what makes the count exact.

Given `--plane` the tool *also* fits the table itself and compares the two, printing `agrees`
or `DISAGREES` — an independent check on the segmentation side before anything depends on it.

**The acceptance test:** run it, remove one plate, run it again. `top plate rim` must fall by
one pitch and nothing else. That exercises the whole chain and needs no robot, no calibration
and no nav stack.

`check_stack.py --png` writes a diagnostic image with no cv2 and no PIL: grey is missing depth,
blue the table, green-to-red rising, and a white ring on the rim it measured. Check the ring is
on the top plate and has not slipped down the stack — that failure is obvious in the picture and
invisible in the numbers.

## What is not known, and refuses to guess

| Unknown | Where it bites |
|---|---|
| **Dexterous hand geometry** | `r1_config.tool_offset()` raises. On the G1 this was a measured 0.2492 m, and ignoring it drove the *wrist* onto the rim — putting the hand 21 cm past it, through the plate and into the table. A plausible default here would reproduce that bug while looking right. Measure wrist-axis → rim-contact point and set `_TOOL_OFFSET`. |
| **Motor index map and IDL** | `r1_config.motor_index()` raises. The G1's 29-slot array with arms at 12–28 and the arm_sdk blend weight at slot 29 is a G1 fact. `R1.urdf` has 26 movable joints, but whether the SDK exposes them in that order is unverified. Get it from Unitree's R1 developer guide. |
| **Whether `rt/arm_sdk` exists on R1** | Unverified. The G1's `move_arm.py` publishes `unitree_hg` IDL to `rt/lowcmd` / `rt/arm_sdk`; do not assume either transfers. |
| **Camera calibration** | `T_pelvis_optical()` raises unless you pass `allow_nominal=True`. Set `T_HEADYAW_OPTICAL_CALIBRATED` once a calibration exists — note it hangs off `head_yaw_link`, not the torso, because that is what the camera is bolted to. |

---

## Robot coordinates

`check_stack.py` reports each stack's top-plate centre in **`pelvis_link`** — x forward,
y left, z up — whose origin sits directly above the floor point midway between the two ankle
pivots, ~0.71 m up when standing. `--json out.json` writes it for the manipulation side.

```bash
python check_mount.py --calibrate      # once, facing open floor: saves camera_calibration.json
python check_stack.py frame.npz        # now also prints the ROBOT FRAME block
```

Where the camera's pose comes from (`camera_calibration.py`):

| | from |
|---|---|
| position on the head, yaw | the helmet CAD |
| pitch, roll, height above the floor | a measured level surface |

The CAD's tilt alone is 4 deg out on this robot, ~44 mm at the plate — four nesting pitches —
so it is never used uncorrected. Tested end to end on a scene built in the robot frame and
seen through a camera mounted wrong like the real one: **3 mm** error.

**What a level surface cannot see is yaw**, and each degree of it is ~1 cm sideways at 0.6 m.
That includes the yaw hidden inside a roll: if the measured -2.3 deg of roll is the helmet
twisted about the camera's own (34 deg down) axis rather than the robot leaning, 1.3 deg of it
is really yaw. So check y once with a tape — stack on the robot's centreline, y should read ~0.

**The robot moving goes stale the calibration.** Every frame compares the table it sees with
the calibration and flags more than 1 deg of difference. `--level table` takes the table in
the current frame as level instead: always current, only as good as the table's levelness.
On the real frames the two agree to within ~1.5 cm sideways and <1 cm forward and in height,
which is the honest accuracy until the pelvis IMU can be read.

First real numbers (`test4`, 4 plates): top plate **59 cm in front, 2-4 cm left, 0.787 m above
the floor, on a table 0.745-0.747 m high.** The table height is a quick tape-measure check of
the whole vertical chain.

## On the robot: head down, live joint angles

At its rest angle the camera looks 33 deg down and sees the table from ~0.5 m out, past the
arm's reach. So the head tilts down, and every frame reads the head and waist angles off the
robot to place the camera. Run on the robot's computer (`eth10` is its 192.168.123.x port):

```bash
python3 r1_head.py eth10                          # read the head and waist (moves nothing)
python3 r1_head.py eth10 --down 30 --go           # tilt, hold until Enter, hand back
python3 check_mount.py --calibrate --robot eth10  # once, head at rest, facing open floor
python3 check_stack.py --live --robot eth10 --head-down 30 --json stack.json
```

- `r1_head.py` moves the head over `rt/arm_sdk` (blend weight in `mode_pr` on the R1), holding
  every other upper-body joint where it was measured. It ramps on and off over 1 s, returns the
  head before handing it back, freezes the head if the robot's reports pause, and lets go if
  the head does not follow. `check_stack.py --head-down` runs it in a process of its own, so
  depth processing cannot starve it.
- The helmet makes the head front-heavy: told 30 deg, it settles ~32.4 deg. That is accepted
  (anything within 5 deg, once still), because the real angle is what every frame uses.
- With the head at 30 deg, the whole plate is in view when the stack's centre is **32-49 cm**
  in front of the ankle midpoint. A clipped stack names the edge it runs off and which way
  to move it. For a stack placed to one side, `--head-left DEG` / `--head-right DEG` turn the
  head toward it (the robot's own left and right); positions stay right at any head angle.
- Photos: with `--json stack.json`, each `stack.json` comes with `stack.jpg` (the frame behind
  those numbers) and `stack_view.jpg` is the latest frame, accepted or not. Green ring =
  accepted, with its robot-frame x/y/z; red ring = rejected, with why; a red bar = the picture
  edge a stack runs off; yellow cross = the picture's centre.
- The calibration stores the head and waist angles it was taken at, so it holds at any head
  angle.
- Only one program may command `rt/arm_sdk`. Detect, Ctrl-C (the head goes back), *then* run
  the arm. The position stays valid while the feet do not move.
- On USB 2 (e.g. behind a USB 2 hub) the camera cannot run 848 x 480 with colour;
  `realsense_capture.py` steps down to a mode it can, and says so. Use a USB 3 port.

**Verified 2026-10-19,** head at 32.4 deg, 4 lab plates: top plate **36.2 cm forward,
3.3-3.4 cm right, 78.7 cm above the floor**, matching a tape measure on all three; diameter
255.2 mm against 254; the same numbers to 1 mm on a second run. The table read tipped 0.6 deg
front-back and 1.7 deg sideways in the robot's frame, of which ~1.1 deg is the waist standing
differently from calibration -- under 1 cm at the plate.

## The lean, head yaw, and calibrating over head poses (23 Oct)

The manipulation side measured three things the one-pose calibration gets wrong
(21-22 Oct, IMU log and a 38-pose table sweep):

1. **The pelvis leans** about 3 deg forward when standing. The calibration assumed it level,
   so the lean went into the camera. A tape check can't catch this, because the tape
   is in the floor's frame too. The arm works in the pelvis frame, so it is 3 deg off there.
   Now the lean is read off the robot (`floor_frame.py`: the IMU, cross-checked by the
   legs with both feet flat) and only what is left over is put in the mount. Tape numbers
   ("forward of the ankles, height off the floor") go through the same frame.
2. **The error grows with head yaw.** At yaw 0 the calibration is right to about 2 mm. The
   table's height came out off by -27.6 mm x sin(yaw), -8 mm at the push's 17 deg.
   One pose can't separate a tilt of the yaw axis from a tilt of the mount; turning
   the head does. `calibrate_sweep.py` fits both over 15 head poses and tests itself on
   poses it was not fitted to.
3. **Head yaw has play** (0.9 deg of roll, depending on the turn direction). `r1_head.py`
   now always arrives at a yaw moving left, by at least 4 deg, so the play is taken up the
   same way every time. **Anything else aiming this head must do the same.**

```bash
python3 r1_state.py eth10                                  # lean: IMU and legs should agree
python3 calibrate_sweep.py capture eth10 --surface table --table-height 74.5 --go
python3 calibrate_sweep.py check sweeps/<folder>           # the current calibration, per pose
python3 calibrate_sweep.py fit sweeps/<folder>             # fit + held-out test; --save to use
```

Every consumer goes through `camera_calibration.T_pelvis_optical(cal, joints, angles)`,
which applies the new yaw-axis term (`neck_tilt_rad`). Code that composes FK with
`T_headyaw_optical` by hand misses it. Old calibration files load unchanged. On a
simulated robot with the measured numbers (3 deg lean, 1.6 deg yaw-axis tilt, a mount off
by 1-1.5 deg), the old calibration put plates 65 mm off at the push pose. The sweep fit
puts them within 3 mm at every head pose up to 30 deg of yaw (`test_sweep_offline.py`).

## Running the tests

```bash
python run_tests.py
```

190 checks across 7 suites. `test_plate_detect_offline.py` skips itself without cv2, which is
expected on a laptop. No robot, no camera, no ROS.

`test_real_frames_offline.py` runs on frames captured by the R1's own head camera on the first
day the helmet was on (`real_frames/`, lab plates at 4, 3 and 2 high). They caught a bug the
synthetic scenes never did: a real rim's top band is a ring with gaps, so the outline traced
both edges of the ring and the plate read ~200 mm with a 35 mm fit residual. It now reads
249–255 mm against 254 mm by ruler.

## First on-robot results

- **Mount** (`check_mount.py --live`, facing open floor): camera 34.0 deg below horizontal
  against the CAD's 30.0, roll -2.3 deg, 1.18 m above the floor, repeatable to +/-0.2 deg.
  This is mount + head-zero offset + torso lean combined. It does not affect stack heights
  (those are measured against the table the camera sees); it matters only for the
  camera-to-robot transform, where the measured value is used rather than the CAD's.
  An earlier run read 40 deg because the floor fit had grabbed a wall; the check now only
  accepts level surfaces.
- **Plates:** 10-inch (254 mm) lab plates, nesting pitch ~9 mm, a single plate's rim 15.6 mm
  above the table. These are `check_stack.py`'s defaults; different crockery needs measuring.

`python urdf_fk.py` and `python r1_config.py` each run their own self-check and are the
quickest smoke test.

---

## Next

1. Port `solve_ik.py` + `plan_approach.py` + `compute_wrist_target.py` to two trunk joints,
   reading names from `r1_config`.
2. Measure the dexterous hand; set `_TOOL_OFFSET`.
3. Get the R1 motor table; port `read_joint_angles.py` read-only and confirm against live
   joint angles before anything is commanded.
4. Port `calibrate_camera.py` to solve `head_yaw_link → optical`, seeded from the CAD nominal.
5. JT128 bring-up. It is 360° × 189° on the head, so it sees the robot's own shoulders, arms
   and the helmet hoop continuously — URDF-based self-filtering is mandatory, not an
   optimisation. `helmet_geometry.hoop_shadow()` already quantifies the hoop's share.

Open question for the banquet task: a ~0.5 m reach from a table edge covers the outer annulus
of a 1.5–1.8 m round table. **Measure where plates actually sit** before assuming the centre is
serviceable.
