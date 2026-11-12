---
id: appendices
---

## Appendices

### Appendix A: Risk Assessment

> TODO: project risk evaluation and activity risk assessments, reference RA/CDE4301/IS-407/2026.

### Appendix B: Bill of Materials

Table: tab:bom | Main hardware
| Item | Quantity | Approximate cost (S$) |
| --- | --- | --- |
| Unitree R1 humanoid robot | 1 | 40,000 |
| BrainCo Revo 2 dexterous hand | 2 | 40,000 (pair) |
| Hesai JT128 lidar | 1 | 1,000 |
| Intel RealSense D435if depth camera | 1 |  |
| Sensor helmet (3D-printed PETG parts, inserts, screws) | 1 set |  |

> TODO: confirm the costs and fill in the camera and helmet; the
> helmet's full parts list is in the repository's hardware folder.

### Appendix C: Additional Test Data

> TODO: raw data and extra plots that support Sections 5.4, 7.7 and 8.6.

### Appendix D: Source Code

All code is in this project's [GitHub
repository](https://github.com/cyanhere/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup).

### Appendix E: Stakeholder Inventory

> TODO: add the plate list.

### Appendix 5A: Camera Model and Calibration

Each CCTV camera is modelled as a pinhole camera with radial lens distortion. A point X in the venue
frame is moved into the camera frame as x = R(X − C), where C is the camera's position and R its
orientation. Its normalised image coordinates (x₁/x₃, x₂/x₃), at radius r from the image centre, are
bent by the lens to r(1 + k₁r² + k₂r⁴) and scaled by the focal length f, in pixels, about the image
centre. Seven unknowns are solved once per camera: f, k₁, three rotation angles and the three
coordinates of C (k₂ stays at zero unless the references can support it). Past the radius at which
r(1 + k₁r² + k₂r⁴) stops increasing, the model folds back on itself, so points there are treated as
outside the view.

The unknowns are fitted by least squares to the references in Table [[tab:calib-residuals]].

Table: tab:calib-residuals | Calibration references and their residuals
| Reference | What is known | Residual |
| --- | --- | --- |
| Floor-plan point (a column's foot, a door frame) | Its pixel and its 3-D position | Reprojection error, in pixels |
| Known edge | One pixel anywhere on an edge whose position on the plan is known | Distance from the pixel to the projected edge, in pixels |
| Plumb edge, found automatically | Two pixels on an edge that is vertical in the room; its position is unknown | The plane through the lens centre and both pixels must be vertical, which fixes pitch, roll and distortion |
| Tabletop | Its four corner pixels | On the plane of the tops, opposite sides and the two diagonals must be equal (1% misfit counts as about 3 px); the tabletop height is solved as an eighth unknown |
| Soft priors | Camera 2.2–6 m high; k₁ between −0.6 and +0.05; tabletops 0.75 ± 0.15 m high | Penalties that keep a loosely constrained fit physical |

The solver (SciPy least_squares) uses a soft-L1 robust loss with an 8-pixel scale, so a single bad
reference cannot drag the fit. On a wide lens, focal length and distortion trade off against each
other: started from the wrong field of view, the fit bends the lens the wrong way (one start from a
100° guess converged to k₁ = +0.6 at 25 px RMS). The solver is therefore started from horizontal
fields of view of 55°, 70°, 85°, 100° and 110°, each with k₁ = −0.25 and −0.05, and the lowest-cost
result is kept. For the Tinkering Corner camera this gave a height of 2.57 m, k₁ = −0.19 and a
13-pixel residual in a 2,560-pixel-wide frame. A calibration is accepted only when the floor plan's
walls and pillars, projected back onto the frame, sit on the walls and pillars in the picture
(Figure [[fig:calibration-check]]).

### Appendix 5B: From Pixels to Table Positions

**Ray–plane intersection.** A pixel is undistorted to normalised coordinates (xₙ, yₙ), giving a ray
from the camera centre C with direction d = Rᵀ[xₙ, yₙ, 1]ᵀ. The ray meets the horizontal plane at
height h at P = C + s·d, where s = (h − C_z)/d_z; rays with s ≤ 0 point above the plane and are
discarded. Table outlines are intersected with the tabletop plane (h = 0.762 m), not the floor.

**Why the height matters.** Mapping a tabletop as if it were on the floor scales its position about
the nadir (the floor point below the camera) by H/(H − h) for a camera at height H. For a table 10 m
from the nadir, that puts it 2.35 m out with a 4 m camera and 1.05 m out with an 8 m camera, and
makes a 1.83 m tabletop read 43 cm and 19 cm too long. Intersecting with the tabletop plane removes
the error exactly: in the simulated hall, offsets of 223 mm and 329 mm fell to 0 mm.

**Far-edge envelope.** Every outline pixel is put on the tabletop plane. The tabletop's own edge
lands in its true place, but anything lower (legs, a skirt, a chair in front) lands short, between
its true position and the nadir, because its ray reaches tabletop height first. Seen from the nadir,
the farthest point in each direction is therefore the tabletop's edge. The outline points are sorted
by bearing around the nadir into 0.3° bins, and only the farthest point in each bin is kept. A bin
whose farthest point lies on the image border (within 2 px) is dropped: there the picture ends, not
the table.

**Shape fit.** A rectangle and a circle are both fitted to the envelope:

- Rectangle: the dominant edge direction comes from a length-weighted histogram of the envelope's
  segment directions (modulo 90°) and is squared to the room's walls when within 12° of them; the
  rectangle spans the envelope along that direction and its perpendicular.
- Circle: an algebraic least-squares (Kåsa) fit.

The table is taken as round only if the circle's radius is 0.3–1.3 m, the far points cover more than
120° of arc, the mean circle error is under 35 mm and under half the rectangle's, and the envelope
is close to square (shorter side over 0.7 of the longer); otherwise it is a rectangle.

**Two cameras.** Two real tables cannot overlap, so overlapping footprints are one table seen twice;
across cameras a larger overlap is required, because the two halves of a back-to-back row are often
each seen best from their own side and only just touch. The sighting kept is the one not cut off by
the frame edge, then the one with the lower range-to-confidence ratio. A sighting is rejected when
one pixel of outline spans more than 0.07 m of floor, where the camera is too far away to place the
table reliably.

### Appendix 5C: Planner Parameters

Table: tab:planner-params | Detection, snapping and export parameters
| Stage | Parameter | Value | Reason |
| --- | --- | --- | --- |
| Clean plate | Frames in the median | 31, one every 2 s | About a minute: long enough for people to move through, short enough for a moved table to appear |
| Detection | Models | Grounding DINO (base) and SAM (ViT-H), on the frame resized to 1,280 px wide | Text prompts, so a new venue needs a new prompt rather than training data |
| Detection | Box and text thresholds | 0.22 and 0.20 | A row of dark tables scored 0.22–0.28 and was missed at the usual 0.30 |
| Detection | Re-run | Every 15 s, only when the clean plate has changed | Tables move a few times an hour; one frame takes a GPU a few seconds |
| Mask clean-up | Duplicates | Masks overlapping by more than 0.5 IoU: the higher score is kept | Two boxes on one table |
| Mask clean-up | Group boxes | A mask holding two or more smaller masks of its class (each over 70% inside it) is removed, keeping only the parts they do not explain | One box around a whole row of tables (Section 5.5) |
| Mask clean-up | Chairs | Chair masks are cut out of table masks | Keeps each outline the table's own |
| Snapping | Stock | 5, 5.5, 6 and 7 ft rounds (8, 9, 10 and 12 covers); 6 × 2.5 ft oblong; 6 × 1.5 ft IBM | Standard banquet stock |
| Snapping | Tolerance | Nearest stock size, unless the diameter or length differs by more than 15% (width by 20%); then the measured size, rounded to 5 cm, is kept | A table outside the stock keeps its own size |
| Settings | Covers and stops | About 0.6 m of rim per cover; robot stop 0.45 m from the table edge | The trade rule gives 8, 10 and 12 covers for 5, 6 and 7 ft rounds |
| Settings | Robot envelope (assumed) | Radius 0.30 m; minimum aisle 0.90 m | Used by the reachability checks; to be measured on the R1 |
| Live mode | Updates | Polled every 1.5 s; a table missing for three updates is removed; a size change needs three agreeing updates; detections under 50% confidence are ignored | One missed frame does not make a table blink |
| Export | Map | .pgm at 0.05 m per pixel: 0 occupied, 254 free, 205 unknown; obstacles not inflated | ROS map_server convention; the costmap adds the robot's radius |
| Export | Goals | One named goal per setting (e.g. T07/cover03) in the map frame: X = x₀ + x, Y = y₀ + (H_map − y), θ = −θ_planner | The planner's y axis points down and the map's up; without the flip the robot drives to the mirror image of every goal |

### Appendix 6A: Sensor Coverage Geometry

Heights are from the floor with the R1 standing and its head at zero pitch and yaw; horizontal
distances are from the neck's yaw axis.

**LiDAR.** The JT128's beams span 4.4° below to 90.5° above its own horizon, measured from its
optical centre, which sits 1.315 m above the floor with the LiDAR tilted forward by τ = 24°. Its
lowest beam therefore points 4.4° + τ = 28.4° below horizontal ahead of the robot, and 4.4° − τ,
that is 19.6° above horizontal, behind it. Ahead, it meets a horizontal surface at height z at a
distance d = (1.315 − z)/tan 28.4° + x₀, where x₀ ≈ 0.03 m is the optical centre's offset ahead of
the neck axis: 2.46 m for the floor and 1.05 m for a 0.762 m tabletop.

**Choosing the tilt.** The tilt came from a search from 25° down to 15° in 1° steps, taking the
steepest tilt at which every point of the helmet and camera lies at least 2° below the lowest beam,
with the LiDAR seat placed so that the payload's centre of mass stays within 8 mm of the head's
pitch axis. The search was run with a provisional LiDAR model (lowest channel at −9°), before the
JT128's manual was obtained. With the real optics the helmet sits well clear of the beams (Table
[[tab:lidar-clearance]]), so the 24° tilt was kept rather than moving the seat and the crest.

Table: tab:lidar-clearance | Highest point of each part relative to the JT128's horizon (lowest beam at −4.4°)
| Part | Elevation from the optical centre |
| --- | --- |
| P6 brow bumper | −21.1° |
| P5 camera cradle | −24.6° |
| P1 crest | −24.7° |
| D435if camera | −25.2° |
| P7 plug cradle | −48.3° |
| P2 / P3 side pieces | −54.7° |

**Depth camera.** The D435if's depth image spans 87° × 58°. Its optical axis points 30° below
horizontal from 1.21 m above the floor, 0.095 m ahead of the neck axis, so the image's lower edge
points 30° + 29° = 59° down and its upper edge 1° down. The lower edge meets a surface at height z
at d = (1.21 − z)/tan 59° + 0.095: about 0.82 m for the floor and 0.37 m for a 0.762 m tabletop.

**Final approach.** The G1 tests used a camera pitched 63.6° down for the table-edge alignment
(Section 7.3.4). On the R1, the 30° mount plus a head pitch of 33.6° gives the same view, within the
joint's ±36° range.

**Mount check.** On the first on-robot check (October 2026), the camera measured 34.0° below
horizontal against the CAD's 30.0°, with −2.3° of roll and a height of 1.18 m, repeatable to ±0.2°.
The difference has not yet been split into mounting error, head-zero offset and torso lean, so the
measured pose, not the CAD value, should be used for the camera-to-robot transform.

### Appendix 6B: Clamp and Skull-Screw Analysis

**Load path.** The helmet is held by two ISO 4762 M3 × 16 screws, with washers, into existing
threaded holes in the R1's skull 42 mm either side of the centreline, and by a rear clamp: the side
pieces close on the head through ear-ring pads preloaded by 0.5 mm, and a thumbscrew draws their
flanges together across a 1.0 mm design gap. The screws alone hold the helmet; the clamp is kept on
purpose as a second load path.

**Finite-element model.** A contact model of the rear clamp (gmsh mesh, scikit-fem solver) treats
the PETG parts as linear elastic (E = 1.5 GPa), the TPU pads as solid and the skull screw as a fixed
support. Table [[tab:clamp-fe]] gives the loads with the flanges touching. The results move by up to
±25% with mesh refinement, so they are approximate.

Table: tab:clamp-fe | Rear-clamp loads from the FE model (±25%)
| Quantity | Value |
| --- | --- |
| Thumbscrew tension, flanges touching | 29 N |
| Ear-cup force | 27 N (19 N with the thumbscrew loose) |
| Rear-pad force | 14 N |
| Skull-screw load | 36 N |
| Peak stress (crown arm, front edge) | 7.0 MPa |
| Clamp stiffness per mm of flange closure | 20.9 N/mm |
| Ear-cup force per 0.1 mm of pad thickness | About 4 N |

**Why the screws were added.** The first, clamp-only design slid backwards off the head. Its FE
model showed the rear pads carrying 44 N of the 52 N clamp force and the ear cups only 13 N, so the
ear cups, which locate the helmet on the head, were barely loaded. Screwing the side pieces to the
skull and preloading the ear pads moved the load onto the ear cups (27 N) and off the rear pads (14
N).

**Other checks in the CAD build.** No interference among the 2,080 pairs of parts, pads, sensors and
fasteners; thread engagement within limits for all 21 threaded fasteners (4.0 mm for the JT128's M3
screws, against the manual's 3–5 mm); no vertex of the R1's head mesh inside a structural part; the
side pieces swing 15° on their hinges without touching the crest or the head; and no part inside the
camera's depth frustum.

### Appendix 6C: Helmet Parts and Print Settings

Table: tab:helmet-parts | Helmet parts for the fit-check build (final material in brackets where it differs)
| ID | Part | Material | Print settings | Filament |
| --- | --- | --- | --- | --- |
| P1 | Crest (spine) | PLA (ASA) | 4 walls, 30% gyroid, 0.2 mm layers; tree supports | 106 g |
| P2, P3 | Side pieces L and R | PETG | 4 walls, 40% gyroid, 0.2 mm layers; tree supports | 46 g, 47 g |
| P4 | LiDAR seat plate | PETG with M3 heat-set inserts (CNC 6061 aluminium) | 100% infill, 0.2 mm layers | 26 g |
| P4b | LiDAR adapter plate | PETG (CNC 6061 aluminium) | 100% infill, 0.2 mm layers | 18 g |
| P5 | Camera cradle | PETG (CNC 6061 aluminium) | 100% infill, 0.2 mm layers | 19 g |
| P6 | Brow bumper | TPU 95A | 3 walls, 25% gyroid, slow | 18 g |
| P7, P7b | Plug cradle; cable clip | PETG; TPU 95A | 4 walls, 40% gyroid; 3 walls, 25% gyroid | 6 g, 4 g |
| P8 | Crown, forehead, ear-ring and rear pads | TPU 95A | 100% infill, slow | 1–3 g each |
| P9 | Roll hoop | PETG (Ø6 steel rod bent to R42) | 100% infill, brim | 9 g |

The side pieces are PETG even for the fit-check: they flex to produce the clamp force, which assumes
E ≈ 1.5 GPa, and PLA, roughly twice as stiff, would clamp about twice as hard and could crack under
the preload. The crest is PLA for the fit-check and is reprinted in ASA for use, as it sits under
the LiDAR's 7.5–10 W. An aluminium LiDAR adapter also gives the grounded metal mount that the
LiDAR's manufacturer recommends. In all, the build takes about 110 g of PLA, 171 g of PETG and 25 g
of TPU.

### Appendix 6D: Cable Take-Up Sizing

**How much cable must be stored.** The span from the cable clip on the rear clamp (P7b) to the slot
in the box's top lengthens and shortens as the head moves. Its length was computed from the R1's
meshes over the head's pitch range (±36°) and a range of yaw limits (Table [[tab:takeup-yaw]]). Up
to about ±90° of yaw the span is a straight line; beyond that the cables lie on the neck and wrap
round it.

Table: tab:takeup-yaw | Cable to store and clearance to the neck, by head-yaw limit (pitch ±36°)
| Head-yaw limit | Cable to store | Gap to neck / torso |
| --- | --- | --- |
| ±60° | 193 mm | 26.9 / 25.9 mm |
| ±70° | 201 mm | 18.6 / 18.5 mm |
| ±80° | 209 mm | 10.5 / 11.3 mm |
| ±90° | 216 mm | 2.0 / 4.7 mm |
| ±115° (joint limit) | 229 mm | 0.4 / 0.9 mm |

**The 2:1 loop.** Both cables run down round the carriage's pulley and back up over a fixed former,
so each millimetre of carriage travel stores 2 mm of cable. The carriage travels 117.4 mm, storing
235 mm. At ±80° the take-up needs 209 mm, leaving about 6 mm of carriage travel at each end, with
the cables at least 10.5 mm from the neck and torso; ±80° is therefore the head-yaw range the box
supports (the joint itself reaches ±115°). The pulley's 28 mm pitch radius is five times the JT128
cable's 5.5 mm diameter, its minimum bend radius.

**Tension.** In the current design a constant-force spring of about 4 N pulls the carriage down,
adding to the carriage's own weight of about 1.2 N. Shared by the four cable strands round the
pulley, that is about 1.3 N per strand, so the two strands to the head pull about 2.6 N in all: at
most 0.5 N·m about the neck, under 2% of the head joints' 33 N·m (R1 URDF). The spring is still an
open choice; without it, the carriage's weight alone would give about 0.3 N per strand, which has
not been tested.

**Clearances.** Checked against the R1's meshes and joint ranges, with the box moving with the
waist: the head and helmet stay at least 14.0 mm from the box over the full neck range, the pelvis
and hips at least 28.4 mm over the waist's roll (±30°) and yaw (±150°), and of 6,384 poses per arm,
none touches the box without also hitting the torso. These are CAD checks; the box has not yet been
built or fitted.

### Appendix 7A: The Decay Calculation

> TODO: Per cell, once per 2.0 s window, with evidence e capped at 10:

> TODO: hit at least once in the window   ->  e <- min(e + 1, 10)

> TODO: swept by a beam but not hit       ->  e <- e - 1      (charged every 3rd window)

> TODO: neither                           ->  e unchanged

> TODO: e >= 2  ->  published as an obstacle

> TODO: e  = 0  ->  removed

> TODO: A beam is sampled from the robot's current position to one
> cell short of its return, up to a range of 15 m, and a sample
> contributes only where its interpolated height lies within 0.20–1.60
> m. Occlusion tolerance is cap × decayevery × window = 10 × 3 × 2 s =
> 60 s. During the first 20 s after startup, penalties are suppressed.

> TODO: The 15 m sampling range was chosen by measurement: extending it
> from 5 m to 15 m multiplied the free-space evidence by 3.4, while 15
> m to 25 m added only 1.26 times more, because beams already terminate
> on walls before reaching that far. An alternative scheme that simply
> discarded any cell unobserved for a fixed time was implemented and
> rejected: it removed 1,085 cells that no beam could physically reach,
> erasing exactly the occluded regions the three-state rule exists to
> preserve.

> TODO: Three refinements make the clearing rule behave:

> TODO: Beams stop one cell short of what they hit. Nothing beyond a
> return is ever counted as empty, so a beam striking a wall cannot
> clear the room behind it.

> TODO: The height band is checked along the beam, not only at its end.
> The beam's height is interpolated over its length, and only the
> portion within 0.20–1.60 m contributes clearing. Without this, a beam
> passing above a table and striking the far wall would clear the
> table's own cells on the way through.

> TODO: Evidence accumulates over a 2 s window rather than per scan.
> The lidar returns about 1,240 points per scan against 14,400 cells in
> a 6 m area, so no single scan can touch most of them; charging a
> penalty per scan erodes real walls faster than they are re-observed.
> Counter-intuitively, a finer grid makes this worse, not better.

### Appendix 7B: FAST-LIO2 Mechanisms

> TODO: Backward propagation. A lidar sweep takes about 100 ms, during
> which the robot moves and turns, so each point in the sweep is
> captured from a different pose. Treating the points as simultaneous
> bends straight surfaces. The IMU's high-rate measurements are used to
> place each point at the pose it was actually taken from,
> straightening the sweep before it is used.

> TODO: The iterated EKF. The IMU propagates the state forward between
> sweeps. Each de-skewed point is then matched to the map, and its
> point-to-plane residual updates the state. The filter re-linearises
> around the improved estimate and repeats, which behaves more like a
> Gauss–Newton optimisation than a single EKF update. Crucially, the
> update corrects the IMU biases as well as the pose, which stops the
> inertial error from compounding.

### Appendix 7C: Localiser Correction and Drift Test Details

> TODO: Accumulator gate. Scan insertion is gated on the relocaliser's inlier ratio:

Table: tab:accumulator-gate | Accumulator gate by relocaliser inlier ratio
| Inlier ratio | What it means | Scan inserted? |
| --- | --- | --- |
| Below 0.15 | No prior map exists yet | Yes, on odometry alone, to bootstrap the map |
| 0.15 to 0.50 | The pose is ambiguous | No |
| Above 0.50 | The pose is trusted | Yes |

> TODO: Without this gate, a poor pose would be written into the map
> and then used to validate itself.

> TODO: Relocaliser settings. Each cycle accumulates five scans,
> downsampled to at most 8,000 points, and runs 15 ICP iterations with
> a coarse-to-fine correspondence gate of 1.5 → 0.8 → 0.4 m, because a
> single narrow gate cannot recover once the estimate is poor.

> TODO: Drift test procedure. Each condition walked five 5 m
> out-and-back laps under Nav2, about 58 m per run, starting from the
> same floor mark, and the final position was measured against that
> mark with a tape. The odometry-only condition was produced by
> disabling the relocaliser's ICP (seedonly mode), so both runs share
> an identical map frame and differ only in whether the correction was
> applied. Laps are repeated because the navigation goal tolerance is
> 0.30 m: a single arrival may be that far off for reasons unrelated to
> drift. Arrival error does not accumulate across arrivals, whereas
> drift does, so over 58 m drift becomes the dominant term.

> TODO: Independent check. The cumulative correction applied by the
> relocaliser was consistent across two runs, at 4.0% and 4.3% of
> distance walked, while the net shift of the map origin was under 1.5
> cm. The relocaliser was therefore not displacing the frame, but
> continuously cancelling an error that re-accumulated at about 4% of
> distance. For comparison, published evaluations report 6.8%
> translation drift for FAST-LIO2 [TODO: citation].

### Appendix 7D: Detection Confidence

> TODO: A detected table or AGV position overwrites the rough map's
> only once the mission planner is confident in it. [TODO: what
> "confident" means, e.g. the same position seen in N consecutive
> scans, within X cm.]

### Appendix 8A: Rim Tracker Method

The tracker (`deploy/pick/pk/tracker.py`) was developed for this project. It starts from the stack
pose found by stack localisation and updates the top plate's pose on every depth frame:

- Depth pixels within a window around the last plate estimate are converted to 3-D points in the
  robot frame using the calibrated camera pose.
- Points belonging to the robot are removed: the arm, hand and fingertips are placed by forward
  kinematics from the measured joint angles and covered by spheres, and points inside the spheres
  are discarded.
- Points within a height band around the rim are kept. The band is asymmetric, from 4 mm below to 25
  mm above the expected rim height, so that a rising rim is not lost.
- Around the plate, in 72 sectors of 5°, the rim's outer edge is taken as a high percentile of the
  points' radius from the current centre, which rejects isolated outliers. Sectors near the robot's
  hand are excluded.
- A circle of known radius (the rim radius measured when tracking starts) is fitted to the edge
  points with a robust (Huber) loss, giving the plate centre. At least 12 of the 72 sectors must be
  valid.
- A plane is fitted to the rim heights of the valid sectors, giving the plate's tilt and the height
  of the rim at any angle.

On synthetic depth data with a 0–45 mm slide and a hand in view, the worst position error was 0.4
mm; processing took 25–35 ms per frame on a laptop. On a stationary plate on the robot, position
noise was 0.7 mm.

### Appendix 8B: Estimated Rim Breaking Load

No published breaking load for the rim of a porcelain plate was found; tableware standards address
impact rather than static load (ASTM C368). An estimate was therefore made from the flexural
strength of porcelain, typically 60–80 MPa for conventional porcelain [19], treating the rim as a
cantilever loaded at its edge by a finger:

F = σ b t² / (6 L)

where σ is the flexural strength, t = 5.5 mm the rim thickness, L = 34.5 mm the distance from the
edge to where the rim meets the plate's slope (radius 127.5 mm to 93 mm), and b the width of rim
that carries the load.

Table: tab:rim-load | Estimated rim breaking load and safety margin
| Load width b | Breaking load F (σ = 60–80 MPa) | Margin against 60 N peak | Margin against 30 N sustained |
| --- | --- | --- | --- |
| 14 mm (fingertip width; conservative) | 120–165 N | 2.0–2.7 | 4.1–5.5 |
| 69 mm (load spread over twice the rim length) | 600–810 N | 10–13 | 20–27 |

The conservative case gives a margin of about 2 at the peak limit. The estimate ignores the rim's
taper and curvature, stress concentration at chips or glaze flaws, and the support from the plate
below during the push, and some porcelains are weaker (about 40 MPa, which gives 86 N). The limits
are therefore treated as provisional, and a static rim-load test is planned: a fingertip-sized pad
pressed on the rim of a spare plate, with the load recorded until the rim chips. In use, the
measured forces are well below the limits during the push (7–9 N downward in a hand-guided
index–middle push on a scale, with the Revo 2 hand held by a person), while the grasp and lift
approach them: in the simulation evaluation, 4 of 24 conditions exceeded the limits (Appendix 8E).

### Appendix 8C: Arm Model Identification

All data were recorded on the R1 with the robot clear of the table and no plates: 30 held poses;
sweeps of each joint alone at 0.01–0.4 rad/s, back-and-forth motions of 5–50 mrad and steps; poses
with a 0.55 kg load on the wrist; and six random multi-joint motions (about 280 s), which were held
out of every fit and used only for scoring.

- **Gravity.** The torque each joint needs against gravity is linear in the mass and mass moment of
  each arm segment. These parameters were fitted to the held poses by least squares, pulled toward
  the CAD (URDF) values, with a constant and a small spring term per joint. The fitted segment
  masses are 10–25% below the URDF values; the held-out torque error fell from 0.07–0.57 N·m (URDF)
  to 0.08–0.26 N·m.
- **Friction.** In the joint sweeps, steady friction was highest at 0.01–0.04 rad/s and 15–30%
  lower above 0.15 rad/s. In the held-out random motion, friction at low speed was much lower
  (0.14–0.26 N·m against 0.6–0.8 N·m), because low speed there occurs just after a change of
  direction, before friction has built up. A friction model that depends only on speed cannot
  represent this, so the LuGre model was used [29]. It gives each joint an internal state z, which
  follows the motion and saturates: dz/dt = v − σ0 |v| z / g(v), with friction F = σ0 z and
  steady-state friction g(v) = Fc + Fs exp(−(v/vs)²). After a reversal the friction builds up over
  a few milliradians of travel. It predicted held-out friction torque better than constant or
  speed-only friction on all ten joints (e.g. left shoulder pitch 0.39 against 0.50 and 0.62 N·m).
- **Latency and gain.** The motor torque expected from the PD law (computed from the commanded and
  measured joint angles) was compared with the torque reported by the motors; the time shift that
  best aligned them was 25 ms on both arms, with a torque gain of 0.94–1.02 (0.86 on the right
  wrist).

In an open-loop MuJoCo replay of the held-out motions with the logged commands, the model tracked
the R1 to 4–12 mrad per joint (20 mrad on the right shoulder roll), within the robot's own lag
behind its plan (13–25 mrad).

### Appendix 8D: Fingertip Friction Identification

The left arm's logged commands from ten R1 pushes were replayed in MuJoCo through the identified arm
model. Each replay was registered to the robot run using the stack height from the first confirmed
fingertip touch and the stack's lateral position from the touches before the push. The fingertip
friction was varied from 0.25 to 2.0, the plate-on- plate friction over 0.45, 0.575 and 0.7, and the
stack position over three offsets. Two observations were compared with the head camera's plate
track: the drag of the plate while the fingertip slid to the rim edge (1.2–5.3 mm on the robot), and
the push distance. MuJoCo produced no drag below a fingertip friction of 0.5; the observed drag and
push distance were reproduced at 1.0 with plate-on-plate friction 0.45, at 1.0–1.3 with 0.575, and
at 1.3–1.6 with 0.7. These are effective values: errors in the arm model's force are included in
them.

### Appendix 8E: Validation Conditions in Simulation

Each candidate motion was evaluated in MuJoCo over a full factorial set of conditions: lateral
camera error of −15, 0 and +15 mm; encoder zero error of 0 or 0.4° per joint; arm joint friction of
0.6 or 1.0 times the model value; and table heights of 0.747 and 0.762 m. At the nominal stance, the
hooked push succeeded in 20 of 24 conditions, the four failures being fingertip slips at full joint
friction. The complete pick sequence lifted the plate in 24 of 24 conditions, of which 20 remained
within the force limits. These evaluations used the initial fingertip friction of 0.25 and predate
the identified arm model, so they will be repeated with the identified parameters.
