---
id: appendices
---

## Appendices

### Appendix A: Risk Assessment

The activity risk assessment for this project is registered as RA/CDE4301/IS-407/2026. Table
[[tab:project-risks]] lists the risks to the project's outcome and schedule, and Table
[[tab:activity-risks]] summarises the hazards of the work itself and how they are controlled.

Table: tab:project-risks | Project risks, rated Low, Medium or High
| Risk | Likelihood | Impact | Mitigation |
| --- | --- | --- | --- |
| Porting navigation from the G1 to the R1 (different LiDAR and ROS setup) takes longer than planned | Medium | High | Port at the start of Semester 2, then repeat the drift and approach tests on the R1 (Table [[tab:open-navigation]]) |
| The AGV specification is not received from PARKROYAL in time | Medium | High | Request it early and prepare the fine-approach options meanwhile (Appendix 7E) |
| Plate lifting on the R1 does not become consistent | High | High | Continue fitting the simulation to the robot; keep the scripted state machine as a fallback (Chapter 8) |
| Little access to the banquet hall for testing | Medium | Medium | Rehearse with a mock layout in the Innovation & Design Hub; book hall sessions early |
| The robot or a hand is damaged during testing | Low | High | Low speeds, joint and force limits, and a spotter at every run |
| Subsystem integration slips past the schedule | Medium | High | Integrate in two stages with fixed milestones (Section 10.2) |
| The CCTV rough map and the robot's own map are not aligned | Medium | Medium | Localise the robot at the start of each run; detected tables overwrite the rough map (Section 7.2.2.1) |

Table: tab:activity-risks | Activity risks and their controls; residual risk is the level after the controls
| Activity | Hazard | Controls | Residual risk |
| --- | --- | --- | --- |
| Navigation tests with the robot walking | Robot collides with or falls onto a person | Test area marked and cleared; spotter within reach of the emergency stop; people kept at least 1 m away; low walking speed | Medium |
| Manipulation tests with the arms and hands | Pinching or striking by the arm or fingers | Stay outside the arm's reach while it is powered; finger force limits (T13); emergency stop within reach | Medium |
| Lifting and moving the robot (about 29 kg) | Back strain; dropping the robot | Two-person lift with the robot powered off | Low |
| Charging and storing robot batteries | Battery fire | Manufacturer's charger only; charge on a non-flammable surface under supervision; check for swelling; extinguisher nearby | Low |
| Handling porcelain plates | Cuts from broken plates | Trials over a padded surface; gloves and a dustpan for breakage; broken pieces in a separate bin | Low |
| 3D printing the helmet parts | Burns from the nozzle and bed; fumes | Printer used in a ventilated room; parts left to cool before removal | Low |
| Fitting heat-set inserts | Burns from the soldering iron | Iron kept in its stand on a heat-resistant mat; switched off after use | Low |
| Running the LiDAR and depth camera | Laser exposure to the eyes | Both are Class 1 eye-safe devices [12], [32]; do not stare into the emitters at close range | Low |
| Cables and floor tape in the test area | Tripping | Cables routed along walls and taped down; walking path kept clear | Low |
| Wiring the sensor helmet and cable box | Electric shock or short circuit | Work with the power off; insulated tools; fused supply | Low |

### Appendix B: Bill of Materials

Table: tab:bom | Main hardware
| Item | Quantity | Approximate cost (S$) |
| --- | --- | --- |
| Unitree R1 humanoid robot | 1 | 40,000 |
| BrainCo Revo 2 dexterous hand | 2 | 40,000 (pair) |
| Hesai JT128 LiDAR | 1 | 1,000 |
| Intel RealSense D435if depth camera | 1 | ≈ 730 [33] |
| Sensor helmet (3D-printed PETG parts, inserts, screws) | 1 set | 35 |
| Cable management box (USB hub, cables, printed parts) | 1 | 40 |
| Test plates, 10.5-inch porcelain (255 mm, 522 g) | [TBD] | 20 |
| Total |  | ≈ 81,825 |

### Appendix C: Source Code

All code is in this project's [GitHub
repository](https://github.com/cyanhere/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup).

### Appendix D: Stakeholder Inventory

Table [[tab:inventory]] lists the tableware the hotel uses for one banquet, from its tableware
specification.

Table: tab:inventory | Tableware inventory for one banquet
| **No.** | **Item** | **Quantity per banquet (pieces)** |
| --- | --- | --- |
| 1 | Dun zhong cover, bone china | 100 |
| 2 | Dun zhong body, bone china | 100 |
| 3 | Dun zhong stove, bone china | 100 |
| 4 | 4.5" soup tureen cover, porcelain | 1,500 |
| 5 | 4.5" soup tureen body, porcelain | 1,500 |
| 6 | 4.5" soup tureen saucer, porcelain | 1,500 |
| 7 | 10.5" round plate, spiral pattern | 3,000 |
| 8 | 8.25" cover, porcelain | 100 |
| 9 | 8.25" round dish, porcelain | 200 |
| 10 | 8.25" fan-shaped plate, porcelain | 600 |
| 11 | 8.25" 5-compartment plate body, porcelain | 100 |
| 12 | 6.5" tower dish, bone china | 300 |
| 13 | 10 cm saucer dish, bone china | 300 |
| 14 | 5.5" oval bowl, porcelain | 300 |
| 15 | 3.75" round dish, gold rim, porcelain | 300 |
| 16 | 9.5" round bowl, porcelain | 200 |
| 17 | 9.5" round bowl stove, porcelain | 100 |
| 18 | 18" round glass plate, gold (for yu sheng) | 45 |
| 19 | Round glass plate, gold, 430 mm ("Turkey") | 20 |
| 20 | 18 cm glass cover | 100 |
| 21 | Chopsticks with silver, 270 mm | 1,500 |
| 22 | Chopstick and spoon rest, silver line on two sides | 1,500 |
| 23 | Tea cup 230 ml, bone china, silver line on rim | 1,500 |
| 24 | 3-compartment dish | 720 |
| 25 | Salt shaker (3 holes) | 50 |
| 26 | Pepper shaker (5 holes) | 50 |
| 27 | EMSA Campo jug 1 L, red | 30 |
| 28 | EMSA Campo jug 1 L, anthracite | 30 |

### Appendix E: Individual Contributions

Table: tab:contributions | Individual areas of work
| Member | Areas of work | Report sections |
| --- | --- | --- |
| Teh Wei Sheng | VATEL Planner: CCTV table segmentation, camera calibration, mapping tables to the 2-D static map, the planner app; sensor helmet & cable management box design and fabrication | Executive Summary, 1, 2, 5, 6 |
| Soh Sze Juin | Autonomous navigation and localisation: drift correction, dynamic costmap, path planning and following, lining up at the table | 7, 9, 10 |
| Song Cheng Yan | Plate perception and manipulation: plate stack detection, push-to-grasp strategy, MuJoCo simulation, state machine, fitting the simulation to the robot | 3, 4, 8 |

### Appendix 4A: Instruction Option Selection

This appendix gives the reasoning behind each row of Table [[tab:morph-instructions]].

- **Capturing the hall.** Measuring by hand is slow, error-prone and must be redone whenever
  furniture moves. A LiDAR survey needs the robot to walk the hall before every event. Tags on every
  table add setup work and raise the same aesthetic concern as at the table edge (Appendix 4B). The
  hall's CCTV already covers the floor and needs no added hardware; once each camera is calibrated
  (Appendix 5B), it shows where every table stands.
- **Entering the event plan.** Spoken or written instructions are ambiguous and hard to check before
  the robot acts. Stored templates cannot cover every event, and floor plans arrive in mixed formats
  and scales that would each need converting. Arranging tables on a map in an app lets staff set
  each event directly and check the result before it is sent.
- **Generating plate positions.** A large language model can give different answers for the same
  plan, and its output is hard to verify. Fixed rules based on table size and shape place every
  setting the same way each time, so staff do not have to mark each seat.

### Appendix 4B: Navigation Option Selection

This appendix gives the reasoning behind each row of Table [[tab:morph-navigation]].

- **Where the robot is.** Leg odometry drifts with every step and slips on carpet. CCTV tracking
  would reuse the planner's cameras, but it adds latency and loses the robot whenever guests or
  staff block the view. Visual SLAM needs a camera facing 360°, and adds heavy weight to the
  performance budget. A LiDAR measures geometry directly in any lighting, the robot needs one anyway
  to see obstacles.
- **Where to go next.** Sending each goal by hand breaks the autonomous requirement. A fixed goal
  list works only if every table is exactly where it was planned. A state machine can alternate
  between the AGV and the tables, and replace each planned position with the one the robot detects.
- **The path.** A trail would have to be laid again for every layout. Sampling planners such as
  RRT\* suit many-dimensional problems; on a flat floor they give a different, jagged path on each
  run. A\* on a grid finds the shortest path every time, takes in obstacles from the live LiDAR, and
  is built into Nav2.
- **Walking the path.** Pure pursuit tracks a path well but cannot steer around a person; it can
  only stop and wait. MPPI handles robot dynamics better, but Nav2 provides it only for newer ROS 2
  releases, not the ROS 2 Foxy and ROS 1 Noetic the robot runs. DWB is available on Foxy. It tries
  the speeds the robot can reach, simulates each, and keeps the best for path, progress and
  clearance. That is enough for slow walking, and it has few parameters to tune.
- **The table's edge.** AprilTags and RFID both need a tag on every table, which raises concerns on
  aesthetic issues. At 20 cm, the tabletop is below the head LiDAR's field of view. A depth camera
  angled down sees the area just ahead, so it measures the edge's distance and angle directly, with
  nothing fitted to the tables. The same question at the AGV is still open (Appendix 7E).

### Appendix 4C: Plate Manipulation Option Selection

This appendix gives the reasoning for the options in Table [[tab:morph-manipulation]] that Chapter 8
does not cover. The separation options A–C and the three holding options were tested in MuJoCo and
on the R1, and are reported in Section 8.4 (Tables [[tab:push-contacts]] and
[[tab:receiving-grasps]]). The hands are fixed by the platform (N5); a gripper or suction cup would
need new hardware and would not handle the other tableware in Appendix E.

1. Find the plate. The hall CCTV (B) is specified to locate tables only to 10 cm (T4), too coarse to
place a fingertip on a 5.5 mm rim. A fixed stack position (C) needs a holder at a known pose
relative to the robot, which the AGV does not yet provide. The head depth camera (A) needs neither
and was selected; wrist cameras (D) are being evaluated as an addition to it (Section 8.3).

2. Separate the top plate. A tool or fixture (D) depends on the AGV's plate holder, which is not yet
defined, and adds hardware to the robot or the AGV. It is kept as an option once the holder is
specified.

4. Carry plates to the table. One plate in the hand (A) was      chosen to start. It needs no added
hardware, the 522 g plate is well within the 2 kg limit per arm, and the hand that receives the
plate already holds it for the walk. Its drawback is one walk per plate: a banquet of 15–20 tables
of 10 guests needs 150–200 trips. A tray carried by the robot (B) would cut the trips, but the 2 kg
limit allows at most three plates per arm before the tray's own mass, and the robot would also have
to load the tray and pick each plate from it at the table. A trolley pushed by the robot (C) has no
payload limit and could be the dish dolly the AGV already brings (Section 1.5), but pushing it
changes how the robot walks and navigates (Chapter 7), and it must be parked at each table.

Concepts. The fixture-assisted concept is rated ◐ for separating and holding because the holder is
not yet defined and the palm-under grasp was deferred rather than rejected (Section 8.4).

### Appendix 5A: Machine-Vision Models

The planner builds on open-source machine-vision models. This appendix explains what each one does
and why it was selected or rejected (Table [[tab:planner-choices]]).

- **Temporal median (selected).** Takes the median of each pixel over a series of frames. A person
  covers any one pixel in only a few frames, while a table covers it in all of them, so the median
  image shows the furniture without the people [13]. It needs no training and keeps a table that has
  stood still for a minute.
- **Background subtraction, MOG2 (rejected).** Models each pixel's usual colour as a mixture of
  Gaussians and marks pixels that differ from it as foreground [10]. It is built to find moving
  objects: anything that stops moving is gradually absorbed into the background, so it cannot give a
  stable picture of the tables.
- **YOLO (rejected).** A neural-network detector that predicts object boxes and classes in a single
  pass [11]. It is fast but finds only the classes it was trained on, and reliable detection of
  banquet tables would need hundreds of labelled hall images, which we did not have.
- **Grounding DINO (selected).** An open-set detector that links image regions to words, so it draws
  boxes around whatever a text prompt names, here "table", without any training on our halls [14].
- **SAM, the Segment Anything Model (selected).** Turns a prompt, such as a box, into a
  pixel-accurate mask [15]. Given Grounding DINO's boxes, it returns each table's outline, which the
  planner needs to find the tabletop edge (Appendix 5C).

### Appendix 5B: Camera Model and Calibration

Each CCTV camera is modelled as a pinhole camera with radial lens distortion: a venue point X maps
into the camera frame as x = R(X − C), and its normalised image radius r is distorted to r(1 + k₁r²)
and scaled by the focal length f. Seven unknowns are solved once per camera (f, k₁, the three angles
of R and the three coordinates of C) by a robust least-squares fit to the references in Table
[[tab:calib-residuals]].

Table: tab:calib-residuals | Calibration references
| Reference | What it constrains |
| --- | --- |
| Floor-plan points (a column's foot, a door frame) | Pixel and known 3-D position: reprojection error |
| Known edges | A pixel on an edge whose position on the plan is known |
| Plumb edges, found automatically | Lines vertical in the room: pitch, roll and distortion |
| Tabletops | Four corners must form a rectangle on the tabletop plane; the tabletop height is an eighth unknown |

On a wide lens, focal length and distortion trade off against each other, so the fit is started from
several fields of view (55°–110°) and the lowest-cost result is kept. For the Tinkering Corner
camera this gave a height of 2.57 m, k₁ = −0.19 and a 13-pixel residual in a 2,560-pixel-wide frame.
A calibration is accepted only when the floor plan, projected back onto the image, lines up with the
walls and pillars (Figure [[fig:calibration-check]]).

### Appendix 5C: From Pixels to Table Positions

**Ray–plane intersection.** Each undistorted pixel defines a ray from the camera centre C with
direction d, which meets the plane at height h at P = C + s·d, where s = (h − C_z)/d_z. Table
outlines are intersected with the tabletop plane (h = 0.762 m), not the floor.

**Why the height matters.** Mapping a tabletop as if it were on the floor scales its position about
the nadir (the floor point below the camera) by H/(H − h) for a camera at height H: a table 10 m
from a 4 m-high camera would be placed 2.35 m too far. In the simulated hall, intersecting with the
tabletop plane reduced offsets of 223 mm and 329 mm to 0 mm.

**Far-edge envelope.** Legs, skirts and chairs lie below the tabletop, so their rays reach tabletop
height early and land between the table and the nadir. Seen from the nadir, the farthest outline
point in each direction (in 0.3° bins) is therefore the tabletop's edge; bins cut off by the image
border are dropped.

**Shape fit.** A rectangle, aligned with the dominant edge direction and squared to the walls within
12°, and a circle are both fitted to the envelope. The table is taken as round only if the circle
fits clearly better and the envelope is close to square.

**Two cameras.** Overlapping footprints from two cameras are one table seen twice; the sighting not
cut off by the frame edge, then the closer and more confident one, is kept.

### Appendix 5D: Planner Parameters

Table: tab:planner-params | Main planner parameters
| Stage | Parameter | Value |
| --- | --- | --- |
| Clean plate | Frames in the median | 31, one every 2 s |
| Detection | Models | Grounding DINO (base) and SAM (ViT-H), on the frame resized to 1,280 px wide |
| Detection | Box and text thresholds | 0.22 and 0.20 (a row of dark tables was missed at the usual 0.30) |
| Detection | Re-run | Every 15 s, only when the clean plate has changed |
| Mask clean-up | Duplicates and chairs | Masks overlapping by more than 0.5 IoU are merged; chair masks are cut out of table masks |
| Snapping | Stock sizes | 5, 5.5, 6 and 7 ft rounds; 6 × 2.5 ft and 6 × 1.5 ft oblongs; the measured size is kept if more than 15% off |
| Settings | Covers and robot stop | About 0.6 m of rim per cover; robot stop 0.50 m from the table edge |
| Live mode | Updates | Every 1.5 s; a table is removed or resized only after three agreeing updates |
| Export | Map and goals | .pgm at 0.05 m per pixel; one named goal per setting (e.g. T07/cover03) |

### Appendix 6A: Sensor Coverage Geometry

Heights are from the floor with the R1 standing and its head at zero pitch and yaw; distances are
from the neck's yaw axis.

**LiDAR.** The JT128's beams span 4.4° below to 90.5° above its own horizon. Its optical centre is
1.315 m above the floor and it is tilted forward by 24°, so its lowest beam points 28.4° down ahead
of the robot and 19.6° up behind it. That beam meets a surface at height z at d = (1.315 − z)/tan
28.4° + 0.03 m: 2.46 m for the floor and 1.05 m for a 0.762 m tabletop. The highest part of the
helmet, the brow bumper, lies 21.1° below the LiDAR's horizon, 16.7° clear of the lowest beam.

**Choosing the tilt.** The tilt was the steepest, searched in 1° steps from 25°, at which every part
of the helmet stays below the lowest beam. The search used a provisional LiDAR model (lowest beam at
−9°) before the JT128's manual arrived; the real optics only increased the clearance, so the 24°
tilt was kept.

**Depth camera.** The D435if's depth image spans 87° × 58°. Tilted 30° down at 1.21 m, 0.095 m ahead
of the neck axis, its lower edge points 59° down and meets a surface at height z at d = (1.21 −
z)/tan 59° + 0.095 m: 0.82 m for the floor and 0.37 m for a tabletop.

**Final approach.** The G1 tests used a camera pitched 63.6° down for the table-edge alignment
(Section 7.2.4). On the R1, the 30° mount plus a 33.6° head pitch gives the same view, within the
joint's ±36° range.

**Mount check.** On the robot, the camera measured 34.0° below horizontal against the CAD's 30.0°,
with −2.3° of roll at a height of 1.18 m, repeatable to ±0.2°. The measured pose, not the CAD value,
is used for the camera-to-robot transform.

### Appendix 6B: Clamp and Skull-Screw Analysis

**Load path.** Two M3 × 16 screws fix the side pieces to existing threaded holes in the R1's skull,
42 mm either side of the centreline. A rear clamp is kept as a second load path: the side pieces
press on the head through ear-ring pads preloaded by 0.5 mm, and a thumbscrew draws their flanges
together across a 1.0 mm gap.

**Finite-element model.** A linear-elastic contact model of one side piece (gmsh mesh, scikit-fem
solver; PETG with E = 1.5 GPa, TPU pads as compression-only springs, skull screw fixed) gives the
loads in Table [[tab:clamp-fe]] and the stresses in Figure [[fig:clamp-fe]]. The results move by up
to ±25% with mesh refinement, so they are approximate.

Table: tab:clamp-fe | Rear-clamp loads with the flanges touching (±25%)
| Quantity | Value |
| --- | --- |
| Thumbscrew tension | 29 N |
| Ear-cup force | 27 N (19 N with the thumbscrew loose) |
| Rear-pad force | 14 N |
| Skull-screw load | 36 N |
| Peak stress (crown arm, front edge) | 7.0 MPa |

![fig:clamp-fe | Von Mises stress in the side piece with the flanges touching; the colour scale is capped at the 99.5th percentile and the 7.0 MPa peak is circled](../assets/interim/helmet-clamp-fe.webp)

**Why the screws were added.** The first, clamp-only design slid backwards off the head: its model
showed the rear pads carrying 44 N of the 52 N clamp force and the ear cups, which locate the
helmet, only 13 N. Screwing the side pieces to the skull and preloading the ear pads moved the load
onto the ear cups (27 N) and off the rear pads (14 N).

**CAD checks.** No two parts interfere, all 21 threaded fasteners have enough thread engagement, and
no part enters the camera's field of view.

### Appendix 6C: Added Mass and Centre of Gravity

The masses are estimates: printed parts from their CAD volumes with the planned walls and infill,
sensors from their datasheets and the cable box from its bill of materials. They will be replaced by
weighed values. The robot's centre of gravity (CoG) is computed from the R1's URDF in its standing
zero pose.

Table: tab:added-mass | Added mass
| Item | Mass |
| --- | --- |
| Hesai JT128 LiDAR, with pigtail cable | 250 g |
| RealSense D435if camera | 75 g |
| Printed parts (crest, side pieces, plates, cradle, bumpers, pads, roll hoop) | 376 g |
| Screws, inserts and cables on the head | 72 g |
| Helmet total | 773 g |
| Cable box, with USB hub and cables | About 960 g |

Table: tab:cog-shift | Robot mass and centre-of-gravity height, standing zero pose
| Configuration | Mass | CoG above the floor |
| --- | --- | --- |
| R1 as supplied | 28.84 kg | 0.658 m |
| With the helmet | 29.61 kg (+2.7%) | 0.673 m (+15 mm) |
| With the helmet and cable box | 30.57 kg (+6.0%) | 0.680 m (+23 mm, 4 mm rearward) |

**Effect on the robot.** The helmet's own CoG sits 10 mm ahead of and 200 mm above the head's pitch
axis, so it adds at most about 1 N·m of gravity torque at the full 36° of head pitch, about 3% of
the head joints' 33 N·m, and triples the head's pitch inertia (0.017 to 0.050 kg·m²). The robot as a
whole becomes 6% heavier with a CoG 23 mm higher, which is expected to stay within what the R1's
walking controller handles; this will be confirmed in walking tests with the helmet and box fitted.

### Appendix 6D: Cable Take-Up Sizing

**Cable to store.** The cable span from the clip on the rear clamp to the box lengthens as the head
turns. Computed from the R1's meshes over the full pitch range (±36°), the length to store depends
on the yaw limit (Table [[tab:takeup-yaw]]).

Table: tab:takeup-yaw | Cable to store and clearance to the neck, by head-yaw limit (pitch ±36°)
| Head-yaw limit | Cable to store | Gap to neck / torso |
| --- | --- | --- |
| ±60° | 193 mm | 26.9 / 25.9 mm |
| ±70° | 201 mm | 18.6 / 18.5 mm |
| ±80° | 209 mm | 10.5 / 11.3 mm |
| ±90° | 216 mm | 2.0 / 4.7 mm |
| ±115° (joint limit) | 229 mm | 0.4 / 0.9 mm |

**The 2:1 loop.** Both cables run round the carriage's pulley and back up over a fixed former, so
each millimetre of carriage travel stores 2 mm of cable: 117.4 mm of travel stores 235 mm. ±80° of
yaw needs 209 mm and keeps the cables at least 10.5 mm from the neck and torso, so ±80° is the
head-yaw range the box is designed for (the joint itself reaches ±115°). The pulley's 28 mm radius
is five times the JT128 cable's diameter, its minimum bend radius.

**Tension.** A constant-force spring of about 4 N plus the carriage's 1.2 N weight gives about 1.3 N
per cable strand, or 2.6 N on the head: at most 0.5 N·m about the neck, under 2% of the head joints'
33 N·m. The spring is not yet final.

**Clearances.** In CAD, the head and helmet stay at least 14.0 mm from the box over the full neck
range, and the waist and arms move through their ranges without touching it. The box has not yet
been built or fitted.

### Appendix 7A: FAST-LIO2 Mechanisms

- **Backward propagation.** A LiDAR sweep takes about 100 ms, during which the robot moves and
  turns, so each point in the sweep is captured from a different pose. Treating the points as
  simultaneous bends straight surfaces. The IMU's high-rate measurements are used to place each
  point at the pose it was actually taken from, straightening the sweep before it is used.
- **The iterated EKF.** The IMU propagates the state forward between sweeps. Each de-skewed point is
  then matched to the map, and its point-to-plane residual updates the state. The filter
  re-linearises around the improved estimate and repeats, which behaves more like a Gauss–Newton
  optimisation than a single EKF update. Crucially, the update corrects the IMU biases as well as
  the pose, which stops the inertial error from compounding.

### Appendix 7B: Localiser Correction and Drift Test Details

**Accumulator gate.** Scan insertion is gated on the relocaliser's inlier ratio (Table
[[tab:accumulator-gate]]):

Table: tab:accumulator-gate | Accumulator gate by relocaliser inlier ratio
| Inlier ratio | What it means | Scan inserted? |
| --- | --- | --- |
| Below 0.15 | No prior map exists yet | Yes, on odometry alone, to bootstrap the map |
| 0.15 to 0.50 | The pose is ambiguous | No |
| Above 0.50 | The pose is trusted | Yes |

Without this gate, a poor pose would be written into the map and then used to validate itself.

**Relocaliser settings.** Each cycle accumulates five scans, downsampled to at most 8,000 points,
and runs 15 ICP iterations with a coarse-to-fine correspondence gate of 1.5 → 0.8 → 0.4 m, because a
single narrow gate cannot recover once the estimate is poor.

**Drift test procedure.** Each condition walked five 5 m out-and-back laps under Nav2, about 58 m
per run, starting from the same floor mark. The odometry-only condition was produced by disabling
the relocaliser's ICP (seed_only mode), so both runs share an identical map frame and differ only in
whether the correction was applied. Laps are repeated because the navigation goal tolerance is 0.30
m: a single arrival may be that far off for reasons unrelated to drift. Arrival error does not
accumulate across arrivals, whereas drift does, so over 58 m drift becomes the dominant term.

**Per-lap measurement.** After each lap, the robot's position was taped from the start mark on
arrival, before the final turn in place (Figure [[fig:tape-method]]). Two distances were recorded:
forward along the starting heading, and sideways to the left of it. The robot's own estimate at the
same moment was taken from the logged pose. The per-lap distances walked are the run total divided
evenly across the five laps, as the log has no per-lap distance. Table [[tab:drift-per-lap]] gives
the readings, and Figures [[fig:laps-reloc-on]] and [[fig:laps-reloc-off]] show the robot at the
mark after each lap.

![fig:tape-method | Measuring the robot's position from the start mark](../assets/interim/nav-start-position.webp)

Table: tab:drift-per-lap | Position error at the start mark after each lap (estimate − tape)
| Lap | Distance walked (m) | Relocaliser off: forward, left (cm) | Relocaliser off: error (cm) | Relocaliser on: forward, left (cm) | Relocaliser on: error (cm) |
| --- | --- | --- | --- | --- | --- |
| 1 | ≈11.5 | +2.0, 0.0 | 2.0 | +2.7, 0.0 | 2.7 |
| 2 | ≈23.0 | −2.3, −4.2 | 4.8 | +1.8, −2.7 | 3.2 |
| 3 | ≈34.6 | −5.8, −6.8 | 8.9 | +1.8, +1.7 | 2.5 |
| 4 | ≈46.1 | −6.4, −7.0 | 9.5 | +2.4, −2.6 | 3.5 |
| 5 | ≈57.6 | +3.8, −7.8 | 8.7 | −2.3, +0.8 | 2.4 |

![fig:laps-reloc-on | Robot at the start mark on arrival after each lap, relocaliser on (laps 1–5)](../assets/interim/nav-laps-reloc-on.webp)

![fig:laps-reloc-off | Robot at the start mark on arrival after each lap, relocaliser off (laps 1–5)](../assets/interim/nav-laps-reloc-off.webp)

**Why sideways error is the better measure on this route.** Every lap walks 5 m out and 5 m back
along the same line. If the odometry over- or under-estimates distance, it errs one way going out
and the opposite way coming back, which cancels at the start mark. This is why the error without the
relocaliser stopped growing after lap 3, and why the forward error changed sign over the run (+2.0,
−2.3, −5.8, −6.4, +3.8 cm). Sideways error comes mainly from small heading errors at the 180° turns,
which the next leg does not undo. It therefore stayed on the same side and grew lap by lap (0, −4.2,
−6.8, −7.0, −7.8 cm).

**Turn in place.** Without the relocaliser, a single turn of about 24° in place after the final lap
took the error from 8.7 cm to 18.9 cm. During that turn the tape shows the robot moved about 14 cm
forward, while its estimate recorded about 6 cm backward. This suggests odometry error builds up
quickly when the robot turns on the spot, though it was observed only once.

**Independent check.** The cumulative correction applied by the relocaliser was consistent across
two runs, at 4.0% and 4.3% of distance walked, while the net shift of the map origin was under 1.5
cm. The relocaliser was therefore not displacing the frame, but continuously cancelling an error
that re-accumulated at about 4% of distance. For comparison, a published benchmark over 51.9 km of
driving reports FAST-LIO2 drift of up to 4.7% of distance travelled [34].

### Appendix 7C: Detection Confidence

A detected table or AGV position overwrites the rough map only once the mission planner is confident
in it. For a table, this takes two stages: the LiDAR registers a candidate, and the RealSense
confirms it.

**LiDAR: registering a candidate.** The accumulated map (Section 7.2.1.3) is divided into 0.15 m
grid cells. Taking the table height as 0.76 m, a cell counts as tabletop if it holds accumulated
points between 0.68 and 0.86 m above the floor. Neighbouring tabletop cells are grouped into
clusters, and a rectangle is fitted around each one. A cluster is registered as a table only if its
rectangle passes every check in Table [[tab:table-checks]].

Table: tab:table-checks | Geometric checks a tabletop cluster must pass to be registered as a table
| Check | Threshold |
| --- | --- |
| Long side of the rectangle | ≤ 3.0 m |
| Short side of the rectangle | ≥ 0.95 m |
| Long-to-short ratio | ≤ 1.30 (close to square) |
| Share of the rectangle filled with tabletop cells | ≥ 40% |

**Camera: confirming it.** A registered table is still only a candidate. It is confirmed when the
RealSense sees a flat (RMS < 20 mm), horizontal plane within ±5 cm of the expected table height.
Only then does its position overwrite the rough map.

The AGV's detection criterion has not been developed yet. It will be defined together with the AGV
fine approach (Appendix 7E).

### Appendix 7D: The Decay Calculation

Per cell, once per 2.0 s window, with evidence e capped at 10:

`hit at least once in the window   ->  e <- min(e + 1, 10)`

`swept by a beam but not hit       ->  e <- e - 1      (charged every 3rd window)`

`neither                           ->  e unchanged`

`e >= 2  ->  published as an obstacle`

`e  = 0  ->  removed`

A beam is sampled from the robot's current position to one cell short of its return, up to a range
of 15 m, and a sample contributes only where its interpolated height lies within 0.20–1.60 m.
Occlusion tolerance is cap × decay_every × window = 10 × 3 × 2 s = 60 s. During the first 20 s after
startup, penalties are suppressed.

The 15 m sampling range was chosen by measurement: extending it from 5 m to 15 m multiplied the
free-space evidence by 3.4, while 15 m to 25 m added only 1.26 times more, because beams already
terminate on walls before reaching that far. An alternative scheme that simply discarded any cell
unobserved for a fixed time was implemented and rejected: it removed 1,085 cells that no beam could
physically reach, erasing exactly the occluded regions the three-state rule exists to preserve.

Three refinements make the clearing rule behave:

- **Beams stop one cell short of what they hit.** Nothing beyond a return is ever counted as empty,
  so a beam striking a wall cannot clear the room behind it.
- **The height band is checked along the beam, not only at its end.** The beam's height is
  interpolated over its length, and only the portion within 0.20–1.60 m contributes clearing.
  Without this, a beam passing above a table and striking the far wall would clear the table's own
  cells on the way through.
- **Evidence accumulates over a 2 s window rather than per scan.** The LiDAR returns about 1,240
  points per scan against 14,400 cells in a 6 m area, so no single scan can touch most of them;
  charging a penalty per scan erodes real walls faster than they are re-observed.
  Counter-intuitively, a finer grid makes this worse, not better.

### Appendix 7E: AGV Fine-Approach Options

The AGV needs the same precise, square stop as the table, but the method is still being explored.
The options under consideration are:

- **RFID.** Tags on the AGV, read by a reader on the robot, confirm the robot is in position.
- **RealSense computer-vision detection (AprilTag).** Fiducial markers on the AGV, detected in the
  RealSense image, give the AGV's position and orientation relative to the robot.
- **Depth-map reuse.** The tabletop edge detector used for the table (Section 7.2.4.2) is reused on
  the AGV's depth image.

**Comparison plan.** We will first obtain the AGV's specification from PARKROYAL. The options that
suit it will then be compared on three criteria:

1. whether it can be built from the robot's current hardware without additional parts
2. whether it gives the accuracy navigation needs to stop at the AGV
3. whether the modification can be applied to the AGV with no maintenance needed.

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
noise was 0.7 mm. The wrist camera's pose, calibrated against the head camera with a checkerboard,
was recovered to 2.9 mm and 0.6° in MuJoCo.

### Appendix 8B: Estimated Rim Breaking Load

No published breaking load for the rim of a porcelain plate was found; tableware standards address
impact rather than static load [35]. An estimate was therefore made from the flexural strength of
porcelain, typically 60–80 MPa for conventional porcelain [22], treating the rim as a cantilever
loaded at its edge by a finger:

F = σ b t² / (6 L)

where σ is the flexural strength, t = 5.5 mm the rim thickness, L = 34.5 mm the distance from the
edge to where the rim meets the plate's slope (radius 127.5 mm to 93 mm), and b the width of rim
that carries the load.

Table: tab:rim-load | Estimated rim breaking load and safety margin
| Load width b | Breaking load F (σ = 60–80 MPa) | Margin against 60 N peak | Margin against 30 N sustained |
| --- | --- | --- | --- |
| 14 mm (fingertip width; conservative) | 120–165 N | 2.0–2.7 | 4.1–5.5 |
| 69 mm (load spread over twice the rim length) | 600–810 N | 10–13 | 20–27 |

The conservative case (Table [[tab:rim-load]]) gives a margin of about 2 at the peak limit. The
estimate ignores the rim's taper and curvature, stress concentration at chips or glaze flaws, and
the support from the plate below during the push, and some porcelains are weaker (about 40 MPa,
which gives 86 N). The limits are therefore treated as provisional, and a static rim-load test is
planned: a fingertip-sized pad pressed on the rim of a spare plate, with the load recorded until the
rim chips. In use, the measured forces are well below the limits during the push (7–9 N downward in
a hand-guided index–middle push on a scale, with the Revo 2 hand held by a person), while the grasp
and lift approach them: in the simulation evaluation, 4 of 24 conditions exceeded the limits
(Appendix 8E).

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
  represent this, so the LuGre model was used [36]. It gives each joint an internal state z, which
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
friction was varied from 0.25 to 2.0, the plate-on-plate friction over 0.45, 0.575 and 0.7, and the
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

### Appendix 8F: Simulation Model

The simulation model was built from the manufacturers' URDF files, with the changes listed in Table
[[tab:sim-model]].

Table: tab:sim-model | Sources of the simulation model and changes made
| **Part** | **Source** | **Changes made for this project** |
| --- | --- | --- |
| R1 body and arms | Unitree R1 URDF [37] | Upper body only, pelvis fixed; stock hand removed from the wrist |
| Revo 2 hands | BrainCo Revo 2 URDF [38] | Mounted on each wrist (adapter plate not modelled); finger coupling kept |
| Assembly | Own script merges the two URDFs into one MuJoCo model | No self-collision within one arm |
| Actuators | Position actuator on every joint | Arm gains matched to the R1 |
| Arm behaviour | Identified from R1 logs (Appendix 8C) | Gravity, joint friction and 25 ms delay added |
| Plate | Measured cross-section (Figure [[fig:plate-geometry]]) | Built from convex shapes for accurate contact |
| Physics | MuJoCo, 1 ms time step | Friction set per surface pair (Table [[tab:friction]]) |
| Cameras | Calibrated head and wrist camera poses | Wrist-camera images rendered for the rim tracker |
