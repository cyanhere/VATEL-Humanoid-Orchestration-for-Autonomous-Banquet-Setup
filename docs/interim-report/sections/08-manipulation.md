---
id: manipulation
---

## 8. Plate Perception and Manipulation

### 8.1. Requirements, Scope and Design Decisions

Of the 28 items in the partner's tableware inventory (Appendix E), the 10.5-inch porcelain dinner
plate is the most numerous (3,000 pieces), so development began with it. Test plates of the same
diameter were procured; they weigh 522 g and dimension is as shown in figure (Figure
[[fig:plate-geometry]]). Two further constraints follow from the partner's operation:

- **Stacked delivery.** Plates arrive in nested stacks with only 0.5–2.0 mm between adjacent rims,
  so a single plate must be separated from the stack.
- **Undefined source fixture.** Stacks are collected from the AGV, but the holder it carries (rack,
  shelf or dish dolly) is not yet specified.

![fig:plate-geometry | Test plate: (a) cross-section; (b) two nested plates](../assets/interim/manip-plate-geometry.webp)

Until the fixture is defined, the base case is a stack on a flat table. The final fixture may hold
the stack at another height, so the robot's working height is treated as adjustable (Section 8.6).

Table [[tab:pick-criteria]] lists the evaluation criteria, and Figure [[fig:pick-decisions]]
summarises the design decisions of Sections 8.3 and 8.4.

Table: tab:pick-criteria | Evaluation criteria for plate picking
| Criterion | Value | Basis |
| --- | --- | --- |
| Object | 10.5-inch round porcelain plate (255 mm test plate) | Partner inventory |
| Initial state | Nested stack of 2–5 plates on a flat surface | Stacked delivery; fixture not yet defined |
| Separation | Top plate displaced 30–70 mm; plates below displaced < 5 mm | a) Lower bound: rim exposure required by the receiving hand. b) Upper bound: an 85 mm push tipped the plate off the stack in simulation; 70 mm keeps a margin.  c) The remaining stack must stay in place for the next pick |
| Contact force | ≤ 30 N hold load and ≤ 60 N peak per finger | Estimated rim fracture load 120–165 N from porcelain flexural strength [19], about twice the peak limit (Appendix 8B) |
| Pose uncertainty | ±15 mm lateral between camera estimate and hand | Measured camera and arm errors (Section 8.2) |

![fig:pick-decisions | Design decisions for single-plate picking](../assets/interim/manip-pick-decisions.webp)

### 8.2. Plate Perception

**Stack localisation.** A RealSense D435if depth camera on the head localises the stack once before
any motion (Figure [[fig:stack-detection]]). The pipeline was revised several times in response to
failures observed on real images; Table [[tab:stack-failures]] lists the options considered for each
function and the basis for the selection.

![fig:stack-detection | Detected stack: outline, height, count and position](../assets/interim/manip-stack-detect.webp)

Table: tab:stack-failures | Stack localisation failures and fixes
| Failure observed | Change made | Result |
| --- | --- | --- |
| Glare removes depth data, mainly in the well | Outline found from colour edges; depth sampled on the rim only | Height and centre unaffected |
| At oblique views the rim appears elliptical; a circle fit gave up to 21% diameter error | Ellipse fit [20] | Diameter 249–256 mm on real images (true 255 mm) |
| On real images the outline followed both edges of the gapped rim; diameter read about 200 mm | Outline follows the outer rim edge only | Centre repeatable to 0.25–0.30 mm (SD) |
| Level table appeared tilted 2.5° as the robot leaned forward (IMU pitch 3.0°) | Table plane fitted around the stack (RANSAC [21]); heights measured from it | Heights unaffected by lean and camera-pose error |
| The well forms a second, lower height cluster; a high depth percentile was biased upward by noise | Median of the topmost (rim) cluster, re-centred until stable | Stable top-plate height |
| Counting side edges under-counted tall stacks | Plate count = stack height ÷ nesting pitch | Exact for 1–20 plates in synthetic tests |

**Hand-eye calibration.** The camera estimates the plate pose, but hand placement relies on joint
kinematics, so the two must agree in the robot frame. Before calibration, the predicted hand
position was 41.7 mm to the side of the observed hand (Figure [[fig:hand-eye-discrepancy]]). Three
error sources were found and corrected (Table [[tab:hand-eye-errors]]).

Table: tab:hand-eye-errors | Hand-eye error sources and corrections
| Error source | Correction | Result |
| --- | --- | --- |
| Camera mounting pose: position and heading taken from CAD | Recalibrated over 15 head poses against IMU gravity; camera found 30 mm to the side and turned 1.2° | 1.8 mm RMS on held-out poses |
| Head yaw play: hand appeared to shift up to 13 mm with head-yaw direction | Every head-yaw move finishes from the same direction | Camera and hand heights agree within 1.4 mm over 38 head poses (previously ±9 mm) |
| Arm joint offsets | Each hand touched the table at nine points (proximity-sensed); joint offsets fitted by least squares | Offsets of up to 3.6° on four joints per arm |

The mean height error fell from 12.6 to 2.3 mm for the left hand and from 4.5 to 3.7 mm for the
right (Figure [[fig:height-error]]). A horizontal table constrains height only, so the remaining
lateral error is handled by tactile edge localisation before each push (Section 8.5).

![fig:hand-eye-discrepancy | Hand-eye offset before calibration (41.7 mm)](../assets/interim/manip-hand-eye.webp)

![fig:height-error | Camera-to-hand height error, before and after calibration](../assets/interim/manip-height-error.webp)

**Rim tracking.** During manipulation, a rim tracker follows the top plate at camera rate (10–30 Hz;
Figure [[fig:rim-tracker]], Appendix 8A). It fits a circle of known radius to the rim's outer edge,
ignoring points on the robot's own arm and hand, to give the plate centre, and a plane to the rim
heights to give the plate's tilt (Section 8.5 explains why the tilt is needed). The state machine
uses it to tell plate motion from fingertip slip.

![fig:rim-tracker | Rim tracker on a 3-plate stack](../assets/interim/manip-rim-tracker.webp)

**Wrist cameras.** The head camera leaves a lateral error of up to about 10 mm, and the hand hides
the gripped rim from it. To fix this the colour cameras mounted on the wrists were used, which a
model-based tracker projects the expected rim into this image, finds the rim edge along lines normal
to the projection and corrects the rim's position and height (Figure [[fig:wrist-camera]]); in
simulation it reduced a 12 mm position error to 2.7 mm. The wrist camera's pose, calibrated against
the head camera with a checkerboard, was recovered to 2.9 mm and 0.6° in simulation. Robot
integration is in progress.

![fig:wrist-camera | Wrist-camera rim tracking in simulation](../assets/interim/manip-wrist-camera.webp)

### 8.3. Single-Plate Separation: Challenges and Concept Selection

A single plate can be clamped between two hands, but a nested stack offers no grasp point: the
0.5–2.0 mm gap between rims is far smaller than the 14 mm fingertip. Human operators use three
strategies (Figure [[fig:human-strategies]]): two grasp the plate in place (fingernail insertion and
rim pinch), and the third draws the plate outward with the thumb to create a grasp point.

![fig:human-strategies | Human strategies: fingernail, rim pinch, thumb draw](../assets/interim/manip-human-strategies.webp)

None of these strategies transfers directly to the R1. Each arm has five joints and no wrist pitch,
so in simulation the fingertip approaches the gap 54° from the required angle (Figure
[[fig:r1-arm]]); each hand has 6 actuated joints against approximately 24 in the human hand, and the
rigid fingertips have no force sensing at the tip.

![fig:r1-arm | R1 arm (left) and simulated fingertip approach (right)](../assets/interim/manip-r1-reach.webp)

The plate must therefore be displaced before it can be grasped. Pre-grasp manipulation of thin
objects has been shown by sliding an object to a table edge [22] or pushing it against a wall [23],
both with a gripper and a wrist. A stack has no such edge, so the plate beneath serves as the
support. Concepts were evaluated in MuJoCo with a plate model built from the measured profile and
initial friction estimates (Table [[tab:friction]]). Pressing the near rim to tip the far rim up
(Figure [[fig:rejected-press]]) was rejected: the near rim can drop only by the rim gap, so the far
rim rose 3–8 mm, against about 16 mm needed for finger entry. Pushing the plate sideways instead
makes its far side to overhang and rise over the plate beneath; the opposite hand then grasps the
exposed rim and the pushing hand regrasps.

![fig:rejected-press | Rejected concept: pressing the near rim](../assets/interim/manip-rejected-press.webp)

Three pushing contacts were compared in simulation (Figure [[fig:push-contacts]], Table
[[tab:push-contacts]]).

![fig:push-contacts | Pushing contacts: (a) pinky end; (b) fingertip drag; (c) hooked finger](../assets/interim/manip-push-contacts.webp)

Table: tab:push-contacts | Pushing contacts in simulation
| Contact | How it moves the plate | Main difficulty | Decision |
| --- | --- | --- | --- |
| (a) Pinky end on the rim edge | Fingertip pushes the rim edge | The edge sinks then rises as the plate slides; tracking it needs combined wrist roll and inward motion, and a missed edge lands the fingertip on the plate below | Rejected |
| (b) Index–middle fingertip drag | Friction under pads pressed on the rim | Relies on fingertip friction, then unknown | Deferred; revisited in Section 8.4 |
| (c) Hooked middle finger on the outer edge | Curled middle finger wedges against the rim's outer edge | Stays located on the edge, unlike the pinky; no reliance on friction; fingertip clear of the plate below; tolerates −15 to +20 mm error | Selected |

The hooked contact was therefore developed first (Figure [[fig:hooked-push]]).

![fig:hooked-push | Hooked push on the R1, side view](../assets/interim/manip-hooked-push.webp)

**Receiving-hand grasp.** Once the rim overhangs, the opposite hand must grasp a 5.5 mm-thick rim
edge with a hand whose fingertip pads cannot oppose one another closely. Of the grasps reachable by
the arm, three were evaluated in simulation (Figure [[fig:receiving-grasps]], Table
[[tab:receiving-grasps]]).

![fig:receiving-grasps | Receiving-hand grasps: (a) pinch; (b) palm under; (c) finger slot](../assets/interim/manip-receiving-grasps.webp)

Table: tab:receiving-grasps | Receiving-hand grasps in simulation
| Grasp | Outcome | Decision |
| --- | --- | --- |
| (a) Thumb–middle fingertip pinch | Pads close only to 19 mm on a 5.5 mm rim; plate slipped out | Rejected |
| (b) Palm under the plate, thumb above | Lifted one side, but sliding deeper pushed the plate back or tilted it up to 40°; thumb pose unreachable (13–15° IK error) | Deferred; revisit with a longer push |
| (c) Rim in the index–middle finger slot, thumb closing | Slot narrows to 5.9 mm, wedging the 5.5 mm rim; held in 14 of 14 trials | Selected |

### 8.4. Control Approach, Simulation and System Identification

**Control approach.** Of the three approaches compared (Table [[tab:control-approaches]]), neither
learning method had been shown to separate nested plates with a dexterous hand. A scripted state
machine (Figure [[fig:pick-sequence]]) was therefore implemented first, to establish feasible
motions and characterise the robot; reinforcement learning is developed in parallel.

Table: tab:control-approaches | Control approaches
| Approach | Strengths | Limitations for this task | Role |
| --- | --- | --- | --- |
| Scripted state machine | Interpretable; each step sensor-checked; no training data | Hand-tuned; limited generalisation | Baseline; implemented |
| Reinforcement learning (RL) | Generalises through training in simulation | Needs accurate contact simulation and reward design | Parallel development |
| Imitation learning, e.g. diffusion policy [24] | Learns from demonstrations; handles multimodal actions | Needs teleoperation and demonstrations of a dexterous push | Fallback |

![fig:pick-sequence | Pick sequence in simulation](../assets/interim/manip-state-machine.webp)

**Simulator selection.** MuJoCo [25] was selected for both motion development and reinforcement
learning (Table [[tab:simulators]]). It performed best on the robotics tests in a comparison of five
physics engines [26], and its GPU implementation, MuJoCo Warp, used here, gives the parallel
throughput reinforcement learning needs; policies trained in GPU MuJoCo have transferred to a real
dexterous hand [27]. In Isaac Lab [28], used for early experiments, the plate had to be rebuilt from
convex shapes because mesh collision (about 1 mm) was coarser than the 0.6 mm slack in the slot
grasp. Using one engine for development, identification and training lets the identified model
(below) be used directly in training.

Table: tab:simulators | Simulator comparison
|  | MuJoCo (CPU) | MuJoCo Warp (GPU) | Isaac Lab |
| --- | --- | --- | --- |
| Use in this project | Motion design, evaluation, log replay, system identification | RL (current) | Early RL experiments |
| Strength | Accurate, stable contact [26] | Thousands of parallel environments, same physics | Thousands of parallel environments |
| Limitation found | One environment per CPU core | GPU memory limits the number of plates per scene | Collision coarser than grasp clearance; out of memory with 3–5 plates |

**Validation in simulation.** Every motion was first run in MuJoCo under the camera, encoder-zero
and joint-friction errors measured on the R1, and taken to the robot only if it succeeded under all
of them within the force limits (Appendix 8E).

**System identification.** Under identical commands, the R1's hand deviated by 13.0 mm RMS from the
simulated hand. Arm gravity loading, joint friction and a 25 ms command latency were identified from
robot logs and added to the simulation (Appendix 8C). Joint friction used the LuGre model [29],
which depends on recent motion as well as speed and so matched the R1's low friction just after a
change of direction. The identified model reduced the index fingertip error from 14.2 to 4.0 mm RMS
(Figure [[fig:fingertip-error]]).

![fig:fingertip-error | Fingertip error: default 14.2 mm vs identified 4.0 mm RMS](../assets/interim/manip-finger-error.webp)

**Contact friction.** No published value exists for the Revo 2 fingertip on glazed porcelain, so the
simulation began with estimates (Table [[tab:friction]]). Ten robot pushes were then replayed in
MuJoCo with their logged commands over a range of friction values and compared with the camera
record (Figure [[fig:friction-id]], Appendix 8D). The data require a fingertip friction of at least
0.75, most likely 1.0–1.3, three to five times the estimate. This makes the drag contact of Section
8.3 feasible, and it is being re-evaluated.

Table: tab:friction | Contact friction: initial and identified
| Contact pair | Initial value in simulation | Source | Identified value | Method |
| --- | --- | --- | --- | --- |
| Revo 2 fingertip on glaze | 0.25 | Material estimate; not measured | ≥ 0.75, most likely 1.0–1.3 | Replay of ten R1 pushes (MuJoCo) |
| Plate foot on glaze (plate on plate) | 0.575 | Estimate | About half the fingertip value; tilt test needed | Same replay (ratio only) |
| Plate foot on table | 0.45 | Estimate | ≥ 0.90 (lower bound) | Tilt test on the table |

![fig:friction-id | Fingertip friction identified from a replayed push](../assets/interim/manip-friction-id.webp)

### 8.5. Robot Deployment and Experimental Iteration

Every robot run is logged, replayed in simulation and used to refine the model (Figure
[[fig:sim-to-robot]]). On the robot, the state machine runs in the loop of Figure
[[fig:control-loop]]: the arm server sends joint targets to the motors at 50 Hz and returns the arms
to a safe pose if a joint, speed or force limit is exceeded.

![fig:sim-to-robot | Simulation-to-robot fitting loop](../assets/interim/manip-sim-to-robot.webp)

![fig:control-loop | Control loop on the R1](../assets/interim/manip-control-loop.webp)

The push was developed over four iterations (Figure [[fig:push-iterations]]), each changing one
element:

- **Baseline.** The hand often landed beside its target, and the push slipped or disturbed the
  plates below.
- **Hand-eye calibration and identified arm model** (Sections 8.2 and 8.4) brought hand placement
  into agreement with the plan.
- **Tactile edge localisation.** Sliding the fingertip along the rim to find its edge moved the
  plate by up to 10 mm, so it was replaced by probing: the fingertip descends in 5 mm steps until
  contact.
- **Three-dimensional rim tracking.** On the robot, a pushed plate's foot rides onto the plate
  below, tilting it by about 5° and raising the overhanging rim by 16–17 mm (Figure
  [[fig:plate-pose]]). The tracker, which assumed a level plate, lost the rim and reported false
  slips. A plane fit to the rim (Section 8.2) fixed this and gives the receiving hand its grasp
  height.

![fig:push-iterations | Hooked-push outcomes by design iteration](../assets/interim/manip-push-iterations.webp)

With all four modifications, 4 of 5 pushes were completed with 35–37 mm displacement. The remaining
failure was caused by an incorrect stack-height estimate at the start, which is now cross-checked
against the tracker before the push. The receiving hand has since closed on the raised rim; lifting
is the next stage.

![fig:plate-pose | Plate pose after a 40 mm push](../assets/interim/manip-plate-pose.webp)

### 8.6. Shortcomings and Semester 2 Plan

- **Working-height adjustment.** Height-controlled locomotion policies decouple a
  reinforcement-learned leg policy, which tracks a commanded pelvis height, from upper-body control
  [30], [31]. A policy of this kind has been trained for the R1 in a parallel project (commanded
  pelvis height 0.40–0.74 m in simulation) and will be evaluated with the pick sequence for fixtures
  at other heights.
- **Force limit.** Measure the rim's breaking load in a static test and set the finger force limits
  from it.
- **Transfer to the table.** Compare carrying single plates, a tray (additional arm load) and a
  trolley (pushing skill; floor covering).
- **Reinforcement learning.** Train a policy that generalises the motion and covers edge cases.
