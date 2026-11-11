---
id: manipulation
---

## 8. Plate Perception and Manipulation

### 8.1. Requirements, Scope and Design Decisions

The partner's tableware inventory (Appendix E) lists 28 items, of which the 10.5-inch round
porcelain dinner plate is the most numerous (3,000 pieces); development therefore began with this
item. Porcelain plates of matching diameter (255 mm) were procured as test specimens, and the
remaining dimensions used in this section (22 mm height, 522 g mass, 5.5 mm rim thickness) are those
of the test plates (Figure [[fig:plate-geometry]]). Two further constraints follow from the
partner's operation:

- **Stacked delivery.** Plates arrive in nested stacks, so a single plate must be separated from the
  stack. Nested plates are designed to stack compactly, leaving a gap of only 0.5–2.0 mm between
  adjacent rims on the test plates.
- **Undefined source fixture.** Stacks are collected from the AGV, but the holder it carries (rack,
  shelf or dish dolly) is not yet specified.

![fig:plate-geometry | Test plate: (a) cross-section; (b) two nested plates](../assets/interim/manip-plate-geometry.webp)

In the absence of a defined fixture, the base case adopted is a stack resting on a flat horizontal
surface, realised in the laboratory as a table. Because the eventual fixture may present the stack
at a different height, the working height of the robot relative to the stack is treated as
adjustable; the means of adjustment is addressed in Section 8.6.

Table [[tab:pick-criteria]] lists the criteria used to evaluate candidate designs, and Figure
[[fig:pick-decisions]] summarises the design decisions developed in Sections 8.3 and 8.4.

Table: tab:pick-criteria | Evaluation criteria for plate picking
| Criterion | Value | Basis |
| --- | --- | --- |
| Object | 10.5-inch round porcelain plate (255 mm test plate) | Partner inventory |
| Initial state | Nested stack of 2–5 plates on a flat surface | Stacked delivery; fixture not yet defined |
| Separation | Top plate displaced 30–70 mm; plates below displaced < 5 mm | a) Lower bound: rim exposure required by the receiving hand. b) Upper bound: an 85 mm push tipped the plate off the stack in simulation, so 70 mm is used to keep a safety margin.  c) The remaining stack must stay where the receiving hand and the next pick expect it |
| Contact force | ≤ 30 N hold load and ≤ 60 N peak per finger | An estimate from the flexural strength of porcelain [19] places rim fracture at roughly 120–165 N for a fingertip-sized load, a factor of about 2 above the peak limit (Appendix 8B) |
| Pose uncertainty | ±15 mm lateral between camera estimate and hand | Measured camera and arm errors (Section 8.2) |

![fig:pick-decisions | Design decisions for single-plate picking](../assets/interim/manip-pick-decisions.webp)

### 8.2. Plate Perception

**Stack localisation.** A RealSense D435i depth camera on the head localises the stack once, before
any motion (Figure [[fig:stack-detection]]). The pipeline was revised several times in response to
failures observed on real images; Table [[tab:stack-failures]] lists the options considered for each
function and the basis for the selection.

![fig:stack-detection | Detected stack: outline, height, count and position](../assets/interim/manip-stack-detect.webp)

Table: tab:stack-failures | Stack localisation failures and fixes
| Failure observed | Change made | Result |
| --- | --- | --- |
| Glare on the glaze removes depth returns, particularly in the well | Outline found from colour edges; depth sampled on the rim only | Height and centre unaffected by missing well depth |
| At oblique viewing angles the rim projects as an ellipse; a circle model gave diameter errors of up to 21% | Ellipse fit [20] | Diameter 249–256 mm on real images of 2–4 plates (true value 255 mm) |
| On the first real images the rim appeared as a ring with gaps, the outline followed both edges of the ring, and the diameter read about 200 mm | Outline extraction revised to follow the outer rim edge | Centre of a stationary stack repeatable to 0.25–0.30 mm (standard deviation) |
| The camera saw the level table tilted by 2.5°, because the robot was leaning forward (IMU pitch 3.0°) | Plane fitted to the table around the stack (RANSAC [21]); heights measured relative to it | Heights unaffected by the robot's lean and residual camera-pose error |
| The well forms a second, lower height cluster; a high percentile of depth was biased upward by sensor noise | Height taken from the topmost cluster (the rim), as a median re-centred until stable | Stable top-plate height |
| Counting edges on the side of the stack under-counted tall stacks | Plate count = stack height ÷ nesting pitch | Exact for 1–20 plates in synthetic tests |

**Hand-eye calibration.** While the camera estimates the plate pose visually, hand placement relies
on joint kinematics; accurate manipulation therefore requires both to be expressed consistently in
the robot frame. Initial testing revealed a significant discrepancy: overlaying the kinematically
predicted hand position on the camera image showed a lateral offset of 41.7 mm (Figure
[[fig:hand-eye-discrepancy]]). Three sources were identified and corrected.

- **Camera mounting pose.** The original calibration measured only the camera's pitch and roll, and
  took its position and heading from the CAD model. The camera was re-calibrated over 15 head poses
  using the IMU's gravity vector as reference. Its downward pitch agreed with the CAD value of 30°,
  but the camera sat 30 mm to the side of its CAD position and was turned by 1.2°. The held-out
  error after recalibration was 1.8 mm RMS.
- **Head yaw play.** Depending on the direction from which the head reached a given yaw angle, the
  hand appeared to shift by up to 13 mm in the image. The arm server now completes every head-yaw
  move from the same direction. After both corrections, the table height seen by the camera agreed
  with the height touched by the hand to within 1.4 mm across 38 head poses, with no remaining
  dependence on head yaw (previously up to ±9 mm at ±20° of yaw).
- **Arm joint offsets.** Each hand was driven to touch the table at nine locations, with contact
  detected by the fingertip proximity sensors. The difference between the table height observed by
  the camera and the height at which the hand made contact gives the kinematic error at each
  location; constant offsets of up to 3.6° on four joints per arm were fitted to these errors by
  least squares.

With these corrections, the mean height error fell from 12.6 to 2.3 mm for the left hand and from
4.5 to 3.7 mm for the right (Figure [[fig:height-error]]). Because a horizontal table constrains
height only, lateral error is reduced but not eliminated; the residual is handled by tactile edge
localisation before each push (Section 8.5).

![fig:hand-eye-discrepancy | Hand-eye offset before calibration (41.7 mm)](../assets/interim/manip-hand-eye.webp)

![fig:height-error | Camera-to-hand height error, before and after calibration](../assets/interim/manip-height-error.webp)

**Rim tracking.** During manipulation, a rim tracker developed for this project follows the top
plate at camera rate (10–30 Hz). It keeps the depth points near the rim's expected height, removes
points belonging to the robot's own arm and hand, finds the rim's outer edge around the plate and
fits a circle of known radius to it, giving the plate centre; a plane fitted to the rim heights
gives the plate's tilt (Figure [[fig:rim-tracker]]). The state machine uses the tracker to tell
plate motion from fingertip slip. The method is given in Appendix 8A, and the reason the tilt is
measured is explained in Section 8.5.

![fig:rim-tracker | Rim tracker on a 3-plate stack](../assets/interim/manip-rim-tracker.webp)

**Wrist cameras.** Two limitations remained with the head camera: a residual lateral error of up to
about 10 mm, and occlusion of the gripped rim by the hand itself. Each Revo 2 hand already carries a
small colour camera on the palm side, which had not been used; it was brought in to address these
limitations. A model-based tracker (Figure [[fig:wrist-camera]]) projects the expected rim into the
wrist image from the hand pose and the known plate geometry, searches for the rim edge along lines
normal to the projection, and corrects the rim's position and height. In simulation, a 12 mm initial
position error was reduced to 2.7 mm. The wrist camera's mounting pose is calibrated against the
head camera using a checkerboard viewed from planned poses, recovering the pose to 2.9 mm and 0.6°
in simulation. Robot integration is in progress.

![fig:wrist-camera | Wrist-camera rim tracking in simulation](../assets/interim/manip-wrist-camera.webp)

### 8.3. Single-Plate Separation: Challenges and Concept Selection

A single plate can be clamped between two hands, but a nested stack offers no grasp point: the
0.5–2.0 mm gap between rims is far smaller than the 14 mm fingertip. Observation of human operators
identified three strategies (Figure [[fig:human-strategies]]): inserting a fingernail into the gap,
pinching the top rim between finger and thumb, and drawing the plate outward with the thumb. The
first two grasp the plate in place; the third displaces the plate to create a grasp point.

![fig:human-strategies | Human strategies: fingernail, rim pinch, thumb draw](../assets/interim/manip-human-strategies.webp)

None of these strategies transfers directly to the R1. Each arm has five joints and no wrist pitch,
so in simulation the fingertip approaches the gap 54° from the required angle (Figure
[[fig:r1-arm]]); each hand has 6 actuated joints against approximately 24 in the human hand, and the
rigid fingertips have no force sensing at the tip.

![fig:r1-arm | R1 arm (left) and simulated fingertip approach (right)](../assets/interim/manip-r1-reach.webp)

The plate must therefore be displaced before it can be grasped. Pre-grasp manipulation of thin
objects has been demonstrated by sliding a single object to a table edge [22] or pushing it against
a wall [23], in both cases with a gripper and a wrist. A stack provides no such edge, so the plate
beneath serves as the support instead. Concepts were evaluated in MuJoCo using a plate model built
from the measured profile and initial friction estimates (Table [[tab:friction]]). Pressing down on
the near rim to tip the far rim upward (Figure [[fig:rejected-press]]) was rejected: the near rim
can drop only by the rim gap before it meets the plate below, so the far rim rose by only 3–8 mm,
against the approximately 16 mm required for finger entry. Pushing the plate sideways instead causes
its far side to overhang and rise over the plate beneath; the opposite hand then grasps the exposed
rim and the pushing hand regrasps.

![fig:rejected-press | Rejected concept: pressing the near rim](../assets/interim/manip-rejected-press.webp)

Three pushing contacts were compared in simulation (Figure [[fig:push-contacts]], Table
[[tab:push-contacts]]).

![fig:push-contacts | Pushing contacts: (a) pinky end; (b) fingertip drag; (c) hooked finger](../assets/interim/manip-push-contacts.webp)

Table: tab:push-contacts | Pushing contacts in simulation
| Contact | How it moves the plate | Main difficulty | Decision |
| --- | --- | --- | --- |
| (a) Pinky end on the rim edge | Fingertip pushes the rim edge | The pushed edge sinks and then rises as the plate slides; following it needs wrist roll and inward motion together, which was hard to tune, and when the fingertip failed to follow it landed on the plate below | Rejected |
| (b) Index–middle fingertip drag | Friction under pads pressed on the rim | Relies entirely on fingertip friction, which was unknown at the time | Deferred; re-evaluated after friction identification (Section 8.4) |
| (c) Hooked middle finger on the outer edge | Side of the curled finger bears on the rim's outer edge | Does not rely on friction; fingertip stays above the plate below; tolerated −15 to +20 mm position error | Selected |

The hooked contact was therefore developed first (Figure [[fig:hooked-push]]). The drag contact was
not rejected: once the fingertip friction had been identified on the robot (Section 8.4), it became
a candidate again.

![fig:hooked-push | Hooked push on the R1, side view](../assets/interim/manip-hooked-push.webp)

**Receiving-hand grasp.** Once the rim overhangs, the opposite hand must grasp a 5.5 mm-thick rim
edge with a hand whose fingertip pads cannot oppose one another closely. Four grasps were evaluated
in simulation (Figure [[fig:receiving-grasps]], Table [[tab:receiving-grasps]]).

![fig:receiving-grasps | Receiving-hand grasps: (a) pinch; (b) curl under; (c) palm under; (d) finger slot](../assets/interim/manip-receiving-grasps.webp)

Table: tab:receiving-grasps | Receiving-hand grasps in simulation
| Grasp | Outcome | Decision |
| --- | --- | --- |
| (a) Thumb–middle fingertip pinch | Thumb and middle pads cannot close to less than 19 mm, against a 5.5 mm rim; the plate slipped out | Rejected |
| (b) Fingers curled under the rim, palm down | Curling moves the fingertips down beside the rim, not under it; every approach path passed through the plate by at least 10 mm | Rejected |
| (c) Palm under the plate, thumb closing from above | Lifted one side, but sliding deeper to close the thumb pushed the plate back or stood it up to 40°; the thumb pose was unreachable (13–15° IK error) | Deferred: potentially more stable; to be revisited with a longer push |
| (d) Rim inserted into the index–middle finger slot, thumb closing | The slot narrows to 5.9 mm near the knuckles, wedging the 5.5 mm rim; held in 14 of 14 simulated trials | Selected |

### 8.4. Control Approach, Simulation and System Identification

**Control approach.** Three control approaches were compared (Table [[tab:control-approaches]]). No
published work was found that separates nested plates with a dexterous hand by either learning
method. A scripted state machine was therefore implemented first (Figure [[fig:pick-sequence]]), to
establish feasible motions and to characterise the robot's real behaviour; reinforcement learning is
being developed in parallel to generalise these motions, with imitation learning retained as a
fallback.

Table: tab:control-approaches | Control approaches
| Approach | Strengths | Limitations for this task | Role |
| --- | --- | --- | --- |
| Scripted state machine | Interpretable; every step verified by a sensor check; no training data | Hand-tuned; limited generalisation | Baseline; implemented |
| Reinforcement learning (RL) | Generalises across conditions through training in simulation | Depends on accurate contact simulation and reward design | Parallel development |
| Imitation learning, e.g. diffusion policy [24] | Learns directly from demonstrations; represents multimodal actions | Requires a teleoperation set-up and demonstration data for a dexterous push | Fallback |

![fig:pick-sequence | Pick sequence in simulation](../assets/interim/manip-state-machine.webp)

**Simulator selection.** MuJoCo [25] was selected for both motion development and reinforcement
learning (Table [[tab:simulators]]). In a comparison of five physics engines, MuJoCo performed best
on the robotics-related tests [26]. GPU implementations of MuJoCo provide the parallel throughput
needed for reinforcement learning, and policies trained in them have transferred to real robots,
including a dexterous hand [27]; this project uses MuJoCo Warp, a GPU implementation of MuJoCo.
Isaac Lab [28] was used for early reinforcement learning experiments, but modelling the plate
required extra work: its mesh collision was coarser (about 1 mm) than the 0.6 mm slack in the slot
grasp, so the plate had to be rebuilt from convex shapes, and scenes with several free plates
exhausted GPU memory. Using one engine for development, system identification and training means the
identified robot model (below) applies directly to training.

Table: tab:simulators | Simulator comparison
|  | MuJoCo (CPU) | MuJoCo Warp (GPU) | Isaac Lab |
| --- | --- | --- | --- |
| Use in this project | Motion design, evaluation, replay of robot logs, system identification | Reinforcement learning (current) | Early reinforcement learning experiments |
| Strength | Accurate, stable contact [26] | Thousands of parallel environments with the same physics model | Thousands of parallel environments |
| Limitation found | One environment per CPU core | GPU memory limits the number of plates per scene | Plate collision coarser than the grasp clearance; out of GPU memory with 3–5 free plates |

**Validation in simulation.** Before any motion was run on the robot, it was evaluated in MuJoCo
under the errors measured on the R1: lateral camera error, encoder zero error and arm joint
friction, on the test table. A motion was taken to the robot only if it succeeded across these
conditions within the force limits. The conditions and results are given in Appendix 8E.

**System identification.** Under identical commands, the R1's hand deviated by 13.0 mm RMS from the
simulated hand. The R1's arm gravity loading, joint friction and a 25 ms command latency were
identified from robot logs and added to the simulation. Joint friction was modelled with the LuGre
model [29], in which friction depends on recent motion as well as on speed; this matched the R1's
low friction just after a change of direction. The identified model reduced the index fingertip
error from 14.2 to 4.0 mm RMS (Figure [[fig:fingertip-error]]). The procedure is given in Appendix
8C.

![fig:fingertip-error | Fingertip error: default 14.2 mm vs identified 4.0 mm RMS](../assets/interim/manip-finger-error.webp)

**Contact friction.** No published value exists for the Revo 2 fingertip on glazed porcelain, so the
simulation began with estimates (Table [[tab:friction]]). The fingertip friction was then identified
by replaying ten robot pushes in MuJoCo with their logged commands over a range of friction values
and comparing the simulated plate motion with the camera record (Figure [[fig:friction-id]],
Appendix 8D). The robot data require a fingertip friction of at least 0.75, most likely 1.0–1.3,
three to five times the initial estimate. At this value the drag contact of Section 8.3 becomes
feasible and is being re-evaluated.

Table: tab:friction | Contact friction: initial and identified
| Contact pair | Initial value in simulation | Source | Identified value | Method |
| --- | --- | --- | --- | --- |
| Revo 2 fingertip on glaze | 0.25 | Estimate for the fingertip material; not measured | ≥ 0.75, most likely 1.0–1.3 | Replay of ten R1 pushes (MuJoCo) |
| Plate foot on glaze (plate on plate) | 0.575 | Estimate | About half the fingertip value; absolute value needs a tilt test | Same replay (constrains the ratio) |
| Plate foot on table | 0.45 | Estimate | ≥ 0.90 (lower bound) | Tilt test on the table |

![fig:friction-id | Fingertip friction identified from a replayed push](../assets/interim/manip-friction-id.webp)

### 8.5. Robot Deployment and Experimental Iteration

Simulation and the robot are linked through a fitting loop (Figure [[fig:sim-to-robot]]): every
robot run is logged and replayed in simulation, discrepancies are used to refine the model, and only
parameter sets that pass in simulation are deployed. On the robot, the control loop runs as shown in
Figure [[fig:control-loop]]. The head camera's plate pose (10–30 Hz) and the joint and fingertip
proximity readings feed the state machine, which selects each step and its targets; the motion
planner converts them to joint targets by inverse kinematics; and the arm server streams the targets
to the arm and hand motors at 50 Hz while enforcing joint, speed and force limits, returning the
arms to a safe pose on any fault.

![fig:sim-to-robot | Simulation-to-robot fitting loop](../assets/interim/manip-sim-to-robot.webp)

![fig:control-loop | Control loop on the R1](../assets/interim/manip-control-loop.webp)

The push was developed over four iterations (Figure [[fig:push-iterations]]), each modifying one
element of the design in response to the preceding trials:

- **Baseline.** The hand frequently landed beside its target, and the push either slipped or
  disturbed the plates below.
- **Hand-eye calibration and identified arm model** (Sections 8.2 and 8.4) brought hand placement
  into agreement with the plan.
- **Tactile edge localisation.** Locating the rim edge by sliding the fingertip along it displaced
  the plate by up to 10 mm before the push began. It was replaced by sequential probing: the
  fingertip descends at 5 mm intervals until contact is detected.
- **Three-dimensional rim tracking.** Robot trials showed that a pushed plate does not remain level:
  its foot rides onto the plate below, tilting it by approximately 5° and raising the overhanging
  rim by 16–17 mm (Figure [[fig:plate-pose]]). The original tracker assumed a level plate, lost the
  rim and reported false slips. A plane fit to the rim was added to the tracker (Section 8.2), which
  also supplies the height at which the receiving hand must grasp.

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
