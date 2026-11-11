---
id: navigation
---

## 7. Autonomous Navigation and Localisation

### 7.1. Hardware

Continuing from Section 4.2 (Table [[tab:morph-navigation]], the morphological chart for the
navigation subsystem), we determined that the navigation subsystem needs two pieces of hardware: a
lidar and a depth camera. We will use a Hesai JT128 lidar and an Intel RealSense depth camera. These
two components are vital to the system, as they are the robot's main way of sensing and receiving
information from the real world around it.

Table: tab:nav-sensors | Navigation sensors
| Sensor | Model | What it outputs | What navigation uses it for |
| --- | --- | --- | --- |
| 3D lidar | **[TODO: model on the G1]**; Hesai JT128 on the R1 | A 360° point cloud of the surroundings, plus readings from its built-in IMU (accelerations and turn rates) | Odometry, drift correction and the obstacle map |
| Depth camera | Intel RealSense **[TODO: model]**, head-mounted (on the helmet on the R1) | A depth image: the distance to the scene at every pixel | Finding the table edge during the final approach |

**Why the lidar is vital.** It is the only sensor that sees all around the robot, out to **[TODO:
range]** m. Its point cloud tells the robot both where it is, by matching what it sees against a
map, and what is in its way. Its built-in IMU measures the robot's motion many times a second, which
gives the odometry used between scans.

**Why the RealSense is vital.** At the table, the robot must stop 30 cm from the edge and square to
it. **[TODO: one line on why the lidar alone cannot measure the edge precisely enough at close
range.]** The RealSense, pitched down at the tabletop, sees it densely and gives the distance and
angle of its edge directly.

All the work in this section was developed on a Unitree G1 borrowed from the BEADZ Lab, and will
move to the R1 next semester.

### 7.2. Inputs

The navigation subsystem takes three inputs: the two sensor feeds above, and a rough 2D map from the
VATEL planner. The sensors describe what the robot sees right now; the map describes the room it is
in and where the tables and the AGV are.

Table: tab:nav-inputs | Navigation inputs
| Input | Source | Input information | Input format | Limitation |
| --- | --- | --- | --- | --- |
| Point cloud and IMU readings | Lidar (Hesai JT128) and its built-in IMU | Point cloud: the 3D positions of the surfaces around the robot, such as walls, tables and people. IMU: the robot's accelerations and turn rates | Point cloud: .ply file (x, y, z per point). IMU: numbers (3-axis acceleration in m/s², 3-axis angular velocity in rad/s) | IMU odometry drifts over time (Section 7.3.1) |
| Depth image | RealSense depth camera | The distance to the scene at every pixel in front of and below the robot | 16-bit depth image, one distance value per pixel (in mm) **[TODO: confirm]** | Short range: useful only near the table |
| Rough map | VATEL planner | The room layout and the approximate positions of the tables and the AGV | .pgm image (2D occupancy grid) | Approximate: positions are off by up to **[TODO: error]**, and the robot starts each run without knowing where it is in this map |

Together these are everything the navigation subsystem needs. Its single output is a walking
command: a forward speed vx, a sideways speed vy and a turning rate (yaw), sent to the robot's
walking controller.

### 7.3. Subsystem Architecture

Seen from outside, navigation is a black box: three inputs go in, and one walking command comes out.

> TODO: add the figure fig:nav-black-box: Navigation as a black box:
> three inputs, one walking command.

Inside the box, navigation answers four questions in a fixed order, and each answer feeds the next:
where am I?, where should I go next?, what is the best path? and how do I walk there? The robot
cannot pick a destination before it knows where it is, or plan a path before it has a destination.

> TODO: add the figure fig:four-questions: The four navigation questions, answered in series.

Each question defines one function the subsystem must perform.

Table: tab:nav-functions | The four questions and their functions
| Question | Function | What the function does |
| --- | --- | --- |
| Where am I? | Localisation | Estimates the robot's pose (x, y, heading) in the map frame |
| Where should I go next? | Goal selection | Decides the next destination, the table or the AGV, and its coordinates |
| What is the best path? | Path planning | Finds the lowest-cost path from the current pose to the destination that avoids every obstacle |
| How do I walk there? | Path following | Turns the path into walking commands, and lines the robot up at the table edge |

The functions lead us to the actual system architecture of the whole navigation system. Each
function is carried out by its own module. Figure [[fig:nav-architecture]] shows the four modules
and the data passed between them.

![fig:nav-architecture | Navigation subsystem architecture](../assets/interim/nav-architecture.webp)

The lidar feeds both the localiser and the path planner's costmap. The current pose from the
localiser is also used by the path planner and the controller. Sections 7.3.1 to 7.3.4 cover these
modules one at a time.

Table: tab:nav-modules | Navigation modules, inputs and outputs
| Function | Module | Section | Takes in | Hands on |
| --- | --- | --- | --- | --- |
| Localisation | Localiser (odometry, accumulator, relocaliser) | 7.3.1 | Lidar point cloud and IMU readings | Current pose (x, y, heading) |
| Goal selection | Mission planner (state machine) | 7.3.2 | Rough map, detected table and AGV positions, current pose | Destination coordinates |
| Path planning | Path planner (costmap and A\*) | 7.3.3 | Lidar point cloud, current pose, destination | Path to the destination |
| Path following | Local controller (DWB, table fine approach) | 7.3.4 | Path, costmap, current pose; RealSense depth near the table | Walking command (vx, vy, yaw) |

#### 7.3.1. Localiser: Where Am I?

##### 7.3.1.1. What the Robot Needs to Know

The robot's position is its pose: coordinates x and y in metres and a heading θ, in a map frame. The
accuracy requirement comes from the task: the robot must stop 30 cm from a table edge to inspect it.
That 30 cm is the entire error budget, and every figure below should be read against it.

The robot starts fresh on every run. It takes its starting pose as the origin (0, 0, 0°) and builds
its map from nothing, because the room is rearranged between runs and a stale map is worse than no
map.

##### 7.3.1.2. Where the Pose Comes From

Odometry is produced by FAST-LIO2 [15], a tightly coupled lidar-inertial estimator. It is not plain
IMU integration. Odometry calculated from the IMU alone would be useless because of noise and
jitter. The IMU measures the robot's 3D acceleration in real time, including the acceleration due to
gravity. Integrating this acceleration twice over time gives the robot's displacement. Even a small
error in the measured gravity, such as from a 0.1° tilt error, is therefore magnified by the square
of time, so the drift keeps growing: about 0.9 m after 10 s, and over 400 m after four minutes of
walking.

Instead, FAST-LIO2 keeps a single filter state, holding the pose, the velocity and the IMU biases
themselves, and corrects it with every lidar sweep. Two mechanisms matter (details in Appendix 7B):

- **Backward propagation.** A lidar sweep takes about 100 ms, during which the robot moves and
  turns, so each point in the sweep is captured from a different pose. Treating the points as
  simultaneous bends straight surfaces. The IMU's high-rate measurements are used to place each
  point at the pose it was actually taken from, straightening the sweep before it is used.
- **The iterated EKF.** The IMU propagates the state forward between sweeps. Each de-skewed point is
  then matched to the map, and its point-to-plane residual updates the state. The filter
  re-linearises around the improved estimate and repeats, which behaves more like a Gauss–Newton
  optimisation than a single EKF update. Crucially, the update corrects the IMU biases as well as
  the pose, which stops the inertial error from compounding.

> TODO: add the figure fig:lio-mechanism: Backward propagation and the iterated EKF.

These mechanisms reduce the drift from growing with the square of time to growing with distance
travelled. Some drift still persists; its cause and its correction are covered in Section 7.3.1.3.

##### 7.3.1.3. Residual Drift and Periodic Correction: Relocaliser and Accumulator

Where does the residual drift come from? FAST-LIO2 registers each sweep against a local map that it
builds itself, incrementally, with no external reference, so small registration errors are written
into that map and inherited by every later pose.

Measured on this robot, drift accumulated at about 4% of distance walked (Section 7.3.1.4). Over a
58 m run that is more than 2 m of error, against a 30 cm budget. A second correction stage is
therefore required.

To fix the remaining drift, we implemented the relocaliser and accumulator. These two modules run
alongside the odometry and form a slow correction cycle around it.

![fig:localiser | Odometry corrected by the relocaliser against the accumulated map](../assets/interim/nav-localiser.webp)

**The accumulator** transforms each lidar scan into the map frame, using the pose at the time of the
scan, and adds it to a growing point-cloud map of the room. It does not insert every scan: insertion
is gated on the relocaliser's inlier ratio.

Table: tab:inlier-gate | Accumulator gate on the relocaliser's inlier ratio
| Inlier ratio | What it means | Scan inserted? |
| --- | --- | --- |
| Below 0.15 | No prior map exists yet | Yes, on odometry alone, to bootstrap the map |
| 0.15 to 0.50 | The pose is ambiguous | No |
| Above 0.50 | The pose is trusted | Yes |

Without this gate, a poor pose would be written into the map and then used to validate itself.

**The relocaliser** runs once per second. It accumulates five scans, downsamples them to at most
8,000 points, and registers them against the accumulated map by ICP. It runs 15 iterations with a
coarse-to-fine correspondence gate of 1.5 → 0.8 → 0.4 m, because a single narrow gate cannot recover
once the estimate is poor. The offset between where the scans best fit the map and where odometry
says the robot is, is the accumulated drift.

The correction is published as a separate transform rather than written back into the odometry. This
is the key design choice: the odometry keeps updating the pose at 10 Hz, keeping it smooth, while
the correction updates at 1 Hz, keeping the drift bounded. Downstream modules compose the two.

##### 7.3.1.4. Improvement Result

Each condition walked five 5 m out-and-back laps under Nav2, about 58 m per run, starting from the
same floor mark. The final position was then measured against that mark with a tape. The
odometry-only condition was produced by disabling the relocaliser's ICP (seed_only mode), so both
runs share an identical map frame and differ only in whether the correction was applied.

Laps are repeated because the navigation goal tolerance is 0.30 m: a single arrival may be that far
off for reasons unrelated to drift. Arrival error does not accumulate across arrivals, whereas drift
does, so over 58 m drift becomes the dominant term.

Table: tab:drift-test | Endpoint error after a 58 m run
| Test | Distance walked (m) | Position error, odometry only (cm) | Position error, with relocaliser (cm) |
| --- | --- | --- | --- |
| 5 × 5 m out-and-back | 57.6 / 57.7 | 18.9 | 2.2 |

> TODO: add the figure fig:drift-relocalizer: Relocaliser correction
> during the run, and endpoint error.

> TODO: add the figure fig:drift-paths: Estimated paths and measured endpoints, both conditions.

The relocaliser reduced position error from 18.9 cm to 2.2 cm, an improvement of about 8.6 times.
Against the 30 cm standoff requirement, this is the difference between using up two-thirds of the
error budget before the approach begins and leaving it essentially intact.

A second measurement supports this independently of the floor marker. The cumulative correction
applied by the relocaliser was consistent across two runs, at 4.0% and 4.3% of distance walked,
while the net shift of the map origin was under 1.5 cm. The relocaliser was therefore not displacing
the frame, but continuously cancelling an error that re-accumulated at about 4% of distance. For
comparison, published evaluations report 6.8% translation drift for FAST-LIO2 **[TODO: citation]**.

##### 7.3.1.5. Limitations and Further Work

- **The improvement is a lower bound.** The out-and-back route lets along-track error partly cancel,
  so 18.9 cm understates what odometry alone would accumulate on a one-way route.
- **The robot's map is not aligned with the rough map.** The VATEL planner's rough map has a fixed
  origin and orientation, whereas the robot's map begins wherever the robot happens to stand. Table
  positions from the rough map therefore cannot yet be used directly. Future work will localise the
  robot within the rough map at the start of each run, so that an approximate table position can be
  taken from the rough map and refined as the robot builds its own view.

#### 7.3.2. Mission Planner: Where Should I Go Next?

The mission planner decides the robot's next destination and sends its coordinates to the path
planner.

A state machine with two destinations. The mission planner is a state machine. At any time the robot
is heading to one of two places: the table or the AGV. After a plate is picked up from the AGV, the
next destination is the table; after the plate is placed, the next destination is the AGV.

> TODO: add the figure fig:mission-states: Mission planner states and transitions.

##### 7.3.2.1. Overwriting the Rough Map with What the Robot Sees

At the start of a run, the table and AGV positions come from the VATEL planner's rough map. These
are good enough to head in the right direction, but not to stop 30 cm from a table edge.

So once the robot's own sensors detect the table or the AGV, and the planner is confident in the
detection, the measured position overwrites the one from the rough map. **[TODO: what "confident"
means, e.g. the same position seen in N consecutive scans, within X cm.]** From then on, the robot
plans to where the table actually is, not to where the map says it was.

##### 7.3.2.2. Sending the Destination

The chosen destination is sent to the path planner as coordinates in the map frame: x, y and the
heading the robot should face on arrival. **[TODO: confirm whether a table destination is an
approach point in front of the edge rather than the table itself, and at what distance.]**

##### 7.3.2.3. Limitations and Future Work

The rough map is not yet connected to the navigation system. We are still resolving where the robot
starts within that map (Section 7.3.1.5), so for now the table and AGV positions come only from the
robot's own detection. Additional work is needed to connect the two.

Once connected, the rough map will still be approximate: it is built from the CCTV view, and a table
or the AGV may have moved since it was made. Future work will measure the difference between the
rough map and the lidar map, to find how far the rough map's positions can be trusted.

#### 7.3.3. Path Planner: What Is the Best Path?

The robot now knows where it is and where it is going. The path planner finds the best route between
the two that avoids every obstacle. It needs two things: a map of all the obstacles around the
robot, not only the tables, and an algorithm that searches that map for a path.

##### 7.3.3.1. Building the Map

The map is built from the lidar point cloud. Not every point is an obstacle: points on the floor are
not, and neither are points above the robot's head. The cloud is first filtered to keep only points
in a height band from 0.20 m to 1.60 m above the floor, the heights at which something would block
the robot. Scans beyond 4 m are also discarded, as they are too sparse to trust, and the robot's own
footprint is cleared explicitly so that it never marks itself as an obstacle.

> TODO: add the figure fig:point-filtering: Point filtering by height, side view.

The remaining points are projected onto a 2D grid of 5 cm cells covering 60 × 60 m, and each cell
holding a point is marked as an obstacle. The grid is published continuously as a ROS topic
containing the occupancy grid, so that it can be updated while the robot drives, which Section
7.3.3.2 depends on.

> TODO: add the figure fig:grid-map: Example 2-D grid map.

The obstacles are then inflated with a graded cost rather than a single radius. For each cell, with
d the distance to the nearest obstacle cell:

Table: tab:inflation | Costmap inflation
| Distance | Cost | Effect |
| --- | --- | --- |
| d = 0 | 254 | The obstacle itself |
| d ≤ 0.25 m | 253 | Impassable: A\* refuses to enter |
| d ≤ 1.50 m | 252·exp(−1.5·(d − 0.25)) | Passable but exponentially expensive |
| Beyond | 0 | Free |

So there is a hard block within 0.25 m of any obstacle, and driving close to one costs more.

##### 7.3.3.2. Two Iterations to Improve the Map Quality

Iteration 1: a map that only adds. The first map only ever added obstacles: once a cell was marked,
it stayed marked. This works for walls and tables, which do not move, but not for anything that
does. A person walking past the robot left a trail of obstacle cells along their route that never
disappeared, and with each cell inflated to a 0.50 m-wide impassable band, those trails soon block
off corridors that were actually clear.

> TODO: add the figure fig:human-trail: Costmap with a person's trail.

Iteration 2: a map that clears what the lidar sees through. The second map removes obstacles, using
the fact that a lidar beam is evidence about everything it passes through. Each beam travels in a
straight line from the sensor to the point it hits, so every cell along the way must be empty: the
beam would have stopped there otherwise.

Each cell therefore falls into one of three states each cycle, and the third is the important one:

Table: tab:beam-evidence | Cell evidence from lidar beams
| What the beams say | Effect on the cell |
| --- | --- |
| Hit at least once | Evidence +1 |
| Swept through by a beam, not hit | Evidence −1 |
| Neither hit nor swept | Unchanged |

Three refinements make the rule behave:

- **Beams stop one cell short of what they hit.** Nothing beyond a return is ever counted as empty,
  so a beam striking a wall cannot clear the room behind it.
- **The height band is checked along the beam, not only at its end.** The beam's height is
  interpolated over its length, and only the portion within 0.20–1.60 m contributes clearing.
  Without this, a beam passing above a table and striking the far wall would clear the table's own
  cells on the way through.
- **Evidence accumulates over a 2 s window rather than per scan.** The lidar returns about 1,240
  points per scan against 14,400 cells in a 6 m area, so no single scan can touch most of them;
  charging a penalty per scan erodes real walls faster than they are re-observed.
  Counter-intuitively, a finer grid makes this worse, not better.

A cell is finally shown as an obstacle once its evidence reaches 2, and is removed only when it
decays to 0.

The video shows the result. A person's trail disappears within about 6 to 12 seconds of their
leaving, while a wall observed continuously reaches full evidence and survives up to 60 seconds of
complete occlusion. The decay calculation is given in Appendix 7A.

![fig:costmap | Permanent trails (left) and decaying trails (right)](../assets/interim/nav-costmap.webp)

> TODO: add the video of the map clearing as a person leaves.

##### 7.3.3.3. Finding the Path: A\* with Replanning

The path planner uses A\* [16] on this map, implemented in the Nav2 navigation framework [17]. A\*
searches outwards from the robot's cell towards the destination, to find the lowest-cost path that
never enters an impassable cell.

Because the map changes as people move, a path that was clear can become blocked, so the planner
replans every 1.5 s from the robot's current pose. Replanning alone is not enough: two paths of
similar cost could swap back and forth, so a rule decides when the path changes. A new path is
therefore adopted only if it is at least 15% shorter and saves at least 0.30 m. The current path is
reconsidered after 10 s regardless, so a stale choice cannot persist indefinitely.

Replanning stops entirely within 1.0 m of the goal. Near the destination the benefit of a better
route is negligible.

> TODO: add the video of the path replanning.

#### 7.3.4. Local Controller: How Do I Walk There?

The path is a line on a map, but the robot only reads walking commands. The local controller turns
the path into a forward speed, a sideways speed and a turning rate, at 5 Hz.

##### 7.3.4.1. The DWB Controller

We use Nav2's DWB controller, based on the dynamic window approach [18]. Every control cycle, 5
times a second, it:

- Works out the dynamic window: the speeds the robot can actually reach before the next cycle, given
  its current speed and its acceleration limits.
- Samples candidate commands (vx, vy, yaw rate) from inside that window.
- Simulates where each candidate would take the robot over the next 3 s.
- Scores each simulated trajectory on how closely it follows the path, how much progress it makes
  towards the goal, and how close it comes to obstacles.
- Sends the best-scoring command, for example a forward speed of 0.25 m/s while turning at 0.17
  rad/s.

The command passes through a Nav2 bridge and a ROS 2 controller to Unitree's walking SDK, which
turns it into steps.

##### 7.3.4.2. Fine Approach at the Table Edge

DWB stops the robot near its goal, within a 30 cm radius, but placing a plate needs it square to the
table and 30 cm from its edge. So near the table, the controller hands over to a fine approach in
three moves:

- **Turn square.** The robot turns on the spot until it faces the table edge squarely, using the
  heading from the localiser.
- **Step in.** With the RealSense on, it steps forward slowly until the edge is 30 cm away.
- **Correct the heading.** It makes a final turn to remove the angle that remains between it and the
  edge.

The RealSense gives the distance and angle used in moves 2 and 3. The camera is 1.262 m above the
floor and pitched down at 63.6°, so the top of a 0.76 m table is 0.502 m below it. The stack
segments the tabletop from everything else. It then fits a line to the near edge of the tabletop,
the distance from the robot to that line is the edge distance, and the line's angle is the heading
correction.

![fig:plane-detector | Tabletop edge from the RealSense plane detector](../assets/interim/nav-table-edge.webp)

### 7.4. Full Navigation Workflow for a Rectangular Table

#### 7.4.1. Open Issues and Next Steps

The workflow in Section 7.4.2 is the target design, not what the robot does today. The issues below
are still open, and the first three gate the full workflow: until they are resolved, the robot
cannot start from the rough map, so the table approach has so far been tested without it (Section
7.5). Resolving them is the next step for the navigation subsystem.

Table: tab:nav-issues | Open issues and next steps
| Open issue | What it blocks | Next step | Section |
| --- | --- | --- | --- |
| The robot does not know where it starts in the rough map. Its own map begins wherever it stands, so the two maps are not aligned. | Steps 1 to 3: the table's position from the rough map cannot be placed in the robot's map. | Localise the robot within the rough map at the start of each run. | 7.3.1.5 |
| The rough map is not yet connected to the navigation system. | Step 2: the mission planner cannot read the table's rectangle. | Connect the VATEL planner's rough map to the mission planner. | 7.3.2.3 |
| All work so far ran on a Unitree G1 borrowed from the BEADZ Lab. | Every step, on the final robot. | Move the stack to the R1, with the JT128 lidar and the helmet-mounted RealSense, and retest it there next semester. | 7.1 |

#### 7.4.2. Target Workflow

With all four functions in place, a trip to a rectangular table runs as follows.

- **Start.** The localiser starts tracking the robot's pose from its starting position.
- **Choose the table.** The mission planner's state is "go to table". It reads the table's rectangle
  from the rough map: its centre, its length and width, and its orientation.
- **Choose the approach point.** The mission planner picks the side of the table to approach. The
  destination is a point **0.5** m out from the middle of that side, facing the table.
- **Plan the path.** The path planner runs A\* from the robot's pose to the approach point on the
  current costmap, and replans every **1.5** s.
- **Walk the path.** DWB follows the path, steering around people and other obstacles that appear on
  the costmap.
- **Update the table position.** Once the robot's sensors detect the table with confidence, its
  measured position overwrites the rough one. The approach point moves with it and the path is
  replanned.
- **Fine approach.** At the approach point, the robot turns square to the edge, steps in until the
  edge is 30 cm away, and makes a final heading correction.
- **Hand over.** The robot stops, and **plate manipulation** takes over.
- **Next destination.** Once the plate is placed, the mission planner switches to "go to AGV", and
  steps 3 to 7 repeat for the AGV.

### 7.5. Partial Tests and Results

> TODO: carried over from the previous draft; fit it into the new flow.

We tested the table approach motion (CCTV map not included) in the Tinkering Corner and the Sandbox
of the Innovation & Design Hub, approaching different tables from each side. The robot found each
table, navigated to it without hitting an obstacle, aligned with it, and stopped consistently at the
distance best suited to placing a plate.

Table: tab:approach-tests | Table approach tests
| Location | Runs | Stop distance from edge, mean ± SD (cm) | Heading error, mean ± SD (°) | Collisions |
| --- | --- | --- | --- | --- |
| Tinkering Corner | **2** |  |  |  |
| Sandbox | **2** |  |  |  |

This is the evidence for specification S5. **[TODO: one sentence comparing the results with the S5
target.]**
