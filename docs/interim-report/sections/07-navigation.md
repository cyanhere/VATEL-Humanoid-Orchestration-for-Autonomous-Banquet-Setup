---
id: navigation
---

## 7. Autonomous Navigation and Localisation

### 7.1. Inputs and Outputs

Development and every test in this chapter use a Unitree G1 fitted with a Livox MID-360 LiDAR. The
final demonstration will run on the Unitree R1, with the Hesai JT128 on the sensor helmet (Chapter
6).

Navigation takes three inputs and produces one output (Figure [[fig:nav-black-box]]). The LiDAR and
depth camera on the sensor helmet (Chapter 6) and the VATEL Planner's rough map (Chapter 5) go in,
and a walking command comes out. Table [[tab:nav-io]] gives what each signal carries and its format.

![fig:nav-black-box | Navigation as a black box](../assets/interim/nav-black-box.webp)

Table: tab:nav-io | Navigation inputs and output
| Signal | Information | Format |
| --- | --- | --- |
| Point cloud and IMU | 360° view of surrounding surfaces (walls, tables, people) up to 20 m; the robot's accelerations and turn rates | Point cloud: x, y, z per point. IMU: 3-axis acceleration (m/s²) and angular velocity (rad/s) |
| Depth image | Distance to the scene ahead of and below the robot, used near the table | 16-bit image, one distance per pixel (mm) |
| Rough map | Room layout and approximate table and AGV positions, later overwritten by what the robot detects (Section 7.2.2.1) | .pgm occupancy grid with .yaml metadata |
| **Walking command (output)** | Forward speed vx, sideways speed vy and turn rate (yaw) | Velocity command, read at 5 Hz |

### 7.2. Subsystem Architecture

Navigation answers four questions in order, each answer feeding the next: where am I, where should I
go next, what is the best path, and how do I walk there? Each question is handled by its own module
(Figure [[fig:nav-architecture]], Table [[tab:nav-modules]]).

![fig:nav-architecture | Navigation subsystem architecture: four modules and the data passed between them](../assets/interim/nav-architecture.webp)

Table: tab:nav-modules | Navigation modules, the question each answers, and the data they exchange
| Question | Module | Section | Takes in | Hands on |
| --- | --- | --- | --- | --- |
| Where am I? (localisation) | Localiser (odometry, accumulator, relocaliser) | 7.2.1 | LiDAR point cloud and IMU readings | Pose (x, y, heading) in the map frame |
| Where should I go next? (goal selection) | Mission planner (state machine) | 7.2.2 | Rough map, detected table and AGV positions, pose | Destination: the table or the AGV |
| What is the best path? (path planning) | Path planner (costmap and A\*) | 7.2.3 | LiDAR point cloud, pose, destination | Lowest-cost path that avoids every obstacle |
| How do I walk there? (path following) | Local controller (DWB, table fine approach) | 7.2.4 | Path, costmap, pose; RealSense depth near the table | Walking command (Section 7.1) |

#### 7.2.1. Localiser: Where Am I?

##### 7.2.1.1. What the Robot Needs to Know

The robot's pose is stored as x, y and heading θ in a map frame. The robot must stop 20 cm from the
table edge, so 20 cm is the entire error budget.

The robot also starts fresh on every run, taking its starting pose as the origin (0, 0, 0°), because
the room is rearranged between runs.

##### 7.2.1.2. Where the Pose Comes From

Pure inertial odometry would be useless because of sensor noise and jitter. The IMU measures the
robot's 3D acceleration, and integrating this acceleration twice over time gives the robot's
displacement. Even a small error in the acceleration measured will be magnified by the square of
time, so the drift keeps growing.

FAST-LIO2 [18] avoids this by keeping the pose, velocity and IMU biases in one filter, corrected
with every LiDAR sweep. It de-skews each sweep using the IMU (Figure [[fig:lio-backprop]]), then
matches it to its map in an iterated EKF that also corrects the IMU biases (Figure [[fig:lio-iekf]],
Appendix 7A).

![fig:lio-backprop | Backward propagation · de-skewing one LiDAR sweep](../assets/interim/nav-lio-backprop.webp)

![fig:lio-iekf | The iterated EKF · one sweep's update loop](../assets/interim/nav-lio-iekf.webp)

This makes drift grow with distance travelled rather than with the square of time, but does not
remove it (Section 7.2.1.3).

##### 7.2.1.3. Residual Drift and Periodic Correction: Relocaliser and Accumulator

Drift remains because FAST-LIO2 matches each sweep against a map it builds itself, with no external
reference, so small registration errors are written into the map and inherited by every later pose.

Measured on this robot, it accumulated at about 4% of distance walked: over 2 m on a 58 m run,
against a 20 cm budget.

We therefore added two modules that form a slow correction cycle around the odometry (Figure
[[fig:localiser]]):

![fig:localiser | Odometry corrected by the relocaliser against the accumulated map](../assets/interim/nav-localiser.webp)

**The accumulator** adds each LiDAR scan to a growing point-cloud map of the room, but only when the
relocaliser trusts the current pose, so a poor pose is never written into the map (Appendix 7B).

**The relocaliser** registers the latest scans against the accumulated map by Iterative Closest
Point (ICP) once per second. The offset between the best fit and the odometry pose is the
accumulated drift (Appendix 7B).

The correction is published as a separate transform rather than written into the odometry, so the
pose stays smooth at 10 Hz while drift is corrected at 1 Hz.

##### 7.2.1.4. Improvement Result

We compared two runs, with and without the relocaliser. Each walked five 5 m out-and-back laps
(about 58 m) from a floor mark (Figure [[fig:start-position]]), and after every lap the robot's
position was taped on arrival and compared with its own estimate (procedure in Appendix 7B).

![fig:start-position | Starting position of robot](../assets/interim/nav-start-position.webp)

Table: tab:drift-test | Position error at the start mark on arrival after lap 5
| Run | Distance walked (m) | Position error (cm) | Ending point |
| --- | --- | --- | --- |
| 5 × 5 m out-and-back, (relocaliser on) | 57.7 | 2.4 | Figure [[fig:end-reloc-on]] |
| 5 × 5 m out-and-back, (relocaliser off) | 57.6 | 8.7 | Figure [[fig:end-reloc-off]] |

![fig:end-reloc-on | Robot at the start mark after lap 5, relocaliser on](../assets/interim/nav-end-reloc-on.webp)

![fig:end-reloc-off | Robot at the start mark after lap 5, relocaliser off](../assets/interim/nav-end-reloc-off.webp)

The relocaliser cut the end-point error from 8.7 cm to 2.4 cm, about 3.6 times less (Table
[[tab:drift-test]], Figure [[fig:drift-paths]]).

![fig:drift-paths | Estimated path over five laps (top) and estimated against taped position on arrival after lap 5 (bottom), both conditions. The drawn path is the robot's own estimate; tape measurements exist only at the start mark.](../assets/interim/nav-drift-paths.webp)

![fig:drift-per-lap | Position error at the start mark after each lap (left) and its direction, estimate minus tape (right), both conditions. All readings taken on arrival, before the final turn in place.](../assets/interim/nav-drift-per-lap.webp)

Without the relocaliser, the error grew from 2.0 cm after the first lap to about 9 cm by the third.
With it, the error stayed between 2.4 and 3.5 cm on every lap, with no trend (Figure
[[fig:drift-per-lap]]). Sideways error is the clearer sign of drift here, because the return leg
cancels much of the forward error. Without the relocaliser it stayed on one side and grew every lap,
to −7.8 cm; with it, both components stayed within ±3 cm. Because the route cancels part of the
drift, these results are a lower bound (lap-by-lap data in Appendix 7B).

#### 7.2.2. Mission Planner: Where Should I Go Next?

The mission planner is a state machine that chooses the next destination: after a plate is picked up
from the AGV, the robot heads to the table; after the plate is placed, it returns to the AGV (Figure
[[fig:mission-states]]).

![fig:mission-states | The mission planner's states and the events that move between them](../assets/interim/nav-mission-states.webp)

##### 7.2.2.1. Overwriting the Rough Map with What the Robot Sees

Once the rough map is connected (Chapter 9 Open Issues), the table and AGV positions at the start of
a run come from the VATEL planner's rough map. These are good enough to head in the right direction,
but not to stop accurately 20 cm from a table edge.

So once the robot's own sensors detect the table or the AGV, and the planner is confident in the
detection (Appendix 7C detection criteria), the measured position overwrites the one from the rough
map. From then on, the robot plans to where the table actually is, not to where the map says it was.

##### 7.2.2.2. Sending the Destination

The destination is sent to the path planner as x, y and arrival heading in the map frame. For a
table, it is an approach point 0.5 m out from the middle of the chosen side.

#### 7.2.3. Path Planner: What Is the Best Path?

The robot now knows where it is and where it is going. It needs two things now for path planning: a
map of all the obstacles around the robot, not only the tables, and an algorithm that searches that
map for a path.

##### 7.2.3.1. Building the Map

The map is built from the LiDAR point cloud, keeping only points 0.20–1.60 m above the floor, where
something would block the robot (Figure [[fig:point-filtering]]). Returns beyond 3 m are discarded
as too sparse, and the robot's own footprint is cleared so it never marks itself as an obstacle.

![fig:point-filtering | Viewer point filtering, side view](../assets/interim/nav-point-filtering.webp)

The remaining points are projected onto a 2D grid of 5 cm cells covering 60 × 60 m, published
continuously so it can be updated while the robot moves (Figure [[fig:grid-map]]).

![fig:grid-map | Example of the 2D grid map (iLounge, Innovation & Design Hub)](../assets/interim/nav-grid-map.webp)

Obstacles are then inflated with a 0.25 m hard block and a decaying cost, where d is the distance to
the nearest obstacle cell (Table [[tab:inflation]]):

Table: tab:inflation | Costmap inflation
| Distance | Cost | Effect |
| --- | --- | --- |
| d = 0 | 254 | The obstacle itself |
| d ≤ 0.25 m | 253 | Impassable: A\* refuses to enter |
| d ≤ 1.50 m | 252·exp(−1.5·(d − 0.25)) | Passable but exponentially expensive |
| Beyond | 0 | Free |

##### 7.2.3.2. Two Iterations to Improve the Map Quality

Iteration 1: a map that only adds. The first map only ever added obstacles. A person walking past
left a permanent trail of obstacle cells, which inflation widened into impassable bands that blocked
corridors that were actually clear (Figure [[fig:human-trail]]).

![fig:human-trail | Costmap with a person's trail, trail highlighted in orange (Tinkering Corner, Innovation & Design Hub)](../assets/interim/nav-human-trail.webp)

Iteration 2: The second map removes obstacles, using the fact that a LiDAR beam is evidence about
everything it passes through. Each beam travels in a straight line from the sensor to the point it
hits, so every cell along the way must be empty.

Each cell is updated by what the beams say (Table [[tab:beam-evidence]]):

Table: tab:beam-evidence | Cell evidence from LiDAR beams
| What the beams say | Effect on the cell |
| --- | --- |
| Hit at least once | Evidence +1 |
| Swept through by a beam, not hit | Evidence −1 |
| Neither hit nor swept | Unchanged |

A cell that is neither hit nor swept is occluded, so areas behind the robot or behind a wall remain
unchanged; the decay calculation is in Appendix 7D.

As a result, a person's trail disappears within about 6 to 12 s of their leaving (Figure
[[fig:trail-clearing]]), while a continuously observed wall survives up to 60 s of occlusion
(Appendix 7D).

![fig:trail-clearing | Costmap updating as a person leaves (featuring Song Cheng Yan)](../assets/interim/nav-decay-video.webp)

##### 7.2.3.3. Finding the Path: A\* with Replanning

Paths are calculated with A\* [19] in the Nav2 framework [20], giving the lowest-cost path that
never enters an impassable cell.

As people move, a clear path can become blocked, so the planner replans every 1.5 s (Figure
[[fig:replanning]]). To stop the robot swapping between similar paths on every replan, a new path is
adopted only if it is at least 15% shorter and saves at least 0.30 m, and the current path is
reconsidered every 10 s regardless.

Replanning stops entirely within 1.0 m of the goal. Near the destination the benefit of a better
route is negligible.

![fig:replanning | Path replanning as people move](../assets/interim/nav-replan-video.webp)

#### 7.2.4. Local Controller: How Do I Walk There?

The local controller turns the path into the walking command (Section 7.1).

##### 7.2.4.1. The DWB Controller

We use Nav2's DWB controller, based on the dynamic window approach [21]. Each cycle, it samples the
speeds the robot can reach before the next cycle, simulates each over the next 3 s, scores them on
path following, progress and obstacle clearance, and sends the best (Figure [[fig:dwb]]).

![fig:dwb | Visualization of how DWB works](../assets/interim/nav-dwb.webp)

##### 7.2.4.2. Fine Approach at the Table Edge

DWB stops the robot within 30 cm of its goal, but placing a plate needs it square to the table and
20 cm from its edge. Near the table, it hands over to a three-move fine approach:

- **Turn square** to the edge, using the localiser's heading.
- **Step in** slowly until the RealSense shows the edge 20 cm away.
- **Correct the heading** to remove any remaining angle.

![fig:approach-geometry | RealSense rays at 63.6° pitch: tabletop hits are used, floor hits rejected](../assets/interim/nav-approach-geometry.webp)

The RealSense, 1.262 m above the floor and pitched down at 63.6°, segments the tabletop from the
depth image and fits a line to its near edge (Figure [[fig:plane-detector]]). The distance to that
line gives the stop point, and its angle gives the heading correction. The AGV needs the same
precise, square stop, but its method has not been chosen yet; the options under consideration are in
Appendix 7E.

![fig:plane-detector | Tabletop edge from the RealSense plane detector](../assets/interim/nav-table-edge.webp)

### 7.3. Preliminary Tests and Results

We tested the table approach without the CCTV map in the Tinkering Corner and the Sandbox of the
Innovation & Design Hub, approaching tables from each side (Table [[tab:approach-tests]]). The robot
reached each table without a collision, aligned with it, and stopped consistently at the distance
suited to placing a plate.

Table: tab:approach-tests | Table approach test results
| Location | Runs | Stop distance from edge, mean ± SD (cm) | Heading error, mean ± SD (°) | Collisions |
| --- | --- | --- | --- | --- |
| **Tinkering Corner** | **2** | **3.2** | **2.1** | **0** |
| **Sandbox** | **2** | **-1.3** | **1.5** | **0** |

However, the robot does not stop centred on the table edge every time yet. The fine approach
corrects only distance and heading, and LiDAR localisation is not accurate enough to fix the
sideways position; this remains an open issue (Table [[tab:open-navigation]]).
