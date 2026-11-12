---
id: navigation
---

## 7. Autonomous Navigation and Localisation

### 7.1. Hardware

Following the morphological chart (Section 4.3), navigation uses a Hesai JT128 lidar and an Intel
RealSense depth camera, its only ways of sensing the room.

Table: tab:nav-sensors | Navigation sensors
| Sensor | Model | What it outputs | What navigation uses it for |
| --- | --- | --- | --- |
| 3D lidar | **Livox MID360**; Hesai JT128 on the R1 | A 360° point cloud of the surroundings, plus readings from its built-in IMU (accelerations and turn rates) | Odometry, drift correction and the obstacle map |
| Depth camera | Intel RealSense **[TODO: model]**, head-mounted (on the helmet on the R1) | A depth image: the distance to the scene at every pixel | Finding the table edge during the final approach |

The lidar is the only sensor that sees all around the robot, out to 20m: its point cloud shows where
the robot is and what blocks it, and its IMU gives odometry. The downward-pitched RealSense covers
the last step, measuring the table edge's distance and angle for the square, 20 cm stop, where
lidar’s view drops when it is closer to the table .

### 7.2. Inputs

Navigation takes three inputs: the two sensor feeds and the VATEL planner's rough 2D map of the
table and AGV positions.

Table: tab:nav-inputs | Navigation inputs
| Input | Source | Input information | Input format | Limitation |
| --- | --- | --- | --- | --- |
| Point cloud and IMU readings | Lidar (Hesai JT128) and its built-in IMU | Point cloud: the 3D positions of the surfaces around the robot, such as walls, tables and people. IMU: the robot's accelerations and turn rates | Point cloud: .ply file (x, y, z per point). IMU: numbers (3-axis acceleration in m/s², 3-axis angular velocity in rad/s) | IMU odometry drifts over time (Section 7.4) |
| Depth image | RealSense depth camera | The distance to the scene at every pixel in front of and below the robot | 16-bit depth image, one distance value per pixel (in mm) | Short range: useful only near the table |
| Rough map | VATEL planner | The room layout and the approximate positions of the tables and the AGV | .pgm image (2D occupancy grid) | Only provides rough location, need to be overwritten by accurate map. |

The output are velocity commands: a forward speed vx, a sideways speed vy and a turning rate (yaw)
to Unitree's walking SDK.

### 7.3. Subsystem Architecture

Seen from the outside, navigation is a black box: three inputs go in, and one walking command comes
out.

*Figure 7.1: fig:nav-black-box | Navigation as a black box — the lidar (point cloud + IMU),
RealSense (depth image) and VATEL planner (rough 2D map) go in; the walking command (vx, vy, yaw)
comes out*

Inside, it answers four questions in order, each answer feeding the next (Figure 7.2).

*Figure 7.2: fig:four-questions | The four questions in series — where am I? → where should I go
next? → what is the best path? → how do I walk there?, each handing its answer (pose, destination,
path) to the next*

Each question defines one function:

Table: tab:nav-functions | The four navigation functions
| Question | Function | What the function does |
| --- | --- | --- |
| Where am I? | Localisation | Estimates the robot's pose (x, y, heading) in the map frame |
| Where should I go next? | Goal selection | Decides the next destination, the table or the AGV, and its coordinates |
| What is the best path? | Path planning | Finds the lowest-cost path from the current pose to the destination that avoids every obstacle |
| How do I walk there? | Path following | Turns the path into walking commands, and lines the robot up at the table edge |

Each function is carried out by its own module. Figure 7.3 shows the four modules and the data
passed between them.

![fig:nav-architecture | Navigation subsystem architecture](../assets/interim/nav-architecture.webp)

We will be focusing on these modules from Sections 7.3.1 to 7.3.4 one at a time.

Table: tab:nav-modules | Navigation modules and the data they exchange
| Function | Module | Section | Takes in | Hands on |
| --- | --- | --- | --- | --- |
| Localisation | Localiser (odometry, accumulator, relocaliser) | 7.3.1 | Lidar point cloud and IMU readings | Current pose (x, y, heading) |
| Goal selection | Mission planner (state machine) | 7.3.2 | Rough map, detected table and AGV positions, current pose | Destination coordinates |
| Path planning | Path planner (costmap and A\*) | 7.3.3 | Lidar point cloud, current pose, destination | Path to the destination |
| Path following | Local controller (DWB, table fine approach) | 7.3.4 | Path, costmap, current pose; RealSense depth near the table | Walking command (vx, vy, yaw) |

#### 7.3.1. Localiser: Where Am I?

##### 7.3.1.1. What the Robot Needs to Know

The robot's pose is stored as x, y and heading θ in a map frame. The task requires 20cm from the
table, that gives only 20cm for the entire error budget.

Additionally, The robot starts fresh on every run, taking its starting pose as the origin (0, 0,
0°), because the room is rearranged between runs.

##### 7.3.1.2. Where the Pose Comes From

A pure inertial calculated odometry would be useless due to the noise and jitter. The IMU measures
the robot's 3D acceleration, and integrating this acceleration twice over time gives the robot's
displacement. Even a small error in the acceleration measured will be magnified by the square of
time, so the drift keeps growing.

FAST-LIO2 [15] avoids this. It did it by keeping the pose, velocity and IMU biases in one filter,
corrected with every lidar sweep. It de-skews each sweep using the IMU, then matches it to its map
in an iterated EKF that also corrects the IMU biases (Appendix 7B).

![fig:lio-backprop | Backward propagation · de-skewing one lidar sweep](../assets/interim/nav-lio-backprop.webp)

![fig:lio-iekf | The iterated EKF · one sweep's update loop](../assets/interim/nav-lio-iekf.webp)

> TODO: add the figure fig:lio-mechanism: Backward propagation and the iterated EKF.

This managed to cut drift growth into just distance travelled which is proportional to time. This
residual drift will be covered in section 7.3.1.3.

##### 7.3.1.3. Residual Drift and Periodic Correction: Relocaliser and Accumulator

Drift remains because FAST-LIO2 matches each sweep against a map it builds itself, with no external
reference, so small registration errors are written into the map and inherited by every later pose.

Measured on this robot, it accumulated at about 4% of distance walked: over 2 m on a 58 m run,
against a 20 cm budget.

We therefore added two modules that form a slow correction cycle around the odometry:

![fig:localiser | Odometry corrected by the relocaliser against the accumulated map](../assets/interim/nav-localiser.webp)

**The accumulator** adds each lidar scan to a growing point-cloud map of the room, but only when the
relocaliser trusts the current pose, so a poor pose is never written into the map (Appendix 7C).

**The relocaliser** registers the latest scans against the accumulated map by Iterative Closest
Point (ICP) once per second. The offset between the best fit and the odometry pose is the
accumulated drift (Appendix 7C).

The correction is published as a separate transform rather than written into the odometry, so the
pose stays smooth at 10 Hz while drift is corrected at 1 Hz.

##### 7.3.1.4. Improvement Result

Each condition is tested on walking five laps of 5 m out-and-back, about 58 m, from a floor mark,
and the end position was measured against it by tape. The odometry-only run disabled the
relocaliser's ICP (Appendix 7C).

> TODO: add the figure fig:start-position: Starting position of robot.

Table: tab:drift-test | Drift test results with and without the relocaliser
| Run | Distance walked (m) | Position error (cm) | Ending point |
| --- | --- | --- | --- |
| 5 × 5 m out-and-back, with ICP (relocaliser on) | 57.7 | 2.2 | *[Figure: ending position of robot, relocaliser on]* |
| 5 × 5 m out-and-back, without ICP (odometry only) | 57.6 | 18.9 | *[Figure: ending position of robot, relocaliser off]* |

> TODO: add the figure fig:drift-relocalizer: Correction applied during the run, and endpoint error.

> TODO: add the figure fig:drift-paths: Estimated path and measured endpoint, both conditions.

The relocaliser cut position error from 18.9 cm to 2.2 cm, about 8.6 times, leaving the 20 cm error
budget essentially intact for the approach.

A second measurement, independent of the floor mark, showed the relocaliser continuously cancelling
drift of about 4% of distance walked (Appendix 7C).

##### 7.3.1.5. Limitations and Further Work

**Limitation : The improvement is a lower bound:** the out-and-back route partly cancels out the
error since it is back and fourth

**Future Work : The robot's map is not aligned with the rough map** (Section 7.4.1).

#### 7.3.2. Mission Planner: Where Should I Go Next?

> TODO: The mission planner is a state machine that chooses the next
> destination: after a plate is picked up from the AGV, the robot heads
> to the table; after the plate is placed, it returns to the AGV.

> TODO: add the figure fig:mission-states: The mission planner's states
> and the events that move between them.

##### 7.3.2.1. Overwriting the Rough Map with What the Robot Sees

At the start of a run, the table and AGV positions come from the VATEL planner's rough map. These
are good enough to head in the right direction, but not to stop accurately 20 cm from a table edge.

So once the robot's own sensors detect the table or the AGV, and the planner is confident in the
detection (Appendix 7D), the measured position overwrites the one from the rough map. From then on,
the robot plans to where the table actually is, not to where the map says it was.

##### 7.3.2.2. Sending the Destination

The destination is sent to the path planner as x, y and arrival heading in the map frame. For a
table, it is an approach point 0.5 m out from the middle of the chosen side.

##### 7.3.2.3. Limitations and Future Work

**Limitations:** The rough map is not yet connected to the navigation system. We are still resolving
where the robot starts within that map (Section 7.3.1.5), so for now the table and AGV positions
come only from the robot's own detection.

**Future Work**: Connect the rough map into the navigation system and measure how far the rough map
differs from the lidar map (Section 7.4.1).

#### 7.3.3. Path Planner: What Is the Best Path?

The robot now knows where it is and where it is going. The path planner finds the best route between
the two that avoids every obstacle. It needs two things: a map of all the obstacles around the
robot, not only the tables, and an algorithm that searches that map for a path.

##### 7.3.3.1. Building the Map

The map is built from the lidar point cloud, keeping only points 0.20–1.60 m above the floor, where
something would block the robot. Returns beyond 3 m are discarded as too sparse, and the robot's own
footprint is cleared so it never marks itself as an obstacle.

![fig:point-filtering | Viewer point filtering, side view](../assets/interim/nav-point-filtering.webp)

The remaining points are projected onto a 2D grid of 5 cm cells covering 60 × 60 m, published
continuously so it can be updated while the robot moves.

> TODO: add the figure fig:grid-map: Example of the 2D grid map.

Obstacles are then inflated with a 0.25m hard block and a decaying cost, where d is the distance to
the nearest obstacle cell:

Table: tab:inflation | Costmap inflation
| Distance | Cost | Effect |
| --- | --- | --- |
| d = 0 | 254 | The obstacle itself |
| d ≤ 0.25 m | 253 | Impassable: A\* refuses to enter |
| d ≤ 1.50 m | 252·exp(−1.5·(d − 0.25)) | Passable but exponentially expensive |
| Beyond | 0 | Free |

##### 7.3.3.2. Two Iterations to Improve the Map Quality

Iteration 1: a map that only adds. The first map only ever added obstacles. A person walking past
left a permanent trail of obstacle cells. Its inflation widened into impassable bands and blocks
corridors that were actually clear.

![fig:human-trail | Costmap with a person's trail, trail highlighted in orange](../assets/interim/nav-human-trail.webp)

Iteration 2: The second map removes obstacles, using the fact that a lidar beam is evidence about
everything it passes through. Each beam travels in a straight line from the sensor to the point it
hits, so every cell along the way must be empty.

Each cell is updated by what the beams say:

Table: tab:beam-evidence | Cell evidence from lidar beams
| What the beams say | Effect on the cell |
| --- | --- |
| Hit at least once | Evidence +1 |
| Swept through by a beam, not hit | Evidence −1 |
| Neither hit nor swept | Unchanged |

A cell that is neither hit nor swept is occluded, so areas behind the robot or behind a wall remian
unchanged. Three refinements, given in Appendix 7A, stop beams from wrongly clearing real obstacles.

As the result, a cell is shown as an obstacle once its evidence reaches 2 and removed when it decays
to 0. A person's trail then disappears within about 6 to 12 s of their leaving, while a continuously
observed wall survives up to 60 s of occlusion (Appendix 7A).

> TODO: [Video: map updating as a person leaves.]

##### 7.3.3.3. Finding the Path: A\* with Replanning

> TODO: Paths are calculated with A\* [16] in the Nav2 framework [17],
> giving the lowest-cost path that never enters an impassable cell.

> TODO: As people move, a clear path can become blocked, so the planner
> replans every 1.5 s. To stop the robot swapping between similar paths
> on every replans, a new path is adopted only if it is at least 15%
> shorter and saves at least 0.30 m, and the current path is
> reconsidered every 10 s regardless.

> TODO: Replanning stops entirely within 1.0 m of the goal. Near the
> destination the benefit of a better route is negligible.

> TODO: [Video: path replanning.]

#### 7.3.4. Local Controller: How Do I Walk There?

The local controller turns the path into walking commands (vx, vy, yaw rate), which the robot reads
at 5 Hz.

##### 7.3.4.1. The DWB Controller

We use Nav2's DWB controller, based on the dynamic window approach [18]. Each cycle, it samples the
speeds the robot can reach before the next cycle, simulates each over the next 3s, scores them on
path following, progress and obstacle clearance, and sends the best.

![fig:dwb | Visualization of how DWB works](../assets/interim/nav-dwb.webp)

##### 7.3.4.2. Fine Approach at the Table Edge

DWB stops the robot within 30 cm of its goal, but placing a plate needs it square to the table and
20 cm from its edge. Near the table, it hands over to a three-move fine approach:

- **Turn square** to the edge, using the localiser's heading.
- **Step in** slowly until the RealSense shows the edge 20 cm away.
- **Correct the heading** to remove any remaining angle.

The RealSense, 1.262 m above the floor and pitched down at 63.6°, segments the tabletop from the
depth image and fits a line to its near edge. The distance to that line gives the stop point, and
its angle the will be used for heading correction.

![fig:plane-detector | Tabletop edge from the RealSense plane detector](../assets/interim/nav-table-edge.webp)

### 7.4. Full Navigation Workflow for a Rectangular Table

#### 7.4.1. Open Issues and Next Steps

The workflow in Section 7.4.2 is the target design. These open issues still gate it. Until they are
resolved, the robot cannot start from the rough map, so the table approach has so far been tested
without it (Section 7.5).

Table: tab:open-issues | Open issues and next steps
| Open issue | What it blocks | Next step | Section |
| --- | --- | --- | --- |
| The robot does not know where it starts in the rough map. Its own map begins wherever it stands, so the two maps are not aligned. | Steps 1 to 3: the table's position from the rough map cannot be placed in the robot's map. | Localise the robot within the rough map at the start of each run. | 7.3.1.5 |
| The rough map is not yet connected to the navigation system. | Step 2: the mission planner cannot read the table's rectangle. | Connect the VATEL planner's rough map to the mission planner. | 7.3.2.3 |
| All work so far ran on a Unitree G1 borrowed from the BEADZ Lab. | Every step, on the final robot. | Move the stack to the R1, with the JT128 lidar and the helmet-mounted RealSense, and retest it there next semester. | 7.1 |

#### 7.4.2. Target Workflow

A trip to a rectangular table runs as follows:

1. **Start.** The localiser begins tracking the robot's pose.
2. **Choose the table.** The mission planner reads the table's rectangle from the rough map.
3. **Choose the approach point**, 0.5 m out from the middle of one side, facing the table.
4. **Plan the path** with A\*, replanning every 1.5 s.
5. **Walk the path** with DWB, steering around people and obstacles.
6. **Update the table position** once the robot detects it, and replan.
7. **Fine approach** to 30 cm from the edge, square to it.
8. **Hand over** to plate manipulation.
9. **Next destination:** return to the AGV and repeat steps 3 to 7.

![fig:nav-workflow | Target workflow for a trip to a rectangular table](../assets/interim/nav-workflow.webp)

### 7.5. Partial Tests and Results

We tested the table approach without the CCTV map in the Tinkering Corner and the Sandbox of the
Innovation & Design Hub, approaching tables from each side. The robot reached each table without a
collision, aligned with it, and stopped consistently at the distance suited to placing a plate.

Table: tab:approach-tests | Table approach test results
| Location | Runs | Stop distance from edge, mean ± SD (cm) | Heading error, mean ± SD (°) | Collisions |
| --- | --- | --- | --- | --- |
| **Tinkering Corner** | **2** |  |  |  |
| **Sandbox** | **2** |  |  |  |

This is the evidence for specification S5. [TODO: one sentence comparing the results with the S5
target.]
