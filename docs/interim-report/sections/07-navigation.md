---
id: navigation
---

## 7. Autonomous Navigation and Localisation

### 7.1. Overview

The navigation stack takes in the planner's static map, the lidar point cloud and the RealSense
depth feed, and puts out walking commands: forward and sideways speeds and a turning rate
(*v<sub>x</sub>*, *v<sub>y</sub>*, yaw). It answers four questions in turn: *where am I?*, *where
next?*, *what is the best path?* and *how do I walk there?*

![fig:nav-architecture | Navigation stack modules](../assets/interim/nav-architecture.webp)

### 7.2. Where Am I?

The lidar's built-in inertial measurement unit (IMU) gives the robot's odometry, the path it has
travelled. Odometry drifts, so a correction loop was added: the **accumulator** collects every scan
into a map, and the **relocaliser** periodically matches the current scan against it to correct the
drift.

![fig:localiser | Localiser: lidar–IMU odometry corrected by relocalisation](../assets/interim/nav-localiser.webp)

> TODO (Sze Juin): confirm which lidar and camera were used in the G1 tests (the deck says the stack
> was developed on a G1 and will move to the JT128 on the R1), and add how much drift remains with
> the correction loop.

### 7.3. Where Should I Go Next?

A mission planner picks the next destination and sends its coordinates to the path planner.

### 7.4. What Is the Best Path?

The path planner runs the A\* search [15] on a Nav2 costmap [16] to find the lowest-cost path to the
destination, and replans periodically so that a blocked path is replaced.

Planning depends heavily on the map. Our first map only added obstacles, so a person walking through
left a trail that never disappeared and blocked the planner. The second version uses a **dynamic
costmap** in which obstacle points decay once the obstacle has gone.

![fig:costmap | Permanent obstacle trails (left) and the decaying dynamic costmap (right)](../assets/interim/nav-costmap.webp)

> TODO (Sze Juin): add how long an obstacle point takes to decay.

### 7.5. How Do I Walk There?

The DWB local controller, based on the dynamic window approach [17], chooses the velocity that best
follows the path while avoiding nearby obstacles, for example a forward speed of 0.25 m/s while
turning at 0.17 rad/s. That command passes through a Nav2 bridge and a ROS 2 controller to Unitree's
walking SDK, which makes the robot walk.

### 7.6. Lining Up at the Table

Near a table the robot lines up in three moves: it turns on the spot until square to the edge, using
the localiser's heading; steps forward slowly with the RealSense on until 30 cm from the edge; and
makes a final heading correction. The camera, 1.262 m above the floor and pitched down at 63.6°,
sees a 0.76 m tabletop 0.502 m below it; the stack segments the tabletop from the depth image, fits
a line to its edge, and corrects the heading by that line's angle.

![fig:table-edge | Tabletop edge detection from depth](../assets/interim/nav-table-edge.webp)

### 7.7. Tests and Results

We tested the table approach in the Tinkering Corner and the Sandbox of the NUS Innovation & Design
Hub, approaching different tables from each side. The robot found each table, navigated to it
without hitting an obstacle, aligned with it, and stopped consistently at the distance best suited
to placing a plate.

> TODO (Sze Juin): add the number of runs and the measured results (stop distance from the edge and
> heading error: mean and spread) as a table. This is the evidence for specification S5.

### 7.8. Problems and Next Steps

All navigation work so far ran on a Unitree G1 borrowed from the BEADZ Lab. Next semester the stack
moves to the R1, with the JT128 lidar and head-mounted RealSense on the helmet, and must be retested
there. The planner's CCTV map also has to feed the pipeline, which raises an open problem: the robot
starts each run without knowing where it is in that map.
