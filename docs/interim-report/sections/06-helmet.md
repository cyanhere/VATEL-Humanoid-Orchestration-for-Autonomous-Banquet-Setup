---
id: helmet
author: Teh Wei Sheng
---

## 6. Sensor Helmet

Navigation needs a lidar and a depth camera with enough resolution to find a table's edge. The R1
comes with neither, so we designed a clamp-on sensor helmet for its head (Figure [[fig:helmet]]). It
holds a Hesai JT128 lidar on top, with a clear view all round, and an Intel RealSense D435if depth
camera at the front, angled down at the area in front of the robot.

![fig:helmet | CAD render of the sensor helmet on the R1's head: JT128 lidar on top, RealSense depth camera at the front](../assets/interim/helmet-assembled.webp)

![fig:helmet-exploded | Exploded view of the helmet's parts](../assets/interim/helmet-exploded.webp)

### 6.1. Sensor Placement

Both sensors sit on the head, the robot's highest point and the only part that can be aimed (Figure
[[fig:sensor-view]]). There they are 1.2 to 1.3 m above the floor, above a 0.76 m tabletop and the
chair backs around it, and the head's two joints, pitch (±36°) and yaw (±115°), turn both sensors
without moving the body. The neck's joint angles are therefore needed to place their data in the
robot's frame.

![fig:sensor-view | What each sensor sees (side view to scale, head at its zero pose): the lidar, tilted 24°, sees everything above its lowest beam, 28.4° down; the camera, tilted 30°, sees the 58° wedge below it, covering the last metre before a table that the lidar cannot](../assets/interim/helmet-sensor-view.webp)

The JT128 scans all round but looks mostly upwards, from 4.4° below its horizon to 90.5° above;
mounted level, its lowest beam would reach the floor only 17 m away. We tilted it 24° forward, the
steepest angle between 15° and 25° at which no part of the helmet enters its view (the nearest, the
brow bumper, stays 16.7° below its lowest beam). Its lowest beam then points 28.4° down, so it sees
a tabletop from 1.05 m ahead and the floor from 2.46 m, while still scanning the hall's walls above
the furniture for localisation. In exchange, its view behind the robot starts 19.6° above the
horizon.

The RealSense sits at the front of the helmet, below the lidar's view, tilted 30° down. Its depth
image spans 87° by 58°, so from 1.21 m up it sees a tabletop from 0.37 m ahead of the neck and the
floor from 0.82 m. The two sensors therefore complement each other: the lidar sees tables from about
a metre away, and the camera covers the last metre, where the robot lines up with the table's edge
(Section 7.6). In the navigation tests on the G1 that step used a camera looking 63.6° down; on the
R1, pitching the head 33.6° down gives the same view from the 30° mount, and yaw (up to ±80° with
the cable box) aims it to either side. Mounted on the robot, the camera measured 34° down at 1.18 m,
so the measured pose should be used, not the CAD value.

### 6.2. Mounting on the Head

The helmet fits the robot without modifying it. A first, clamp-only version slid backwards off the
head in a fit check. A finite-element analysis of the clamping forces showed that most of the load
went through the rear pads, so later versions also screw into an existing threaded hole in the R1's
head and preload the pads. The parts are 3D-printed in PETG, with heat-set inserts for the screws.

### 6.3. Cable Management

The helmet's two cables, the lidar lead and the camera's USB-C cable, run down to a cable management
box that hangs on the R1's back, behind its onboard computer (Figure [[fig:cable-box]]). As the head
turns, the cables pull a pulley up a sliding carriage inside the box, which stores up to 235 mm of
cable: enough for the head to turn ±80° while the cables stay clear of the neck and torso. The box
also holds a USB hub for the Wi-Fi dongle, the lidar's cable splitter and a spool for the lidar
lead's spare length (Figure [[fig:cable-box-exploded]]). It is built from 3 mm laser-cut acrylic
panels between 3D-printed PETG end caps, and it hooks over the robot's back handle and straps to its
frame, again without modifying the robot. In CAD, the head and helmet clear it by at least 14 mm
over the neck's full range; the box has not yet been built or fitted.

![fig:cable-box | The cable management box (CAD renders of the current design): (a) hung on the R1's back, with the lidar lead (red) and the camera cable (blue) running from the sensors, through a clip on the helmet, into the box; (b) assembled, with the acrylic panels drawn clear to show the take-up pulley, hub, spool and splitter inside](../assets/interim/helmet-cable-box.webp)

![fig:cable-box-exploded | Exploded view of the cable management box (CAD render)](../assets/interim/cable-box-exploded.webp)

> TODO (Wei Sheng): add the print settings, the total mass and the fit test result on the robot. The
> cable box is designed and checked in CAD but not yet built: update this section once it is fitted,
> or note it under shortcomings if it is not by submission.
