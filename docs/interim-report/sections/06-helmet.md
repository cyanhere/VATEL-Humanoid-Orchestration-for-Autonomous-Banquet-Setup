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

The helmet fits the robot without modifying it. A first, clamp-only version slid backwards off the
head in a fit check. A finite-element analysis of the clamping forces showed that most of the load
went through the rear pads, so later versions also screw into an existing threaded hole in the R1's
head and preload the pads. The parts are 3D-printed in PETG, with heat-set inserts for the screws.
The latest version replaces the lidar mount, after we found that the real JT128 has no mounting
flange, only screw holes in its base.

The helmet's two cables, the lidar lead and the camera's USB-C cable, run down to a cable management
box that hangs on the R1's back, behind its onboard computer (Figure [[fig:cable-box]]). As the head
turns, the cables pull a pulley up a sliding carriage inside the box, which stores up to 235 mm of
cable: enough for the head to turn ±80° while the cables stay clear of the neck and torso. The box
also holds a USB hub for the Wi-Fi dongle, the lidar's cable splitter and a spool for the lidar
lead's spare length (Figure [[fig:cable-box-exploded]]). It is built from 3 mm laser-cut acrylic
panels between 3D-printed PETG end caps, and it hooks over the robot's back handle and straps to its
frame, again without modifying the robot. In CAD, the head and helmet clear it by at least 14 mm
over the neck's full range; the box has not yet been built or fitted.

![fig:cable-box | The cable management box (CAD renders of the current design): (a) hung on the R1's back, with the lidar lead (red) and the camera cable (blue) running down from the helmet; (b) assembled, with the acrylic panels drawn clear to show the take-up pulley, hub, spool and splitter inside](../assets/interim/helmet-cable-box.webp)

![fig:cable-box-exploded | Exploded view of the cable management box (CAD render)](../assets/interim/cable-box-exploded.webp)

> TODO (Wei Sheng): add the print settings, the total mass and the fit test result on the robot. The
> cable box is designed and checked in CAD but not yet built: update this section once it is fitted,
> or note it under shortcomings if it is not by submission.
