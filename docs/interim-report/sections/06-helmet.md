---
id: helmet
author: Teh Wei Sheng
---

## 6. Sensor Helmet

Navigation needs a lidar and a depth camera with enough resolution to find a table's edge. The R1
comes with neither, so we designed a clamp-on sensor helmet for its head (Figure [[fig:helmet]]). It
holds a Hesai JT128 lidar on top, with a clear view all round, and an Intel RealSense D435if depth
camera at the front, angled down at the area in front of the robot.

![fig:helmet | CAD render of the sensor helmet on the R1's head: JT128 lidar on top, RealSense depth camera at the front](../assets/hardware/sensor-helmet.webp)

![fig:helmet-exploded | Exploded view of the helmet's parts](../assets/interim/helmet-exploded.webp)

The helmet fits the robot without modifying it. A first, clamp-only version slid backwards off the
head in a fit check. A finite-element analysis of the clamping forces showed that most of the load
went through the rear pads, so later versions also screw into an existing threaded hole in the R1's
head and preload the pads. The parts are 3D-printed in PETG, with heat-set inserts for the screws.
The latest version replaces the lidar mount, after we found that the real JT128 has no mounting
flange, only screw holes in its base.

> TODO (Wei Sheng): add the print settings, the total mass and the fit test result on the robot.
> Cable routing for the lidar's right-angle plug is still open; mention it under shortcomings if it
> is not solved by submission.
