---
id: helmet
---

## 6. Sensor Helmet

The navigation system requires a LiDAR and a depth camera with sufficient resolution to detect table
edges. Since the R1 has neither, we designed a clamp-on sensor helmet for its head (Figure
[[fig:helmet]]). It mounts a Hesai JT128 LiDAR on top for 360° scanning and an Intel RealSense
D435if depth camera at the front, angled down towards the area ahead of the robot.

![fig:helmet | Sensor helmet on the R1 (CAD)](../assets/interim/helmet-assembled.webp)

![fig:helmet-exploded | Helmet, exploded view](../assets/interim/helmet-exploded.webp)

### 6.1. Sensor Placement

Both sensors are mounted on the head, the robot's highest point and the only part that can be aimed
independently of the body (Figure [[fig:sensor-view]]). At 1.2–1.3 m above the floor, they sit above
the 0.76 m tabletops and surrounding chair backs. The head's pitch (±36°) and yaw (±115°) joints
allow both sensors to be repositioned without moving the body. Their data must therefore be
transformed using the head joint angles to align with the robot's coordinate frame.

![fig:sensor-view | Sensor fields of view (side view, to scale)](../assets/interim/helmet-sensor-view.webp)

The JT128 scans 360° horizontally, but its vertical field of view extends from 4.4° below to 90.5°
above its horizon. If mounted level, its lowest beam would only reach the floor at 17 m away.
Therefore, we tilted it forward by 24°, which is the steepest angle between 15° and 25° that keeps
the helmet outside its field of view. This directs the lowest beam 28.4° downwards, allowing it to
detect tabletops from 1.05 m away and the floor from 2.46 m away. The trade-off is that its rearward
field of view begins 19.6° above the horizon.

The RealSense camera is mounted at the front of the helmet, below the LiDAR's field of view, and
tilted 30° downwards. Its depth image spans 87° × 58°. At a height of 1.21 m, it can detect
tabletops from 0.37 m ahead of the neck and the floor from 0.82 m. The two sensors therefore
complement each other: the LiDAR detects tables from about a metre away, while the camera covers the
final metre as the robot aligns itself with the table edge (Section 7.6). The G1 navigation tests
used a camera tilted 63.6° downwards for this final alignment step. On the R1, the equivalent view
requires pitching the head 33.6° downwards from its 30° camera mount.

### 6.2. Mounting on the Head

The helmet mounts without modifying the robot. During the initial fit check, a clamp-only design
slid backwards off the head. Finite-element analysis showed that most of the clamping load was
concentrated on the rear pads. The subsequent design iteration therefore added a screw connection to
an existing threaded hole in the R1's head to secure the helmet and preload the pads. The parts are
3D-printed in PETG and TPU, with heat-set inserts for the screws.

### 6.3. Cable Management

The LiDAR and camera cables run from the helmet to a cable management box mounted on the R1's back,
behind its onboard computer (Figure [[fig:cable-box]]). As the head turns, the cables pull a pulley
along a sliding carriage inside the box, providing up to 235 mm of cable take-up. This accommodates
head rotation of ±80° while keeping the cables clear of the neck and torso.

The box also houses a USB hub for the Wi-Fi dongle, the LiDAR cable splitter and a spool for excess
LiDAR cable (Figure [[fig:cable-box-exploded]]). It consists of 3 mm laser-cut acrylic panels
between 3D-printed PETG end caps. The assembly hooks over the robot's back handle and straps to its
frame, requiring no modifications to the robot.

![fig:cable-box | Cable box (CAD): (a) on the R1's back; (b) assembled, panels transparent](../assets/interim/helmet-cable-box.webp)

![fig:cable-box-exploded | Cable box, exploded view](../assets/interim/cable-box-exploded.webp)
