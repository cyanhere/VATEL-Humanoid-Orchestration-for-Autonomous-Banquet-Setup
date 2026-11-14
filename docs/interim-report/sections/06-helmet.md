---
id: helmet
---

## 6. Sensor Helmet

The navigation system requires a LiDAR and a depth camera with sufficient resolution to detect table
edges. Since the R1 has neither, we designed a clamp-on sensor helmet for its head (Figure
[[fig:helmet]]). It mounts a Hesai JT128 LiDAR on top for 360° scanning and an Intel RealSense
D435if depth camera at the front, angled down towards the area ahead of the robot. Its design
follows from three considerations: where the sensors must see (Section 6.1), how the helmet attaches
to the head without modifying the robot (Section 6.2) and how the cables follow the head as it turns
(Section 6.3).

![fig:helmet | Sensor helmet on the R1 (CAD)](../assets/interim/helmet-assembled.webp)

![fig:helmet-exploded | Helmet, exploded view](../assets/interim/helmet-exploded.webp)

### 6.1. Sensor Placement

Both sensors sit on the head, the robot's highest point (1.2–1.3 m, above the 0.76 m tabletops) and
the only part that can be aimed independently of the body (Figure [[fig:sensor-view]]). Their data
are transformed into the robot's frame through the head's pitch (±36°) and yaw (±115°) angles.

![fig:sensor-view | Sensor fields of view (side view, to scale)](../assets/interim/helmet-sensor-view.webp)

The JT128 is tilted forward by 24°, so its lowest beam points 28.4° down and sees tabletops from
1.05 m and the floor from 2.46 m, while the whole helmet stays below its beams. The RealSense is
tilted 30° down, below the LiDAR's view, and sees tabletops from 0.37 m and the floor from 0.82 m.
The LiDAR therefore finds tables from about a metre away and the camera covers the final metre of
the approach (Section 7.2.4). Appendix 6A gives the geometry.

### 6.2. Mounting and Added Weight

The helmet mounts without modifying the robot. During the initial fit check, a clamp-only design
slid backwards off the head. Finite-element analysis showed that most of the clamping load was
concentrated on the rear pads (Appendix 6B). The subsequent design iteration therefore added a screw
connection to an existing threaded hole in the R1's head to secure the helmet and preload the pads.
The parts are 3D-printed in PETG and TPU. The helmet adds about 0.77 kg to the head; with the cable
box (Section 6.3), the robot becomes 6% heavier and its centre of gravity rises by about 23 mm
(Appendix 6C).

### 6.3. Cable Management

The LiDAR and camera cables run from the helmet to a cable management box on the R1's back, behind
its onboard computer (Figure [[fig:cable-box]]). The box does not come with the robot; we designed
it for the helmet. As the head turns, the cables pull a pulley along a sliding carriage inside the
box, providing up to 235 mm of cable take-up. This accommodates head rotation of ±80° while keeping
the cables clear of the neck and torso (Appendix 6D).

The box also houses a USB hub for the Wi-Fi dongle, the LiDAR cable splitter and a spool for excess
LiDAR cable (Figure [[fig:cable-box-exploded]]). It consists of 3 mm laser-cut acrylic panels
between 3D-printed PETG end caps. The assembly hooks over the robot's back handle and straps to its
frame, requiring no modifications to the robot.

![fig:cable-box | Cable box (CAD): (a) on the R1's back; (b) assembled, panels transparent](../assets/interim/helmet-cable-box.webp)

![fig:cable-box-exploded | Cable box, exploded view](../assets/interim/cable-box-exploded.webp)
