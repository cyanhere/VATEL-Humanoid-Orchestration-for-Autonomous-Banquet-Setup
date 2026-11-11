---
id: appendices
---

## Appendices

### Appendix A: Risk Assessment

> TODO: project risk evaluation and activity risk assessments, reference RA/CDE4301/IS-407/2026.

### Appendix B: Bill of Materials

Table: tab:bom | Main hardware
| Item | Quantity | Approximate cost (S$) |
| --- | --- | --- |
| Unitree R1 humanoid robot | 1 | 40,000 |
| BrainCo Revo 2 dexterous hand | 2 | 40,000 (pair) |
| Hesai JT128 lidar | 1 | 1,000 |
| Intel RealSense D435if depth camera | 1 |  |
| Sensor helmet (3D-printed PETG parts, inserts, screws) | 1 set |  |

> TODO: confirm the costs and fill in the camera and helmet; the
> helmet's full parts list is in the repository's hardware folder.

### Appendix C: Additional Test Data

> TODO: raw data and extra plots that support Sections 5.4, 7.7 and 8.6.

### Appendix D: Source Code

All code is in this project's [GitHub
repository](https://github.com/cyanhere/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup).

### Appendix E: Stakeholder Inventory

> TODO: add the plate list.

### Appendix 7A: The Decay Calculation

Per cell, once per 2.0 s window, with evidence e capped at 10:

- Hit at least once in the window: e ← min(e + 1, 10).
- Swept by a beam but not hit: e ← e − 1, charged every third window.
- Neither hit nor swept: e is unchanged.

A cell is published as an obstacle once e ≥ 2 and removed when e = 0.

A beam is sampled from the robot's current position to one cell short of its return, up to a range
of 15 m, and a sample contributes only where its interpolated height lies within 0.20–1.60 m.
Occlusion tolerance is cap × decay_every × window = 10 × 3 × 2 s = 60 s. During the first 20 s after
startup, penalties are suppressed.

The 15 m sampling range was chosen by measurement: extending it from 5 m to 15 m multiplied the
free-space evidence by 3.4, while 15 m to 25 m added only 1.26 times more, because beams already
terminate on walls before reaching that far. An alternative scheme that simply discarded any cell
unobserved for a fixed time was implemented and rejected: it removed 1,085 cells that no beam could
physically reach, erasing exactly the occluded regions the three-state rule exists to preserve.

### Appendix 7B: FAST-LIO2 Mechanisms

> TODO: details of backward propagation and the iterated EKF, referenced from Section 7.3.1.2.

### Appendix 8A: Rim Tracker Method

The tracker (`deploy/pick/pk/tracker.py`) was developed for this project. It starts from the stack
pose found by stack localisation and updates the top plate's pose on every depth frame:

- Depth pixels within a window around the last plate estimate are converted to 3-D points in the
  robot frame using the calibrated camera pose.
- Points belonging to the robot are removed: the arm, hand and fingertips are placed by forward
  kinematics from the measured joint angles and covered by spheres, and points inside the spheres
  are discarded.
- Points within a height band around the rim are kept. The band is asymmetric, from 4 mm below to 25
  mm above the expected rim height, so that a rising rim is not lost.
- Around the plate, in 72 sectors of 5°, the rim's outer edge is taken as a high percentile of the
  points' radius from the current centre, which rejects isolated outliers. Sectors near the robot's
  hand are excluded.
- A circle of known radius (the rim radius measured when tracking starts) is fitted to the edge
  points with a robust (Huber) loss, giving the plate centre. At least 12 of the 72 sectors must be
  valid.
- A plane is fitted to the rim heights of the valid sectors, giving the plate's tilt and the height
  of the rim at any angle.

On synthetic depth data with a 0–45 mm slide and a hand in view, the worst position error was 0.4
mm; processing took 25–35 ms per frame on a laptop. On a stationary plate on the robot, position
noise was 0.7 mm.

### Appendix 8B: Estimated Rim Breaking Load

No published breaking load for the rim of a porcelain plate was found; tableware standards address
impact rather than static load (ASTM C368). An estimate was therefore made from the flexural
strength of porcelain, typically 60–80 MPa for conventional porcelain [19], treating the rim as a
cantilever loaded at its edge by a finger:

F = σ b t² / (6 L)

where σ is the flexural strength, t = 5.5 mm the rim thickness, L = 34.5 mm the distance from the
edge to where the rim meets the plate's slope (radius 127.5 mm to 93 mm), and b the width of rim
that carries the load.

Table: tab:rim-load | Estimated rim breaking load and safety margin
| Load width b | Breaking load F (σ = 60–80 MPa) | Margin against 60 N peak | Margin against 30 N sustained |
| --- | --- | --- | --- |
| 14 mm (fingertip width; conservative) | 120–165 N | 2.0–2.7 | 4.1–5.5 |
| 69 mm (load spread over twice the rim length) | 600–810 N | 10–13 | 20–27 |

The conservative case gives a margin of about 2 at the peak limit. The estimate ignores the rim's
taper and curvature, stress concentration at chips or glaze flaws, and the support from the plate
below during the push, and some porcelains are weaker (about 40 MPa, which gives 86 N). The limits
are therefore treated as provisional, and a static rim-load test is planned: a fingertip-sized pad
pressed on the rim of a spare plate, with the load recorded until the rim chips. In use, the
measured forces are well below the limits during the push (7–9 N downward in a hand-guided
index–middle push on a scale, with the Revo 2 hand held by a person), while the grasp and lift
approach them: in the simulation evaluation, 4 of 24 conditions exceeded the limits (Appendix 8E).

### Appendix 8C: Arm Model Identification

All data were recorded on the R1 with the robot clear of the table and no plates: 30 held poses;
sweeps of each joint alone at 0.01–0.4 rad/s, back-and-forth motions of 5–50 mrad and steps; poses
with a 0.55 kg load on the wrist; and six random multi-joint motions (about 280 s), which were held
out of every fit and used only for scoring.

- **Gravity.** The torque each joint needs against gravity is linear in the mass and mass moment of
  each arm segment. These parameters were fitted to the held poses by least squares, pulled toward
  the CAD (URDF) values, with a constant and a small spring term per joint. The fitted segment
  masses are 10–25% below the URDF values; the held-out torque error fell from 0.07–0.57 N·m (URDF)
  to 0.08–0.26 N·m.
- **Friction.** In the joint sweeps, steady friction was highest at 0.01–0.04 rad/s and 15–30%
  lower above 0.15 rad/s. In the held-out random motion, friction at low speed was much lower
  (0.14–0.26 N·m against 0.6–0.8 N·m), because low speed there occurs just after a change of
  direction, before friction has built up. A friction model that depends only on speed cannot
  represent this, so the LuGre model was used [29]. It gives each joint an internal state z, which
  follows the motion and saturates: dz/dt = v − σ0 |v| z / g(v), with friction F = σ0 z and
  steady-state friction g(v) = Fc + Fs exp(−(v/vs)²). After a reversal the friction builds up over
  a few milliradians of travel. It predicted held-out friction torque better than constant or
  speed-only friction on all ten joints (e.g. left shoulder pitch 0.39 against 0.50 and 0.62 N·m).
- **Latency and gain.** The motor torque expected from the PD law (computed from the commanded and
  measured joint angles) was compared with the torque reported by the motors; the time shift that
  best aligned them was 25 ms on both arms, with a torque gain of 0.94–1.02 (0.86 on the right
  wrist).

In an open-loop MuJoCo replay of the held-out motions with the logged commands, the model tracked
the R1 to 4–12 mrad per joint (20 mrad on the right shoulder roll), within the robot's own lag
behind its plan (13–25 mrad).

### Appendix 8D: Fingertip Friction Identification

The left arm's logged commands from ten R1 pushes were replayed in MuJoCo through the identified arm
model. Each replay was registered to the robot run using the stack height from the first confirmed
fingertip touch and the stack's lateral position from the touches before the push. The fingertip
friction was varied from 0.25 to 2.0, the plate-on- plate friction over 0.45, 0.575 and 0.7, and the
stack position over three offsets. Two observations were compared with the head camera's plate
track: the drag of the plate while the fingertip slid to the rim edge (1.2–5.3 mm on the robot), and
the push distance. MuJoCo produced no drag below a fingertip friction of 0.5; the observed drag and
push distance were reproduced at 1.0 with plate-on-plate friction 0.45, at 1.0–1.3 with 0.575, and
at 1.3–1.6 with 0.7. These are effective values: errors in the arm model's force are included in
them.

### Appendix 8E: Validation Conditions in Simulation

Each candidate motion was evaluated in MuJoCo over a full factorial set of conditions: lateral
camera error of −15, 0 and +15 mm; encoder zero error of 0 or 0.4° per joint; arm joint friction of
0.6 or 1.0 times the model value; and table heights of 0.747 and 0.762 m. At the nominal stance, the
hooked push succeeded in 20 of 24 conditions, the four failures being fingertip slips at full joint
friction. The complete pick sequence lifted the plate in 24 of 24 conditions, of which 20 remained
within the force limits. These evaluations used the initial fingertip friction of 0.25 and predate
the identified arm model, so they will be repeated with the identified parameters.
