---
id: manipulation
---

## 8. Plate Perception and Manipulation

### 8.1. The Plate

Of the many plates and bowls used at events, we started with the most common: a porcelain dinner
plate, 255 mm across, 22 mm tall and about 500 g.

### 8.2. Finding the Plate

A segmentation model finds the plate stack in the camera image, and the RealSense depth data locate
it in the robot's frame. The segmented stack also gives the top plate's height, and so the number of
plates; this was tested with stacks of two to four.

![fig:stack-detect | Detected plate stack: top height, count and position](../assets/interim/manip-stack-detect.webp)

![fig:stack-height | Top-plate height and count for stacks of two to four plates](../assets/interim/manip-stack-height.webp)

> TODO (Cheng Yan): name the segmentation model and add its accuracy (top-plate height error, count
> correct out of how many trials).

### 8.3. Why Picking a Plate off a Stack Is Hard

Carrying a plate is within the R1's limits (hand rated for 20 kg, arm for 2 kg, plate 500 g), and a
single plate can simply be gripped at the rim. A stack has no grip point: nested plates are about
1.3 mm apart, while a fingertip, ours or the robot's, is about 14 mm thick. People get around this
by slipping a fingernail into the gap, clamping onto the top plate's rim, or pulling the plate out
with the thumb. The R1 can do none of these:

- **Arm joints:** 5 degrees of freedom (DOF) against a human's 7, while reaching a point at any
  orientation needs 6; the R1 could not reach a human pose we tried.
- **Hand joints:** 6 DOF against a human's 24.
- **Touch:** rigid fingers with less friction than skin, sensing touch only at the fingertip
  centres.

![fig:human-strategies | How people take a plate off a stack](../assets/interim/manip-human-strategies.webp)

![fig:r1-reach | R1 arm joints (left) and reach error in simulation (right)](../assets/interim/manip-r1-reach.webp)

### 8.4. Push to Grasp

So the R1 does the reverse: it **pushes** the top plate sideways so that part of the rim overhangs,
creating a grip point. A state machine runs the grasp: verify the plate's edge, push, grip with the
right hand, lift, which frees a grip point for the left hand, and grip with the left; a slipping
plate is regripped.

![fig:push-real | R1 pushing the top plate out to make a grip point](../assets/interim/manip-push-real.webp)

![fig:state-machine | Grasp sequence in MuJoCo](../assets/interim/manip-state-machine.webp)

### 8.5. Simulation, and Bringing It to the Robot

The method was first tested in MuJoCo [18] with models of the R1 and the plate: a search sweeps
through settings, inverse kinematics turns them into joint targets, the state machine decides each
move, and the physics runs with the robot's errors and sensor signals modelled, scoring each setting
across error conditions and for reachability. The same state machine then runs on the R1 through an
arm server with safety limits, reading the depth camera, joints and fingertip sensors, and deciding
at each step to move on, retry or stop.

> TODO (Cheng Yan): add the simulation-to-robot pipeline diagram (slide 82) as a figure.

### 8.6. Tests and Results

The R1 detects the plate stack and locates it in its own frame. In simulation the full lift
succeeded for stacks of different sizes. On the real robot the push creates a grip point, but not
yet consistently, and the R1 cannot yet pick up a plate. Two differences between simulation and the
robot explain most of the gap:

- **Finger positions:** over a whole motion, the default model's fingertip error was 14.2 mm RMS;
  fitting a gravity model and friction offset, driven by the robot's real motor commands, reduced it
  to 4 mm RMS.
- **Friction:** real friction differs from the simulation, so we are sweeping push configurations on
  the robot, measuring how far the plate moves, and comparing with simulation.

![fig:finger-error | Fingertip error on the R1: default model 14.2 mm vs fitted 4.0 mm RMS](../assets/interim/manip-finger-error.webp)

![fig:friction-sweep | Push sweeps on the real robot compared with simulation](../assets/interim/manip-friction-sweep.webp)

> TODO (Cheng Yan): add the number of trials and the push success rate on the real robot.

### 8.7. Next Steps

A simulation that matches the robot also makes it possible to train a learned policy, which can
react to any observation rather than to five fixed states. Next semester we will train the grasp by
**reinforcement learning** in the fitted simulation and by **imitation learning** from
demonstrations recorded with an Apple Vision Pro.
