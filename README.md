# VATEL: Humanoid Orchestration for Autonomous Banquet Setup

**Project Site:** [cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup](https://cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/)

A Unitree R1 humanoid that sets banquet tables. The banquet team plans the hall in the VATEL
Planner on an iPad or a computer and sends the plan to the robot. The robot maps the real hall
with its JT128 lidar, then takes plates one at a time off a stack delivered by an AGV and places
each at its designated position. Plate transfer is the representative case for handling banquet
tableware in general. The project site has the full pipeline, success criteria, hardware and
the CDE4301 reports.

## Repository layout

```text
VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/
├── planner/        # VATEL Planner web app and its link to the robot
├── navigation/     # JT128 mapping, table-position correction, navigation to set positions
├── perception/     # plate-stack detection on the AGV, plate pose in the robot frame
├── manipulation/   # take one plate off the stack, carry it, place it
├── mission/        # on-robot executive that runs the plan and scores the result
├── hardware/       # sensor helmet CAD, bill of materials, print files
└── docs/           # the project site (GitHub Pages)
```

Each folder's README covers that subsystem: what it does, how to run it, and its current status.

## Team

Teh Wei Sheng, Song Cheng Yan and Soh Sze Juin. CDE4301 Innovation & Design Capstone, College of
Design and Engineering, National University of Singapore, AY2026/27.
