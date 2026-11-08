---
id: summary
author: Team
---

## Executive Summary

Every banquet hall is set up by hand before guests arrive: 1,500 to 2,000 items for a banquet of 15
to 20 tables. VATEL lets a Unitree R1 humanoid robot take over one part of that work, placing plates.

VATEL has three parts. The **VATEL Planner** is a browser app that reads the hall's existing CCTV,
finds every table and turns the event plan into a 2-D static map with every set position. The
**navigation stack** takes the robot to each table and lines it up 30 cm from the edge. The **plate
manipulation** pipeline takes one plate at a time off a stack delivered by our industry partner's
AGV.

In Semester 1 we built and tested each part on its own. The planner found all 23 tables in a
simulated hall (2.3 cm mean error) and all 6 tables in real CCTV footage of a test room. The robot
navigated to tables in two test rooms and stopped at the target distance. The R1 detected plate
stacks and pushed the top plate out to make a grip point; the lift works in simulation but not yet on
the robot. Semester 2 focuses on reliable grasping, porting navigation to the R1, and integration.
