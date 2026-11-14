---
id: summary
---

## Executive Summary

Banquet halls are traditionally set up manually before guests arrive. With about ten pieces of
tableware in each guest's place setting [1], a typical banquet of 15 to 20 tables of 10 guests
requires the placement of an estimated 1,500 to 2,000 individual items. VATEL aims to streamline
this process by automating one aspect of banquet setup: plate placement using a Unitree R1 humanoid
robot.

The VATEL system consists of three main subsystems. First, the **VATEL Planner**, a browser-based
application, uses the hall's existing CCTV infrastructure to detect tables and convert the event
layout into a static 2D map containing the designated plate positions. Second, the **navigation
stack** guides the robot to each table and positions it approximately 20 cm from the table edge.
Finally, the **plate manipulation pipeline** retrieves individual plates from a stack supplied by an
AGV from [PARKROYAL COLLECTION
Pickering](https://www.panpacific.com/en/hotels-and-resorts/pr-collection-pickering.html) and places
them at the designated positions on the table.

During Semester 1, each subsystem was developed and tested independently. The VATEL Planner
successfully identified all 6 tables from real CCTV footage of the Tinkering Corner, followed by the
detection of all 23 tables in a simulated banquet hall, achieving a mean positioning error of 2.3
cm. The navigation stack was tested in two separate rooms, where the robot successfully navigated to
the target tables and stopped at the specified distance. For plate manipulation, the R1 successfully
detected plate stacks and pushed the top plate outward to create an accessible grasping point.
Although the lifting motion has been demonstrated in simulation, it has yet to be successfully
implemented on the physical robot at the time of this report’s submission.

In Semester 2, the focus will shift towards improving grasping reliability, deploying the navigation
stack directly on the R1, and integrating all three subsystems into a unified system.
