---
id: plan
---

## 10. Project Plan

### 10.1. Test Plan and Progress

Testing climbs three levels (Table [[tab:test-ladder]]). Each level must pass before the next, and
each integration stage ends with fault tests of extreme cases. The segment tests also set each
subsystem’s development targets; about 35% of them are done. Table [[tab:milestones]] shows where
each subsystem stands.

Table: tab:test-ladder | The test ladder
| Level | What is tested | Status |
| --- | --- | --- |
| Segment tests | Planner, navigation and plate manipulation, each on its own | About 35% done |
| Integration tests | Stage 1: navigation and plate manipulation, on one table with 4 plates. Stage 2: the planner added, with 4 tables at random positions and different plate layouts | Semester 2 |
| Acceptance test | Full mission: 5 tables with 8 plates each, in the banquet hall | Semester 2 |

Table: tab:milestones | Milestones reached, and the next ones
| Subsystem | Reached in Semester 1 | Next |
| --- | --- | --- |
| VATEL Planner | Tables from recorded CCTV placed on the 2-D map; map exported; runs on a tablet and a laptop | Verify in the actual hall; live CCTV updates; user testing; send the map straight to the robot |
| Navigation | Finds a table, navigates to it, aligns and stops at the target distance, in two rooms | Connect the CCTV map; find the start position in it; port to the R1 |
| Plate manipulation | Detects and locates the stack; pushes the top plate out on the robot, not yet consistently; lifts in simulation | Train the grasp by reinforcement and imitation learning; consistent real picks |

### 10.2. Semester 2 Timeline

Figure [[fig:gantt]] sets out the plan by subsystem, and Table [[tab:timeline]] lists the major
milestones.

![fig:gantt | Semester 2 plan by subsystem](../assets/interim/plan-gantt.webp)

Table: tab:timeline | Major milestones in Semester 2
| Week | Milestone |
| --- | --- |
| 1–2 | Present the workflow to PARKROYAL; evaluate the planner's interface with users |
| 5 | Subsystems fully developed and tested |
| 6 | First integration |
| 7 | First demonstration to PARKROYAL |
| 7–9 | Iterate on the subsystems |
| 9–11 | Second integration |
| 12 | Final demonstration to PARKROYAL |
| 12–13 | Final presentation and report |
