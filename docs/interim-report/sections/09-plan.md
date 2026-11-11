---
id: plan
---

## 9. Project Plan

### 9.1. Test Plan and Progress

Testing climbs a ladder of three levels (Table [[tab:test-ladder]]): each subsystem on its own, then
step-by-step integration, then the full mission in the banquet hall. The segment tests set the
targets for each subsystem's development, and we are about 35% of the way through them. Table
[[tab:milestones]] lists what each subsystem has reached and what comes next.

Table: tab:test-ladder | The test ladder
| Level | What is tested | Status |
| --- | --- | --- |
| Segment tests | Planner, navigation and plate manipulation, each on its own | About 35% done |
| Integration tests | Map to navigation, then AGV stop to plate manipulation, up to one full table | Semester 2 |
| Acceptance test | Full mission: 5 tables with 8 plates each, in the banquet hall | Semester 2 |

Table: tab:milestones | Milestones reached, and the next ones
| Subsystem | Reached in Semester 1 | Next |
| --- | --- | --- |
| VATEL Planner | Tables from recorded CCTV placed on the 2-D map; map exported; runs on a tablet and a laptop | Verify in the actual hall; live CCTV updates; user testing; send the map straight to the robot |
| Navigation | Finds a table, navigates to it, aligns and stops at the target distance, in two rooms | Connect the CCTV map; find the start position in it; port to the R1 |
| Plate manipulation | Detects and locates the stack; pushes the top plate out on the robot, not yet consistently; lifts in simulation | Train the grasp by reinforcement and imitation learning; consistent real picks |

### 9.2. Integration Plan

Once the segment tests pass, the subsystems are integrated in two stages, each ending with a fault
test of extreme cases:

- **Navigation and plate manipulation.** The robot, parked at a table's edge, places plates: one
  table in the Tinkering Corner, 4 plates (one per edge), table position set by hand.
- **Adding the planner.** The planner's map, tables and plate layout drive the whole mission: 4
  tables in the Tinkering Corner at random positions, each with a different plate layout.

A second round, after users' feedback, repeats the same order.

### 9.3. Semester 2 Timeline

Figure [[fig:gantt]] sets out the plan by subsystem, and Table [[tab:timeline]] lists the major
milestones.

![fig:gantt | Semester 2 plan by subsystem](../assets/interim/plan-gantt.webp)

> TODO: the Gantt chart and the milestone table disagree on integration
> (chart: full-system integration in Weeks 7–11; table: first
> integration in Week 6, second in Weeks 9–11). Align them.

Table: tab:timeline | Major milestones in Semester 2
| Week | Milestone |
| --- | --- |
| 1–2 | Present the workflow to the partner; evaluate the planner's interface with users |
| 5 | Subsystems fully developed and tested |
| 6 | First integration |
| 7 | First demonstration to the partner |
| 7–9 | Iterate on the subsystems |
| 9–11 | Second integration |
| 12 | Final demonstration to the partner |
| 12–13 | Final presentation and report |

### 9.4. Risks

Table: tab:risks | Main risks and how we plan to handle them
| Risk | Plan |
| --- | --- |
| The R1 cannot yet pick up a plate reliably | Fit the simulation to the robot; pursue reinforcement and imitation learning in parallel |
| The planner is untested in the actual hall | Test there in Weeks 1–2, alongside the workflow presentation |
| Navigation was developed on a G1, not the R1 | Port it first; the helmet already carries the R1's sensors |
| The robot's start position in the CCTV map is unknown | Solve it before the second integration, which depends on that map |
