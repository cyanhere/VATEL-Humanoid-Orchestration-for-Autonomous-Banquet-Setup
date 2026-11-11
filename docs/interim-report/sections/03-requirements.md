---
id: specifications
---

## 3. Requirements and Specifications

### 3.1. Stakeholder Requirements

The partner set three requirements:

- **A dynamic environment.** The table setup differs for every event, so nothing can be fixed to one
  layout.
- **Autonomy with minimal human interference.** Staff should rarely need to step in.
- **Work with the AGV.** The robot picks up plates from the AGV's stack at the hall door.

The planner adds a fourth requirement of our own: banquet staff must be able to use it with almost
no training.

### 3.2. Design Specifications

Table [[tab:specs]] turns the objectives and requirements into measurable targets.

Table: tab:specs | Design specifications
| ID | Specification | Target | Basis |
| --- | --- | --- | --- |
| S1 | Find the tables from the hall's CCTV and place them on the 2-D map | Every table found; centre within 10 cm | Enough to drive to the right table; close-range sensing does the rest |
| S2 | Hand the plan to the robot | Static map and named goals load in the navigation stack unedited | Objective 1 |
| S3 | Planner platform | Runs in a browser on a tablet and a laptop | Banquet staff |
| S4 | Navigation coverage | Robot reaches at least 80% of the required workspace | Project success criterion |
| S5 | Stop at the table | 30 cm from the table edge, square to it, every time | Reach for placing plates; objective 2 |
| S6 | Payload | One plate per trip | 2 kg payload per arm; a dinner plate weighs 500 g |
| S7 | Plate transfer | At least 90% of plates at their positions; none broken | Project success criterion; objective 3 |
| S8 | Autonomy | No staff intervention during a run | Stakeholder requirement |
| S9 | Acceptance mission | 5 tables with 8 plates each, in the banquet hall | Test plan (Section 9.1) |

> TODO: Confirm the 10 cm target for S1 and the 30 cm target for S5 with the subsystem leads, and
> add any target from the partner (for example, time allowed to set one table).
