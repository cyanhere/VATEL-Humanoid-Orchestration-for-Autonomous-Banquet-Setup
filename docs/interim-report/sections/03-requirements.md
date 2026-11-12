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

Table [[tab:specs]] turns the partner's requirements (Section 3.1) and the objectives (Section 1.5)
into measurable targets, each with the test that verifies it (Section 9.1). Targets set by the
platform or the plate are fixed; those marked \* are our own and are to be confirmed with the
partner.

Table: tab:specs | Design specifications
| ID | Specification | Target | Basis | Verified by |
| --- | --- | --- | --- | --- |
| **Planner** |  |  |  |  |
| S1 | Find the tables from the hall's CCTV and place them on the 2-D map | Every table found; centre within 10 cm\* | Enough to drive to the right table; close-range sensing does the rest | CCTV test in the hall (Section 5.4) |
| S2 | Hand the plan to the robot | Static map and named goals load in the navigation stack unedited | Objective 1 | Integration test 2 (Section 9.2) |
| S3 | Planner platform | Runs in a browser on a tablet and a laptop | Banquet staff | User test |
| **Navigation** |  |  |  |  |
| S4 | Navigation coverage | Robot reaches at least 80% of the required workspace\* | [state source of 80%] | Navigation segment test |
| S5 | Stop at the table | 30 cm\* from the table edge, square to it, in [n of n] approaches | Reach for placing plates; objective 2 | Navigation segment test (Section 7.5) |
| **Plate manipulation** |  |  |  |  |
| S6 | Payload | One plate per trip | 522 g plate against 2 kg per arm | Lift test on the R1 |
| S7 | Separation | One plate taken; plates below moved < 5 mm | The remaining stack must stay in place for the next pick (Table [[tab:pick-criteria]]) | Robot trials, stacks of 2–5 plates |
| S8 | Contact force | ≤ 60 N peak and ≤ 30 N hold per finger | Rim breaking load estimated at 120–165 N (Appendix 8B) | Static rim test; force log of each run |
| S9 | Placement accuracy | Plate centre within [TBD] mm of its set position\* | Partner's setting standard | Placement trials |
| S10 | Plate transfer | At least 90% of plates at their positions\*; none broken | Objective 3; [state source of 90%] | Acceptance test |
| **System** |  |  |  |  |
| S11 | Autonomy | No staff intervention during a run | Stakeholder requirement | Acceptance test |
| S12 | Acceptance mission | 5 tables with 8 plates each, in the banquet hall | Test plan (Section 9.1) | Acceptance test |

> TODO: Confirm the 10 cm target for S1 and the 30 cm target for S5
> with the subsystem leads, and add any target from the partner (for
> example, time allowed to set one table).
