---
id: specifications
---

## 3. Requirements and Specifications

### 3.1. Stakeholder Requirements

PARKROYAL identified three requirements, with a fourth added by our team for the planner:

- **N1: Adaptability to a dynamic environment.** Table layouts vary between events, so the system
  must adapt to different arrangements rather than rely on a fixed layout.
- **N2: Autonomy with minimal human intervention.** The robot should complete its tasks
  independently, with staff intervention required only when necessary.
- **N3: Compatibility with the AGV.** The robot must retrieve plates from the AGV's stack at the
  banquet hall entrance.
- **N4: Ease of use.** The planner must be intuitive enough for banquet staff to operate with
  minimal training.

### 3.2. System Requirements

Table [[tab:specs]] translates these needs and the objectives in Section 1.5 into system
requirements. Technical requirements specify what VATEL must do and its expected performance, with
measurable targets for the overall system and each subsystem. Non-technical requirements define how
VATEL must fit into the hotel's operations, including its layout, staff, workflow, safety, platform
and schedule. Each requirement is linked to its source and a corresponding verification test in
Section 10.1. Targets determined by the robot platform or plate specifications are fixed; those
marked \* are proposed by our team and remain subject to confirmation with the hotel.

Table: tab:specs | System requirements: technical (T) and non-technical (N)
| ID | Requirement | Target | Source | Verified by |
| --- | --- | --- | --- | --- |
| **System** |  |  |  |  |
| T1 | Set plates without staff help | No staff intervention during a run | Need 2 | Acceptance test: 5 tables with 8 plates each, in the banquet hall |
| T2 | Plate transfer | At least 90% of plates at their positions\*; none broken | Objective 3; [state source of 90%] | Acceptance test |
| T3 | Setup time | 5 minutes per table\* | The hall must be set before guests arrive | Acceptance test |
| **Planner** |  |  |  |  |
| T4 | Capture the hall's current table layout | Every table found; centre within 10 cm\* | Need 1; enough to drive to the right table | Hall test (Section 5.4) |
| T5 | Hand the plan to the robot | Map and table goals load on the robot without manual editing | Objective 1 | Integration test, stage 2 |
| **Navigation** |  |  |  |  |
| T6 | Table detection | Every table in the layout found, though not necessarily on every attempt\* | Need 1; detected positions replace the rough map's (Section 7.2.2.1, Appendix 7C) | Detection test in the hall |
| T7 | Reach every table | Approach point of every table reached without staff help\* | Need 2; objective 2 | Navigation segment test (Section 7.3) |
| T8 | No collisions | No contact with tables, chairs or people; at least 0.25 m clearance kept\* | Works beside banquet staff; costmap hard block (Table [[tab:inflation]]) | Approach tests (Section 7.3); acceptance test |
| T9 | Moving people | A person's trail cleared within 12 s; a blocked path replaced within 1.5 s\* | Staff move through the hall during setup (Sections 7.2.3.2–7.2.3.3) | Map and replanning test |
| T10 | Stop at the table | 20 cm from the table edge, square to it | Objective 2; reach for placing plates | Navigation segment test (Section 7.3) |
| **Plate Manipulation** |  |  |  |  |
| T11 | Payload | One plate per trip | 2 kg limit per arm | Lift test on the R1 |
| T12 | Separate one plate | One plate taken; plates below moved < 5 mm | The next pick needs the stack in place (Table [[tab:pick-criteria]]) | Robot trials, stacks of 2–5 plates |
| T13 | Contact force | ≤ 60 N peak and ≤ 30 N hold per finger | Rim breaking load estimated at 120–165 N (Appendix 8B) | Static rim test; force log of each run |
| T14 | Placement accuracy | Plate centre within 5 cm of its set position\* | The hotel's setting standard | Placement trials |
| **Non-technical** |  |  |  |  |
| N1 | Fit any event layout | Nothing fixed to the hall, tables or floor | Need 1 | Integration test, stage 2 (tables at random positions) |
| N2 | Usability | Staff plan an event with almost no training; [time for a first-time user]\* | Need 4 | User test |
| N3 | Fit the hotel's workflow | Plates taken from the AGV's stack at the hall door; staff only plan the event and load the stacks | Need 3; Section 1.5 | Acceptance test |
| N4 | Safety near people | Robot stops for people in its path; arms return to a safe pose on any joint, speed or force fault | Works beside banquet staff | Fault tests at each integration stage |
| N5 | Platform | Unitree R1 with Revo 2 hands as supplied | Platform provided for the project (Section 1.2) | Inspection |
| N6 | Schedule | Subsystems complete by Week 5 of Semester 2; acceptance test by Week [TBD] | Semester 2 timeline (Section 10.2) | Milestone review |
