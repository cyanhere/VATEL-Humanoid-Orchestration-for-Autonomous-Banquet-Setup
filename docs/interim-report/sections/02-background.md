---
id: background
---

## 2. Background

### 2.1. Current Plate-Setting Practice

During banquet preparation, servers follow the event layout and setup instructions, carrying stacks
of plates to each table and placing one plate at every designated seat (Figure
[[fig:server-steps]]). This process is repeated until all tables are fully set, as described in
banquet setup procedures [5].

![fig:server-steps | A server's plate-setting steps](../assets/interim/background-server-steps.webp)

### 2.2. Gap Addressed by This Project

We compared the steps involved in manual plate setting (Figure [[fig:server-steps]]) with the R1's
capabilities out of the box (Table [[tab:gap]]). The R1 cannot independently perform any of these
steps, while the final step of repeating the process depends on the successful execution of the
preceding four.

Table: tab:gap | Comparison of manual plate-setting tasks and the R1's capabilities out of the box
| Server's step | R1 out of the box |
| --- | --- |
| 1. Follow the layout and instructions | Cannot understand spoken or written instructions, or break them into ordered actions |
| 2. Pick up plates | No existing method to grasp individual plates or plate stacks |
| 3. Walk to the target table | No knowledge of the event layout or its location within the hall |
| 4. Place plates at designated seats | No existing method for placing plates |
| 5. Repeat until every table is complete | Depends on the successful execution of Steps 1–4 |

These limitations highlight three key capabilities VATEL must provide: **task planning**, to
translate the event layout into actionable instructions; **autonomous navigation**, to guide the
robot through the hall; and **plate manipulation**, to retrieve and place individual plates.

### 2.3. Value Proposition and Novelty

VATEL aims to automates the repetitive task of retrieving and placing individual plates, reducing
the banquet team's workload and streamlining the overall setup process. As a humanoid designed to
operate in human-oriented environments, it requires no fixed installation and can adapt to different
event layouts.

Its novelty lies in three contributions:

- **Automatic layout capture** using the hall's existing CCTV, converting the footage into a static
  map for robot navigation;
- **A staff-friendly planning interface** that translates event layouts into navigation goals for
  every place setting;
- **A push-to-grasp strategy** that enables a humanoid hand with limited degrees of freedom to
  retrieve individual plates from a tightly nested stack.
