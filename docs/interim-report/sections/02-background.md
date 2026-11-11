---
id: background
---

## 2. Background

### 2.1. Current Plate-Setting Practice

During banquet preparation, servers follow the event layout and setup instructions, carrying stacks
of plates to each table and placing one plate at every designated seat (Figure
[[fig:server-steps]]). This process is repeated until all tables are fully set.

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

Existing banquet equipment, such as trolleys, dollies and racks, helps staff transport items in bulk
(Section 1.3), but does not automate the placement process. VATEL addresses this gap by introducing
a humanoid robot that can perform repetitive plate-setting tasks consistently, reducing reliance on
temporary labour and minimising human handling of tableware. Unlike fixed automation, which is often
limited by changing venue layouts, a humanoid robot can operate in spaces designed for people,
provided it has an accurate representation of the venue and can manipulate tableware designed for
human use.

VATEL addresses these challenges through three key contributions:

- **automatic layout capture** from the hall's existing CCTV, turned into a static map the robot can
  navigate with;
- **a planner simple enough for banquet staff**, which turns the event plan into a goal for every
  place setting;
- **a push-to-grasp strategy** that lets a humanoid hand with few joints take one plate off a
  tightly nested stack.
