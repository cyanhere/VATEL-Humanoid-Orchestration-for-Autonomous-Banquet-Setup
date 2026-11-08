---
id: background
author: Team
---

## 2. Background

### 2.1. Current Banquet Setup Practice

To set plates, servers follow the event layout and their instructions, pick up a stack of plates,
walk to the target table and set one plate at each seat, then repeat until every table is done.

Venues already use equipment that speeds up parts of this work:

- folding round tables with table trolleys, and stacking chairs with chair trolleys;
- fitted or stretch table covers;
- cutlery baskets, glass racks and polishing machines;
- plate dollies and plate carts;
- modular stage platforms and truss systems.

All of these help staff move or prepare items in bulk. The last step, putting each item in its
place, is still done by hand.

### 2.2. Existing Robotic Solutions

> TODO: Add sourced examples for each group (product names or papers, with references): service and
> delivery robots in restaurants (carry trays, do not place items); AGVs and AMRs (move stacks, like
> the partner's AGV); mobile manipulators; humanoid robots in hospitality or logistics. One or two
> sentences each on what they can and cannot do for banquet setup. About 150 words.

### 2.3. Gap Addressed by This Project

We compared each step a server performs with what the R1 can do out of the box (Table
[[tab:gap]]). The robot cannot carry out any step on its own, and the fifth step, repeating until
every table is done, only works once the first four do. These gaps define the three functions VATEL
has to provide: turning the event plan into instructions the robot can use, navigating the hall, and
manipulating plates.

Table: tab:gap | What a server does, and what the R1 can do without VATEL
| Server's step | R1 out of the box |
| --- | --- |
| 1. Follow the layout and instructions | Cannot understand spoken or written instructions, or break them into ordered actions |
| 2. Pick up plates | No method to pick up a plate or a stack |
| 3. Walk to the target table | Does not know the event's layout, or where it is in the hall |
| 4. Set each plate at a seat | No method to put a plate down |
| 5. Repeat until every table is done | Only works when steps 1 to 4 all work |
