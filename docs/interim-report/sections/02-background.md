---
id: background
author: Teh Wei Sheng
---

## 2. Background

### 2.1. Current Banquet Setup Practice

During banquet preparation, servers follow the event layout and setup instructions, carrying stacks of
plates to each table and placing one plate at every designated seat. This process is repeated until
all tables are fully set.

Venues already employ various types of equipment to streamline different aspects of banquet
preparation (Figure [[fig:equipment]]), including:

- **Table and chair trolleys:** Facilitate the transportation of folding tables and stacking chairs.
- **Table covers:** Fitted or stretch covers simplify table dressing.
- **Tableware equipment:** Cutlery baskets, glass racks and polishing machines assist with handling
  and preparation.
- **Plate dollies and carts:** Allow plates to be transported in bulk.
- **Stage and truss systems:** Modular platforms and trusses simplify stage setup.

![fig:equipment | Equipment that venues use to set up in bulk: (a) table trolley; (b) chair trolley; (c) stretch table cover; (d) cutlery basket and polishing machine; (e) plate dolly; (f) modular stage platforms](../assets/interim/background-equipment.webp)

While these tools improve bulk transportation and preparation, the final placement of individual
items at their designated positions remains a manual task.

### 2.2. Existing Robotic Solutions

Several types of robots already support parts of this work (Figure [[fig:existing-robots]]).
Restaurant delivery robots, such as Pudu Robotics' BellaBot, carry food on trays of up to 10 kg each
(Pudu Robotics, n.d.), but have no arm, so staff or diners still remove each dish by hand. AGVs and
autonomous mobile robots (AMRs) move loads in bulk: the MiR250 Hook tows carts of up to 500 kg (Mobile
Industrial Robots, n.d.), and the industry partner is deploying an AGV to bring plate stacks to the
hall entrance. None of these robots can place a plate at a seat. Mobile manipulators can pick and
place objects: Toyota's Human Support Robot fetches items and tidies rooms, but grasps at most 1.2 kg
and was designed for homes (Yamamoto et al., 2019). Humanoid robots are entering logistics, where
Agility Robotics' Digit moves totes onto conveyors for GXO (Agility Robotics, 2024), but this is a
single repetitive task in a fixed warehouse, whereas each banquet has a different layout.

![fig:existing-robots | Existing robots: (a) a Pudu BellaBot delivery robot in a sports club restaurant; (b) a MiR250 AMR in a factory; (c) Toyota's Human Support Robot; (d) Agility Robotics' Digit placing a tote on a conveyor at GXO. Photos: (a) John Robert McPherson, CC0, and (b) AnnaEbdrup, CC BY-SA 4.0, both via Wikimedia Commons; (c) Yamamoto et al. (2019), CC BY 4.0; (d) Agility Robotics (2024)](../assets/interim/background-existing-robots.webp)

### 2.3. Gap Addressed by This Project

We compared the steps involved in manual plate setting (Figure [[fig:server-steps]]) with the R1's
capabilities out of the box (Table [[tab:gap]]). The R1 cannot independently perform any of these
steps, while the final step of repeating the process depends on the successful execution of the
preceding four.

![fig:server-steps | How a server sets plates: four steps, repeated until every table is done](../assets/interim/background-server-steps.webp)

These limitations highlight three key capabilities VATEL must provide: **task planning**, to
translate the event layout into actionable instructions; **autonomous navigation**, to guide the
robot through the hall; and **plate manipulation**, to retrieve and place individual plates.

Table: tab:gap | Comparison of manual plate-setting tasks and the R1's capabilities out of the box
| Server's step | R1 out of the box |
| --- | --- |
| 1. Follow the layout and instructions | Cannot understand spoken or written instructions, or break them into ordered actions |
| 2. Pick up plates | No existing method to grasp individual plates or plate stacks |
| 3. Walk to the target table | No knowledge of the event layout or its location within the hall |
| 4. Place plates at designated seats | No existing method for placing plates |
| 5. Repeat until every table is complete | Depends on the successful execution of Steps 1–4 |
