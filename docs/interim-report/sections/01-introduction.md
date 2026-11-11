---
id: introduction
author: Teh Wei Sheng
---

## 1. Introduction

### 1.1. Problem of Interest

Banquet halls typically begin as empty spaces that must be transformed into fully prepared dining
venues within a few hours, with all table settings arranged manually before guests arrive (Figure
[[fig:banquet-setup]]). A typical banquet comprising 15 to 20 tables, with 10 place settings per
table and approximately 10 items per setting, requires staff to manually position an estimated 1,500
to 2,000 individual items. This makes banquet preparation a labour-intensive and time-consuming
process, particularly when tight setup schedules must be met.

![fig:banquet-setup | Setting up a banquet hall: (a) the empty hall; (b) tables in place; (c) staff laying out the settings by hand; (d) a finished table setting](../assets/interim/intro-banquet-setup.webp)

This challenge is especially relevant in Singapore, where business events, including meetings,
incentives, conventions and exhibitions (MICE), form a significant and growing part of the tourism
industry. In 2025, business events generated S$2.3 billion in tourism receipts, representing a 35%
increase from 2024 (Singapore Tourism Board, 2026). Weddings further contribute to the demand for
banquet services. Singapore recorded 24,688 marriages in 2025 (Department of Statistics Singapore,
2026), while a 2017 survey by Bridestory found that 59.6% of 1,500 surveyed couples preferred hotel
ballrooms as their wedding venues. Applying this proportion to the number of marriages registered in
2025 suggests that hotel wedding banquets could represent a substantial share of the demand for
banquet preparation services.

Given the frequency of such events and the considerable manual effort involved, our initial problem
statement was broad: *How can the banquet setup process be made less labour-intensive,
time-consuming and more efficient?*

### 1.2. Context and Stakeholders

The project was provided with two key conditions. The EDIC department supplied a **Unitree R1
humanoid robot equipped with two BrainCo Revo 2 hands** as the development platform (Figure
[[fig:platform]]), while an **industry partner** (name withheld under a non-disclosure agreement)
provides the operational context.

![fig:platform | The development platform: (a) the Unitree R1 humanoid robot; (b) the BrainCo Revo 2 hand, one on each arm](../assets/interim/intro-platform.webp)

Thus, the main stakeholders are:

- **Partner's Banquet Operations Team:** Primary users responsible for banquet hall preparation and
  planning.
- **Industry Partner's Management:** Oversees operational requirements and potential system
  adoption.
- **EDIC Department:** Provides the robotic platform and technical support.

### 1.3. Existing Banquet Equipment

Venues already employ various types of equipment to streamline different aspects of banquet
preparation (Figure [[fig:equipment]]), including:

- **Table and chair trolleys:** Facilitate the transportation of folding tables and stacking chairs.
- **Table covers:** Fitted or stretch covers simplify table dressing.
- **Tableware equipment:** Cutlery baskets, glass racks and polishing machines assist with handling
  and preparation.
- **Plate dollies and carts:** Allow plates to be transported in bulk.
- **Stage and truss systems:** Modular platforms and trusses simplify stage setup.

![fig:equipment | Equipment that venues use to set up in bulk: (a) table trolley; (b) chair trolley; (c) stretch table cover; (d) cutlery basket and polishing machine; (e) plate dolly; (f) modular stage platforms](../assets/interim/background-equipment.webp)

Although these tools improve transportation and preparation efficiency, placing individual items at
their designated positions remains a manual task.

Given this gap, VATEL focuses on automating plate placement as an initial step. Plates were selected
based on the R1's manipulation limitations: its hands are difficult to program for fine-grasp tasks,
while its 2 kg payload per arm limits its ability to handle heavier objects such as tables and
chairs. Furthermore, the industry partner identified plates as the simplest starting point for
automation.

### 1.4. Existing Robotic Solutions

Several types of robots already support parts of this work (Figure [[fig:existing-robots]]).
Restaurant delivery robots, such as Pudu Robotics' BellaBot, carry food on trays of up to 10 kg each
(Pudu Robotics, n.d.), but have no arm, so staff or diners still remove each dish by hand. Automated
guided vehicles (AGVs) and autonomous mobile robots (AMRs) move loads in bulk: the MiR250 Hook tows
carts of up to 500 kg (Mobile Industrial Robots, n.d.), and the industry partner is deploying an AGV
to bring plate stacks to the hall entrance. None of these robots can place a plate at a seat. Mobile
manipulators can pick and place objects: Toyota's Human Support Robot fetches items and tidies
rooms, but grasps at most 1.2 kg and was designed for homes (Yamamoto et al., 2019). Humanoid robots
are entering logistics, where Agility Robotics' Digit moves totes onto conveyors for GXO (Agility
Robotics, 2024), but this is a single repetitive task in a fixed warehouse, whereas each banquet has
a different layout.

![fig:existing-robots | Existing robots: (a) a Pudu BellaBot delivery robot in a sports club restaurant; (b) a MiR250 AMR in a factory; (c) Toyota's Human Support Robot; (d) Agility Robotics' Digit placing a tote on a conveyor at GXO. Photos: (a) John Robert McPherson, CC0, and (b) AnnaEbdrup, CC BY-SA 4.0, both via Wikimedia Commons; (c) Yamamoto et al. (2019), CC BY 4.0; (d) Agility Robotics (2024).](../assets/interim/background-existing-robots.webp)

With this narrower focus, the project seeks to address the following problem: *How could we use the
Unitree R1 humanoid robot to optimise banquet setup by automating plate setup?*

### 1.5. Project Scope and Objectives

The plate-setting workflow consists of five steps (Figure [[fig:workflow]] and Table
[[tab:workflow]]). The first three get the layout decided and transfer the plates to the hall: the
banquet team sets the table layout for the event, staff load plate stacks onto a dish dolly, and the
industry partner is deploying an AGV to transport them from the kitchen to the banquet hall
entrance. The project is therefore scoped to the work **inside the hall** (Table [[tab:scope]]):
retrieving plates from the AGV at the entrance and placing each one at its designated seat. Because
the layout changes with every event, the robot must also know where each table and seat is, but
moving tables and chairs stays out of scope.

![fig:workflow | The plate-setting workflow: (a) the five steps, of which VATEL takes on the two inside the banquet hall; (b) inside the hall, the robot carries each plate from the AGV at the door to its seat, on a layout that changes from one event to the next](../assets/interim/intro-workflow.webp)

Table: tab:workflow | The current plate-setting workflow, and the part VATEL addresses
| Step | Process | Done by |
| --- | --- | --- |
| 1 | Set the table layout for the event | Banquet team |
| 2 | Load plate stacks onto dish dolly | Staff |
| 3 | Transport plates to banquet hall | AGV |
| 4 | Retrieve and transport plates to tables | Humanoid robot |
| 5 | Place plates at designated seats | Humanoid robot |

By the end of CDE4301, VATEL aims to achieve three objectives:

1. Transfer the event's banquet hall layout to the robot.
2. Navigate autonomously to each designated table position and stop accurately and consistently.
3. Retrieve plates from the AGV and place them at their designated table positions.

Table: tab:scope | Project scope
| In scope | Out of scope |
| --- | --- |
| In-hall operations, from the AGV at the entrance to individual seats | Kitchen-to-hall transportation (handled by the partner's AGV) |
| Porcelain dinner plates, one per trip | Cutlery, glassware, bowls and linen |
| Knowing the event's table layout and each plate's position | Moving tables and chairs |
