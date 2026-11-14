---
id: introduction
---

## 1. Introduction

### 1.1. Problem of Interest

Banquet halls typically begin as empty spaces that must be transformed into fully prepared dining
venues within a few hours, with all table settings arranged manually before guests arrive (Figure
[[fig:banquet-setup]]). Each guest's place setting is made up of plates, cutlery, glassware and a
napkin [1], which we estimate at about ten pieces. At 10 guests per table, a typical banquet of 15
to 20 tables therefore requires staff to position an estimated 1,500 to 2,000 individual items by
hand. This makes banquet preparation a labour-intensive process, particularly when tight setup
schedules must be met.

![fig:banquet-setup | Banquet hall setup: (a) empty hall; (b) tables placed; (c) staff laying settings; (d) finished setting. Images from Magnific](../assets/interim/intro-banquet-setup.webp)

This challenge is especially relevant in Singapore, where business events, including meetings,
incentives, conventions and exhibitions (MICE), form a significant and growing part of the tourism
industry. In 2025, business events generated S$2.3 billion in tourism receipts, representing a 35%
increase from 2024 [2]. Weddings further contribute to the demand for banquet services. Singapore
recorded 24,688 marriages in 2025 [3], while a 2017 survey [4] found that 59.6% of 1,500 surveyed
couples preferred hotel ballrooms as their wedding venues. Applying this proportion to the number of
marriages registered in 2025 suggests that hotel wedding banquets could represent a substantial
share of the demand for banquet preparation services.

Given the frequency of such events and the considerable manual effort involved, our initial problem
statement was broad: *How can the banquet setup process be made less labour-intensive,
time-consuming and more efficient?*

### 1.2. Context and Stakeholders

The project was provided with two key conditions. The EDIC department supplied a **Unitree R1
humanoid robot equipped with two BrainCo Revo 2 hands** as the development platform (Figure
[[fig:platform]]), while **PARKROYAL COLLECTION Pickering**, our industry partner, provides the
operational context.

![fig:platform | Development platform: (a) Unitree R1; (b) BrainCo Revo 2 hand](../assets/interim/intro-platform.webp)

Thus, the main stakeholders are:

1. **PARKROYAL Banquet Operations Team:** Primary users responsible for banquet hall preparation and
   planning.
2. **PARKROYAL Management:** Oversees operational requirements and potential system adoption.
   PARKROYAL is also deploying an AGV that carries plate stacks from the kitchen to the banquet hall
   entrance, where VATEL's work begins.
3. **EDIC Department:** Provides the robotic platform and technical support.

### 1.3. Existing Banquet Equipment

To see what is already mechanised, our team did a quick study of banquet operation equipment and
found that it mainly helps staff move and prepare items in bulk (Figure [[fig:equipment]]):

- **Table and chair trolleys:** Facilitate the transportation of folding tables and stacking chairs.
- **Table covers:** Fitted or stretch covers simplify table dressing.
- **Tableware equipment:** Cutlery baskets, glass racks and polishing machines.
- **Plate dollies and carts:** Transport plates in bulk.
- **Stage and truss systems:** Modular and simplify stage setup.

![fig:equipment | Bulk setup equipment: (a) table trolley; (b) chair trolley; (c) stretch cover; (d) cutlery basket and polisher; (e) plate dolly; (f) stage platforms](../assets/interim/background-equipment.webp)

Although these tools improve transportation and preparation efficiency, placing individual items at
their designated positions remains a manual task. A typical banquet standard operating procedure
assigns the table setup to staff in its pre-function checklist [5], and we observed the same during
our site visit to PARKROYAL.

Given this gap, VATEL focuses on **automating plate placement** as an initial step. Plates were
selected partly because they suit the R1's manipulation capabilities. Its hands are difficult to
program for precise grasping, while the 2 kg payload limit per arm makes heavier objects such as
tables and chairs impractical to handle. Furthermore, PARKROYAL also identified plates as their
desired starting point for automation.

### 1.4. Existing Robotic Solutions

Several types of robots already automate material handling and service operations (Figure
[[fig:existing-robots]]). Restaurant delivery robots, such as Pudu Robotics' BellaBot, transport
food on trays with capacities of up to 10 kg each [6], but rely on staff or diners to retrieve the
items. Similarly, AGVs and AMRs handle bulk transportation. The MiR250 Hook can tow carts weighing
up to 500 kg [7], while PARKROYAL's AGV, introduced in Section 1.2, will carry plate stacks to the
banquet hall entrance.

Mobile manipulators can both transport and handle objects. Toyota's Human Support Robot, for
example, retrieves items and performs household tasks, but has a maximum payload of 1.2 kg [8].
Humanoid robots are also entering logistics operations. Agility Robotics' Digit transfers totes onto
conveyors in GXO warehouses [9]. However, these applications typically involve repetitive tasks in
structured environments, unlike banquet halls where table layouts and plate positions vary between
events.

Existing solutions therefore address transportation or object manipulation, but adapting both
capabilities to variable banquet layouts remains a challenge. VATEL explores this gap by combining
autonomous navigation and plate manipulation in a human-oriented environment.

![fig:existing-robots | Existing robots: (a) Pudu BellaBot; (b) MiR250 AMR; (c) Toyota HSR; (d) Agility Digit. Photos: (a) J. R. McPherson, CC0; (b) AnnaEbdrup, CC BY-SA 4.0, via Wikimedia Commons; (c) [8], CC BY 4.0; (d) [9]](../assets/interim/background-existing-robots.webp)

With this narrower focus, the project seeks to address the following problem: *How could we use the
Unitree R1 humanoid robot to optimize banquet setup by automating plate setup?*

### 1.5. Project Scope and Objectives

Based on our understanding of typical banquet setup [5] and discussions with PARKROYAL, we propose a
five-step plate-setting workflow(Figure [[fig:workflow]] and Table [[tab:workflow]]). As PARKROYAL
has shared limited operational details so far, this initial proposal includes assumptions that will
be verified with the hotel next semester. The workflow follows the necessary sequence: the banquet
team arranges the table layout, staff load plate stacks onto a dish dolly, and PARKROYAL's AGV
transports them from the kitchen to the banquet hall entrance. Our project focuses on the remaining
work **inside the hall** (Figure [[fig:hall-actual]] and Table [[tab:scope]]): retrieving plates
from the AGV and placing them at their designated seats. Since table layouts vary between events,
the robot must also identify the positions of each table and seat. Moving tables and chairs,
however, remains outside the project scope.

![fig:workflow | Plate-setting workflow: (a) the five steps, with VATEL's two in the hall; (b) the robot carrying plates from the AGV to each seat](../assets/interim/intro-workflow.webp)

![fig:hall-actual | The banquet hall at PARKROYAL COLLECTION Pickering](../assets/interim/intro-hall-actual.webp)

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

Together, these objectives form the minimum chain for VATEL to set plates on its own: the robot must
know where each plate goes (1), reach each table reliably (2) and handle the plates once there (3).
If any link is missing, staff would still have to step in and little labour would be saved; each
objective is also the goal of one subsystem (Chapters 5, 7 and 8).

Table: tab:scope | Project scope
| In scope | Out of scope |
| --- | --- |
| In-hall operations, from the AGV at the entrance to individual seats | Kitchen-to-hall transportation (handled by PARKROYAL's AGV) |
| Porcelain dinner plates, one per trip | Cutlery, glassware, bowls and linen |
| Knowing the event's table layout and each plate's position | Moving tables and chairs |
