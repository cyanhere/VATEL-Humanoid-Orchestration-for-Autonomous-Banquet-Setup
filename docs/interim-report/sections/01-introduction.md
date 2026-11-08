---
id: introduction
author: Team
---

## 1. Introduction

### 1.1. Problem of Interest

Every banquet starts as an empty hall that, within a few hours, has to become a room of dressed
tables with every setting laid out, all by hand. A banquet of 15 to 20 tables, with 10 settings per
table and about 10 items per setting, means 1,500 to 2,000 items placed by hand before the first
guest arrives.

Such events are frequent in Singapore. Business events (meetings, incentives, conventions and
exhibitions) brought in S$2.3 billion of tourism receipts in 2025, up 35% on 2024 (Ministry of Trade
and Industry, 2026), and 24,688 marriages were registered in 2025, many celebrated with a banquet
(Department of Statistics Singapore, 2026). Demand comes in peaks, and venues staff up for them.

> TODO: The deck's "~14,800 hotel wedding banquets in 2025" has no source that gives it (SingStat
> counts registered marriages, not banquets): cite its source or keep the 24,688 marriages. Consider
> adding the labour-cost evidence from the earlier draft slides.

Our first problem statement was broad: *how could banquet setup be less tedious and better
optimised?*

### 1.2. Context and Stakeholders

Two conditions were set at the start. The College's EDIC provided a **Unitree R1 humanoid robot**
as the development platform, and the project works with an **industry partner** (name withheld under
a non-disclosure agreement).

Plates reach their seats in five steps (Table [[tab:workflow]]). The partner is deploying an **AGV**
(automated guided vehicle) to bring plate stacks from the kitchen to the hall door, covering the
transport steps, and envisions a humanoid robot transferring the plates from the AGV to the tables.
Our project therefore works **inside the hall**: planning the layout and setting the plates.

Table: tab:workflow | The current plate-setting workflow, and the part VATEL addresses
| Step | Today, by hand | With VATEL |
| --- | --- | --- |
| 1 | Set the table layout for the event | VATEL Planner |
| 2 | Load plate stacks from storage onto a dish dolly | AGV (partner) |
| 3 | Move the dolly to the banquet hall | AGV (partner) |
| 4 | Bring plates from the dolly to each table | Robot |
| 5 | Set each plate at its seat | Robot |

The stakeholders are the partner's banquet team, who set up the halls and would use the planner,
the partner's management, and EDIC.

### 1.3. Value Proposition and Novelty

Venues' trolleys, dollies and racks help staff move items in bulk (Section 2.1), but none places
anything. A robot that does has less downtime than temporary staff, follows the operating procedure
exactly, and raises fewer hygiene concerns. Fixed automation does not suit a layout that changes every
event; a humanoid can work in a space built for people if it knows tonight's layout and can handle
tableware made for human hands. VATEL offers three pieces for that:

- **automatic layout capture** from the hall's existing CCTV, turned into a static map the robot can
  navigate with;
- **a planner simple enough for banquet staff**, which turns the event plan into a goal for every
  place setting;
- **a push-to-grasp strategy** that lets a humanoid hand with few joints take one plate off a tightly
  nested stack.

### 1.4. Objectives and Scope

We chose plates first: the R1's hands are hard to program for utensils needing a fine grasp, its
2 kg payload per arm rules out tables and chairs, and plates are the partner's own starting point as
the least complex item. The refined
problem statement is: *how could we use the Unitree R1 humanoid robot to optimise banquet setup by
automating plate setup?* By the end of CDE4301, VATEL should:

1. transfer the banquet hall configuration from the planner to the robot;
2. reach and stop accurately and consistently at each planned destination, autonomously;
3. transfer plates from the AGV to the table.

Table: tab:scope | Project scope
| In scope | Out of scope |
| --- | --- |
| Inside the hall, from the AGV at the door to each seat | Transport from the kitchen (the partner's AGV) |
| Porcelain dinner plates, one per trip | Cutlery, glassware, bowls and linen |
| Planning the layout and set positions | Moving tables and chairs |
