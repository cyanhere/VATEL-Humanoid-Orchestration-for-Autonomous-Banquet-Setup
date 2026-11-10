---
id: introduction
author: Teh Wei Sheng
---

## 1. Introduction

### 1.1. Problem of Interest

Banquet halls typically begin as empty spaces that must be transformed into fully prepared dining
venues within a few hours, with all table settings arranged manually before guests arrive (Figure
[[fig:banquet-setup]]). A typical banquet comprising 15 to 20 tables, with 10 place settings per table
and approximately 10 items per setting, requires staff to manually position an estimated 1,500 to
2,000 individual items. This makes banquet preparation a labour-intensive and time-consuming process,
particularly when tight setup schedules must be met.

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

The plate-setting workflow consists of five steps (Figure [[fig:workflow]] and Table
[[tab:workflow]]), as summarised below. The industry partner is deploying an **automated guided vehicle (AGV)** to
transport plate stacks from the kitchen to the banquet hall entrance. VATEL focuses on the subsequent
in-hall operations: planning the table layout and using the humanoid robot to retrieve and distribute
plates.

![fig:workflow | The plate-setting workflow: (a) the five steps, with the parts VATEL handles in green; (b) inside the hall, the robot carries each plate from the AGV at the door to its seat, on a layout that changes from one event to the next](../assets/interim/intro-workflow.webp)

Table: tab:workflow | The current plate-setting workflow, and the part VATEL addresses
| Step | Process | Responsible system |
| --- | --- | --- |
| 1 | Arrange table layout | VATEL Planner |
| 2 | Load plate stacks onto dish dolly | Manual |
| 3 | Transport plates to banquet hall | AGV (partner) |
| 4 | Retrieve and transport plates to tables | Humanoid robot |
| 5 | Place plates at designated seats | Humanoid robot |

Thus, the main stakeholders are:

- **Partner's Banquet Operations Team:** Primary users responsible for banquet hall preparation and
  operation of the VATEL Planner.
- **Industry Partner's Management:** Oversees operational requirements and potential system
  adoption.
- **EDIC Department:** Provides the robotic platform and technical support.

### 1.3. Value Proposition and Novelty

Existing banquet equipment, such as trolleys, dollies and racks, helps staff transport items in bulk
(Section 2.1), but does not automate the placement process. VATEL addresses this gap by introducing a
humanoid robot that can perform repetitive plate-setting tasks consistently, reducing reliance on
temporary labour and minimising human handling of tableware. Unlike fixed automation, which is often
limited by changing venue layouts, a humanoid robot can operate in spaces designed for people,
provided it has an accurate representation of the venue and can manipulate tableware designed for
human use.

VATEL addresses these challenges through three key contributions:

- **automatic layout capture** from the hall's existing CCTV, turned into a static map the robot can
  navigate with;
- **a planner simple enough for banquet staff**, which turns the event plan into a goal for every
  place setting;
- **a push-to-grasp strategy** that lets a humanoid hand with few joints take one plate off a tightly
  nested stack.

### 1.4. Objectives and Scope

Plates were selected as the initial focus due to the R1's manipulation limitations. Its hands are
difficult to program for fine-grasp tasks, while its 2 kg payload per arm limits its ability to handle
heavier objects such as tables and chairs. Plates were also identified by the industry partner as the
simplest starting point for automation.

With this narrower focus, the project seeks to address the following problem: *How could we use the
Unitree R1 humanoid robot to optimise banquet setup by automating plate setup?*

By the end of CDE4301, VATEL aims to achieve three objectives:

1. Transfer the banquet hall layout from the planner to the robot.
2. Navigate autonomously to each designated table position and stop accurately and consistently.
3. Retrieve plates from the AGV and place them at their designated table positions.

Table: tab:scope | Project scope
| In scope | Out of scope |
| --- | --- |
| In-hall operations, from the AGV at the entrance to individual seats | Kitchen-to-hall transportation (handled by the partner's AGV) |
| Porcelain dinner plates, one per trip | Cutlery, glassware, bowls and linen |
| Table layout planning and plate-position assignment | Moving tables and chairs |
