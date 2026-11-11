---
id: concept
---

## 4. Concept Development

### 4.1. Functional Decomposition

The gaps in Table [[tab:gap]] break down into three functions and the questions each must answer
(Table [[tab:functions]]).

Table: tab:functions | The three functions VATEL must provide
| Function | Questions to answer |
| --- | --- |
| Instructions | How do we capture the hall's current layout? How does the robot learn the planned event layout? How is the plan turned into tasks for the robot? |
| Navigation | How does the robot know where it is? How does it plan a path to its destination? How does it find the table's edge? |
| Plate manipulation | How does the robot know where the plate is? How does it separate the top plate from the stack? How does it hold the plate? |

### 4.2. Morphological Charts and Selection

We developed morphological charts to compare possible solutions for each design question, with the
selected options highlighted in bold.

Table: tab:morph-instructions | Morphological chart: instructions
| Question | Option A | Option B | Option C | Option D |
| --- | --- | --- | --- | --- |
| 1. Capture the current hall layout | Staff measure and enter it | Robot surveys the hall using LiDAR | **Use existing hall CCTV** | Camera reads tags on each table |
| 2. Input the planned event layout | Spoken or written instructions | Select a stored layout template | Import a floor plan | **Staff arrange tables on a map in an app** |
| 3. Generate plate-placement tasks | Use a large language model | **Apply fixed rules based on table size and shape** | Staff mark every position manually |  |

Each option was evaluated against the requirements in Section 3. For hall mapping, existing CCTV was
selected as it covers the venue without requiring additional hardware or a preliminary robot survey.
For event planning, an app-based interface allows staff to specify each event's layout directly,
avoiding the limitations of fixed templates and inconsistent floor plan formats. Finally, fixed
rules were chosen to generate plate positions consistently and eliminate the need for staff to
manually mark each setting.

The selected options were combined into three concepts (Table [[tab:concepts]]) and evaluated
against the project requirements. The VATEL Planner was the only concept that met all four
requirements and was therefore selected for implementation (Section 5).

Table: tab:concepts | Three concepts combined from the instructions chart, scored against the requirements in Section 3 (● meets, ◐ partly, ○ does not)
| Concept | Options | Adapts to every layout | Little staff effort | Almost no training | Same, checkable result |
| --- | --- | --- | --- | --- | --- |
| LLM-Based Planning | 1B, 2A, 3A | ● | ● | ◐ | ○ |
| Manual Layout Planning | 1A, 2C, 3C | ● | ○ | ◐ | ◐ |
| **VATEL Planner** | 1C, 2D, 3B | ● | ● | ● | ● |

Table: tab:morph-navigation | Morphological chart: navigation
| Question | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Know where the robot is | **Lidar with its built-in IMU** | Hall cameras track the robot in real time | Follow a mapped trail |
| Plan a path | Line following along a provided trail | **A\* on a Nav2 costmap** |  |
| Find the table's edge | AprilTag and a vision model | **RealSense depth map** | RFID reader on each table |

Lidar localisation needs nothing installed in the hall, while tracking the robot by CCTV would add
latency and fail where the view is blocked. A trail would have to be laid again for every layout; a
costmap planner adapts to any layout. The depth camera measures the distance to a table directly,
while tags or RFID would have to be fitted to every table.

Table: tab:morph-manipulation | Morphological chart: plate manipulation
| Question | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Find the plate | **The robot's own camera** | The hall's cameras | Plates always in a fixed spot |
| Separate the top plate | Copy how people do it | **Push it sideways to make a grip point** |  |
| Hold the plate | Parallel gripper | **Dexterous hand** | Suction end effector |

The robot's own camera sees the stack from close range, far more accurately than the hall cameras.
Section 8.3 explains why the R1 cannot copy how people lift a plate off a stack, which led to
pushing it instead. The dexterous hands come with the platform and keep other tableware within reach
later.

> TODO: If the team scored the options (an evaluation matrix with weighted criteria), add it here;
> the rubric rewards a systematic comparison (A2).

### 4.3. Selected Concept and System Architecture

The selected concept has three subsystems (Figure [[fig:architecture]]): the planner on the user's
side, and navigation and plate manipulation on the robot.

![fig:architecture | System architecture: planner on the user's side, navigation and manipulation on the R1](../assets/interim/arch-subsystems.webp)

Figure [[fig:pipeline]] shows the full pipeline. The banquet team plans the event in the VATEL
Planner, which hands the robot a static map with the tables and every set position. The robot
navigates from that map, checks the real table positions with its lidar, and takes plates one at a
time from the AGV to their positions.

![fig:pipeline | The VATEL pipeline: segments, hand-offs and success criteria](../assets/interim/vatel-pipeline.webp)

Sections 5 to 8 cover the planner, the sensor helmet, navigation, and plate perception and
manipulation.
