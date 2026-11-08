---
id: concept
author: Team
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

For each question we compared two or three options in a morphological chart; the selected option
is in bold in the three charts below.

Table: tab:morph-instructions | Morphological chart: instructions
| Question | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Capture the current hall layout | **Read it from the hall's existing cameras** | Staff measure and enter it | The robot surveys the hall itself |
| Learn the planned event layout | Spoken or written instructions to the robot | **A visual planner app** | |
| Turn the plan into tasks | A large language model plans the steps | **A fixed program generates every set position** | |

The hall's CCTV is already installed and sees the whole room, so it needs no new hardware or staff
time. Staff plan visually and the robot cannot parse speech, so the layout is drawn in a planner. Set
positions follow fixed rules, so a deterministic program gives the same, checkable answer every time,
which a language model cannot guarantee.

Table: tab:morph-navigation | Morphological chart: navigation
| Question | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Know where the robot is | **Lidar with its built-in IMU** | Hall cameras track the robot in real time | Follow a mapped trail |
| Plan a path | Line following along a provided trail | **A\* on a Nav2 costmap** | |
| Find the table's edge | AprilTag and a vision model | **RealSense depth map** | RFID reader on each table |

Lidar localisation needs nothing installed in the hall, while tracking the robot by CCTV would add
latency and fail where the view is blocked. A trail would have to be laid again for every layout; a
costmap planner adapts to any layout. The depth camera measures the distance to a table directly,
while tags or RFID would have to be fitted to every table.

Table: tab:morph-manipulation | Morphological chart: plate manipulation
| Question | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Find the plate | **The robot's own camera** | The hall's cameras | Plates always in a fixed spot |
| Separate the top plate | Copy how people do it | **Push it sideways to make a grip point** | |
| Hold the plate | Parallel gripper | **Dexterous hand** | Suction end effector |

The robot's own camera sees the stack from close range, far more accurately than the hall cameras.
Section 8.3 explains why the R1 cannot copy how people lift a plate off a stack, which led to pushing
it instead. The dexterous hands come with the platform and keep other tableware within reach later.

> TODO: If the team scored the options (an evaluation matrix with weighted criteria), add it here;
> the rubric rewards a systematic comparison (A2).

### 4.3. Selected Concept and System Architecture

The selected concept has three subsystems (Figure [[fig:architecture]]): the planner on the user's
side, and navigation and plate manipulation on the robot.

![fig:architecture | System architecture: the VATEL Planner on the user's side, and navigation and plate manipulation on the R1](../assets/interim/arch-subsystems.webp)

Figure [[fig:pipeline]] shows the full pipeline. The banquet team plans the event in the VATEL
Planner, which hands the robot a static map with the tables and every set position. The robot
navigates from that map, checks the real table positions with its lidar, and takes plates one at a
time from the AGV to their positions.

![fig:pipeline | The VATEL pipeline: the four segments, what each one passes on, and the success criteria](../assets/interim/vatel-pipeline.webp)

Sections 5 to 8 cover the planner, the sensor helmet, navigation, and plate perception and
manipulation.
