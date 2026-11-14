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
| Navigation | Where is the robot? Where should it go next? What is the best path? How does it walk there safely? How does it stop square at the table’s edge? |
| Plate manipulation | How does the robot know where the plate is? How does it separate the top plate from the stack? How does it hold the plate? |

### 4.2. Morphological Charts and Selection

For each function, a morphological chart lists the options for every question, with the selected
ones in bold. The options were then combined into concepts and scored against the requirements in
Chapter 3 (Table [[tab:specs]]).

#### 4.2.1. Instructions

Table: tab:morph-instructions | Morphological chart: instructions
| Question | Option A | Option B | Option C | Option D |
| --- | --- | --- | --- | --- |
| 1. Capture the current hall layout | Staff measure and enter it | Robot surveys the hall using LiDAR | **Use existing hall CCTV** | Camera reads tags on each table |
| 2. Input the planned event layout | Spoken or written instructions | Select a stored layout template | Import a floor plan | **Staff arrange tables on a map in an app** |
| 3. Generate plate-placement tasks | Use a large language model | **Apply fixed rules based on table size and shape** |  |  |

Of the three concepts built from the selected options (Table [[tab:concepts]]), only the VATEL
Planner met all four requirements, so it was implemented (Chapter 5); Appendix 4A gives the
reasoning for each row.

Table: tab:concepts | Instruction concepts (● meets, ◐ partly, ○ does not)
| Concept | Options | Adapts to every layout | Little staff effort | Almost no training | Same, checkable result |
| --- | --- | --- | --- | --- | --- |
| LLM-Based Planning | 1B, 2A, 3A | ● | ● | ◐ | ○ |
| Manual Layout Planning | 1A, 2C, 3B | ● | ○ | ◐ | ◐ |
| **VATEL Planner** | 1C, 2D, 3B | ● | ● | ● | ● |

#### 4.2.2. Navigation

Table: tab:morph-navigation | Morphological chart: navigation
| Question | Option A | Option B | Option C | Option D |
| --- | --- | --- | --- | --- |
| 1. Know where the robot is | Leg odometry and IMU only | Hall CCTV tracks the robot | Visual SLAM on a camera | **LiDAR–inertial odometry** |
| 2. Choose where to go next | Staff send each goal | Follow the planner's goal list in order | **State machine that alternates between AGV and table** |  |
| 3. Plan the path | Follow a trail laid on the floor | Sampling planner (RRT\*) | **A\* on a costmap from the map and live LiDAR** |  |
| 4. Walk the path past people | Pure pursuit: track the path, stop when blocked | **DWB: sample reachable speeds, score each** | MPPI: optimise many sampled trajectories |  |
| 5. Stop square at the table's edge | AprilTag on each table | RFID tag on each table | LiDAR scan of the table | **RealSense depth: detect the tabletop edge** |

The choices were made before prototyping, from the robot's sensors and software and published
performance; Appendix 4B gives the reasoning for each row.

Table: tab:nav-concepts | Navigation concepts (● meets, ◐ partly, ○ does not)
| Concept | Options | Adapts to every layout | Nothing fitted to the hall or tables | Runs without staff | Square stop at 20 cm (T10) |
| --- | --- | --- | --- | --- | --- |
| Infrastructure-guided | 1B, 2B, 3A, 4A, 5B | ○ | ○ | ◐ | ○ |
| Camera-only | 1C, 2B, 3B, 4A, 5A | ◐ | ◐ | ◐ | ● |
| **LiDAR-based navigation** | 1D, 2C, 3C, 4B, 5D | ● | ● | ● | ● |

Only LiDAR-based navigation was expected to meet all four requirements (Table [[tab:nav-concepts]]),
so it was taken forward; Section 7.3 gives the first measured results.

#### 4.2.3. Plate Manipulation

Table: tab:morph-manipulation | Morphological chart: plate manipulation
| Question | Option A | Option B | Option C | Option D |
| --- | --- | --- | --- | --- |
| 1. Find the plate | **Head depth camera** | Hall CCTV | Stack at a fixed position | Wrist cameras |
| 2. Separate the top plate | Grasp in the rim gap, as people do | Press the near rim to tip the plate | **Push sideways, then grasp** | Tool or fixture |
| 3. Hold the plate | Thumb–middle pinch | Palm under, thumb above | **Rim in the index–middle slot** |  |
| 4. Carry plates to the table | **One plate in the hand** | Tray carried by the robot | Trolley pushed by the robot |  |

Of the three concepts built from the options (Table [[tab:manip-concepts]]), only push-and-grasp met
all four requirements, so it was developed; Section 8.4 gives the tests behind each choice and
Appendix 4C the reasoning for the other options.

Table: tab:manip-concepts | Plate manipulation concepts (● meets, ◐ partly, ○ does not)
| Concept | Options | Separates one plate, stack kept in place (T12) | Holds the plate through the lift (T11) | Tolerates ±15 mm pose error | Nothing added to the robot or AGV (N5) |
| --- | --- | --- | --- | --- | --- |
| Human-like grasp | 1A, 2A, 3A | ○ | ○ | ○ | ● |
| Fixture-assisted | 1C, 2D, 3B | ◐ | ◐ | ● | ○ |
| **Push-and-grasp** | 1A, 2C, 3C | ● | ● | ● | ● |

### 4.3. Selected Concept and System Architecture

The selected concept has three subsystems (Figure [[fig:architecture]]): the planner on the user's
side, and navigation and plate manipulation on the robot.

![fig:architecture | System architecture: planner on the user's side, navigation and manipulation on the R1](../assets/interim/arch-subsystems.webp)

Figure [[fig:pipeline]] shows the full pipeline. The banquet team plans the event in the VATEL
Planner, which hands the robot a static map with the tables and every set position. The robot
navigates from that map, checks the real table positions with its LiDAR, and takes plates one at a
time from the AGV to their positions.

![fig:pipeline | The VATEL pipeline: segments, hand-offs and success criteria](../assets/interim/vatel-pipeline.webp)

Sections 5 to 8 cover the planner, the sensor helmet, navigation, and plate perception and
manipulation.
