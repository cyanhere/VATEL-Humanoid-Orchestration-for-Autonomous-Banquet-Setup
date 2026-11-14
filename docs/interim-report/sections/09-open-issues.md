---
id: open-issues
---

## 9. Open Issues and Targeted Workflow

### 9.1. Problems and Open Issues

Tables [[tab:open-planner]], [[tab:open-navigation]] and [[tab:open-manipulation]] list the problems
met in each subsystem, how the solved ones were addressed, and the work needed to close the rest in
Semester 2 (Chapter 10).

Table: tab:open-planner | Problems: VATEL Planner
| Problem | Solution or work needed |
| --- | --- |
| A four-corner homography could not handle wide-angle lens distortion, tabletop height or multiple cameras | Solved: a full camera model, calibrated once per camera (Section 5.3, Appendix 5B) |
| SAM's masks included table legs and chairs | Solved: the far-edge fit (Section 5.3, Appendix 5C) |
| A bounding box spanning two rows of tables made the clean-up discard the second row | Solved: mask regions not already explained by smaller masks are kept |
| Not yet tested in PARKROYAL's hall | Validate detection and positioning accuracy with the hall's CCTV |
| Live mode, which updates the map from the CCTV stream, is not complete | Complete live map updates from the CCTV stream |
| Interface not yet evaluated with banquet staff | User-test the planner with PARKROYAL's banquet team |
| The map is exported as files, not sent to the robot | Send the map directly to the robot once the subsystems are integrated |

Table: tab:open-navigation | Problems: navigation
| Problem | Solution or work needed |
| --- | --- |
| The robot does not know where it starts in the rough map, so its own map and the rough map are not aligned | Localise the robot within the rough map at the start of each run |
| The rough map is not yet connected to the navigation system | Connect the VATEL Planner's rough map to the mission planner |
| All tests so far ran on a Unitree G1 borrowed from the BEADZ Lab | Move the stack to the R1 with the JT128 LiDAR and the helmet RealSense, and retest |
| The AGV detection criterion and fine-approach method are not yet developed | Compare the fine-approach options (Appendix 7E), define when an AGV detection is trusted (Appendix 7C), then test the AGV leg |
| The robot does not stop centred on the table edge: the fine approach corrects only distance and heading, and the sideways position from LiDAR localisation is not accurate enough | Add a sideways correction to the fine approach, such as locating the table's ends in the RealSense image, and measure the sideways offset in the approach tests |

Table: tab:open-manipulation | Problems: plate manipulation
| Problem | Solution or work needed |
| --- | --- |
| Lifting shown only in simulation; on the robot, the receiving hand has closed on the raised rim | Complete the lift on the R1 and make picks consistent |
| The motion is a hand-tuned state machine | Train a reinforcement-learning policy that generalises and covers edge cases |
| Wrist-camera rim tracking tested only in simulation | Integrate it on the robot |
| Rim breaking load only estimated (120–165 N) | Measure it in a static test and set the finger force limits from it |
| The AGV's stack holder is undefined, so the working height may differ | Evaluate the height-controlled locomotion policy with the pick sequence |
| How plates are carried to the table is not decided | Compare single plates, a tray and a trolley |
| Placing the plate at its set position is not yet developed | Develop the place motion: lower and release the plate, then check its position with the head camera against T14 |

### 9.2. Targeted Workflow

Once these issues are closed, a full run will follow Figure [[fig:target-workflow]], from planning
the event in the VATEL Planner to placing the last plate.

![fig:target-workflow | Targeted end-to-end workflow (animated)](../assets/interim/target-workflow.webp)
