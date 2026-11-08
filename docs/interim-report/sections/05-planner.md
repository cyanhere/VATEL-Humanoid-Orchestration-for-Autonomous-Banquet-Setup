---
id: planner
author: Teh Wei Sheng
---

## 5. VATEL Planner

### 5.1. Purpose

The VATEL Planner is a browser app the banquet team uses to plan the hall and send the plan to the
robot (Figure [[fig:planner-app]]). It uses the hall's existing CCTV to draw every table on a 2-D map,
so no one measures the floor plan by hand; staff plan the event on it, and the planner works out every
place setting and where the robot stands for each. Because its
users are banquet staff, the main screen needs only two numbers and two buttons; everything only the
robot needs sits in a Settings dialog.

![fig:planner-app | The VATEL Planner's main page](../assets/hardware/vatel-planner.webp)

### 5.2. Design Choices

Table: tab:planner-choices | Design choices within the planner (selected option in bold)
| Function | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Remove people from the picture | Use the live frame | Background subtraction (MOG2) | **Temporal median** |
| Find the tables | YOLO (trained detector) | **Grounding DINO + SAM** | |
| Picture position to room position | **Calibrated ray–plane intersection** | Depth camera | |
| Refine each table | Bounding box | Convex hull | **Far-edge fit** |

Background subtraction (Zivkovic, 2004) lets a still table fade into the background; a temporal
median keeps whatever stays put. YOLO (Redmon et al., 2016) needs hundreds of labelled hall frames we
do not have, while Grounding DINO and SAM need only a text prompt. A depth camera cannot cover a hall
from a wall. Boxes and hulls include legs and chairs; the far-edge fit does not.

### 5.3. From CCTV to a 2-D Static Map

Figure [[fig:planner-pipeline]] summarises the six steps.

![fig:planner-pipeline | From the hall's CCTV to the planner: clean plate, detection and segmentation, ray–plane intersection and edge clean-up, with a one-time calibration](../assets/interim/planner-pipeline.webp)

**Step 1: clean plate.** Staff walking through the picture would confuse the detector. The planner
keeps a frame every 2 s for the last minute (31 frames) and takes the median of each pixel (Figure
[[fig:clean-plate]]). A table covers its pixels the whole time and a person only briefly, so the
median is the furniture (Cucchiara et al., 2003). A moved table appears in its new place within about
a minute.

![fig:clean-plate | Clean plate: the median of each pixel over the last minute keeps the furniture and drops the people](../assets/interim/planner-clean-plate.webp)

**Step 2: find the tables.** Grounding DINO (Liu et al., 2024) draws a box around every object that
matches the prompt "table", and SAM (Kirillov et al., 2023) turns each box into the table's exact
outline. Our code splits boxes that cover several tables, drops duplicates and cuts chairs out of the
table outlines.

![fig:detect-segment | Step 2 on real CCTV of the Tinkering Corner: Grounding DINO's boxes (left) and SAM's outlines (right)](../assets/interim/planner-detect-segment.webp)

**Step 3: calibrate the camera.** Each camera is calibrated once (Figure [[fig:calibration]]) from
points whose floor-plan position is known, edges that must stand upright, which the code finds itself,
and tabletops, which must be rectangles seen from above. A least-squares fit finds the focal length,
lens distortion, position and orientation that best fit these clues (Hartley & Zisserman, 2004;
Zhang, 2000), checked by drawing the floor plan back onto the picture. For a test-room camera it gave
a height of 2.57 m and a distortion coefficient of −0.19, with a 13-pixel residual on a 2,560-pixel
picture.

![fig:calibration | Calibrating a camera once: match a few points between the picture and the floor plan, solve for the camera, and check the result](../assets/interim/planner-calibration.webp)

![fig:calibration-check | The check on a real camera: the floor plan's wall bases (green) and pillars (orange) drawn back through the solved camera](../assets/interim/planner-calibration-check.webp)

**Step 4: ray–plane intersection.** Every pixel is a ray into the room (Figure [[fig:ray]]). A
table's outline lies on its top, so the ray is followed to the tabletop's height, not the floor. For a
camera 3 m up and a ray 18° below horizontal, the ray drops 2.25 m to the tabletop, which is therefore
2.25 / tan 18° ≈ 6.9 m away. Following it to the floor would put the table 2.3 m too far away.

![fig:ray | A pixel is a ray into the room; stopping it at the tabletop's height gives the table's true position](../assets/interim/planner-ray.webp)

**Step 5: clean up each table.** SAM's outline also catches legs and chairs. These sit lower than
the tabletop, so their rays reach the tabletop's height early and land short, nearer the camera. Seen
from the floor point under the camera, the farthest outline point in every direction is the tabletop's
real edge (Figure [[fig:far-edge]]), and a rectangle or circle is fitted to those points.

![fig:far-edge | Keeping the far edge: lower parts always land short, so the farthest points are the tabletop's edge](../assets/interim/planner-far-edge.webp)

![fig:cleanup | Step 5 on real CCTV: SAM's raw masks (green) and the cleaned-up tabletops (red)](../assets/interim/planner-cleanup.webp)

**Step 6: snap onto the map.** Each table is matched to the nearest size the venue owns (Figure
[[fig:snap]]), or keeps its measured size if it is more than 15% off every stock size. The planner
places every setting, with a spot for the robot 0.45 m from the edge, and in live mode updates every
1.5 s, so the plan follows when staff move a table. The plan is exported as the static map the
navigation stack loads (`.pgm` and `.yaml`), with a named goal for every place setting.

![fig:snap | Snapping a measured table to the venue's stock, with its sets and the robot's stopping points](../assets/interim/planner-snap.webp)

![fig:to-robot-map | The Tinkering Corner's six tables in the planner (left), and drawn into the robot's lidar map as the exported static map (right)](../assets/interim/planner-to-robot-map.webp)

### 5.4. Tests and Results

Table: tab:planner-tests | Planner tests in Semester 1
| Test | Method | Result |
| --- | --- | --- |
| Simulated hall | Corner CCTV camera, 23 tables at known positions | All 23 found; centre error 2.3 cm mean, 6.6 cm worst |
| Height correction | Same scene, with and without it | Offsets of 223 mm and 329 mm reduced to 0 mm |
| Real CCTV | Recorded footage of the Tinkering Corner, NUS Innovation & Design Hub | All 6 tables found and placed on the floor plan |
| Devices | Tablet and laptop | Works in both |
| Hand-off | Export `.pgm` and `.yaml` | Accepted by the navigation stack |

![fig:floorplan-result | Real CCTV result: the tables found by SAM placed on the Tinkering Corner's floor plan in metres, with the camera's field of view shaded and the walls used to check distances in blue](../assets/interim/planner-floorplan-result.webp)

> TODO (Wei Sheng): add the measured position error in the Tinkering Corner (tape-measured gaps from
> each table to the walls against the planner's).

### 5.5. Problems and Next Steps

Three problems shaped the design. A four-corner homography cannot model wide lenses, tabletop
height or several cameras, which led to the full camera model. SAM's masks included legs and chairs,
which led to the far-edge fit. A box spanning two rows of tables once made the clean-up drop the second row; it
now keeps any part of a mask that smaller ones do not explain.

The planner has not yet been verified in the partner's hall. Next semester we will verify its
accuracy there, finish the live mode that updates the map from the CCTV stream, test the interface
with banquet staff, and send the map straight to the robot once the subsystems are integrated.
