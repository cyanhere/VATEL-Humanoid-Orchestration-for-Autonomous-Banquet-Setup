---
id: planner
---

## 5. VATEL Planner

### 5.1. Purpose

The [VATEL
Planner](https://cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/planner/)
is a browser-based application that allows the banquet team to plan the hall layout and send the
plan to the robot (Figure [[fig:planner-app]]). Using the hall's existing CCTV, it automatically
maps every table onto a 2-D layout, eliminating the need for manual measurements. Staff can then
arrange the tables for the event, while the planner automatically determines each place setting and
the robot's corresponding stopping position. As the application is designed for banquet staff, the
main screen is kept simple. Robot-specific settings are accessed separately through a Settings
dialog.

![fig:planner-app | VATEL Planner main page](../assets/interim/planner-app.webp)

### 5.2. Design Choices

Table: tab:planner-choices | Design choices within the planner (selected option in bold)
| Function | Option A | Option B | Option C |
| --- | --- | --- | --- |
| Remove people from the picture | Use the live frame | Background subtraction (MOG2) | **Temporal median** |
| Find the tables | YOLO (trained detector) | **Grounding DINO + SAM** |  |
| Picture position to room position | **Calibrated ray–plane intersection** | Depth camera |  |
| Refine each table | Bounding box | Convex hull | **Far-edge fit** |

The planner builds on open-source machine-vision models, with Appendix 5A explaining their functions
and selection rationale. A temporal median was chosen over background subtraction [10], which
gradually absorbs stationary objects into the background. Grounding DINO with SAM was preferred over
a trained detector such as YOLO [11], which would require hundreds of labelled hall images that were
unavailable. A depth camera was also considered, but even a long-range model such as the Intel
RealSense D455 has a depth range of only 0.6 to 6 m and an 86° × 57° field of view [12], making it
unsuitable for covering an entire banquet hall from a wall-mounted position. Finally, bounding boxes
and convex hulls were rejected because they may include table legs and chairs, whereas the far-edge
fit provides a more accurate estimate of the tabletop boundary.

### 5.3. From CCTV to a 2-D Static Map

The planner was developed and tested in the [Tinkering
Corner](https://cde.nus.edu.sg/edic/facilities/innovation-hub/spaces-in-the-hub/), a space within
the NUS College of Design and Engineering's Innovation & Design Hub where we develop and test the
system alongside the robot (Figure [[fig:tinkering-corner]]). As we have regular access to this
space but limited access to PARKROYAL's banquet hall, preliminary development and testing were
conducted here to validate the concept.

![fig:tinkering-corner | The Tinkering Corner, Innovation & Design Hub](../assets/interim/planner-tinkering-corner.webp)

Figure [[fig:planner-pipeline]] summarises the six-step process.

![fig:planner-pipeline | CCTV-to-map pipeline](../assets/interim/planner-pipeline.webp)

**Step 1: Generate a clean plate.** People walking through the camera's view can interfere with
table detection. To remove them, the planner captures one frame every 2 s over the past minute (31
frames) and calculates the median value of each pixel (Figure [[fig:clean-plate]]). Since tables
remain stationary while people move through the scene, the median image retains the furniture while
filtering out people [13]. If a table is moved, its new position becomes visible in the clean plate
within approximately one minute.

![fig:clean-plate | Clean plate: a per-pixel median removes people](../assets/interim/planner-clean-plate.webp)

**Step 2: Detect and segment the tables.** Grounding DINO [14] identifies objects matching the text
prompt "table" and draws a bounding box around each detection. SAM [15] then converts each box into
a precise table outline. Our code further splits boxes containing multiple tables, removes duplicate
detections and excludes chairs from the table outlines.

![fig:detect-segment | Grounding DINO boxes (left) and SAM outlines (right) on Tinkering Corner CCTV](../assets/interim/planner-detect-segment.webp)

**Step 3: Calibrate the camera.** Each camera is calibrated once using three types of visual
references (Figure [[fig:calibration]]): points with known positions on the floor plan, vertical
edges detected automatically by the code, and tabletops assumed to be rectangular when viewed from
above. A least-squares fit estimates the camera's focal length, lens distortion, position and
orientation from these references [16], [17] (Appendix 5B). The calibration is then checked by
projecting the floor plan back onto the camera image. For the Tinkering Corner CCTV camera, this
produced an estimated camera height of 2.57 m and a distortion coefficient of −0.19, with a residual
error of 13 pixels in a 2,560-pixel-wide image.

![fig:calibration | One-time camera calibration](../assets/interim/planner-calibration.webp)

![fig:calibration-check | Calibration check: floor-plan walls (green) and pillars (orange) projected onto the image](../assets/interim/planner-calibration-check.webp)

**Step 4: Ray–plane intersection.** Every pixel corresponds to a ray extending into the room (Figure
[[fig:ray]]). Since the table outline lies on the tabletop, each ray is intersected with the
tabletop plane rather than the floor. For example, with a camera 3 m above the floor and a ray
angled 18° below the horizontal, the ray travels 2.25 m vertically to reach the tabletop, placing
the table 2.25 / tan 18° ≈ 6.9 m away. Repeating this for the table outline converts its image
coordinates into positions in the room.

![fig:ray | Ray–tabletop intersection gives the true table position](../assets/interim/planner-ray.webp)

**Step 5: Refine each table outline.** SAM's segmentation may also include table legs and chairs.
Since these are below the tabletop, their projected points fall closer to the camera when
intersected with the tabletop plane. By viewing the outline from the floor point directly below the
camera, the farthest point in each direction corresponds to the tabletop's actual edge (Figure
[[fig:far-edge]]). A rectangle or circle is then fitted to these points (Appendix 5C).

![fig:far-edge | Far-edge fit: the farthest points mark the tabletop edge](../assets/interim/planner-far-edge.webp)

![fig:cleanup | SAM masks (green) and refined tabletops (red)](../assets/interim/planner-cleanup.webp)

**Step 6: Snap tables onto the map.** Each detected table is matched to the nearest standard size
used by the venue (Figure [[fig:snap]]). If its measured dimensions differ by more than 15% from all
available standard sizes, the measured dimensions are retained. The planner then generates every
place setting and assigns a robot approach point 0.5m from the table edge. In live mode, the map
updates every 1.5 s to reflect table movements by staff. Finally, the plan is exported as a static
map (.pgm and .yaml) for the navigation stack, with a named goal assigned to each place setting
(parameters in Appendix 5D).

![fig:snap | Snapping a table to stock sizes, with its settings and robot stops](../assets/interim/planner-snap.webp)

![fig:to-robot-map | Tinkering Corner tables in the planner (left) and on the robot's LiDAR map (right)](../assets/interim/planner-to-robot-map.webp)

### 5.4. Tests and Results

Table: tab:planner-tests | Planner tests in Semester 1
| Test | Method | Result |
| --- | --- | --- |
| Simulated hall | Corner CCTV camera, 23 tables at known positions | All 23 tables detected; mean centre error of 2.3 cm and worst-case error of 6.6 cm |
| Height correction | Same scene, with and without it | Position offsets of 223 mm and 329 mm reduced to 0 mm |
| Real CCTV | Recorded footage of the Tinkering Corner | All 6 tables detected and placed on the floor plan |
| Devices | Tablet and laptop | Successfully runs on both devices |
| Hand-off | Export to .pgm and .yaml | Exported map successfully loaded by the navigation stack |

![fig:floorplan-result | Detected tables on the Tinkering Corner floor plan (m); camera view shaded, reference walls in blue](../assets/interim/planner-floorplan-result.webp)
