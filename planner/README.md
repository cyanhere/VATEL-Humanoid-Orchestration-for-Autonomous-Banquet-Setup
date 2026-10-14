# Planner (Steps 1–2)

The VATEL Planner: the banquet team's planning app, used from an iPad or a computer. It shows
the whole banquet hall, segments out the tables onto a 2-D map, lets the team plan the event's
arrangement and the sets each table needs, calculates the position of every set, and sends that
static map to the robot.

**Open it:** [cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/planner](https://cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/planner/). The app is one page,
[`docs/planner/index.html`](../docs/planner/index.html), published with the project site; nothing
to install. This folder holds what runs beside it: the hall-camera bridge, the robot link, the
static-map export, and their tests.

**Status:** planning, seating guests, checks, printing and saving work in any browser. The live
hall camera and the hand-over to the robot run end to end against a simulated hall and a test
receiver; neither has met the real CCTV or the R1 yet.

---

## Using it

Two numbers and two buttons get a banquet planned:

| | |
|---|---|
| **Guests** + **Seat guests** | Type how many are coming and press **Seat guests** (or Enter). An empty room gets just enough tables. With tables already there, it asks: *spread* the guests evenly across them, or *plan new tables* for that number. |
| **Hall camera** → **Go live** | The tables the hall's CCTV sees appear on the plan where they actually stand, and follow it as staff move them. Tables laid by hand make way for what the camera sees (one Undo puts them back). |
| **Click a table** | Its guests, with − / +. A table set for a different number from its size says so on the plan ("9 pax"). |
| **Robot** tab | **Send plan to robot**, then **Start mission**. Start sends the latest plan first if it changed, and asks before the robot moves. |

Everything else is optional: **Fill the room** lays a full banquet at a chosen aisle, **Add a
table / fixture** places single pieces, **Checks** lists what the robot cannot reach,
**Save** and **Print** do what they say. Press <kbd>?</kbd> for the shortcuts.

The navigation details -- the occupancy map, the goal names, the map frame -- are never in
the user's way. They live in **Settings** (the cog), for whoever sets the hall up.

### Settings, for whoever sets up the hall

| | |
|---|---|
| **Robot link** | Address of `robot_link.py` on the robot. Blank or unreachable, and **Send** downloads the map files instead. |
| **Robot map frame** | Where the room's south-west corner sits in the robot's own map, and its grid. |
| **Hall camera (CCTV)** | Address of `cctv_bridge.py`, and the one-time calibration: four floor points clicked on the live view. |
| **Import detections** | A one-off detection file, reviewed before anything lands on the plan. |
| **Static map for navigation** | The `.pgm` + `.yaml` + goals, or the JSON, as files. |
| **This layout** | Save and open layout files. A layout file carries these settings too. |

Settings are kept in the browser and in saved layout files, and are not part of Undo:
undoing a table move does not forget the robot's address.

---

## Running it with the hall camera and the robot

From this folder:

```bash
python cctv_bridge.py --demo            # a simulated hall seen from a corner CCTV
python robot_link.py                    # the robot's end of Send / Start (port 8780)
```

Open **http://localhost:8765/** (the bridge serves the planner too). Then, once: Settings →
**Calibrate on the live view** → *Use the camera's own points* (the simulation knows them;
on a real camera, click four floor points) → *Use these points*. Close Settings and press
**Go live**. In the demo, tables are rolled in one every few seconds and then nudged now and
then; the plan follows. Set Settings → Robot link to `http://localhost:8780`, and **Start
mission** writes the plan to `missions/` and logs the start.

For the real hall: `python cctv_bridge.py --source rtsp://...` and
`python robot_link.py --on-start "<the command that runs the mission>"` on the robot.
Measured on the simulated hall: all 23 tables found, centres within 2.3 cm on average
(6.6 cm worst) after calibration and the table-height correction.

---

## What this is, and what it is not

A ceiling-mounted CCTV camera gives a rough, static picture of where the tables are. This
app is where a manager corrects that picture and confirms the plan for the evening. What
comes out is a **prior**: obstacle footprints and named goals like `T07/cover03`.

It is **not** a localisation system and not a substitute for SLAM. That is a separate
subsystem (a groupmate's), which does auto-navigation with local obstacle avoidance. This
just hands it a guideline. The robot localises itself and avoids obstacles for real once
it is in the room.

It is also **nothing to do with the plate manipulation pipeline** in
[`../perception/`](../perception/). Positions here are good to roughly a hand's width, which is the
right precision for driving to a table and completely wrong for placing anything on one.
Anything the arms touch is re-detected at close range by the perception pipeline.

---

## Saving

**Save** writes a layout file, and that is the one that always works. A page opened
straight from disk may get an opaque origin, and then the browser refuses it any storage
at all -- "Keep in browser" is disabled when that happens rather than silently losing the
layout. Served from `cctv_bridge.py` (or any web server) it has storage like any page.

---

## Room presets and table stock

Hard-coded near the top of the script. The room names are generic presets; pick
**Custom** for any other room and type its size.

| | |
|---|---|
| Grand Ballroom | 20.0 × 21.0 m |
| Function Room A + B (partition open) | 18.0 × 9.7 m |
| Function Room A or B | 9.0 × 9.7 m |
| Table height | **0.762 m** (2.5 ft) |
| Round stock | 5 ft, 5.5 ft, 6 ft, 7 ft |
| Rectangle stock | Oblong 6×2.5 ft, IBM 6×1.5 ft |

Covers per round follow the trade rule of about 0.6 m of rim each, which reproduces the
usual 8 / 10 / 12 for 5 / 6 / 7 ft.

### Fixtures

Stage, dance floor, buffet, bar, reception desk, pillar, service door, dish return. Three
separate questions are tracked per fixture, and they genuinely differ:

- `nav` — what the export calls it: an obstacle, a zone, or an access point
- `solid` — does it stop the robot's footprint
- `clear` — may a table stand on it

A dance floor is **not solid** (the robot may cross it) but **no table goes on it**. A
doorway is neither. A dish return is both. Collapsing these into one flag put tables on
the dance floor.

### Robot envelope

| | | why |
|---|---|---|
| `ROBOT_RADIUS` | 0.30 m | G1 footprint as a circle, generously |
| `STAND_OFF` | 0.45 m | table edge to robot centre when working a cover |
| `MIN_AISLE` | 0.90 m | clear floor needed to pass between tables |
| `TRADE_AISLE` | 1.50 m | what banquet service normally wants |

**Assumptions, not measurements.** `MIN_AISLE` and `STAND_OFF` in particular should be
checked against the real robot before anyone plans a room around them.

---

## The camera

Nothing is installed in the hall yet. The whole path is built and runs end to end against
the simulated hall in `cctv_bridge.py --demo`.

1. **Calibrate, once.** Click four floor points on the camera's picture and say where each
   is on the plan. Room corners are the default; a camera in a corner cannot see the one
   below it, so tape marks or anything else it can see on the floor will do. Four points
   are a homography, so overhead or in a corner makes no difference.
2. **Detect.** `cctv_bridge.py` grabs frames, finds the tables and serves them as JSON in
   pixels. It runs wherever the camera is, not in the browser.
3. **Live.** With **Go live** the planner polls it every 1.5 s, maps each rim onto the
   floor and keeps the tables on the plan in step: moved when they move, added when one
   appears, dropped after three updates without it (one missed frame does not make a table
   blink). A table only changes size after three frames agree, and keeps the guests set
   on it. Under 50% confident, or outside the room, is ignored.
4. **Or review a file.** Settings → Import detections lists every detection with the stock
   it matched; nothing lands on the plan unticked.

### Two numbers you must measure once

A tabletop is 0.762 m **above** the floor the four corners calibrated. The ray through a
table's rim carries on past it to a floor point further from the camera, so without a
correction every table maps too far out and too big:

| camera height | a table 10 m from below the camera | a 1.83 m top reads |
|---|---|---|
| 4 m | 2.35 m out of place | 2.26 m (+43 cm) |
| 6 m | 1.45 m out of place | 2.09 m (+27 cm) |
| 8 m | 1.05 m out of place | 2.02 m (+19 cm) |

The whole effect is a scaling about the nadir — the floor point directly below the
camera — by `(H - 0.762) / H`, which is exact. Give Settings → Hall camera the camera's
height and where the nadir falls on the plan, and it disappears: measured in the browser,
223 mm and +329 mm of error become **0 mm and 0 mm**.

### The rim, not the centre

Detections carry `outline_px`, the rim as 48 points, and the planner maps those onto the
floor and fits a circle there. Sending a centre and a diameter is not enough: a round
table seen from anywhere but straight overhead projects as an ellipse, and the centre of
that ellipse is not the projection of the table's centre. That bias cost the plate
detector next door 9–14 mm; a banquet table is eight times bigger, so it costs about
10 cm — the planner's whole stated accuracy.

---

## Testing the camera path before there is a camera

`table_segmentation.py` finds tables with a **depth** camera (a RealSense). That is not
the sensor that ships — the hall's camera will be RGB only — and it is not meant
to be. It is here for two jobs:

1. It exercises the whole path end to end, with real hardware, in a room you can get into.
2. It gives you **ground truth for free**. Depth measures where the tables really are to
   a centimetre, with no hand-labelling, so an RGB detector can be scored automatically
   on every frame instead of by squinting at overlays. The RealSense is the ruler, not
   the thing being tested.

```bash
python3 table_segmentation.py --synthetic     # no camera at all
python3 table_segmentation.py --capture lab01 # save a frame
python3 table_segmentation.py lab01.npz       # detect on it
python3 test_table_segmentation.py            # 28 checks
```

It is the plate detector's own geometry, one storey up: there a plate stands on a table,
here a table stands on the floor. Same plane fit, same hole filling, same circle-in-its-
own-plane fit. Only the height band and the diameters change.

### What a D435 can actually see

Field of view decides the whole test, and it is smaller than people expect:

| mounted at | covers, at table height | six-foot rounds across |
|---|---|---|
| 2.0 m | 2.5 × 1.4 m | none fit |
| 2.5 m | 3.5 × 2.0 m | one |
| 3.2 m | 4.9 × 2.8 m | two |
| 3.5 m | 5.5 × 3.1 m | two |

So: **one table needs about 2.4 m of height, two need about 3.2 m.** You cannot reproduce
a thirty-table hall with one D435, and you do not need to — what you are testing is
the pipeline, not the capacity. Angle it down from a shelf or a tripod if you cannot get
it overhead; the homography does not care.

### The failure to expect first

In a hall the floor is the biggest flat surface in view. **In a lab it is not** — put
a camera two metres above one 6 ft round and the tabletop fills the frame, so a naive
dominant-plane fit returns the *table*, every table then measures 0.00 m above "the
floor", and nothing is found at all. `floor_plane()` checks whether a large slab of the
frame lies below the plane it just fitted, and re-fits on that if so. It is the single
most likely way a first lab test fails.

### Setting up the lab test

1. Tape out a rectangle on the floor and **measure it**. Pick **Custom** in the Room list
   and enter that size — the corner calibration maps the four clicked corners onto
   the selected room, so a lab still against a 20 × 21 m ballroom comes back scaled
   to a ballroom.
2. Mount the camera above it, ideally 2.5–3.2 m, and put a table inside the rectangle.
3. `python3 table_segmentation.py --live` — it writes `live_detections.json` and an
   overlay.
4. In the planner: Settings → Hall camera → load the still → click the four taped
   corners → enter the camera height and the nadir → Import detections → Review.
5. Compare what the planner places against the `measured` block in the JSON, which is
   what the depth camera actually saw. They should agree to a few centimetres. If they do
   not, the calibration is wrong, not the detector.

### Which segmentation model

**With depth, none.** Plane fit, height band, connected components, circle in the plane.
No training, no weights, deterministic, already tested. Use it wherever depth exists.

**For the RGB ceiling camera**, there is no labelled data and there will not be until the
camera is mounted, so start **zero-shot**: Grounding DINO prompted with something like
"round dining table" to get boxes, then SAM 2 for the masks (this pairing is usually
packaged as Grounded-SAM). SAM 2 alone in automatic-mask mode plus a shape filter also
works and is simpler to stand up. Latency does not matter here — this runs **once per
room setup**, not at thirty frames a second — so there is no reason to reach for a
small fast model first.

Once the camera is up, the review step labels data as a side effect: every table a
manager keeps or drops is a judgement on a real frame. A few hundred of those is enough
to fine-tune **YOLO-seg** if the zero-shot route proves too slow or too noisy.

Whatever the model, take the **mask outline** and send it as `outline_px`. A bounding box
throws away the shape the planner needs, and a centre carries the projective bias.

`detect_tables.py` (Hough circles) is a placeholder that fixes the JSON format, and what
`cctv_bridge.py --source ...` uses until a model replaces `detect_frame()`. It will find
chair backs and ceiling lights on a real ceiling view. **Do not ship it.**

### Choosing the model without waiting for the hardware

You do not need the ceiling camera to decide this. Run the candidates on overhead
photographs of banquet halls — there is no shortage of them, and they have the
clutter, glare and occlusion that a lab bench does not. That answers "which model" now;
the RealSense answers "does the pipeline work" now. Neither has to wait for the other.

---

## Export format

Three hand-offs, for three different readers.

### For the robot: robot_link.py

**Send plan** posts `vatel-plan/0.1` to `POST /plan`: the static-map JSON below, plus the
three files `map_server` loads (`ros.pgm_base64`, `ros.yaml`, `ros.goals_yaml`).
`robot_link.py` writes them to `missions/<time>_<name>/`. **Start mission** posts to
`POST /mission/start`, which runs the `--on-start` command with `{yaml}`, `{goals}`,
`{plan}` and `{dir}` filled in; `GET /status` reports `idle`, `ready` or `running`. The
robot side of that command -- nav stack plus plate laying -- is what plugs in next.

### For the navigation stack: a ROS occupancy grid

```bash
python3 export_ros_map.py plan.static-map.json -o maps/
python3 test_export_ros_map.py                  # 42 checks
```

Two modes. **Bare** draws a rectangular room with walls from the planner's own
dimensions, which is what the planner itself sends (its origin and grid set in Settings →
Robot map frame). **Composited** starts from a map somebody already has and draws only the
tables onto it, keeping its resolution, origin and extent exactly:

```bash
python3 export_ros_map.py plan.json --base ilounge.pgm --base-yaml ilounge.yaml
```

That is the honest split, and it is how it should work in the hall too: walls and pillars
are permanent and a lidar sees them far better than a ceiling camera ever will; tables
change every event and are what the camera is for. Redrawing the structure from our own
guess throws away good data and proves nothing — the robot would be localising
against a cleaned-up copy of its own earlier SLAM.

Writes `map.pgm`, `map.yaml` and `map_goals.yaml` — what `map_server` loads:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=map.yaml
```

| pixel | meaning |
|---|---|
| 0 | occupied — tables, solid fixtures, walls |
| 254 | free floor |
| 205 | unknown (outside the room) |

Obstacles are drawn at their **true footprint, not inflated**. Inflating by the robot
radius is the local costmap's job and doing it twice makes a room look impassable.

**The frame is the part that bites.** The planner measures y **south** from the
north-west corner, the way you read a floor plan. A ROS map measures y **north** from the
south-west, with `origin` giving the pose of the image's lower-left pixel. Those are
mirror images of each other. The export converts once, here:

```
X_ros = x_planner
Y_ros = room_height - y_planner
yaw_ros = -heading_planner
```

Get it wrong and nothing looks wrong — the map is the right size, the tables are the
right size — the robot simply drives to the wrong side of the room. So the tests do
not check the image; they ask what the nav stack asks, *"what is at this world
coordinate?"*, and one of them checks that the un-flipped lookup finds **nothing**.

`map_goals.yaml` carries the named service stations already in the map frame, so nothing
downstream needs to know any of the above. Stations the planner knows are unreachable are
left out rather than exported as goals that cannot be met.

### For anything else: the JSON

`schema: "banquet-static-map/0.1"`.

```
room           name, width_m, height_m, origin
obstacles      tables and solid fixtures: circle {centre_m, radius_m}
               or rect {centre_m, size_m, rotation_deg}
zones          crossable but keep out during service (the dance floor)
access_points  service door, dish return — with `solid` saying which is furniture
stations       per cover: goal name, stand_m, heading_deg, reachable, blocked_by
totals         tables, stations, covers, unreachable
```

Coordinates are metres from the **north-west corner, x east, y south**. The plan draws
that origin so the convention is visible rather than buried in a README.

---

## Open questions

1. **The export format**, with whoever owns the nav stack. Occupancy grid instead of
   polygons? A ROS message rather than JSON? Everything below depends on this answer.
2. **A physical origin reference.** The map's origin is a room corner, which the robot
   cannot see. Something it can localise against — a fiducial at a known spot, or a
   surveyed dock — has to tie this frame to the robot's.
3. **Whether `STAND_OFF = 0.45 m` is right**, which depends on the arms' actual working
   reach — measurable from the plate pipeline.
4. **Where the camera goes**, and whether one covers a 20 × 21 m ballroom at all.

---

## Status

Layout, seating guests, reachability, printing, save/export, the live camera path and the
hand-over to `robot_link.py` all work and have been exercised in a browser, the camera
against the simulated hall. Loading a lidar `.pgm` as a backdrop and tracing it into walls
has been taken out of the page; `map_to_plan.py` still does the tracing as a script.

```bash
python3 check_geom.py       # 34 checks, no browser needed
```

`check_geom.py` transcribes the three pieces of geometry that are easy to get quietly
wrong — the staggered arrangement, the view fit, and the camera homography — and asserts
what they are *supposed* to do rather than what they currently compute. It is a Python
port of JavaScript, so it can drift; it is worth keeping anyway, because it caught the
stagger laying rows a full pitch apart instead of half, which had quietly closed every
aisle to 1.07 m while the app reported 1.50.

The app has never been fed real camera data, and the export has never been read by the
nav stack.
