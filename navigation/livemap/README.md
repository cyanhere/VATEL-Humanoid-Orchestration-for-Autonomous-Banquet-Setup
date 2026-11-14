# livemap — the robot starts knowing nothing

No prior point cloud, no prior occupancy grid, no surveyed frame. Every run
begins empty and the robot builds both maps as it goes. Self-contained:
nothing here writes to `~/.scuba/nav`, `~/.scuba/reloc`, `~/.scuba/growmap`,
`~/.scuba/viz` or `~/.scuba/lio_ws`.

## Why there is nothing to seed

With no surveyed map there is no external frame to line up with, so **the start
pose IS the origin**. `seed_x`, `seed_y` and `seed_yaw` are 0 by definition —
nothing to measure, nothing to get wrong. That removes the failure that put the
first growmap run out: the map there had to match `ilounge_02`'s frame, and a
hand-typed `seed_z` was the one number nobody could verify.

The sensor height still matters, because the cloud must have the floor at z=0
(`z_band` and every consumer assume it). So it is **measured, not typed**:
gravity levelling already fixes which way is up, so the floor is the dominant
horizontal layer below the sensor. `auto_z` finds it in the first few scans.

How wrong the typed value was, from the growmap run: the floor landed at
**−0.15 m** instead of 0, i.e. the real sensor height was ~1.35 m against the
1.2 m entered. Re-exporting that same cloud with the correction recovers **32%
more free space**.

`auto_z` refuses rather than guesses — if no layer holds at least 4% of the
lowest band it warns and falls back to `seed_z`, so a bad estimate is loud
rather than silent.

## Both maps are built live

**3D** — `accumulator.py` writes `maps/live/live_cloud.npy`, and
`relocalizer_live.py` reloads it as it grows and registers against it. Same
accumulate → relocalize → accumulate cycle as growmap, same insertion gate.

**2D** — there is no map server at all. The global costmap's `obstacle_layer`
IS the map: the global costmap does not roll, so marks persist and accumulate
for the whole run. `cloud_to_grid.py` exports a real `.pgm` + `.yaml` from the
accumulated cloud whenever you want one.

Nav2 changes this required, versus the walls-only pipeline:

| | walls-only | livemap |
|---|---|---|
| `map_server` | runs | **removed**, also from `node_names` |
| global `plugins` | static + obstacle + inflation | **obstacle + inflation** |
| global size | inherited from the static map | **explicit** 60 × 60 m, origin (−30, −30) |
| global resolution | 0.05 m | **0.10 m** — 0.05 over 60 m is 1.44 M cells and inflation over it every update is slow |
| `track_unknown_space` | true | **false** |

`track_unknown_space: false` is the load-bearing one: everything is unexplored
at startup, so with it true A* (`allow_unknown: false`) would refuse to plan
anywhere at all. False makes unseen cells free, A* plans straight lines into
the unknown, and `obstacle_layer` corrects them as the lidar sees the room.

## ⚠ Safety

That is also the risk. **A\* will plan through walls the robot has never
seen**, and DWB only looks `sim_time × max_vel_x = 0.75 m` ahead — about 3
seconds at 0.25 m/s. The `allow_unknown: false` comment in the original config
("unknown may hide a stairwell") was written for exactly this situation, and
here that protection is gone by necessity.

**Keep goals within line of sight and work in short hops.** Do not send a far
target across unexplored space. No config can fix this; it is what starting
blind costs.

`min_obstacle_height: 0.20` still applies, so anything below 20 cm is invisible
to Nav2 — the deferred floor fix matters more here than anywhere else.

## Running it

Every round starts empty, so clear the previous cloud first:

    mv ~/.scuba/livemap/maps/live/live_cloud.npy{,.$(date +%H%M)} 2>/dev/null

**A — lidar + FAST-LIO** (shared, unchanged). Hold the robot **still** until
the first `/Odometry` — gravity is estimated at init, and `auto_z` needs those
first scans:

    ~/.scuba/lio_ws/run_lio.sh

**B — relocalizer. No arguments.** That is the point:

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/relocalizer_live.py

**C — accumulator:**

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/livemap/accumulator.py

**D — viewer.** `--mode map` and an EMPTY `--map`:

    source ~/.scuba/lio_ws/env.sh
    python3 ~/.scuba/viz/path_viewer.py --mode map --map ""

`--mode` picks the reference FRAME; `--map` picks the static FILE. They are
independent, and conflating them is how the map came out mirrored:

  * `--mode odom` draws in `camera_init`, which on this robot is UPSIDE DOWN.
    Projecting that to 2D mirrors the grid - the same effect the nav2 config
    warns about ("no costmap may use camera_init as its frame: its plane is
    upside down, mirrored grid, height band slicing the floor").
  * `--mode map` draws in `map`, which is gravity levelled, so it is correct.
    It also never downgrades to odom: if the relocalizer is not up yet the
    viewer simply shows nothing, instead of silently showing a mirror.
  * `--map ""` skips the static overlay, which is what you want with no
    surveyed map. Leaving it at the default would draw the ilounge walls over
    a completely different room.

The localisation itself is NOT mirrored - verified numerically: the seed
rotation has determinant +1, is orthonormal to 1e-16, maps gravity exactly
onto +z, and preserves handedness. Only the display frame was wrong.

**E — Nav2:**

    source ~/.scuba/lio_ws/env.sh
    ros2 launch ~/.scuba/livemap/launch/nav_livemap.launch.py

**F — bridge.** Omit the flag for a dry run:

    source ~/.scuba/sdk_teleop/env.sh
    ros2 run g1_teleop nav_bridge --ros-args -p enable:=true

**G — goal.** Line of sight only:

    source ~/.scuba/lio_ws/env.sh
    /usr/bin/python3 ~/.scuba/nav/send_goal.py X Y YAW_DEG

**H — keep the map it built, any time:**

    PYTHONNOUSERSITE=1 /usr/bin/python3 -s ~/.scuba/livemap/cloud_to_grid.py \
        --out ~/.scuba/livemap/maps/run_$(date +%m%d_%H%M)

## What you should see

1. **B** logs `auto_z: floor measured at -X.XX m below the sensor` — check that
   number against a tape measure once. It should be the lidar's real height.
2. **B** then warns `no map at .../live_cloud.npy yet -> holding the seed`.
3. **C** logs `snapshot 1: N points written` after ~20 s.
4. **B** logs `map loaded: N pts @ 0.15 m voxel` and starts printing
   `rmse … inliers …`. That is the cycle engaging.

## Watch for

* **A mirrored map.** The display frame is wrong, not the pose. Check the
  viewer logged `display frame: map (map)` and not `odom (camera_init)`.
* **`auto_z` warning instead of a measurement** — it found no dominant floor
  layer and fell back to `seed_z: 0.0`, which will put the map a metre out.
  Usually means the robot was started somewhere with no floor in view.
* **`skipped: gate N` climbing** in C — the pose is sitting in the ambiguous
  inlier band and little is being recorded.
* **`map->odom jumped`** warnings — each pauses insertion; frequent ones mean
  registration is unstable and the map should not be grown through them.
* **Drift is unbounded until step 4.** Keep the opening minute short.

## Files

| file | origin |
|---|---|
| `relocalizer_live.py` | built from `growmap/relocalizer_grow.py` by `build_reloc_live.py`; adds `auto_z`, zero seed |
| `accumulator.py` | copy of growmap's, output path changed |
| `cloud_to_grid.py` | new — cloud → `.pgm` + `.yaml` |
| `config/nav2_livemap.yaml` | from growmap's, map server and static layer removed |
| `launch/nav_livemap.launch.py` | from growmap's, `map_server` node removed |
| `maps/live/live_cloud.npy` | written by the accumulator |
