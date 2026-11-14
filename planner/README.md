# Planner (Steps 1–2)

The VATEL Planner is the banquet team's planning app for an iPad or a computer. It shows the hall, finds the
tables from the hall camera and puts them on a 2-D map. The team plans the arrangement on it, and the app sends
the map with every set position to the robot.

**Open it:** [cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/planner](https://cyanhere.github.io/VATEL-Humanoid-Orchestration-for-Autonomous-Banquet-Setup/planner/)
(the page is [`docs/planner/index.html`](../docs/planner/index.html)).

This folder holds the scripts that run beside the app:

- `cctv_bridge.py`: the hall-camera bridge and table detection.
- `robot_link.py`: the robot's end of **Send plan** / **Start mission**.
- `export_ros_map.py`: the static map for navigation.
- The tests for each.

To try it with a simulated hall:

```bash
python cctv_bridge.py --demo   # then open http://localhost:8765/
python robot_link.py
```
