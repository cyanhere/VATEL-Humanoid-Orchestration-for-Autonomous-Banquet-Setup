"""Populate ~/.scuba/livemap: nav2 with NO map server at all."""
import io, os, re, shutil

L = '/home/unitree/.scuba/livemap'
G = '/home/unitree/.scuba/growmap'
os.makedirs(L + '/config', exist_ok=True)
os.makedirs(L + '/launch', exist_ok=True)
os.makedirs(L + '/maps/live', exist_ok=True)

shutil.copy2(G + '/accumulator.py', L + '/accumulator.py')
os.chmod(L + '/accumulator.py', 0o755)
print('copied accumulator.py')

# ---------------------------------------------------------------- nav2 config
s = io.open(G + '/config/nav2_growmap.yaml', encoding='utf-8').read()

# 1. drop the whole map_server block
s2 = re.sub(r'map_server:\n(?:  .*\n|\n)*?(?=\n[a-z_]+:)', '', s, count=1)
assert 'map_server:\n' not in s2, 'map_server block not removed'
s = s2

# 2. lifecycle must not wait for a node that no longer exists
OLD = 'node_names: ["map_server", "planner_server", "controller_server"]'
NEW = 'node_names: ["planner_server", "controller_server"]'
assert s.count(OLD) == 1
s = s.replace(OLD, NEW)

# 3. the global costmap has no static map to size itself from, and no
#    static_layer to read. The obstacle_layer IS the map now.
OLD = '''      global_frame: map
      robot_base_frame: body
      resolution: 0.05
      robot_radius: 0.28
      track_unknown_space: true
      transform_tolerance: 0.5'''
NEW = '''      global_frame: map
      robot_base_frame: body
      # Without a static map the costmap cannot inherit a size or an origin,
      # so both are explicit. 60 x 60 m centred on the start pose, which IS
      # the origin here. At 0.05 m that is 1.44 M cells and inflation over it
      # every update is slow, so the GLOBAL grid is 0.10 m - ample for a
      # 0.28 m robot radius. The local costmap stays at 0.05 m.
      resolution: 0.10
      width: 60
      height: 60
      origin_x: -30.0
      origin_y: -30.0
      rolling_window: false
      robot_radius: 0.28
      # Everything is unexplored at startup. With track_unknown_space true,
      # A* (allow_unknown:false) would refuse to plan anywhere at all. False
      # makes unseen cells FREE, so it plans straight lines into the unknown
      # and the obstacle_layer corrects them as the lidar sees the room.
      track_unknown_space: false
      transform_tolerance: 0.5'''
assert s.count(OLD) == 1, 'global costmap header anchor'
s = s.replace(OLD, NEW)

# 4. no static_layer, and no static_layer block
OLD = '      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]'
NEW = '      plugins: ["obstacle_layer", "inflation_layer"]'
assert s.count(OLD) == 1
s = s.replace(OLD, NEW)
s2 = re.sub(r'      static_layer:\n(?:        .*\n)*', '', s, count=1)
assert 'static_layer' not in s2, 'static_layer block not removed'
s = s2

hdr = ('# LIVE-MAP PIPELINE - see ~/.scuba/livemap/README.md\n'
       '# No map server and no static layer: the robot starts knowing nothing\n'
       '# and the global costmap IS the map it builds, marked by obstacle_layer\n'
       '# from the lidar. The global costmap does not roll, so those marks\n'
       '# persist and accumulate for the whole run.\n'
       '#\n'
       '# SAFETY: with no walls known in advance, A* will plan through space\n'
       '# the robot has never seen, and DWB only looks sim_time * max_vel_x =\n'
       '# 0.75 m ahead. Keep goals within line of sight.\n')
io.open(L + '/config/nav2_livemap.yaml', 'w', encoding='utf-8').write(hdr + s)
print('wrote config/nav2_livemap.yaml')

# ---------------------------------------------------------------- launch file
s = io.open(G + '/launch/nav_growmap.launch.py', encoding='utf-8').read()
s = s.replace("CFG = '/home/unitree/.scuba/growmap/config/nav2_growmap.yaml'",
              "CFG = '/home/unitree/.scuba/livemap/config/nav2_livemap.yaml'")
OLD = """        Node(package='nav2_map_server', executable='map_server',
             name='map_server', **common),
"""
assert s.count(OLD) == 1, 'map_server node anchor'
s = s.replace(OLD, """        # no map_server: there is no map to serve. The global costmap's
        # obstacle_layer is the map, built live from the lidar.
""")
s = s.replace('"""Nav2 with DWB on the G1 - GROW-MAP PIPELINE copy.',
              '"""Nav2 with DWB on the G1 - LIVE-MAP PIPELINE (no map server).', 1)
io.open(L + '/launch/nav_livemap.launch.py', 'w', encoding='utf-8').write(s)
print('wrote launch/nav_livemap.launch.py')
