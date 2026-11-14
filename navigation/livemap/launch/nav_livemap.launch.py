"""Nav2 with DWB on the G1 - LIVE-MAP PIPELINE (no map server).

Same nodes as ~/.scuba/nav/launch/nav_dwb.launch.py, pointed at
growmap's config. odom_2d.py is shared and read only.

    ros2 launch ~/.scuba/nav/launch/nav_dwb.launch.py

Assumes ALREADY RUNNING (each in its own terminal):
  1. ~/.scuba/lio_ws/run_lio.sh          -> /Odometry, TF camera_init->body
  2. ~/.scuba/reloc/relocalizer.py       -> TF map->camera_init
  3. ros2 run g1_teleop nav_bridge       -> /cmd_vel to the robot

Brings up map_server + planner_server + controller_server(DWB) + lifecycle,
plus odom_2d.py - DWB's velocity feedback (FAST-LIO's /Odometry has no twist).
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

CFG = '/home/unitree/.scuba/livemap/config/nav2_livemap.yaml'


def generate_launch_description():
    common = dict(output='screen', parameters=[CFG])
    return LaunchDescription([
        # body -> base_link is NOT published here any more. It used to be the
        # identity, i.e. "the lidar is level", which is wrong on this robot and
        # wrong again whenever the bracket is adjusted. run_lio.sh now measures
        # the mount from gravity at startup and publishes the levelled frame.
        # odom_2d.py is NOT started here any more - it runs in its own
        # terminal, started BEFORE this launch. Two reasons:
        #   * DWB wants velocity feedback available before controller_server
        #     comes up, and a node started inside this launch races it
        #   * when it ran here AND in a terminal there were two publishers on
        #     /odom_2d, so DWB's sampling-window centre alternated between two
        #     values measured at different instants
        # Start it with:
        #   source ~/.scuba/lio_ws/env.sh && /usr/bin/python3 ~/.scuba/nav/odom_2d.py
        # no map_server: there is no map to serve. The global costmap's
        # obstacle_layer is the map, built live from the lidar.
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', **common),
    ])
