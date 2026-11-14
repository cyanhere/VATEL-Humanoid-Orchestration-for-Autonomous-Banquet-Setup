"""Nav2 with DWB on the G1, onboard.

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

CFG = '/home/unitree/.scuba/nav/config/nav2_dwb.yaml'


def generate_launch_description():
    common = dict(output='screen', parameters=[CFG])
    return LaunchDescription([
        # body -> base_link is NOT published here any more. It used to be the
        # identity, i.e. "the lidar is level", which is wrong on this robot and
        # wrong again whenever the bracket is adjusted. run_lio.sh now measures
        # the mount from gravity at startup and publishes the levelled frame.
        ExecuteProcess(cmd=['/usr/bin/python3', '/home/unitree/.scuba/nav/odom_2d.py'],
                       output='screen'),
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', **common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', **common),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', **common),
    ])
