# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Phases 4-6 -- full Nav2 stack (wraps nav2_bringup/bringup_launch.py).

Two modes:

  1. Localization on the saved map (default, the normal mission mode):
         ros2 launch delivery_bot_navigation nav.launch.py
     -> map_server + AMCL provide map->odom. Requires maps/warehouse.yaml
        (Phase 3). AMCL auto-initializes at (0,0,0) per nav2_params.yaml.

  2. SLAM mode (navigate while mapping, no saved map needed):
         ros2 launch delivery_bot_navigation nav.launch.py slam:=True
     -> slam_toolbox provides map->odom instead of AMCL.
        If you use this with the delivery manager, set its `localizer`
        parameter to `slam_toolbox` (see delivery_bot_bringup/config/mission.yaml).

Everything (controllers, costmaps incl. the social layer, planner, behaviors,
velocity smoother) comes from config/nav2_params.yaml.

Quick gate test (Phase 4): with sim + this launch running, open RViz with the
project config and send a "2D Goal Pose" -- the robot should reach it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_nav = get_package_share_directory('delivery_bot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_nav, 'maps', 'warehouse.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Map YAML from Phase 3 (only used when slam:=False)'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='True: slam_toolbox provides map->odom. '
                        'False: AMCL + saved map.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'params_file': params_file,
                'slam': slam,
                'use_sim_time': 'true',
            }.items()),
    ])
