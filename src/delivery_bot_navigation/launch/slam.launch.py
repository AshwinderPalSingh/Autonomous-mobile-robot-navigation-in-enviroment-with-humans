# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Phase 3 -- build the map (slam_toolbox, async online mapping).

Workflow:
    # terminal 1: simulation
    ros2 launch delivery_bot_gazebo sim.launch.py

    # terminal 2: SLAM
    ros2 launch delivery_bot_navigation slam.launch.py

    # terminal 3: drive every aisle slowly, close the loop at least once
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    # terminal 4 (watch the map grow): rviz2, fixed frame `map`, add Map /map

    # when the map looks clean, save it INTO THE SOURCE TREE so it's versioned:
    ros2 run nav2_map_server map_saver_cli \
        -f ~/social_nav_ws/src/delivery_bot_navigation/maps/warehouse

    # then rebuild so the installed share/ copy exists for nav.launch.py:
    cd ~/social_nav_ws && colcon build --packages-select delivery_bot_navigation

Gate before Phase 4: maps/warehouse.yaml + warehouse.pgm exist, walls are
crisp (no double walls -- if you see them, drive slower / re-map).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('delivery_bot_navigation')
    default_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='slam_toolbox parameter file'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[params_file],
            output='screen'),
    ])
