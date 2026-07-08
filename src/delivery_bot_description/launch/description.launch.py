# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Publish the robot description (URDF from xacro) via robot_state_publisher.

Standalone URDF check (no Gazebo needed):
    ros2 launch delivery_bot_description description.launch.py use_sim_time:=false
    ros2 run joint_state_publisher_gui joint_state_publisher_gui   # spin wheels
    rviz2   # add RobotModel (topic /robot_description) + TF, fixed frame base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('delivery_bot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'delivery_bot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ParameterValue(value_type=str) stops launch from trying to YAML-parse
    # the URDF string -- required on Humble.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use Ignition /clock (bridged) instead of wall time'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
