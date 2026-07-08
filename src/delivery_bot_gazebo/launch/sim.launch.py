# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Simulation bringup: Ignition Fortress + robot spawn + ROS<->GZ bridge.

Starts:
  1. ign gazebo (via ros_gz_sim) with the warehouse world, running (-r)
  2. robot_state_publisher (delivery_bot_description)
  3. spawn of the robot from the /robot_description topic
  4. ros_gz_bridge parameter_bridge for every topic the stack needs

Bridge direction syntax:  [ = GZ -> ROS,  ] = ROS -> GZ,  @ = bidirectional.
If a topic is silent later, a typo in one of these strings is the usual
culprit -- check with `ros2 topic hz /scan` and `ign topic -l`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = 'delivery_bot'
WORLD_NAME = 'warehouse'  # must match <world name=...> in the SDF


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('delivery_bot_gazebo')
    pkg_description = get_package_share_directory('delivery_bot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_gazebo, 'worlds', 'warehouse.sdf')
    world = LaunchConfiguration('world')

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Full path to the SDF world file')

    # --- 1. Ignition Fortress ------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items())

    # --- 2. Robot description ------------------------------------------------
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # --- 3. Spawn the robot at the world origin ------------------------------
    # odom frame origin == world origin, so pedestrian/world coordinates match.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', ROBOT_NAME,
            '-x', '0.0', '-y', '0.0', '-z', '0.20',
        ],
        output='screen')

    # --- 4. ROS <-> Ignition bridge ------------------------------------------
    joint_state_gz_topic = (
        f'/world/{WORLD_NAME}/model/{ROBOT_NAME}/joint_state')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Sim clock -> ROS (drives use_sim_time everywhere)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Nav2 velocity command -> DiffDrive
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # DiffDrive odometry -> ROS (used by DWB / velocity feedback)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # DiffDrive odom->base_footprint transform -> /tf
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # Sensors
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            # Wheel joint states (default scoped GZ topic, remapped below)
            joint_state_gz_topic +
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            (joint_state_gz_topic, '/joint_states'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    return LaunchDescription([
        declare_world,
        gz_sim,
        description,
        spawn,
        bridge,
    ])
