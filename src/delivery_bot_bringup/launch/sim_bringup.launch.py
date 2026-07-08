# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
"""Top-level orchestration: sim + navigation + pedestrians + RViz.

One command to bring up the whole demo. Startup is staged with timers so each
layer has its prerequisites before it starts (Gazebo/bridge up before Nav2
tries to read /scan; Nav2 up before RViz subscribes to costmaps).

Common invocations
------------------
Mapping session (Phase 3) -- no Nav2, just sim + SLAM + teleop yourself:
    ros2 launch delivery_bot_bringup sim_bringup.launch.py mode:=slam

Full demo on the saved map (Phases 5-6):
    ros2 launch delivery_bot_bringup sim_bringup.launch.py mode:=nav

Full demo, navigate-while-mapping (no saved map):
    ros2 launch delivery_bot_bringup sim_bringup.launch.py mode:=nav slam:=True
    #   (then set delivery_manager localizer:=slam_toolbox)

Toggle pieces off if you want to run them by hand:
    ... pedestrians:=False rviz:=False

The delivery_manager and metrics_logger are intentionally NOT launched here --
run them by hand so you control the A/B toggle and CSV labelling per run.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('delivery_bot_gazebo')
    pkg_nav = get_package_share_directory('delivery_bot_navigation')
    pkg_ped = get_package_share_directory('pedestrian_sim')

    mode = LaunchConfiguration('mode')            # 'nav' | 'slam'
    slam = LaunchConfiguration('slam')            # only relevant when mode==nav
    pedestrians = LaunchConfiguration('pedestrians')
    rviz = LaunchConfiguration('rviz')

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('delivery_bot_navigation'), 'rviz', 'nav.rviz'])

    declares = [
        DeclareLaunchArgument(
            'mode', default_value='nav',
            description="'nav' = full Nav2 stack; 'slam' = mapping only "
                        '(sim + slam_toolbox, drive it yourself)'),
        DeclareLaunchArgument(
            'slam', default_value='False',
            description='When mode:=nav, run Nav2 with slam_toolbox instead '
                        'of AMCL+saved map'),
        DeclareLaunchArgument(
            'pedestrians', default_value='True',
            description='Start the pedestrian simulator'),
        DeclareLaunchArgument(
            'rviz', default_value='True',
            description='Start RViz with the project config'),
    ]

    # --- Simulation (always) -------------------------------------------------
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'sim.launch.py')))

    # --- Navigation layer ----------------------------------------------------
    # mode == 'nav'  -> full Nav2 (nav.launch.py, slam arg forwarded)
    # mode == 'slam' -> slam_toolbox only (mapping session)
    is_nav = PythonExpression(["'", mode, "' == 'nav'"])
    is_slam = PythonExpression(["'", mode, "' == 'slam'"])

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav.launch.py')),
        launch_arguments={'slam': slam}.items(),
        condition=IfCondition(is_nav))

    slam_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')),
        condition=IfCondition(is_slam))

    # Give Gazebo + the bridge a few seconds before the nav layer subscribes.
    nav_layer = TimerAction(period=5.0, actions=[nav, slam_only])

    # --- Pedestrians ---------------------------------------------------------
    peds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ped, 'launch', 'pedestrians.launch.py')),
        condition=IfCondition(pedestrians))
    peds_layer = TimerAction(period=6.0, actions=[peds])

    # --- RViz ----------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(rviz))
    rviz_layer = TimerAction(period=7.0, actions=[rviz_node])

    return LaunchDescription(
        declares + [sim, nav_layer, peds_layer, rviz_layer])
