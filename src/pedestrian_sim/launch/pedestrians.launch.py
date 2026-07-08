import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('pedestrian_sim')
    params_file = os.path.join(pkg, 'config', 'pedestrians.yaml')

    return LaunchDescription([
        Node(
            package='pedestrian_sim',
            executable='pedestrian_simulator',
            name='pedestrian_simulator',
            parameters=[
                params_file,
                {'use_sim_time': True},
            ],
            output='screen'),
    ])
