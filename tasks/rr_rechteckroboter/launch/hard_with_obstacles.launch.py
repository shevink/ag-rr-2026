"""RR (Rechteckroboter) task -- hard map with obstacles.

rectangle_robot on the hard map (10×10 m, roundabout and star intersection). Manual +
autonomous control via twist_mux. Dynamic obstacles from obstacles_hard_map.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ag_shared'),
                    'launch', 'task_bringup.launch.py',
                )
            ),
            launch_arguments={
                'robot': 'rectangle_robot',
                'path_follower': 'rectangle_path_follow',
                'spawn_z': '0.1',
                'map': 'hard_map',
                'width': '10',
                'height': '10',
                'obstacle_config': '/opt/ros_ws/tasks/shared/config/obstacles_hard_map.yaml',
            }.items(),
        )
    ])
