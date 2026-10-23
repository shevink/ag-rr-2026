"""RR (Rechteckroboter) task -- easy map, no obstacles.

rectangle_robot on the easy map (8×8 m, straight lanes). Manual + autonomous control
via twist_mux. No dynamic obstacles.
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
                'map': 'easy_map',
                'width': '8',
                'height': '8',
            }.items(),
        )
    ])
