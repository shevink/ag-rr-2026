"""RR (Rechteckroboter) task -- maze map, no obstacles.

rectangle_robot on the maze map (8×8 m, procedurally generated DFS labyrinth with
start/end markers). Manual + autonomous control via twist_mux. The maze itself
is the challenge; no additional dynamic obstacles.
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
                'map': 'maze_map',
                'width': '8',
                'height': '8',
            }.items(),
        )
    ])
