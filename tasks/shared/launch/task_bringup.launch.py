"""Shared parameterized bringup for all rr/sr task launch files.

Accepts robot, map, and obstacle arguments. All per-task launch files delegate
here and supply only the values that differ between them.

Launch arguments
----------------
robot           disc_robot | rectangle_robot
path_follower   naive_path_follow | rectangle_path_follow
spawn_x         float string, spawn x coordinate in Gazebo world (default '0.0')
spawn_y         float string, spawn y coordinate in Gazebo world (default '0.0')
spawn_z         float string, e.g. '0.0' or '0.1'
map             map name, e.g. 'easy_map'
width           map width in metres, e.g. '8'
height          map height in metres, e.g. '8'
obstacle_config absolute path to an obstacle YAML; '' (default) skips obstacle spawning
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def _maybe_spawn_obstacles(context, *args, **kwargs):
    obstacle_config = LaunchConfiguration('obstacle_config').perform(context)
    if not obstacle_config:
        return []
    robot = LaunchConfiguration('robot').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ag_shared'),
                    'launch', 'spawn_obstacles.launch.py',
                )
            ),
            launch_arguments={
                'obstacle_config': obstacle_config,
                'robot': robot,
            }.items(),
        )
    ]


def _launch_rviz(context, *args, **kwargs):
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    args = ['-d', rviz_config] if rviz_config else []
    return [
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=args,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    robot = LaunchConfiguration('robot')
    path_follower = LaunchConfiguration('path_follower')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    map_name = LaunchConfiguration('map')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')

    robot_share = FindPackageShare(robot)
    map_gazebo_share = FindPackageShare('ag_map_gazebo')

    urdf_path = PathJoinSubstitution([robot_share, 'urdf', [robot, '.urdf.xacro']])
    bridge_config = PathJoinSubstitution(
        [robot_share, 'launch', 'include', 'ros_gazebo_bridge.yaml']
    )
    empty_world = PathJoinSubstitution([map_gazebo_share, 'worlds', 'empty_no_ground.sdf'])
    twist_mux_config = '/opt/ros_ws/tasks/shared/config/twist_mux.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('robot'),
        DeclareLaunchArgument('path_follower'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('width'),
        DeclareLaunchArgument('height'),
        DeclareLaunchArgument('obstacle_config', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),

        SetParameter(name='use_sim_time', value=True),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'frame_prefix': [robot, '/'],
            }],
            remappings=[('joint_states', ['/', robot, '/joint_states'])],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
                )
            ),
            launch_arguments={'gz_args': ['-r ', empty_world]}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([map_gazebo_share, 'launch', 'ground_plane.launch.py'])
            ),
            launch_arguments={'map': map_name, 'width': width, 'height': height}.items(),
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'empty',
                '-topic', 'robot_description',
                '-name', robot,
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            parameters=[{'config_file': bridge_config}],
        ),

        OpaqueFunction(function=_launch_rviz),

        Node(
            package='ag_labs_py',
            executable='wasd_teleop_node',
        ),

        Node(
            package='ag_labs_py',
            executable=path_follower,
            remappings=[(['/', robot, '/cmd_vel'], '/cmd_vel/path')],
        ),

        Node(
            package='ag_labs_cpp',
            executable='zero_motor_node',
        ),

        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out', ['/', robot, '/cmd_vel'])],
        ),

        OpaqueFunction(function=_maybe_spawn_obstacles),
    ])
