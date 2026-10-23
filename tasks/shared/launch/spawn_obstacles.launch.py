"""Spawn obstacles from a YAML config into an already-running Gazebo world.

Does NOT start Gazebo — include this from a launch file that already has Gazebo running.
The including launch file's SetParameter(use_sim_time=True) is inherited by everything
here; the per-obstacle robot_state_publisher nodes also set it explicitly.

Arguments
---------
obstacle_config
    Absolute path to a YAML file with 'dynamic' and/or 'static' sections.
    Defaults to the easy-map config in shared/config.
"""

import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEFAULT_CONFIG = '/opt/ros_ws/tasks/shared/config/obstacles_easy_map.yaml'


def _spawn_obstacles(context, *args, **kwargs):
    config_path = LaunchConfiguration('obstacle_config').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    with open(config_path, 'r') as f:
        config: dict = yaml.safe_load(f)

    dynamic_obstacles = config.get('dynamic', {})
    static_obstacles = config.get('static', {})
    all_obstacles = {**dynamic_obstacles, **static_obstacles}

    obstacle_urdf_path = str(
        Path(get_package_share_directory('obstacle')) / 'urdf' / 'obstacle.urdf.xacro'
    )

    actions = []

    # Bridge entries for dynamic obstacles (need cmd_vel ROS→GZ and pose GZ→ROS).
    bridge_entries = []
    for name in dynamic_obstacles:
        bridge_entries.append(
            {
                'ros_topic_name': f'/{name}/cmd_vel',
                'gz_topic_name': f'/{name}/cmd_vel',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
        )
    for name in all_obstacles:
        bridge_entries.append(
            {
                'ros_topic_name': f'/{name}/pose_ground_truth',
                'gz_topic_name': f'/model/{name}/pose',
                'ros_type_name': 'geometry_msgs/msg/PoseArray',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
        )

    bridge_config_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(bridge_entries, bridge_config_file)
    bridge_config_file.close()

    # Per-obstacle: publish robot_description then spawn into Gazebo.
    for name, obstacle_data in all_obstacles.items():
        is_static = name in static_obstacles
        robot_description = subprocess.run(
            ['xacro', obstacle_urdf_path, f'name:={name}', f'static:={str(is_static).lower()}'],
            capture_output=True, text=True, check=True,
        ).stdout
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=name,
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
                'frame_prefix': f'{name}/',
            }],
            remappings=[('joint_states', f'/{name}/joint_states')],
        ))
        spawn_x = str(obstacle_data['spawn_position']['x'])
        spawn_y = str(obstacle_data['spawn_position']['y'])
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', 'empty',
                '-topic', f'/{name}/robot_description',
                '-name', name,
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', '0.0',
            ],
        ))

    # Obstacle mover drives the dynamic obstacles along their configured paths.
    actions.append(Node(
        package='ag_labs_py',
        executable='obstacle_mover',
        parameters=[{'config_path': config_path}],
    ))

    actions.append(Node(
        package='ag_labs_py',
        executable='mock_obstacle_sensor',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 10.0,
            'detection_range': 2.0,
            'minimum_distance': 0.1,
            'field_of_view': 1.571,
            'obstacle_sizes': [0.2],
            'source_frame': f'{robot}/base_footprint',
            'auto_configure_obstacles': True,
            'config_path': config_path,
        }],
        remappings=[('/racer/pose_ground_truth', f'/{robot}/pose_ground_truth')],
    ))

    # Bridge for dynamic-obstacle cmd_vel and pose topics.
    if bridge_entries:
        actions.append(Node(
            package='ros_gz_bridge',
            executable='bridge_node',
            parameters=[{'config_file': bridge_config_file.name}],
        ))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacle_config',
            default_value=_DEFAULT_CONFIG,
            description='Absolute path to obstacle YAML config (dynamic + static sections).',
        ),
        DeclareLaunchArgument(
            'robot',
            description='Robot model name, e.g. disc_robot or rectangle_robot.',
        ),
        OpaqueFunction(function=_spawn_obstacles),
    ])
