"""Spawn a textured ground plane into an already-running Gazebo world.

Arguments
---------
map
    Name of the texture to use, without extension (e.g. ``easy_map``,
    ``medium_map``, ``hard_map``). The corresponding PNG must exist in
    ``ag_map_gazebo``'s installed ``textures/`` directory.
width
    Width of the visual plane in metres (x-axis). Defaults to ``8``.
height
    Height of the visual plane in metres (y-axis). Defaults to ``8``.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn_ground_plane(context, *args, **kwargs):
    map_name = LaunchConfiguration('map').perform(context)
    width = float(LaunchConfiguration('width').perform(context))
    height = float(LaunchConfiguration('height').perform(context))

    texture_path = (
        get_package_share_directory('ag_map_gazebo')
        + f'/textures/{map_name}.png'
    )

    sdf = (
        "<?xml version='1.0'?>"
        "<sdf version='1.7'>"
        "<model name='ground_plane'>"
        "  <static>true</static>"
        "  <link name='link'>"
        "    <collision name='collision'>"
        "      <geometry>"
        "        <plane><normal>0 0 1</normal></plane>"
        "      </geometry>"
        "    </collision>"
        "    <visual name='visual'>"
        f"      <pose>{width / 2.0} {height / 2.0} 0 0 0 0</pose>"
        "      <geometry>"
        "        <plane><normal>0 0 1</normal>"
        f"          <size>{width} {height}</size>"
        "        </plane>"
        "      </geometry>"
        "      <material>"
        "        <diffuse>1 1 1 1</diffuse>"
        "        <pbr><metal>"
        f"          <albedo_map>{texture_path}</albedo_map>"
        "          <metalness>0.0</metalness>"
        "          <roughness>1.0</roughness>"
        "        </metal></pbr>"
        "      </material>"
        "    </visual>"
        "  </link>"
        "</model>"
        "</sdf>"
    )

    return [Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'empty',
            '-string', sdf,
            '-name', 'ground_plane',
            '-allow_renaming', 'false',
        ],
    )]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='easy_map',
            description='Map texture name (without .png extension).',
        ),
        DeclareLaunchArgument(
            'width',
            default_value='8',
            description='Ground plane width in metres (x-axis).',
        ),
        DeclareLaunchArgument(
            'height',
            default_value='8',
            description='Ground plane height in metres (y-axis).',
        ),
        OpaqueFunction(function=_spawn_ground_plane),
    ])
