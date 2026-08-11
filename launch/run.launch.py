#!/usr/bin/env python3
"""Load pcd2pgm settings from a YAML params file.

All settings live in the YAML. Point params_file at your own copy to avoid
editing the installed one:

    ros2 launch pcd2pgm run.launch.py params_file:=/work/pcd2pgm.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('pcd2pgm'),
        'config',
        'pcd2pgm.yaml',
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the pcd2pgm YAML params file.',
    )

    node = Node(
        package='pcd2pgm',
        executable='pcd2pgm',
        # Must match the top-level key in the YAML.
        name='pcd2pgm',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, node])