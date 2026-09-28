"""
Launch the milestone1 wall-following stack.

Starts dist_finder, pid and safety_node, all loaded with the global parameter
file config/params.yaml. The simulator is launched separately (see the
Milestone 1 part 1 instructions).

Usage:
    ros2 launch milestone1 milestone1_py.py
    ros2 launch milestone1 milestone1_py.py params_file:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Build the launch description for the three milestone1 nodes.

    Returns:
        launch.LaunchDescription: the nodes and the params_file argument.
    """
    default_params = os.path.join(
        get_package_share_directory('milestone1'), 'config', 'params.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Global parameter file for all milestone1 nodes'),

        Node(
            package='milestone1',
            executable='dist_finder',
            name='dist_finder',
            output='screen',
            parameters=[params_file]),

        Node(
            package='milestone1',
            executable='pid',
            name='pid',
            output='screen',
            parameters=[params_file]),

        Node(
            package='milestone1',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[params_file]),
    ])
