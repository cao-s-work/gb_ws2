#!/usr/bin/env python3
"""
gb_perception / launch / perception.launch.py

Launch the points filter node that converts FAST-LIO output to Nav2-ready /points_nav.

Usage:
    ros2 launch gb_perception perception.launch.py
    ros2 launch gb_perception perception.launch.py input_topic:=/cloud_registered
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('gb_perception')
    default_config = os.path.join(pkg_share, 'config', 'points_filter.yaml')

    # Launch arguments
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/cloud_body',
        description='Input point cloud topic (e.g. /cloud_body, /cloud_registered)'
    )
    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/points_nav',
        description='Output filtered point cloud topic for Nav2'
    )
    declare_config = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to points_filter.yaml config file'
    )

    # Points filter node
    points_filter_node = Node(
        package='gb_perception',
        executable='points_filter_node',
        name='points_filter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
            }
        ],
        # Remap so named topics in YAML override correctly
        remappings=[],
    )

    return LaunchDescription([
        declare_input_topic,
        declare_output_topic,
        declare_config,
        points_filter_node,
    ])
