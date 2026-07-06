#!/usr/bin/env python3
"""
gb_nav2 / launch / localization.launch.py

AMCL 定位 + pointcloud_to_laserscan:
  /cloud_registered_body (PointCloud2, body帧) → /scan (LaserScan, base_link) → AMCL → map→camera_init

依赖:
  - FAST-LIO 运行中，发布 /cloud_registered_body
  - map_server 运行中，提供 /map

Usage:
    ros2 launch gb_nav2 localization.launch.py
    ros2 launch gb_nav2 localization.launch.py map_file:=/home/nvidia/gb_maps/20260703_gb_map/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default='')

    # AMCL 参数文件
    amcl_params = os.path.join(
        get_package_share_directory('gb_nav2'),
        'config', 'amcl_params.yaml')

    # 0. map_server: 提供 /map (PGM/YAML 格式已有地图)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    # 0.5 lifecycle_manager for map_server
    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # 1. pointcloud_to_laserscan: /cloud_registered_body → /scan
    pcl_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='cloud_to_scan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1,
        }],
        remappings=[
            ('cloud_in', '/cloud_body'),
            ('scan', '/scan'),
        ],
    )

    # 2. AMCL: /scan + /map → map→camera_init
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('scan', '/scan'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map_file',
            default_value='/home/nvidia/gb_maps/20260704_gb_pointfoot/map.yaml',
            description='Path to map.yaml for map_server'),
        LogInfo(msg='=== Starting AMCL localization stack ==='),
        map_server_node,
        map_lifecycle_node,
        pcl_to_scan,
        amcl_node,
    ])
