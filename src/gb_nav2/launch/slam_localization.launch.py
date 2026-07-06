#!/usr/bin/env python3
"""
gb_nav2 / launch / slam_localization.launch.py

slam_toolbox 定位模式:
  /cloud_registered_body → /scan → slam_toolbox(localization) → map→camera_init

与 AMCL 定位的区别:
  - 使用 scan-to-submap 匹配，类似 FAST-LIO 的 scan-to-map
  - 利用 slam_toolbox 建图时保存的序列化地图
  - 对非结构化环境更鲁棒

依赖:
  - FAST-LIO 运行中，发布 /cloud_registered_body
  - 预建序列化地图（需先跑一次 mapping 并 save_map）

Usage:
    ros2 launch gb_nav2 slam_localization.launch.py
    ros2 launch gb_nav2 slam_localization.launch.py map_pgm:=/home/nvidia/gb_maps/20260703_gb_map/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_pgm = LaunchConfiguration('map_pgm', default='')

    # slam_toolbox 定位参数
    slam_params = os.path.join(
        get_package_share_directory('gb_mapping'),
        'config', 'mapper_params_localization.yaml')

    # 0. map_server: 提供 /map (OccupancyGrid 用于可视化)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_pgm,
        }],
    )

    map_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam_localization',
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
            ('cloud', '/cloud_body'),
            ('scan', '/scan'),
        ],
    )

    # 2. slam_toolbox: localization 模式
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'map_pgm',
            default_value='/home/nvidia/gb_maps/20260703_gb_map/map.yaml',
            description='Path to map.yaml for visualization'),
        LogInfo(msg='=== Starting slam_toolbox localization stack ==='),
        map_server_node,
        map_lifecycle_node,
        pcl_to_scan,
        slam_node,
    ])
