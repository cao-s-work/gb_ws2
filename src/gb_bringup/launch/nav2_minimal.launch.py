#!/usr/bin/env python3
"""
gb_bringup / launch / nav2_minimal.launch.py

Minimal Nav2 bringup for gangbeng robot.
Launches only the nodes needed for single-goal navigation (no waypoint_follower).
Custom lifecycle_manager with autostart=true for reliable bringup.

Usage:
    ros2 launch gb_bringup nav2_minimal.launch.py params_file:=<path>
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level', default='info')

    # Default params file
    default_params = os.path.join(
        get_package_share_directory('gb_bringup'),
        'config', 'nav2_params.yaml')

    # Lifecycle node list: no waypoint_follower, no AMCL
    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    # Remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # Launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 params YAML file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')

    # Nav2 nodes (non-composed)
    # respawn=True: 节点崩溃后自动重启 (Phase 8.5 前置修复 — controller_server 偶发静默崩溃)
    load_nodes = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_controller')],
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            respawn=False,
            respawn_delay=2.0,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings +
                    [('cmd_vel', 'cmd_vel_controller'), ('cmd_vel_smoothed', 'cmd_vel_nav')],
            respawn=False,
            respawn_delay=2.0,
        ),
        # Lifecycle manager (custom node_names, no waypoint_follower)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
                {'bond_timeout': 10.0},
                {'attempts': 5},
                {'retry_interval': 3.0},
            ],
        ),
    ])

    return LaunchDescription([
        LogInfo(msg='=== Starting gb_bringup minimal Nav2 stack (no waypoint_follower) ==='),

        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,

        load_nodes,
    ])
