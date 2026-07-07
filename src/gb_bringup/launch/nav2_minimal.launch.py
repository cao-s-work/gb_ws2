#!/usr/bin/env python3
"""
gb_bringup / launch / nav2_minimal.launch.py

Minimal Nav2 bringup for ZSL-1W / GB robot.

设计原则：
1. 默认只启动 Nav2 节点，不启动 lifecycle_manager。
2. lifecycle 由 gb_full_chain.sh 手动 configure / activate。
3. 避免 lifecycle_manager 与启动脚本抢节点状态。
4. 保留 use_lifecycle_manager 参数，必要时仍可手动启用。
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile

from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='false')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level', default='info')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager', default='false')

    default_params = os.path.join(
        get_package_share_directory('gb_bringup'),
        'config',
        'nav2_params.yaml'
    )

    lifecycle_nodes = [
        'map_server',
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to Nav2 params YAML file',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='false',
        description='Autostart Nav2 lifecycle nodes. Usually false when gb_full_chain.sh manages lifecycle.',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level',
    )

    declare_use_lifecycle_manager_cmd = DeclareLaunchArgument(
        'use_lifecycle_manager',
        default_value='false',
        description='Whether to start Nav2 lifecycle_manager_navigation',
    )

    load_nodes = GroupAction([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [('cmd_vel', 'cmd_vel_controller')],
            respawn=True,
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
            respawn=True,
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
            respawn=True,
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
            respawn=True,
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
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_controller'),
                ('cmd_vel_smoothed', 'cmd_vel_nav'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            condition=IfCondition(use_lifecycle_manager),
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
        LogInfo(msg='=== Starting gb_bringup minimal Nav2 stack ==='),
        LogInfo(msg='=== lifecycle_manager disabled by default; gb_full_chain.sh should manage lifecycle ==='),

        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_sim_time_cmd,
        declare_log_level_cmd,
        declare_use_lifecycle_manager_cmd,

        load_nodes,
    ])
