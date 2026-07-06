import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')

    # Default params file path
    params_file_launch = os.path.join(
        get_package_share_directory('gb_bringup'),
        'config', 'nav2_params.yaml')

    # ============================================================
    # controller_server 已通过 nav2_bringup 的 -r cmd_vel:=cmd_vel_nav
    # 将输出重定向到 /cmd_vel_nav，无需额外 topic_tools relay
    # ============================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file_launch,
            'use_lifecycle_mgr': 'true',
            'map_subscribe_transient_local': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params_file_launch),

        LogInfo(msg='=== Starting Nav2 for gangbeng robot ==='),

        navigation_launch,
    ])
