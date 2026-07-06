import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('gb_safety'),
        'config'
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_cmd_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('output_cmd_topic', default_value='/cmd_vel_safety'),
        DeclareLaunchArgument('require_odom', default_value='true'),
        DeclareLaunchArgument('require_points', default_value='true'),
        DeclareLaunchArgument('require_battery', default_value='false'),
        DeclareLaunchArgument('publish_base_cmd', default_value='false'),
        DeclareLaunchArgument('base_cmd_topic', default_value='/cmd_vel_base'),
        DeclareLaunchArgument('allow_real_base', default_value='false'),
        DeclareLaunchArgument('use_mock_base', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(config_dir, 'safety.yaml')),

        Node(
            package='gb_safety',
            executable='safety_node',
            name='safety_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'input_cmd_topic': LaunchConfiguration('input_cmd_topic'),
                'output_cmd_topic': LaunchConfiguration('output_cmd_topic'),
                'require_odom': LaunchConfiguration('require_odom'),
                'require_points': LaunchConfiguration('require_points'),
                'require_battery': LaunchConfiguration('require_battery'),
                'publish_base_cmd': LaunchConfiguration('publish_base_cmd'),
                'base_cmd_topic': LaunchConfiguration('base_cmd_topic'),
                'allow_real_base': LaunchConfiguration('allow_real_base'),
                'use_mock_base': LaunchConfiguration('use_mock_base'),
            }],
            remappings=[
                ('cmd_vel_in', LaunchConfiguration('input_cmd_topic')),
                ('cmd_vel_safety', LaunchConfiguration('output_cmd_topic')),
            ]
        ),
    ])
