import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port = LaunchConfiguration('port', default='8080')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080',
                              description='Web server port'),

        Node(
            package='gb_web',
            executable='gb_web_node',
            name='gb_web_node',
            output='screen',
            parameters=[{
                'port': port,
                'cmd_vel_web_topic': '/cmd_vel_web',
                'teleop_timeout': 0.5,
            }],
        ),
    ])
