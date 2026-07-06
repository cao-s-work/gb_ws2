import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'base_driver.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config_file),

        Node(
            package='gb_base_driver',
            executable='gb_base_driver_node',
            name='gb_base_driver_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
            emulate_tty=True,
        ),
    ])
