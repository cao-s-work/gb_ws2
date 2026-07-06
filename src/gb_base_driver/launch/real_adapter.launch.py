"""钢镚真实底盘 adapter launch

Phase 1 (只读):
  ros2 launch gb_base_driver real_adapter.launch.py read_only:=true

Phase 2+ (架空/落地):
  ros2 launch gb_base_driver real_adapter.launch.py read_only:=false max_linear_speed:=0.05 max_angular_speed:=0.15
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('read_only', default_value='true'),
        DeclareLaunchArgument('sdk_local_ip', default_value='192.168.168.216'),
        DeclareLaunchArgument('sdk_local_port', default_value='43988'),
        DeclareLaunchArgument('sdk_dog_ip', default_value='192.168.168.168'),
        DeclareLaunchArgument('max_linear_speed', default_value='0.60'),
        DeclareLaunchArgument('max_angular_speed', default_value='0.80'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),

        Node(
            package='gb_base_driver',
            executable='real_base_adapter',
            name='gb_base_driver_node',  # 同名替换 mock
            output='screen',
            parameters=[{
                'read_only': LaunchConfiguration('read_only'),
                'sdk_local_ip': LaunchConfiguration('sdk_local_ip'),
                'sdk_local_port': LaunchConfiguration('sdk_local_port'),
                'sdk_dog_ip': LaunchConfiguration('sdk_dog_ip'),
                'max_linear_speed': LaunchConfiguration('max_linear_speed'),
                'max_angular_speed': LaunchConfiguration('max_angular_speed'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
        ),
    ])
