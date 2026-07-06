import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # MID-360 LiDAR driver
    livox_config_path = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config',
        'MID360_config.json'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': livox_config_path,
        }]
    )

    # TF for MID-360: lidar_link -> livox_frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_lidar_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'livox_frame', 'lidar_link']
    )

    return LaunchDescription([
        livox_driver,
        static_tf,
    ])
