import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # MID-360 LiDAR driver
    livox_config = os.path.join(
        get_package_share_directory('livox_ros_driver2'),
        'config', 'MID360_config.json'
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
            'user_config_path': livox_config,
        }]
    )

    # FAST-LIO2 node
    fastlio_config = os.path.join(
        get_package_share_directory('fastlio2'),
        'config', 'mid360_fastlio.yaml'
    )

    fastlio_node = Node(
        package='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': fastlio_config}],
        remappings=[
            ('lio_odom', '/Odometry'),
            ('body_cloud', '/cloud_body'),
            ('world_cloud', '/cloud_registered'),
            ('lio_path', '/lio_path'),
        ]
    )

    # NOTE: Static TF lidar_link -> livox_frame 移至 gb_bringup/navigation.launch.py
    # 此处只负责传感器驱动和 FAST-LIO 里程计

    return LaunchDescription([
        DeclareLaunchArgument('use_gpu', default_value='false'),
        livox_driver,
        fastlio_node,
    ])
