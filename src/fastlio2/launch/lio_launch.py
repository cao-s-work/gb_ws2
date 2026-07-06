import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # FAST-LIO2 node
    config_path = os.path.join(
        get_package_share_directory('fastlio2'),
        'config', 'mid360_fastlio.yaml'
    )

    fastlio_node = Node(
        package='fastlio2',
        executable='lio_node',
        name='lio_node',
        output='screen',
        parameters=[{'config_path': config_path}],
        remappings=[
            ('lio_odom', '/lio_odom'),
            ('body_cloud', '/cloud_body'),
            ('world_cloud', '/cloud_registered'),
            ('lio_path', '/lio_path'),
        ]
    )

    # Static TF: livox_frame == lidar_link (same physical point)
    static_tf_livox = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_to_lidarlink',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'livox_frame']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gpu', default_value='false',
                              description='Use GPU acceleration (unused in CPU version)'),
        fastlio_node,
        static_tf_livox,
    ])
