# ============================================================
# LIO 2D Odometry Projector 启动文件
# 用法: ros2 launch gb_lio odom_2d.launch.py
# ============================================================
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gb_lio',
            executable='lio_odom_2d_projector',
            name='lio_odom_2d_projector',
            output='screen',
            parameters=[{
                'input_odom_topic': '/Odometry',
                'output_odom_topic': '/Odometry_2d',
                'odom_frame': 'camera_init',
                'base_frame': 'base_link',
            }],
        ),
    ])
