#!/usr/bin/env python3
"""
gb_perception / launch / test_obstacle_cloud.launch.py

Launch the test obstacle cloud node for collision_monitor polygon testing.

Usage:
    ros2 launch gb_perception test_obstacle_cloud.launch.py mode:=stop
    ros2 launch gb_perception test_obstacle_cloud.launch.py mode:=slow
    ros2 launch gb_perception test_obstacle_cloud.launch.py mode:=none

Modes:
    none  - Empty point cloud (no obstacles)
    stop  - Points in PolygonStop zone (x≈0.60m)
    slow  - Points in PolygonSlow zone (x≈1.10m)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    output_topic = LaunchConfiguration("output_topic")
    frame_id = LaunchConfiguration("frame_id")
    publish_rate = LaunchConfiguration("publish_rate")
    num_points = LaunchConfiguration("num_points")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="none"),
        DeclareLaunchArgument("output_topic", default_value="/points_nav_test"),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("publish_rate", default_value="10.0"),
        DeclareLaunchArgument("num_points", default_value="20"),

        Node(
            package="gb_perception",
            executable="test_obstacle_cloud_node",
            name="test_obstacle_cloud_node",
            output="screen",
            parameters=[{
                "mode": mode,
                "output_topic": output_topic,
                "frame_id": frame_id,
                "publish_rate": publish_rate,
                "num_points": num_points,
                "use_sim_time": False,
            }],
        ),
    ])
