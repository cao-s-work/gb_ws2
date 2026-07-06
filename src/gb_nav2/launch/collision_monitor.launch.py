from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")

    default_params = PathJoinSubstitution([
        FindPackageShare("gb_nav2"),
        "config",
        "collision_monitor.yaml",
    ])

    collision_monitor_node = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["collision_monitor"],
            "bond_timeout": 10.0,
            "attempts": 5,
            "retry_interval": 3.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Full path to collision monitor parameter file",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically startup lifecycle nodes",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock",
        ),
        collision_monitor_node,
        lifecycle_manager_node,
    ])
