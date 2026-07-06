import os
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_urdf(context):
    """通过 xacro 生成 URDF XML 字符串"""
    src_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'gb_dog.urdf.xacro'
    )
    share_path = os.path.join(
        get_package_share_directory('gb_description'),
        'urdf', 'gb_dog.urdf.xacro'
    )
    xacro_path = src_path if os.path.exists(src_path) else share_path

    try:
        result = subprocess.run(
            ['xacro', xacro_path],
            capture_output=True, text=True, check=True
        )
        return result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        LogInfo(msg=f'xacro failed: {e}').execute(context)
        return ''


def launch_setup(context):
    urdf_xml = generate_urdf(context)
    publish_frequency = LaunchConfiguration('publish_frequency').perform(context)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_xml,
            'publish_frequency': float(publish_frequency),
        }],
    )

    # NOTE: odom_lio -> base_link 由 FAST-LIO2 动态发布，不在此处发布静态 TF

    return [robot_state_publisher]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_frequency', default_value='50.0'),
        LogInfo(msg='Starting gb_description...'),
        OpaqueFunction(function=launch_setup),
    ])
