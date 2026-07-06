import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    use_gpu = LaunchConfiguration('use_gpu', default='false')

    # ============================================================
    # 1. gb_description: URDF + robot_state_publisher
    # ============================================================
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_description'),
                         'launch', 'description.launch.py')
        ),
        launch_arguments={'publish_frequency': '50.0'}.items()
    )

    # ============================================================
    # 2. FAST-LIO2: LiDAR 驱动 + LiDAR-IMU 里程计
    # ============================================================
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_lio'),
                         'launch', 'fastlio.launch.py')
        ),
        launch_arguments={'use_gpu': use_gpu}.items()
    )

    # ============================================================
    # 3. 静态 TF
    # ============================================================
    # lidar_link -> livox_frame (恒等)
    static_tf_livox = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='livox_to_lidarlink',
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'livox_frame']
    )

    # map -> camera_init (恒等, FAST-LIO 发布 camera_init→base_link)
    static_tf_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_camera_init',
        arguments=['0', '0', '0', '0', '0', '0',
                   'map', 'camera_init']
    )

    # ============================================================
    # 4. 感知滤波：FAST-LIO /cloud_body → /points_nav (Nav2 输入)
    # ============================================================
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_perception'),
                         'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'input_topic': '/cloud_body',
            'output_topic': '/points_nav',
        }.items()
    )

    # ============================================================
    # 5. Nav2 导航栈 (最小配置，不含 waypoint_follower)
    # ============================================================
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_bringup'),
                         'launch', 'nav2_minimal.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items()
    )

    # ============================================================
    # 6. collision_monitor 碰撞检测 (可选，由 enable_collision_monitor 控制)
    #    输入: /cmd_vel_nav (velocity_smoother 输出) + /points_nav
    #    输出: /cmd_vel_collision → safety_node
    # ============================================================
    enable_collision_monitor = LaunchConfiguration('enable_collision_monitor', default='false')
    safety_input_topic = PythonExpression([
        '"/cmd_vel_collision" if "', enable_collision_monitor, '" == "true" else "/cmd_vel_nav"'
    ])

    collision_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_nav2'),
                         'launch', 'collision_monitor.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': os.path.join(
                get_package_share_directory('gb_nav2'),
                'config', 'collision_monitor.yaml'),
        }.items(),
        condition=IfCondition(PythonExpression(['"', enable_collision_monitor, '" == "true"']))
    )

    # ============================================================
    # 7. gb_safety 安全闸门 (可选，由 enable_safety 控制)
    # ============================================================
    safety_output_topic = LaunchConfiguration('safety_output_topic', default='/cmd_vel_safety')
    enable_safety = LaunchConfiguration('enable_safety', default='false')
    connect_base_arg = LaunchConfiguration('connect_base', default='false')
    use_mock_base_arg = LaunchConfiguration('use_mock_base', default='true')
    allow_real_base_arg = LaunchConfiguration('allow_real_base', default='false')
    base_driver_params_file_arg = LaunchConfiguration(
        'base_driver_params_file',
        default=os.path.join(get_package_share_directory('gb_base_driver'),
                             'config', 'base_driver.yaml'))


    # ⛔ 负向保护：禁止 connect_base=true + use_mock_base=false
    # 不使用 LaunchConfiguration.compare 避免复杂度，在节点参数中做逻辑判断
    _connect_base = connect_base_arg
    _use_mock_base = use_mock_base_arg

    safety_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_safety'),
                         'launch', 'safety.launch.py')
        ),
        launch_arguments={
            'input_cmd_topic': safety_input_topic,
            'output_cmd_topic': safety_output_topic,
            'require_odom': LaunchConfiguration('require_odom', default='true'),
            'require_points': LaunchConfiguration('require_points', default='true'),
            'require_battery': 'false',
            'publish_base_cmd': _connect_base,
            'base_cmd_topic': '/cmd_vel_base',
            'allow_real_base': allow_real_base_arg,
            'use_mock_base': _use_mock_base,
        }.items()
    )

    # ⛔ 负向保护日志 — 在 safety_node 内部做最终裁决
    base_protection_log = LogInfo(
        condition=None,
        msg=(
            f'🔒 安全保护: connect_base={_connect_base} + use_mock_base={_use_mock_base} | '
            f'如果 use_mock_base=false 且 connect_base=true, safety_node 会强制禁用 base_cmd'
        )
    )

    # ============================================================
    # 7b. gb_base_driver mock (仅 use_mock_base=true + connect_base=true)
    # ============================================================
    base_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gb_base_driver'),
                         'launch', 'base_driver.launch.py')
        ),
        launch_arguments={
            'config_file': base_driver_params_file_arg,
        }.items(),
        condition=IfCondition(
            PythonExpression([
                '"', _connect_base, '" == "true" and "',
                _use_mock_base, '" == "true"',
            ])
        ),
    )

    # ============================================================
    # 7c. 日志打印参数摘要
    # ============================================================
    param_summary = LogInfo(
        msg=(
            f'enable_safety={LaunchConfiguration("enable_safety", default="false")} | '
            f'connect_base={LaunchConfiguration("connect_base", default="false")} | '
            f'safety_input={LaunchConfiguration("safety_input_topic", default="/cmd_vel_nav")} | '
            f'safety_output={LaunchConfiguration("safety_output_topic", default="/cmd_vel_safety")}'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_gpu', default_value='false'),
        DeclareLaunchArgument('enable_safety', default_value='false'),
        DeclareLaunchArgument('enable_collision_monitor', default_value='false'),
        DeclareLaunchArgument('connect_base', default_value='false'),
        DeclareLaunchArgument('use_mock_base', default_value='true'),
        DeclareLaunchArgument('allow_real_base', default_value='false'),
        DeclareLaunchArgument('base_driver_params_file',
            default_value=os.path.join(
                get_package_share_directory('gb_base_driver'),
                'config', 'base_driver.yaml')),
        DeclareLaunchArgument('safety_input_topic', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('safety_output_topic', default_value='/cmd_vel_safety'),
        DeclareLaunchArgument('require_odom', default_value='true'),
        DeclareLaunchArgument('require_points', default_value='true'),
        LogInfo(msg='=== Starting gangbeng full navigation stack ==='),

        description_launch,
        fastlio_launch,
        static_tf_livox,
        static_tf_map,
        perception_launch,
        nav2_launch,

        # 碰撞检测 (条件加载由 enable_collision_monitor 控制)
        collision_monitor_launch,

        # 安全闸门 (条件加载由 enable_safety 控制)
        safety_launch,
        base_protection_log,

        # gb_base_driver mock (仅 use_mock_base=true + connect_base=true)
        base_driver_launch,

        param_summary,
    ])
