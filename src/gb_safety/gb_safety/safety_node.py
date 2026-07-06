#!/usr/bin/env python3
"""
gb_safety — 钢镚机器人软件安全闸门

输入:  /cmd_vel_nav (或其他速度 Topic)
输出:  /cmd_vel_safety (安全裁剪后)
       /safety_state (安全状态)
       /diagnostics (诊断信息)

安全逻辑:
  1. ESTOP latch — 一旦触发必须 reset_estop 才能恢复
  2. CMD_TIMEOUT — 输入速度超时归零
  3. ODOM_TIMEOUT / POINTS_TIMEOUT / BATTERY_LOW (可选)
  4. 速度 / 加速度限幅
  5. 所有异常默认 fail-safe: 输出 0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String, Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger, SetBool
import time


class SafetyState:
    OK = "OK"
    DISABLED = "DISABLED"
    ESTOP = "ESTOP"
    CMD_TIMEOUT = "CMD_TIMEOUT"
    ODOM_TIMEOUT = "ODOM_TIMEOUT"
    POINTS_TIMEOUT = "POINTS_TIMEOUT"
    BATTERY_LOW = "BATTERY_LOW"
    LIMITING = "LIMITING"
    ERROR = "ERROR"
    STARTING = "STARTING"
    WAITING_ODOM = "WAITING_ODOM"
    WAITING_POINTS = "WAITING_POINTS"


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # ---- 参数 ----
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_cmd_topic', '/cmd_vel_nav'),
                ('web_cmd_topic', '/cmd_vel_web'),
                ('output_cmd_topic', '/cmd_vel_safety'),
                ('estop_topic', '/emergency_stop'),
                ('odom_topic', '/Odometry'),
                ('points_topic', '/points_nav'),
                ('battery_topic', '/battery_state'),
                ('safety_state_topic', '/safety_state'),
                ('publish_rate', 20.0),
                ('max_linear_x', 0.80),
                ('min_linear_x', -0.15),
                ('max_angular_z', 1.00),
                ('max_linear_accel', 0.30),
                ('max_angular_accel', 0.60),
                ('cmd_timeout_sec', 0.50),
                ('odom_timeout_sec', 0.70),
                ('points_timeout_sec', 1.00),
                ('battery_timeout_sec', 5.00),
                ('require_odom', False),
                ('require_points', False),
                ('require_battery', False),
                ('battery_low_threshold', 0.20),
                ('estop_latched', True),
                ('zero_on_disable', True),
                ('publish_base_cmd', False),
                ('base_cmd_topic', '/cmd_vel_base'),
                ('allow_real_base', False),
                ('use_mock_base', True),
                ('startup_grace_sec', 10.0),
            ]
        )

        self._input_cmd_topic = self.get_parameter('input_cmd_topic').value
        self._web_cmd_topic = self.get_parameter('web_cmd_topic').value
        self._output_cmd_topic = self.get_parameter('output_cmd_topic').value
        self._estop_topic = self.get_parameter('estop_topic').value
        self._odom_topic = self.get_parameter('odom_topic').value
        self._points_topic = self.get_parameter('points_topic').value
        self._battery_topic = self.get_parameter('battery_topic').value
        self._safety_state_topic = self.get_parameter('safety_state_topic').value
        self._rate = self.get_parameter('publish_rate').value

        self._max_vx = self.get_parameter('max_linear_x').value
        self._min_vx = self.get_parameter('min_linear_x').value
        self._max_wz = self.get_parameter('max_angular_z').value
        self._max_ax = self.get_parameter('max_linear_accel').value
        self._max_aw = self.get_parameter('max_angular_accel').value

        self._cmd_tout = self.get_parameter('cmd_timeout_sec').value
        self._odom_tout = self.get_parameter('odom_timeout_sec').value
        self._points_tout = self.get_parameter('points_timeout_sec').value
        self._batt_tout = self.get_parameter('battery_timeout_sec').value

        self._require_odom = self.get_parameter('require_odom').value
        self._require_points = self.get_parameter('require_points').value
        self._require_battery = self.get_parameter('require_battery').value
        self._batt_threshold = self.get_parameter('battery_low_threshold').value

        self._estop_latched = self.get_parameter('estop_latched').value
        self._zero_on_disable = self.get_parameter('zero_on_disable').value
        self._publish_base_cmd = self.get_parameter('publish_base_cmd').value
        self._base_cmd_topic = self.get_parameter('base_cmd_topic').value
        self._allow_real_base = self.get_parameter('allow_real_base').value
        self._use_mock_base = self.get_parameter('use_mock_base').value
        self._startup_grace_sec = self.get_parameter('startup_grace_sec').value

        # ⛔ 负向保护: connect_base=true + use_mock_base=false → 强制断开
        if self._publish_base_cmd and not self._use_mock_base and not self._allow_real_base:
            self.get_logger().error(
                '⛔ connect_base:=true 仅在 use_mock_base:=true 时允许！'
                '当前 use_mock_base=false → 强制禁用 /cmd_vel_base 发布'
            )
            self._publish_base_cmd = False
            self._base_pub = None

        # ---- 内部状态 ----
        self._enabled = True
        self._estop_triggered = False          # latch flag
        self._estop_msg = False                # current message
        self._safety_state_str = SafetyState.OK

        # 上次接收时间戳
        self._last_cmd_time = 0.0
        self._last_odom_time = 0.0
        self._last_points_time = 0.0
        self._last_battery_time = 0.0

        # Phase 10.3: 分离 Nav2 输入和 Web 遥控输入，实现仲裁
        self._last_nav_cmd_time = 0.0    # /cmd_vel_collision (Nav2) 最后接收时间
        self._last_web_cmd_time = 0.0    # /cmd_vel_web (Web teleop) 最后接收时间
        self._nav_vx = 0.0
        self._nav_vy = 0.0
        self._nav_wz = 0.0
        self._web_vx = 0.0
        self._web_vy = 0.0
        self._web_wz = 0.0
        self._web_active = False          # Web 遥控是否活跃（有近期指令）

        # 启动时间（用于 startup grace period）
        self._startup_time = time.time()
        self._odom_received = False   # has received at least one odom msg
        self._points_received = False  # has received at least one points msg

        # 上次发布的速度（用于加速度限幅）
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

        # 当前输入速度 (仲裁后)
        self._input_vx = 0.0
        self._input_vy = 0.0
        self._input_wz = 0.0
        self._input_valid = False

        # 健康检查
        self._odom_healthy = not self._require_odom
        self._points_healthy = not self._require_points
        self._battery_healthy = not self._require_battery

        # ---- 订阅者 ----
        # 传感器数据使用 BEST_EFFORT 以匹配发布者 QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # 输入速度 (Nav2 /cmd_vel_collision, 使用 BEST_EFFORT QoS)
        self._cmd_sub = self.create_subscription(
            Twist, self._input_cmd_topic, self._nav_cmd_callback, sensor_qos
        )

        # Web 遥控输入 (独立 callback, Phase 10.3 仲裁, Phase 5.1: RELIABLE QoS)
        # 必须与 gb_web_node 发布侧 RELIABLE QoS 匹配，否则永不送达
        web_cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self._web_cmd_sub = self.create_subscription(
            Twist, self._web_cmd_topic, self._web_cmd_callback, web_cmd_qos
        )

        # 急停
        self._estop_sub = self.create_subscription(
            Bool, self._estop_topic, self._estop_callback, 10
        )

        # 里程计（可选）
        if self._require_odom:
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._odom_callback, sensor_qos
            )

        # 点云（可选）
        if self._require_points:
            self._points_sub = self.create_subscription(
                PointCloud2, self._points_topic, self._points_callback, sensor_qos
            )

        # 电池（可选）
        if self._require_battery:
            self._battery_sub = self.create_subscription(
                BatteryState, self._battery_topic, self._battery_callback, 10
            )

        # ---- 发布者 ----
        self._cmd_pub = self.create_publisher(Twist, self._output_cmd_topic, 10)
        self._base_pub = None
        if self._publish_base_cmd:
            self._base_pub = self.create_publisher(Twist, self._base_cmd_topic, 10)
            self.get_logger().info(f'  同时发布 base cmd 到 {self._base_cmd_topic}')
        self._state_pub = self.create_publisher(String, self._safety_state_topic, 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # ---- 服务 ----
        self._reset_srv = self.create_service(Trigger, '/gb_safety/reset_estop',
                                              self._reset_estop_callback)
        self._enable_srv = self.create_service(SetBool, '/gb_safety/set_enabled',
                                               self._set_enabled_callback)

        # ---- 定时器 ----
        period = 1.0 / self._rate
        self._timer = self.create_timer(period, self._safety_loop)

        self.get_logger().info(
            f'gb_safety 启动: in={self._input_cmd_topic} → out={self._output_cmd_topic}'
            f' | 速率={self._rate}Hz | max_vx={self._max_vx} | max_wz={self._max_wz}'
        )

    # ---- 回调函数 ----

    def _nav_cmd_callback(self, msg: Twist):
        """Nav2 速度输入回调 (/cmd_vel_collision)"""
        self._last_nav_cmd_time = time.time()
        self._nav_vx = msg.linear.x
        self._nav_wz = msg.angular.z

    def _web_cmd_callback(self, msg: Twist):
        """Web 遥控速度输入回调 (/cmd_vel_web)
        Phase 10.3: Web 遥控优先于 Nav2"""
        self._last_web_cmd_time = time.time()
        self._web_vx = msg.linear.x
        self._web_vy = msg.linear.y
        self._web_wz = msg.angular.z

    def _arbitrate(self, now: float):
        """Phase 10.3: 控制权仲裁
        优先级: Web teleop (近期有指令) > Nav2 > 无输入
        Web teleop 超时 (cmd_timeout_sec) 后自动降级到 Nav2"""
        web_age = now - self._last_web_cmd_time if self._last_web_cmd_time > 0 else 999.0
        nav_age = now - self._last_nav_cmd_time if self._last_nav_cmd_time > 0 else 999.0

        # Web 遥控活跃判定：有近期指令 (在 cmd_timeout 内)
        self._web_active = web_age < self._cmd_tout

        if self._web_active:
            # Web 遥控优先
            self._input_vx = self._web_vx
            self._input_vy = self._web_vy
            self._input_wz = self._web_wz
            self._input_valid = True
            self._last_cmd_time = self._last_web_cmd_time
        elif nav_age < self._cmd_tout:
            # Nav2 自动导航
            self._input_vx = self._nav_vx
            self._input_vy = self._nav_vy
            self._input_wz = self._nav_wz
            self._input_valid = True
            self._last_cmd_time = self._last_nav_cmd_time
        else:
            # 两个源都超时
            self._input_valid = False

    def _estop_callback(self, msg: Bool):
        self._estop_msg = msg.data
        if msg.data:
            if self._estop_latched:
                self._estop_triggered = True
                self.get_logger().warn('🔴 急停触发 (latch) — 需要 reset_estop 才能恢复')
            else:
                self._estop_triggered = True
                self.get_logger().warn('🔴 急停触发')
        else:
            if not self._estop_latched:
                self._estop_triggered = False
                self.get_logger().info('🟢 急停解除 (非 latch 模式)')
            # latch 模式: 必须通过 reset_estop 服务解除

    def _odom_callback(self, msg: Odometry):
        self._last_odom_time = time.time()
        self._odom_healthy = True
        self._odom_received = True

    def _points_callback(self, msg: PointCloud2):
        self._last_points_time = time.time()
        self._points_healthy = True
        self._points_received = True

    def _battery_callback(self, msg: BatteryState):
        self._last_battery_time = time.time()
        if msg.percentage >= 0.0:  # valid percentage
            self._battery_healthy = msg.percentage > self._batt_threshold
        else:
            self._battery_healthy = True  # unknown percentage

    # ---- 服务 ----

    def _reset_estop_callback(self, request, response):
        if not self._enabled:
            response.success = False
            response.message = 'Safety node is disabled'
            return response

        if self._estop_msg:
            response.success = False
            response.message = 'ESTOP 信号仍为 true，无法复位'
            return response

        if self._estop_latched:
            self._estop_triggered = False
            # Phase 10.3: reset 后清除所有输入状态，不恢复旧速度
            self._nav_vx = 0.0
            self._nav_vy = 0.0
            self._nav_wz = 0.0
            self._web_vx = 0.0
            self._web_vy = 0.0
            self._web_wz = 0.0
            self._input_vx = 0.0
            self._input_vy = 0.0
            self._input_wz = 0.0
            self._input_valid = False
            self._last_vx = 0.0
            self._last_vy = 0.0
            self._last_wz = 0.0
            self._web_active = False
            msg = '🟢 ESTOP latch 已复位 (输入已清零，等待新指令)'
            self.get_logger().info(msg)
            response.success = True
            response.message = msg
        else:
            response.success = False
            response.message = '非 latch 模式，无需复位'
        return response

    def _set_enabled_callback(self, request, response):
        self._enabled = request.data
        if not self._enabled:
            self._safety_state_str = SafetyState.DISABLED
            self.get_logger().warn(f'⛔ Safety 节点 {"启用" if self._enabled else "禁用"}')
        else:
            self._safety_state_str = SafetyState.OK
            self.get_logger().info(f'✅ Safety 节点 {"启用" if self._enabled else "禁用"}')
        response.success = True
        response.message = f'safety_node enabled={self._enabled}'
        return response

    # ---- 主循环 ----

    def _safety_loop(self):
        now = time.time()
        reasons = []

        # 0. 检查是否禁用
        if not self._enabled:
            self._safety_state_str = SafetyState.DISABLED
            if self._zero_on_disable:
                self._publish_zero()
            else:
                self._publish_safe(self._input_vx, self._input_vy, self._input_wz, now)
            self._publish_state()
            self._publish_diagnostics(SafetyState.DISABLED, ['safety_node disabled'])
            return

        # 1. ESTOP 检查 (最高优先级)
        if self._estop_triggered:
            self._safety_state_str = SafetyState.ESTOP
            self._publish_zero()
            self._publish_state()
            self._publish_diagnostics(SafetyState.ESTOP, ['Emergency stop latched'])
            return

        # --- Startup Grace Period ---
        # 启动初期等待传感器数据稳定，不输出速度
        elapsed = now - self._startup_time
        in_grace = self._startup_grace_sec > 0.0 and elapsed < self._startup_grace_sec

        if in_grace:
            if self._require_odom and not self._odom_received:
                self._safety_state_str = SafetyState.WAITING_ODOM
                self._publish_zero()
                self._publish_state()
                self._publish_diagnostics(
                    SafetyState.WAITING_ODOM,
                    [f'Waiting for {self._odom_topic} '
                     f'(grace {elapsed:.1f}/{self._startup_grace_sec:.0f}s)'])
                return
            if self._require_points and not self._points_received:
                self._safety_state_str = SafetyState.WAITING_POINTS
                self._publish_zero()
                self._publish_state()
                self._publish_diagnostics(
                    SafetyState.WAITING_POINTS,
                    [f'Waiting for {self._points_topic} '
                     f'(grace {elapsed:.1f}/{self._startup_grace_sec:.0f}s)'])
                return
            # 传感器已就绪，fall through 到正常逻辑

        # 2. 控制权仲裁 (Phase 10.3: Web > Nav2, 超时降级)
        self._arbitrate(now)

        # 输入速度超时检查 (仲裁后统一检查)
        if self._input_valid:
            dt = now - self._last_cmd_time
            if dt > self._cmd_tout:
                reasons.append(f'CMD_TIMEOUT ({dt:.2f}s > {self._cmd_tout:.2f}s)')
                self._input_valid = False

        # 3. Odom：grace 后从未收到 → 永久超时
        if self._require_odom:
            if not self._odom_received:
                reasons.append(f'ODOM_TIMEOUT - never received on {self._odom_topic}')
                self._safety_state_str = SafetyState.ODOM_TIMEOUT
                self._publish_zero()
                self._publish_state()
                self._publish_diagnostics(SafetyState.ODOM_TIMEOUT, reasons)
                return
            if self._odom_healthy:
                dt = now - self._last_odom_time
                if dt > self._odom_tout:
                    reasons.append(f'ODOM_TIMEOUT ({dt:.2f}s > {self._odom_tout:.2f}s)')
                    self._odom_healthy = False

        # 4. Points：grace 后从未收到 → 永久超时
        if self._require_points:
            if not self._points_received:
                reasons.append(f'POINTS_TIMEOUT - never received on {self._points_topic}')
                self._safety_state_str = SafetyState.POINTS_TIMEOUT
                self._publish_zero()
                self._publish_state()
                self._publish_diagnostics(SafetyState.POINTS_TIMEOUT, reasons)
                return
            if self._points_healthy:
                dt = now - self._last_points_time
                if dt > self._points_tout:
                    reasons.append(f'POINTS_TIMEOUT ({dt:.2f}s > {self._points_tout:.2f}s)')
                    self._points_healthy = False

        # 5. Battery 超时/低电量
        if self._require_battery and not self._battery_healthy:
            reasons.append('BATTERY_LOW')

        # 6. 如果任何 watchdog 触发，输出 0
        if not self._input_valid:
            reasons.append('input invalid')
            self._safety_state_str = SafetyState.CMD_TIMEOUT
            self._publish_zero()
            self._publish_state()
            self._publish_diagnostics(SafetyState.CMD_TIMEOUT, reasons)
            return

        if not self._odom_healthy:
            self._safety_state_str = SafetyState.ODOM_TIMEOUT
            self._publish_zero()
            self._publish_state()
            self._publish_diagnostics(SafetyState.ODOM_TIMEOUT, reasons)
            return

        if not self._points_healthy:
            self._safety_state_str = SafetyState.POINTS_TIMEOUT
            self._publish_zero()
            self._publish_state()
            self._publish_diagnostics(SafetyState.POINTS_TIMEOUT, reasons)
            return

        if not self._battery_healthy:
            self._safety_state_str = SafetyState.BATTERY_LOW
            self._publish_zero()
            self._publish_state()
            self._publish_diagnostics(SafetyState.BATTERY_LOW, reasons)
            return

        # 7. 正常 — 应用速度/加速度限幅
        safe_vx, safe_vy, safe_wz = self._apply_limits(self._input_vx, self._input_vy, self._input_wz, now)


        is_limiting = (abs(safe_vx - self._input_vx) > 0.001 or
                       abs(safe_vy - self._input_vy) > 0.001 or
                       abs(safe_wz - self._input_wz) > 0.001)

        if is_limiting:
            self._safety_state_str = SafetyState.LIMITING
            self._publish_safe(safe_vx, safe_vy, safe_wz, now)
            self._publish_state()
            self._publish_diagnostics(SafetyState.LIMITING,
                                      [f'vel clipped: vx={self._input_vx:.2f}→{safe_vx:.2f}',
                                       f'wz={self._input_wz:.2f}→{safe_wz:.2f}'])
        else:
            self._safety_state_str = SafetyState.OK
            self._publish_safe(safe_vx, safe_vy, safe_wz, now)
            self._publish_state()
            self._publish_diagnostics(SafetyState.OK, ['All checks passed'])

    # ---- 核心限幅 ----

    # MCU 死区阈值 (ZSL-1W 真实底盘 — 阶段 7 落地实测)
    # ⚠️ 三轴死区不同，不可统一处理！
    # vy 横向平移有效阈值 (~0.12-0.15) 显著高于 vx (~0.05)，这是 ZSL-1W MCU 硬件特性
    MCU_DEADZONE_VX = 0.05   # |vx| < 0.05 m/s → 0
    MCU_DEADZONE_VY = 0.12   # |vy| < 0.12 m/s → 0 (横向死区比纵向高 2.4x)
    MCU_DEADZONE_WZ = 0.10   # |wz| < 0.10 rad/s → 0

    def _apply_limits(self, req_vx: float, req_vy: float, req_wz: float, now: float):
        """应用速度限幅 + 加速度限幅 + MCU 死区过滤"""
        dt = 1.0 / self._rate

        # 速度限幅
        vx = max(self._min_vx, min(self._max_vx, req_vx))
        vy = max(self._min_vx, min(self._max_vx, req_vy))
        wz = max(-self._max_wz, min(self._max_wz, req_wz))

        # 加速度限幅
        dvx = vx - self._last_vx
        dvy = vy - self._last_vy
        dwz = wz - self._last_wz

        max_dvx = self._max_ax * dt
        max_dwz = self._max_aw * dt

        dvx = max(-max_dvx, min(max_dvx, dvx))
        dvy = max(-max_dvx, min(max_dvx, dvy))
        dwz = max(-max_dwz, min(max_dwz, dwz))

        safe_vx = self._last_vx + dvx
        safe_vy = self._last_vy + dvy
        safe_wz = self._last_wz + dwz

        # 更新上次速度
        self._last_vx = safe_vx
        self._last_vy = safe_vy
        self._last_wz = safe_wz

        return safe_vx, safe_vy, safe_wz

    # ---- 发布辅助 ----

    def _publish_zero(self):
        msg = Twist()
        self._cmd_pub.publish(msg)
        if self._base_pub is not None:
            self._base_pub.publish(msg)
        self._last_vx = 0.0
        self._last_vy = 0.0
        self._last_wz = 0.0

    def _publish_safe(self, vx: float, vy: float, wz: float, now: float):
        # MCU 死区过滤 — 在最终输出层过滤，不影响内部加速限幅状态
        if vx != 0.0 and abs(vx) < self.MCU_DEADZONE_VX:
            vx = 0.0
        if vy != 0.0 and abs(vy) < self.MCU_DEADZONE_VY:
            vy = 0.0
        if wz != 0.0 and abs(wz) < self.MCU_DEADZONE_WZ:
            wz = 0.0

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        self._cmd_pub.publish(msg)
        if self._base_pub is not None:
            self._base_pub.publish(msg)

    def _publish_state(self):
        msg = String()
        msg.data = self._safety_state_str
        self._state_pub.publish(msg)

    def _publish_diagnostics(self, state: str, reasons: list):
        now = self.get_clock().now()
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()
        diag.header.frame_id = ''

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        if state in (SafetyState.ESTOP, SafetyState.ERROR):
            status.level = DiagnosticStatus.ERROR
        elif state in (SafetyState.CMD_TIMEOUT, SafetyState.ODOM_TIMEOUT,
                       SafetyState.POINTS_TIMEOUT, SafetyState.BATTERY_LOW,
                       SafetyState.DISABLED,
                       SafetyState.WAITING_ODOM, SafetyState.WAITING_POINTS):
            status.level = DiagnosticStatus.WARN
        elif state == SafetyState.LIMITING:
            status.level = DiagnosticStatus.OK  # limiting is normal operation

        status.name = 'gb_safety: Safety Gate'
        status.message = f'State: {state}'
        status.hardware_id = 'gb_safety'

        for r in reasons:
            kv = KeyValue()
            kv.key = 'reason'
            kv.value = r
            status.values.append(kv)

        # 附加数值
        for key, val in [
            ('enabled', str(self._enabled)),
            ('estop_triggered', str(self._estop_triggered)),
            ('input_vx', f'{self._input_vx:.3f}'),
            ('input_vy', f'{self._input_vy:.3f}'),
            ('input_wz', f'{self._input_wz:.3f}'),
            ('last_vx', f'{self._last_vx:.3f}'),
            ('last_vy', f'{self._last_vy:.3f}'),
            ('last_wz', f'{self._last_wz:.3f}'),
            ('web_active', str(self._web_active)),
            ('nav_vx', f'{self._nav_vx:.3f}'),
            ('web_vx', f'{self._web_vx:.3f}'),
            ('source', 'web' if self._web_active else ('nav' if self._input_valid else 'none')),
        ]:
            kv = KeyValue()
            kv.key = key
            kv.value = val
            status.values.append(kv)

        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
