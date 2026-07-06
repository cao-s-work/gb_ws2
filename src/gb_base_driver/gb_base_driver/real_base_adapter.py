#!/usr/bin/env python3
"""
钢镚真实底盘 adapter — ZSL-1W SDK ROS2 桥接节点

Phase 1 (只读接入):
  - SDK 连接钢镚底盘
  - 读取 battery / mode / RPY / velocity / motor state
  - 发布 /battery_state, /robot_state, /diagnostics
  - 订阅 /cmd_vel_base 但 read_only=True 时忽略速度指令
  - 不调用 standUp() / move() / passive()
  - 不抢 /cmd_vel_base publisher (只订阅)

Phase 2+ (架空/落地测试):
  - read_only=False 时, /cmd_vel_base → SDK move(vx, 0, wz)
  - 低速限制: max_linear_speed, max_angular_speed
  - 急停: /emergency_stop → SDK passive() + move(0,0,0)

安全链路:
  safety_node → /cmd_vel_base → 本节点 (订阅) → SDK move()
  本节点永远不发布 /cmd_vel_base
"""

import gc
import json
import math
import os
import sys
import platform
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import Trigger, SetBool

# ---- SDK 路径设置 ----
arch = platform.machine().replace('amd64', 'x86_64').replace('arm64', 'aarch64')
# 从本文件向上查找包含 sdk/ 的工作空间根目录
_this_dir = os.path.dirname(os.path.abspath(__file__))
_ws_root = _this_dir
for _ in range(6):
    if os.path.isdir(os.path.join(_ws_root, 'sdk')):
        break
    _ws_root = os.path.dirname(_ws_root)
_SDK_LIB_PATH = os.environ.get(
    'GB_SDK_LIB_PATH',
    os.path.join(_ws_root, 'sdk', 'genisom_l1_sdk-main', 'lib', 'zsl-1', arch))
sys.path.insert(0, _SDK_LIB_PATH)


class RealBaseAdapter(Node):
    """钢镚真实底盘 adapter (ZSL-1W SDK)"""

    # 控制模式映射
    MODE_MAP = {
        0: 'PASSIVE',      # 阻尼/趴下
        1: 'STAND',        # 站立
        10: 'LIE_FREE',    # 趴下(自由)
        18: 'MOVE',        # 移动
        21: 'ACTION',      # 动作
        51: 'LIE_DOWN',    # 趴下
    }

    def __init__(self):
        super().__init__('gb_base_driver_node')  # 同名替换 mock

        # ===== 参数 =====
        self.declare_parameter('sdk_local_ip', '192.168.168.216')
        self.declare_parameter('sdk_local_port', 43988)
        self.declare_parameter('sdk_dog_ip', '192.168.168.168')
        self.declare_parameter('read_only', True)  # Phase 1: 只读
        self.declare_parameter('max_linear_speed', 0.60)   # Phase 2: 0.05 m/s → 0.60
        self.declare_parameter('max_angular_speed', 0.80)  # Phase 2: 0.15 rad/s → 0.80
        self.declare_parameter('cmd_timeout_ms', 500)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('sdk_lib_path', _SDK_LIB_PATH)

        self._local_ip = self.get_parameter('sdk_local_ip').value
        self._local_port = self.get_parameter('sdk_local_port').value
        self._dog_ip = self.get_parameter('sdk_dog_ip').value
        self._read_only = self.get_parameter('read_only').value
        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._cmd_timeout_ns = self.get_parameter('cmd_timeout_ms').value * 1_000_000
        self._pub_rate = self.get_parameter('publish_rate').value

        # ===== SDK 初始化 =====
        self._sdk = None
        self._sdk_connected = False
        self._sdk_lock = threading.Lock()

        # ===== 自动重连 (Phase 6.2) =====
        self._was_connected = False           # 曾经连上过（用于检测断连事件）
        self._reconnect_attempts = 0           # 当前断连期间的重试次数
        self._last_reconnect_time = 0.0        # 上次重连尝试时间 (time.time())
        self._reconnect_interval = 2.0         # 重连间隔 (秒)
        self._max_reconnect_attempts = 100     # 防 fd 泄漏, 100 次后停止重连
        self._force_safe_stop = True           # 断连时强制归零速度

        self._init_sdk()

        # ===== 状态 =====
        self._cmd_vel = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._estop = False
        self._dog_mode = -1  # SDK 模式码 (18=MOVE, 1=STAND, 0=PASSIVE, 51=LIE_DOWN)

        # ===== 运动状态机 (防止 PASSIVE 下轰炸 MCU) =====
        self.MOVE_MODES = (1, 18)         # STAND, MOVE
        self._movement_allowed = False
        self._mode_stable_count = 0
        self._mode_stable_target = 3      # 连续稳定 STAND/MOVE 3 帧才允许 move
        self._sent_stop_move = False      # 归零 move 只发一次, 不重复刷

        # SDK 读取失败检测 (Phase 6.2)
        self._read_fail_count = 0          # 连续读取失败次数
        self._read_fail_max = 5            # 连续失败 N 次视为断连

        # ===== 回调组 (分离 cmd_vel 与 timer, 避免 SDK 阻塞回调) =====
        self._cb_cmd = ReentrantCallbackGroup()       # cmd_vel / estop — 高优先级
        self._cb_timer = MutuallyExclusiveCallbackGroup()  # timer — 低优先级
        self._cb_srv = ReentrantCallbackGroup()        # 服务 — 独立，不阻塞 timer

        # ===== 订阅 (只订阅, 不发布 /cmd_vel_base) =====
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_base', self._on_cmd_vel, qos,
            callback_group=self._cb_cmd)
        self._sub_estop = self.create_subscription(
            Bool, '/emergency_stop', self._on_estop, qos,
            callback_group=self._cb_cmd)

        # ===== 发布 =====
        self._pub_battery = self.create_publisher(BatteryState, '/battery_state', qos)
        self._pub_state = self.create_publisher(String, '/robot_state', qos)
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', qos)

        # ===== 服务 (Phase 2: standUp / lieDown / passive) =====
        self._srv_standup = self.create_service(
            Trigger, '/gb_base/stand_up', self._srv_standup,
            callback_group=self._cb_srv)
        self._srv_liedown = self.create_service(
            Trigger, '/gb_base/lie_down', self._srv_liedown,
            callback_group=self._cb_srv)
        self._srv_passive = self.create_service(
            Trigger, '/gb_base/passive', self._srv_passive,
            callback_group=self._cb_srv)

        # ===== 定时器 =====
        self._timer = self.create_timer(
            1.0 / self._pub_rate, self._on_timer,
            callback_group=self._cb_timer)

        # ===== SDK 心跳保活 (狗 3s 超时, 每 1.5s 发一次, 独立回调组避免阻塞) =====
        self._cb_heartbeat = ReentrantCallbackGroup()
        self._heartbeat_timer = self.create_timer(
            1.5, self._on_heartbeat,
            callback_group=self._cb_heartbeat)

        mode_str = 'READ_ONLY' if self._read_only else 'ACTIVE'
        self.get_logger().info(
            f'real_base_adapter 启动 [{mode_str}], rate={self._pub_rate}Hz, '
            f'max_vx={self._max_linear}, max_wz={self._max_angular}')

    # ---- SDK 连接管理 (Phase 6.2) ----

    def _init_sdk(self):
        """首次 SDK 初始化 (__init__ 调用)"""
        try:
            import mc_sdk_zsl_1_py
            self._sdk = mc_sdk_zsl_1_py.HighLevel()
            self._sdk.initRobot(self._local_ip, self._local_port, self._dog_ip)
            self.get_logger().info(
                f'SDK initRobot: local={self._local_ip}:{self._local_port}, '
                f'dog={self._dog_ip}')
            time.sleep(2)
            self._sdk_connected = self._sdk.checkConnect()
            if self._sdk_connected:
                self._was_connected = True
                self.get_logger().info('✅ SDK 连接成功')
            else:
                self.get_logger().warn('⚠️ SDK 连接中... 数据流可能需要更长时间')
        except Exception as e:
            self.get_logger().error(f'❌ SDK 初始化失败: {e}')
            self._sdk = None

    def _connect_sdk(self):
        """尝试重连 SDK (Phase 6.2).
        返回 True 表示重连成功。
        安全保证: 不自动 standUp / move / 恢复旧速度
        """
        # 释放旧的 SDK 对象 + socket，避免端口冲突
        if self._sdk is not None:
            try:
                del self._sdk
            except Exception:
                pass
            self._sdk = None
            gc.collect()  # 强制 GC 触发 C++ 析构
            time.sleep(2.0)  # 等 OS 释放 socket (TIME_WAIT) + DNS 解析完成

        try:
            import mc_sdk_zsl_1_py
            self._sdk = mc_sdk_zsl_1_py.HighLevel()
            self._sdk.initRobot(self._local_ip, self._local_port, self._dog_ip)
            time.sleep(1)
            connected = self._sdk.checkConnect()
            if connected:
                self._sdk_connected = True
                self._was_connected = True
                self._reconnect_attempts = 0
                self._cmd_vel = Twist()
                self._movement_allowed = False
                self._mode_stable_count = 0
                self._sent_stop_move = True

                # 重连后同步 SDK 状态机: 若狗在线需重新 standUp 让 SDK 知道当前状态
                # ⚠️ read_only 模式下禁止任何运动指令
                time.sleep(0.5)
                try:
                    mode = self._sdk.getCurrentCtrlmode()
                    self._dog_mode = mode
                    if mode == 1 and not self._read_only:  # STAND — 仅非只读模式下同步
                        self._sdk.standUp()
                        self.get_logger().info('✅ SDK 状态同步: standUp (狗已在站立)')
                    elif mode == 1:
                        self.get_logger().info('🛡️ read_only 模式: 跳过 standUp 状态同步 (狗当前站立)')
                except Exception:
                    pass

                self.get_logger().info('✅ SDK 自动重连成功 — 等待新指令')
                return True
        except Exception as e:
            self.get_logger().warn(f'SDK 重连失败: {e}')
            if self._sdk is not None:
                try:
                    del self._sdk
                except Exception:
                    pass
            self._sdk = None
            gc.collect()
            time.sleep(2.0)  # 等 OS 释放 fd
        return False

    def _on_disconnect(self):
        """SDK 断连时的安全处理 (Phase 6.2).
        仅首次检测到断连时调用一次 (由 _on_timer 中的状态变化触发)."""
        self.get_logger().error('🔌 SDK 断连! 清零速度, 禁止运动')
        # 强制清零目标速度
        self._cmd_vel = Twist()
        self._movement_allowed = False
        self._mode_stable_count = 0
        self._sent_stop_move = True
        self._reconnect_attempts = 0
        self._read_fail_count = 0
        self._last_reconnect_time = time.time()

    # ---- 心跳保活 (狗 3s 超时) ----
    def _on_heartbeat(self):
        """每 1s 调用 sdk.getCurrentCtrlmode() 保活, 确保 UDP 包发出"""
        if not self._sdk or not self._sdk_connected:
            return
        try:
            with self._sdk_lock:
                self._sdk.getCurrentCtrlmode()
        except Exception:
            pass

    # ---- 回调 ----

    def _on_cmd_vel(self, msg: Twist):
        """订阅 /cmd_vel_base — 只缓存目标速度, 不直接调 move()
        实际 move() 调用由 timer 状态机统一管理, 防止 PASSIVE 下轰炸 MCU"""
        self._last_cmd_time = self.get_clock().now()

        if self._read_only:
            return

        if self._estop:
            return

        # 低速限制 + 缓存
        vx = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        vy = max(-self._max_linear, min(self._max_linear, msg.linear.y))
        wz = max(-self._max_angular, min(self._max_angular, msg.angular.z))

        # MCU 死区过滤: 非零但低于死区的速度 → 0（debug 级别记录）
        if vx != 0.0 and abs(vx) < 0.05:
            self.get_logger().debug(f'MCU deadzone vx={vx:.3f} → 0')
            vx = 0.0
        if vy != 0.0 and abs(vy) < 0.12:  # ZSL-1W 横向死区 0.12–0.15 (阶段 7 实测)
            self.get_logger().debug(f'MCU deadzone vy={vy:.3f} → 0')
            vy = 0.0
        if wz != 0.0 and abs(wz) < 0.10:
            self.get_logger().debug(f'MCU deadzone wz={wz:.3f} → 0')
            wz = 0.0

        self._cmd_vel.linear.x = vx
        self._cmd_vel.linear.y = vy
        self._cmd_vel.angular.z = wz
        self._sent_stop_move = False  # 新指令到来, 重置归零标记

    def _on_estop(self, msg: Bool):
        self._estop = msg.data
        if self._estop:
            self._cmd_vel = Twist()
            self.get_logger().warn('🔴 急停触发')
            if not self._read_only and self._sdk:
                try:
                    with self._sdk_lock:
                        self._sdk.move(0, 0, 0)
                        time.sleep(0.05)
                        self._sdk.passive()
                except Exception:
                    pass

    # ---- 服务回调 (Phase 2) ----

    def _srv_standup(self, request, response):
        """站立 — 只在非 read_only 模式可用，轮询直到 MOVE(18)"""
        if self._read_only:
            response.success = False
            response.message = 'read_only 模式, 禁止 standUp'
            return response
        if not self._sdk or not self._sdk_connected:
            response.success = False
            response.message = 'SDK 未连接'
            return response
        try:
            with self._sdk_lock:
                self._sdk.standUp()
            self.get_logger().info('🦾 standUp() 已发送, 等待 STAND(1) 模式...')
            # 轮询直到狗进入 STAND 模式 (1), 最多等 10 秒
            # 参考代码: 明确等 mode==1, 非 mode!=0 (过渡态可能不接受 move)
            stand_ok = False
            for i in range(100):
                time.sleep(0.1)
                with self._sdk_lock:
                    mode = self._sdk.getCurrentCtrlmode()
                if mode == 1:
                    self._dog_mode = mode
                    stand_ok = True
                    self.get_logger().info(f'  → STAND(1) 确认 (尝试 {i+1} 次)')
                    break
            if stand_ok:
                response.success = True
                response.message = f'standUp 完成, 模式=STAND(1) ({i*0.1:.1f}s)'
                self.get_logger().info(f'✅ standUp → STAND(1) ({i*0.1:.1f}s)')
                return response
            # 超时
            response.success = False
            response.message = 'standUp 超时: 狗未进入 STAND(1)'
            self.get_logger().error('❌ standUp 超时: 狗未进入 STAND(1)')
        except Exception as e:
            response.success = False
            response.message = f'standUp() 失败: {e}'
            self.get_logger().error(f'standUp() 失败: {e}')
        return response

    def _srv_liedown(self, request, response):
        """趴下"""
        if self._read_only:
            response.success = False
            response.message = 'read_only 模式, 禁止 lieDown'
            return response
        if not self._sdk or not self._sdk_connected:
            response.success = False
            response.message = 'SDK 未连接'
            return response
        try:
            with self._sdk_lock:
                self._sdk.move(0, 0, 0)
                time.sleep(0.05)
                self._sdk.lieDown()
            self._cmd_vel = Twist()
            response.success = True
            response.message = 'lieDown() 已发送'
            self.get_logger().info('🐕 lieDown() 已发送')
        except Exception as e:
            response.success = False
            response.message = f'lieDown() 失败: {e}'
            self.get_logger().error(f'lieDown() 失败: {e}')
        return response

    def _srv_passive(self, request, response):
        """阻尼模式 (立即停止)"""
        if not self._sdk or not self._sdk_connected:
            response.success = False
            response.message = 'SDK 未连接'
            return response
        try:
            with self._sdk_lock:
                self._sdk.move(0, 0, 0)
                time.sleep(0.05)
                self._sdk.passive()
            self._cmd_vel = Twist()
            response.success = True
            response.message = 'passive() 已发送'
            self.get_logger().info('🛑 passive() 已发送')
        except Exception as e:
            response.success = False
            response.message = f'passive() 失败: {e}'
            self.get_logger().error(f'passive() 失败: {e}')
        return response

    # ---- 定时器 ----

    def _on_timer(self):
        now = self.get_clock().now()
        now_sec = time.time()

        # 1. 检查 SDK 连接 + 自动重连 (Phase 6.2)
        was_connected = self._sdk_connected
        if self._sdk:
            try:
                self._sdk_connected = self._sdk.checkConnect()
            except Exception:
                self._sdk_connected = False

        # 检测 connected→disconnected 事件
        if was_connected and not self._sdk_connected:
            self._on_disconnect()

        # 自动重连循环
        if not self._sdk_connected:
            if (self._max_reconnect_attempts == 0 or
                    self._reconnect_attempts < self._max_reconnect_attempts):
                if now_sec - self._last_reconnect_time >= self._reconnect_interval:
                    self._reconnect_attempts += 1
                    self._last_reconnect_time = now_sec
                    max_str = "∞" if self._max_reconnect_attempts == 0 else str(self._max_reconnect_attempts)
                    self.get_logger().info(
                        f'🔄 尝试重连 SDK ({self._reconnect_attempts}/{max_str})...')
                    self._connect_sdk()
            elif self._max_reconnect_attempts > 0:
                # 超过最大重试次数，不再重连
                pass  # _sdk_connected 保持 False, 运动永久禁止

        # 2. 读取 SDK 数据
        data = self._read_sdk()

        # 读取失败检测: 连续失败 N 次视为 SDK 断连 (Phase 6.2)
        if data is None and self._sdk_connected:
            self._read_fail_count += 1
            if self._read_fail_count >= self._read_fail_max:
                self.get_logger().error(
                    f'🔌 SDK 读取连续失败 {self._read_fail_count} 次, 标记断连')
                self._sdk_connected = False
                self._on_disconnect()
                self._read_fail_count = 0
        elif data is not None:
            self._read_fail_count = 0

        # 3. 更新模式稳定性计数器 (防止过渡态误判)
        if data and isinstance(data.get('mode'), (int, float)):
            new_mode = int(data['mode'])
            if new_mode in self.MOVE_MODES:
                if self._dog_mode == new_mode:
                    self._mode_stable_count += 1
                else:
                    self._mode_stable_count = 1
            else:
                self._mode_stable_count = 0
                self._movement_allowed = False
            self._dog_mode = new_mode

        # 4. 运动允许门禁: read_only? SDK连接? 模式? 稳定?
        if (not self._read_only and self._sdk_connected and
                self._dog_mode in self.MOVE_MODES and
                self._mode_stable_count >= self._mode_stable_target):
            self._movement_allowed = True
        else:
            self._movement_allowed = False

        # 5. 执行 move (由状态机统一管理, 绝不轰炸 MCU)
        if self._movement_allowed and not self._estop and self._sdk:
            dt = (now - self._last_cmd_time).nanoseconds
            if dt <= self._cmd_timeout_ns:
                # 新鲜指令: 持续发出目标速度 (参考代码 50ms 循环模式)
                try:
                    vx_cmd = self._cmd_vel.linear.x
                    vy_cmd = self._cmd_vel.linear.y
                    wz_cmd = self._cmd_vel.angular.z
                    with self._sdk_lock:
                        self._sdk.move(vx_cmd, vy_cmd, wz_cmd)
                    self._sent_stop_move = False
                    if vx_cmd != 0.0 or vy_cmd != 0.0 or wz_cmd != 0.0:
                        self.get_logger().info(f'🏃 SDK.move({vx_cmd:.3f}, {vy_cmd:.3f}, {wz_cmd:.3f})')
                except Exception as e:
                    self.get_logger().warn(f'SDK.move 失败: {e}')
            elif not self._sent_stop_move:
                # 超时: 归零一次, 不重复刷
                try:
                    with self._sdk_lock:
                        self._sdk.move(0, 0, 0)
                    self._sent_stop_move = True
                except Exception:
                    pass
        else:
            # 不在运动模式: 重置归零标记, 绝不调 move
            if not self._sent_stop_move:
                # 首次阻塞时记录原因
                reasons = []
                if self._read_only: reasons.append('read_only')
                if not self._sdk_connected: reasons.append('SDK断开')
                if self._estop: reasons.append('急停')
                if self._dog_mode not in self.MOVE_MODES: reasons.append(f'mode={self._dog_mode}')
                if self._mode_stable_count < self._mode_stable_target: reasons.append(f'稳定={self._mode_stable_count}/{self._mode_stable_target}')
                if reasons:
                    self.get_logger().warn(f'⛔ 运动禁止: {", ".join(reasons)}')
                self._sent_stop_move = True

        # 6. 发布状态
        self._pub_battery.publish(self._make_battery(now, data))
        self._pub_state.publish(self._make_state(now, data))
        self._pub_diag.publish(self._make_diag(now, data))

    # ---- SDK 数据读取 ----

    def _read_sdk(self):
        """读取 SDK 数据, 返回 dict"""
        if not self._sdk or not self._sdk_connected:
            return None

        try:
            with self._sdk_lock:
                return {
                    'battery': self._sdk.getBatteryPower(),
                    'mode': self._sdk.getCurrentCtrlmode(),
                    'rpy': self._sdk.getRPY(),
                    'gyro': self._sdk.getBodyGyro(),
                    'body_vel': self._sdk.getBodyVelocity(),
                    'position': self._sdk.getPosition(),
                    'quaternion': self._sdk.getQuaternion(),
                }
        except Exception as e:
            self.get_logger().warn(f'SDK 读取失败: {e}')
            return None

    # ---- 消息构造 ----

    def _make_battery(self, now, data=None):
        msg = BatteryState()
        msg.header = Header(stamp=now.to_msg(), frame_id='base_link')

        if data is None:
            data = self._read_sdk()
        if data and isinstance(data['battery'], (int, float)) and 0 <= data['battery'] <= 100:
            msg.percentage = float(data['battery']) / 100.0
            # 估算电压 (24V 系统, 0%≈20V, 100%≈29.4V)
            msg.voltage = 20.0 + (data['battery'] / 100.0) * 9.4
        else:
            msg.percentage = 0.0
            msg.voltage = 0.0

        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return msg

    def _make_state(self, now, data=None):
        if data is None:
            data = self._read_sdk()
        mode_id = data['mode'] if data else -1
        mode_name = self.MODE_MAP.get(mode_id, f'UNKNOWN({mode_id})')

        state = {
            'source': 'real_base_adapter',
            'sdk_connected': self._sdk_connected,
            'reconnecting': not self._sdk_connected and self._was_connected,
            'reconnect_attempts': self._reconnect_attempts,
            'read_only': self._read_only,
            'mode': mode_name,
            'mode_id': mode_id,
            'estop': self._estop,
            'battery_pct': data['battery'] if data and isinstance(data.get('battery'), (int, float)) else -1,
            'rpy': [round(x, 4) for x in data['rpy']] if data else [0, 0, 0],
            'body_vel': [round(x, 4) for x in data['body_vel']] if data else [0, 0, 0],
            'position': [round(x, 4) for x in data['position']] if data else [0, 0, 0],
            'cmd_vel': {
                'vx': round(self._cmd_vel.linear.x, 3),
                'wz': round(self._cmd_vel.angular.z, 3),
            },
        }

        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False)
        return msg

    def _make_diag(self, now, data=None):
        array = DiagnosticArray()
        array.header = Header(stamp=now.to_msg(), frame_id='base_link')

        status = DiagnosticStatus()
        if data is None:
            data = self._read_sdk()
        mode_id = data['mode'] if data else -1
        mode_name = self.MODE_MAP.get(mode_id, 'UNKNOWN')

        if not self._sdk_connected:
            status.level = DiagnosticStatus.ERROR
            status.message = 'SDK not connected'
            if self._was_connected:
                status.message += f' (reconnecting: {self._reconnect_attempts} attempts)'
        elif self._estop:
            status.level = DiagnosticStatus.ERROR
            status.message = 'EMERGENCY STOP'
        elif mode_id in (0, 51):  # PASSIVE / LIE_DOWN
            status.level = DiagnosticStatus.OK
            status.message = f'Mode: {mode_name} (idle)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Mode: {mode_name}'

        status.name = 'gb_base_driver (real)'
        status.hardware_id = 'zsl-1'

        battery = data['battery'] if data and isinstance(data.get('battery'), (int, float)) else -1
        status.values = [
            KeyValue(key='sdk_connected', value=str(self._sdk_connected)),
            KeyValue(key='reconnecting', value=str(not self._sdk_connected and self._was_connected)),
            KeyValue(key='reconnect_attempts', value=str(self._reconnect_attempts)),
            KeyValue(key='read_only', value=str(self._read_only)),
            KeyValue(key='mode', value=mode_name),
            KeyValue(key='mode_id', value=str(mode_id)),
            KeyValue(key='estop', value=str(self._estop)),
            KeyValue(key='battery_pct', value=str(battery)),
            KeyValue(key='max_linear', value=str(self._max_linear)),
            KeyValue(key='max_angular', value=str(self._max_angular)),
        ]

        if data:
            rpy = data.get('rpy', [0, 0, 0])
            vel = data.get('body_vel', [0, 0, 0])
            status.values.append(KeyValue(key='roll', value=f'{rpy[0]:.4f}'))
            status.values.append(KeyValue(key='pitch', value=f'{rpy[1]:.4f}'))
            status.values.append(KeyValue(key='yaw', value=f'{rpy[2]:.4f}'))
            status.values.append(KeyValue(key='vx', value=f'{vel[0]:.4f}'))
            status.values.append(KeyValue(key='vy', value=f'{vel[1]:.4f}'))
            status.values.append(KeyValue(key='vz', value=f'{vel[2]:.4f}'))

        array.status.append(status)
        return array

    # ---- 清理 ----

    def destroy_node(self):
        if self._sdk and not self._read_only:
            try:
                with self._sdk_lock:
                    self._sdk.move(0, 0, 0)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealBaseAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
