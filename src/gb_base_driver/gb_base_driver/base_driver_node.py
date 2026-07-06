"""钢镚底盘驱动节点 — Mock 版

订阅:
  /cmd_vel_base (geometry_msgs/Twist)
  /emergency_stop (std_msgs/Bool)

发布:
  /battery_state (sensor_msgs/BatteryState)
  /robot_state (std_msgs/String)
  /diagnostics (diagnostic_msgs/DiagnosticArray)
  /odom_raw (nav_msgs/Odometry) — mock 里程计

参数:
  max_linear_speed, max_angular_speed, cmd_timeout_ms
  enable_mock_odom, publish_rate
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String, Header
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class GbBaseDriverNode(Node):
    """钢镚底盘驱动 (Mock)"""

    def __init__(self):
        super().__init__('gb_base_driver_node')

        # ===== 参数 =====
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_linear_accel', 1.0)
        self.declare_parameter('max_angular_accel', 2.0)
        self.declare_parameter('cmd_timeout_ms', 500)
        self.declare_parameter('enable_mock_odom', True)
        self.declare_parameter('mock_battery_percentage', 85)
        self.declare_parameter('mock_battery_voltage', 24.0)
        self.declare_parameter('publish_rate', 20.0)

        self._max_linear = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._max_lin_accel = self.get_parameter('max_linear_accel').value
        self._max_ang_accel = self.get_parameter('max_angular_accel').value
        self._cmd_timeout_ns = self.get_parameter('cmd_timeout_ms').value * 1_000_000
        self._enable_mock = self.get_parameter('enable_mock_odom').value
        self._bat_pct = self.get_parameter('mock_battery_percentage').value
        self._bat_volt = self.get_parameter('mock_battery_voltage').value
        self._pub_rate = self.get_parameter('publish_rate').value

        # ===== 状态 =====
        self._cmd_vel = Twist()          # 当前 cmd_vel (限速后)
        self._cmd_vel_raw = Twist()      # 原始 cmd_vel
        self._estop = False
        self._last_cmd_time = self.get_clock().now()

        # Mock odometry 积分
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom_time = self.get_clock().now()

        # ===== 订阅 =====
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel_base', self._on_cmd_vel, qos)
        self._sub_estop = self.create_subscription(
            Bool, '/emergency_stop', self._on_estop, qos)

        # ===== 发布 =====
        self._pub_battery = self.create_publisher(BatteryState, '/battery_state', qos)
        self._pub_state = self.create_publisher(String, '/robot_state', qos)
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', qos)
        self._pub_odom = self.create_publisher(Odometry, '/odom_raw', qos) if self._enable_mock else None

        # ===== 定时器 =====
        self._timer = self.create_timer(1.0 / self._pub_rate, self._on_timer)

        self.get_logger().info(
            f'gb_base_driver_node started (mock), rate={self._pub_rate}Hz')

    # ---- 回调 ----

    def _on_cmd_vel(self, msg: Twist):
        self._cmd_vel_raw = msg
        self._last_cmd_time = self.get_clock().now()

        # 限速
        vx = max(-self._max_linear, min(self._max_linear, msg.linear.x))
        vy = max(-self._max_linear, min(self._max_linear, msg.linear.y))
        wz = max(-self._max_angular, min(self._max_angular, msg.angular.z))

        # 限加速度 (简单一阶)
        dt = (self.get_clock().now() - self._last_odom_time).nanoseconds / 1e9
        if dt > 0:
            dvx = vx - self._cmd_vel.linear.x
            dvy = vy - self._cmd_vel.linear.y
            dwz = wz - self._cmd_vel.angular.z
            max_dv = self._max_lin_accel * dt
            max_dw = self._max_ang_accel * dt
            vx = self._cmd_vel.linear.x + max(-max_dv, min(max_dv, dvx))
            vy = self._cmd_vel.linear.y + max(-max_dv, min(max_dv, dvy))
            wz = self._cmd_vel.angular.z + max(-max_dw, min(max_dw, dwz))

        self._cmd_vel.linear.x = vx
        self._cmd_vel.linear.y = vy
        self._cmd_vel.angular.z = wz

    def _on_estop(self, msg: Bool):
        self._estop = msg.data
        if self._estop:
            self._cmd_vel = Twist()
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

    # ---- Timer ----

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self._last_cmd_time).nanoseconds

        # cmd_vel timeout → 归零
        if dt > self._cmd_timeout_ns and not self._estop:
            self._cmd_vel = Twist()

        # 急停 → 强制归零
        if self._estop:
            self._cmd_vel = Twist()

        # 发布电池
        self._pub_battery.publish(self._make_battery(now))

        # 发布机器人状态
        self._pub_state.publish(self._make_state(now))

        # 发布诊断
        self._pub_diag.publish(self._make_diag(now))

        # Mock 里程计积分
        if self._enable_mock and self._pub_odom:
            odom_dt = (now - self._last_odom_time).nanoseconds / 1e9
            if odom_dt > 0:
                vx = self._cmd_vel.linear.x
                vy = self._cmd_vel.linear.y
                wz = self._cmd_vel.angular.z
                self._odom_x += (vx * math.cos(self._odom_yaw) - vy * math.sin(self._odom_yaw)) * odom_dt
                self._odom_y += (vx * math.sin(self._odom_yaw) + vy * math.cos(self._odom_yaw)) * odom_dt
                self._odom_yaw += wz * odom_dt
            self._last_odom_time = now
            self._pub_odom.publish(self._make_odom(now))

    # ---- 消息构造 ----

    def _make_battery(self, now):
        msg = BatteryState()
        msg.header = Header(stamp=now.to_msg(), frame_id='base_link')
        msg.voltage = self._bat_volt
        msg.percentage = self._bat_pct / 100.0
        # 模拟放电
        self._bat_pct = max(10, self._bat_pct - 0.001 * self._pub_rate)
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return msg

    def _make_state(self, now):
        state = {
            'mode': 'standby' if self._estop else 'active',
            'estop': self._estop,
            'cmd_vel': {
                'vx': self._cmd_vel.linear.x,
                'vy': self._cmd_vel.linear.y,
                'wz': self._cmd_vel.angular.z,
            },
            'battery_pct': self._bat_pct,
        }
        msg = String()
        msg.data = json.dumps(state, ensure_ascii=False)
        return msg

    def _make_diag(self, now):
        array = DiagnosticArray()
        array.header = Header(stamp=now.to_msg(), frame_id='base_link')

        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = 'gb_base_driver'
        status.message = 'Running (mock)' if not self._estop else 'ESTOP'
        status.values = [
            KeyValue(key='estop', value=str(self._estop)),
            KeyValue(key='vx', value=f'{self._cmd_vel.linear.x:.3f}'),
            KeyValue(key='vy', value=f'{self._cmd_vel.linear.y:.3f}'),
            KeyValue(key='wz', value=f'{self._cmd_vel.angular.z:.3f}'),
            KeyValue(key='cmd_timeout_ns', value=str(self._cmd_timeout_ns)),
            KeyValue(key='battery_pct', value=f'{self._bat_pct:.1f}'),
        ]
        array.status.append(status)
        return array

    def _make_odom(self, now):
        msg = Odometry()
        msg.header = Header(stamp=now.to_msg(), frame_id='odom_lio')
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        # orientation from yaw
        msg.pose.pose.orientation.z = math.sin(self._odom_yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self._odom_yaw / 2)
        msg.twist.twist = self._cmd_vel
        # covariance (mock)
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.1
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = GbBaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
