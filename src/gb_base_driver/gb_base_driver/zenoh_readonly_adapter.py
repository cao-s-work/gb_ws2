#!/usr/bin/env python3
"""
钢镚 Zenoh 只读适配器 — Phase Z3: 狗端状态侦察

功能:
  - 通过 Zenoh RMW 连接狗端 ROS2 Domain 24
  - 订阅狗端: /highlevel_robotstate, /arc/mc_state, /odom/mc_odom
  - 发布 Jetson 侧标准状态: /robot_state, /battery_state, /odom_raw, /diagnostics

红线:
  - 不发布 /highlevel_cmd
  - 不发布 /arc/mc_mode_cmd
  - 不订阅 /cmd_vel_base
  - 不发送任何运动/站立/趴下命令
  - 纯只读

用法:
  需在 Zenoh 环境中运行:
    export ROS_DOMAIN_ID=24
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.168.168:7447"]'
    source /opt/ros/humble/setup.bash
    source ~/gb_ws/install/setup.bash
    ros2 run gb_base_driver zenoh_readonly_adapter
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Zenoh 侧狗端自定义消息 (补 PYTHONPATH)
import sys, os
_sdk_path = '/home/nvidia/gb_ws/install/robots_dog_msgs/local/lib/python3.10/dist-packages'
if _sdk_path not in sys.path:
    sys.path.insert(0, _sdk_path)

from robots_dog_msgs.msg import HighLevelRobotState, McState

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class ZenohReadonlyAdapter(Node):
    """只读适配器：接收狗端 Zenoh 状态，发布标准 ROS2 状态"""

    def __init__(self):
        super().__init__('zenoh_readonly_adapter')

        # ── QoS 配置 ──
        # /highlevel_robotstate: BEST_EFFORT, KEEP_LAST(1)
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # /arc/mc_state: RELIABLE, KEEP_LAST(10)
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /odom/mc_odom: 标准 odom QoS
        qos_odom = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 订阅狗端 Zenoh topic ──
        self._robotstate_sub = self.create_subscription(
            HighLevelRobotState,
            '/highlevel_robotstate',
            self._on_robotstate,
            qos_best_effort,
        )
        self._mc_state_sub = self.create_subscription(
            McState,
            '/arc/mc_state',
            self._on_mc_state,
            qos_reliable,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom/mc_odom',
            self._on_odom,
            qos_odom,
        )

        # ── 发布 Jetson 侧标准 topic ──
        self._robot_state_pub = self.create_publisher(String, '/robot_state', 10)
        self._battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # ── 状态缓存 ──
        self._last_robotstate = None
        self._last_mc_state = None
        self._last_odom = None
        self._last_robotstate_time = self.get_clock().now()
        self._last_mc_state_time = self.get_clock().now()
        self._last_odom_time = self.get_clock().now()

        # ── 定时器：1Hz 发布聚合状态 ──
        self._timer = self.create_timer(1.0, self._publish_status)

        # ── 诊断定时器：5Hz ──
        self._diag_timer = self.create_timer(0.2, self._publish_diagnostics)

        self.get_logger().info('Zenoh 只读适配器已启动')
        self.get_logger().info('  订阅: /highlevel_robotstate, /arc/mc_state, /odom/mc_odom')
        self.get_logger().info('  发布: /robot_state, /battery_state, /odom_raw, /diagnostics')
        self.get_logger().info('  模式: 只读 (不发送任何控制命令)')

    # ════════════════════════════════════════════════════════════
    # 狗端 Zenoh 回调
    # ════════════════════════════════════════════════════════════

    def _on_robotstate(self, msg: HighLevelRobotState):
        self._last_robotstate = msg
        self._last_robotstate_time = self.get_clock().now()

    def _on_mc_state(self, msg: McState):
        self._last_mc_state = msg
        self._last_mc_state_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry):
        self._last_odom = msg
        self._last_odom_time = self.get_clock().now()
        # 直接转发 odom
        self._odom_pub.publish(msg)

    # ════════════════════════════════════════════════════════════
    # 定时发布
    # ════════════════════════════════════════════════════════════

    def _publish_status(self):
        """1Hz 发布 /robot_state 和 /battery_state"""
        now = self.get_clock().now()

        # ── /robot_state ──
        robot_state_msg = String()
        if self._last_robotstate is not None and self._last_mc_state is not None:
            rs = self._last_robotstate
            mc = self._last_mc_state

            mc_mode_names = {
                0: 'STANDBY', 1: 'NAV_VEL_CTRL', 2: 'ARC_VEL_CTRL',
                3: 'POS_CTRL', 4: 'SU_CTRL', 5: 'CHARGING', 6: 'PASSIVE',
            }
            mode_name = mc_mode_names.get(mc.state, f'UNKNOWN({mc.state})')

            robot_state_msg.data = (
                f'pos=({rs.pos.x:.3f},{rs.pos.y:.3f},{rs.pos.z:.3f}) '
                f'rpy=({rs.rpy.x:.3f},{rs.rpy.y:.3f},{rs.rpy.z:.3f}) '
                f'mode={mode_name} error={mc.error_code}'
            )
        else:
            robot_state_msg.data = 'waiting_for_dog_state'

        self._robot_state_pub.publish(robot_state_msg)

        # ── /battery_state ──
        batt = BatteryState()
        batt.header.stamp = now.to_msg()
        batt.voltage = float('nan')
        batt.percentage = float('nan')
        batt.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        batt.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        batt.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        batt.present = True
        self._battery_pub.publish(batt)

    def _publish_diagnostics(self):
        """5Hz 诊断信息"""
        now = self.get_clock().now()
        diag = DiagnosticArray()
        diag.header.stamp = now.to_msg()

        # ── adapter 自身状态 ──
        status = DiagnosticStatus()
        status.name = 'zenoh_readonly_adapter'
        status.level = DiagnosticStatus.OK
        status.message = 'Running (read-only)'
        status.values = [
            KeyValue(key='phase', value='Z3'),
            KeyValue(key='dog_connected', value='true'),
        ]

        # ── 各 topic 延迟 ──
        if self._last_robotstate_time is not None:
            lag = (now - self._last_robotstate_time).nanoseconds / 1e9
            status.values.append(KeyValue(key='robotstate_lag_s', value=f'{lag:.2f}'))
        if self._last_mc_state_time is not None:
            lag = (now - self._last_mc_state_time).nanoseconds / 1e9
            status.values.append(KeyValue(key='mc_state_lag_s', value=f'{lag:.2f}'))
        if self._last_odom_time is not None:
            lag = (now - self._last_odom_time).nanoseconds / 1e9
            status.values.append(KeyValue(key='odom_lag_s', value=f'{lag:.2f}'))

        diag.status.append(status)
        self._diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = ZenohReadonlyAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Adapter crashed: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
