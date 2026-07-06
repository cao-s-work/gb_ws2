#!/usr/bin/env python3
"""
Phase 2 架空测试脚本

测试项目:
1. standUp() — 站立
2. 前进 0.05 m/s × 0.5s → 停止 (验证方向+限幅+归零)
3. 后退 -0.05 m/s × 0.5s → 停止
4. 左转 0.15 rad/s × 0.5s → 停止
5. 右转 -0.15 rad/s × 0.5s → 停止
6. 急停测试: 运动中急停 → 验证停止 → 急停中遥控不动 → reset_estop → 无残留速度
7. 超时停车验证
8. Web 断连停车验证

每个动作后确认 /cmd_vel_base 归零。
加速度限制器是安全行为, 测试验证方向正确+速度在限幅内+归零。
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import time
import json


class Phase2Test(Node):
    def __init__(self):
        super().__init__('phase2_test')

        # 使用 BEST_EFFORT QoS 匹配 safety_node 订阅
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        # /cmd_vel_web 发布用 BEST_EFFORT 匹配 safety_node 订阅 QoS
        sensor_qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_web', sensor_qos_pub)
        self._pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)

        self._cmd_vel_base = Twist()
        # /cmd_vel_base 由 safety_node 发布 (RELIABLE), 用 RELIABLE 订阅匹配
        self._sub_base = self.create_subscription(
            Twist, '/cmd_vel_base', self._on_cmd_vel_base, 10)

        self._robot_state = {}
        self._sub_state = self.create_subscription(
            String, '/robot_state', self._on_robot_state, 10)

        self._cli_standup = self.create_client(Trigger, '/gb_base/stand_up')
        self._cli_liedown = self.create_client(Trigger, '/gb_base/lie_down')
        self._cli_passive = self.create_client(Trigger, '/gb_base/passive')
        self._cli_reset_estop = self.create_client(Trigger, '/gb_safety/reset_estop')

        self.get_logger().info('Phase 2 测试节点已启动, 等待 DDS 发现...')
        time.sleep(2)
        # 预热: 发送零速度让 safety_node 发现 publisher
        warmup = Twist()
        for _ in range(20):
            self._pub_cmd.publish(warmup)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info('DDS 预热完成')

    def _on_cmd_vel_base(self, msg):
        self._cmd_vel_base = msg

    def _on_robot_state(self, msg):
        try:
            self._robot_state = json.loads(msg.data)
        except:
            pass

    def _send_cmd(self, vx, wz, duration_s):
        """发送速度指令并持续 duration_s 秒, 然后归零。
        返回 (峰值vx, 峰值wz, 归零vx, 归零wz)"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)

        self.get_logger().info(f'  发送: vx={vx}, wz={wz}, 持续={duration_s}s')
        start = time.time()
        peak_vx = 0.0
        peak_wz = 0.0
        while time.time() - start < duration_s:
            self._pub_cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
            base_vx = self._cmd_vel_base.linear.x
            base_wz = self._cmd_vel_base.angular.z
            if abs(base_vx) > abs(peak_vx):
                peak_vx = base_vx
            if abs(base_wz) > abs(peak_wz):
                peak_wz = base_wz

        # 归零
        self._pub_cmd.publish(Twist())
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        zero_vx = self._cmd_vel_base.linear.x
        zero_wz = self._cmd_vel_base.angular.z

        self.get_logger().info(
            f'  峰值 /cmd_vel_base: vx={peak_vx:.4f}, wz={peak_wz:.4f}')
        self.get_logger().info(
            f'  归零后 /cmd_vel_base: vx={zero_vx:.4f}, wz={zero_wz:.4f}')

        return peak_vx, peak_wz, zero_vx, zero_wz

    def _stop_cmd(self):
        """发送零速度"""
        self._pub_cmd.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)
        base_vx = self._cmd_vel_base.linear.x
        base_wz = self._cmd_vel_base.angular.z
        self.get_logger().info(
            f'  停止后 /cmd_vel_base: vx={base_vx:.4f}, wz={base_wz:.4f}')
        return base_vx, base_wz

    def _call_service(self, client, name):
        """调用服务并等待结果"""
        if not client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(f'  服务 {name} 不可用')
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            self.get_logger().info(
                f'  {name}: success={future.result().success}, '
                f'msg={future.result().message}')
            return future.result()
        else:
            self.get_logger().error(f'  {name} 调用失败')
            return None

    def _send_estop(self, estop: bool):
        """发送急停/解除"""
        msg = Bool()
        msg.data = estop
        self._pub_estop.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.3)

    def _wait_robot_state(self, timeout_s=5.0):
        """等待收到 robot_state"""
        start = time.time()
        while time.time() - start < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self._robot_state:
                return True
            time.sleep(0.2)
        return False

    def run_tests(self):
        results = []

        # ===== 测试 0: 确认初始状态 =====
        self.get_logger().info('\n=== 测试 0: 确认初始状态 ===')
        self._wait_robot_state()
        mode = self._robot_state.get('mode', 'UNKNOWN')
        mode_id = self._robot_state.get('mode_id', -1)
        sdk = self._robot_state.get('sdk_connected', False)
        bat = self._robot_state.get('battery_pct', -1)
        self.get_logger().info(f'  模式={mode}(id={mode_id}), SDK连接={sdk}, 电池={bat}%')
        if not sdk:
            self.get_logger().error('  ❌ SDK 未连接, 中止测试')
            results.append({'test': '初始状态', 'pass': False, 'detail': 'SDK未连接'})
            return results
        results.append({'test': '初始状态', 'pass': True, 'detail': f'mode={mode}, bat={bat}%'})

        # ===== 测试 1: standUp() =====
        self.get_logger().info('\n=== 测试 1: standUp() ===')
        resp = self._call_service(self._cli_standup, 'stand_up')
        if resp and resp.success:
            self.get_logger().info('  等待站立完成 (5s)...')
            time.sleep(5.0)
            for _ in range(5):
                rclpy.spin_once(self, timeout_sec=0.3)
                time.sleep(0.2)
            mode = self._robot_state.get('mode', 'UNKNOWN')
            mode_id = self._robot_state.get('mode_id', -1)
            self.get_logger().info(f'  当前模式: {mode} (id={mode_id})')
            passed = mode_id not in (0, -1)
            results.append({'test': 'standUp()', 'pass': passed,
                            'detail': f'mode={mode}(id={mode_id})'})
            if not passed:
                self.get_logger().warn(f'  ⚠️ 模式仍是 {mode}, 但继续运动链路测试')
        else:
            results.append({'test': 'standUp()', 'pass': False, 'detail': '服务调用失败'})
            self.get_logger().error('  ❌ standUp 失败, 中止后续运动测试')
            return results

        # ===== 测试 2: 前进 0.05 m/s × 0.5s =====
        self.get_logger().info('\n=== 测试 2: 前进 0.05 m/s × 0.5s ===')
        peak_vx, peak_wz, zero_vx, zero_wz = self._send_cmd(0.05, 0.0, 0.5)
        dir_ok = peak_vx > 0.001  # 方向正确 (正)
        limit_ok = abs(peak_vx) <= 0.05 + 0.001  # 在限幅内
        zero_ok = abs(zero_vx) < 0.001 and abs(zero_wz) < 0.001  # 归零
        passed = dir_ok and limit_ok and zero_ok
        results.append({'test': '前进 0.05', 'pass': passed,
                        'detail': f'peak=({peak_vx:.4f},{peak_wz:.4f}) zero=({zero_vx:.4f},{zero_wz:.4f}) dir={dir_ok} limit={limit_ok} zero={zero_ok}'})

        # ===== 测试 3: 后退 -0.05 m/s × 0.5s =====
        self.get_logger().info('\n=== 测试 3: 后退 -0.05 m/s × 0.5s ===')
        peak_vx, peak_wz, zero_vx, zero_wz = self._send_cmd(-0.05, 0.0, 0.5)
        dir_ok = peak_vx < -0.001  # 方向正确 (负)
        limit_ok = abs(peak_vx) <= 0.05 + 0.001
        zero_ok = abs(zero_vx) < 0.001 and abs(zero_wz) < 0.001
        passed = dir_ok and limit_ok and zero_ok
        results.append({'test': '后退 -0.05', 'pass': passed,
                        'detail': f'peak=({peak_vx:.4f},{peak_wz:.4f}) zero=({zero_vx:.4f},{zero_wz:.4f}) dir={dir_ok} limit={limit_ok} zero={zero_ok}'})

        # ===== 测试 4: 左转 0.15 rad/s × 0.5s =====
        self.get_logger().info('\n=== 测试 4: 左转 0.15 rad/s × 0.5s ===')
        peak_vx, peak_wz, zero_vx, zero_wz = self._send_cmd(0.0, 0.15, 0.5)
        dir_ok = peak_wz > 0.001
        limit_ok = abs(peak_wz) <= 0.15 + 0.001
        zero_ok = abs(zero_vx) < 0.001 and abs(zero_wz) < 0.001
        passed = dir_ok and limit_ok and zero_ok
        results.append({'test': '左转 0.15', 'pass': passed,
                        'detail': f'peak=({peak_vx:.4f},{peak_wz:.4f}) zero=({zero_vx:.4f},{zero_wz:.4f}) dir={dir_ok} limit={limit_ok} zero={zero_ok}'})

        # ===== 测试 5: 右转 -0.15 rad/s × 0.5s =====
        self.get_logger().info('\n=== 测试 5: 右转 -0.15 rad/s × 0.5s ===')
        peak_vx, peak_wz, zero_vx, zero_wz = self._send_cmd(0.0, -0.15, 0.5)
        dir_ok = peak_wz < -0.001
        limit_ok = abs(peak_wz) <= 0.15 + 0.001
        zero_ok = abs(zero_vx) < 0.001 and abs(zero_wz) < 0.001
        passed = dir_ok and limit_ok and zero_ok
        results.append({'test': '右转 -0.15', 'pass': passed,
                        'detail': f'peak=({peak_vx:.4f},{peak_wz:.4f}) zero=({zero_vx:.4f},{zero_wz:.4f}) dir={dir_ok} limit={limit_ok} zero={zero_ok}'})

        # ===== 测试 6: 停止后归零验证 =====
        self.get_logger().info('\n=== 测试 6: 停止后 /cmd_vel_base 归零 ===')
        vx, wz = self._stop_cmd()
        passed = abs(vx) < 0.001 and abs(wz) < 0.001
        results.append({'test': '停止归零', 'pass': passed,
                        'detail': f'cmd_vel_base=({vx:.4f},{wz:.4f})'})

        # ===== 测试 7: 急停测试 =====
        self.get_logger().info('\n=== 测试 7: 急停测试 ===')
        self.get_logger().info('  7a: 发送急停...')
        self._send_estop(True)
        rclpy.spin_once(self, timeout_sec=0.2)
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.2)
        estop_state = self._robot_state.get('estop', False)
        self.get_logger().info(f'  急停状态: {estop_state}')

        # 急停中继续发送遥控, 底盘不能动
        self.get_logger().info('  7b: 急停中发送前进指令, 底盘不应动...')
        peak_vx, peak_wz, _, _ = self._send_cmd(0.05, 0.0, 0.5)
        passed_estop_block = abs(peak_vx) < 0.001 and abs(peak_wz) < 0.001
        self.get_logger().info(
            f'  急停中 peak cmd_vel_base=({peak_vx:.4f},{peak_wz:.4f}), '
            f'阻断={passed_estop_block}')
        results.append({'test': '急停阻断遥控', 'pass': passed_estop_block,
                        'detail': f'peak=({peak_vx:.4f},{peak_wz:.4f})'})

        # reset_estop
        self.get_logger().info('  7c: reset_estop...')
        self._send_estop(False)
        time.sleep(0.3)
        resp = self._call_service(self._cli_reset_estop, 'reset_estop')
        time.sleep(0.5)

        # 验证 reset 后无残留速度
        self.get_logger().info('  7d: 验证 reset 后无残留速度...')
        vx, wz = self._stop_cmd()
        passed_no_residual = abs(vx) < 0.001 and abs(wz) < 0.001
        self.get_logger().info(
            f'  reset 后 cmd_vel_base=({vx:.4f},{wz:.4f}), '
            f'无残留={passed_no_residual}')
        results.append({'test': 'reset无残留', 'pass': passed_no_residual,
                        'detail': f'cmd_vel_base=({vx:.4f},{wz:.4f})'})

        # ===== 测试 8: 超时停车验证 =====
        self.get_logger().info('\n=== 测试 8: 超时停车 (发送 0.3s 后停止发送) ===')
        msg = Twist()
        msg.linear.x = 0.05
        self._pub_cmd.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.3)
        self.get_logger().info('  停止发送, 等待 1.0s 超时...')
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.2)
        vx = self._cmd_vel_base.linear.x
        wz = self._cmd_vel_base.angular.z
        passed_timeout = abs(vx) < 0.001 and abs(wz) < 0.001
        self.get_logger().info(
            f'  超时后 cmd_vel_base=({vx:.4f},{wz:.4f}), '
            f'停车={passed_timeout}')
        results.append({'test': '超时停车', 'pass': passed_timeout,
                        'detail': f'cmd_vel_base=({vx:.4f},{wz:.4f})'})

        # ===== 测试 9: Web 断连停车 (停止发布 1s 后检查) =====
        self.get_logger().info('\n=== 测试 9: Web 断连停车 ===')
        # 先发送一个指令
        msg = Twist()
        msg.linear.x = 0.05
        for _ in range(5):
            self._pub_cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        # 停止发布 (模拟 Web 断连)
        self.get_logger().info('  模拟 Web 断连, 停止发布 1.5s...')
        time.sleep(1.5)
        rclpy.spin_once(self, timeout_sec=0.2)
        vx = self._cmd_vel_base.linear.x
        wz = self._cmd_vel_base.angular.z
        passed_disconnect = abs(vx) < 0.001 and abs(wz) < 0.001
        self.get_logger().info(
            f'  断连后 cmd_vel_base=({vx:.4f},{wz:.4f}), '
            f'停车={passed_disconnect}')
        results.append({'test': 'Web断连停车', 'pass': passed_disconnect,
                        'detail': f'cmd_vel_base=({vx:.4f},{wz:.4f})'})

        # ===== 清理: lieDown =====
        self.get_logger().info('\n=== 清理: lieDown() ===')
        resp = self._call_service(self._cli_liedown, 'lie_down')
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        mode = self._robot_state.get('mode', 'UNKNOWN')
        self.get_logger().info(f'  最终模式: {mode}')

        # ===== 汇总 =====
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('Phase 2 架空测试结果汇总')
        self.get_logger().info('=' * 60)
        all_pass = True
        for r in results:
            status = '✅ PASS' if r['pass'] else '❌ FAIL'
            self.get_logger().info(f'  {status} | {r["test"]}: {r["detail"]}')
            if not r['pass']:
                all_pass = False
        self.get_logger().info('=' * 60)
        total = len(results)
        passed = sum(1 for r in results if r['pass'])
        self.get_logger().info(f'总计: {passed}/{total} 通过')
        if all_pass:
            self.get_logger().info('🎉 Phase 2 架空测试全部通过!')
        else:
            self.get_logger().warn('⚠️ 部分测试未通过, 请检查')
        self.get_logger().info('=' * 60)

        return results


def main():
    rclpy.init()
    node = Phase2Test()
    try:
        node.run_tests()
    except KeyboardInterrupt:
        node.get_logger().info('测试中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
