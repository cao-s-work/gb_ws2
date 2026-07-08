#!/bin/bash
# 快速重启 AMCL（调参用）
# 用法: bash restart_amcl.sh [x] [y] [yaw_rad]
# 默认: x=0.49 y=0.72 yaw=-0.314

set -e
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws2/install/setup.bash

X="${1:-0.49}"
Y="${2:-0.72}"
YAW="${3:--0.314}"

AMCL_PARAMS="/home/nvidia/gb_ws2/src/gb_nav2/config/amcl_params.yaml"

# 1. 杀旧 AMCL
pkill -f "nav2_amcl/amcl" 2>/dev/null || true
sleep 1

# 2. 启动 AMCL
ros2 run nav2_amcl amcl \
    --ros-args \
    -r __node:=amcl \
    -r scan:=/scan \
    --params-file "$AMCL_PARAMS" \
    -p use_sim_time:=false \
    > /tmp/amcl_restart.log 2>&1 &
AMCL_PID=$!
echo "AMCL PID=$AMCL_PID"

# 3. 等节点就绪
for i in $(seq 1 10); do
    ros2 node list 2>/dev/null | grep -q "/amcl" && break
    sleep 1
done
sleep 2

# 4. lifecycle
timeout 20 ros2 lifecycle set /amcl configure 2>/dev/null && sleep 1
timeout 20 ros2 lifecycle set /amcl activate 2>/dev/null && sleep 1

# 5. 发 initialpose (Python rclpy)
GB_INIT_X="$X" GB_INIT_Y="$Y" GB_INIT_YAW="$YAW" /usr/bin/python3 - <<'PY'
import os, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('gb_initialpose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    def make_msg(self):
        x = float(os.environ.get('GB_INIT_X', '0.49'))
        y = float(os.environ.get('GB_INIT_Y', '0.72'))
        yaw = float(os.environ.get('GB_INIT_YAW', '-0.314'))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.068
        msg.pose.covariance = cov
        return msg
def main():
    rclpy.init()
    node = InitialPosePublisher()
    for _ in range(30):
        if node.pub.get_subscription_count() > 0: break
        rclpy.spin_once(node, timeout_sec=0.1)
    for i in range(5):
        msg = node.make_msg()
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.25)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__': main()
PY

# 6. 等 TF
for i in $(seq 1 15); do
    if timeout 2 ros2 run tf2_ros tf2_echo map camera_init 2>&1 | grep -q "Translation"; then
        echo "✅ AMCL 重启完成，map→camera_init TF 就绪"
        exit 0
    fi
    sleep 1
done
echo "⚠️ TF 未就绪，但 AMCL 已启动"
