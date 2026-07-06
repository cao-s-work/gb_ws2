#!/bin/bash
# phase2_retest.sh — 阶段2保守运动复测（安全版）
# 用法: bash phase2_retest.sh
set -e

source /opt/ros/humble/setup.bash
source ~/gb_ws/install/setup.bash

# 安全清理函数
cleanup_cmd() {
    pkill -f 'ros2 topic pub.*cmd_vel_web' 2>/dev/null || true
}

trap cleanup_cmd EXIT

echo "========================================="
echo "  阶段2 架空运动复测（保守参数）"
echo "  vx=0.03 m/s  |  wz=0.10 rad/s"
echo "  每动作 0.3s   |  30s 内完成"
echo "========================================="

# 0. 确保无残留发布
cleanup_cmd
sleep 0.5

# 1. standUp
echo ""
echo ">>> [1/7] standUp..."
ros2 service call /gb_base/stand_up std_srvs/srv/Trigger 2>&1 | grep -o 'success=[^,]*'
sleep 2
echo "     狗站立完成，进入 MOVE 模式"

# 2. 前进 0.3s
echo ""
echo ">>> [2/7] 前进 0.3s (vx=0.03)..."
ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_web geometry_msgs/msg/Twist "{linear: {x: 0.03}, angular: {z: 0.0}}" &
PID=$!
sleep 0.3
kill $PID 2>/dev/null
cleanup_cmd
sleep 0.3
echo "     前进完成 → 停止"

# 3. 后退 0.3s
echo ""
echo ">>> [3/7] 后退 0.3s (vx=-0.03)..."
ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_web geometry_msgs/msg/Twist "{linear: {x: -0.03}, angular: {z: 0.0}}" &
PID=$!
sleep 0.3
kill $PID 2>/dev/null
cleanup_cmd
sleep 0.3
echo "     后退完成 → 停止"

# 4. 左转 0.3s
echo ""
echo ">>> [4/7] 左转 0.3s (wz=0.10)..."
ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_web geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.10}}" &
PID=$!
sleep 0.3
kill $PID 2>/dev/null
cleanup_cmd
sleep 0.3
echo "     左转完成 → 停止"

# 5. 右转 0.3s
echo ""
echo ">>> [5/7] 右转 0.3s (wz=-0.10)..."
ros2 topic pub --rate 10 --qos-reliability best_effort /cmd_vel_web geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.10}}" &
PID=$!
sleep 0.3
kill $PID 2>/dev/null
cleanup_cmd
sleep 0.3
echo "     右转完成 → 停止"

# 6. lieDown
echo ""
echo ">>> [6/7] lieDown..."
ros2 service call /gb_base/lie_down std_srvs/srv/Trigger 2>&1 | grep -o 'success=[^,]*'
sleep 1
echo "     狗已趴下"

# 7. 最终清理
cleanup_cmd
echo ""
echo "========================================="
echo "  复测完成！"
echo "========================================="
