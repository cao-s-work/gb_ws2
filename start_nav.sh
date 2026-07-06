#!/bin/bash
# Phase 10.2.1 测试启动脚本
# 启动完整导航栈 (mock base + collision_monitor + safety) + gb_web_node
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws/install/setup.bash

echo "=== 启动导航栈 ==="
ros2 launch gb_bringup navigation.launch.py \
    enable_collision_monitor:=true \
    connect_base:=true \
    use_mock_base:=true &
NAV_PID=$!

echo "=== 等待导航栈就绪 (8s) ==="
sleep 8

echo "=== 启动 gb_web_node ==="
ros2 run gb_web gb_web_node &
WEB_PID=$!

echo "=== 等待 gb_web_node 就绪 (2s) ==="
sleep 2

echo "=== 系统启动完成 ==="
echo "NAV_PID=$NAV_PID, WEB_PID=$WEB_PID"
echo "Web: http://0.0.0.0:8080/"

# 等待子进程
wait
