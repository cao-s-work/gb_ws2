#!/bin/bash
# ============================================================
# Phase 2: 真实底盘架空测试启动脚本
# 
# 限速: max_linear_x=0.05, max_angular_z=0.15
# 模式: read_only=false (允许运动)
# 禁止: Nav2 goal, 自动导航
# ============================================================

source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws/install/setup.bash

echo "=== Phase 2: 真实底盘架空测试 ==="
echo "⚠️  限速: 0.05 m/s / 0.15 rad/s"
echo "⚠️  禁止 Nav2 goal / 自动导航"
echo ""

# 1. 启动导航栈 (FAST-LIO + Nav2 + collision_monitor + safety)
#    use_mock_base=false (不启动 mock)
#    connect_base=true → publish_base_cmd=true
#    但 safety_node 需要 allow_real_base=true
echo "=== 启动导航栈 + safety_node ==="
ros2 launch gb_bringup navigation.launch.py \
    enable_collision_monitor:=true \
    connect_base:=true \
    use_mock_base:=false \
    allow_real_base:=true \
    require_odom:=false \
    require_points:=false &
NAV_PID=$!

echo "=== 等待导航栈就绪 (8s) ==="
sleep 8

# 2. 动态设置 safety_node 的 allow_real_base=true
echo "=== 设置 safety_node allow_real_base=true ==="
ros2 param set /safety_node allow_real_base true 2>/dev/null || true
ros2 param set /safety_node max_linear_x 0.05 2>/dev/null || true
ros2 param set /safety_node min_linear_x -0.05 2>/dev/null || true
ros2 param set /safety_node max_angular_z 0.15 2>/dev/null || true
ros2 param set /safety_node max_linear_accel 0.10 2>/dev/null || true
ros2 param set /safety_node max_angular_accel 0.30 2>/dev/null || true

# 3. 启动真实底盘 adapter (read_only=false)
echo "=== 启动真实底盘 adapter (read_only=false) ==="
ros2 launch gb_base_driver real_adapter.launch.py \
    read_only:=false \
    max_linear_speed:=0.05 \
    max_angular_speed:=0.15 &
ADAPTER_PID=$!

echo "=== 等待 adapter 就绪 (3s) ==="
sleep 3

# 4. 启动 gb_web_node
echo "=== 启动 gb_web_node ==="
ros2 run gb_web gb_web_node &
WEB_PID=$!

echo "=== 等待 gb_web_node 就绪 (2s) ==="
sleep 2

echo ""
echo "=== Phase 2 系统启动完成 ==="
echo "NAV_PID=$NAV_PID, ADAPTER_PID=$ADAPTER_PID, WEB_PID=$WEB_PID"
echo "Web: http://0.0.0.0:8080/"
echo ""
echo "服务:"
echo "  站立: ros2 service call /gb_base/stand_up std_srvs/srv/Trigger"
echo "  趴下: ros2 service call /gb_base/lie_down std_srvs/srv/Trigger"
echo "  阻尼: ros2 service call /gb_base/passive std_srvs/srv/Trigger"
echo ""

# 等待子进程
wait
