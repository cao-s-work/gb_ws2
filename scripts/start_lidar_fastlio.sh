#!/bin/bash
# ============================================================
# ZSL-1W LiDAR + FAST-LIO 自启动脚本
# 解决两个常挂问题：
#   1. LiDAR IP (192.168.1.216) 未配置
#   2. source gb_ws 遗漏
# ============================================================
set -e

echo "=== [$(date)] LiDAR + FAST-LIO 启动 ==="

# 1. 配 LiDAR 子网 IP（如果没配）
if ! ip addr show enP8p1s0 | grep -q "192.168.1.216"; then
    echo "配置 LiDAR IP 192.168.1.216/24..."
    echo "nvidia" | sudo -S ip addr add 192.168.1.216/24 dev enP8p1s0
fi

# 2. 验证 LiDAR 在线
echo -n "等待 LiDAR (192.168.1.142)..."
for i in $(seq 1 20); do
    if ping -c 1 -W 1 192.168.1.142 &>/dev/null; then
        echo " OK ($(($i))s)"
        break
    fi
    sleep 1
done

# 3. Source 环境
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH
unset PYTHONHOME VIRTUAL_ENV
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws/install/setup.bash

# 4. 杀旧进程
pkill -f livox_ros_driver2 2>/dev/null || true
pkill -f lio_node 2>/dev/null || true
sleep 1

# 5. 启动 LiDAR 驱动
echo "启动 LiDAR 驱动..."
ros2 launch gb_lio fastlio.launch.py use_gpu:=false &
LIO_PID=$!

# 6. 等待 FAST-LIO 就绪
echo "等待 /cloud_body..."
for i in $(seq 1 30); do
    if timeout 2 ros2 topic echo /cloud_body --once --field header &>/dev/null; then
        echo "/cloud_body 就绪 ($(($i * 2))s)"
        break
    fi
    sleep 2
done

echo "=== LiDAR + FAST-LIO 启动完成 ==="
wait $LIO_PID
