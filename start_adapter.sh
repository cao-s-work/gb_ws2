#!/bin/bash
# ZSL-1W 钢镚机器人 adapter 启动脚本 (nohup 版)
# 用法: bash ~/gb_ws/start_adapter.sh [true|false]
#   true  → read_only 模式 (默认, 安全)
#   false → 允许运动 (危险! 需授权)

set -e

READ_ONLY="${1:-true}"
READ_ONLY="${READ_ONLY#read_only:=}"

LOG_DIR="/home/nvidia/gb_ws/logs"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/adapter_$(date +%Y%m%d_%H%M%S).log"

# ---- 0. SDK 库路径 (genisom ZSL-1W SDK .so) ----
export SDK_LIB=/home/nvidia/gb_ws/sdk/genisom_l1_sdk-main/lib/zsl-1w/aarch64
export PYTHONPATH=$SDK_LIB:$PYTHONPATH
export LD_LIBRARY_PATH=$SDK_LIB:$LD_LIBRARY_PATH

# ---- 0b. 修复 PATH: 系统 python3.10 必须在最前面 (Hermes venv 有 3.11) ----
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:$PATH

# ---- 1. 确保 ros2 daemon 使用 python3.10 ----
DAEMON_PY=$(ps aux | grep "ros2.*daemon" | grep -v grep | awk '{print $11}' | head -1)
if [ "$DAEMON_PY" != "/usr/bin/python3.10" ]; then
    echo "🔄 重启 ros2 daemon (强制 python3.10)..."
    /usr/bin/python3.10 /opt/ros/humble/bin/ros2 daemon stop 2>/dev/null || true
    sleep 1
    /usr/bin/python3.10 /opt/ros/humble/bin/ros2 daemon start
    sleep 1
fi

# ---- 2. 加载 ROS 环境 ----
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws/install/setup.bash

# ---- 3. 检查网络 ----
echo "🔍 检查狗连接..."
if ping -c 1 -W 1 192.168.168.168 > /dev/null 2>&1; then
    echo "✅ 狗可达 (192.168.168.168)"
else
    echo "❌ 狗不可达! 请检查网络"
    exit 1
fi

# ---- 4. 启动 adapter (nohup 后台) ----
echo "🚀 启动 adapter (read_only=$READ_ONLY)..."
echo "📄 日志: $LOG_FILE"

nohup ros2 launch gb_base_driver real_adapter.launch.py read_only:=${READ_ONLY} > "$LOG_FILE" 2>&1 &
ADP_PID=$!
echo "✅ Adapter PID=$ADP_PID"

# ---- 5. 等待连接确认 ----
sleep 5
if grep -q "✅ SDK 连接成功" "$LOG_FILE" 2>/dev/null; then
    echo "🎉 SDK 已连接!"
    echo "🌐 Web: http://192.168.144.128:8080"
elif grep -q "❌ SDK 初始化失败" "$LOG_FILE" 2>/dev/null; then
    echo "⚠️ SDK 初始化失败 — 查看日志: tail -f $LOG_FILE"
else
    echo "⏳ 等待连接中... 查看日志: tail -f $LOG_FILE"
fi
