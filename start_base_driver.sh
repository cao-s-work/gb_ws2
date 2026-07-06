#!/bin/bash
# GB钢镚 - 启动 base_driver_node (read_only 安全模式)
# 绕过 Hermes Python 3.11 环境，使用系统 Python 3.10

set -e

# 清掉 Hermes 的 Python 3.11 路径
unset PYTHONHOME
unset PYTHONPATH
unset VIRTUAL_ENV

# 只用系统路径
export PATH=/usr/bin:/bin:/opt/ros/humble/bin:/usr/local/bin
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/lib/aarch64-linux-gnu
export PYTHONPATH=/opt/ros/humble/local/lib/python3.10/dist-packages
export AMENT_PREFIX_PATH=/opt/ros/humble

# 加上 gb_ws 和 SDK
export PYTHONPATH=/home/nvidia/gb_ws/install/gb_base_driver/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/home/nvidia/gb_ws/install/gb_safety/lib/python3.10/site-packages:$PYTHONPATH
export PYTHONPATH=/home/nvidia/Desktop/gangbeng/genisom_l1_sdk-main/lib/zsl-1w/aarch64:$PYTHONPATH
export LD_LIBRARY_PATH=/home/nvidia/Desktop/gangbeng/genisom_l1_sdk-main/lib/zsl-1w/aarch64:$LD_LIBRARY_PATH

echo "[start] PYTHON=$(which python3) $(python3 --version)"
echo "[start] ROS_DISTRO=$ROS_DISTRO"

exec python3 /home/nvidia/gb_ws/install/gb_base_driver/lib/gb_base_driver/base_driver_node.py "$@"
