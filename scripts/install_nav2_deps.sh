#!/bin/bash
# ZSL-1W Nav2 依赖安装脚本
# 一次性补齐所有缺失的 Nav2 系统包
# 使用方法: bash install_nav2_deps.sh

set -e

echo "=== ZSL-1W Nav2 依赖安装 ==="

PACKAGES=(
    # Nav2 核心导航栈
    ros-humble-nav2-controller
    ros-humble-nav2-planner
    ros-humble-nav2-behaviors
    ros-humble-nav2-bt-navigator
    ros-humble-nav2-smoother
    ros-humble-nav2-velocity-smoother

    # Nav2 控制器/规划器插件
    ros-humble-nav2-regulated-pure-pursuit-controller
    ros-humble-nav2-navfn-planner

    # AMCL 定位（map→odom 动态变换）
    ros-humble-nav2-amcl

    # pointcloud_to_laserscan（3D 点云→2D 激光扫描）
    ros-humble-pointcloud-to-laserscan
)

echo "需要安装 ${#PACKAGES[@]} 个包..."
sudo apt-get update -qq
sudo apt-get install -y "${PACKAGES[@]}"

echo ""
echo "=== 安装完成 ==="
echo "已安装: ${PACKAGES[*]}"
echo ""
echo "验证:"
for pkg in "${PACKAGES[@]}"; do
    if dpkg -l "$pkg" &>/dev/null; then
        echo "  ✅ $pkg"
    else
        echo "  ❌ $pkg (缺失)"
    fi
done
