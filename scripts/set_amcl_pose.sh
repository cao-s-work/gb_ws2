#!/bin/bash
# ============================================================
# AMCL 初始位姿设置 — 开机后手动执行
# 用法: bash ~/gb_ws2/scripts/set_amcl_pose.sh
#
# 默认位姿 (map 帧):
#   position: (0.49, 0.72), yaw: -18° (≈ -0.314 rad)
#
# 可传参覆盖:
#   bash set_amcl_pose.sh <x> <y> <yaw_deg>
# ============================================================
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws2/install/setup.bash

X=${1:-0.49}
Y=${2:-0.72}
YAW_DEG=${3:--18}

# deg → rad → quat
YAW=$(python3 -c "import math; print(math.radians($YAW_DEG))")
QZ=$(python3 -c "import math; print(math.sin($YAW/2))")
QW=$(python3 -c "import math; print(math.cos($YAW/2))")

echo "设置 AMCL 初始位姿: x=$X, y=$Y, yaw=${YAW_DEG}°"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: map},
  pose: {
    pose: {
      position: {x: $X, y: $Y, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
  }
}"

sleep 3
echo ""
echo "AMCL 更新后位姿:"
ros2 topic echo /amcl_pose --once --field pose.pose.position 2>/dev/null
