#!/bin/bash
# ============================================================
# GB钢镚 — 全链路开机自启动脚本
# 文件: /home/nvidia/gb_ws2/scripts/gb_full_chain.sh
#
# 启动顺序:
#   1. LiDAR + FAST-LIO           → /cloud_body, /Odometry
#   2. 静态 TF                     → livox_imu_link→base_link→lidar_link→livox_frame
#   3. Perception (点云滤波)       → /points_nav
#   4. Odometry 2D 投影            → /Odometry_2d (供 Nav2 使用)
#   5. Nav2 导航栈                 → /cmd_vel_nav, /map
#   6. 定位 (cloud→scan + AMCL)    → map→camera_init 变换
#   7. 碰撞监控                    → /cmd_vel_collision
#   8. 安全闸门                    → /cmd_vel_base
#   9. Web 遥控
#  10. 底盘适配器                  → SDK → 狗
# ============================================================
set -e

# ==== 配置 ====
MAP_FILE="/home/nvidia/gb_maps/20260704_gb_pointfoot_v2/gb_map.yaml"
COLLISION_CONFIG="/home/nvidia/gb_ws2/src/gb_nav2/config/collision_monitor_93.yaml"
AMCL_PARAMS="/home/nvidia/gb_ws2/src/gb_nav2/config/amcl_params.yaml"
NAV2_PARAMS="/home/nvidia/gb_ws2/src/gb_bringup/config/nav2_params.yaml"
ROS_DOMAIN_ID=0

# ==== 环境 ====
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

# 确保 ros2 daemon 运行（否则 lifecycle/topic list 会 hang）
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null
sleep 1

LOG_DIR="/home/nvidia/gb_ws2/logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG="$LOG_DIR/chain_${TIMESTAMP}.log"

# ==== 清理函数 ====
cleanup() {
    echo "[$(date)] ⚠️ 启动脚本被中断，正在清理..."
    kill $(jobs -p) 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ==== 工具函数 ====
log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"; }
wait_for_topic() {
    local topic=$1 timeout=${2:-30}
    log "等待话题 $topic ..."
    for i in $(seq 1 $timeout); do
        if ros2 topic list 2>/dev/null | grep -q "$topic"; then
            log "✅ $topic 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done
    log "❌ $topic 超时 (${timeout}s)"
    return 1
}
wait_for_tf() {
    local from=$1 to=$2 timeout=${3:-30}
    log "等待 TF: $from → $to ..."
    for i in $(seq 1 $timeout); do
        if ros2 run tf2_ros tf2_echo "$from" "$to" 2>&1 | timeout 2 grep -q "Translation" 2>/dev/null; then
            log "✅ TF $from→$to 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done
    log "❌ TF $from→$to 超时 (${timeout}s)"
    return 1
}

log "══════════════════════════════════════"
log "GB钢镚 全链路自启动"
log "══════════════════════════════════════"

# ============================================================
# 1. LiDAR 驱动 + FAST-LIO
# ============================================================
log ""
log "━━━ 第1步: LiDAR + FAST-LIO ━━━"
ros2 launch gb_lio fastlio.launch.py > "$LOG_DIR/lio_${TIMESTAMP}.log" 2>&1 &
LIO_PID=$!
log "  PID=$LIO_PID"

wait_for_topic "/cloud_body" 30 || {
    log "❌ /cloud_body 话题未出现，终止"
    exit 1
}

# 额外验证：确认 /cloud_body 有实际数据（而不只是话题存在）
log "  确认 /cloud_body 数据流..."
for i in $(seq 1 10); do
    if timeout 3 ros2 topic hz /cloud_body 2>/dev/null | grep -q "average rate"; then
        log "  ✅ /cloud_body 数据流正常"
        break
    fi
    if [ $i -eq 10 ]; then
        log "❌ /cloud_body 无数据，LiDAR 驱动可能绑定失败（检查 IP 配置和硬件连接）"
        exit 1
    fi
    sleep 1
done

# ============================================================
# 2. 静态 TF（完整链: livox_imu_link→base_link→lidar_link→livox_frame）
# FAST-LIO 发布 camera_init→livox_imu_link，静态 TF 补完到 base_link
# 必须在 perception 之前启动，否则 points_filter_node TF 失败会丢帧
# ============================================================
log ""
log "━━━ 第2步: 静态 TF ━━━"

# livox_imu_link → base_link (去除雷达斜装 pitch，使 base_link 保持水平)
ros2 run tf2_ros static_transform_publisher \
    -0.119 0 -0.135 0 -0.261799 0 livox_imu_link base_link \
    --ros-args -r __node:=tf_livoximu_to_base \
    > "$LOG_DIR/tf_livoximu_base_${TIMESTAMP}.log" 2>&1 &
TF_IMU_BASE_PID=$!

# base_link → lidar_link (外参: 0.15, 0, 0.10, pitch=15°)
ros2 run tf2_ros static_transform_publisher \
    0.15 0 0.10 0 0.261799 0 base_link lidar_link \
    --ros-args -r __node:=tf_base_to_lidarlink \
    > "$LOG_DIR/tf_base_lidar_${TIMESTAMP}.log" 2>&1 &
TF_BASE_LIDAR_PID=$!

# lidar_link → livox_frame (恒等)
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 lidar_link livox_frame \
    --ros-args -r __node:=tf_lidar_to_livox \
    > "$LOG_DIR/tf_lidar_livox_${TIMESTAMP}.log" 2>&1 &
TF_LIDAR_LIVOX_PID=$!

log "  TF PIDs: livoximu→base=$TF_IMU_BASE_PID, base→lidar=$TF_BASE_LIDAR_PID, lidar→livox=$TF_LIDAR_LIVOX_PID"

# ============================================================
# 3. Perception 点云滤波 (依赖静态 TF: livox_imu_link→base_link)
# ============================================================
log ""
log "━━━ 第3步: Perception 点云滤波 ━━━"
ros2 launch gb_perception perception.launch.py \
    input_topic:=/cloud_body \
    output_topic:=/points_nav \
    > "$LOG_DIR/perception_${TIMESTAMP}.log" 2>&1 &
PERCEP_PID=$!
log "  PID=$PERCEP_PID"

wait_for_topic "/points_nav" 15 || {
    log "⚠️ /points_nav 未就绪，继续（可能延迟）"
}

# ============================================================
# 4. Odometry 2D 投影 (/Odometry → /Odometry_2d, camera_init→base_link)
# 必须在 Nav2 之前，因为 Nav2 使用 /Odometry_2d
# ============================================================
log ""
log "━━━ 第4步: Odometry 2D 投影 ━━━"
ros2 launch gb_lio odom_2d.launch.py \
    > "$LOG_DIR/odom2d_${TIMESTAMP}.log" 2>&1 &
ODOM2D_PID=$!
log "  PID=$ODOM2D_PID"

wait_for_topic "/Odometry_2d" 20 || {
    log "⚠️ /Odometry_2d 未就绪"
}

# ============================================================
# 5. Nav2 导航栈 (含 map_server，提供 /map，使用 /Odometry_2d)
# ============================================================
log ""
log "━━━ 第5步: Nav2 导航栈 ━━━"
ros2 launch gb_bringup nav2_minimal.launch.py \
    params_file:="$NAV2_PARAMS" \
    use_sim_time:=false \
    autostart:=true \
    > "$LOG_DIR/nav2_${TIMESTAMP}.log" 2>&1 &
NAV2_PID=$!
log "  PID=$NAV2_PID"

wait_for_topic "/map" 30 || {
    log "❌ /map 未就绪，终止"
    exit 1
}

# ============================================================
# 6. 定位: pointcloud_to_laserscan + AMCL
#    (不使用 localization.launch.py 以避免 map_server 重复)
# ============================================================
log ""
log "━━━ 第6步: 定位 ━━━"

# 5a. pointcloud_to_laserscan: /cloud_body → /scan
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r __node:=cloud_to_scan \
    -r cloud_in:=/cloud_body \
    -r scan:=/scan \
    -p target_frame:=base_link \
    -p transform_tolerance:=0.1 \
    -p min_height:=0.05 \
    -p max_height:=0.60 \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.0087 \
    -p scan_time:=0.1 \
    -p range_min:=0.35 \
    -p range_max:=8.0 \
    -p use_inf:=true \
    -p inf_epsilon:=1.0 \
    -p concurrency_level:=1 \
    > "$LOG_DIR/scan_${TIMESTAMP}.log" 2>&1 &
SCAN_PID=$!

# 5b. AMCL: /scan + /map → map→camera_init
ros2 run nav2_amcl amcl \
    --ros-args \
    -r __node:=amcl \
    -r scan:=/scan \
    --params-file "$AMCL_PARAMS" \
    -p use_sim_time:=false \
    > "$LOG_DIR/amcl_${TIMESTAMP}.log" 2>&1 &
AMCL_PID=$!
log "  PIDs: scan=$SCAN_PID, amcl=$AMCL_PID"

# 5c. AMCL lifecycle 手动激活（因为不在 nav2_minimal 的 lifecycle_manager 管理下）
log "  等待 AMCL 节点就绪..."
for i in $(seq 1 10); do
    if ros2 node list 2>/dev/null | grep -q "/amcl"; then
        log "  AMCL 节点已就绪"
        break
    fi
    sleep 1
done
sleep 2
log "  configure AMCL..."
timeout 5 ros2 lifecycle set /amcl configure 2>/dev/null && sleep 1 || log "  ⚠️ configure 超时或失败"
log "  activate AMCL..."
timeout 5 ros2 lifecycle set /amcl activate 2>/dev/null && sleep 1 || log "  ⚠️ activate 超时或失败"

# 自动设置初始位姿 (set_initial_pose=false 时需要)
# 仅当 GB_AUTO_INITIALPOSE=1 时才自动发布 /initialpose
if [ "${GB_AUTO_INITIALPOSE:-0}" = "1" ]; then
    DEFAULT_X="${GB_INIT_X:-0.49}"
    DEFAULT_Y="${GB_INIT_Y:-0.72}"
    YAW_RAD="${GB_INIT_YAW:--0.314}"
    log "  设置 AMCL 初始位姿: x=${DEFAULT_X}, y=${DEFAULT_Y}, yaw_rad=${YAW_RAD}"
    for i in 1 2 3; do
        ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {stamp: {sec:0, nanosec:0}, frame_id: map}, \
          pose: {pose: {position: {x: ${DEFAULT_X}, y: ${DEFAULT_Y}, z: 0.0}, \
                 orientation: {x: 0.0, y: 0.0, z: -0.1564, w: 0.9877}}, \
                 covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.068]}}" \
        2>/dev/null && sleep 0.3
    done
    log "  ✅ 已发布 /initialpose x3"
else
    log "  跳过自动 initialpose，请使用 Web/RViz 重定位"
fi

wait_for_tf "map" "camera_init" 30 || {
    log "⚠️ TF map→camera_init 未就绪，AMCL 可能还在初始化"
}
# 额外等待 AMCL 粒子收敛
sleep 5
log "✅ 定位栈就绪"

# ============================================================
# 7. 碰撞监控
# ============================================================
log ""
log "━━━ 第7步: 碰撞监控 ━━━"
ros2 launch gb_nav2 collision_monitor.launch.py \
    params_file:="$COLLISION_CONFIG" \
    use_sim_time:=false \
    autostart:=true \
    > "$LOG_DIR/collision_${TIMESTAMP}.log" 2>&1 &
COLL_PID=$!
log "  PID=$COLL_PID"

wait_for_topic "/cmd_vel_collision" 20 || {
    log "⚠️ /cmd_vel_collision 未就绪，碰撞监控可能未完全启动"
}

# ============================================================
# 8. 安全闸门 (先清残留，避免双发布者 Ghost)
# ============================================================
log ""
log "━━━ 第8步: 安全闸门 ━━━"
log "  清理旧 safety_node..."
pkill -f "gb_safety.*safety_node" 2>/dev/null || true
pkill -f "safety_node" 2>/dev/null || true
sleep 1
ros2 run gb_safety safety_node \
    --ros-args \
    -r __node:=safety_node \
    -p input_cmd_topic:=/cmd_vel_collision \
    -p output_cmd_topic:=/cmd_vel_base \
    -p web_cmd_topic:=/cmd_vel_web \
    -p require_odom:=false \
    -p require_points:=false \
    -p require_battery:=false \
    -p publish_base_cmd:=true \
    -p allow_real_base:=true \
    -p use_mock_base:=false \
    -p estop_latched:=false \
    -p cmd_timeout_sec:=5.0 \
    > "$LOG_DIR/safety_${TIMESTAMP}.log" 2>&1 &
SAFETY_PID=$!
log "  PID=$SAFETY_PID"

wait_for_topic "/cmd_vel_base" 15 || {
    log "⚠️ /cmd_vel_base 未就绪"
}

# ============================================================
# 9. Web 遥控
# ============================================================
log ""
log "━━━ 第9步: Web 遥控 ━━━"
ros2 launch gb_web gb_web.launch.py \
    port:=8080 \
    > "$LOG_DIR/web_${TIMESTAMP}.log" 2>&1 &
WEB_PID=$!
log "  PID=$WEB_PID"
sleep 2
ros2 topic list 2>/dev/null | grep -q "/cmd_vel_web" && log "✅ /cmd_vel_web 就绪" || log "⚠️ /cmd_vel_web 未就绪"

# ============================================================
# 10. 底盘适配器 (连接狗 SDK)
# ============================================================
log ""
log "━━━ 第10步: 底盘适配器 ━━━"
ros2 launch gb_base_driver real_adapter.launch.py \
    read_only:=false \
    max_linear_speed:=0.60 \
    max_angular_speed:=0.80 \
    > "$LOG_DIR/adapter_${TIMESTAMP}.log" 2>&1 &
ADAPTER_PID=$!
log "  PID=$ADAPTER_PID"

# ============================================================
# 11. SDK 连接健康检查 (不阻塞主流程)
# ============================================================
log ""
log "━━━ 第11步: SDK 连接检查 ━━━"

check_sdk_connected() {
    timeout 5 ros2 topic echo /robot_state --once 2>/dev/null | grep -q '"sdk_connected": true'
}

sleep 8
if check_sdk_connected; then
    log "  ✅ SDK 已连接，无需修复"
else
    log "  ⚠️ SDK 未连接，后台触发狗端 repair..."
    (
        DOG_IP="${DOG_IP:-192.168.168.168}"
        DOG_USER="${DOG_USER:-firefly}"
        DOG_PASS="${DOG_PASS:-}"
        if [ -n "$DOG_PASS" ]; then
            timeout 15 sshpass -p "$DOG_PASS" ssh \
                -o ConnectTimeout=3 \
                -o ServerAliveInterval=2 \
                -o ServerAliveCountMax=2 \
                -o StrictHostKeyChecking=no \
                "${DOG_USER}@${DOG_IP}" \
                "sudo -n systemctl stop gosdk-watchdog 2>/dev/null || true;
                 sudo -n pkill -f gosdk 2>/dev/null || true;
                 sudo -n pkill -f mc_ctrl 2>/dev/null || true;
                 sudo -n systemctl start --no-block gosdk-watchdog 2>/dev/null || true" \
                >> "$LOG_DIR/dog_sdk_repair_${TIMESTAMP}.log" 2>&1 \
                || echo "[$(date)] repair timeout/failed, main chain continues" >> "$LOG_DIR/dog_sdk_repair_${TIMESTAMP}.log"
        else
            echo "[$(date)] DOG_PASS not set, skip repair" >> "$LOG_DIR/dog_sdk_repair_${TIMESTAMP}.log"
        fi
    ) &
    log "  🔧 dog sdk repair 后台执行中 (日志: dog_sdk_repair_${TIMESTAMP}.log)"
fi

sleep 3
log ""
log "══════════════════════════════════════"
log "✅ 全链路启动完成！"
log ""
log "进程清单:"
log "  LiDAR+FAST-LIO  PID=$LIO_PID"
log "  Perception      PID=$PERCEP_PID"
log "  Odometry 2D     PID=$ODOM2D_PID"
log "  Nav2            PID=$NAV2_PID"
log "  cloud_to_scan   PID=$SCAN_PID"
log "  AMCL            PID=$AMCL_PID"
log "  Collision Mon   PID=$COLL_PID"
log "  Safety Node     PID=$SAFETY_PID"
log "  Adapter         PID=$ADAPTER_PID"
log ""
log "日志目录: $LOG_DIR"
log "══════════════════════════════════════"
log ""
log "💡 提示: 适配器连接 SDK 后需发送 standUp 指令使狗站立"
log "   ros2 service call /gb_base/stand_up std_srvs/srv/Trigger \"{}\""
log ""
log "💡 停止全链路: pkill -f 'gb_full_chain' 或 systemctl stop gb-full-chain"

# ============================================================
# 保持运行，等待所有子进程
# ============================================================
wait
