#!/bin/bash
# ============================================================
# GB钢镚 — 全链路开机自启动脚本
# 文件: /home/nvidia/gb_ws2/scripts/gb_full_chain.sh
#
# 启动顺序:
#   1. LiDAR + FAST-LIO
#   2. 静态 TF
#   3. Perception 点云滤波
#   4. Odometry 2D 投影
#   5. Nav2 节点启动
#   6. 手动 lifecycle 激活 Nav2
#   7. 定位 cloud_to_scan + AMCL
#   8. 碰撞监控
#   9. 安全闸门
#  10. Web 遥控
#  11. 底盘适配器
#  12. SDK 连接健康检查
#
# 关键设计：
#   - nav2_minimal.launch.py 不启动 lifecycle_manager
#   - 本脚本唯一管理 Nav2 lifecycle
#   - 避免 lifecycle_manager 与脚本抢状态
#   - 关键 Nav2 节点未 active 时直接退出，不启动真机运动链路
# ============================================================

set -e

# ==== 配置 ====
MAP_FILE="/home/nvidia/gb_maps/20260704_gb_pointfoot_v2/gb_map.yaml"
COLLISION_CONFIG="/home/nvidia/gb_ws2/src/gb_nav2/config/collision_monitor_93.yaml"
AMCL_PARAMS="/home/nvidia/gb_ws2/src/gb_nav2/config/amcl_params.yaml"
NAV2_PARAMS="/home/nvidia/gb_ws2/src/gb_bringup/config/nav2_params.yaml"
ROS_DOMAIN_ID=0

# 是否自动发布 /initialpose
# 默认关闭，推荐使用 Web/RViz 重定位
GB_AUTO_INITIALPOSE="${GB_AUTO_INITIALPOSE:-0}"

# ==== 环境 ====
source /opt/ros/humble/setup.bash
source /home/nvidia/gb_ws2/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID

# ==== ros2 daemon ====
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true
sleep 1

# ==== 日志 ====
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
log() {
    echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOG"
}

wait_for_topic() {
    local topic=$1
    local timeout_sec=${2:-30}

    log "等待话题 $topic ..."

    for i in $(seq 1 "$timeout_sec"); do
        if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
            log "✅ $topic 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done

    log "❌ $topic 超时 (${timeout_sec}s)"
    return 1
}

wait_for_node() {
    local node=$1
    local timeout_sec=${2:-30}

    log "等待节点 $node ..."

    for i in $(seq 1 "$timeout_sec"); do
        if ros2 node list 2>/dev/null | grep -qx "$node"; then
            log "✅ $node 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done

    log "❌ $node 超时 (${timeout_sec}s)"
    return 1
}

wait_for_tf() {
    local from=$1
    local to=$2
    local timeout_sec=${3:-30}

    log "等待 TF: $from → $to ..."

    for i in $(seq 1 "$timeout_sec"); do
        if timeout 2 ros2 run tf2_ros tf2_echo "$from" "$to" 2>/dev/null | grep -q "Translation"; then
            log "✅ TF $from→$to 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done

    log "❌ TF $from→$to 超时 (${timeout_sec}s)"
    return 1
}

ensure_lifecycle_active() {
    local node=$1
    local state=""
    local configure_timeout=15
    local activate_timeout=10

    if [ "$node" = "/map_server" ]; then
        configure_timeout=60
        activate_timeout=20
    fi

    log "检查 lifecycle: $node"

    for i in $(seq 1 25); do
        state=$(timeout 3 ros2 lifecycle get "$node" 2>/dev/null || echo "TIMEOUT")

        if echo "$state" | grep -q "active"; then
            log "✅ $node active"
            return 0
        fi

        if echo "$state" | grep -q "unconfigured"; then
            log "  $node unconfigured → configure"
            timeout "$configure_timeout" ros2 lifecycle set "$node" configure 2>/dev/null || true
            sleep 1

        elif echo "$state" | grep -q "inactive"; then
            log "  $node inactive → activate"
            timeout "$activate_timeout" ros2 lifecycle set "$node" activate 2>/dev/null || true
            sleep 1

        else
            log "  $node state=$state，等待..."
            sleep 1
        fi
    done

    log "❌ $node 未能进入 active"
    return 1
}

# 检查 /cmd_vel_base 是否仅由 safety_node 发布（允许 safety_node 多 endpoint）
check_cmd_vel_base_owner() {
    log "检查 /cmd_vel_base publisher 归属..."
    local info
    info=$(timeout 5 ros2 topic info /cmd_vel_base --verbose 2>/dev/null || true)
    echo "$info" > "$LOG_DIR/cmd_vel_base_info_${TIMESTAMP}.log"

    local pub_count
    pub_count=$(echo "$info" | grep -E "Publisher count:" | awk '{print $3}')
    if [ -z "$pub_count" ] || [ "$pub_count" = "0" ]; then
        log "❌ /cmd_vel_base 没有 publisher"
        return 1
    fi

    # 只解析 Publisher 区域里的 Node name，不解析 subscriber 区域
    local pub_nodes
    pub_nodes=$(echo "$info" | awk '
        /Publisher count:/ {inpub=1; next}
        /Subscription count:/ {inpub=0}
        inpub && /Node name:/ {print $3}
    ' | sort -u)

    if [ -z "$pub_nodes" ]; then
        log "❌ 无法解析 /cmd_vel_base publisher 节点"
        return 1
    fi

    log "  /cmd_vel_base publisher count = $pub_count"
    log "  publisher nodes:"
    echo "$pub_nodes" | while read -r n; do log "    - $n"; done

    # 允许 safety_node 有多个 publisher endpoint
    # 但不允许其他节点发布 /cmd_vel_base
    local bad_nodes
    bad_nodes=$(echo "$pub_nodes" | grep -Ev '^(safety_node)$' || true)
    if [ -n "$bad_nodes" ]; then
        log "❌ /cmd_vel_base 存在非 safety_node 发布者:"
        echo "$bad_nodes" | while read -r n; do log "   BAD: $n"; done
        return 1
    fi

    # 额外检查：确认只有一个 safety_node 进程（非 Ghost 双进程）
    log "  检查 safety_node 进程..."
    pgrep -af "safety_node|gb_safety" >> "$LOG_DIR/safety_process_${TIMESTAMP}.log" 2>&1 || true
    local safety_count
    safety_count=$(pgrep -cf "safety_node|gb_safety" 2>/dev/null || echo 0)
    if [ "$safety_count" -gt 1 ]; then
        log "  ⚠️ 检测到 $safety_count 个 safety_node 进程，可能为 Ghost 残留"
    fi

    log "  ✅ /cmd_vel_base 仅由 safety_node 发布，publisher endpoint=$pub_count，允许"
    return 0
}

log "══════════════════════════════════════"
log "GB钢镚 全链路自启动"
log "══════════════════════════════════════"

# ============================================================
# 1. LiDAR 驱动 + FAST-LIO
# ============================================================
log ""
log "━━━ 第1步: LiDAR + FAST-LIO ━━━"

ros2 launch gb_lio fastlio.launch.py \
    > "$LOG_DIR/lio_${TIMESTAMP}.log" 2>&1 &
LIO_PID=$!
log "  PID=$LIO_PID"

wait_for_topic "/cloud_body" 30 || {
    log "❌ /cloud_body 话题未出现，终止"
    exit 1
}

log "  确认 /cloud_body 数据流..."
for i in $(seq 1 10); do
    if timeout 3 ros2 topic hz /cloud_body 2>/dev/null | grep -q "average rate"; then
        log "  ✅ /cloud_body 数据流正常"
        break
    fi

    if [ "$i" -eq 10 ]; then
        log "❌ /cloud_body 无数据，终止"
        exit 1
    fi

    sleep 1
done

# ============================================================
# 2. 静态 TF
# ============================================================
log ""
log "━━━ 第2步: 静态 TF ━━━"

ros2 run tf2_ros static_transform_publisher \
    -0.119 0 -0.135 0 -0.261799 0 \
    livox_imu_link base_link \
    --ros-args -r __node:=tf_livoximu_to_base \
    > "$LOG_DIR/tf_livoximu_base_${TIMESTAMP}.log" 2>&1 &
TF_IMU_BASE_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0.15 0 0.10 0 0.261799 0 \
    base_link lidar_link \
    --ros-args -r __node:=tf_base_to_lidarlink \
    > "$LOG_DIR/tf_base_lidar_${TIMESTAMP}.log" 2>&1 &
TF_BASE_LIDAR_PID=$!

ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 \
    lidar_link livox_frame \
    --ros-args -r __node:=tf_lidar_to_livox \
    > "$LOG_DIR/tf_lidar_livox_${TIMESTAMP}.log" 2>&1 &
TF_LIDAR_LIVOX_PID=$!

log "  TF PIDs: $TF_IMU_BASE_PID, $TF_BASE_LIDAR_PID, $TF_LIDAR_LIVOX_PID"

wait_for_tf "camera_init" "base_link" 20 || {
    log "⚠️ camera_init→base_link 暂未就绪，继续，后续节点会等待 TF"
}

# ============================================================
# 3. Perception 点云滤波
# ============================================================
log ""
log "━━━ 第3步: Perception 点云滤波 ━━━"

ros2 launch gb_perception perception.launch.py \
    input_topic:=/cloud_body \
    output_topic:=/points_nav \
    > "$LOG_DIR/perception_${TIMESTAMP}.log" 2>&1 &
PERCEP_PID=$!
log "  PID=$PERCEP_PID"

wait_for_topic "/points_nav" 20 || {
    log "⚠️ /points_nav 未就绪，继续"
}

# ============================================================
# 4. Odometry 2D 投影
# ============================================================
log ""
log "━━━ 第4步: Odometry 2D 投影 ━━━"

ros2 launch gb_lio odom_2d.launch.py \
    > "$LOG_DIR/odom2d_${TIMESTAMP}.log" 2>&1 &
ODOM2D_PID=$!
log "  PID=$ODOM2D_PID"

wait_for_topic "/Odometry_2d" 20 || {
    log "❌ /Odometry_2d 未就绪，Nav2 无法稳定运行，终止"
    exit 1
}

# ============================================================
# 5. Nav2 节点启动，不启动 lifecycle_manager
# ============================================================
log ""
log "━━━ 第5步: Nav2 导航栈节点启动 ━━━"

ros2 launch gb_bringup nav2_minimal.launch.py \
    params_file:="$NAV2_PARAMS" \
    use_sim_time:=false \
    autostart:=false \
    use_lifecycle_manager:=false \
    > "$LOG_DIR/nav2_${TIMESTAMP}.log" 2>&1 &
NAV2_PID=$!
log "  PID=$NAV2_PID"

NAV2_NODES=(
    /map_server
    /planner_server
    /controller_server
    /smoother_server
    /behavior_server
    /bt_navigator
    /velocity_smoother
)

for n in "${NAV2_NODES[@]}"; do
    wait_for_node "$n" 30 || {
        log "❌ Nav2 节点 $n 未出现，终止"
        exit 1
    }
done

# ============================================================
# 6. 手动 lifecycle 激活 Nav2
# ============================================================
log ""
log "━━━ 第6步: 手动激活 Nav2 lifecycle ━━━"

ensure_lifecycle_active /map_server || {
    log "❌ map_server 激活失败，终止"
    exit 1
}

wait_for_topic "/map" 60 || {
    log "❌ /map 未就绪，终止"
    exit 1
}

for n in \
    /planner_server \
    /controller_server \
    /smoother_server \
    /behavior_server \
    /bt_navigator \
    /velocity_smoother
do
    ensure_lifecycle_active "$n" || {
        log "❌ $n 激活失败，停止启动后续真机链路"
        exit 1
    }
done

log "✅ Nav2 lifecycle 全部 active"

# ============================================================
# 7. 定位: pointcloud_to_laserscan + AMCL
# ============================================================
log ""
log "━━━ 第7步: 定位 cloud_to_scan + AMCL ━━━"

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

ros2 run nav2_amcl amcl \
    --ros-args \
    -r __node:=amcl \
    -r scan:=/scan \
    --params-file "$AMCL_PARAMS" \
    -p use_sim_time:=false \
    > "$LOG_DIR/amcl_${TIMESTAMP}.log" 2>&1 &
AMCL_PID=$!

log "  PIDs: scan=$SCAN_PID, amcl=$AMCL_PID"

wait_for_node /amcl 20 || {
    log "❌ AMCL 节点未出现，终止"
    exit 1
}

sleep 2

log "  configure AMCL..."
timeout 10 ros2 lifecycle set /amcl configure 2>/dev/null || log "  ⚠️ configure AMCL 失败或超时"

sleep 1

log "  activate AMCL..."
timeout 10 ros2 lifecycle set /amcl activate 2>/dev/null || log "  ⚠️ activate AMCL 失败或超时"

wait_for_topic "/scan" 20 || {
    log "⚠️ /scan 未就绪，AMCL 可能无法定位"
}

if [ "$GB_AUTO_INITIALPOSE" = "1" ]; then
    DEFAULT_X="${GB_INIT_X:-0.49}"
    DEFAULT_Y="${GB_INIT_Y:-0.72}"
    YAW_RAD="${GB_INIT_YAW:--0.314}"

    QZ=$(python3 - <<PY
import math
yaw=float("$YAW_RAD")
print(math.sin(yaw/2.0))
PY
)
    QW=$(python3 - <<PY
import math
yaw=float("$YAW_RAD")
print(math.cos(yaw/2.0))
PY
)

    log "  自动设置 AMCL 初始位姿: x=${DEFAULT_X}, y=${DEFAULT_Y}, yaw_rad=${YAW_RAD}"

    for i in 1 2 3; do
        ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: map},
          pose: {pose: {position: {x: ${DEFAULT_X}, y: ${DEFAULT_Y}, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}},
          covariance: [0.25,0,0,0,0,0,
                       0,0.25,0,0,0,0,
                       0,0,0,0,0,0,
                       0,0,0,0,0,0,
                       0,0,0,0,0,0,
                       0,0,0,0,0,0.068]}}" \
        2>/dev/null || true
        sleep 0.3
    done

    wait_for_tf "map" "camera_init" 30 || {
        log "⚠️ 自动 initialpose 后 map→camera_init 未就绪"
    }
else
    log "  跳过自动 initialpose，请使用 Web/RViz 重定位"
    log "  手动重定位前没有 map→camera_init 是正常现象"
fi

log "✅ 定位节点已启动"

# ============================================================
# 8. 碰撞监控
# ============================================================
log ""
log "━━━ 第8步: 碰撞监控 ━━━"

ros2 launch gb_nav2 collision_monitor.launch.py \
    params_file:="$COLLISION_CONFIG" \
    use_sim_time:=false \
    autostart:=true \
    > "$LOG_DIR/collision_${TIMESTAMP}.log" 2>&1 &
COLL_PID=$!
log "  PID=$COLL_PID"

wait_for_topic "/cmd_vel_collision" 20 || {
    log "⚠️ /cmd_vel_collision 未就绪"
}

# ============================================================
# 9. 安全闸门
# ============================================================
log ""
log "━━━ 第9步: 安全闸门 ━━━"

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
    log "❌ /cmd_vel_base 未就绪，终止"
    exit 1
}

check_cmd_vel_base_owner || {
    log "❌ /cmd_vel_base 发布者归属异常，终止"
    exit 1
}

# ============================================================
# 10. Web 遥控
# ============================================================
log ""
log "━━━ 第10步: Web 遥控 ━━━"

ros2 launch gb_web gb_web.launch.py \
    port:=8080 \
    > "$LOG_DIR/web_${TIMESTAMP}.log" 2>&1 &
WEB_PID=$!
log "  PID=$WEB_PID"

sleep 2
ros2 topic list 2>/dev/null | grep -qx "/cmd_vel_web" \
    && log "✅ /cmd_vel_web 就绪" \
    || log "⚠️ /cmd_vel_web 未就绪"

# ============================================================
# 11. 底盘适配器
# ============================================================
log ""
log "━━━ 第11步: 底盘适配器 ━━━"

ros2 launch gb_base_driver real_adapter.launch.py \
    read_only:=false \
    max_linear_speed:=0.60 \
    max_angular_speed:=0.80 \
    > "$LOG_DIR/adapter_${TIMESTAMP}.log" 2>&1 &
ADAPTER_PID=$!
log "  PID=$ADAPTER_PID"

# ============================================================
# 12. SDK 连接健康检查，不阻塞主流程
# ============================================================
log ""
log "━━━ 第12步: SDK 连接检查 ━━━"

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

    log "  🔧 dog sdk repair 后台执行中"
fi

# ============================================================
# 完成
# ============================================================
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
log "  Web             PID=$WEB_PID"
log "  Adapter         PID=$ADAPTER_PID"
log ""
log "日志目录: $LOG_DIR"
log "══════════════════════════════════════"
log ""
log "💡 使用流程:"
log "   1. 打开 Web: http://机器人IP:8080"
log "   2. 使用 Web/RViz 重定位"
log "   3. 调用 standUp:"
log "      ros2 service call /gb_base/stand_up std_srvs/srv/Trigger \"{}\""
log "   4. 再发送导航目标"
log ""
log "💡 停止全链路:"
log "   systemctl stop gb-full-chain"
log "   或 pkill -f gb_full_chain"

wait
