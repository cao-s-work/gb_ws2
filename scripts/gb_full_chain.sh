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
#   5. Nav2 节点启动 + map_server 激活 → /map
#   6. cloud_to_scan + AMCL 定位  → map→camera_init TF
#   7. Nav2 导航节点激活 (planner/controller/smoother/...) → /cmd_vel_nav
#   8. 碰撞监控                    → /cmd_vel_collision
#   9. 安全闸门                    → /cmd_vel_base
#  10. Web 遥控
#  11. 底盘适配器                  → SDK → 狗
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
        if timeout 2 ros2 run tf2_ros tf2_echo "$from" "$to" 2>/dev/null | grep -q "Translation"; then
            log "✅ TF $from→$to 就绪 (${i}s)"
            return 0
        fi
        sleep 1
    done
    log "❌ TF $from→$to 超时 (${timeout}s)"
    return 1
}

# lifecycle 统一激活函数
ensure_lifecycle_active() {
    local node=$1
    local timeout=${2:-20}
    log "  激活 $node ..."
    if ! timeout $timeout ros2 lifecycle set "$node" configure 2>/dev/null; then
        log "  ⚠️ $node configure 超时，尝试直接 activate"
    fi
    sleep 0.5
    if ! timeout $timeout ros2 lifecycle set "$node" activate 2>/dev/null; then
        log "  ❌ $node activate 失败"
        return 1
    fi
    log "  ✅ $node active"
    return 0
}

# 使用 Python/rclpy 发布 /initialpose，带当前时间戳
publish_initialpose_now() {
    local x="${1:-0.49}" y="${2:-0.72}" yaw="${3:--0.314}"
    log "  发布 AMCL initialpose: x=$x, y=$y, yaw=$yaw (Python/rclpy + now timestamp)"
    GB_INIT_X="$x" GB_INIT_Y="$y" GB_INIT_YAW="$yaw" /usr/bin/python3 - <<'PY'
import os, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('gb_initialpose_publisher')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def make_msg(self):
        x = float(os.environ.get('GB_INIT_X', '0.49'))
        y = float(os.environ.get('GB_INIT_Y', '0.72'))
        yaw = float(os.environ.get('GB_INIT_YAW', '-0.314'))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.068
        msg.pose.covariance = cov
        return msg

def main():
    rclpy.init()
    node = InitialPosePublisher()
    for _ in range(30):
        if node.pub.get_subscription_count() > 0: break
        rclpy.spin_once(node, timeout_sec=0.1)
    for i in range(5):
        msg = node.make_msg()
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.25)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
PY
    local rc=$?
    [ $rc -ne 0 ] && { log "  ❌ Python initialpose 发布失败 (exit=$rc)"; return 1; }
    log "  ✅ /initialpose 已发布 x5 (now timestamp)"
    return 0
}

# 检查 /cmd_vel_base 仅由 safety_node 发布（带 DDS discovery 重试）
check_cmd_vel_base_owner() {
    log "检查 /cmd_vel_base 发布者归属..."
    local attempts=0 max_attempts=5
    while [ $attempts -lt $max_attempts ]; do
        local info
        info=$(timeout 5 ros2 topic info /cmd_vel_base --verbose 2>/dev/null || true)
        local pub_count
        pub_count=$(echo "$info" | grep "Publisher count:" | awk '{print $3}')
        if [ -z "$pub_count" ] || [ "$pub_count" = "0" ]; then
            attempts=$((attempts + 1))
            log "    ⚠️ /cmd_vel_base 尚无 publisher (${attempts}/${max_attempts})"
            sleep 2
            continue
        fi
        local pub_nodes
        pub_nodes=$(echo "$info" | awk '/Publisher count:/{inpub=1;next} /Subscription count:/{inpub=0} inpub&&/Node name:/{print $3}' | sort -u)
        local bad_nodes
        bad_nodes=$(echo "$pub_nodes" | grep -Ev '^(safety_node|_NODE_NAME_UNKNOWN_)$' || true)
        if [ -n "$bad_nodes" ]; then
            log "❌ /cmd_vel_base 存在非 safety_node 发布者: $bad_nodes"
            return 1
        fi
        log "  ✅ /cmd_vel_base 仅由 safety_node 发布 (endpoint=$pub_count)"
        return 0
    done
    log "❌ /cmd_vel_base publisher 检查超时"
    return 1
}

# 狗端 SSH 封装
dog_ssh() {
    local cmd="$1"
    local dog_ip="${DOG_IP:-192.168.168.168}"
    local dog_user="${DOG_USER:-firefly}"
    local dog_pass="${DOG_PASS:-}"
    if [ -n "$dog_pass" ]; then
        timeout 12 sshpass -p "$dog_pass" ssh \
            -o ConnectTimeout=3 \
            -o ServerAliveInterval=2 \
            -o ServerAliveCountMax=2 \
            -o StrictHostKeyChecking=no \
            "${dog_user}@${dog_ip}" "$cmd"
    else
        timeout 12 ssh \
            -o ConnectTimeout=3 \
            -o ServerAliveInterval=2 \
            -o ServerAliveCountMax=2 \
            -o StrictHostKeyChecking=no \
            "${dog_user}@${dog_ip}" "$cmd"
    fi
}

# 启动/确认狗端 SDK watchdog
ensure_dog_sdk_daemon() {
    log "启动/确认狗端 SDK watchdog..."
    dog_ssh "
        sudo -n systemctl start --no-block gosdk-watchdog 2>/dev/null || true
        sleep 2
        pgrep -af 'mc_ctrl r|gosdk' || true
    " >> "$LOG_DIR/dog_sdk_start_${TIMESTAMP}.log" 2>&1 || {
        log "⚠️ 狗端 SDK watchdog 启动检查失败，继续让 adapter 自动重连"
        return 1
    }
    log "✅ 狗端 SDK watchdog 已触发"
    return 0
}

# 等待 /robot_state 中 sdk_connected=true
wait_for_sdk_connected() {
    local timeout_sec=${1:-45}
    log "等待 SDK connected=true ..."
    for i in $(seq 1 "$timeout_sec"); do
        if timeout 3 ros2 topic echo /robot_state --once 2>/dev/null | grep -q '"sdk_connected": true'; then
            log "✅ SDK connected=true (${i}s)"
            return 0
        fi
        sleep 1
    done
    log "❌ SDK connected=true 超时 (${timeout_sec}s)"
    return 1
}

# 强制清理旧节点（防止残留端口占用、双发布者 Ghost）
log "清理旧 ROS 节点..."
pkill -f "nav2_minimal.launch.py" 2>/dev/null || true
pkill -f "fastlio.launch.py" 2>/dev/null || true
pkill -f "perception.launch.py" 2>/dev/null || true
pkill -f "odom_2d.launch.py" 2>/dev/null || true
pkill -f "collision_monitor.launch.py" 2>/dev/null || true
pkill -f "gb_web.launch.py" 2>/dev/null || true
pkill -f "real_adapter.launch.py" 2>/dev/null || true
pkill -f "map_server" 2>/dev/null || true
pkill -f "planner_server" 2>/dev/null || true
pkill -f "controller_server" 2>/dev/null || true
pkill -f "smoother_server" 2>/dev/null || true
pkill -f "behavior_server" 2>/dev/null || true
pkill -f "bt_navigator" 2>/dev/null || true
pkill -f "velocity_smoother" 2>/dev/null || true
pkill -f "lifecycle_manager" 2>/dev/null || true
pkill -f "nav2_amcl" 2>/dev/null || true
pkill -f "amcl" 2>/dev/null || true
pkill -f "pointcloud_to_laserscan" 2>/dev/null || true
pkill -f "collision_monitor" 2>/dev/null || true
pkill -f "safety_node" 2>/dev/null || true
pkill -f "static_transform_publisher" 2>/dev/null || true
sleep 3
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true
sleep 1

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
    autostart:=false \
    use_lifecycle_manager:=false \
    > "$LOG_DIR/nav2_${TIMESTAMP}.log" 2>&1 &
NAV2_PID=$!
log "  PID=$NAV2_PID"

wait_for_topic "/map" 30 || {
    log "❌ /map 未就绪，终止"
    exit 1
}

# 先只激活 map_server，提供 /map；其余节点等 AMCL 定位就绪后再激活
log "  激活 map_server..."
timeout 15 ros2 lifecycle set /map_server configure 2>/dev/null || log "  ⚠️ map_server configure 超时"
sleep 0.5
timeout 15 ros2 lifecycle set /map_server activate 2>/dev/null || log "  ⚠️ map_server activate 超时"
log "  ✅ map_server active"

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

# 5c. AMCL lifecycle 手动激活 + 自动 initialpose
log "  等待 AMCL 节点就绪..."
for i in $(seq 1 10); do
    if ros2 node list 2>/dev/null | grep -q "/amcl"; then
        log "  AMCL 节点已就绪"
        break
    fi
    sleep 1
done
sleep 2

ensure_lifecycle_active /amcl || {
    log "❌ AMCL 未能 active，终止"
    exit 1
}

# 自动设置初始位姿
DEFAULT_X="${GB_INIT_X:-0.49}"
DEFAULT_Y="${GB_INIT_Y:-0.72}"
YAW_RAD="${GB_INIT_YAW:--0.314}"
publish_initialpose_now "$DEFAULT_X" "$DEFAULT_Y" "$YAW_RAD" || {
    log "❌ initialpose 发布失败，终止"
    exit 1
}

wait_for_tf "map" "camera_init" 30 || {
    log "❌ map→camera_init 未就绪，终止"
    exit 1
}
# 额外等待 AMCL 粒子收敛
sleep 5
log "✅ 定位栈就绪"

# AMCL 就绪后，激活其余 Nav2 节点（严格检查每个节点）
log "  激活 Nav2 导航节点..."
NAV2_ACTIVE_OK=1
for n in \
    /planner_server \
    /controller_server \
    /smoother_server \
    /behavior_server \
    /bt_navigator \
    /velocity_smoother
do
    ensure_lifecycle_active "$n" || {
        log "❌ $n 未能 active"
        NAV2_ACTIVE_OK=0
    }
done
if [ "$NAV2_ACTIVE_OK" != "1" ]; then
    log "❌ Nav2 导航节点未全部 active，停止启动后续链路"
    exit 1
fi
log "✅ Nav2 导航节点全部 active"

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
    -p input_cmd_topic:=/cmd_vel_nav \
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

check_cmd_vel_base_owner || {
    log "⚠️ /cmd_vel_base 发布者归属检查未通过，继续（可能 DDS 未就绪）"
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
# 10. 底盘 SDK + adapter
# ============================================================
log ""
log "━━━ 第10步: 底盘 SDK + adapter ━━━"
ensure_dog_sdk_daemon || true
ros2 launch gb_base_driver real_adapter.launch.py \
    read_only:=false \
    sdk_local_ip:="${SDK_LOCAL_IP:-192.168.168.216}" \
    sdk_dog_ip:="${SDK_DOG_IP:-192.168.168.168}" \
    max_linear_speed:=0.35 \
    max_angular_speed:=0.35 \
    publish_rate:=10.0 \
    > "$LOG_DIR/adapter_${TIMESTAMP}.log" 2>&1 &
ADAPTER_PID=$!
log "  Adapter PID=$ADAPTER_PID"
wait_for_topic "/robot_state" 20 || {
    log "⚠️ /robot_state 未就绪，adapter 可能未启动成功"
}
if ! wait_for_sdk_connected 45; then
    log "⚠️ SDK 未连接，触发一次狗端 repair 后继续等待"
    dog_ssh "
        sudo -n systemctl restart gosdk-watchdog 2>/dev/null || true
        sleep 3
        pgrep -af 'mc_ctrl r|gosdk' || true
    " >> "$LOG_DIR/dog_sdk_repair_${TIMESTAMP}.log" 2>&1 || true
    wait_for_sdk_connected 30 || {
        log "❌ SDK 仍未连接，保留 Web/导航显示，但不要实机运动"
    }
fi
if [ "${GB_AUTO_STANDUP:-0}" = "1" ]; then
    log "自动 standUp..."
    timeout 15 ros2 service call /gb_base/stand_up std_srvs/srv/Trigger "{}" \
        >> "$LOG_DIR/standup_${TIMESTAMP}.log" 2>&1 || log "⚠️ standUp 调用失败"
else
    log "跳过自动 standUp；需要手动执行：ros2 service call /gb_base/stand_up std_srvs/srv/Trigger \"{}\""
fi

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
log "💡 停止全链路: pkill -f 'gb_full_chain' 或 systemctl stop gb-full-chain"

# ============================================================
# 保持运行，等待所有子进程
# ============================================================
wait
