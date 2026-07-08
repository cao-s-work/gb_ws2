#!/bin/bash
# ================================================================
# 钢镚 ZSL-1W 全链路启动脚本  v2
# ================================================================
# 用法:
#   ./start.sh              # 安全模式 (read_only=true, 狗不动)
#   ./start.sh --real       # 实机模式 (⚠️ 狗会运动! preflight 更严)
#   ./start.sh --dry-run    # 仅网络+SDK诊断，不启动任何东西
# ================================================================

set -e

# ── 颜色 ──
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'
OK="${GREEN}✓${NC}"; FAIL="${RED}✗${NC}"; WARN="${YELLOW}⚠${NC}"

# ── 参数解析 ──
REAL_MODE="false"; DRY_RUN="false"
for arg in "$@"; do
    case "$arg" in
        --real)   REAL_MODE="true" ;;
        --dry-run) DRY_RUN="true" ;;
        --help|-h) echo "用法: $0 [--real] [--dry-run]"; exit 0 ;;
    esac
done

# ── 配置 ──
DOG_IP="192.168.168.168"
JETSON_IP="192.168.168.216"
SDK_PORT="43988"
GB_WS="/home/nvidia/gb_ws2"
MAP_YAML="/home/nvidia/gb_maps/$(date +%Y%m%d)_gb_pointfoot/map.yaml"
WEB_PORT="8080"
DOG_SSH="sshpass -p firefly ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 firefly@${DOG_IP}"
SDK_PYTHON="python3.10"
SDK_PATH="/home/nvidia/gb_ws2/sdk/genisom_l1_sdk-main/lib/zsl-1w/aarch64"

# 实机模式用更保守的速度上限
if [[ "$REAL_MODE" == "true" ]]; then
    ADAPTER_MAX_LINEAR="0.10"
    ADAPTER_MAX_ANGULAR="0.20"
else
    ADAPTER_MAX_LINEAR="0.25"
    ADAPTER_MAX_ANGULAR="0.25"
fi

# ── 日志 ──
LOG_DIR="/home/nvidia/gb_ws2/logs"
mkdir -p "$LOG_DIR"

echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   钢镚 ZSL-1W 全链路启动  v2            ║${NC}"
if [[ "$REAL_MODE" == "true" ]]; then
    echo -e "${CYAN}║   ${RED}⚠ 实机模式 — 狗会运动!${CYAN}               ║${NC}"
else
    echo -e "${CYAN}║   安全模式 — read_only=true             ║${NC}"
fi
echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
echo ""

# ================================================================
# Phase 0: SHM 清理
# ================================================================
echo -e "${BLUE}[Phase 0] SHM 清理${NC}"

ros2 daemon stop 2>/dev/null || true
sudo rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
sudo rm -f /dev/shm/sem.fastrtps* 2>/dev/null || true
ros2 daemon start 2>/dev/null || true
echo -e "  ${OK} SHM 已清理"

# ================================================================
# Phase 1: 网络诊断
# ================================================================
echo -e "\n${BLUE}[Phase 1] 网络诊断${NC}"

if ping -c 1 -W 2 "$DOG_IP" &>/dev/null; then
    echo -e "  ${OK} 狗 $DOG_IP 可达"
else
    echo -e "  ${FAIL} 无法 ping 通 $DOG_IP"
    echo -e "  ${RED}请检查: 狗是否上电? 网线是否插好?${NC}"
    exit 1
fi

JETSON_IF=$(ip -4 addr show enP8p1s0 2>/dev/null | grep -oP 'inet \K[\d.]+' || true)
if [ -n "$JETSON_IF" ]; then
    echo -e "  ${OK} Jetson 有线: $JETSON_IF"
else
    echo -e "  ${WARN} Jetson 有线接口无 IP"
fi

if [[ "$DRY_RUN" == "true" ]]; then
    echo -e "\n${YELLOW}--dry-run 模式: 网络 OK，退出。${NC}"
    exit 0
fi

# ================================================================
# Phase 2: 狗端 SDK 重启 + 验证
# ================================================================
echo -e "\n${BLUE}[Phase 2] 狗端 SDK 服务重启${NC}"

echo -n "  重启 robot-launch ... "
$DOG_SSH 'sudo systemctl restart robot-launch' 2>/dev/null && echo -e "${OK}" || {
    echo -e "${FAIL} SSH 失败"
    echo -e "  ${RED}无法 SSH 进狗，请手动检查。${NC}"
    exit 1
}

echo -n "  等待狗端服务恢复 ... "
for i in $(seq 1 15); do
    if $DOG_SSH 'systemctl is-active robot-launch' 2>/dev/null | grep -q active; then
        echo -e "${OK} (${i}s)"
        break
    fi
    sleep 1
    if [ $i -eq 15 ]; then
        echo -e "${WARN} 超时，继续..."
    fi
done

sleep 3  # 等 mc_ctrl 完全就绪

echo -n "  SDK 连接验证 ... "
SDK_OUT=$($SDK_PYTHON -c "
import sys; sys.path.insert(0,'$SDK_PATH')
import mc_sdk_zsl_1w_py, time
s = mc_sdk_zsl_1w_py.HighLevel()
s.initRobot('$JETSON_IP', $SDK_PORT, '$DOG_IP')
time.sleep(2)
mode = s.getCurrentCtrlmode()
batt = s.getBatteryPower()
print(f'MODE={mode} BATT={batt}')
" 2>&1)

# Stale Session 自动重试
if echo "$SDK_OUT" | grep -q "InValid"; then
    echo -e "${WARN} Stale Session 残留，再次重启狗端..."
    $DOG_SSH 'sudo systemctl restart robot-launch' 2>/dev/null
    sleep 8
    SDK_OUT=$($SDK_PYTHON -c "
import sys; sys.path.insert(0,'$SDK_PATH')
import mc_sdk_zsl_1w_py, time
s = mc_sdk_zsl_1w_py.HighLevel()
s.initRobot('$JETSON_IP', $SDK_PORT, '$DOG_IP')
time.sleep(2)
print(f'MODE={s.getCurrentCtrlmode()} BATT={s.getBatteryPower()}')
" 2>&1)
fi

MODE=$(echo "$SDK_OUT" | grep -oP 'MODE=\K\d+')
BATT=$(echo "$SDK_OUT" | grep -oP 'BATT=\K\d+')

if [ "$MODE" != "0" ] && [ "$BATT" != "0" ]; then
    echo -e "  ${OK} SDK 就绪 (Mode=$MODE, Battery=$BATT%)"
else
    echo -e "  ${FAIL} SDK 异常 (Mode=$MODE, Battery=$BATT)"
    echo -e "  ${RED}请检查狗端，可能需要物理断电重启。${NC}"
    exit 1
fi

# ================================================================
# Phase 3: 地图检查
# ================================================================
echo -e "\n${BLUE}[Phase 3] 地图检查${NC}"

if [ -f "$MAP_YAML" ]; then
    echo -e "  ${OK} 地图: $MAP_YAML"
    # 检查 negate 值
    NEGATE=$(grep -oP 'negate:\s*\K\d+' "$MAP_YAML" || echo "?")
    if [ "$NEGATE" = "1" ]; then
        echo -e "  ${OK} negate=$NEGATE (正确)"
    else
        echo -e "  ${WARN} negate=$NEGATE (预期=1, 如果建图时没反相可能不对)"
    fi
else
    echo -e "  ${FAIL} 地图文件不存在: $MAP_YAML"
    echo "  可用地图:"
    ls -d /home/nvidia/gb_maps/*/ 2>/dev/null || echo "  (无)"
    exit 1
fi

# ================================================================
# Phase 4: Source 工作空间
# ================================================================
echo -e "\n${BLUE}[Phase 4] 加载 ROS 2 工作空间${NC}"

source /opt/ros/humble/setup.bash
source "$GB_WS/install/setup.bash"
echo -e "  ${OK} ROS 2 Humble + gb_ws 已加载"

export PYTHONPATH="$SDK_PATH:$PYTHONPATH"
export LD_LIBRARY_PATH="$SDK_PATH:$LD_LIBRARY_PATH"

# ================================================================
# Phase 5: 清理已有 ROS 2 进程
# ================================================================
echo -e "\n${BLUE}[Phase 5] 清理已有进程${NC}"

OLD_PROCS=$(ps aux | grep -E "fast_lio|nav2|collision_monitor|safety_node|gb_web|gb_base_driver|real_base_adapter|points_filter|pointcloud_to_laserscan|amcl" | grep -v grep | awk '{print $2}')
if [ -n "$OLD_PROCS" ]; then
    echo "  发现旧进程: $(echo "$OLD_PROCS" | wc -l) 个，正在终止..."
    echo "$OLD_PROCS" | xargs kill 2>/dev/null || true
    sleep 2
    echo "$OLD_PROCS" | xargs kill -9 2>/dev/null || true
    echo -e "  ${OK} 已清理"
else
    echo -e "  ${OK} 无旧进程"
fi

# ================================================================
# Phase 6: 启动 navigation.launch.py
# (FAST-LIO + 感知滤波 + Nav2 + collision_monitor + safety_node)
# ================================================================
echo -e "\n${BLUE}[Phase 6] 启动导航栈${NC}"
echo "  组件: FAST-LIO → points_filter → Nav2 → collision_monitor → safety_node"

LOG_FILE="$LOG_DIR/navigation_$(date +%Y%m%d_%H%M%S).log"

ros2 launch gb_bringup navigation.launch.py \
    enable_safety:=true \
    enable_collision_monitor:=true \
    connect_base:=true \
    use_mock_base:=false \
    allow_real_base:=true \
    safety_output_topic:=/cmd_vel_base \
    require_odom:=false \
    require_points:=false \
    params_file:="$GB_WS/src/gb_bringup/config/nav2_params.yaml" \
    > "$LOG_FILE" 2>&1 &

NAV_PID=$!
echo -e "  ${OK} navigation.launch.py PID=$NAV_PID"
echo "  日志: $LOG_FILE"

# ================================================================
# Phase 7: 等待关键话题就绪
# ================================================================
echo -e "\n${BLUE}[Phase 7] 等待节点就绪${NC}"

wait_for_topic() {
    local topic="$1"; local timeout="${2:-30}"; local desc="${3:-$topic}"
    echo -n "  等待 $desc ... "
    for i in $(seq 1 "$timeout"); do
        if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
            echo -e "${OK} (${i}s)"
            return 0
        fi
        sleep 1
    done
    echo -e "${WARN} 超时"
    return 1
}

wait_for_topic "/cloud_registered" 30 "FAST-LIO /cloud_registered" || true
wait_for_topic "/cloud_body" 10 "FAST-LIO /cloud_body" || true
wait_for_topic "/Odometry" 30 "FAST-LIO /Odometry" || true
wait_for_topic "/points_nav" 10 "points_filter /points_nav" || true
wait_for_topic "/map" 15 "map_server /map" || true
wait_for_topic "/plan" 15 "planner /plan" || true
wait_for_topic "/cmd_vel_collision" 15 "collision_monitor" || true

# ================================================================
# Phase 8: 启动底盘适配器
# ================================================================
echo -e "\n${BLUE}[Phase 8] 启动底盘适配器${NC}"

READ_ONLY="true"
if [[ "$REAL_MODE" == "true" ]]; then
    READ_ONLY="false"
    echo -e "  ${RED}⚠ read_only=false — 狗将响应速度指令!${NC}"
else
    echo -e "  ${GREEN}read_only=true — 安全模式，狗不运动${NC}"
fi

ros2 launch gb_base_driver real_adapter.launch.py \
    read_only:="$READ_ONLY" \
    sdk_local_ip:="$JETSON_IP" \
    sdk_local_port:="$SDK_PORT" \
    sdk_dog_ip:="$DOG_IP" \
    max_linear_speed:="$ADAPTER_MAX_LINEAR" \
    max_angular_speed:="$ADAPTER_MAX_ANGULAR" \
    > "$LOG_DIR/adapter_$(date +%Y%m%d_%H%M%S).log" 2>&1 &

ADAPTER_PID=$!
echo -e "  ${OK} real_adapter PID=$ADAPTER_PID"
sleep 3

# ================================================================
# Phase 9: 启动 Web 控制 (显式传 port 参数)
# ================================================================
echo -e "\n${BLUE}[Phase 9] 启动 Web 控制${NC}"

ros2 run gb_web gb_web_node --ros-args \
    -p port:="$WEB_PORT" \
    > "$LOG_DIR/web_$(date +%Y%m%d_%H%M%S).log" 2>&1 &

WEB_PID=$!
echo -e "  ${OK} gb_web PID=$WEB_PID"

sleep 3
if curl -s --max-time 2 "http://127.0.0.1:${WEB_PORT}/api/state" > /dev/null 2>&1; then
    echo -e "  ${OK} Web 就绪: http://127.0.0.1:${WEB_PORT}"
else
    echo -e "  ${WARN} Web 尚未响应，可能还需等待"
fi

# ================================================================
# Phase 10: 全链路验证
# ================================================================
echo -e "\n${BLUE}[Phase 10] 全链路验证${NC}"

# ── 10.1 节点清单 ──
echo -e "\n  ${CYAN}── 节点清单 ──${NC}"
NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
echo -e "  运行中节点: ${GREEN}${NODE_COUNT}${NC}"
ros2 node list 2>/dev/null | while read n; do echo "    - $n"; done

# ── 10.2 关键话题 (只查频率最重要的几个) ──
echo -e "\n  ${CYAN}── 关键话题频率 ──${NC}"
for t in /Odometry /cloud_body /points_nav /cmd_vel_base; do
    if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
        HZ=$(timeout 3 ros2 topic hz "$t" 2>/dev/null | grep "average" | awk '{print $3}' | head -1)
        echo -e "  ${OK} $t ${GREEN}${HZ:-?}${NC} Hz"
    else
        echo -e "  ${WARN} $t ${YELLOW}(未检测到)${NC}"
    fi
done

# 其余话题只确认存在，不查频率
echo -e "\n  ${CYAN}── 其他话题存在性 ──${NC}"
for t in /cloud_registered /map /scan /plan /cmd_vel_nav /cmd_vel_collision; do
    if ros2 topic list 2>/dev/null | grep -q "^${t}$"; then
        echo -e "  ${OK} $t"
    else
        echo -e "  ${WARN} $t ${YELLOW}(未检测到)${NC}"
    fi
done

# ── 10.3 TF 链路 ──
echo -e "\n  ${CYAN}── TF 链路 ──${NC}"
for pair in "map camera_init" "camera_init base_link" "base_link lidar_link"; do
    if timeout 3 ros2 run tf2_ros tf2_echo $pair 2>/dev/null | head -1 | grep -q "Translation"; then
        echo -e "  ${OK} $pair"
    else
        echo -e "  ${WARN} $pair ${YELLOW}(暂无)${NC}"
    fi
done

# ── 10.4 /cmd_vel_base 安全链路 (强制校验) ──
echo -e "\n  ${CYAN}── /cmd_vel_base 安全链路 ──${NC}"
CMD_VEL_INFO=$(ros2 topic info /cmd_vel_base --verbose 2>/dev/null || echo "")

if [ -z "$CMD_VEL_INFO" ]; then
    echo -e "  ${WARN} /cmd_vel_base 话题不存在"
else
    echo "$CMD_VEL_INFO" > "$LOG_DIR/cmd_vel_base_info.txt"

    if echo "$CMD_VEL_INFO" | grep -q "Node name: safety_node"; then
        echo -e "  ${OK} publisher: safety_node ✓"
    else
        echo -e "  ${RED}✗ /cmd_vel_base publisher 不是 safety_node!${NC}"
        echo "$CMD_VEL_INFO"
        [[ "$REAL_MODE" == "true" ]] && exit 1
    fi

    if echo "$CMD_VEL_INFO" | grep -q "Node name: gb_base_driver_node"; then
        echo -e "  ${OK} subscriber: gb_base_driver_node ✓"
    else
        echo -e "  ${RED}✗ /cmd_vel_base subscriber 不是 gb_base_driver_node!${NC}"
        [[ "$REAL_MODE" == "true" ]] && exit 1
    fi

    # 检查是否有多个 publisher
    PUB_COUNT=$(echo "$CMD_VEL_INFO" | grep -c "Publisher count:" | head -1 || true)
    PUB_ACTUAL=$(echo "$CMD_VEL_INFO" | grep "Publisher count:" | grep -oP '\d+')
    if [ "${PUB_ACTUAL:-0}" -gt 1 ]; then
        echo -e "  ${RED}✗ /cmd_vel_base 有 ${PUB_ACTUAL} 个 publisher (应只有 1 个)!${NC}"
        [[ "$REAL_MODE" == "true" ]] && exit 1
    fi
fi

# ── 10.5 adapter 重复启动检查 ──
echo -e "\n  ${CYAN}── 重复启动检查 ──${NC}"
ADAPTER_COUNT=$(ros2 node list 2>/dev/null | grep -c "gb_base_driver_node" || echo "0")
if [ "${ADAPTER_COUNT:-0}" -eq 1 ]; then
    echo -e "  ${OK} gb_base_driver_node 数量: 1"
elif [ "${ADAPTER_COUNT:-0}" -eq 0 ]; then
    echo -e "  ${WARN} gb_base_driver_node 未启动"
else
    echo -e "  ${RED}✗ gb_base_driver_node 重复 (${ADAPTER_COUNT} 个)!${NC}"
    [[ "$REAL_MODE" == "true" ]] && exit 1
fi

# ── 10.6 碰撞监测 ──
echo -e "\n  ${CYAN}── 碰撞监测 ──${NC}"
LC_STATE=$(ros2 lifecycle get /collision_monitor 2>/dev/null || echo "unknown")
echo "  collision_monitor lifecycle: $LC_STATE"

if echo "$LC_STATE" | grep -q "active \[3\]"; then
    echo -e "  ${OK} collision_monitor active"
else
    echo -e "  ${WARN} 非 active，尝试 configure → activate..."
    ros2 lifecycle set /collision_monitor configure 2>/dev/null || true
    sleep 1
    ros2 lifecycle set /collision_monitor activate 2>/dev/null || true
    sleep 1
    LC_STATE2=$(ros2 lifecycle get /collision_monitor 2>/dev/null || echo "unknown")
    if echo "$LC_STATE2" | grep -q "active"; then
        echo -e "  ${OK} collision_monitor active (手动激活成功)"
    else
        echo -e "  ${WARN} collision_monitor 仍未激活: $LC_STATE2"
        [[ "$REAL_MODE" == "true" ]] && echo -e "  ${RED}实机模式建议修复后再运行!${NC}"
    fi
fi

# ── 10.7 参数检查 ──
echo -e "\n  ${CYAN}── Base Adapter 参数 ──${NC}"
for p in read_only max_linear_speed max_angular_speed; do
    VAL=$(ros2 param get /gb_base_driver_node "$p" 2>/dev/null | sed 's/^.*value: //' || echo "?")
    echo -e "  /gb_base_driver_node/$p = ${VAL:-?}"
done

echo -e "\n  ${CYAN}── Safety 参数 ──${NC}"
for p in max_linear_x max_angular_z require_odom require_points; do
    VAL=$(ros2 param get /safety_node "$p" 2>/dev/null | sed 's/^.*value: //' || echo "?")
    echo -e "  /safety_node/$p = ${VAL:-?}"
done

# ================================================================
# Phase 11: 完成
# ================================================================
echo -e "\n${GREEN}╔══════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   ✅ 全链路启动完成!                     ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════╝${NC}"

echo ""
echo -e "  进程 PID:"
echo -e "    NAV=$NAV_PID  ADAPTER=$ADAPTER_PID  WEB=$WEB_PID"
echo ""
echo -e "  管理命令:"
echo -e "    ${CYAN}Web 控制台:${NC}  http://127.0.0.1:${WEB_PORT}"
echo -e "    ${CYAN}查看日志:${NC}    tail -f $LOG_DIR/navigation_*.log"
echo -e "    ${CYAN}急停:${NC}        ros2 topic pub --once /emergency_stop std_msgs/msg/Bool 'data: true'"
echo ""

if [[ "$REAL_MODE" != "true" ]]; then
    echo -e "  ${YELLOW}当前是安全模式 (read_only=true)，狗不会运动。${NC}"
    echo -e "  ${YELLOW}切换实机模式请重新运行:${NC} ./start.sh --real"
else
    echo -e "  ${RED}⚠ 实机模式 — 请确保有人物理接管!${NC}"
fi

echo -e "\n${YELLOW}按 Ctrl+C 安全停止所有进程${NC}"

# ================================================================
# cleanup: 先停车，再杀进程 (真机模式下关键!)
# ================================================================
cleanup() {
    echo -e "\n${YELLOW}╔══════════════════════════════════════════╗${NC}"
    echo -e "${YELLOW}║   正在安全停止...                         ║${NC}"
    echo -e "${YELLOW}╚══════════════════════════════════════════╝${NC}"

    # 1. 先发急停 (真机模式下必须)
    if [[ "$REAL_MODE" == "true" ]]; then
        echo -e "${YELLOW}  发送 emergency_stop=true ...${NC}"
        timeout 2 ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true
        sleep 0.5
    fi

    # 2. 尝试 lie_down (如果 dog 站立)
    if [[ "$REAL_MODE" == "true" ]]; then
        echo -e "${YELLOW}  尝试 lie_down ...${NC}"
        timeout 3 ros2 service call /gb_base/lie_down std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true
        sleep 0.5
    fi

    # 3. 先 graceful kill
    echo -e "${YELLOW}  停止进程...${NC}"
    for pid in "$WEB_PID" "$NAV_PID" "$ADAPTER_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    sleep 2

    # 4. 强制 kill 残留
    for pid in "$WEB_PID" "$NAV_PID" "$ADAPTER_PID"; do
        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    echo -e "${GREEN}已停止。${NC}"
}
trap cleanup SIGINT SIGTERM EXIT

wait
