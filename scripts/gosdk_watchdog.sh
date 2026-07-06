#!/bin/bash
# Go2 狗本体 SDK 自启动 watchdog
# SDK: /opt/export/mc/bin/mc_ctrl
set -e

SDK_DIR="/opt/export/mc/bin"
SDK_BIN="./mc_ctrl"
SDK_ARGS="r"
SDK_CLIENT_IP="192.168.168.168"
LOG_DIR="/home/firefly/logs"
LOG_FILE="${LOG_DIR}/gosdk_watchdog.log"
PID_FILE="/home/firefly/gosdk_mc_ctrl.pid"
CHECK_INTERVAL=5
MAX_FAILURES=3

mkdir -p "$LOG_DIR"
log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"; }

is_running() {
    [ -f "$PID_FILE" ] && kill -0 "$(cat $PID_FILE)" 2>/dev/null && return 0
    pgrep -f "mc_ctrl r" >/dev/null 2>&1 && return 0
    return 1
}

start_sdk() {
    # 先检查是否已有 mc_ctrl 在运行（可能由 robot-launch 管理）
    if pgrep -f "mc_ctrl r" >/dev/null 2>&1; then
        log "ℹ️ mc_ctrl 已在运行，跳过启动（由 robot-launch 或其他进程管理）"
        pgrep -f "mc_ctrl r" | head -1 > "$PID_FILE"
        return 0
    fi
    log "🚀 启动 mc_ctrl ..."
    ifconfig lo multicast 2>/dev/null || true
    route add -net 224.0.0.0 netmask 240.0.0.0 dev lo 2>/dev/null || true
    export LD_LIBRARY_PATH="$SDK_DIR"
    export ROBOT_TYPE=XG
    export SDK_CLIENT_IP="$SDK_CLIENT_IP"
    cd "$SDK_DIR"
    taskset -c 7 $SDK_BIN $SDK_ARGS &
    echo $! > "$PID_FILE"
    log "  PID=$(cat $PID_FILE)"
    sleep 5
    pgrep -f "mc_ctrl r" >/dev/null 2>&1 && log "✅ mc_ctrl 启动成功" || { log "❌ 启动失败"; return 1; }
}

log "🔄 Go2 SDK Watchdog 启动 (mc_ctrl)"
trap 'log "退出"; exit 0' SIGTERM SIGINT

failures=0
while true; do
    if is_running; then
        failures=0
    else
        failures=$((failures + 1))
        log "⚠️ mc_ctrl 未运行 (${failures}/${MAX_FAILURES})"
        if [ $failures -ge $MAX_FAILURES ]; then
            start_sdk && failures=0 || log "❌ 重启失败"
        fi
    fi
    sleep "$CHECK_INTERVAL"
done
