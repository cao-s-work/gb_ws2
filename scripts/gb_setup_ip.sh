#!/bin/bash
# ============================================================
# GB钢镚 — 虚拟IP配置脚本 (LiDAR子网)
# 由 systemd gb-ip-setup.service 在开机时调用
# 作用: 在 enP8p1s0 上添加虚拟IP 192.168.1.216 用于 LiDAR 通信
# ============================================================
set -e

INTERFACE="enP8p1s0"
VIRTUAL_IP="192.168.1.216/24"

# 等待网卡就绪
for i in $(seq 1 30); do
    if ip link show "$INTERFACE" 2>/dev/null | grep -q "state UP"; then
        break
    fi
    echo "[gb-ip-setup] 等待网卡 $INTERFACE 就绪... ($i/30)"
    sleep 2
done

if ! ip link show "$INTERFACE" 2>/dev/null | grep -q "state UP"; then
    echo "[gb-ip-setup] ⚠️ 网卡 $INTERFACE 未就绪，跳过虚拟IP配置"
    exit 1
fi

# 检查虚拟IP是否已存在
if ip addr show "$INTERFACE" | grep -q "$VIRTUAL_IP"; then
    echo "[gb-ip-setup] ✅ 虚拟IP $VIRTUAL_IP 已存在于 $INTERFACE"
    exit 0
fi

# 添加虚拟IP
sudo ip addr add "$VIRTUAL_IP" dev "$INTERFACE"
echo "[gb-ip-setup] ✅ 虚拟IP $VIRTUAL_IP 已添加到 $INTERFACE"
