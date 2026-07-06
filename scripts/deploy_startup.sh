#!/bin/bash
# ============================================================
# GB钢镚 — 自启动部署脚本
# 一次性安装所有 systemd 服务和启动脚本
#
# 用法:
#   bash /home/nvidia/gb_ws2/scripts/deploy_startup.sh
#
# 包含:
#   1. Jetson 端: 虚拟IP + 全链路自启动
#   2. 狗端部署指引 (需手动 SSH 部署)
# ============================================================
set -e

SCRIPTS_DIR="/home/nvidia/gb_ws2/scripts"
echo "══════════════════════════════════════"
echo "GB钢镚 自启动部署"
echo "══════════════════════════════════════"

# ============================================================
# Jetson 端
# ============================================================
echo ""
echo "━━━ Jetson 端 ━━━"

# 1. 安装 systemd 服务文件
echo "[1/3] 安装 systemd 服务文件..."
sudo cp "$SCRIPTS_DIR/gb-ip-setup.service" /etc/systemd/system/
sudo cp "$SCRIPTS_DIR/gb-full-chain.service" /etc/systemd/system/
sudo systemctl daemon-reload

# 2. 启用服务
echo "[2/3] 启用开机自启动..."
sudo systemctl enable gb-ip-setup.service
sudo systemctl enable gb-full-chain.service

# 3. 验证
echo "[3/3] 验证服务状态..."
sudo systemctl status gb-ip-setup.service --no-pager 2>/dev/null || echo "  (尚未启动，下次开机生效)"
sudo systemctl status gb-full-chain.service --no-pager 2>/dev/null || echo "  (尚未启动，下次开机生效)"

echo ""
echo "✅ Jetson 端部署完成"
echo ""
echo "━━━ 管理命令 ━━━"
echo "  启动:   sudo systemctl start gb-full-chain"
echo "  停止:   sudo systemctl stop gb-full-chain"
echo "  状态:   sudo systemctl status gb-full-chain"
echo "  日志:   sudo journalctl -u gb-full-chain -f"
echo "  禁用:   sudo systemctl disable gb-full-chain"
echo ""

# ============================================================
# 狗端部署指引
# ============================================================
echo "━━━ 狗端部署 (需手动 SSH 到 192.168.234.1) ━━━"
echo ""
echo "  请手动执行以下命令 (狗本体上):"
echo ""
echo "  # 1. 复制文件到狗"
echo "  scp $SCRIPTS_DIR/gosdk_watchdog.sh firefly@192.168.234.1:/home/firefly/"
echo "  scp $SCRIPTS_DIR/gosdk-watchdog.service firefly@192.168.234.1:/home/firefly/"
echo ""
echo "  # 2. SSH 到狗"
echo "  ssh firefly@192.168.234.1"
echo ""
echo "  # 3. 狗上安装"
echo "  chmod +x /home/firefly/sdk_watchdog.sh"
echo "  # 编辑脚本, 确认 SDK_PATH 和 SDK_SCRIPT 正确"
echo "  vim /home/firefly/sdk_watchdog.sh"
echo ""
echo "  sudo cp /home/firefly/gosdk-watchdog.service /etc/systemd/system/"
echo "  sudo systemctl daemon-reload"
echo "  sudo systemctl enable gosdk-watchdog.service"
echo "  sudo systemctl start gosdk-watchdog.service"
echo ""
echo "  # 4. 验证"
echo "  sudo systemctl status gosdk-watchdog.service"
echo "  ss -tlnp | grep 8082"
echo ""
echo "══════════════════════════════════════"
echo "部署指引完成"
