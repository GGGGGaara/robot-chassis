#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then
  echo '已有底盘驱动，拒绝重配接口。先停止导航并确认车辆静止。'; exit 1
fi
ip -br addr
if ! ip -4 addr show | grep -q '192.168.1.111/'; then
  echo '缺少雷达网卡 IP 192.168.1.111，请检查连接和 Windows 网络配置。'; exit 1
fi
sudo ip link set lo multicast on
sudo ip route replace 224.0.0.0/4 dev lo
sudo sysctl -w net.core.rmem_max=67108864
sudo sysctl -w net.core.rmem_default=67108864
bash "$TASK_DIR/prepare_ros_loopback.sh"
sudo modprobe gs_usb
ip link show can0 >/dev/null
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 listen-only on
sudo ip link set can0 up
echo '网络和 CAN 只听模式已准备好。以下仅采样底盘反馈，不发送运动命令。'
python3 "$TASK_DIR/check_can_feedback.py"
ip -details -statistics link show can0
