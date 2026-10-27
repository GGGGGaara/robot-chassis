#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then
  echo '已有底盘驱动，拒绝重配 CAN。先停止旧驱动并确认车辆静止。'; exit 1
fi
bash "$TASK_DIR/commission.sh" check
echo '实车导航启动：进入 CAN 控制模式，初始未解锁。保持现场急停可用。'
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 listen-only off
sudo ip link set can0 up
restore_listen_only() {
  sudo -n ip link set can0 down
  sudo -n ip link set can0 type can bitrate 500000 listen-only on
  sudo -n ip link set can0 up
}
trap restore_listen_only EXIT
bash "$TASK_DIR/start_navigation.sh" enable_drive:=true
