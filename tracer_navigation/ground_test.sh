#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
if [[ "${1:-}" != --execute ]]; then
  exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/ground_test_pulse.py"
fi
case "${2:-forward}" in forward|left|right) ;; *) echo '方向只能为 forward、left、right'; exit 2;; esac
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then
  echo '已有底盘驱动，拒绝重配 CAN。先停止实车导航并确认车辆静止。'; exit 1
fi
echo "实车短时测试：${2:-forward}；前进上限 0.03 m/s，转向上限 0.10 rad/s，最长 1 秒。"
echo '物理急停须由现场人员掌握；出现异常立即停止。'
# The operator terminal obtains sudo normally; no embedded credentials.
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000 listen-only off
sudo ip link set can0 up
restore_listen_only() {
  sudo -n ip link set can0 down
  sudo -n ip link set can0 type can bitrate 500000 listen-only on
  sudo -n ip link set can0 up
}
trap restore_listen_only EXIT
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/ground_test_pulse.py" --execute --direction "${2:-forward}"
