#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
if pgrep -f '/tracer_safe_base( |$)|dimos_ros_bridge.py|ros_mapping_backend.py|navigation.launch.py' >/dev/null; then
  echo '已有驱动或导航实例，先在原终端退出，避免重复 TF 和底盘驱动。'; exit 1
fi
bash "$TASK_DIR/prepare_ros_loopback.sh"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/generate_config.py"
export TRACER_PROJECT_ROOT="$TASK_DIR"
if [[ " $* " == *" enable_drive:=true "* ]]; then
  bash "$TASK_DIR/commission.sh" check
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 500000 listen-only off
  sudo ip link set can0 up
  restore() {
    sudo -n ip link set can0 down
    sudo -n ip link set can0 type can bitrate 500000 listen-only on
    sudo -n ip link set can0 up
  }
  trap restore EXIT
fi
ros2 launch tracer_bringup system.launch.py project_root:="$TASK_DIR" "$@"
