#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
bash "$TASK_DIR/prepare_ros_loopback.sh"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/generate_config.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/wait_for_localization.py"
if [[ " $* " == *" enable_drive:=true "* ]]; then
  echo '实车模式：将启动 CAN 驱动，初始未使能；任务通过检查后才允许行驶。'
else
  echo '导航预览：不启动 CAN 驱动，速度输出到 /nav_cmd_preview；初始未使能。'
fi
exec ros2 launch "$TASK_DIR/navigation.launch.py" "$@"
