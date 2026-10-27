#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
if [[ "${1:-}" == status ]]; then
  exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/check_runtime.py"
fi
exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/navctl.py" "$@"
