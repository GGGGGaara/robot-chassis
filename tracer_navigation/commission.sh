#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/commission.py" "$@"
