#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/mapping_session.py" "$@"
