#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if pgrep -f '/tracer_safe_base( |$)' >/dev/null; then echo 'Stop existing driver first'; exit 1; fi
if [[ $EUID != 0 ]]; then echo 'Run as root to configure CAN test mode'; exit 1; fi
restore() {
  ip link set can0 down
  ip link set can0 type can bitrate 500000 listen-only on
  ip link set can0 up
}
trap restore EXIT
ip link set can0 down
ip link set can0 type can bitrate 500000 listen-only off
ip link set can0 up
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/probe_zero_can_rate.py"
