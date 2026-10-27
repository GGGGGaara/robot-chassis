#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
bash "$TASK_DIR/prepare_ros_loopback.sh"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/generate_config.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/offline_check.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/test_safety.py"
if ! ip link show vcan_navtest >/dev/null 2>&1; then
  sudo modprobe vcan
  sudo ip link add dev vcan_navtest type vcan
  sudo ip link set vcan_navtest up
fi
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/test_driver_vcan.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/integration_test.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/test_mission_offline.py"
"$TASK_DIR/.ros_venv/bin/python" "$TASK_DIR/test_closed_loop.py" --selected --vcan --soak-s 90
PYTHONPATH="$TRACER_DIMOS_ROOT" OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2 "$TRACER_MAPPING_VENV/bin/python" "$TASK_DIR/test_recorded_relocalization.py"
echo '离线验证结束。以上不代表实车运动或制动测试已完成。'
