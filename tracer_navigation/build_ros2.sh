#!/usr/bin/env bash
set -eo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/ros_env.sh"
RUNTIME="$TRACER_RUNTIME_ROOT/ws"
mkdir -p "$RUNTIME/src/tracer_bringup"
cp -a "$TASK_DIR/ws/src/tracer_bringup/." "$RUNTIME/src/tracer_bringup/"
cd "$RUNTIME"
colcon build --packages-select tracer_bringup
