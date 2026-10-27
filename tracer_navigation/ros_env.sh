#!/usr/bin/env bash
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/project_env.sh"
# ROS Humble Python is 3.10; do not inherit the DimOS 3.12 environment.
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV
source /opt/ros/humble/setup.bash
if [[ -f "$TRACER_RUNTIME_ROOT/ws/install/setup.bash" ]]; then
  source "$TRACER_RUNTIME_ROOT/ws/install/setup.bash"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# Avoid stale or root-owned FastDDS shared-memory segments after service checks.
# Every project process uses the same explicit loopback-only UDP transport.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$TASK_DIR/fastdds_loopback.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$TASK_DIR/fastdds_loopback.xml"
unset FASTDDS_BUILTIN_TRANSPORTS
export PATH="$TASK_DIR/.ros_venv/bin:/usr/bin:/bin:$PATH"
