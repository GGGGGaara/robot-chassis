#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source "$ROOT/tracer_navigation/ros_env.sh"
bash "$ROOT/tools/team/demo_network.sh"
[[ -x "$ROOT/tracer_navigation/.ros_venv/bin/python" ]] || { echo 'Run tools/team/install_visualization.sh first'; exit 2; }
[[ -f "$TRACER_RUNTIME_ROOT/ws/install/tracer_bringup/share/tracer_bringup/launch/visualization.launch.py" ]] || { echo 'Run bash tracer_navigation/build_ros2.sh first'; exit 2; }
echo 'OFFLINE SYNTHETIC DEMO, domain 74. No hardware, no Nav2 execution, no cmd_vel.'
exec ros2 launch tracer_bringup visualization.launch.py project_root:="$TRACER_PROJECT_ROOT" demo:=true "$@"
