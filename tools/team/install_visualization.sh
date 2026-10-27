#!/usr/bin/env bash
# Adds tooling to an EXISTING Ubuntu 22.04 + Humble + Nav2 installation.
set -euo pipefail
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 && -f /opt/ros/humble/setup.bash ]] || { echo 'Requires existing Ubuntu 22.04 + ROS 2 Humble'; exit 2; }
sudo apt-get update
sudo apt-get install -y ros-humble-rviz2 ros-humble-rviz-default-plugins \
  ros-humble-nav2-rviz-plugins ros-humble-rqt-graph ros-humble-tf2-tools \
  ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py \
  python3-colcon-common-extensions python3-venv python3-numpy python3-yaml \
  python3-scipy python3-pil build-essential cmake git libasio-dev can-utils
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -x "$ROOT/tracer_navigation/.ros_venv/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$ROOT/tracer_navigation/.ros_venv"
fi
echo 'Visualization tooling ready. No sensors, CAN setup, or robot processes were started.'
