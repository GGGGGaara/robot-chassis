#!/usr/bin/env bash
set -euo pipefail
if [[ $(id -u) -ne 0 ]]; then
  echo '请用 sudo bash setup_dependencies.sh 安装 ROS 2 官方包。' >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Per-command sources only; retain Ubuntu/ROS keyring validation and TLS.
APT_OPTIONS=(-o "Dir::Etc::sourcelist=$TASK_DIR/apt_sources.list" -o Dir::Etc::sourceparts=- -o Acquire::Retries=1 -o Acquire::http::Timeout=20 -o Acquire::https::Timeout=20)
apt-get "${APT_OPTIONS[@]}" update -qq
apt-get "${APT_OPTIONS[@]}" install -y --no-install-recommends \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  libasio-dev can-utils python3-scipy python3-pil python3-yaml python3-venv
/usr/bin/python3 -m venv --system-site-packages "$TASK_DIR/.ros_venv"
"$TASK_DIR/.ros_venv/bin/python" -m pip install lcm
