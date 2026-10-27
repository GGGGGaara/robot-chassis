#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/tracer_navigation/ros_env.sh"
PYTHON="$ROOT/tracer_navigation/.ros_venv/bin/python"
[[ -x "$PYTHON" ]] || { echo 'Missing project .ros_venv. Run install_visualization.sh'; exit 2; }
exec "$PYTHON" "$ROOT/tools/team/doctor.py" "$@"
