#!/usr/bin/env bash
# Optional phase 2, after the standalone visualization demo works.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
[[ -x "$ROOT/tracer_navigation/.ros_venv/bin/python" ]] || { echo 'Run install_visualization.sh first'; exit 2; }
PYTHON="$ROOT/tracer_navigation/.ros_venv/bin/python"
"$PYTHON" -m pip install -r "$ROOT/config/requirements-ros.txt"
# Only generated schema code is shared with Python 3.10; do not pull in DimOS's native LCM fork.
"$PYTHON" -m pip install --no-deps dimos_lcm==0.1.4
echo 'ROS adapter dependencies ready; no mapping backend installed or started.'
