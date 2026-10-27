#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID=74 ROS_LOCALHOST_ONLY=1
source "$ROOT/tracer_navigation/ros_env.sh"
OUT="$ROOT/tracer_navigation/results/team_alignment_20261027"
mkdir -p "$OUT"
python "$ROOT/tools/team/test_portability.py" > "$OUT/portability.log" 2>&1
python "$ROOT/tools/team/smoke_demo.py"
bash "$ROOT/tools/team/doctor.sh" --profile demo > "$OUT/doctor_demo.json"
python -m unittest discover -s "$ROOT/tracer_navigation" -p test_safety.py > "$OUT/safety.log" 2>&1
python -m unittest discover -s "$ROOT/tracer_navigation" -p test_coordinates_and_models.py > "$OUT/coordinates_models.log" 2>&1
echo 'Portability, isolated demo, demo environment, safety, and coordinate/model unit checks passed.'
