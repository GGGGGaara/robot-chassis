#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$TASK_DIR/../tracer_navigation/project_env.sh"
source "$TRACER_MAPPING_VENV/bin/activate"
export PYTHONPATH="$TRACER_DIMOS_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=2
exec python "$TASK_DIR/fixed_map_path_preview.py" "$@"
