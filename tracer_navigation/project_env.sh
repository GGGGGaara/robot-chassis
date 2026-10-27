#!/usr/bin/env bash
# Shared paths. config/local.env is optional, local, and never committed.
TRACER_WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$TRACER_WORKSPACE_ROOT/config/local.env" ]]; then
  source "$TRACER_WORKSPACE_ROOT/config/local.env"
fi
export TRACER_WORKSPACE_ROOT
export TRACER_PROJECT_ROOT="$TRACER_WORKSPACE_ROOT/tracer_navigation"
export TRACER_RUNTIME_ROOT="${TRACER_RUNTIME_ROOT:-$HOME/tracer_nav_runtime}"
export TRACER_VENDOR_ROOT="${TRACER_VENDOR_ROOT:-$TRACER_WORKSPACE_ROOT/third_party/tracer_vendor}"
if [[ ! -d "$TRACER_VENDOR_ROOT/sdk" && -d "$TRACER_WORKSPACE_ROOT/TRACER2.0/TRACER2.0/sdk" ]]; then
  export TRACER_VENDOR_ROOT="$TRACER_WORKSPACE_ROOT/TRACER2.0/TRACER2.0"
fi
export TRACER_DIMOS_ROOT="${TRACER_DIMOS_ROOT:-$TRACER_WORKSPACE_ROOT/ku/dimos-main}"
export TRACER_MAPPING_VENV="${TRACER_MAPPING_VENV:-$HOME/venvs/dimos-mapping}"
export TRACER_PREMAP="${TRACER_PREMAP:-$TRACER_WORKSPACE_ROOT/reconstruction_20261017_113920/raytrace/global_map.pc2.lcm}"
