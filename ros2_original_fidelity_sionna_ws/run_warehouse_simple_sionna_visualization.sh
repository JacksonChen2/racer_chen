#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RACER_FIDELITY_HEADLESS=0
export RACER_FIDELITY_VISUALIZE=1
# Sionna solves in a worker thread; rendering remains capped for responsive camera input.
export RACER_INTERACTIVE_RENDER_HZ="${RACER_INTERACTIVE_RENDER_HZ:-30}"
export RACER_VISUALIZATION_MAX_MAP_POINTS="${RACER_VISUALIZATION_MAX_MAP_POINTS:-12000}"
exec "${workspace_dir}/run_warehouse_simple_sionna.sh" "$@"
