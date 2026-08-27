#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export C2_HEADLESS=0
export C2_VISUALIZE=1
export C2_DRONE_COUNT="${C2_DRONE_COUNT:-5}"
export C2_DURATION="${C2_DURATION:-900}"
export C2_INTERACTIVE_RENDER_HZ="${C2_INTERACTIVE_RENDER_HZ:-30}"
export C2_VISUALIZATION_MAX_MAP_POINTS="${C2_VISUALIZATION_MAX_MAP_POINTS:-12000}"
exec "${script_dir}/run_c2_warehouse.sh"
