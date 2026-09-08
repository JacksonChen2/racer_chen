#!/usr/bin/env bash
set -euo pipefail
final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RACER_MAP_PERFECT_DELIVERY=true
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false
export RACER_EXPERIMENT_DURATION=300
export RACER_START_LAYOUT_FILE="${RACER_START_LAYOUT_FILE:-${final_root}/config/warehouse_full_10uav_five_sites_layout.json}"
export RACER_SUITE_DIR="${RACER_SUITE_DIR:-${final_root}/results/map_perfect_other_sionna_300s_$(date +%Y%m%d_%H%M%S)}"
export RACER_ALGORITHM_LABEL=final_racer_map_perfect_other_sionna_300s
exec bash "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh" "${1:---run}"
