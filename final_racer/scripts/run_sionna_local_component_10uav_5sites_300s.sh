#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RACER_EXPLORATION_ASSIGNMENT_MODE=local_component
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false
export RACER_UAV_TX_POWER_DBM="${RACER_UAV_TX_POWER_DBM:-23}"
export RACER_ALGORITHM_LABEL="${RACER_ALGORITHM_LABEL:-final_racer_local_component_10uav_5sites_300s_sionna_23dbm}"

exec "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh" "${@}"
