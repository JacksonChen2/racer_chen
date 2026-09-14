#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/hybrid_communication_racer_warehouse_full_with_ap_central_10uav_ideal_vs_pure_distributed_200s_${run_id}}"

export RACER_DURATION_S=200
export RACER_SUITE_DIR="${suite}"
export RACER_EXPLORATION_ASSIGNMENT_MODE=global_cooperative
export RACER_SIONNA_RUNTIME_DIR="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
export RACER_IDEAL_DOMAIN_ID="${RACER_IDEAL_DOMAIN_ID:-93}"
export RACER_SIONNA_DOMAIN_ID="${RACER_SIONNA_DOMAIN_ID:-94}"

"${workspace}/run_warehouse_full_10uav_central_ideal_vs_sionna_300s.sh"

python3 "${workspace}/scripts/analyze_hybrid_communication_pair.py" "${suite}"
printf 'HYBRID_COMMUNICATION_SUITE=%s\n' "${suite}"
