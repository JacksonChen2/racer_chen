#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"

export RACER_RUN_ID="${run_id}"
export RACER_EXPERIMENT_BANDWIDTH_HZ=100000000
export RACER_EXPERIMENT_RESOURCE_BLOCKS=66
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=true
export RACER_COMPARISON_BASIS="100 MHz distributed Sionna; epoch-1 initialization guaranteed, subsequent communication channel-gated"
export RACER_ALGORITHM_LABEL="racer_10uav_sionna_distributed_100mhz_initial_assignment_perfect_600s"
export RACER_SUITE_DIR="${RACER_SUITE_DIR:-${workspace}/experiments/five_sites_same_aisle_10uav_sionna_distributed_100mhz_initial_assignment_perfect_600s_${run_id}}"

exec "${workspace}/run_warehouse_full_10uav_five_sites_sionna_no_bs_same_training_600s.sh"
