#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
reference="${training_root}/../final_racer/reference_result/formal_300s/warehouse_full_distributed_result.json"
reference_coverage="$(jq -r '.metrics.mapping_coverage_joint' "${reference}")"
if [[ "${reference_coverage}" != "0.752913131313" ]]; then
  printf 'Expected the 75.2913%% perfect reference, got %s from %s\n' \
    "${reference_coverage}" "${reference}" >&2
  exit 2
fi

export RACER_QWEN8_CONFIG="${training_root}/config/qwen8b_fp8_four_process_ltask_20402020_gamma005.yaml"
export RACER_QWEN8_PERFECT_RESULT="${reference}"
export RACER_QWEN8_EPISODES=20
export RACER_QWEN8_DURATION=300
export RACER_QWEN8_COVERAGE_UPDATE_RATE_HZ=0.5
export RACER_QWEN8_GAMMA_TASK=0.05
export RACER_QWEN8_ALGORITHM_LABEL=qwen8b_fp8_crpo_ltask_20402020_gamma005
export RACER_QWEN8_CAMPAIGN="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/qwen8b_fp8_20ep_ltask_20402020_gamma005_$(date +%Y%m%d_%H%M%S)}"

exec "${training_root}/scripts/run_four_process_campaign.sh"
