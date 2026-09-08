#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
final_root="${RACER_FINAL_RACER_ROOT:-$(realpath "${training_root}/../final_racer")}" 
reference_result="${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json"
ground_truth="${final_root}/data/gt_occupied_voxels.txt"

export RACER_QWEN8_CAMPAIGN="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/qwen8b_fp8_latest_$(date +%Y%m%d_%H%M%S)}"
export RACER_QWEN8_CONFIG="${training_root}/config/qwen8b_fp8_four_process.yaml"
export RACER_QWEN8_REFERENCE_ROOT="${RACER_QWEN8_CAMPAIGN}/reference"
export RACER_QWEN8_GT_PATH="${ground_truth}"
export RACER_QWEN8_PERFECT_RESULT="${reference_result}"
export RACER_QWEN8_EPISODES="${RACER_QWEN8_EPISODES:-30}"
export RACER_QWEN8_DURATION="${RACER_QWEN8_DURATION:-300}"
export RACER_QWEN8_GAMMA_TASK="${RACER_QWEN8_GAMMA_TASK:-0.25}"
export RACER_QWEN8_ACTIVATION_FN="${RACER_QWEN8_ACTIVATION_FN:-silu}"
export RACER_QWEN8_COMMUNICATION_MODE="${RACER_QWEN8_COMMUNICATION_MODE:-sionna}"
export RACER_QWEN8_FIXED_MCS_INDEX="${RACER_QWEN8_FIXED_MCS_INDEX:--1}"
export RACER_QWEN8_MINIMUM_COVERAGE="${RACER_QWEN8_MINIMUM_COVERAGE-0.73}"
export RACER_QWEN8_DOMAIN_BASE=190
export RACER_QWEN8_TRAINING_LABEL=final_racer_qwen8b_fp8_resource_ltask025_sionna_adaptive
export RACER_REQUIRE_QWEN_MODEL=true
exec "${training_root}/scripts/run_four_process_campaign.sh"
