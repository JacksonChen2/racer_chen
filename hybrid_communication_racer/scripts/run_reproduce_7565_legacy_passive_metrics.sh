#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/hybrid_communication_racer"
overlay="${workspace}/overlays/legacy_7565_passive_metrics_ws"
reference_root="${workspace}/experiments/five_sites_same_aisle_10uav_300s_20260831/qwen14b_crpo_fixed_constraint_30ep_lr2e4_b64_s384_tanh_20260901/reference"

export RACER_RUN_ID=7565_legacy_passive_metrics_20260902
export RACER_SUITE_DIR="${workspace}/experiments/reproduce_7565_legacy_passive_metrics_20260902"
export RACER_EXPECTED_CPU_THREADS=18
export RACER_ALLOW_SHARED_GPU=1
export RACER_DEBUG_OVERLAY_SETUP="${overlay}/install/setup.bash"
export RACER_PASSIVE_TASK_METRICS=true
export RACER_TASK_METRIC_OBSERVER_MODE=async
export RACER_TASK_METRIC_SAMPLE_PERIOD_S=0.1
export RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${reference_root}/gt_occupied_voxels.txt"
export RACER_REFERENCE_COVERAGE=0.756492045455
export RACER_COVERAGE_RELATIVE_TOLERANCE=0.10

exec "${workspace}/scripts/run_reproduce_75pct_legacy.sh" --run
