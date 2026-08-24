#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"

export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/jiazheng/software/isaacsim}"
export SIONNA_RUNTIME_DIR="${SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-183}"
export RACER_COMMUNICATION_MODE=sionna
export RACER_NETWORK_TOPOLOGY=bs_round_robin
export RACER_FIDELITY_DURATION=900
export RACER_FIDELITY_DRONE_COUNT=5
export RACER_FIDELITY_SCENARIO=warehouse_loaded_full
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_PHYSICS_RATE_HZ=1000
export RACER_SENSOR_RATE_HZ=30
export RACER_DEPTH_WIDTH=640
export RACER_DEPTH_HEIGHT=480
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-80}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_ALGORITHM_LABEL=original_racer_bs_round_robin_sionna_rt_full_factory
export RACER_LKH_DIR="/tmp/racer_warehouse_loaded_full_bs_rr_${run_id}_lkh"
export RACER_RESULT_DIR="${RACER_RESULT_DIR:-${workspace_dir}/experiments/warehouse_loaded_full_bs_rr_${run_id}_900s}"

exec "${workspace_dir}/run_warehouse_simple_sionna.sh"
