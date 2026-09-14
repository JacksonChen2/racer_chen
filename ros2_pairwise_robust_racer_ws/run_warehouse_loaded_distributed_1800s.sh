#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"

export RACER_COMMUNICATION_MODE="${RACER_COMMUNICATION_MODE:-sionna}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_FIDELITY_DURATION="${RACER_FIDELITY_DURATION:-1800}"
export RACER_FIDELITY_DRONE_COUNT="${RACER_FIDELITY_DRONE_COUNT:-5}"
export RACER_FIDELITY_SCENARIO=warehouse_loaded
export RACER_FIDELITY_HEADLESS="${RACER_FIDELITY_HEADLESS:-1}"
export RACER_FIDELITY_VISUALIZE="${RACER_FIDELITY_VISUALIZE:-0}"
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET="${RACER_MAPPING_COVERAGE_TARGET:-0}"

# Match the formal BS run so that the communication topology is the only change.
export RACER_PHYSICS_RATE_HZ="${RACER_PHYSICS_RATE_HZ:-200}"
export RACER_SENSOR_RATE_HZ="${RACER_SENSOR_RATE_HZ:-30}"
export RACER_DEPTH_WIDTH="${RACER_DEPTH_WIDTH:-640}"
export RACER_DEPTH_HEIGHT="${RACER_DEPTH_HEIGHT:-480}"
export RACER_CAMERA_RAY_BUDGET="${RACER_CAMERA_RAY_BUDGET:-76800}"
export RACER_RECORD_TRAJECTORY_HISTORY="${RACER_RECORD_TRAJECTORY_HISTORY:-1}"
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-40}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"

export RACER_RESULT_DIR="${RACER_RESULT_DIR:-${workspace_dir}/experiments/warehouse_loaded_distributed_${run_id}}"
export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}"
export SIONNA_RUNTIME_DIR="${SIONNA_RUNTIME_DIR:-/home/jackson/racer2/RACER/ros2_3d_sionna_ws/.sionna_runtime}"

exec "${workspace_dir}/run_warehouse_simple_sionna.sh"
