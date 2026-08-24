#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"
suite="${workspace}/experiments/warehouse_full_15uav_sionna_original_comm_no_retries_50hz_1200s_20260823_190126"
case_dir="${suite}/sionna_distributed_original_racer_no_link_retries"
mkdir -p "${case_dir}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"

export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=1200
export RACER_FIDELITY_DRONE_COUNT=15
export RACER_PHYSICS_RATE_HZ=50
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_SENSOR_WORKER_COUNT=8
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER=300
export RACER_WALL_TIME_GRACE_SECONDS=600
export RACER_RANDOM_SEED=42

# Preserve original RACER application-layer behavior: pair-opt requests and
# responses are each published ten times, states/trajectories are periodic,
# and missing map chunks are repaired by the chunk-stamp exchange. Disable
# only the added communication-proxy PER/ARQ retries.
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

export RACER_START_POSITIONS="\
-7.8 16.2 0.8  -7.8 15.0 1.5  -6.6 15.0 2.2  -6.6 16.2 1.15  \
-6.6 17.4 1.85 -7.8 17.4 0.8  -9.0 17.4 1.5  -9.0 16.2 2.2   \
-14.49 14.91 0.8 -13.2 14.91 1.5 -11.91 14.91 2.2 -14.29 16.2 1.5 \
-13.2 16.2 0.8 -11.91 16.2 1.5 -13.2 17.49 2.2"

export RACER_SCENE_USD="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
export RACER_SIONNA_SCENE_XML="${suite}/sionna_scene/warehouse.xml"
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_warehouse_full_15uav_20260823_190126_sionna_original_comm_lkh"
export RACER_ALGORITHM_LABEL="warehouse_full_15uav_sionna_original_racer_comm_no_link_retries_50hz"
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_NETWORK_TOPOLOGY=distributed

set +e
"${workspace}/run_warehouse_simple_sionna.sh"
status=$?
set -e
printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
exit "${status}"
