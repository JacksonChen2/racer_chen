#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"
suite="${workspace}/experiments/warehouse_full_10uav_center_racer_layout_ideal_vs_sionna_no_retries_50hz_1200s_20260823_203213"
scene_usd="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${suite}/sionna_scene/warehouse.xml"
active_child=""

printf '%s\n' "$$" >"${suite}/supervisor.pid"

stop_active_case() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  exit 130
}
trap stop_active_case INT TERM HUP

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=1200
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=50
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_SENSOR_WORKER_COUNT=8
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER=300
export RACER_WALL_TIME_GRACE_SECONDS=600
export RACER_RANDOM_SEED=42

# RACER's compact central layout: a 3x3 formation plus the tenth UAV on the
# same centerline behind the formation, adapted to the warehouse center aisle.
export RACER_START_POSITIONS="\
-7.8 16.2 0.8  -7.8 15.0 1.5  -6.6 15.0 2.2  -6.6 16.2 1.15 \
-6.6 17.4 1.85 -7.8 17.4 0.8  -9.0 17.4 1.5  -9.0 16.2 2.2  \
-9.0 15.0 1.15 -7.8 11.0 1.85"

export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

run_case() {
  local case_name="$1"
  local communication_mode="$2"
  local require_sionna="$3"
  local ros_domain_id="$4"
  local algorithm_label="$5"
  local case_dir="${suite}/${case_name}"
  local status

  mkdir -p "${case_dir}"
  export ROS_DOMAIN_ID="${ros_domain_id}"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/racer_warehouse_full_10uav_20260823_203213_${case_name}_lkh"
  export RACER_ALGORITHM_LABEL="${algorithm_label}"
  export RACER_COMMUNICATION_MODE="${communication_mode}"
  export RACER_REQUIRE_SIONNA="${require_sionna}"

  printf '%s\n' "${case_name}" >"${suite}/active_case.txt"
  printf 'START case=%s mode=%s domain=%s time=%s\n' \
    "${case_name}" "${communication_mode}" "${ros_domain_id}" "$(date --iso-8601=seconds)"

  set +e
  "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  status=$?
  active_child=""
  set -e

  printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
  printf 'END case=%s status=%s time=%s\n' \
    "${case_name}" "${status}" "$(date --iso-8601=seconds)"
}

run_case \
  ideal_no_loss_original_broadcast \
  ideal false 41 \
  warehouse_full_10uav_center_racer_layout_ideal_no_loss_50hz

sleep 5

run_case \
  sionna_distributed_original_comm_no_link_retries \
  sionna true 42 \
  warehouse_full_10uav_center_racer_layout_sionna_distributed_no_link_retries_50hz

printf '%s\n' "complete" >"${suite}/active_case.txt"
printf 'SUITE_COMPLETE time=%s\n' "$(date --iso-8601=seconds)"
