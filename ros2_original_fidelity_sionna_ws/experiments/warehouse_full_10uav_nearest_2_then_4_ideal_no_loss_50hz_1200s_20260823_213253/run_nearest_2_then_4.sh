#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"
suite="${workspace}/experiments/warehouse_full_10uav_nearest_2_then_4_ideal_no_loss_50hz_1200s_20260823_213253"
scene_usd="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/experiments/warehouse_full_10uav_center_racer_layout_ideal_vs_sionna_no_retries_50hz_1200s_20260823_203213/sionna_scene/warehouse.xml"
active_child=""

printf '%s\n' "$$" >"${suite}/supervisor.pid"

stop_active_case() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "interrupted" >"${suite}/active_case.txt"
  exit 130
}
trap stop_active_case INT TERM HUP

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

# Preserve the current experiment's physical, sensing, layout, and random-seed
# settings. Only the communication topology and nearest-neighbor count vary.
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
export RACER_START_POSITIONS="\
-7.8 16.2 0.8  -7.8 15.0 1.5  -6.6 15.0 2.2  -6.6 16.2 1.15 \
-6.6 17.4 1.85 -7.8 17.4 0.8  -9.0 17.4 1.5  -9.0 16.2 2.2  \
-9.0 15.0 1.15 -7.8 11.0 1.85"

export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

validate_case_result() {
  local case_dir="$1"
  local expected_mode="$2"
  local expected_topology="$3"
  local expected_lossless_count="$4"
  local expected_range_m="$5"
  local expected_delivery="$6"

  python3 - "${case_dir}" "${expected_mode}" "${expected_topology}" \
    "${expected_lossless_count}" "${expected_range_m}" \
    "${expected_delivery}" <<'PY'
import json
import math
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_mode = sys.argv[2]
expected_topology = sys.argv[3]
expected_lossless_count = int(sys.argv[4])
expected_range_m = float(sys.argv[5])
expected_delivery = sys.argv[6]
result_files = list(case_dir.glob("*_result.json"))
if len(result_files) != 1:
    raise SystemExit(f"expected one result JSON, found {len(result_files)}")

result = json.loads(result_files[0].read_text())
communication = result.get("communication", {})
stats = communication.get("statistics", {})
metrics = result.get("metrics", {})
acceptance = result.get("acceptance", {})
errors = []

if communication.get("mode") != expected_mode:
    errors.append("unexpected communication mode")
if communication.get("network_topology") != expected_topology:
    errors.append("unexpected network topology")
if not acceptance.get("isaac_exit_ok", False):
    errors.append("Isaac did not exit normally")
if result.get("algorithm_evidence", {}).get("process_crashes", 0) != 0:
    errors.append("a ROS process crashed")
if float(metrics.get("elapsed", 0.0)) < 1199.0:
    errors.append("simulation did not reach the requested duration")

if expected_delivery == "sionna_with_lossless_nearest":
    if communication.get("lossless_nearest_neighbor_count") != expected_lossless_count:
        errors.append("requested lossless-neighbor count mismatch")
    if stats.get("lossless_nearest_neighbor_count") != expected_lossless_count:
        errors.append("active lossless-neighbor count mismatch")
    if int(stats.get("lossless_nearest_forwarded_packets", 0)) <= 0:
        errors.append("no lossless nearest-neighbor forwarding was observed")
    if int(stats.get("sionna_direct_attempted_packets", 0)) <= 0:
        errors.append("non-nearest receivers did not enter the Sionna path")
    exact_samples = (
        int(stats.get("sionna_exact_samples", 0))
        + int(stats.get("sionna_cache_corrected_samples", 0))
    )
    if exact_samples <= 0:
        errors.append("no exact or cache-corrected Sionna samples were observed")
    if not acceptance.get("lossless_nearest_override_active", False):
        errors.append("runner did not recognize the lossless-nearest override")
elif expected_delivery == "none":
    actual_range = float(communication.get("communication_range_m", math.nan))
    if not math.isclose(actual_range, expected_range_m, rel_tol=0.0, abs_tol=1e-18):
        errors.append("blackout range mismatch")
    if int(stats.get("attempted_packets", 0)) != 0:
        errors.append("inter-UAV packet attempts were observed")
    if int(stats.get("delivered_packets", 0)) != 0:
        errors.append("inter-UAV packet delivery was observed")
    if int(stats.get("range_filtered_receivers", 0)) <= 0:
        errors.append("zero-range receiver filtering was not observed")
else:
    errors.append("unknown expected-delivery policy")

(case_dir / "scientific_acceptance_passed.txt").write_text(
    ("true" if result.get("passed") else "false") + "\n"
)
if errors:
    for error in errors:
        print(f"VALIDATION_ERROR {error}", file=sys.stderr)
    raise SystemExit(1)
print(
    f"VALIDATION_OK topology={expected_topology} "
    f"delivery={expected_delivery} scientific_passed={result.get('passed')}"
)
PY
}

run_case() {
  local case_name="$1"
  local communication_mode="$2"
  local require_sionna="$3"
  local topology="$4"
  local lossless_nearest_count="$5"
  local communication_range_m="$6"
  local expected_delivery="$7"
  local ros_domain_id="$8"
  local case_dir="${suite}/${case_name}"
  local status
  local validation_status

  mkdir -p "${case_dir}"
  export ROS_DOMAIN_ID="${ros_domain_id}"
  export RACER_COMMUNICATION_MODE="${communication_mode}"
  export RACER_REQUIRE_SIONNA="${require_sionna}"
  export RACER_NETWORK_TOPOLOGY="${topology}"
  export RACER_NEAREST_NEIGHBOR_COUNT=0
  export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT="${lossless_nearest_count}"
  export RACER_COMMUNICATION_RANGE_M="${communication_range_m}"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/racer_warehouse_full_10uav_20260823_213253_${case_name}_lkh"
  export RACER_ALGORITHM_LABEL="warehouse_full_10uav_center_racer_layout_${case_name}_50hz"

  printf '%s\n' "${case_name}" >"${suite}/active_case.txt"
  printf 'START case=%s mode=%s topology=%s lossless_nearest=%s range_m=%s domain=%s time=%s\n' \
    "${case_name}" "${communication_mode}" "${topology}" \
    "${lossless_nearest_count}" "${communication_range_m}" \
    "${ros_domain_id}" "$(date --iso-8601=seconds)"

  set +e
  "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  status=$?
  active_child=""
  set -e

  printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
  set +e
  validate_case_result "${case_dir}" "${communication_mode}" "${topology}" \
    "${lossless_nearest_count}" "${communication_range_m}" \
    "${expected_delivery}" \
    >"${case_dir}/case_validation.log" 2>&1
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'END case=%s topology=%s runner_status=%s validation_status=%s time=%s\n' \
    "${case_name}" "${topology}" "${status}" "${validation_status}" \
    "$(date --iso-8601=seconds)"
  if (( validation_status != 0 )); then
    printf 'failed:%s\n' "${case_name}" >"${suite}/active_case.txt"
    return "${validation_status}"
  fi
}

run_case sionna_with_nearest_2_lossless sionna true distributed 2 4.0 \
  sionna_with_lossless_nearest 43
sleep 5
run_case sionna_with_nearest_4_lossless sionna true distributed 4 4.0 \
  sionna_with_lossless_nearest 44
sleep 5
run_case no_uav_communication ideal false distance_radius 0 1e-12 none 45

printf '%s\n' "complete" >"${suite}/active_case.txt"
printf 'SUITE_COMPLETE time=%s\n' "$(date --iso-8601=seconds)"
