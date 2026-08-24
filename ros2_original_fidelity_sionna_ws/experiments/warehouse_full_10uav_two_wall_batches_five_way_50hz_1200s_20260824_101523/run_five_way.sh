#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_two_wall_batches_five_way_50hz_1200s_20260824_101523}"
layout_tag="${RACER_LAYOUT_TAG:-two_wall}"
active_child=""

printf '%s\n' "$$" >"${suite}/supervisor.pid"

stop_active_case() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "interrupted" >"${suite}/active_case.txt"
  printf '%s\n' "stopped" >"${suite}/queue_state.txt"
  exit 130
}
trap stop_active_case INT TERM HUP

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
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
export RACER_START_POSITIONS="${RACER_START_POSITIONS:-\
-12.9 23.4 0.8  -11.7 23.4 1.5  -9.3 23.4 2.2  -8.1 23.4 1.15  -6.8 23.4 1.85 \
-12.9 1.3 0.8   -11.7 1.3 1.5   -10.5 1.3 2.2   -9.3 1.3 1.15   -8.1 1.3 1.85}"

export RACER_SCENE_USD="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
export RACER_SIONNA_SCENE_XML="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

validate_preflight() {
  python3 - "${suite}/_preflight_start_layout/warehouse_full_distributed_isaac.log" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
prefix = "RACER_3D_ISAAC_RESULT "
matches = [line[len(prefix):] for line in path.read_text(errors="replace").splitlines()
           if line.startswith(prefix)]
if not matches:
    raise SystemExit("PREFLIGHT_ERROR missing Isaac result")
metrics = json.loads(matches[-1])
errors = []
if float(metrics.get("elapsed", 0.0)) < 1.9:
    errors.append("preflight did not reach two simulation seconds")
if int(metrics.get("collision_events", -1)) != 0:
    errors.append("collision event detected at the proposed starts")
if int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("physics contact detected at the proposed starts")
if float(metrics.get("min_inter_drone", 0.0)) < 1.0:
    errors.append("minimum start separation fell below 1.0 m")
if float(metrics.get("min_obstacle_clearance", -1.0)) <= 0.0:
    errors.append("proposed starts do not have positive obstacle clearance")
if errors:
    for error in errors:
        print(f"PREFLIGHT_ERROR {error}", file=sys.stderr)
    raise SystemExit(1)
print(
    "PREFLIGHT_OK "
    f"min_inter_drone={metrics['min_inter_drone']:.6f} "
    f"min_obstacle_clearance={metrics['min_obstacle_clearance']:.6f}"
)
PY
}

run_preflight() {
  local preflight_dir="${suite}/_preflight_start_layout"
  mkdir -p "${preflight_dir}"
  printf '%s\n' "preflight_start_layout" >"${suite}/active_case.txt"
  printf '%s\n' "running_preflight" >"${suite}/queue_state.txt"
  set +e
  ROS_DOMAIN_ID=50 \
  RACER_FIDELITY_DURATION=2 \
  RACER_STOP_ON_COMPLETION=0 \
  RACER_RECORD_TRAJECTORY_HISTORY=0 \
  RACER_COMMUNICATION_MODE=ideal \
  RACER_REQUIRE_SIONNA=false \
  RACER_NETWORK_TOPOLOGY=distributed \
  RACER_NEAREST_NEIGHBOR_COUNT=0 \
  RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0 \
  RACER_COMMUNICATION_RANGE_M=4.0 \
  RACER_RESULT_DIR="${preflight_dir}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_10uav_${layout_tag}_preflight_lkh" \
  RACER_ALGORITHM_LABEL="warehouse_full_10uav_${layout_tag}_start_preflight" \
    "${workspace}/run_warehouse_simple_sionna.sh" >"${preflight_dir}/runner.log" 2>&1 &
  active_child=$!
  wait "${active_child}"
  printf '%s\n' "$?" >"${preflight_dir}/runner_exit_status.txt"
  active_child=""
  set -e
  validate_preflight >"${preflight_dir}/preflight_validation.log" 2>&1
  printf '%s\n' "0" >"${preflight_dir}/preflight_validation_status.txt"
}

validate_case() {
  local case_dir="$1"
  local expected_mode="$2"
  local expected_topology="$3"
  local expected_lossless="$4"
  local policy="$5"
  python3 - "${case_dir}" "${expected_mode}" "${expected_topology}" \
    "${expected_lossless}" "${policy}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_mode, expected_topology = sys.argv[2:4]
expected_lossless = int(sys.argv[4])
policy = sys.argv[5]
result_files = list(case_dir.glob("*_result.json"))
if len(result_files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(result_files)}")
result = json.loads(result_files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < 1199.0:
    errors.append("simulation did not reach 1200 s")
if result.get("algorithm_evidence", {}).get("process_crashes", 0) != 0:
    errors.append("a ROS process crashed")
if communication.get("mode") != expected_mode:
    errors.append("communication mode mismatch")
if communication.get("network_topology") != expected_topology:
    errors.append("network topology mismatch")
if communication.get("lossless_nearest_neighbor_count") != expected_lossless:
    errors.append("lossless nearest-neighbor setting mismatch")

attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", 0))
exact = int(stats.get("sionna_exact_samples", 0)) + int(stats.get("sionna_cache_corrected_samples", 0))
if policy == "ideal":
    drops = sum(int(stats.get(name, 0)) for name in
                ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"))
    if attempted <= 0 or delivered != attempted or drops != 0:
        errors.append("ideal transport was not fully lossless")
elif policy == "sionna":
    if attempted <= 0 or exact <= 0:
        errors.append("Sionna transport was not active")
elif policy == "mixed":
    if int(stats.get("lossless_nearest_neighbor_count", 0)) != expected_lossless:
        errors.append("proxy mixed-transport count mismatch")
    if int(stats.get("lossless_nearest_forwarded_packets", 0)) <= 0:
        errors.append("no nearest-neighbor lossless forwarding observed")
    if int(stats.get("sionna_direct_attempted_packets", 0)) <= 0 or exact <= 0:
        errors.append("remaining links did not use Sionna")
elif policy == "blackout":
    if attempted != 0 or delivered != 0:
        errors.append("inter-UAV traffic escaped the blackout gate")
    if int(stats.get("range_filtered_receivers", 0)) <= 0:
        errors.append("blackout receiver filtering was not observed")
else:
    errors.append("unknown validation policy")

(case_dir / "scientific_acceptance_passed.txt").write_text(
    ("true" if result.get("passed") else "false") + "\n"
)
if errors:
    for error in errors:
        print(f"INTEGRITY_ERROR {error}", file=sys.stderr)
    raise SystemExit(1)
print(
    f"INTEGRITY_OK policy={policy} scientific_passed={result.get('passed')} "
    f"collisions={metrics.get('collision_events')} coverage={metrics.get('mapping_coverage_joint')}"
)
PY
}

run_case() {
  local case_name="$1"
  local mode="$2"
  local require_sionna="$3"
  local topology="$4"
  local lossless_count="$5"
  local range_m="$6"
  local policy="$7"
  local domain_id="$8"
  local case_dir="${suite}/${case_name}"
  local runner_status validation_status

  mkdir -p "${case_dir}"
  printf '%s\n' "${case_name}" >"${suite}/active_case.txt"
  printf 'START case=%s time=%s\n' "${case_name}" "$(date --iso-8601=seconds)"
  set +e
  ROS_DOMAIN_ID="${domain_id}" \
  RACER_FIDELITY_DURATION=1200 \
  RACER_STOP_ON_COMPLETION=1 \
  RACER_RECORD_TRAJECTORY_HISTORY=1 \
  RACER_COMMUNICATION_MODE="${mode}" \
  RACER_REQUIRE_SIONNA="${require_sionna}" \
  RACER_NETWORK_TOPOLOGY="${topology}" \
  RACER_NEAREST_NEIGHBOR_COUNT=0 \
  RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT="${lossless_count}" \
  RACER_COMMUNICATION_RANGE_M="${range_m}" \
  RACER_RESULT_DIR="${case_dir}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_10uav_${layout_tag}_${case_name}_lkh" \
  RACER_ALGORITHM_LABEL="warehouse_full_10uav_${layout_tag}_${case_name}_50hz" \
    "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  runner_status=$?
  active_child=""
  set -e
  printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

  set +e
  validate_case "${case_dir}" "${mode}" "${topology}" "${lossless_count}" "${policy}" \
    >"${case_dir}/case_validation.log" 2>&1
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'END case=%s runner_status=%s integrity_status=%s time=%s\n' \
    "${case_name}" "${runner_status}" "${validation_status}" "$(date --iso-8601=seconds)"
  if (( validation_status != 0 )); then
    printf '%s\n' "failed_integrity:${case_name}" >"${suite}/active_case.txt"
    printf '%s\n' "failed_integrity" >"${suite}/queue_state.txt"
    return "${validation_status}"
  fi
}

printf '%s\n' "starting" >"${suite}/queue_state.txt"
run_preflight
printf '%s\n' "running_formal_suite" >"${suite}/queue_state.txt"
sleep 5
run_case ideal_no_loss_original_broadcast ideal false distributed 0 4.0 ideal 51
sleep 5
run_case sionna_distributed_original_comm_no_retries sionna true distributed 0 4.0 sionna 52
sleep 5
run_case sionna_with_nearest_2_lossless sionna true distributed 2 4.0 mixed 53
sleep 5
run_case sionna_with_nearest_4_lossless sionna true distributed 4 4.0 mixed 54
sleep 5
run_case no_uav_communication ideal false distance_radius 0 1e-12 blackout 55

printf '%s\n' "complete" >"${suite}/active_case.txt"
printf '%s\n' "complete" >"${suite}/queue_state.txt"
printf 'SUITE_COMPLETE time=%s\n' "$(date --iso-8601=seconds)"
