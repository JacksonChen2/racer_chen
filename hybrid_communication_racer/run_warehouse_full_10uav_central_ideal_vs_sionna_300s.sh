#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-300}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_central_pairwise_robust_ideal_vs_sionna_100hz_76800rays_${duration_s}s_${run_id}}"
assignment_mode="${RACER_EXPLORATION_ASSIGNMENT_MODE:-original}"
layout_tag="${RACER_LAYOUT_TAG:-ten_uavs_two_sides_central_rack_5_5}"
scene_query_rate_hz="${RACER_SCENE_QUERY_RATE_HZ:-50}"
sionna_uav_tx_power_dbm="${RACER_SIONNA_UAV_TX_POWER_DBM:-20}"
sionna_fixed_mcs_index="${RACER_SIONNA_FIXED_MCS_INDEX:-14}"
sionna_max_retries="${RACER_SIONNA_MAX_RETRIES:-0}"
sionna_bandwidth_hz="${RACER_SIONNA_BANDWIDTH_HZ:-100000000}"
sionna_resource_blocks="${RACER_SIONNA_RESOURCE_BLOCKS:-66}"
sionna_initial_assignment_perfect_delivery="${RACER_SIONNA_INITIAL_ASSIGNMENT_PERFECT_DELIVERY:-false}"
task_metric_observer_mode="${RACER_TASK_METRIC_OBSERVER_MODE:-inline}"
trigger_minimum_cloud_frames="${RACER_TRIGGER_MINIMUM_CLOUD_FRAMES:-1}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""

# Five collision-checked UAVs in each aisle immediately beside the centre rack
# row at x≈-10.5 m.  UAV 1-5 are west/left of the rack and UAV 6-10 are
# east/right of it.  All points passed a 0.45 m start-box query and a 0.40 m
# vertical-corridor query through z=2.75 m in the exact warehouse USD.
default_start_positions="\
-14.49 14.91 0.80  -13.20 14.91 1.50  -11.91 14.91 2.20  \
-11.91 16.20 1.15  -13.20 16.20 1.85  \
-7.80 15.00 0.80  -6.60 15.00 1.50  -7.80 16.20 2.20  \
-6.60 16.20 1.15  -7.80 17.40 1.85"
start_positions="${RACER_START_POSITIONS:-${default_start_positions}}"

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "queued" >"${suite}/run_state.txt"

stop_experiment() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${suite}/run_state.txt"
  exit 130
}
trap stop_experiment INT TERM HUP

# When queued behind another experiment, also pin its Linux start time so a
# recycled PID cannot accidentally keep this suite waiting.
wait_pid="${RACER_WAIT_FOR_PID:-}"
if [[ -n "${wait_pid}" && -r "/proc/${wait_pid}/stat" ]]; then
  wait_start_time="$(awk '{print $22}' "/proc/${wait_pid}/stat")"
  printf 'WAITING pid=%s start_time=%s since=%s\n' \
    "${wait_pid}" "${wait_start_time}" "$(date --iso-8601=seconds)"
  while [[ -r "/proc/${wait_pid}/stat" ]]; do
    current_start_time="$(awk '{print $22}' "/proc/${wait_pid}/stat" 2>/dev/null || true)"
    [[ "${current_start_time}" == "${wait_start_time}" ]] || break
    printf 'waiting_for_pid:%s\n' "${wait_pid}" >"${suite}/run_state.txt"
    sleep 15
  done
  printf 'PREVIOUS_EXPERIMENT_FINISHED time=%s\n' "$(date --iso-8601=seconds)"
  # Give Isaac/Fast DDS children time to complete their normal teardown.
  sleep 30
fi

printf '%s\n' "preflight_checks" >"${suite}/run_state.txt"
actual_scene_sha256="$(sha256sum "${scene_usd}" | awk '{print $1}')"
actual_sionna_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_sionna_sha256}" != "${expected_sionna_sha256}" ]]; then
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  exit 2
fi
if [[ ! -d "${sionna_runtime}/sionna" ]]; then
  printf '%s\n' "blocked:missing_sionna_runtime" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "$(wc -w <<<"${start_positions}")" -ne 30 ]]; then
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${sionna_initial_assignment_perfect_delivery}" != "true" &&
      "${sionna_initial_assignment_perfect_delivery}" != "false" ]]; then
  printf '%s\n' "blocked:invalid_initial_assignment_mode" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix racer_original_core)" != "${workspace}/install/racer_original_core" ||
      "$(ros2 pkg prefix racer_fidelity_msgs)" != "${workspace}/install/racer_fidelity_msgs" ]]; then
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi

python3 - "${suite}/experiment_manifest.json" "${scene_usd}" \
  "${actual_scene_sha256}" "${sionna_scene_xml}" \
  "${actual_sionna_sha256}" "${start_positions}" "${duration_s}" \
  "${layout_tag}" "${scene_query_rate_hz}" "${assignment_mode}" \
  "${sionna_uav_tx_power_dbm}" "${sionna_fixed_mcs_index}" \
  "${sionna_max_retries}" "${sionna_bandwidth_hz}" \
  "${sionna_resource_blocks}" \
  "${sionna_initial_assignment_perfect_delivery}" \
  "${task_metric_observer_mode}" "${trigger_minimum_cloud_frames}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[6].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
duration_s = int(sys.argv[7])
layout_tag = sys.argv[8]
scene_query_rate_hz = float(sys.argv[9])
assignment_mode = sys.argv[10]
sionna_uav_tx_power_dbm = float(sys.argv[11])
sionna_fixed_mcs_index = int(sys.argv[12])
sionna_max_retries = int(sys.argv[13])
sionna_bandwidth_hz = float(sys.argv[14])
sionna_resource_blocks = int(sys.argv[15])
sionna_initial_assignment_perfect_delivery = sys.argv[16] == "true"
task_metric_observer_mode = sys.argv[17]
trigger_minimum_cloud_frames = int(sys.argv[18])
center = (-10.02891489217081, 14.888611215255622)
manifest = {
    "algorithm": (
        "hybrid_communication_racer"
        if assignment_mode == "global_cooperative"
        else "pairwise_robust_racer"
    ),
    "exploration_assignment_mode": assignment_mode,
    "hybrid_global_state_channel": (
        "direct_perfect_global_information"
        if assignment_mode == "global_cooperative"
        else "disabled"
    ),
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[2],
    "scene_usd_sha256": sys.argv[3],
    "sionna_scene_xml": sys.argv[4],
    "sionna_scene_xml_sha256": sys.argv[5],
    "layout": layout_tag,
    "layout_source": "PhysX-validated left and right aisles beside centre rack x=-10.5 m",
    "takeoff_groups": [
        {"name": "left_of_central_rack", "drone_ids": list(range(1, 6)), "start_positions": starts[:5]},
        {"name": "right_of_central_rack", "drone_ids": list(range(6, 11)), "start_positions": starts[5:]},
    ],
    "start_positions": starts,
    "start_centroid": [
        sum(point[axis] for point in starts) / len(starts)
        for axis in range(3)
    ],
    "maximum_xy_distance_from_scene_center_m": max(
        math.dist(point[:2], center) for point in starts
    ),
    "minimum_start_spacing_m": min(
        math.dist(left, right)
        for left, right in itertools.combinations(starts, 2)
    ),
    "controlled_parameters": {
        "drone_count": 10,
        "duration_s": duration_s,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": scene_query_rate_hz,
        "random_seed": 42,
        "task_metric_observer_mode": task_metric_observer_mode,
        "trigger_minimum_cloud_frames": trigger_minimum_cloud_frames,
    },
    "cases": [
        {
            "name": "ideal_perfect_communication",
            "communication_mode": "ideal",
            "network_topology": "distributed",
            "perfect_lossless_broadcast": True,
        },
        {
            "name": "sionna_actual_communication",
            "communication_mode": "sionna",
            "network_topology": "distributed",
            "ap_relay_enabled": False,
            "lossless_link_overrides": False,
            "require_sionna": True,
            "radio_map_cache_enabled": False,
            "uav_tx_power_dbm": sionna_uav_tx_power_dbm,
            "bandwidth_hz": sionna_bandwidth_hz,
            "resource_blocks": sionna_resource_blocks,
            "fixed_mcs_index": sionna_fixed_mcs_index,
            "max_retries": sionna_max_retries,
            "initial_assignment_perfect_delivery": (
                sionna_initial_assignment_perfect_delivery
            ),
        },
    ],
}
Path(sys.argv[1]).write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n"
)
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${duration_s}"
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES="${trigger_minimum_cloud_frames}"
export RACER_SCENE_QUERY_RATE_HZ="${scene_query_rate_hz}"
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_COVERAGE_UPDATE_RATE_HZ=10.0
export RACER_TASK_METRIC_OBSERVER_MODE="${task_metric_observer_mode}"
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${start_positions}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_EXPLORATION_ASSIGNMENT_MODE="${assignment_mode}"
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_BANDWIDTH_HZ="${sionna_bandwidth_hz}"
export RACER_RESOURCE_BLOCKS="${sionna_resource_blocks}"
export RACER_BS_MAX_RETRIES="${sionna_max_retries}"

last_runner_status=0
last_validation_status=0
run_case() {
  local case_name="$1"
  local communication_mode="$2"
  local domain_id="$3"
  local case_dir="${suite}/${case_name}"
  local runner_status
  local validation_status

  mkdir -p "${case_dir}"
  printf 'running:%s\n' "${case_name}" >"${suite}/run_state.txt"
  export ROS_DOMAIN_ID="${domain_id}"
  export RACER_COMMUNICATION_MODE="${communication_mode}"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/racer_${assignment_mode}_central_10uav_${run_id}_${case_name}_lkh"
  export RACER_ALGORITHM_LABEL="${assignment_mode}_central_10uav_${case_name}_100hz_76800rays_${duration_s}s"

  if [[ "${communication_mode}" == "sionna" ]]; then
    export RACER_REQUIRE_SIONNA=true
    export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
    export RACER_UAV_TX_POWER_DBM="${sionna_uav_tx_power_dbm}"
    export RACER_FIXED_MCS_INDEX="${sionna_fixed_mcs_index}"
    export RACER_MAX_RETRIES="${sionna_max_retries}"
    export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY="${sionna_initial_assignment_perfect_delivery}"
  else
    export RACER_REQUIRE_SIONNA=false
    export RACER_MAX_RETRIES=0
    export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false
    unset RACER_SIONNA_RADIO_MAP_CACHE RACER_UAV_TX_POWER_DBM RACER_FIXED_MCS_INDEX
  fi

  printf 'START case=%s mode=%s time=%s domain=%s\n' \
    "${case_name}" "${communication_mode}" "$(date --iso-8601=seconds)" "${domain_id}"
  set +e
  "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  runner_status=$?
  active_child=""
  set -e
  printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

  set +e
  python3 - "${case_dir}" "${communication_mode}" "${duration_s}" \
    "${sionna_initial_assignment_perfect_delivery}" \
    "${sionna_uav_tx_power_dbm}" "${sionna_fixed_mcs_index}" \
    "${sionna_max_retries}" "${sionna_bandwidth_hz}" \
    "${sionna_resource_blocks}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_mode = sys.argv[2]
expected_duration = float(sys.argv[3])
expect_initial_assignment_perfect = sys.argv[4] == "true"
expected_tx_power_dbm = float(sys.argv[5])
expected_fixed_mcs = int(sys.argv[6])
expected_max_retries = int(sys.argv[7])
expected_bandwidth_hz = float(sys.argv[8])
expected_resource_blocks = int(sys.argv[9])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
statistics = communication.get("statistics", {})
evidence = result.get("algorithm_evidence", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < expected_duration - 1.0:
    errors.append(f"simulation did not reach {expected_duration:g} seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if communication.get("mode") != expected_mode:
    errors.append(f"communication mode is not {expected_mode}")
if expected_mode == "ideal":
    attempted = int(statistics.get("attempted_packets", 0))
    delivered = int(statistics.get("delivered_packets", 0))
    drops = sum(int(statistics.get(name, 0)) for name in (
        "dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"
    ))
    if attempted <= 0 or delivered != attempted or drops != 0:
        errors.append("ideal communication was not perfectly lossless")
else:
    if communication.get("sionna_ready") is not True:
        errors.append("Sionna RT was not active")
    if int(communication.get("exact_link_samples", 0)) <= 0:
        errors.append("Sionna produced no exact link samples")
    phy = statistics.get("phy", {})
    if float(phy.get("uav_tx_power_dbm", -1)) != expected_tx_power_dbm:
        errors.append("unexpected Sionna UAV transmit power")
    if int(phy.get("fixed_mcs_index", -1)) != expected_fixed_mcs:
        errors.append("unexpected Sionna fixed MCS")
    if int(phy.get("max_retries", -1)) != expected_max_retries:
        errors.append("unexpected Sionna retry setting")
    if float(phy.get("bandwidth_hz", -1)) != expected_bandwidth_hz:
        errors.append("unexpected Sionna bandwidth")
    if int(phy.get("resource_blocks", -1)) != expected_resource_blocks:
        errors.append("unexpected Sionna resource-block count")
    if expect_initial_assignment_perfect:
        if statistics.get("initial_assignment_perfect_delivery_enabled") is not True:
            errors.append("initial-assignment perfect delivery was not enabled")
        if statistics.get("initial_assignment_perfect_delivery_complete") is not True:
            errors.append("initial-assignment perfect window did not close")
        if int(statistics.get("initial_assignment_perfect_epoch", 0)) != 1:
            errors.append("initial-assignment epoch was not one")
        if int(statistics.get("initial_assignment_perfect_acks", 0)) != 10:
            errors.append("not all ten UAVs acknowledged the initial assignment")
        if int(statistics.get("sionna_direct_attempted_packets", 0)) <= 0:
            errors.append("no channel-gated traffic occurred after initialization")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"attempted={statistics.get('attempted_packets')} "
    f"delivered={statistics.get('delivered_packets')}"
)
PY
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'FINISH case=%s time=%s runner=%s validation=%s\n' \
    "${case_name}" "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
  # The generic wrapper can return 1 when optional acceptance criteria (for
  # example returning to the start point) are unmet even though this paired
  # experiment's result and communication integrity checks pass.  Keep the
  # case status as data instead of returning nonzero under errexit, otherwise
  # the second matched case would never start.
  last_runner_status="${runner_status}"
  last_validation_status="${validation_status}"
  return 0
}

if [[ "${RACER_SKIP_IDEAL:-0}" == "1" ]]; then
  ideal_runner_status="$(cat "${suite}/ideal_perfect_communication/runner_exit_status.txt")"
  ideal_validation_status="$(cat "${suite}/ideal_perfect_communication/case_validation_status.txt")"
  printf 'RESUME skip_completed_ideal validation=%s time=%s\n' \
    "${ideal_validation_status}" "$(date --iso-8601=seconds)"
else
  run_case ideal_perfect_communication ideal "${RACER_IDEAL_DOMAIN_ID:-85}"
  ideal_runner_status="${last_runner_status}"
  ideal_validation_status="${last_validation_status}"
  sleep 30
fi
run_case sionna_actual_communication sionna "${RACER_SIONNA_DOMAIN_ID:-86}"
sionna_runner_status="${last_runner_status}"
sionna_validation_status="${last_validation_status}"

python3 - "${suite}/suite_summary.json" "${ideal_runner_status}" \
  "${ideal_validation_status}" "${sionna_runner_status}" \
  "${sionna_validation_status}" <<'PY'
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
summary = {
    "ideal_runner_status": int(sys.argv[2]),
    "ideal_validation_status": int(sys.argv[3]),
    "sionna_runner_status": int(sys.argv[4]),
    "sionna_validation_status": int(sys.argv[5]),
    "all_integrity_checks_passed": sys.argv[3] == "0" and sys.argv[5] == "0",
}
output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
PY

if [[ "${ideal_validation_status}" -eq 0 && "${sionna_validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error ideal_validation=%s sionna_validation=%s\n' \
    "${ideal_validation_status}" "${sionna_validation_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s ideal_runner=%s ideal_validation=%s sionna_runner=%s sionna_validation=%s\n' \
  "$(date --iso-8601=seconds)" "${ideal_runner_status}" \
  "${ideal_validation_status}" "${sionna_runner_status}" \
  "${sionna_validation_status}"
