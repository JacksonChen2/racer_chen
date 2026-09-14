#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_five_sites_ideal_vs_sionna_startup_recovery_600s_${run_id}}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
layout_selection="${workspace}/config/warehouse_full_10uav_five_sites_layout.json"
active_child=""

# Five regions, two UAVs per region, from the single canonical layout.
start_positions="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${layout_selection}")"

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "preflight_checks" >"${suite}/run_state.txt"

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

if [[ "$(sha256sum "${scene_usd}" | awk '{print $1}')" != "${expected_scene_sha256}" ||
      "$(wc -w <<<"${start_positions}")" -ne 30 ||
      ! -f "${layout_selection}" ||
      ! -f "${sionna_scene_xml}" || ! -d "${sionna_runtime}/sionna" ]]; then
  printf '%s\n' "blocked:invalid_scene_layout_or_sionna" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix racer_original_core)" != "${workspace}/install/racer_original_core" ||
      "$(ros2 pkg prefix racer_isaac_adapter)" != "${workspace}/install/racer_isaac_adapter" ]]; then
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi

python3 - "${suite}/experiment_manifest.json" "${scene_usd}" "${sionna_scene_xml}" "${start_positions}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[4].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "algorithm": "pairwise_robust_racer_with_startup_recovery",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[2],
    "sionna_scene_xml": sys.argv[3],
    "layout": "five_requested_sites_two_collision_checked_uavs_each",
    "start_positions": starts,
    "minimum_start_spacing_m": min(
        math.dist(left, right)
        for left, right in itertools.combinations(starts, 2)
    ),
    "controlled_parameters": {
        "drone_count": 10,
        "duration_s": 600,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "random_seed": 42,
    },
    "startup_recovery": {
        "maximum_free_space_yaw": True,
        "yaw_sweep_samples": 72,
        "stationary_scan_degrees": 360,
        "stationary_scan_duration_s": 8.0,
        "truth_checked_unknown_corridor_distance_m": 0.8,
        "truth_checked_unknown_corridor_speed_mps": 0.25,
        "post_corridor_settle_duration_s": 1.0,
        "exploration_trigger_delay_s": 14.0,
        "truth_query_changes_planning_map": False,
        "idle_shared_map_wakeup_interval_s": 0.5,
    },
    "cases": [
        {
            "name": "ideal_perfect_communication",
            "communication_mode": "ideal",
            "perfect_lossless_broadcast": True,
        },
        {
            "name": "sionna_actual_communication",
            "communication_mode": "sionna",
            "require_sionna": True,
            "radio_map_cache_enabled": False,
            "uav_tx_power_dbm": 20,
            "fixed_mcs_index": 14,
            "max_retries": 0,
        },
    ],
}
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=600
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_TRIGGER_DELAY_S=14.0
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_STARTUP_FREE_SPACE_YAW=1
export RACER_STARTUP_SCAN_DURATION=8.0
export RACER_STARTUP_UNKNOWN_CORRIDOR_DISTANCE=0.8
export RACER_STARTUP_CORRIDOR_SPEED=0.25
export RACER_STARTUP_SETTLE_DURATION=1.0
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${start_positions}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

last_runner_status=0
run_case() {
  local case_name="$1"
  local mode="$2"
  local domain_id="$3"
  local case_dir="${suite}/${case_name}"
  local runner_status

  mkdir -p "${case_dir}"
  printf 'running:%s\n' "${case_name}" >"${suite}/run_state.txt"
  export ROS_DOMAIN_ID="${domain_id}"
  export RACER_COMMUNICATION_MODE="${mode}"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/racer_pairwise_robust_10uav_600s_${run_id}_${case_name}_lkh"
  export RACER_ALGORITHM_LABEL="pairwise_robust_10uav_five_sites_${case_name}_startup_recovery_600s"

  if [[ "${mode}" == "sionna" ]]; then
    export RACER_REQUIRE_SIONNA=true
    export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
    export RACER_UAV_TX_POWER_DBM=20
    export RACER_FIXED_MCS_INDEX=14
  else
    export RACER_REQUIRE_SIONNA=false
    unset RACER_SIONNA_RADIO_MAP_CACHE RACER_UAV_TX_POWER_DBM RACER_FIXED_MCS_INDEX
  fi

  printf 'START case=%s mode=%s time=%s domain=%s\n' \
    "${case_name}" "${mode}" "$(date --iso-8601=seconds)" "${domain_id}"
  set +e
  "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  runner_status=$?
  active_child=""
  set -e
  printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"
  last_runner_status="${runner_status}"
  printf 'FINISH case=%s time=%s runner=%s\n' \
    "${case_name}" "$(date --iso-8601=seconds)" "${runner_status}"
}

run_case ideal_perfect_communication ideal "${RACER_IDEAL_DOMAIN_ID:-91}"
ideal_runner_status="${last_runner_status}"
printf '%s\n' "cleanup_between_cases" >"${suite}/run_state.txt"
sleep 30
run_case sionna_actual_communication sionna "${RACER_SIONNA_DOMAIN_ID:-92}"
sionna_runner_status="${last_runner_status}"

python3 - "${suite}" "${ideal_runner_status}" "${sionna_runner_status}" <<'PY'
import json
from pathlib import Path
import sys

suite = Path(sys.argv[1])
summary = {
    "ideal_runner_status": int(sys.argv[2]),
    "sionna_runner_status": int(sys.argv[3]),
    "cases": {},
}
for case_dir in sorted(path for path in suite.iterdir() if path.is_dir()):
    result_files = list(case_dir.glob("*_result.json"))
    if len(result_files) != 1:
        continue
    result = json.loads(result_files[0].read_text())
    metrics = result.get("metrics", {})
    statistics = result.get("communication", {}).get("statistics", {})
    summary["cases"][case_dir.name] = {
        "coverage": metrics.get("mapping_coverage_joint"),
        "total_path_m": sum(metrics.get("path_lengths", [])),
        "collision_events": metrics.get("collision_events"),
        "executed_drone_ids": result.get("executed_drone_ids"),
        "communication_mode": result.get("communication", {}).get("mode"),
        "attempted_packets": statistics.get("attempted_packets"),
        "delivered_packets": statistics.get("delivered_packets"),
    }
(suite / "suite_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
PY

if [[ -f "${suite}/ideal_perfect_communication/warehouse_full_distributed_result.json" &&
      -f "${suite}/sionna_actual_communication/warehouse_full_distributed_result.json" ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error ideal_runner=%s sionna_runner=%s\n' \
    "${ideal_runner_status}" "${sionna_runner_status}" >"${suite}/run_state.txt"
fi
