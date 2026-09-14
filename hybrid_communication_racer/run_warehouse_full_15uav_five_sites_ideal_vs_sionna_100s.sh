#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-100}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_15uav_five_sites_pairwise_robust_ideal_vs_sionna_100hz_76800rays_${duration_s}s_${run_id}}"
layout="${RACER_LAYOUT_FILE:-${workspace}/config/warehouse_full_15uav_five_sites_layout.json}"
drone_count="$(jq -r '.drone_count' "${layout}")"
scene_query_rate_hz="${RACER_SCENE_QUERY_RATE_HZ:-20}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""

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
  printf 'STOPPED time=%s\n' "$(date --iso-8601=seconds)"
  exit 130
}
trap stop_experiment INT TERM HUP

actual_scene_sha256="$(sha256sum "${scene_usd}" | awk '{print $1}')"
actual_sionna_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_sionna_sha256}" != "${expected_sionna_sha256}" ]]; then
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  printf 'Scene hash mismatch: USD=%s Sionna=%s\n' \
    "${actual_scene_sha256}" "${actual_sionna_sha256}" >&2
  exit 2
fi
if [[ ! -d "${sionna_runtime}/sionna" ]]; then
  printf '%s\n' "blocked:missing_sionna_runtime" >"${suite}/run_state.txt"
  exit 2
fi

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${layout}")"
if ! [[ "${drone_count}" =~ ^[1-9][0-9]*$ ]] ||
      [[ "$(wc -w <<<"${starts}")" -ne "$((3 * drone_count))" ||
      "$(jq -r '.validation_ok // true' "${layout}")" != "true" ]]; then
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
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

python3 - "${suite}/experiment_manifest.json" "${layout}" "${scene_usd}" \
  "${actual_scene_sha256}" "${sionna_scene_xml}" "${actual_sionna_sha256}" \
  "${duration_s}" "${scene_query_rate_hz}" "${drone_count}" <<'PY'
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
layout_path = Path(sys.argv[2])
layout = json.loads(layout_path.read_text())
duration_s = int(sys.argv[7])
scene_query_rate_hz = float(sys.argv[8])
drone_count = int(sys.argv[9])
manifest = {
    "algorithm": "pairwise_robust_racer",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[3],
    "scene_usd_sha256": sys.argv[4],
    "sionna_scene_xml": sys.argv[5],
    "sionna_scene_xml_sha256": sys.argv[6],
    "layout": layout["layout"],
    "layout_selection_file": str(layout_path),
    "layout_selection_rule": layout["selection_rule"],
    "regions": layout["regions"],
    "start_positions": layout["start_positions"],
    "minimum_start_spacing_m": layout["minimum_global_spacing_m"],
    "controlled_parameters": {
        "drone_count": drone_count,
        "duration_s": duration_s,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": scene_query_rate_hz,
        "trigger_minimum_cloud_frames": 0,
        "random_seed": 42,
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
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${duration_s}"
export RACER_FIDELITY_DRONE_COUNT="${drone_count}"
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
# The Isaac runner already waits until every point-cloud topic has all expected
# subscribers before advancing simulation.  Avoid making the single-threaded
# trigger node deserialize all full 76,800-ray clouds merely as a second barrier.
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_SCENE_QUERY_RATE_HZ="${scene_query_rate_hz}"
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${starts}"
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
  export RACER_LKH_DIR="/tmp/racer_pairwise_robust_${drone_count}uav_${run_id}_${case_name}_lkh"
  export RACER_ALGORITHM_LABEL="pairwise_robust_${drone_count}uav_${case_name}_100hz_76800rays_${duration_s}s"

  if [[ "${communication_mode}" == "sionna" ]]; then
    export RACER_REQUIRE_SIONNA=true
    export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
    export RACER_UAV_TX_POWER_DBM=20
    export RACER_FIXED_MCS_INDEX=14
  else
    export RACER_REQUIRE_SIONNA=false
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
  python3 - "${case_dir}" "${communication_mode}" "${duration_s}" "${drone_count}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_mode = sys.argv[2]
duration_s = float(sys.argv[3])
drone_count = int(sys.argv[4])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
statistics = communication.get("statistics", {})
evidence = result.get("algorithm_evidence", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < duration_s - 1.0:
    errors.append(f"simulation did not reach {duration_s:g} seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, drone_count + 1)):
    errors.append(f"not all {drone_count} UAV algorithms executed")
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
  last_runner_status="${runner_status}"
  last_validation_status="${validation_status}"
  return 0
}

run_case ideal_perfect_communication ideal "${RACER_IDEAL_DOMAIN_ID:-87}"
ideal_runner_status="${last_runner_status}"
ideal_validation_status="${last_validation_status}"
sleep 30
run_case sionna_actual_communication sionna "${RACER_SIONNA_DOMAIN_ID:-88}"
sionna_runner_status="${last_runner_status}"
sionna_validation_status="${last_validation_status}"

python3 - "${suite}" "${ideal_runner_status}" "${ideal_validation_status}" \
  "${sionna_runner_status}" "${sionna_validation_status}" <<'PY'
import json
from pathlib import Path
import sys

suite = Path(sys.argv[1])
summary = {
    "ideal_runner_status": int(sys.argv[2]),
    "ideal_validation_status": int(sys.argv[3]),
    "sionna_runner_status": int(sys.argv[4]),
    "sionna_validation_status": int(sys.argv[5]),
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
        "logical_delivery_ratio": statistics.get("logical_delivery_ratio"),
    }
(suite / "suite_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
PY

if [[ "${ideal_validation_status}" -eq 0 && "${sionna_validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error ideal_validation=%s sionna_validation=%s\n' \
    "${ideal_validation_status}" "${sionna_validation_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s ideal_runner=%s ideal_validation=%s sionna_runner=%s sionna_validation=%s\n' \
  "$(date --iso-8601=seconds)" "${ideal_runner_status}" "${ideal_validation_status}" \
  "${sionna_runner_status}" "${sionna_validation_status}"
