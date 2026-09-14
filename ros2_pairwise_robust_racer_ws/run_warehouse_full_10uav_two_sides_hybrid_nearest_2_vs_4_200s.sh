#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-200}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_two_sides_central_rack_5_5_hybrid_lossless_nearest_2_vs_4_100hz_76800rays_${duration_s}s_${run_id}}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""

# UAV 1-5 are west of the central rack and UAV 6-10 are east of it.
start_positions="\
-14.49 14.91 0.80  -13.20 14.91 1.50  -11.91 14.91 2.20  \
-11.91 16.20 1.15  -13.20 16.20 1.85  \
-7.80 15.00 0.80  -6.60 15.00 1.50  -7.80 16.20 2.20  \
-6.60 16.20 1.15  -7.80 17.40 1.85"

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
  "${actual_scene_sha256}" "${sionna_scene_xml}" "${actual_sionna_sha256}" \
  "${start_positions}" "${duration_s}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[6].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "algorithm": "pairwise_robust_racer",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[2],
    "scene_usd_sha256": sys.argv[3],
    "sionna_scene_xml": sys.argv[4],
    "sionna_scene_xml_sha256": sys.argv[5],
    "layout": "ten_uavs_two_sides_central_rack_5_5",
    "start_positions": starts,
    "minimum_start_spacing_m": min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ),
    "takeoff_groups": [
        {"name": "left_of_central_rack", "drone_ids": list(range(1, 6)), "start_positions": starts[:5]},
        {"name": "right_of_central_rack", "drone_ids": list(range(6, 11)), "start_positions": starts[5:]},
    ],
    "controlled_parameters": {
        "drone_count": 10,
        "duration_s": int(sys.argv[7]),
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": 50,
        "random_seed": 42,
    },
    "cases": [
        {
            "name": "hybrid_lossless_nearest_2_sionna_others",
            "communication_mode": "sionna",
            "lossless_nearest_neighbor_count": 2,
            "other_links": "sionna",
        },
        {
            "name": "hybrid_lossless_nearest_4_sionna_others",
            "communication_mode": "sionna",
            "lossless_nearest_neighbor_count": 4,
            "other_links": "sionna",
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
export RACER_FIDELITY_DURATION="${duration_s}"
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=1
export RACER_SCENE_QUERY_RATE_HZ=50
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
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
export RACER_UAV_TX_POWER_DBM=20
export RACER_FIXED_MCS_INDEX=14
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

last_runner_status=0
last_validation_status=0
run_case() {
  local nearest_count="$1"
  local domain_id="$2"
  local case_name="hybrid_lossless_nearest_${nearest_count}_sionna_others"
  local case_dir="${suite}/${case_name}"
  local runner_status
  local validation_status

  mkdir -p "${case_dir}"
  printf 'running:%s\n' "${case_name}" >"${suite}/run_state.txt"
  export ROS_DOMAIN_ID="${domain_id}"
  export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT="${nearest_count}"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/racer_pairwise_robust_10uav_two_sides_${run_id}_nearest_${nearest_count}_lkh"
  export RACER_ALGORITHM_LABEL="pairwise_robust_10uav_two_sides_nearest_${nearest_count}_lossless_sionna_others_100hz_76800rays_${duration_s}s"

  printf 'START case=%s time=%s domain=%s\n' \
    "${case_name}" "$(date --iso-8601=seconds)" "${domain_id}"
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
  python3 - "${case_dir}" "${duration_s}" "${nearest_count}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
duration_s = float(sys.argv[2])
nearest_count = int(sys.argv[3])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
evidence = result.get("algorithm_evidence", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < duration_s - 1.0:
    errors.append(f"simulation did not reach {duration_s:g} seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if int(evidence.get("lkh_call_failures", 0)) != 0:
    errors.append("an LKH call failed")
if communication.get("mode") != "sionna" or communication.get("sionna_ready") is not True:
    errors.append("Sionna RT was not active")
if communication.get("network_topology") != "distributed":
    errors.append("distributed topology was not active")
if int(communication.get("lossless_nearest_neighbor_count", -1)) != nearest_count:
    errors.append("lossless nearest-neighbor count did not match the case")
if int(communication.get("exact_link_samples", 0)) <= 0:
    errors.append("Sionna produced no exact link samples")
if int(stats.get("lossless_nearest_forwarded_packets", 0)) <= 0:
    errors.append("the lossless nearest-neighbor path forwarded no packets")
if int(stats.get("sionna_direct_attempted_packets", 0)) <= 0:
    errors.append("the remaining links sent no packets through Sionna")
if int(stats.get("lossless_nearest_position_unavailable_events", 0)) != 0:
    errors.append("nearest-neighbor selection lacked UAV positions")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    f"INTEGRITY_OK nearest={nearest_count} "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"lossless={stats.get('lossless_nearest_forwarded_packets')} "
    f"sionna={stats.get('sionna_direct_attempted_packets')}"
)
PY
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'FINISH case=%s time=%s runner=%s validation=%s\n' \
    "${case_name}" "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
  last_runner_status="${runner_status}"
  last_validation_status="${validation_status}"
}

run_case 2 "${RACER_NEAREST_2_DOMAIN_ID:-194}"
nearest_2_runner_status="${last_runner_status}"
nearest_2_validation_status="${last_validation_status}"
sleep 30
run_case 4 "${RACER_NEAREST_4_DOMAIN_ID:-195}"
nearest_4_runner_status="${last_runner_status}"
nearest_4_validation_status="${last_validation_status}"

python3 - "${suite}" "${nearest_2_runner_status}" "${nearest_2_validation_status}" \
  "${nearest_4_runner_status}" "${nearest_4_validation_status}" <<'PY'
import json
from pathlib import Path
import sys

suite = Path(sys.argv[1])
summary = {
    "nearest_2_runner_status": int(sys.argv[2]),
    "nearest_2_validation_status": int(sys.argv[3]),
    "nearest_4_runner_status": int(sys.argv[4]),
    "nearest_4_validation_status": int(sys.argv[5]),
    "cases": {},
}
for case_dir in sorted(path for path in suite.iterdir() if path.is_dir()):
    files = list(case_dir.glob("*_result.json"))
    if len(files) != 1:
        continue
    result = json.loads(files[0].read_text())
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    stats = communication.get("statistics", {})
    evidence = result.get("algorithm_evidence", {})
    summary["cases"][case_dir.name] = {
        "elapsed_s": metrics.get("elapsed"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "total_path_m": sum(metrics.get("path_lengths", [])),
        "collision_events": metrics.get("collision_events"),
        "min_obstacle_clearance_m": metrics.get("min_obstacle_clearance"),
        "min_inter_drone_distance_m": metrics.get("min_inter_drone"),
        "safety_interventions": metrics.get("safety_interventions"),
        "executed_drone_ids": result.get("executed_drone_ids"),
        "process_crashes": evidence.get("process_crashes"),
        "lossless_nearest_neighbor_count": communication.get("lossless_nearest_neighbor_count"),
        "attempted_packets": stats.get("attempted_packets"),
        "delivered_packets": stats.get("delivered_packets"),
        "logical_delivery_ratio": stats.get("logical_delivery_ratio"),
        "lossless_nearest_forwarded_packets": stats.get("lossless_nearest_forwarded_packets"),
        "sionna_direct_attempted_packets": stats.get("sionna_direct_attempted_packets"),
    }
(suite / "suite_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
PY

if [[ "${nearest_2_validation_status}" -eq 0 && "${nearest_4_validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error nearest_2_validation=%s nearest_4_validation=%s\n' \
    "${nearest_2_validation_status}" "${nearest_4_validation_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s nearest_2_runner=%s nearest_2_validation=%s nearest_4_runner=%s nearest_4_validation=%s\n' \
  "$(date --iso-8601=seconds)" "${nearest_2_runner_status}" "${nearest_2_validation_status}" \
  "${nearest_4_runner_status}" "${nearest_4_validation_status}"
