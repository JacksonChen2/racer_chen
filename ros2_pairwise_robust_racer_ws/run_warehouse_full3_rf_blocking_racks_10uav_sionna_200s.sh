#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-200}"
communication_mode="${RACER_EXPERIMENT_COMMUNICATION_MODE:-sionna}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full3_rf_blocking_racks_10uav_two_sides_${communication_mode}_100hz_76800rays_${duration_s}s_${run_id}}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full3.usd"
rf_overlay_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full3_rf_blocking_racks.usda"
sionna_scene_dir="${workspace}/../warehouse_scenes/sionna/warehouse_full3_roof_center_bs_20260826_220000"
sionna_scene_xml="${sionna_scene_dir}/warehouse.xml"
sionna_metal_mesh="${sionna_scene_dir}/meshes/warehouse_metal.ply"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="ca91a374fe639afe644b8321d2625307620c8546c15a26cf4369dc51195dc7a5"
expected_overlay_sha256="bcdff84122bde8159c9132503d1bf91692b6bfb6c96d1850707892e7cfe11c42"
expected_sionna_xml_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
expected_sionna_metal_sha256="22143d42cb69c2f04b8187b4191d4b4862ca951234db46b0eb2e8d3e707ce70c"
active_child=""

if [[ "${communication_mode}" != "sionna" && "${communication_mode}" != "ideal" ]]; then
  printf 'RACER_EXPERIMENT_COMMUNICATION_MODE must be sionna or ideal.\n' >&2
  exit 2
fi

# Two open aisles separated by one RF-blocking rack row.  UAV 1 is shifted
# 0.24 m from the warehouse_full baseline to clear the added metal barrier.
start_positions="\
-14.25 14.91 0.80  -13.20 14.91 1.50  -11.91 14.91 2.20  \
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
actual_overlay_sha256="$(sha256sum "${rf_overlay_usd}" | awk '{print $1}')"
actual_sionna_xml_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
actual_sionna_metal_sha256="$(sha256sum "${sionna_metal_mesh}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_overlay_sha256}" != "${expected_overlay_sha256}" ||
      "${actual_sionna_xml_sha256}" != "${expected_sionna_xml_sha256}" ||
      "${actual_sionna_metal_sha256}" != "${expected_sionna_metal_sha256}" ]]; then
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
  "${actual_scene_sha256}" "${rf_overlay_usd}" "${actual_overlay_sha256}" \
  "${sionna_scene_xml}" "${actual_sionna_xml_sha256}" \
  "${sionna_metal_mesh}" "${actual_sionna_metal_sha256}" \
  "${start_positions}" "${duration_s}" "${communication_mode}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[10].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
communication_mode = sys.argv[12]
communication = {
    "mode": communication_mode,
    "network_topology": "distributed",
}
if communication_mode == "sionna":
    communication.update({
        "require_sionna": True,
        "radio_map_cache_enabled": False,
        "uav_tx_power_dbm": 20,
        "fixed_mcs_index": 14,
        "max_retries": 0,
    })
else:
    communication["perfect_lossless_broadcast"] = True

manifest = {
    "algorithm": "pairwise_robust_racer",
    "scene": "warehouse_full3_rf_blocking_racks",
    "isaac_composite_scene_usd": sys.argv[2],
    "isaac_composite_scene_sha256": sys.argv[3],
    "rf_blocking_overlay_usd": sys.argv[4],
    "rf_blocking_overlay_sha256": sys.argv[5],
    "sionna_scene_xml": sys.argv[6],
    "sionna_scene_xml_sha256": sys.argv[7],
    "sionna_metal_mesh": sys.argv[8],
    "sionna_metal_mesh_sha256": sys.argv[9],
    "sionna_rf_blocking_material": "RFBlockingRackMetal",
    "sionna_rf_blocking_rack_rows": 7,
    "layout": "ten_uavs_two_adjacent_aisles_5_5",
    "start_positions": starts,
    "minimum_start_spacing_m": min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ),
    "takeoff_groups": [
        {"name": "left_aisle", "drone_ids": list(range(1, 6)), "start_positions": starts[:5]},
        {"name": "right_aisle", "drone_ids": list(range(6, 11)), "start_positions": starts[5:]},
    ],
    "controlled_parameters": {
        "drone_count": 10,
        "duration_s": int(sys.argv[11]),
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": 50,
        "random_seed": 42,
    },
    "communication": communication,
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

export RACER_COMMUNICATION_MODE="${communication_mode}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

if [[ "${communication_mode}" == "sionna" ]]; then
  case_name="sionna_actual_communication"
  export RACER_REQUIRE_SIONNA=true
  export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
  export RACER_UAV_TX_POWER_DBM=20
  export RACER_FIXED_MCS_INDEX=14
else
  case_name="ideal_perfect_communication"
  export RACER_REQUIRE_SIONNA=false
  unset RACER_SIONNA_RADIO_MAP_CACHE RACER_UAV_TX_POWER_DBM RACER_FIXED_MCS_INDEX
fi

case_dir="${suite}/${case_name}"
mkdir -p "${case_dir}"
printf 'running:%s\n' "${case_name}" >"${suite}/run_state.txt"
export ROS_DOMAIN_ID="${RACER_FULL3_SIONNA_DOMAIN_ID:-197}"
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_full3_rf_blocking_10uav_${run_id}_${communication_mode}_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_full3_rf_blocking_10uav_${communication_mode}_100hz_76800rays_${duration_s}s"

printf 'START case=%s time=%s domain=%s\n' \
  "${case_name}" "$(date --iso-8601=seconds)" "${ROS_DOMAIN_ID}"
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
python3 - "${case_dir}" "${duration_s}" "${sionna_scene_xml}" \
  "${communication_mode}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
duration_s = float(sys.argv[2])
expected_sionna_scene = str(Path(sys.argv[3]).resolve())
expected_mode = sys.argv[4]
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
if communication.get("mode") != expected_mode:
    errors.append(f"communication mode was not {expected_mode}")
if communication.get("network_topology") != "distributed":
    errors.append("distributed topology was not active")
if expected_mode == "sionna":
    if communication.get("sionna_ready") is not True:
        errors.append("Sionna RT was not active")
    if int(communication.get("exact_link_samples", 0)) <= 0:
        errors.append("Sionna produced no exact link samples")
    if int(stats.get("sionna_direct_attempted_packets", 0)) <= 0:
        errors.append("no packets were attempted through Sionna")
    if str(Path(result.get("sionna_scene", "")).resolve()) != expected_sionna_scene:
        errors.append("result used the wrong Sionna scene")
else:
    attempted = int(stats.get("attempted_packets", 0))
    delivered = int(stats.get("delivered_packets", -1))
    drops = sum(int(stats.get(name, 0)) for name in (
        "dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"
    ))
    if attempted <= 0 or delivered != attempted or drops != 0:
        errors.append("ideal communication was not perfectly lossless")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    f"INTEGRITY_OK warehouse_full3_rf_blocking_racks_{expected_mode} "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"attempted={stats.get('attempted_packets')} "
    f"delivered={stats.get('delivered_packets')}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

python3 - "${case_dir}" "${suite}/suite_summary.json" \
  "${runner_status}" "${validation_status}" "${communication_mode}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
files = list(case_dir.glob("*_result.json"))
summary = {
    "runner_status": int(sys.argv[3]),
    "validation_status": int(sys.argv[4]),
    "case": f"warehouse_full3_rf_blocking_racks_{sys.argv[5]}",
}
if len(files) == 1:
    result = json.loads(files[0].read_text())
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    stats = communication.get("statistics", {})
    evidence = result.get("algorithm_evidence", {})
    summary.update({
        "elapsed_s": metrics.get("elapsed"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "total_path_m": sum(metrics.get("path_lengths", [])),
        "collision_events": metrics.get("collision_events"),
        "min_obstacle_clearance_m": metrics.get("min_obstacle_clearance"),
        "min_inter_drone_distance_m": metrics.get("min_inter_drone"),
        "safety_interventions": metrics.get("safety_interventions"),
        "executed_drone_ids": result.get("executed_drone_ids"),
        "process_crashes": evidence.get("process_crashes"),
        "sionna_ready": communication.get("sionna_ready"),
        "exact_link_samples": communication.get("exact_link_samples"),
        "attempted_packets": stats.get("attempted_packets"),
        "delivered_packets": stats.get("delivered_packets"),
        "logical_delivery_ratio": stats.get("logical_delivery_ratio"),
        "dropped_no_link": stats.get("dropped_no_link"),
        "dropped_per": stats.get("dropped_per"),
    })
Path(sys.argv[2]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
PY

if [[ "${validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error validation=%s\n' "${validation_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s runner=%s validation=%s\n' \
  "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
