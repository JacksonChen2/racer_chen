#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-200}"
drone_count=15
nearest_count=4
scene_profile="${RACER_EXPERIMENT_SCENE_PROFILE:-warehouse_full3_rf_blocking_racks}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/${scene_profile}_15uav_three_sites_nearest4_ideal_100hz_76800rays_${duration_s}s_${run_id}}"
active_child=""

if [[ "${scene_profile}" == "warehouse_full3_rf_blocking_racks" ]]; then
  scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full3.usd"
  rf_overlay_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full3_rf_blocking_racks.usda"
  sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full3_roof_center_bs_20260826_220000/warehouse.xml"
  sionna_metal_mesh="${workspace}/../warehouse_scenes/sionna/warehouse_full3_roof_center_bs_20260826_220000/meshes/warehouse_metal.ply"
  expected_scene_sha256="ca91a374fe639afe644b8321d2625307620c8546c15a26cf4369dc51195dc7a5"
  expected_overlay_sha256="bcdff84122bde8159c9132503d1bf91692b6bfb6c96d1850707892e7cfe11c42"
  expected_sionna_xml_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
  expected_sionna_metal_sha256="22143d42cb69c2f04b8187b4191d4b4862ca951234db46b0eb2e8d3e707ce70c"
  first_uav_x="-14.25"
elif [[ "${scene_profile}" == "warehouse_full_with_industrial_ap" ]]; then
  scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
  rf_overlay_usd=""
  sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
  sionna_metal_mesh="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/meshes/warehouse_metal.ply"
  expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
  expected_overlay_sha256=""
  expected_sionna_xml_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
  expected_sionna_metal_sha256="82ab23fa7d8c07c89881cd6e27541465c9452f296faf660e638e1d4d745ac448"
  first_uav_x="-14.49"
else
  printf 'Unsupported RACER_EXPERIMENT_SCENE_PROFILE: %s\n' "${scene_profile}" >&2
  exit 2
fi

start_positions="\
${first_uav_x} 14.91 0.80  -13.20 14.91 1.50  -11.91 14.91 2.20  \
-11.91 16.20 1.15  -13.20 16.20 1.85  \
-7.80 15.00 0.80  -6.60 15.00 1.50  -7.80 16.20 2.20  \
-6.60 16.20 1.15  -7.80 17.40 1.85  \
-3.75 16.60 0.80  -2.45 15.30 1.50  -2.45 16.60 2.20  \
-3.75 14.00 1.15  -3.75 15.30 1.85"

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
actual_overlay_sha256=""
if [[ -n "${rf_overlay_usd}" ]]; then
  actual_overlay_sha256="$(sha256sum "${rf_overlay_usd}" | awk '{print $1}')"
fi
actual_sionna_xml_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
actual_sionna_metal_sha256="$(sha256sum "${sionna_metal_mesh}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_overlay_sha256}" != "${expected_overlay_sha256}" ||
      "${actual_sionna_xml_sha256}" != "${expected_sionna_xml_sha256}" ||
      "${actual_sionna_metal_sha256}" != "${expected_sionna_metal_sha256}" ]]; then
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "$(wc -w <<<"${start_positions}")" -ne 45 ]]; then
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
  "${start_positions}" "${duration_s}" "${nearest_count}" \
  "${scene_profile}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[10].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "algorithm": "pairwise_robust_racer",
    "scene": sys.argv[13],
    "isaac_composite_scene_usd": sys.argv[2],
    "isaac_composite_scene_sha256": sys.argv[3],
    "rf_blocking_overlay_usd": sys.argv[4],
    "rf_blocking_overlay_sha256": sys.argv[5],
    "sionna_scene_xml": sys.argv[6],
    "sionna_scene_xml_sha256": sys.argv[7],
    "sionna_metal_mesh": sys.argv[8],
    "sionna_metal_mesh_sha256": sys.argv[9],
    "layout": "fifteen_uavs_three_adjacent_aisles_5_5_5",
    "start_positions": starts,
    "minimum_start_spacing_m": min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ),
    "takeoff_groups": [
        {"name": "left_aisle", "drone_ids": list(range(1, 6)), "start_positions": starts[:5]},
        {"name": "center_aisle", "drone_ids": list(range(6, 11)), "start_positions": starts[5:10]},
        {"name": "right_aisle", "drone_ids": list(range(11, 16)), "start_positions": starts[10:]},
    ],
    "controlled_parameters": {
        "drone_count": 15,
        "duration_s": int(sys.argv[11]),
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": 50,
        "trigger_minimum_cloud_frames": 0,
        "random_seed": 42,
    },
    "communication": {
        "mode": "ideal",
        "network_topology": "nearest_neighbors",
        "nearest_neighbor_count": int(sys.argv[12]),
        "selected_links": "perfect_lossless",
        "all_other_receivers": "filtered_before_transport",
        "selection_distance": "dynamic_3d_euclidean",
    },
}
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${duration_s}"
export RACER_FIDELITY_DRONE_COUNT="${drone_count}"
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
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

export RACER_COMMUNICATION_MODE=ideal
export RACER_REQUIRE_SIONNA=false
export RACER_NETWORK_TOPOLOGY=nearest_neighbors
export RACER_NEAREST_NEIGHBOR_COUNT="${nearest_count}"
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

case_dir="${suite}/ideal_nearest4_communication"
mkdir -p "${case_dir}"
printf '%s\n' "running:ideal_nearest4_communication" >"${suite}/run_state.txt"
export ROS_DOMAIN_ID="${RACER_15UAV_NEAREST4_DOMAIN_ID:-199}"
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_${scene_profile}_15uav_three_sites_${run_id}_nearest4_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_${scene_profile}_15uav_three_sites_ideal_nearest4_100hz_76800rays_${duration_s}s"

printf 'START case=ideal_nearest4_communication time=%s domain=%s\n' \
  "$(date --iso-8601=seconds)" "${ROS_DOMAIN_ID}"
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
python3 - "${case_dir}" "${duration_s}" "${drone_count}" \
  "${nearest_count}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
duration_s = float(sys.argv[2])
drone_count = int(sys.argv[3])
nearest_count = int(sys.argv[4])
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
if sorted(result.get("executed_drone_ids", [])) != list(range(1, drone_count + 1)):
    errors.append("not all fifteen UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if int(evidence.get("lkh_call_failures", 0)) != 0:
    errors.append("an LKH call failed")
if communication.get("mode") != "ideal":
    errors.append("communication mode was not ideal")
if communication.get("network_topology") != "nearest_neighbors":
    errors.append("nearest-neighbor topology was not active")
configured_nearest_count = communication.get("nearest_neighbor_count")
if configured_nearest_count is None:
    configured_nearest_count = stats.get("nearest_neighbor_count", -1)
if int(configured_nearest_count) != nearest_count:
    errors.append("nearest-neighbor count did not equal four")
attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", -1))
drops = sum(int(stats.get(name, 0)) for name in (
    "dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"
))
if attempted <= 0 or delivered != attempted or drops != 0:
    errors.append("selected nearest-neighbor links were not perfectly lossless")
if int(stats.get("nearest_filtered_receivers", 0)) <= 0:
    errors.append("non-nearest receivers were not filtered")
if int(stats.get("nearest_position_unavailable", 0)) != 0:
    errors.append("nearest-neighbor selection lacked UAV positions")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK 15uav_nearest4 "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"attempted={attempted} delivered={delivered} "
    f"nearest_filtered={stats.get('nearest_filtered_receivers')}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

python3 - "${case_dir}" "${suite}/suite_summary.json" \
  "${runner_status}" "${validation_status}" "${scene_profile}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
files = list(case_dir.glob("*_result.json"))
summary = {
    "runner_status": int(sys.argv[3]),
    "validation_status": int(sys.argv[4]),
    "case": f"{sys.argv[5]}_15uav_nearest4_ideal",
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
        "network_topology": communication.get("network_topology"),
        "nearest_neighbor_count": (
            communication.get("nearest_neighbor_count")
            if communication.get("nearest_neighbor_count") is not None
            else stats.get("nearest_neighbor_count")
        ),
        "attempted_packets": stats.get("attempted_packets"),
        "delivered_packets": stats.get("delivered_packets"),
        "nearest_filtered_receivers": stats.get("nearest_filtered_receivers"),
        "logical_delivery_ratio": stats.get("logical_delivery_ratio"),
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
