#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_DURATION_S:-200}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_two_sides_central_rack_5_5_no_uav_communication_100hz_76800rays_${duration_s}s_${run_id}}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""

# UAV 1-5 are in the aisle west/left of the centre rack row at x≈-10.5 m;
# UAV 6-10 are in the aisle east/right of it.  These are the same PhysX-
# validated starts used by the matched ideal-vs-Sionna 200 s experiment.
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
    "communication": {
        "case": "no_uav_communication",
        "proxy_mode": "ideal",
        "network_topology": "distance_radius",
        "communication_range_m": 1e-12,
        "expected_attempted_packets": 0,
        "expected_delivered_packets": 0,
        "implementation_note": "all inter-UAV receivers are filtered before transport",
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

# A positive epsilon is required by parameter validation.  Since every pair
# starts at least 1.25 m apart, 1e-12 m is an exact blackout for distinct UAVs.
export RACER_COMMUNICATION_MODE=ideal
export RACER_REQUIRE_SIONNA=false
export RACER_NETWORK_TOPOLOGY=distance_radius
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=1e-12
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

case_dir="${suite}/no_uav_communication"
mkdir -p "${case_dir}"
printf '%s\n' "running:no_uav_communication" >"${suite}/run_state.txt"
export ROS_DOMAIN_ID="${RACER_NO_COMM_DOMAIN_ID:-193}"
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_10uav_two_sides_${run_id}_no_comm_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_10uav_two_sides_no_uav_communication_100hz_76800rays_${duration_s}s"

printf 'START case=no_uav_communication time=%s domain=%s\n' \
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
python3 - "${case_dir}" "${duration_s}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_duration = float(sys.argv[2])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
evidence = result.get("algorithm_evidence", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < expected_duration - 1.0:
    errors.append(f"simulation did not reach {expected_duration:g} seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if int(evidence.get("lkh_call_failures", 0)) != 0:
    errors.append("an LKH call failed")
if communication.get("mode") != "ideal":
    errors.append("proxy mode was not ideal")
if communication.get("network_topology") != "distance_radius":
    errors.append("distance-radius blackout topology was not active")
if int(stats.get("attempted_packets", -1)) != 0:
    errors.append("inter-UAV traffic escaped the blackout gate")
if int(stats.get("delivered_packets", -1)) != 0:
    errors.append("an inter-UAV packet was delivered")
if int(stats.get("range_filtered_receivers", 0)) <= 0:
    errors.append("no blackout receiver filtering was observed")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK no_uav_communication "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"range_filtered_receivers={stats.get('range_filtered_receivers')}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

python3 - "${case_dir}" "${suite}/suite_summary.json" \
  "${runner_status}" "${validation_status}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
result_files = list(case_dir.glob("*_result.json"))
summary = {
    "runner_status": int(sys.argv[3]),
    "validation_status": int(sys.argv[4]),
    "case": "no_uav_communication",
}
if len(result_files) == 1:
    result = json.loads(result_files[0].read_text())
    metrics = result.get("metrics", {})
    stats = result.get("communication", {}).get("statistics", {})
    summary.update(
        {
            "elapsed_s": metrics.get("elapsed"),
            "coverage": metrics.get("mapping_coverage_joint"),
            "total_path_m": sum(metrics.get("path_lengths", [])),
            "collision_events": metrics.get("collision_events"),
            "min_obstacle_clearance_m": metrics.get("min_obstacle_clearance"),
            "attempted_packets": stats.get("attempted_packets"),
            "delivered_packets": stats.get("delivered_packets"),
            "range_filtered_receivers": stats.get("range_filtered_receivers"),
            "process_crashes": result.get("algorithm_evidence", {}).get("process_crashes"),
        }
    )
Path(sys.argv[2]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
PY

if [[ "${validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error validation=%s\n' "${validation_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s runner=%s validation=%s\n' \
  "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
