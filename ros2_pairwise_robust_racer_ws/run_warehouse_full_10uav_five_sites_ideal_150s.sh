#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration_s="${RACER_EXPERIMENT_DURATION_S:-150}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_five_sites_pairwise_robust_ideal_100hz_76800rays_${duration_s}s_${run_id}}"
case_dir="${suite}/ideal_no_loss_pairwise_robust"
selection="${workspace}/config/warehouse_full_10uav_five_sites_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/warehouse.xml"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
active_child=""

mkdir -p "${case_dir}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

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
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ]]; then
  printf 'Scene changed after start-position validation: expected=%s actual=%s\n' \
    "${expected_scene_sha256}" "${actual_scene_sha256}" >&2
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  exit 2
fi

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf 'Expected exactly 10 XYZ start positions in %s\n' "${selection}" >&2
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
core_prefix="$(ros2 pkg prefix racer_original_core)"
msgs_prefix="$(ros2 pkg prefix racer_fidelity_msgs)"
if [[ "${core_prefix}" != "${workspace}/install/racer_original_core" || \
      "${msgs_prefix}" != "${workspace}/install/racer_fidelity_msgs" ]]; then
  printf 'Wrong ROS overlay: core=%s messages=%s\n' "${core_prefix}" "${msgs_prefix}" >&2
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi

python3 - "${suite}/experiment_manifest.json" "${selection}" "${scene_usd}" \
  "${actual_scene_sha256}" "${duration_s}" <<'PY'
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
selection_path = Path(sys.argv[2])
scene_path = Path(sys.argv[3])
selection = json.loads(selection_path.read_text())
manifest = {
    "algorithm": "pairwise_robust_racer",
    "workspace": str(output.parents[2]),
    "scene": "warehouse_full_with_industrial_ap_user_modified_20260825",
    "scene_usd": str(scene_path),
    "scene_usd_sha256": sys.argv[4],
    "layout": "five_good_uav_bs_sites_near_four_corners_and_center_two_uavs_each",
    "layout_selection_file": str(selection_path),
    "regions": selection["regions"],
    "start_positions": selection["start_positions"],
    "drone_count": 10,
    "duration_s": int(sys.argv[5]),
    "physics_rate_hz": 100,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 76800,
    "sensor_worker_count": 8,
    "random_seed": 42,
    "communication": {
        "mode": "ideal",
        "network_topology": "distributed",
        "perfect_lossless_broadcast": True,
        "nearest_neighbor_limit": 0,
        "lossless_range_m": 0.0,
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_DOMAIN_ID="${RACER_DISCOVERY_DOMAIN_ID:-77}"
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
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_SCENE_QUERY_RATE_HZ=20
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
export RACER_COMMUNICATION_MODE=ideal
export RACER_REQUIRE_SIONNA=false
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_10uav_five_sites_ideal_${duration_s}s_${run_id}_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_racer_10uav_five_sites_ideal_100hz_76800rays_${duration_s}s"

printf '%s\n' "running" >"${suite}/run_state.txt"
printf 'START time=%s mode=ideal drones=10 duration=%s physics_hz=100 rays=76800 domain=%s\n' \
  "$(date --iso-8601=seconds)" "${duration_s}" "${ROS_DOMAIN_ID}"
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
duration_s = float(sys.argv[2])
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
if float(metrics.get("physics_rate_hz", 0.0)) != 100.0:
    errors.append("physics rate is not 100 Hz")
if int(metrics.get("camera_ray_budget", 0)) != 76800:
    errors.append("camera ray budget is not 76800")
if communication.get("mode") != "ideal":
    errors.append("communication mode is not ideal")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", 0))
drops = sum(int(stats.get(name, 0)) for name in
            ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"))
if attempted <= 0 or delivered != attempted or drops != 0:
    errors.append("ideal transport was not fully lossless")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"attempted={attempted} delivered={delivered}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

if [[ "${runner_status}" -eq 0 && "${validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s validation=%s\n' \
    "${runner_status}" "${validation_status}" >"${suite}/run_state.txt"
fi
printf 'FINISH time=%s runner=%s validation=%s\n' \
  "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
exit "${runner_status}"
