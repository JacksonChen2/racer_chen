#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_15uav_five_sites_ideal_startup_recovery_100s_${run_id}}"
case_dir="${suite}/ideal_perfect_communication"
layout="${workspace}/config/warehouse_full_15uav_five_sites_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
active_child=""

mkdir -p "${case_dir}"
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

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${layout}")"
if [[ "$(wc -w <<<"${starts}")" -ne 45 ||
      "$(jq -r '.drone_count' "${layout}")" -ne 15 ]]; then
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix racer_original_core)" != "${workspace}/install/racer_original_core" ]]; then
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi

python3 - "${suite}/experiment_manifest.json" "${layout}" "${scene_usd}" <<'PY'
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
layout_path = Path(sys.argv[2])
layout = json.loads(layout_path.read_text())
manifest = {
    "algorithm": "pairwise_robust_racer_with_startup_recovery",
    "case": "ideal_perfect_communication",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[3],
    "layout": layout["layout"],
    "layout_selection_file": str(layout_path),
    "start_positions": layout["start_positions"],
    "controlled_parameters": {
        "drone_count": 15,
        "duration_s": 100,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "random_seed": 42,
        "communication_mode": "ideal",
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
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export ROS_DOMAIN_ID="${RACER_IDEAL_DOMAIN_ID:-89}"
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=100
export RACER_FIDELITY_DRONE_COUNT=15
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
export RACER_START_POSITIONS="${starts}"
export RACER_SCENE_USD="${scene_usd}"
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
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_15uav_startup_recovery_${run_id}_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_15uav_five_sites_ideal_startup_recovery_100s"

printf '%s\n' "running:ideal_perfect_communication" >"${suite}/run_state.txt"
set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
active_child=$!
printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
wait "${active_child}"
runner_status=$?
active_child=""
set -e
printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

python3 - "${case_dir}" "${runner_status}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
runner_status = int(sys.argv[2])
files = list(case_dir.glob("*_result.json"))
summary = {"runner_status": runner_status, "result_found": len(files) == 1}
if len(files) == 1:
    result = json.loads(files[0].read_text())
    metrics = result.get("metrics", {})
    statistics = result.get("communication", {}).get("statistics", {})
    summary.update({
        "elapsed_s": metrics.get("elapsed"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "total_path_m": sum(metrics.get("path_lengths", [])),
        "collision_events": metrics.get("collision_events"),
        "executed_drone_ids": result.get("executed_drone_ids", []),
        "attempted_packets": statistics.get("attempted_packets"),
        "delivered_packets": statistics.get("delivered_packets"),
        "startup_recovery": metrics.get("startup_recovery"),
    })
(case_dir.parent / "suite_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
PY

if [[ -f "${case_dir}/warehouse_full_distributed_result.json" ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s\n' "${runner_status}" >"${suite}/run_state.txt"
fi
