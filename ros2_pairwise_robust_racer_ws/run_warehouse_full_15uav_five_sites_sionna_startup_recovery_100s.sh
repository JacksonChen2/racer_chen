#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_15uav_five_sites_sionna_startup_recovery_100s_${run_id}}"
case_dir="${suite}/sionna_actual_communication"
layout="${workspace}/config/warehouse_full_15uav_five_sites_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
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
      "$(jq -r '.drone_count' "${layout}")" -ne 15 ||
      ! -d "${sionna_runtime}/sionna" || ! -f "${sionna_scene_xml}" ]]; then
  printf '%s\n' "blocked:invalid_inputs_or_missing_sionna" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

python3 - "${suite}/experiment_manifest.json" "${layout}" "${scene_usd}" "${sionna_scene_xml}" <<'PY'
import json
from pathlib import Path
import sys

layout_path = Path(sys.argv[2])
layout = json.loads(layout_path.read_text())
manifest = {
    "algorithm": "pairwise_robust_racer_with_startup_recovery",
    "case": "sionna_actual_communication",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": sys.argv[3],
    "sionna_scene_xml": sys.argv[4],
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
        "communication_mode": "sionna",
        "radio_map_cache_enabled": False,
        "uav_tx_power_dbm": 20,
        "fixed_mcs_index": 14,
        "max_retries": 0,
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
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"
export ROS_DOMAIN_ID="${RACER_SIONNA_DOMAIN_ID:-90}"
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
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_UAV_TX_POWER_DBM=20
export RACER_FIXED_MCS_INDEX=14
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_pairwise_robust_15uav_sionna_startup_recovery_${run_id}_lkh"
export RACER_ALGORITHM_LABEL="pairwise_robust_15uav_five_sites_sionna_startup_recovery_100s"

printf '%s\n' "running:sionna_actual_communication" >"${suite}/run_state.txt"
set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
active_child=$!
printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
wait "${active_child}"
runner_status=$?
active_child=""
set -e
printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

if compgen -G "${case_dir}/*_result.json" >/dev/null; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s\n' "${runner_status}" >"${suite}/run_state.txt"
fi
