#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
result_dir="${RACER_RESULT_DIR:-${workspace}/experiments/warehouse_full_5uav_four_corners_center_bs_round_robin_no_retries_50hz_900s_${run_id}}"
active_child=""

mkdir -p "${result_dir}"
printf '%s\n' "$$" >"${result_dir}/supervisor.pid"
printf '%s\n' "starting" >"${result_dir}/run_state.txt"

stop_run() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${result_dir}/run_state.txt"
  exit 130
}
trap stop_run INT TERM HUP

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-63}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=900
export RACER_FIDELITY_DRONE_COUNT=5
export RACER_PHYSICS_RATE_HZ=50
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_SENSOR_WORKER_COUNT=5
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED="${RACER_RANDOM_SEED:-42}"

export RACER_START_POSITIONS="\
-26.6 1.0 0.75  5.6 1.0 0.75  -25.6 29.7 0.75  4.6 29.7 0.75  -11.5 15.6 0.75"
export RACER_SCENE_USD="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
export RACER_SIONNA_SCENE_XML="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"

export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_NETWORK_TOPOLOGY=bs_round_robin
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_BS_TX_POWER_DBM=33
export RACER_UAV_TX_POWER_DBM=23

export RACER_RESULT_DIR="${result_dir}"
export RACER_LKH_DIR="/tmp/racer_warehouse_full_5uav_${run_id}_bs_round_robin_no_retries_lkh"
export RACER_ALGORITHM_LABEL="warehouse_full_5uav_four_corners_center_bs_round_robin_no_retries_50hz"

python3 - "${result_dir}/experiment_manifest.json" \
  "${RACER_RANDOM_SEED}" "${RACER_START_POSITIONS}" <<'PY'
import json
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[3].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "scene": "warehouse_full_with_industrial_ap",
    "layout": "four_corners_plus_center",
    "drone_count": 5,
    "duration_s": 900,
    "physics_rate_hz": 50,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 19200,
    "random_seed": int(sys.argv[2]),
    "start_positions": starts,
    "communication": {
        "mode": "sionna",
        "network_topology": "bs_round_robin",
        "uav_direct_broadcast_preserved": True,
        "bs_uplink_selection": "one UAV per round-robin turn",
        "bs_downlink": "logical multicast of missing cached chunks via per-destination Sionna links",
        "uav_link_max_retries": 0,
        "bs_link_max_retries": 0,
        "bs_tx_power_dbm": 33,
        "uav_tx_power_dbm": 23,
        "bs_position_m": [-10.02891489217081, 14.888611215255622, 7.55],
    },
}
Path(sys.argv[1]).write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n"
)
PY

printf '%s\n' "running" >"${result_dir}/run_state.txt"
printf 'START time=%s mode=sionna topology=bs_round_robin bs_retries=0\n' \
  "$(date --iso-8601=seconds)"
set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${result_dir}/runner.log" 2>&1 &
active_child=$!
printf '%s\n' "${active_child}" >"${result_dir}/runner.pid"
wait "${active_child}"
runner_status=$?
active_child=""
set -e
printf '%s\n' "${runner_status}" >"${result_dir}/runner_exit_status.txt"

set +e
python3 - "${result_dir}" <<'PY' >"${result_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

result_dir = Path(sys.argv[1])
files = list(result_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
phy = stats.get("phy", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < 899.0:
    errors.append("simulation did not reach 900 s")
if result.get("algorithm_evidence", {}).get("process_crashes", 0) != 0:
    errors.append("a ROS process crashed")
if communication.get("mode") != "sionna":
    errors.append("communication mode was not Sionna")
if communication.get("network_topology") != "bs_round_robin":
    errors.append("network topology was not BS round robin")
if stats.get("bs_round_robin_enabled") is not True:
    errors.append("BS round robin did not become active")
if int(stats.get("bs_round_robin_turns", 0)) < 5:
    errors.append("fewer than one complete five-UAV polling cycle")
if int(stats.get("bs_control_attempted_packets", 0)) <= 0:
    errors.append("BS did not send polling grants")
if int(stats.get("bs_uplink_attempted_packets", 0)) <= 0:
    errors.append("BS did not attempt to receive UAV data")
if int(stats.get("bs_downlink_attempted_packets", 0)) <= 0:
    errors.append("BS did not attempt multicast-equivalent downlinks")
if int(stats.get("retried_packets", -1)) != 0:
    errors.append("a communication packet was retransmitted")
if int(phy.get("max_retries", -1)) != 0:
    errors.append("UAV-link retry configuration was not zero")
if int(phy.get("bs_max_retries", -1)) != 0:
    errors.append("BS-link retry configuration was not zero")
exact = (int(stats.get("sionna_exact_samples", 0)) +
         int(stats.get("sionna_cache_corrected_samples", 0)))
if exact <= 0:
    errors.append("Sionna RT was not active")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"turns={stats.get('bs_round_robin_turns')} "
    f"uplink_attempted={stats.get('bs_uplink_attempted_packets')} "
    f"downlink_attempted={stats.get('bs_downlink_attempted_packets')} "
    f"retried={stats.get('retried_packets')}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${result_dir}/case_validation_status.txt"

if (( validation_status == 0 )); then
  printf '%s\n' "complete" >"${result_dir}/run_state.txt"
else
  printf '%s\n' "integrity_failed" >"${result_dir}/run_state.txt"
fi
printf 'END time=%s runner_status=%s integrity_status=%s\n' \
  "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
exit "${validation_status}"
