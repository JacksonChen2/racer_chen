#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_5uav_four_corners_center_ideal_vs_sionna_50hz_900s_${run_id}}"
formal_duration="${RACER_FORMAL_DURATION:-900}"
active_child=""

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/queue_state.txt"

stop_active_case() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${suite}/active_case.txt"
  printf '%s\n' "stopped" >"${suite}/queue_state.txt"
  exit 130
}
trap stop_active_case INT TERM HUP

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"

export RACER_FIDELITY_SCENARIO=warehouse_full
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

# Four collision-free warehouse corners plus one collision-free center point.
# These positions come from the 0.75 m layer of the scene's 5x5 USD query grid.
export RACER_START_POSITIONS="\
-26.6 1.0 0.75  5.6 1.0 0.75  -25.6 29.7 0.75  4.6 29.7 0.75  -11.5 15.6 0.75"

export RACER_SCENE_USD="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
export RACER_SIONNA_SCENE_XML="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

python3 - "${suite}/experiment_manifest.json" "${formal_duration}" \
  "${RACER_RANDOM_SEED}" "${RACER_START_POSITIONS}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

output = Path(sys.argv[1])
duration = int(sys.argv[2])
seed = int(sys.argv[3])
values = [float(value) for value in sys.argv[4].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "scene": "warehouse_full_with_industrial_ap",
    "layout": "four_corners_plus_center",
    "layout_source": "warehouse_full_with_industrial_ap_uav_uav_28ghz_25points/collision_free_tx_points.json",
    "drone_count": 5,
    "duration_s": duration,
    "physics_rate_hz": 50,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 19200,
    "random_seed": seed,
    "start_positions": starts,
    "minimum_start_distance_m": min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ),
    "cases": [
        {
            "name": "ideal_no_loss_original_broadcast",
            "communication_mode": "ideal",
            "sionna_required": False,
        },
        {
            "name": "sionna_distributed_original_comm_no_retries",
            "communication_mode": "sionna",
            "sionna_required": True,
            "max_retries": 0,
        },
    ],
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

validate_preflight() {
  python3 - "${suite}/_preflight_start_layout/warehouse_full_distributed_isaac.log" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
prefix = "RACER_3D_ISAAC_RESULT "
matches = [line[len(prefix):] for line in path.read_text(errors="replace").splitlines()
           if line.startswith(prefix)]
if not matches:
    raise SystemExit("PREFLIGHT_ERROR missing Isaac result")
metrics = json.loads(matches[-1])
errors = []
if float(metrics.get("elapsed", 0.0)) < 1.9:
    errors.append("preflight did not reach two simulation seconds")
if int(metrics.get("collision_events", -1)) != 0:
    errors.append("collision event detected at the proposed starts")
if int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("physics contact detected at the proposed starts")
if float(metrics.get("min_inter_drone", 0.0)) < 1.0:
    errors.append("minimum start separation fell below 1.0 m")
if float(metrics.get("min_obstacle_clearance", -1.0)) <= 0.0:
    errors.append("proposed starts do not have positive obstacle clearance")
if errors:
    raise SystemExit("\n".join(f"PREFLIGHT_ERROR {error}" for error in errors))
print(
    "PREFLIGHT_OK "
    f"min_inter_drone={metrics['min_inter_drone']:.6f} "
    f"min_obstacle_clearance={metrics['min_obstacle_clearance']:.6f}"
)
PY
}

run_preflight() {
  local case_dir="${suite}/_preflight_start_layout"
  mkdir -p "${case_dir}"
  printf '%s\n' "preflight_start_layout" >"${suite}/active_case.txt"
  printf '%s\n' "running_preflight" >"${suite}/queue_state.txt"
  set +e
  ROS_DOMAIN_ID=60 \
  RACER_FIDELITY_DURATION=2 \
  RACER_STOP_ON_COMPLETION=0 \
  RACER_RECORD_TRAJECTORY_HISTORY=0 \
  RACER_COMMUNICATION_MODE=ideal \
  RACER_REQUIRE_SIONNA=false \
  RACER_RESULT_DIR="${case_dir}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_5uav_${run_id}_preflight_lkh" \
  RACER_ALGORITHM_LABEL="warehouse_full_5uav_four_corners_center_start_preflight" \
    "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  local status=$?
  active_child=""
  set -e
  printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
  validate_preflight >"${case_dir}/preflight_validation.log" 2>&1
  printf '%s\n' "0" >"${case_dir}/preflight_validation_status.txt"
}

validate_case() {
  local case_dir="$1"
  local expected_mode="$2"
  python3 - "${case_dir}" "${expected_mode}" "${formal_duration}" <<'PY'
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_mode = sys.argv[2]
expected_duration = float(sys.argv[3])
result_files = list(case_dir.glob("*_result.json"))
if len(result_files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(result_files)}")
result = json.loads(result_files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < expected_duration - 1.0:
    errors.append(f"simulation did not reach {expected_duration:g} s")
if result.get("algorithm_evidence", {}).get("process_crashes", 0) != 0:
    errors.append("a ROS process crashed")
if communication.get("mode") != expected_mode:
    errors.append("communication mode mismatch")
attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", 0))
if expected_mode == "ideal":
    drops = sum(int(stats.get(name, 0)) for name in
                ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"))
    if attempted <= 0 or delivered != attempted or drops != 0:
        errors.append("ideal transport was not fully lossless")
else:
    exact = (int(stats.get("sionna_exact_samples", 0)) +
             int(stats.get("sionna_cache_corrected_samples", 0)))
    if attempted <= 0 or exact <= 0:
        errors.append("Sionna transport was not active")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    f"INTEGRITY_OK mode={expected_mode} scientific_passed={result.get('passed')} "
    f"collisions={metrics.get('collision_events')} "
    f"coverage={metrics.get('mapping_coverage_joint')}"
)
PY
}

run_case() {
  local case_name="$1"
  local mode="$2"
  local require_sionna="$3"
  local domain_id="$4"
  local case_dir="${suite}/${case_name}"
  local runner_status validation_status

  mkdir -p "${case_dir}"
  printf '%s\n' "${case_name}" >"${suite}/active_case.txt"
  printf 'START case=%s mode=%s time=%s\n' \
    "${case_name}" "${mode}" "$(date --iso-8601=seconds)"
  set +e
  ROS_DOMAIN_ID="${domain_id}" \
  RACER_FIDELITY_DURATION="${formal_duration}" \
  RACER_COMMUNICATION_MODE="${mode}" \
  RACER_REQUIRE_SIONNA="${require_sionna}" \
  RACER_RESULT_DIR="${case_dir}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_5uav_${run_id}_${case_name}_lkh" \
  RACER_ALGORITHM_LABEL="warehouse_full_5uav_four_corners_center_${case_name}_50hz" \
    "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  runner_status=$?
  active_child=""
  set -e
  printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

  set +e
  validate_case "${case_dir}" "${mode}" >"${case_dir}/case_validation.log" 2>&1
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'END case=%s runner_status=%s integrity_status=%s time=%s\n' \
    "${case_name}" "${runner_status}" "${validation_status}" "$(date --iso-8601=seconds)"
  if (( validation_status != 0 )); then
    printf '%s\n' "integrity_failed:${case_name}" >"${suite}/queue_state.txt"
    return "${validation_status}"
  fi
}

run_preflight
printf '%s\n' "running_formal_suite" >"${suite}/queue_state.txt"
sleep 5
run_case ideal_no_loss_original_broadcast ideal false 61
sleep 5
run_case sionna_distributed_original_comm_no_retries sionna true 62

printf '%s\n' "complete" >"${suite}/active_case.txt"
printf '%s\n' "complete" >"${suite}/queue_state.txt"
printf 'SUITE_COMPLETE time=%s\n' "$(date --iso-8601=seconds)"
