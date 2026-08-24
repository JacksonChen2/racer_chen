#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite_dir="${RACER_SUITE_DIR:-${script_dir}/experiments/warehouse_full_15uav_two_batch_50hz_1200s_${run_id}}"
mkdir -p "${suite_dir}"
printf '%s\n' "$$" >"${suite_dir}/supervisor.pid"

export ROS_DOMAIN_ID="${RACER_DISCOVERY_DOMAIN_ID:-31}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${script_dir}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
trap 'exit 130' INT TERM HUP

# Shared comparison controls.  These match the preceding optimized 10-UAV
# experiment except for the requested 15 UAVs and 50 Hz physics rate.
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${RACER_FIDELITY_DURATION:-1200}"
export RACER_FIDELITY_DRONE_COUNT=15
export RACER_PHYSICS_RATE_HZ=50
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_SENSOR_WORKER_COUNT=8
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_IDEAL_COALESCE_WINDOW_MS=50
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED="${RACER_RANDOM_SEED:-42}"

# Layout option 2: UAVs 1-8 use the central aisle; UAVs 9-15 use the aisle
# immediately west of the intervening rack.  The adjacent-aisle points were
# checked against the real USD collision geometry with a 0.36 m half-extent.
# The minimum pairwise start distance over all 15 positions is 1.389 m.
export RACER_START_POSITIONS="\
-7.8 16.2 0.8  -7.8 15.0 1.5  -6.6 15.0 2.2  -6.6 16.2 1.15  \
-6.6 17.4 1.85 -7.8 17.4 0.8  -9.0 17.4 1.5  -9.0 16.2 2.2   \
-14.49 14.91 0.8 -13.2 14.91 1.5 -11.91 14.91 2.2 -14.29 16.2 1.5 \
-13.2 16.2 0.8 -11.91 16.2 1.5 -13.2 17.49 2.2"

python3 - "${suite_dir}/experiment_manifest.json" \
  "${RACER_RANDOM_SEED}" "${RACER_START_POSITIONS}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

output = Path(sys.argv[1])
seed = int(sys.argv[2])
values = [float(value) for value in sys.argv[3].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
minimum_start_distance = min(
    math.dist(left, right) for left, right in itertools.combinations(starts, 2)
)
manifest = {
    "scene": "warehouse_full_with_industrial_ap",
    "layout": "option_2_two_batches_adjacent_rack_aisles",
    "drone_count": 15,
    "duration_s": 1200,
    "physics_rate_hz": 50,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 19200,
    "sensor_worker_count": 8,
    "scene_query_rate_hz": 20,
    "random_seed": seed,
    "start_positions": starts,
    "central_batch_drone_ids": list(range(1, 9)),
    "adjacent_rack_batch_drone_ids": list(range(9, 16)),
    "minimum_start_distance_m": minimum_start_distance,
    "cases": {
        "ideal_no_loss_default_safety": {
            "communication_mode": "ideal",
            "network_topology": "distributed",
            "sionna_enabled": False,
            "safety_configuration": "unchanged optimized default",
        },
        "sionna_distributed_safety_0p12m": {
            "communication_mode": "sionna",
            "network_topology": "distributed",
            "sionna_enabled": True,
            "scene_query_safety_margin_m": 0.12,
        },
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

run_case() {
  local case_name="$1"
  local mode="$2"
  local require_sionna="$3"

  printf '[%s] starting %s (mode=%s, physics=50Hz, sensor=10Hz, seed=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${mode}" "${RACER_RANDOM_SEED}"
  set +e
  RACER_RESULT_DIR="${suite_dir}/${case_name}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_15uav_${run_id}_${case_name}_lkh" \
  RACER_ALGORITHM_LABEL="warehouse_full_15uav_${case_name}_50hz" \
  RACER_COMMUNICATION_MODE="${mode}" \
  RACER_REQUIRE_SIONNA="${require_sionna}" \
  RACER_NETWORK_TOPOLOGY=distributed \
    "${script_dir}/run_warehouse_simple_sionna.sh"
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${suite_dir}/${case_name}/runner_exit_status.txt"
  printf '[%s] finished %s (runner status=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${status}"
}

# Run sequentially so both cases receive the full CPU/GPU budget.  Keeping the
# ideal case first provides the lower-cost baseline before Sionna RT starts.
run_case ideal_no_loss_default_safety ideal false
run_case sionna_distributed_safety_0p12m sionna true

python3 - "${suite_dir}" <<'PY'
import json
from pathlib import Path
import sys

suite = Path(sys.argv[1])
summary = {"suite_dir": str(suite), "cases": {}}
for case_dir in sorted(path for path in suite.iterdir() if path.is_dir()):
    result_files = list(case_dir.glob("*_result.json"))
    if not result_files:
        summary["cases"][case_dir.name] = {"result_missing": True}
        continue
    result = json.loads(result_files[0].read_text())
    metrics = result.get("metrics", {})
    stats = result.get("communication", {}).get("statistics", {})
    summary["cases"][case_dir.name] = {
        "passed": result.get("passed"),
        "random_seed": result.get("random_seed"),
        "sim_time_s": metrics.get("elapsed"),
        "physics_rate_hz": metrics.get("physics_rate_hz"),
        "sensor_rate_hz": metrics.get("sensor_rate_hz"),
        "scene_query_safety_margin_m": metrics.get("scene_query_clearance_m"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "collision_events": metrics.get("collision_events"),
        "executed_drone_ids": result.get("executed_drone_ids"),
        "communication_mode": result.get("communication", {}).get("mode"),
        "delivered_packets": stats.get("delivered_packets"),
        "dropped_packets": stats.get("dropped_packets"),
    }
(suite / "comparison_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
