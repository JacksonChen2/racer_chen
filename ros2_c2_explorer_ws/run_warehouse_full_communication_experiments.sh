#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${C2_EXPERIMENT_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration="${C2_EXPERIMENT_DURATION:-60}"
drone_count="${C2_EXPERIMENT_DRONE_COUNT:-5}"
suite="${C2_EXPERIMENT_SUITE_DIR:-${workspace}/experiments/warehouse_full_c2_${drone_count}uav_ideal_vs_sionna_${duration}s_${run_id}}"
runner="${workspace}/src/c2_explorer_isaac/scripts/run_c2_warehouse.sh"
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
  printf '%s\n' "stopped" >"${suite}/queue_state.txt"
  exit 130
}
trap stop_active_case INT TERM HUP

if [[ ! -x "${runner}" || ! -f "${workspace}/install/setup.bash" ]]; then
  printf 'Build the C2 ROS2 workspace before running the experiment suite.\n' >&2
  exit 2
fi

# The two cases deliberately differ only at the communication transport.
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export C2_SCENARIO=warehouse_full
export C2_DRONE_COUNT="${drone_count}"
export C2_DURATION="${duration}"
export C2_PHYSICS_RATE_HZ="${C2_EXPERIMENT_PHYSICS_RATE_HZ:-50}"
export C2_SENSOR_RATE_HZ="${C2_EXPERIMENT_SENSOR_RATE_HZ:-10}"
export C2_CAMERA_RAY_BUDGET="${C2_EXPERIMENT_CAMERA_RAY_BUDGET:-76800}"
export C2_HEADLESS=1
export C2_VISUALIZE=0
export C2_REQUIRE_COMPLETION=0
export C2_STOP_ON_COMPLETION=0
export C2_MAPPING_COVERAGE_TARGET=0
export C2_RANDOM_SEED="${C2_EXPERIMENT_RANDOM_SEED:-42}"
export C2_COMMUNICATION_MAX_RETRIES=0
export C2_WALL_TIMEOUT_FACTOR="${C2_WALL_TIMEOUT_FACTOR:-30}"
export C2_WALL_TIMEOUT_GRACE="${C2_WALL_TIMEOUT_GRACE:-600}"

# Keep the original five-UAV layout as the default.  The ten-UAV layout uses
# five two-UAV sites based on the same warehouse layout.  Its southeast pair
# also passes the C2 forward-depth preflight required by the trigger barrier.
if [[ -n "${C2_EXPERIMENT_START_POSITIONS:-}" ]]; then
  layout="custom"
  export C2_START_POSITIONS="${C2_EXPERIMENT_START_POSITIONS}"
elif [[ "${drone_count}" == "5" ]]; then
  layout="four_corners_plus_center"
  export C2_START_POSITIONS="\
-26.6 1.0 0.75  5.6 1.0 0.75  -25.6 29.7 0.75  4.6 29.7 0.75  -11.5 15.6 0.75"
elif [[ "${drone_count}" == "10" ]]; then
  layout="five_sites_two_uavs_each_with_southeast_depth_compatible_fix"
  export C2_START_POSITIONS="\
-19.625 4.475000381469727 0.75  -19.625 6.225000381469727 0.75  \
-2.875 7.225000381469727 0.75  -1.125 5.975000381469727 0.75  \
-24.125 25.475000381469727 0.75  -22.625 26.225000381469727 0.75  \
2.375 25.975000381469727 0.75  1.375 27.225000381469727 0.75  \
-9.375 14.225000381469727 0.75  -9.375 15.725000381469727 0.75"
else
  printf 'Set C2_EXPERIMENT_START_POSITIONS for %s UAVs.\n' "${drone_count}" >&2
  exit 2
fi

export C2_SCENE_USD="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
export C2_SIONNA_SCENE_XML="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"

python3 - "${suite}/experiment_manifest.json" "${duration}" \
  "${C2_RANDOM_SEED}" "${C2_START_POSITIONS}" \
  "${C2_PHYSICS_RATE_HZ}" "${C2_SENSOR_RATE_HZ}" \
  "${C2_CAMERA_RAY_BUDGET}" "${drone_count}" "${layout}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[4].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
drone_count = int(sys.argv[8])
if len(starts) != drone_count or any(len(start) != 3 for start in starts):
    raise SystemExit(
        f"expected {drone_count} XYZ start positions, found {len(starts)}"
    )
manifest = {
    "algorithm": "C2-Explorer ROS1-source-faithful ROS2 port",
    "scene": "warehouse_full_with_industrial_ap",
    "layout": sys.argv[9],
    "drone_count": drone_count,
    "duration_s": float(sys.argv[2]),
    "random_seed": int(sys.argv[3]),
    "physics_rate_hz": float(sys.argv[5]),
    "sensor_rate_hz": float(sys.argv[6]),
    "camera_ray_budget": int(sys.argv[7]),
    "start_positions": starts,
    "minimum_start_distance_m": min(
        math.dist(left, right)
        for left, right in itertools.combinations(starts, 2)
    ),
    "controlled_difference": "communication transport only",
    "cases": [
        {
            "name": "ideal_perfect_communication",
            "communication_mode": "ideal",
            "packet_loss": "none",
            "latency": "zero",
        },
        {
            "name": "sionna_actual_communication",
            "communication_mode": "sionna",
            "topology": "distributed UAV-to-UAV",
            "ray_tracing": "Sionna RT exact, no analytic fallback",
            "phy": "5G NR LDPC/CP-OFDM abstraction",
            "max_retries": 0,
        },
    ],
}
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

validate_case() {
  local case_dir="$1"
  local expected_mode="$2"
  python3 - "${case_dir}/warehouse_full_result.json" "${expected_mode}" \
    "${duration}" "${C2_START_POSITIONS}" "${drone_count}" \
    "${C2_SENSOR_RATE_HZ}" <<'PY'
import json
import math
from pathlib import Path
import sys

result_path = Path(sys.argv[1])
expected_mode = sys.argv[2]
expected_duration = float(sys.argv[3])
expected_values = [float(value) for value in sys.argv[4].split()]
expected_starts = [
    expected_values[index:index + 3]
    for index in range(0, len(expected_values), 3)
]
expected_drone_count = int(sys.argv[5])
expected_sensor_rate_hz = float(sys.argv[6])
if not result_path.is_file():
    raise SystemExit("INTEGRITY_ERROR result JSON is missing")
result = json.loads(result_path.read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < expected_duration - 1.0:
    errors.append("simulation did not reach the requested duration")
if int(result.get("algorithm_evidence", {}).get("process_crashes", -1)) != 0:
    errors.append("a ROS process crashed")
if communication.get("mode") != expected_mode:
    errors.append("communication mode mismatch")
if int(result.get("drone_count", 0)) != expected_drone_count:
    errors.append("drone count mismatch")
if not result.get("acceptance", {}).get("all_agents_executed_c2_fsm", False):
    errors.append("not every C2 FSM executed or reached the original FINISH state")
if not result.get("acceptance", {}).get("c2_algorithm_pipeline_observed", False):
    errors.append("the complete C2 planning pipeline was not observed")
actual_starts = metrics.get("start_positions", [])
if len(actual_starts) != len(expected_starts) or any(
    len(actual) != 3 or math.dist(actual, expected) > 1.0e-6
    for actual, expected in zip(actual_starts, expected_starts)
):
    errors.append("initial vehicle positions differ from the controlled layout")
frames_per_drone = metrics.get("point_cloud_frames_per_drone", [])
minimum_frames = max(
    1, int((expected_duration - 1.0) * expected_sensor_rate_hz)
)
if len(frames_per_drone) != expected_drone_count or any(
    int(count) < minimum_frames for count in frames_per_drone
):
    errors.append(
        f"one or more UAV point-cloud streams were incomplete: {frames_per_drone}"
    )
attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", 0))
drops = sum(
    int(stats.get(name, 0))
    for name in ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl")
)
if expected_mode == "ideal":
    if attempted <= 0 or delivered != attempted or drops != 0:
        errors.append("ideal transport was not lossless and immediate")
else:
    exact = (
        int(stats.get("sionna_exact_samples", 0))
        + int(stats.get("sionna_cache_corrected_samples", 0))
    )
    if not communication.get("sionna_ready") or attempted <= 0 or exact <= 0:
        errors.append("strict Sionna RT transport was not active")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    f"INTEGRITY_OK mode={expected_mode} "
    f"scientific_acceptance={result.get('passed')} "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"attempted={attempted} delivered={delivered} drops={drops}"
)
PY
}

run_case() {
  local case_name="$1"
  local mode="$2"
  local domain_id="$3"
  local case_dir="${suite}/${case_name}"
  local runner_status validation_status

  mkdir -p "${case_dir}"
  printf '%s\n' "${case_name}" >"${suite}/active_case.txt"
  printf 'START case=%s mode=%s time=%s\n' \
    "${case_name}" "${mode}" "$(date --iso-8601=seconds)"
  set +e
  ROS_DOMAIN_ID="${domain_id}" \
  C2_COMMUNICATION_MODE="${mode}" \
  C2_RESULT_DIR="${case_dir}" \
  C2_LKH_DIR="/tmp/c2_warehouse_full_${run_id}_${case_name}_lkh" \
    "${runner}" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  runner_status=$?
  active_child=""
  set -e
  printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

  set +e
  validate_case "${case_dir}" "${mode}" \
    >"${case_dir}/case_validation.log" 2>&1
  validation_status=$?
  set -e
  printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
  printf 'END case=%s runner_status=%s integrity_status=%s time=%s\n' \
    "${case_name}" "${runner_status}" "${validation_status}" \
    "$(date --iso-8601=seconds)"
  # The inner runner returns 1 when a scientific acceptance metric is false
  # (for example an agent has not entered EXEC_TRAJ before this fixed-duration
  # experiment ends).  Preserve that result in JSON; only an integrity failure
  # prevents the paired suite from continuing.
  if (( validation_status != 0 )); then
    printf 'failed:%s\n' "${case_name}" >"${suite}/queue_state.txt"
    return 1
  fi
}

write_comparison() {
  python3 - "${suite}" <<'PY'
import json
from pathlib import Path
import sys

suite = Path(sys.argv[1])
cases = {
    "ideal_perfect_communication": suite / "ideal_perfect_communication" / "warehouse_full_result.json",
    "sionna_actual_communication": suite / "sionna_actual_communication" / "warehouse_full_result.json",
}
output = {}
for name, path in cases.items():
    result = json.loads(path.read_text())
    metrics = result["metrics"]
    communication = result["communication"]
    stats = communication["statistics"]
    attempted = int(stats["attempted_packets"])
    delivered = int(stats["delivered_packets"])
    drops = sum(
        int(stats.get(key, 0))
        for key in ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl")
    )
    output[name] = {
        "drone_count": int(result.get("drone_count", 0)),
        "passed": bool(result["passed"]),
        "elapsed_s": metrics.get("elapsed"),
        "mapping_coverage_joint": metrics.get("mapping_coverage_joint"),
        "path_lengths_m": metrics.get("path_lengths"),
        "collision_events": metrics.get("collision_events"),
        "minimum_inter_drone_distance_m": metrics.get("min_inter_drone"),
        "minimum_obstacle_clearance_m": metrics.get("min_obstacle_clearance"),
        "attempted_packets": attempted,
        "delivered_packets": delivered,
        "dropped_packets": drops,
        "packet_delivery_ratio": delivered / attempted if attempted else None,
        "mean_delivery_delay_ms": stats.get("mean_delivery_delay_ms"),
        "sionna_ready": communication.get("sionna_ready"),
        "sionna_exact_samples": (
            int(stats.get("sionna_exact_samples", 0))
            + int(stats.get("sionna_cache_corrected_samples", 0))
        ),
        "algorithm_evidence": result.get("algorithm_evidence", {}),
    }
summary = {
    "comparison": "C2-Explorer warehouse_full perfect communication vs Sionna actual communication",
    "drone_count": next(iter(output.values()))["drone_count"],
    "controlled_difference": "communication transport only",
    "cases": output,
}
(suite / "comparison_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
}

printf '%s\n' "running_ideal" >"${suite}/queue_state.txt"
run_case ideal_perfect_communication ideal 221
printf '%s\n' "running_sionna" >"${suite}/queue_state.txt"
run_case sionna_actual_communication sionna 222
write_comparison | tee "${suite}/comparison_summary.log"
printf '%s\n' "complete" >"${suite}/active_case.txt"
printf '%s\n' "complete" >"${suite}/queue_state.txt"
printf 'SUITE_COMPLETE directory=%s time=%s\n' \
  "${suite}" "$(date --iso-8601=seconds)"
