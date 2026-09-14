#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite_dir="${RACER_SUITE_DIR:-${script_dir}/experiments/warehouse_full_20uav_five_way_1200s_${run_id}}"
mkdir -p "${suite_dir}"

# One experiment launches more than 100 DDS participants.  Fast DDS defaults
# to 100 attempts at selecting a unique participant port, so raise that bound
# for every ROS/Isaac participant in this suite.
export ROS_DOMAIN_ID="${RACER_DISCOVERY_DOMAIN_ID:-26}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${script_dir}/config/fastdds_large_scale.xml"
# ROS 2 Humble ships Fast DDS under its former Fast RTPS name and reads this
# legacy variable; keep the new spelling too for Isaac Sim/newer Fast DDS.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
trap 'exit 130' INT TERM HUP

export RACER_FIDELITY_SCENARIO=warehouse_full
: "${RACER_FIDELITY_DURATION:=1200}"
: "${RACER_FIDELITY_DRONE_COUNT:=20}"
: "${RACER_PHYSICS_RATE_HZ:=60}"
: "${RACER_SENSOR_RATE_HZ:=10}"
: "${RACER_CAMERA_RAY_BUDGET:=19200}"
export RACER_FIDELITY_DURATION RACER_FIDELITY_DRONE_COUNT RACER_PHYSICS_RATE_HZ
export RACER_SENSOR_RATE_HZ RACER_CAMERA_RAY_BUDGET
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
# The measured 20-camera real-time factor is roughly 1/113 initially and
# declines as maps grow.  Leave enough wall-clock headroom for 1200 sim s.
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED="${RACER_RANDOM_SEED:-42}"

run_case() {
  local case_name="$1"
  local mode="$2"
  local topology="$3"
  local nearest_count="$4"
  local domain_id="$5"
  local require_sionna=false
  if [[ "${mode}" != "ideal" ]]; then
    require_sionna=true
  fi

  printf '\n[%s] starting %s (mode=%s, topology=%s, nearest=%s, sionna=%s, seed=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${mode}" "${topology}" \
    "${nearest_count}" "${require_sionna}" "${RACER_RANDOM_SEED}"
  set +e
  RACER_RESULT_DIR="${suite_dir}/${case_name}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_20uav_${run_id}_${case_name}_lkh" \
  RACER_ALGORITHM_LABEL="${case_name}" \
  RACER_COMMUNICATION_MODE="${mode}" \
  RACER_REQUIRE_SIONNA="${require_sionna}" \
  RACER_NETWORK_TOPOLOGY="${topology}" \
  RACER_NEAREST_NEIGHBOR_COUNT="${nearest_count}" \
  ROS_DOMAIN_ID="${domain_id}" \
    "${script_dir}/run_warehouse_simple_sionna.sh"
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${suite_dir}/${case_name}/runner_exit_status.txt"
  printf '[%s] finished %s (runner status=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${status}"
}

run_case ideal_no_loss ideal distributed 0 26
run_case sionna_distributed sionna distributed 0 26
run_case nearest_3_no_loss ideal nearest_neighbors 3 26
run_case nearest_5_no_loss ideal nearest_neighbors 5 26
run_case nearest_7_no_loss ideal nearest_neighbors 7 26

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
    stats = result.get("communication", {}).get("statistics", {})
    metrics = result.get("metrics", {})
    summary["cases"][case_dir.name] = {
        "passed": result.get("passed"),
        "random_seed": result.get("random_seed"),
        "sim_time_s": metrics.get("elapsed"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "collision_events": metrics.get("collision_events"),
        "communication_mode": result.get("communication", {}).get("mode"),
        "network_topology": result.get("communication", {}).get("network_topology"),
        "nearest_neighbor_count": stats.get("nearest_neighbor_count"),
        "delivered_packets": stats.get("delivered_packets"),
        "dropped_packets": stats.get("dropped_packets"),
    }
(suite / "comparison_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
