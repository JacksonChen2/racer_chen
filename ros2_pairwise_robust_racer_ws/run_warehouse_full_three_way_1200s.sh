#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite_dir="${RACER_SUITE_DIR:-${script_dir}/validation/warehouse_full_three_way_${run_id}}"
mkdir -p "${suite_dir}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${RACER_FIDELITY_DURATION:-1200}"
export RACER_FIDELITY_DRONE_COUNT=5
export RACER_PHYSICS_RATE_HZ=100
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_SENSOR_RATE_HZ="${RACER_SENSOR_RATE_HZ:-10}"
export RACER_DEPTH_WIDTH="${RACER_DEPTH_WIDTH:-640}"
export RACER_DEPTH_HEIGHT="${RACER_DEPTH_HEIGHT:-480}"
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-80}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_BS_TX_POWER_DBM=40
export RACER_UAV_TX_POWER_DBM=23
export RACER_RANDOM_SEED="${RACER_RANDOM_SEED:-42}"
export RACER_AP_POSITION_X=-10.02891489217081
export RACER_AP_POSITION_Y=14.888611215255622
export RACER_AP_POSITION_Z=7.55

run_case() {
  local case_name="$1"
  local mode="$2"
  local topology="$3"
  local bs_retries="$4"
  local domain_id="$5"

  printf '\n[%s] starting %s (mode=%s, topology=%s, BS retries=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${mode}" "${topology}" "${bs_retries}"
  set +e
  RACER_RESULT_DIR="${suite_dir}/${case_name}" \
  RACER_LKH_DIR="/tmp/racer_warehouse_full_${run_id}_${case_name}_lkh" \
  RACER_ALGORITHM_LABEL="${case_name}" \
  RACER_COMMUNICATION_MODE="${mode}" \
  RACER_NETWORK_TOPOLOGY="${topology}" \
  RACER_BS_MAX_RETRIES="${bs_retries}" \
  ROS_DOMAIN_ID="${domain_id}" \
    "${script_dir}/run_warehouse_simple_sionna.sh"
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${suite_dir}/${case_name}/runner_exit_status.txt"
  printf '[%s] finished %s (runner status=%s)\n' \
    "$(date --iso-8601=seconds)" "${case_name}" "${status}"
}

run_case ideal_no_loss ideal distributed 3 186
run_case sionna_distributed_no_bs sionna distributed 3 187
run_case sionna_bs_round_robin_best_effort sionna bs_round_robin 0 188

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
        "delivered_packets": stats.get("delivered_packets"),
        "dropped_packets": stats.get("dropped_packets"),
        "retried_packets": stats.get("retried_packets"),
        "bs_round_robin_turns": stats.get("bs_round_robin_turns"),
        "bs_tx_power_dbm": stats.get("phy", {}).get("bs_tx_power_dbm"),
        "uav_tx_power_dbm": stats.get("phy", {}).get("uav_tx_power_dbm"),
        "bs_max_retries": stats.get("phy", {}).get("bs_max_retries"),
    }
(suite / "comparison_summary.json").write_text(
    json.dumps(summary, indent=2, sort_keys=True) + "\n"
)
print(json.dumps(summary, indent=2, sort_keys=True))
PY
