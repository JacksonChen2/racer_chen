#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deadline="${RACER_AB_DEADLINE_SECONDS:-900}"
run_id="${RACER_AB_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
experiment_root="${RACER_AB_RESULT_ROOT:-${workspace_dir}/experiments/warehouse_simple_ap_ab_${run_id}}"
scene_usd="${RACER_SCENE_USD:-$(realpath "${workspace_dir}/../ros2_3d_py_ws/warehouse_simple_with_industrial_ap.usda")}"
sionna_scene="${RACER_SIONNA_SCENE_XML:-${workspace_dir}/src/racer_sionna_comm/assets/warehouse_simple_with_ap_sionna/warehouse.xml}"

if [[ ! "${deadline}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'RACER_AB_DEADLINE_SECONDS must be a positive number.\n' >&2
  exit 2
fi
if ! python3 -c 'import sys; raise SystemExit(0 if float(sys.argv[1]) > 0 else 1)' "${deadline}"; then
  printf 'RACER_AB_DEADLINE_SECONDS must be positive.\n' >&2
  exit 2
fi
if [[ ! -f "${scene_usd}" || ! -f "${sionna_scene}" ]]; then
  printf 'Missing AP-enabled Isaac or Sionna scene.\n' >&2
  exit 2
fi

mkdir -p "${experiment_root}/distributed" "${experiment_root}/ap_assisted"
python3 - "${experiment_root}/manifest.json" "${deadline}" "${scene_usd}" "${sionna_scene}" <<'PY'
import json
from pathlib import Path
import sys
Path(sys.argv[1]).write_text(json.dumps({
    "deadline_seconds": float(sys.argv[2]),
    "scene_usd": sys.argv[3],
    "sionna_scene_xml": sys.argv[4],
    "drone_count": 5,
    "vehicle": "crazyflie_with_racer_dynamics.usd",
    "communication_mode": "sionna",
    "topologies": ["distributed", "ap_assisted"],
    "random_seed": 42,
    "carrier_frequency_hz": 2437000000.0,
    "bandwidth_hz": 20000000.0,
    "deadline_rule": "stop at original RACER task completion or simulation deadline",
    "wall_time_multiplier": 120,
    "interactive_visualization": False,
    "trajectory_replay_recording": False,
    "propeller_animation": False,
}, indent=2, sort_keys=True) + "\n")
PY

run_one() {
  local topology="$1"
  local result_dir="${experiment_root}/${topology}"
  printf 'RACER_AB_PHASE topology=%s state=starting result_dir=%s\n' \
    "${topology}" "${result_dir}"
  set +e
  env \
    RACER_COMMUNICATION_MODE=sionna \
    RACER_NETWORK_TOPOLOGY="${topology}" \
    RACER_FIDELITY_DURATION="${deadline}" \
    RACER_FIDELITY_DRONE_COUNT=5 \
    RACER_FIDELITY_SCENARIO=warehouse_simple \
    RACER_FIDELITY_HEADLESS=1 \
    RACER_FIDELITY_VISUALIZE=0 \
    RACER_RECORD_TRAJECTORY_HISTORY=0 \
    RACER_REQUIRE_COMPLETION=0 \
    RACER_STOP_ON_COMPLETION=1 \
    RACER_MAPPING_COVERAGE_TARGET=0 \
    RACER_WALL_TIME_MULTIPLIER=120 \
    RACER_SCENE_USD="${scene_usd}" \
    RACER_SIONNA_SCENE_XML="${sionna_scene}" \
    RACER_RESULT_DIR="${result_dir}" \
    "${workspace_dir}/run_warehouse_simple_sionna.sh"
  local status=$?
  set -e
  printf '%s\n' "${status}" > "${result_dir}/runner_exit_status.txt"
  printf 'RACER_AB_PHASE topology=%s state=finished exit_status=%s\n' \
    "${topology}" "${status}"
  return 0
}

run_one distributed
run_one ap_assisted

distributed_result="${experiment_root}/distributed/warehouse_simple_distributed_result.json"
ap_result="${experiment_root}/ap_assisted/warehouse_simple_ap_assisted_result.json"
if [[ -f "${distributed_result}" && -f "${ap_result}" ]]; then
  python3 "${workspace_dir}/scripts/compare_ap_mapping_results.py" \
    --distributed "${distributed_result}" \
    --ap-assisted "${ap_result}" \
    --output-dir "${experiment_root}"
  printf 'RACER_AB_COMPLETE result_dir=%s\n' "${experiment_root}"
else
  printf 'RACER_AB_INCOMPLETE result_dir=%s\n' "${experiment_root}" >&2
  exit 1
fi
