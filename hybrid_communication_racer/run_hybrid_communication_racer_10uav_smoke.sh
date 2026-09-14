#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
duration_s="${RACER_SMOKE_DURATION_S:-30}"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/hybrid_communication_racer_10uav_smoke_${duration_s}s_${run_id}}"
selection="${workspace}/config/hybrid_communication_racer_10uav_smoke_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/warehouse.xml"
domain_base="${RACER_DISCOVERY_DOMAIN_ID:-81}"

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf 'Expected exactly ten XYZ starts in %s\n' "${selection}" >&2
  exit 2
fi
if [[ ! -f "${scene_usd}" ]]; then
  printf 'Warehouse scene is missing: %s\n' "${scene_usd}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

mkdir -p "${suite}"
python3 - "${suite}/experiment_manifest.json" "${duration_s}" "${selection}" <<'PY'
import json
from pathlib import Path
import sys

selection = json.loads(Path(sys.argv[3]).read_text(encoding="utf-8"))
manifest = {
    "algorithm": "hybrid_communication_racer",
    "comparison_modes": ["original", "global_cooperative"],
    "communication": "ideal_perfect_lossless",
    "duration_s": float(sys.argv[2]),
    "drone_count": 10,
    "physics_rate_hz": 100,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 76800,
    "random_seed": 42,
    "layout": selection["layout"],
    "selection_rule": selection["selection_rule"],
    "layout_file": str(Path(sys.argv[3]).resolve()),
    "start_positions": selection["start_positions"],
}
Path(sys.argv[1]).write_text(
    json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
PY

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
export RACER_SENSOR_WORKER_COUNT="${RACER_SENSOR_WORKER_COUNT:-8}"
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_TRIGGER_DELAY_S=0
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
export RACER_SIONNA_SCENE_XML="${scene_xml}"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_COMMUNICATION_MODE=ideal
export RACER_REQUIRE_SIONNA=false
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0

run_case() {
  local mode="$1"
  local domain_id="$2"
  local case_dir="${suite}/${mode}"
  mkdir -p "${case_dir}"
  export ROS_DOMAIN_ID="${domain_id}"
  export RACER_EXPLORATION_ASSIGNMENT_MODE="${mode}"
  export RACER_ALGORITHM_LABEL="hybrid_communication_racer_${mode}_10uav_smoke"
  export RACER_RESULT_DIR="${case_dir}"
  export RACER_LKH_DIR="/tmp/hybrid_communication_racer_${mode}_${run_id}_lkh"
  printf 'START mode=%s duration=%s domain=%s time=%s\n' \
    "${mode}" "${duration_s}" "${domain_id}" "$(date --iso-8601=seconds)"
  set +e
  "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
  printf 'FINISH mode=%s status=%s time=%s\n' \
    "${mode}" "${status}" "$(date --iso-8601=seconds)"
}

run_case original "${domain_base}"
run_case global_cooperative "$((domain_base + 1))"
python3 "${workspace}/scripts/analyze_hybrid_10uav_smoke.py" "${suite}" \
  | tee "${suite}/comparison.stdout.json"
printf 'SMOKE_SUITE=%s\n' "${suite}"
