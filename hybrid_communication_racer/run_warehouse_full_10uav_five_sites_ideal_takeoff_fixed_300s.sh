#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
random_seed="${RACER_RANDOM_SEED:-42}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/warehouse_full_10uav_five_sites_pairwise_robust_ideal_takeoff_fixed_300s_${run_id}}"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/warehouse.xml"
selection="${workspace}/config/warehouse_full_10uav_five_sites_layout.json"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
active_child=""

# Five user-requested site centres with two PhysX-validated starts per site.
start_positions="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

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

actual_scene_sha256="$(sha256sum "${scene_usd}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ]]; then
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  printf 'Scene hash mismatch: %s\n' "${actual_scene_sha256}" >&2
  exit 2
fi
if [[ ! -f "${selection}" || "$(wc -w <<<"${start_positions}")" -ne 30 ]]; then
  printf '%s\n' "blocked:invalid_layout" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix racer_original_core)" != "${workspace}/install/racer_original_core" || \
      "$(ros2 pkg prefix racer_fidelity_msgs)" != "${workspace}/install/racer_fidelity_msgs" ]]; then
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi

python3 - "${suite}/experiment_manifest.json" "${selection}" \
  "${actual_scene_sha256}" "${random_seed}" <<'PY'
import itertools
import json
import math
from pathlib import Path
import sys

output = Path(sys.argv[1])
selection_path = Path(sys.argv[2])
selection = json.loads(selection_path.read_text())
starts = selection["start_positions"]
manifest = {
    "algorithm": "pairwise_robust_racer",
    "scene": "warehouse_full_with_industrial_ap_user_modified_20260825",
    "scene_usd_sha256": sys.argv[3],
    "layout": selection["layout"],
    "layout_selection_file": str(selection_path),
    "selection_rule": selection["selection_rule"],
    "regions": selection["regions"],
    "start_positions": starts,
    "minimum_start_spacing_m": min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ),
    "preflight": {
        "duration_s": 15,
        "requires_all_10_nodes": True,
        "requires_all_uav_movement_m": 0.25,
        "requires_zero_collision_and_contact_events": True,
    },
    "formal_case": {
        "duration_s": 300,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "communication_mode": "ideal",
        "network_topology": "distributed",
        "random_seed": int(sys.argv[4]),
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RANDOM_SEED="${random_seed}"
export RACER_START_POSITIONS="${start_positions}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
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
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"

run_case() {
  local name="$1"
  local duration="$2"
  local domain="$3"
  local case_dir="${suite}/${name}"
  mkdir -p "${case_dir}"
  printf '%s\n' "running:${name}" >"${suite}/run_state.txt"
  printf 'START case=%s time=%s duration=%s domain=%s seed=%s\n' \
    "${name}" "$(date --iso-8601=seconds)" "${duration}" "${domain}" "${random_seed}"
  set +e
  ROS_DOMAIN_ID="${domain}" \
  RACER_FIDELITY_DURATION="${duration}" \
  RACER_RECORD_TRAJECTORY_HISTORY=1 \
  RACER_RESULT_DIR="${case_dir}" \
  RACER_LKH_DIR="/tmp/racer_pairwise_robust_takeoff_fixed_${run_id}_${name}_lkh" \
  RACER_ALGORITHM_LABEL="pairwise_robust_10uav_takeoff_fixed_${name}_100hz_76800rays" \
    "${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
  wait "${active_child}"
  local status=$?
  active_child=""
  printf '%s\n' "${status}" >"${case_dir}/runner_exit_status.txt"
  return "${status}"
}

preflight_dir="${suite}/_preflight_15s"
if [[ ! -f "${preflight_dir}/warehouse_full_distributed_result.json" ]]; then
  # The generic experiment wrapper still expects the legacy "send opt
  # request/get response" log strings.  The robust epoch implementation emits
  # transaction logs instead, so its generic exit status can be 1 even when
  # Isaac, all ten FSMs and the complete planner pipeline are healthy.  Record
  # that status, then let the stricter case-specific checks below decide.
  set +e
  run_case _preflight_15s 15 78
  preflight_runner_status=$?
  set -e
else
  preflight_runner_status="$(cat "${preflight_dir}/runner_exit_status.txt" 2>/dev/null || printf 'unknown')"
fi

set +e
python3 - "${preflight_dir}" <<'PY' >"${preflight_dir}/preflight_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"PREFLIGHT_ERROR expected one result, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
evidence = result.get("algorithm_evidence", {})
paths = metrics.get("path_lengths", [])
launch_log = (case_dir / "warehouse_full_distributed_launch.log").read_text(errors="replace")
errors = []
if float(metrics.get("elapsed", 0.0)) < 14.0:
    errors.append("preflight did not reach 15 simulation seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten exploration nodes remained active")
if int(evidence.get("process_crashes", 0)) != 0 or "process has died" in launch_log:
    errors.append("a ROS process crashed")
if int(metrics.get("collision_events", -1)) != 0 or int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("collision/contact detected")
if len(paths) != 10 or any(float(path) < 0.25 for path in paths):
    errors.append(f"not all UAVs took off and moved at least 0.25 m: paths={paths}")
if errors:
    raise SystemExit("\n".join(f"PREFLIGHT_ERROR {error}" for error in errors))
print(
    "PREFLIGHT_OK "
    f"minimum_uav_path={min(paths):.3f} "
    f"coverage={metrics.get('mapping_coverage_joint')}"
)
PY
preflight_validation_status=$?
set -e
printf '%s\n' "${preflight_validation_status}" >"${preflight_dir}/preflight_validation_status.txt"
if [[ "${preflight_validation_status}" -ne 0 ]]; then
  printf '%s\n' "blocked:preflight_validation_failed" >"${suite}/run_state.txt"
  exit 4
fi

formal_dir="${suite}/ideal_no_loss_takeoff_fixed_300s"
set +e
run_case ideal_no_loss_takeoff_fixed_300s 300 79
formal_status=$?
set -e

set +e
python3 - "${formal_dir}" "${random_seed}" <<'PY' >"${formal_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_seed = int(sys.argv[2])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
evidence = result.get("algorithm_evidence", {})
acceptance = result.get("acceptance", {})
errors = []
if int(result.get("random_seed", -1)) != expected_seed:
    errors.append(f"random seed is not {expected_seed}")
if float(metrics.get("elapsed", 0.0)) < 299.0:
    errors.append("simulation did not reach 300 seconds")
if not acceptance.get("isaac_exit_ok", False):
    errors.append("Isaac process did not exit cleanly")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten exploration nodes remained active")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if int(metrics.get("collision_events", -1)) != 0 or int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("collision/contact detected")
if communication.get("mode") != "ideal":
    errors.append("communication mode is not ideal")
attempted = int(stats.get("attempted_packets", 0))
delivered = int(stats.get("delivered_packets", 0))
drops = sum(int(stats.get(name, 0)) for name in
            ("dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"))
if attempted <= 0 or delivered != attempted or drops != 0:
    errors.append("ideal communication was not fully lossless")
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"uav3_path={metrics.get('path_lengths', [None] * 10)[2]} "
    f"uav4_path={metrics.get('path_lengths', [None] * 10)[3]}"
)
PY
formal_validation_status=$?
set -e
printf '%s\n' "${formal_validation_status}" >"${formal_dir}/case_validation_status.txt"

if [[ "${formal_validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s validation=%s\n' \
    "${formal_status}" "${formal_validation_status}" >"${suite}/run_state.txt"
fi
exit "${formal_validation_status}"
