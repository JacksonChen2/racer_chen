#!/usr/bin/env bash
set -euo pipefail

current_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
legacy_workspace="${current_workspace}/ros2_ws"
legacy_entrypoint="${legacy_workspace}/run_warehouse_simple_sionna.sh"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${current_workspace}/results/perfect_75_2913_${run_id}}"
formal_duration="${RACER_EXPERIMENT_DURATION:-300}"
formal_label="formal_${formal_duration}s"
passive_task_metrics="${RACER_PASSIVE_TASK_METRICS:-true}"
task_metric_sample_period_s="${RACER_TASK_METRIC_SAMPLE_PERIOD_S:-0.1}"
ground_truth_occupied_voxels_path="${RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH:-${current_workspace}/data/gt_occupied_voxels.txt}"
debug_overlay_setup="${RACER_DEBUG_OVERLAY_SETUP:-${current_workspace}/passive_metrics_overlay_ws/install/setup.bash}"
reference_coverage="${RACER_REFERENCE_COVERAGE:-0.752913131313}"
coverage_relative_tolerance="${RACER_COVERAGE_RELATIVE_TOLERANCE:-0.10}"

scene_usd="${current_workspace}/assets/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
vehicle_usd="${current_workspace}/assets/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
sionna_scene_xml="${current_workspace}/assets/sionna_scene/warehouse.xml"
fastdds_profile="${legacy_workspace}/config/fastdds_large_scale.xml"

# Exact starts used by the completed 75.2913131313% run on 2026-09-02.
default_start_positions="\
-20.75 26 0.75  -19.25 26 0.75  \
-18.75 5 0.75   -17.25 5 0.75  \
-11.75 14.25 0.75  -11.75 15.75 0.75  \
-0.75 27 0.75   0.75 27 0.75  \
-3.75 5 0.75   -2.25 5 0.75"
start_positions="${RACER_START_POSITIONS:-${default_start_positions}}"
custom_start_positions=false
if [[ -n "${RACER_START_POSITIONS:-}" ]]; then
  custom_start_positions=true
fi
if ! [[ "${formal_duration}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_EXPERIMENT_DURATION must be a positive integer: %s\n' \
    "${formal_duration}" >&2
  exit 2
fi

check_sha256() {
  local expected="$1"
  local path="$2"
  local actual
  if [[ ! -e "${path}" ]]; then
    printf 'REPRO_CHECK_ERROR missing file: %s\n' "${path}" >&2
    return 1
  fi
  actual="$(sha256sum "${path}" | awk '{print $1}')"
  if [[ "${actual}" != "${expected}" ]]; then
    printf 'REPRO_CHECK_ERROR hash mismatch: %s\n' "${path}" >&2
    printf '  expected=%s\n  actual=%s\n' "${expected}" "${actual}" >&2
    return 1
  fi
}

static_checks() {
  check_sha256 7ac5c0066524d95d3d3aec1c42342b1e5bcd8ddb3f84de0459a5e2c083cf2989 \
    "${legacy_entrypoint}"
  check_sha256 23852369427d0d8d97f495d8b895e36e7c3731c1b430b07af233e5f157069a29 \
    "${legacy_workspace}/src/racer_isaac_adapter/isaac_sim/original_racer_isaac.py"
  check_sha256 c4fee3f64db0d2f07614d53149aaeea14ef753d274c4ae2c9a2d1f954f11c923 \
    "${legacy_workspace}/src/racer_sionna_comm/src/communication_proxy_node.cpp"
  check_sha256 3f8d37073ff76f664b85b3e9460e40c2b3ea52fb67e995f544b52d89418456f8 \
    "${current_workspace}/passive_metrics_overlay_ws/src/racer_sionna_comm/src/communication_proxy_node.cpp"
  check_sha256 76320a75459fd3c07c2ba06196307f689c1d08d33f4b15f27778dd9a0efcf9b4 \
    "${legacy_workspace}/src/racer_original_core/upstream/exploration_manager/src/fast_exploration_fsm.cpp"
  check_sha256 2d625f59fc5b6f19e3e4f7b9b68f0e26e57ca9fd4bed7dc167f43d0f48295bd2 \
    "${fastdds_profile}"
  check_sha256 e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e \
    "${scene_usd}"
  check_sha256 c0d3c998ce8202fd46f1d51a71ce5b68fb5a3e63338575e335597be847cb4a8c \
    "${vehicle_usd}"
  check_sha256 b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168 \
    "${sionna_scene_xml}"
  check_sha256 240ffffb3d14b062721748cb81fa5fcd3c30524d40b9ebff958c327a431a02f2 \
    "${ground_truth_occupied_voxels_path}"
  [[ -f "${sionna_scene_xml}" ]] || {
    printf 'REPRO_CHECK_ERROR missing legacy Sionna scene: %s\n' "${sionna_scene_xml}" >&2
    return 1
  }
  [[ -x /home/jiazheng/software/isaacsim/python.sh ]] || {
    printf 'REPRO_CHECK_ERROR Isaac Sim Python is unavailable\n' >&2
    return 1
  }
  if [[ "${passive_task_metrics}" != "true" &&
        "${passive_task_metrics}" != "false" ]]; then
    printf 'REPRO_CHECK_ERROR RACER_PASSIVE_TASK_METRICS must be true or false\n' >&2
    return 1
  fi
  if [[ "$(wc -w <<<"${start_positions}")" -ne 30 ]] ||
     ! python3 -c 'import math,sys; values=[float(v) for v in sys.argv[1].split()]; raise SystemExit(not (len(values) == 30 and all(map(math.isfinite, values))))' "${start_positions}"; then
    printf 'REPRO_CHECK_ERROR RACER_START_POSITIONS must contain 10 finite XYZ positions\n' >&2
    return 1
  fi
  if [[ "${passive_task_metrics}" == "true" ]]; then
    [[ -f "${debug_overlay_setup}" ]] || {
      printf 'REPRO_CHECK_ERROR passive metrics overlay is not built\n' >&2
      printf '  run: %s/scripts/build.sh\n' "${current_workspace}" >&2
      return 1
    }
    [[ -s "${ground_truth_occupied_voxels_path}" ]] || {
      printf 'REPRO_CHECK_ERROR passive metrics GT file is missing or empty\n' >&2
      return 1
    }
    python3 - "${task_metric_sample_period_s}" \
      "${reference_coverage}" "${coverage_relative_tolerance}" <<'PY'
import math
import sys
values = [float(value) for value in sys.argv[1:]]
if not all(math.isfinite(value) and value > 0.0 for value in values):
    raise SystemExit("REPRO_CHECK_ERROR invalid passive metric/tolerance value")
PY
  fi
  [[ -f "${legacy_workspace}/install/setup.bash" ]] || {
    printf 'REPRO_CHECK_ERROR ROS 2 workspace is not built; run scripts/build.sh\n' >&2
    return 1
  }
  printf 'REPRO_STATIC_CHECK_OK pinned source, DDS profile, assets, and local builds match\n'
}

resource_checks() {
  local available_cores gpu_apps expected_cpu_threads allow_shared_gpu
  available_cores="$(nproc)"
  expected_cpu_threads="${RACER_EXPECTED_CPU_THREADS:-18}"
  allow_shared_gpu="${RACER_ALLOW_SHARED_GPU:-1}"
  gpu_apps="$(nvidia-smi --query-compute-apps=pid,process_name,used_memory \
    --format=csv,noheader 2>/dev/null || true)"

  if [[ ! "${expected_cpu_threads}" =~ ^[1-9][0-9]*$ ]]; then
    printf 'REPRO_RESOURCE_BLOCKED RACER_EXPECTED_CPU_THREADS must be positive\n' >&2
    return 1
  fi
  if [[ "${allow_shared_gpu}" != "0" && "${allow_shared_gpu}" != "1" ]]; then
    printf 'REPRO_RESOURCE_BLOCKED RACER_ALLOW_SHARED_GPU must be 0 or 1\n' >&2
    return 1
  fi
  if [[ "${available_cores}" != "${expected_cpu_threads}" ]]; then
    printf 'REPRO_RESOURCE_BLOCKED expected %s available CPU threads, found %s\n' \
      "${expected_cpu_threads}" "${available_cores}" >&2
    return 1
  fi
  if [[ -n "${gpu_apps}" && "${allow_shared_gpu}" != "1" ]]; then
    printf 'REPRO_RESOURCE_BLOCKED GPU has active compute processes:\n%s\n' \
      "${gpu_apps}" >&2
    return 1
  fi
  if [[ -n "${gpu_apps}" ]]; then
    printf 'REPRO_RESOURCE_WARNING shared GPU explicitly allowed; active processes:\n%s\n' \
      "${gpu_apps}" >&2
  fi
  printf 'REPRO_RESOURCE_CHECK_OK cpu_threads=%s shared_gpu=%s\n' \
    "${available_cores}" "${allow_shared_gpu}"
}

write_manifest() {
  mkdir -p "${suite}"
  python3 - "${suite}/reproduction_manifest.json" "${legacy_workspace}" \
    "${scene_usd}" "${vehicle_usd}" "${sionna_scene_xml}" \
    "${fastdds_profile}" "${start_positions}" \
    "${ground_truth_occupied_voxels_path}" "${debug_overlay_setup}" \
    "${reference_coverage}" "${coverage_relative_tolerance}" \
    "${passive_task_metrics}" "${RACER_TASK_METRIC_OBSERVER_MODE:-async}" \
    "${task_metric_sample_period_s}" \
    "${RACER_EXPECTED_CPU_THREADS:-18}" "${RACER_ALLOW_SHARED_GPU:-1}" \
    "${formal_duration}" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys

output = Path(sys.argv[1])
paths = [Path(value) for value in sys.argv[2:7]]
starts = [float(value) for value in sys.argv[7].split()]
ground_truth = Path(sys.argv[8])
overlay_setup = Path(sys.argv[9])
reference_coverage = float(sys.argv[10])
coverage_tolerance = float(sys.argv[11])
passive_metrics = sys.argv[12] == "true"
observer_mode = sys.argv[13]
sample_period_s = float(sys.argv[14])
expected_cpu_threads = int(sys.argv[15])
allow_shared_gpu = sys.argv[16] == "1"
formal_duration = int(sys.argv[17])
manifest = {
    "reference_result": {
        "coverage": reference_coverage,
        "date": "2026-09-02",
    },
    "legacy_workspace": str(paths[0]),
    "scene_usd": str(paths[1]),
    "vehicle_usd": str(paths[2]),
    "sionna_scene_xml": str(paths[3]),
    "fastdds_profile": str(paths[4]),
    "start_positions": [starts[i:i + 3] for i in range(0, len(starts), 3)],
    "settings": {
        "seed": 42,
        "communication": "ideal",
        "network_topology": "distributed",
        "preflight_s": 15,
        "formal_s": formal_duration,
        "physics_hz": 100,
        "sensor_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_workers": 8,
        "scene_query_hz": 20,
        "fixed_exploration_horizon": False,
        "ros_localhost_only": True,
        "rmw_implementation": "rmw_fastrtps_cpp",
    },
    "runtime_resource_override": {
        "expected_cpu_threads": expected_cpu_threads,
        "allow_shared_gpu": allow_shared_gpu,
    },
    "passive_task_metrics": {
        "enabled": passive_metrics,
        "observer_mode": observer_mode,
        "sample_period_s": sample_period_s,
        "ground_truth_occupied_voxels_path": str(ground_truth),
        "coverage_relative_tolerance": coverage_tolerance,
        "overlay_setup": str(overlay_setup),
    },
    "sha256": {
        str(path): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in paths[1:]
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY
}

run_case() {
  local label="$1"
  local duration="$2"
  local domain="$3"
  local result_dir="${suite}/${label}"
  local lkh_dir="/tmp/final_racer_75_2913_${run_id}_${label}_lkh"
  local task_metric_output="${result_dir}/passive_task_metrics.jsonl"

  mkdir -p "${result_dir}"
  printf 'running:%s\n' "${label}" >"${suite}/run_state.txt"
  set +e
  env \
    ROS_DOMAIN_ID="${domain}" \
    ROS_LOCALHOST_ONLY=1 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTDDS_DEFAULT_PROFILES_FILE="${fastdds_profile}" \
    FASTRTPS_DEFAULT_PROFILES_FILE="${fastdds_profile}" \
    RACER_FIDELITY_SCENARIO=warehouse_full \
    RACER_FIDELITY_DRONE_COUNT=10 \
    RACER_FIDELITY_DURATION="${duration}" \
    RACER_PHYSICS_RATE_HZ=100 \
    RACER_SENSOR_RATE_HZ=10 \
    RACER_CAMERA_RAY_BUDGET=76800 \
    RACER_DEPTH_SENSOR_BACKEND=warp \
    RACER_SENSOR_WORKER_COUNT=8 \
    RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0 \
    RACER_SCENE_QUERY_RATE_HZ=20 \
    RACER_FIDELITY_HEADLESS=1 \
    RACER_FIDELITY_VISUALIZE=0 \
    RACER_REQUIRE_COMPLETION=0 \
    RACER_STOP_ON_COMPLETION=0 \
    RACER_MAPPING_COVERAGE_TARGET=0 \
    RACER_RANDOM_SEED=42 \
    RACER_START_POSITIONS="${start_positions}" \
    RACER_SCENE_USD="${scene_usd}" \
    RACER_VEHICLE_USD="${vehicle_usd}" \
    RACER_SIONNA_SCENE_XML="${sionna_scene_xml}" \
    RACER_NETWORK_TOPOLOGY=distributed \
    RACER_COMMUNICATION_MODE=ideal \
    RACER_REQUIRE_SIONNA=false \
    RACER_EXPLORATION_ASSIGNMENT_MODE=original \
    RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
    RACER_NEAREST_NEIGHBOR_COUNT=0 \
    RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0 \
    RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0 \
    RACER_LOSSLESS_CONTROL_ONLY=false \
    RACER_COMMUNICATION_RANGE_M=4.0 \
    RACER_IDEAL_COALESCE_WINDOW_MS=20 \
    RACER_MAX_RETRIES=0 \
    RACER_BS_MAX_RETRIES=0 \
    RACER_SENSOR_PROFILING=1 \
    RACER_RECORD_TRAJECTORY_HISTORY=1 \
    RACER_DEBUG_OVERLAY_SETUP="${debug_overlay_setup}" \
    RACER_TASK_METRIC_OBSERVER_MODE="${RACER_TASK_METRIC_OBSERVER_MODE:-async}" \
    RACER_TASK_METRIC_SAMPLE_PERIOD_S="${task_metric_sample_period_s}" \
    RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${ground_truth_occupied_voxels_path}" \
    RACER_TASK_METRIC_OUTPUT_PATH="${task_metric_output}" \
    RACER_RESULT_DIR="${result_dir}" \
    RACER_LKH_DIR="${lkh_dir}" \
    RACER_ALGORITHM_LABEL="${RACER_ALGORITHM_LABEL:-final_racer_10uav_ideal_no_loss_75_2913_reference_100hz_76800rays}" \
    RACER_WALL_TIME_MULTIPLIER=300 \
    RACER_WALL_TIME_GRACE_SECONDS=600 \
    "${legacy_entrypoint}" >"${result_dir}/supervisor.log" 2>&1
  local status=$?
  set -e
  printf '%s\n' "${status}" >"${result_dir}/runner_exit_status.txt"
  if rg -q -- '--fixed-exploration-horizon' \
      "${result_dir}/warehouse_full_distributed_isaac.log"; then
    printf 'REPRO_RUN_ERROR legacy run unexpectedly enabled fixed exploration horizon\n' >&2
    return 1
  fi
  if [[ "${status}" -ne 0 ]]; then
    printf 'REPRO_RUN_WARNING legacy runner exited %s; result validation will decide validity\n' \
      "${status}" >&2
  fi
  if [[ "${passive_task_metrics}" == "true" ]]; then
    merge_passive_metrics "${label}" "${duration}"
  fi
  return 0
}

merge_passive_metrics() {
  local label="$1"
  local duration="$2"
  python3 - \
    "${suite}/${label}/warehouse_full_distributed_result.json" \
    "${suite}/${label}/passive_task_metrics.jsonl" \
    "${ground_truth_occupied_voxels_path}" \
    "${task_metric_sample_period_s}" "${duration}" <<'PY'
import json
import math
from pathlib import Path
import sys

result_path = Path(sys.argv[1])
history_path = Path(sys.argv[2])
gt_path = Path(sys.argv[3])
sample_period_s = float(sys.argv[4])
duration_s = float(sys.argv[5])
if not result_path.is_file() or not history_path.is_file():
    raise SystemExit("REPRO_METRIC_ERROR missing result or passive metric history")
history = []
for line in history_path.read_text().splitlines():
    if not line.strip():
        continue
    item = json.loads(line)
    values = (
        float(item["time_s"]),
        float(item["redundant_exploration_ratio"]),
        float(item["bs_global_map_iou"]),
    )
    if not all(math.isfinite(value) for value in values):
        raise SystemExit("REPRO_METRIC_ERROR passive metric contains NaN/Inf")
    history.append(item)
history.sort(key=lambda item: float(item["time_s"]))
deduplicated = {}
for item in history:
    stamp = round(float(item["time_s"]), 9)
    if stamp <= duration_s + sample_period_s:
        deduplicated[stamp] = item
history = [deduplicated[key] for key in sorted(deduplicated)]
if not history:
    raise SystemExit("REPRO_METRIC_ERROR passive metric history is empty")
result = json.loads(result_path.read_text())
statistics = result.setdefault("communication", {}).setdefault("statistics", {})
statistics["task_quality_history"] = history
statistics["task_metric_observer_mode"] = "async_passive_legacy_overlay"
statistics["task_metric_sample_period_s"] = sample_period_s
statistics["ground_truth_occupied_voxels"] = sum(
    1 for line in gt_path.read_text().splitlines() if line.strip()
)
statistics["redundant_exploration_ratio"] = float(
    history[-1]["redundant_exploration_ratio"]
)
statistics["bs_global_map_iou"] = float(history[-1]["bs_global_map_iou"])
temporary = result_path.with_suffix(result_path.suffix + ".tmp")
temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
temporary.replace(result_path)
print(
    "REPRO_METRIC_MERGE_OK"
    f" samples={len(history)} first={history[0]['time_s']}"
    f" last={history[-1]['time_s']}"
)
PY
}

validate_case() {
  local label="$1"
  local minimum_elapsed="$2"
  python3 - "${suite}/${label}/warehouse_full_distributed_result.json" \
    "${minimum_elapsed}" "${passive_task_metrics}" \
    "${task_metric_sample_period_s}" "${start_positions}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
minimum_elapsed = float(sys.argv[2])
passive_task_metrics = sys.argv[3] == "true"
sample_period_s = float(sys.argv[4])
flat_starts = [float(value) for value in sys.argv[5].split()]
expected_starts = [
    flat_starts[index:index + 3]
    for index in range(0, len(flat_starts), 3)
]
data = json.loads(path.read_text())
metrics = data.get("metrics", {})
stats = data.get("communication", {}).get("statistics", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < minimum_elapsed:
    errors.append("simulation duration is too short")
if sorted(data.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(metrics.get("collision_events", -1)) != 0:
    errors.append("collision detected")
if int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("physics contact detected")
if data.get("communication", {}).get("mode") != "ideal":
    errors.append("communication mode is not ideal")
if metrics.get("start_positions") != expected_starts:
    errors.append("actual starts differ from the requested layout")
if int(stats.get("attempted_packets", 0)) > 0:
    if int(stats.get("delivered_packets", -1)) != int(stats["attempted_packets"]):
        errors.append("ideal communication was not lossless")
if passive_task_metrics:
    history = stats.get("task_quality_history", [])
    expected = minimum_elapsed / sample_period_s
    if len(history) < 0.90 * expected:
        errors.append(
            f"passive task history is too short: {len(history)} < {0.90 * expected:.0f}"
        )
    times = [float(item.get("time_s", -1.0)) for item in history]
    if times and times[-1] < minimum_elapsed - 2.0 * sample_period_s:
        errors.append("passive task history does not span the simulation")
    gaps = [right - left for left, right in zip(times, times[1:])]
    if gaps and max(gaps) > 2.1 * sample_period_s:
        errors.append(f"passive task history has a large gap: {max(gaps):.6f}s")
    for item in history:
        redundancy = float(item.get("redundant_exploration_ratio", -1.0))
        map_iou = float(item.get("bs_global_map_iou", -1.0))
        if not (0.0 <= redundancy <= 1.0 and 0.0 <= map_iou <= 1.0):
            errors.append("passive task metric is outside [0, 1]")
            break
launch_path = path.parent / "warehouse_full_distributed_launch.log"
if not launch_path.is_file():
    errors.append("launch log is missing")
else:
    launch_text = launch_path.read_text(errors="replace")
    coordinator_marker = (
        "[racer_original_exploration_1]: Initial spatial partition"
    )
    if coordinator_marker not in launch_text:
        errors.append("UAV 1 did not publish the original initial partition")
    if "RACER_LOCAL_COMPONENT_ENABLED" in launch_text or \
            "RACER_ORACLE_GLOBAL_ASSIGNMENT" in launch_text:
        errors.append("a non-original assignment mode was active")
if errors:
    raise SystemExit("\n".join(f"REPRO_VALIDATION_ERROR {e}" for e in errors))
print(
    "REPRO_VALIDATION_OK"
    f" elapsed={metrics.get('elapsed')}"
    f" coverage={metrics.get('mapping_coverage_joint')}"
    f" total_path={metrics.get('total_path_length')}"
    f" task_quality_samples={len(stats.get('task_quality_history', []))}"
)
PY
}

validate_coverage_tolerance() {
  python3 - "${suite}/${formal_label}/warehouse_full_distributed_result.json" \
    "${reference_coverage}" "${coverage_relative_tolerance}" <<'PY'
import json
from pathlib import Path
import sys
result = json.loads(Path(sys.argv[1]).read_text())
reference = float(sys.argv[2])
tolerance = float(sys.argv[3])
coverage = float(result.get("metrics", {}).get("mapping_coverage_joint", -1.0))
relative_error = abs(coverage - reference) / reference
if relative_error > tolerance:
    raise SystemExit(
        "REPRO_COVERAGE_REJECTED"
        f" coverage={coverage} reference={reference}"
        f" relative_error={relative_error} tolerance={tolerance}"
    )
print(
    "REPRO_COVERAGE_ACCEPTED"
    f" coverage={coverage} reference={reference}"
    f" relative_error={relative_error} tolerance={tolerance}"
)
PY
}

mode="${1:---run}"
static_checks
if [[ "${mode}" == "--check" ]]; then
  resource_checks
  exit 0
fi
if [[ "${mode}" != "--run" ]]; then
  printf 'Usage: %s [--check|--run]\n' "$0" >&2
  exit 2
fi

resource_checks
write_manifest
printf '%s\n' "$$" >"${suite}/supervisor.pid"
if [[ ! -f "${suite}/preflight_15s/warehouse_full_distributed_result.json" ]]; then
  run_case preflight_15s 15 "${RACER_PREFLIGHT_DOMAIN_ID:-78}"
else
  printf 'REPRO_RESUME reusing completed preflight result\n'
fi
validate_case preflight_15s 14
sleep 30
if [[ ! -f "${suite}/${formal_label}/warehouse_full_distributed_result.json" ]]; then
  run_case "${formal_label}" "${formal_duration}" "${RACER_FORMAL_DOMAIN_ID:-79}"
else
  printf 'REPRO_RESUME reusing completed formal result\n'
fi
{
  validate_case "${formal_label}" "$((formal_duration - 1))"
  if [[ "${custom_start_positions}" == "false" ]]; then
    validate_coverage_tolerance
  else
    printf 'REPRO_COVERAGE_REFERENCE_SKIPPED custom start layout\n'
  fi
} | tee "${suite}/summary.txt"
printf '%s\n' completed >"${suite}/run_state.txt"
