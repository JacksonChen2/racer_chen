#!/usr/bin/env bash
set -euo pipefail

current_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
repo_root="$(cd "${current_workspace}/.." && pwd)"
legacy_workspace="${repo_root}/hybrid_communication_racer copy"
legacy_entrypoint="${legacy_workspace}/run_warehouse_simple_sionna.sh"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${current_workspace}/experiments/reproduce_75pct_legacy_${run_id}}"
passive_task_metrics="${RACER_PASSIVE_TASK_METRICS:-false}"
task_metric_sample_period_s="${RACER_TASK_METRIC_SAMPLE_PERIOD_S:-0.1}"
ground_truth_occupied_voxels_path="${RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH:-}"
reference_coverage="${RACER_REFERENCE_COVERAGE:-0.750511111111}"
coverage_relative_tolerance="${RACER_COVERAGE_RELATIVE_TOLERANCE:-0.10}"

scene_usd="${repo_root}/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
vehicle_usd="${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
sionna_scene_xml="${repo_root}/ros2_original_fidelity_sionna_ws/experiments/warehouse_full_new_ap_10uav_five_good_bs_sites_ideal_50hz_900s_20260825_182420/sionna_scene/warehouse.xml"
fastdds_profile="${legacy_workspace}/config/fastdds_large_scale.xml"

# Exact starts recorded by the 75.0511111111% run on 2026-08-31.
start_positions="\
-20.75 26 0.75  -19.25 26 0.75  \
-18.75 5 0.75   -17.25 5 0.75  \
-11.75 14.25 0.75  -11.75 15.75 0.75  \
-0.75 27 0.75   0.75 27 0.75  \
-3.75 5 0.75   -2.25 5 0.75"

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
  check_sha256 0c1136528a573b0afa009c5e47aae95896b83b704016dcaec0680ab92d2c84aa \
    "${legacy_entrypoint}"
  check_sha256 23852369427d0d8d97f495d8b895e36e7c3731c1b430b07af233e5f157069a29 \
    "${legacy_workspace}/src/racer_isaac_adapter/isaac_sim/original_racer_isaac.py"
  check_sha256 7ab69559bb83ed017924160fc7a803ef4c2ff2de6ad1b2ddfd56329eadec84ca \
    "${legacy_workspace}/install/racer_sionna_comm/lib/racer_sionna_comm/racer_sionna_communication_proxy"
  check_sha256 58c717d1811d44b9069afdb412b54ed1d29e57fc3ca37af7d5a6d8a3c61dfc72 \
    "${legacy_workspace}/build/racer_original_core/racer_original_exploration_node"
  check_sha256 bb3515eb6a7ba0685fb0f05b75c8a339822c13046c7bce4abe386b58b6d973d7 \
    "${legacy_workspace}/build/racer_original_core/racer_original_lkh_server"
  check_sha256 cae043dac59f661b3729575451763513689b9d050e0accdbab1ffcccd073f7ca \
    "${legacy_workspace}/build/racer_original_core/racer_original_traj_server"
  check_sha256 2d625f59fc5b6f19e3e4f7b9b68f0e26e57ca9fd4bed7dc167f43d0f48295bd2 \
    "${fastdds_profile}"
  check_sha256 e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e \
    "${scene_usd}"
  check_sha256 c0d3c998ce8202fd46f1d51a71ce5b68fb5a3e63338575e335597be847cb4a8c \
    "${vehicle_usd}"
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
  if [[ "${passive_task_metrics}" == "true" ]]; then
    [[ -n "${RACER_DEBUG_OVERLAY_SETUP:-}" &&
       -f "${RACER_DEBUG_OVERLAY_SETUP}" ]] || {
      printf 'REPRO_CHECK_ERROR passive metrics overlay is not built\n' >&2
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
  printf 'REPRO_STATIC_CHECK_OK legacy source, binaries, DDS profile, and assets match\n'
}

resource_checks() {
  local available_cores gpu_apps expected_cpu_threads allow_shared_gpu
  available_cores="$(nproc)"
  expected_cpu_threads="${RACER_EXPECTED_CPU_THREADS:-32}"
  allow_shared_gpu="${RACER_ALLOW_SHARED_GPU:-0}"
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
    "${fastdds_profile}" "${start_positions}" <<'PY'
import hashlib
import json
import os
from pathlib import Path
import sys

output = Path(sys.argv[1])
paths = [Path(value) for value in sys.argv[2:7]]
starts = [float(value) for value in sys.argv[7].split()]
manifest = {
    "reference_result": {
        "coverage": float(os.environ.get("RACER_REFERENCE_COVERAGE", "0.750511111111")),
        "date": "2026-08-31",
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
        "formal_s": 300,
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
        "expected_cpu_threads": int(os.environ.get("RACER_EXPECTED_CPU_THREADS", "32")),
        "allow_shared_gpu": os.environ.get("RACER_ALLOW_SHARED_GPU", "0") == "1",
    },
    "passive_task_metrics": {
        "enabled": os.environ.get("RACER_PASSIVE_TASK_METRICS", "false") == "true",
        "observer_mode": os.environ.get("RACER_TASK_METRIC_OBSERVER_MODE", "off"),
        "sample_period_s": float(os.environ.get("RACER_TASK_METRIC_SAMPLE_PERIOD_S", "0.1")),
        "ground_truth_occupied_voxels_path": os.environ.get("RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH", ""),
        "coverage_relative_tolerance": float(os.environ.get("RACER_COVERAGE_RELATIVE_TOLERANCE", "0.10")),
        "overlay_setup": os.environ.get("RACER_DEBUG_OVERLAY_SETUP", ""),
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
  local lkh_dir="/tmp/racer_reproduce_75pct_${run_id}_${label}_lkh"
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
    RACER_TASK_METRIC_OBSERVER_MODE="${RACER_TASK_METRIC_OBSERVER_MODE:-off}" \
    RACER_TASK_METRIC_SAMPLE_PERIOD_S="${task_metric_sample_period_s}" \
    RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${ground_truth_occupied_voxels_path}" \
    RACER_TASK_METRIC_OUTPUT_PATH="${task_metric_output}" \
    RACER_RESULT_DIR="${result_dir}" \
    RACER_LKH_DIR="${lkh_dir}" \
    RACER_ALGORITHM_LABEL="pairwise_robust_10uav_takeoff_fixed_ideal_no_loss_takeoff_fixed_300s_100hz_76800rays" \
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
    "${task_metric_sample_period_s}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
minimum_elapsed = float(sys.argv[2])
passive_task_metrics = sys.argv[3] == "true"
sample_period_s = float(sys.argv[4])
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
  python3 - "${suite}/formal_300s/warehouse_full_distributed_result.json" \
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
  run_case preflight_15s 15 78
else
  printf 'REPRO_RESUME reusing completed preflight result\n'
fi
validate_case preflight_15s 14
sleep 30
if [[ ! -f "${suite}/formal_300s/warehouse_full_distributed_result.json" ]]; then
  run_case formal_300s 300 79
else
  printf 'REPRO_RESUME reusing completed formal result\n'
fi
{
  validate_case formal_300s 299
  validate_coverage_tolerance
} | tee "${suite}/summary.txt"
printf '%s\n' completed >"${suite}/run_state.txt"
