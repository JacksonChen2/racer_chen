#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${final_root}/ros2_ws"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
random_seed="${RACER_RANDOM_SEED:-42}"
bandwidth_hz="${RACER_BANDWIDTH_HZ:-100000000.0}"
resource_blocks="${RACER_RESOURCE_BLOCKS:-66}"
uav_tx_power_dbm="${RACER_UAV_TX_POWER_DBM:-23}"
coverage_update_rate_hz="${RACER_COVERAGE_UPDATE_RATE_HZ:-0.5}"
initial_assignment_perfect_delivery="${RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY:-false}"
exploration_assignment_mode="${RACER_EXPLORATION_ASSIGNMENT_MODE:-original}"
duration="${RACER_EXPERIMENT_DURATION:-300}"
suite="${RACER_SUITE_DIR:-${final_root}/results/sionna_distributed_10uav_5sites_300s_${run_id}}"
case_label="formal_${duration}s"
case_dir="${suite}/${case_label}"
canonical_selection="${final_root}/config/warehouse_full_10uav_five_sites_layout.json"
selection="${RACER_START_LAYOUT_FILE:-${canonical_selection}}"
reference_result="${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json"
scene_usd="${final_root}/assets/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
vehicle_usd="${final_root}/assets/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
sionna_scene_xml="${final_root}/assets/sionna_scene/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/.sionna_runtime}"
overlay_setup="${final_root}/passive_metrics_overlay_ws/install/setup.bash"
communication_overlay_setup="${final_root}/sionna_distributed_overlay_ws/install/setup.bash"
ground_truth="${final_root}/data/gt_occupied_voxels.txt"
passive_metrics="${case_dir}/passive_task_metrics.jsonl"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""
mode="${1:---run}"

if [[ "${mode}" != "--check" && "${mode}" != "--run" ]]; then
  printf 'Usage: %s [--check|--run]\n' "$0" >&2
  exit 2
fi
if ! [[ "${duration}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_EXPERIMENT_DURATION must be a positive integer: %s\n' \
    "${duration}" >&2
  exit 2
fi

mkdir -p "${case_dir}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

stop_experiment() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${suite}/run_state.txt"
  printf 'STOPPED time=%s\n' "$(date --iso-8601=seconds)"
  exit 130
}
trap stop_experiment INT TERM HUP

actual_scene_sha256="$(sha256sum "${scene_usd}" | awk '{print $1}')"
actual_sionna_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_sionna_sha256}" != "${expected_sionna_sha256}" ]]; then
  printf 'Scene changed: USD=%s Sionna=%s\n' \
    "${actual_scene_sha256}" "${actual_sionna_sha256}" >&2
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  exit 2
fi
if [[ ! -d "${sionna_runtime}/sionna" ]]; then
  printf 'Missing Sionna RT runtime: %s\n' "${sionna_runtime}" >&2
  printf '%s\n' "blocked:missing_sionna_runtime" >"${suite}/run_state.txt"
  exit 2
fi
if [[ ! -f "${overlay_setup}" || ! -f "${communication_overlay_setup}" ||
      ! -s "${ground_truth}" ]]; then
  printf 'Missing final_racer metric/communication overlay or GT data\n' >&2
  printf '%s\n' "blocked:missing_passive_metrics" >"${suite}/run_state.txt"
  exit 2
fi

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]] ||
   [[ ! "${resource_blocks}" =~ ^[1-9][0-9]*$ ]] ||
   ! python3 -c 'import math,sys; bandwidth=float(sys.argv[1]); resource_blocks=int(sys.argv[2]); occupied=12.0*120000.0*resource_blocks; raise SystemExit(not (math.isfinite(bandwidth) and bandwidth > 0.0 and occupied <= bandwidth))' "${bandwidth_hz}" "${resource_blocks}"; then
  printf 'Expected exactly 10 XYZ start positions in %s\n' "${selection}" >&2
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
  exit 2
fi
if ! python3 -c 'import math,sys; rate=float(sys.argv[1]); raise SystemExit(not (math.isfinite(rate) and rate > 0.0))' "${coverage_update_rate_hz}"; then
  printf 'RACER_COVERAGE_UPDATE_RATE_HZ must be a positive finite number: %s\n' \
    "${coverage_update_rate_hz}" >&2
  printf '%s\n' "blocked:invalid_coverage_update_rate" >"${suite}/run_state.txt"
  exit 2
fi
if ! python3 -c 'import math,sys; power=float(sys.argv[1]); raise SystemExit(not math.isfinite(power))' "${uav_tx_power_dbm}"; then
  printf 'RACER_UAV_TX_POWER_DBM must be finite: %s\n' \
    "${uav_tx_power_dbm}" >&2
  printf '%s\n' "blocked:invalid_uav_tx_power" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${initial_assignment_perfect_delivery}" != "true" &&
      "${initial_assignment_perfect_delivery}" != "false" ]]; then
  printf 'RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY must be true or false: %s\n' \
    "${initial_assignment_perfect_delivery}" >&2
  printf '%s\n' "blocked:invalid_initial_assignment_perfect_delivery" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${exploration_assignment_mode}" != "original" &&
      "${exploration_assignment_mode}" != "local_component" &&
      "${exploration_assignment_mode}" != "global_cooperative" ]]; then
  printf 'RACER_EXPLORATION_ASSIGNMENT_MODE is invalid: %s\n' \
    "${exploration_assignment_mode}" >&2
  printf '%s\n' "blocked:invalid_assignment_mode" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${exploration_assignment_mode}" == "local_component" &&
      "${initial_assignment_perfect_delivery}" != "false" ]]; then
  printf 'local_component requires normal Sionna startup delivery.\n' >&2
  printf '%s\n' "blocked:local_component_perfect_initial_delivery" >"${suite}/run_state.txt"
  exit 2
fi
if [[ ! -f "${reference_result}" ]]; then
  printf 'Missing reference result: %s\n' "${reference_result}" >&2
  printf '%s\n' "blocked:missing_reference_result" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${selection}" == "${canonical_selection}" ]] &&
   ! python3 - "${selection}" "${reference_result}" <<'PY'
import json
import os
from pathlib import Path
import sys

selection = json.loads(Path(sys.argv[1]).read_text())
reference = json.loads(Path(sys.argv[2]).read_text())
if selection["start_positions"] != reference["metrics"]["start_positions"]:
    raise SystemExit("canonical starts differ from the final_racer 75.2913% reference")
PY
then
  printf 'Start layout differs from reference result: %s\n' \
    "${reference_result}" >&2
  printf '%s\n' "blocked:start_layout_reference_mismatch" >"${suite}/run_state.txt"
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
core_prefix="$(ros2 pkg prefix racer_original_core)"
msgs_prefix="$(ros2 pkg prefix racer_fidelity_msgs)"
if [[ "${core_prefix}" != "${workspace}/install/racer_original_core" ||
      "${msgs_prefix}" != "${workspace}/install/racer_fidelity_msgs" ]]; then
  printf 'Wrong ROS overlay: core=%s messages=%s\n' "${core_prefix}" "${msgs_prefix}" >&2
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  exit 2
fi
if [[ "${mode}" == "--check" ]]; then
  set +u
  source "${communication_overlay_setup}"
  set -u
  comm_prefix="$(ros2 pkg prefix racer_sionna_comm)"
  if [[ "${comm_prefix}" != "${final_root}/sionna_distributed_overlay_ws/install/racer_sionna_comm" ]]; then
    printf 'Wrong Sionna communication overlay: %s\n' "${comm_prefix}" >&2
    exit 2
  fi
  printf 'SIONNA_STATIC_CHECK_OK workspace=%s runtime=%s assignment_mode=%s bandwidth_hz=%s resource_blocks=%s uav_tx_power_dbm=%s coverage_callback_hz=%s initial_assignment_perfect_delivery=%s seed=%s\n' \
    "${workspace}" "${sionna_runtime}" "${exploration_assignment_mode}" "${bandwidth_hz}" \
    "${resource_blocks}" "${uav_tx_power_dbm}" \
    "${coverage_update_rate_hz}" "${initial_assignment_perfect_delivery}" \
    "${random_seed}"
  printf '%s\n' "checked" >"${suite}/run_state.txt"
  exit 0
fi

python3 - "${suite}/experiment_manifest.json" "${selection}" "${scene_usd}" \
  "${actual_scene_sha256}" "${sionna_scene_xml}" "${actual_sionna_sha256}" \
  "${random_seed}" "${bandwidth_hz}" "${resource_blocks}" \
  "${reference_result}" "${coverage_update_rate_hz}" \
  "${uav_tx_power_dbm}" "${initial_assignment_perfect_delivery}" \
  "${exploration_assignment_mode}" "${duration}" <<'PY'
import json
import os
from pathlib import Path
import sys

output = Path(sys.argv[1])
selection_path = Path(sys.argv[2])
selection = json.loads(selection_path.read_text())
assignment_mode = sys.argv[14]
duration_s = int(sys.argv[15])
manifest = {
    "algorithm": (
        "final_racer_local_component"
        if assignment_mode == "local_component"
        else "final_racer_pairwise_robust"
    ),
    "exploration_assignment_mode": assignment_mode,
    "workspace": str(output.parents[1] / "ros2_ws"),
    "scene": "warehouse_full_with_industrial_ap_user_modified_20260825",
    "scene_usd": sys.argv[3],
    "scene_usd_sha256": sys.argv[4],
    "sionna_scene_xml": sys.argv[5],
    "sionna_scene_xml_sha256": sys.argv[6],
    "layout": selection["layout"],
    "selection_rule": selection["selection_rule"],
    "layout_selection_file": str(selection_path),
    "reference_result": sys.argv[10],
    "regions": selection["regions"],
    "start_positions": selection["start_positions"],
    "drone_count": 10,
    "duration_s": duration_s,
    "physics_rate_hz": 100,
    "sensor_rate_hz": 10,
    "camera_ray_budget": 76800,
    "sensor_worker_count": 8,
    "random_seed": int(sys.argv[7]),
    "coverage_callback": {
        "rate_hz": float(sys.argv[11]),
        "period_s": 1.0 / float(sys.argv[11]),
        "scope": "per-UAV RACER map coverage callback",
        "reference_design": "same 0.5 Hz callback as perfect communication",
    },
    "communication": {
        "map_perfect_delivery": os.environ.get("RACER_MAP_PERFECT_DELIVERY", "false") == "true",
        "perfect_map_topics": ["chunk_data", "chunk_stamps"] if os.environ.get("RACER_MAP_PERFECT_DELIVERY") == "true" else [],
        "mode": "sionna",
        "network_topology": "distributed",
        "require_sionna": True,
        "radio_map_cache_enabled": False,
        "uav_tx_power_dbm": float(sys.argv[12]),
        "initial_assignment_perfect_delivery": sys.argv[13] == "true",
        "initial_assignment_perfect_delivery_scope": (
            "startup packets use normal Sionna transport with no delivery bypass"
            if sys.argv[13] == "false"
            else (
                "normal Sionna bidirectional-neighbor discovery and component-scoped ACKs"
                if assignment_mode == "local_component"
                else "drone-state exchange through epoch 1 assignment and 10/10 acknowledgements"
            )
        ),
        "post_initial_assignment_mode": "sionna",
        "fixed_mcs_index": 14,
        "fixed_mcs_modulation": "16QAM",
        "bandwidth_hz": float(sys.argv[8]),
        "subcarrier_spacing_hz": 120000.0,
        "resource_blocks": int(sys.argv[9]),
        "occupied_bandwidth_hz": 12.0 * 120000.0 * int(sys.argv[9]),
        "shared_uav_ofdma": True,
        "uav_transport": "UDP",
        "uav_udp_directed_unicast": True,
        "bs_enabled": False,
        "max_retries": 0,
        "bs_max_retries": 0,
    },
    "task_metrics": {
        "observer_mode": "off",
        "reason": "keep the final_racer exploration path free of training-only map/trajectory inspection",
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

export ROS_DOMAIN_ID="${RACER_DISCOVERY_DOMAIN_ID:-84}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${duration}"
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_TRIGGER_DELAY_S=5.0
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_STARTUP_FREE_SPACE_YAW=0
export RACER_STARTUP_SCAN_DURATION=0.0
export RACER_STARTUP_UNKNOWN_CORRIDOR_DISTANCE=0.0
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED="${random_seed}"
export RACER_START_POSITIONS="${starts}"
export RACER_COVERAGE_UPDATE_RATE_HZ="${coverage_update_rate_hz}"
export RACER_DEBUG_OVERLAY_SETUP="${overlay_setup}"
export RACER_COMM_OVERLAY_SETUP="${communication_overlay_setup}"
export RACER_TASK_METRIC_OBSERVER_MODE=off
export RACER_TASK_METRIC_SAMPLE_PERIOD_S=0.1
export RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${ground_truth}"
export RACER_TASK_METRIC_OUTPUT_PATH="${passive_metrics}"
export RACER_EXPLORATION_ASSIGNMENT_MODE="${exploration_assignment_mode}"

export RACER_SCENE_USD="${scene_usd}"
export RACER_VEHICLE_USD="${vehicle_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY="${initial_assignment_perfect_delivery}"
export RACER_RL_BS_SCHEDULER_ENABLED=false
export RACER_RL_SYNC_ENABLED=false
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_DIRECTED_MESSAGE_UNICAST=false
export RACER_CHUNK_DATA_PRE_ENQUEUE_DEDUP=false
export RACER_CHUNK_DATA_MAX_PENDING_PER_LINK=0
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_UAV_TX_POWER_DBM="${uav_tx_power_dbm}"
export RACER_BANDWIDTH_HZ="${bandwidth_hz}"
export RACER_RESOURCE_BLOCKS="${resource_blocks}"
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_FIXED_MCS_INDEX=14
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/final_racer_10uav_sionna_bw${bandwidth_hz}_rb${resource_blocks}_${duration}s_${uav_tx_power_dbm}dbm_mcs14_${run_id}_lkh"
export RACER_ALGORITHM_LABEL="${RACER_ALGORITHM_LABEL:-final_racer_${exploration_assignment_mode}_10uav_no_bs_sionna_bw${bandwidth_hz}_rb${resource_blocks}_100hz_10hz_76800rays_${duration}s_uav${uav_tx_power_dbm}dbm_mcs14_initperfect${initial_assignment_perfect_delivery}}"

printf '%s\n' "running" >"${suite}/run_state.txt"
printf 'START time=%s workspace=final_racer mode=sionna topology=distributed assignment_mode=%s bs=off drones=10 duration=%s bandwidth_hz=%s resource_blocks=%s physics_hz=100 sensor_hz=10 coverage_callback_hz=%s rays=76800 uav_dbm=%s mcs=14 retries=0 initial_assignment_perfect_delivery=%s domain=%s seed=%s\n' \
  "$(date --iso-8601=seconds)" "${exploration_assignment_mode}" "${duration}" "${bandwidth_hz}" "${resource_blocks}" \
  "${coverage_update_rate_hz}" "${uav_tx_power_dbm}" \
  "${initial_assignment_perfect_delivery}" "${ROS_DOMAIN_ID}" "${random_seed}"
set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
active_child=$!
printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
wait "${active_child}"
runner_status=$?
active_child=""
set -e
printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

result_path="${case_dir}/warehouse_full_distributed_result.json"
if [[ -f "${result_path}" && -f "${passive_metrics}" ]]; then
  python3 "${final_root}/scripts/merge_passive_metrics.py" \
    "${result_path}" "${passive_metrics}" "${ground_truth}" 0.1 "${duration}" \
    >"${case_dir}/passive_metric_merge.log" 2>&1
fi

set +e
python3 - "${case_dir}" "${random_seed}" "${bandwidth_hz}" \
  "${resource_blocks}" "${selection}" "${coverage_update_rate_hz}" \
  "${uav_tx_power_dbm}" "${initial_assignment_perfect_delivery}" \
  "${exploration_assignment_mode}" "${duration}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
import os
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
expected_seed = int(sys.argv[2])
expected_bandwidth_hz = float(sys.argv[3])
expected_resource_blocks = int(sys.argv[4])
selection = json.loads(Path(sys.argv[5]).read_text())
expected_coverage_rate_hz = float(sys.argv[6])
expected_uav_tx_power_dbm = float(sys.argv[7])
expected_initial_assignment_perfect_delivery = sys.argv[8] == "true"
expected_assignment_mode = sys.argv[9]
expected_duration_s = float(sys.argv[10])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"INTEGRITY_ERROR expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
phy = stats.get("phy", {})
evidence = result.get("algorithm_evidence", {})
acceptance = result.get("acceptance", {})
errors = []
expected_map_perfect = os.environ.get("RACER_MAP_PERFECT_DELIVERY", "false") == "true"
if bool(stats.get("map_perfect_delivery_enabled", False)) != expected_map_perfect:
    errors.append("map perfect delivery mode mismatch")
if expected_map_perfect and int(stats.get("map_perfect_messages", 0)) <= 0:
    errors.append("no map messages used perfect delivery")
if int(result.get("random_seed", -1)) != expected_seed:
    errors.append(f"random seed is not {expected_seed}")
if float(metrics.get("elapsed", 0.0)) < expected_duration_s - 1.0:
    errors.append(f"simulation did not reach {expected_duration_s:g} seconds")
if not acceptance.get("isaac_exit_ok", False):
    errors.append("Isaac process did not exit cleanly")
if float(metrics.get("physics_rate_hz", 0.0)) != 100.0:
    errors.append("physics rate is not 100 Hz")
if int(metrics.get("camera_ray_budget", 0)) != 76800:
    errors.append("camera ray budget is not 76800")
if communication.get("mode") != "sionna" or communication.get("sionna_ready") is not True:
    errors.append("Sionna RT was not active")
if int(communication.get("exact_link_samples", 0)) <= 0:
    errors.append("Sionna produced no exact link samples")
if int(phy.get("fixed_mcs_index", -1)) != 14 or float(phy.get("uav_tx_power_dbm", -1)) != expected_uav_tx_power_dbm:
    errors.append(
        f"PHY is not fixed MCS14 at {expected_uav_tx_power_dbm:g} dBm"
    )
if float(phy.get("bandwidth_hz", -1)) != expected_bandwidth_hz:
    errors.append(f"PHY bandwidth is not {expected_bandwidth_hz}")
if int(phy.get("resource_blocks", -1)) != expected_resource_blocks:
    errors.append(f"PHY resource blocks are not {expected_resource_blocks}")
if int(phy.get("max_retries", -1)) != 0 or int(phy.get("bs_max_retries", -1)) != 0:
    errors.append("retransmission is enabled")
if stats.get("network_topology") != "distributed" or stats.get("ap_enabled") is not False:
    errors.append("experiment is not distributed/no-BS")
if stats.get("shared_uav_ofdma_enabled") is not True:
    errors.append("shared UAV OFDMA is not active")
if stats.get("uav_transport") != "UDP" or stats.get("uav_udp_directed_unicast") is not True:
    errors.append("UDP directed unicast is not active")
if bool(stats.get("initial_assignment_perfect_delivery_enabled", False)) != expected_initial_assignment_perfect_delivery:
    errors.append("initial-assignment perfect-delivery mode differs from the request")
if expected_initial_assignment_perfect_delivery:
    if stats.get("initial_assignment_perfect_delivery_complete") is not True:
        errors.append("initial-assignment perfect-delivery window did not close")
    if int(stats.get("initial_assignment_perfect_epoch", 0)) <= 0:
        errors.append("no initial assignment epoch was selected")
    if int(stats.get("initial_assignment_perfect_acks", 0)) != 10:
        errors.append("initial assignment did not receive 10/10 acknowledgements")
    if int(stats.get("initial_assignment_perfect_global_assignment_messages", 0)) <= 0:
        errors.append("no initial global assignment was perfectly delivered")
    if int(stats.get("initial_assignment_perfect_forwarded_packets", 0)) <= 0:
        errors.append("no initial-assignment packets used perfect delivery")
if metrics.get("start_positions") != selection["start_positions"]:
    errors.append("actual starts differ from the final_racer reference layout")
if metrics.get("startup_recovery", {}).get("enabled") is not False:
    errors.append("startup recovery differs from the reference run")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if int(evidence.get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if int(metrics.get("collision_events", -1)) != 0 or int(metrics.get("physics_contact_events", -1)) != 0:
    errors.append("collision/contact detected")
history = stats.get("task_quality_history", [])
if stats.get("task_metric_observer_mode") != "off":
    errors.append("training-only task metric observer is not disabled")
launch_log = case_dir / "warehouse_full_distributed_launch.log"
callback_count = 0
if launch_log.is_file():
    launch_text = launch_log.read_text(errors="replace")
    callback_count = launch_text.count("RACER_MAP_COVERAGE known=")
    if expected_assignment_mode == "local_component":
        if launch_text.count("RACER_LOCAL_COMPONENT_ENABLED") != 10:
            errors.append("local-component mode was not enabled by all ten UAVs")
        if "RACER_LOCAL_COMPONENT_ASSIGN" not in launch_text:
            errors.append("no component coordinator published an initial assignment")
        if launch_text.count("RACER_LOCAL_COMPONENT_APPLIED") != 10:
            errors.append("not all UAVs applied a component-scoped assignment")
        if launch_text.count("RACER_LOCAL_COMPONENT_COMMITTED") != 10:
            errors.append("not all UAVs completed their component-scoped ACK gate")
    elif expected_assignment_mode == "original":
        if "RACER_LOCAL_COMPONENT_ENABLED" in launch_text or \
                "RACER_ORACLE_GLOBAL_ASSIGNMENT" in launch_text:
            errors.append("a non-original assignment mode was active")
else:
    errors.append("launch log is missing")
expected_callback_count = float(
    metrics.get("elapsed", expected_duration_s)
) * 10 * expected_coverage_rate_hz
callback_tolerance = max(20.0, expected_callback_count * 0.08)
if abs(callback_count - expected_callback_count) > callback_tolerance:
    errors.append(
        f"coverage callback count {callback_count} differs from expected "
        f"{expected_callback_count:.1f} at {expected_coverage_rate_hz} Hz"
    )
if errors:
    raise SystemExit("\n".join(f"INTEGRITY_ERROR {error}" for error in errors))
print(
    "INTEGRITY_OK "
    f"coverage={metrics.get('mapping_coverage_joint')} "
    f"collisions={metrics.get('collision_events')} "
    f"attempted={stats.get('attempted_packets')} "
    f"delivered={stats.get('delivered_packets')} "
    f"exact_samples={communication.get('exact_link_samples')} "
    f"coverage_callback_hz={expected_coverage_rate_hz} "
    f"coverage_callback_count={callback_count} "
    f"statistics_samples={len(history)}"
)
PY
validation_status=$?
set -e
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

if [[ "${validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s validation=%s\n' \
    "${runner_status}" "${validation_status}" >"${suite}/run_state.txt"
fi
printf 'FINISH time=%s runner=%s validation=%s\n' \
  "$(date --iso-8601=seconds)" "${runner_status}" "${validation_status}"
exit "${validation_status}"
