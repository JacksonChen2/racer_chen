#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
search_dir="${script_dir}"
workspace_dir=""
while [[ "${search_dir}" != "/" ]]; do
  if [[ -f "${search_dir}/install/setup.bash" ]]; then
    workspace_dir="${search_dir}"
    break
  fi
  search_dir="$(dirname "${search_dir}")"
done
if [[ -z "${workspace_dir}" ]]; then
  printf 'Cannot locate ros2_original_fidelity_sionna_ws; build the workspace first.\n' >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
if [[ -n "${RACER_DEBUG_OVERLAY_SETUP:-}" ]]; then
  # Optional diagnostic overlay (for example an ASan build of the unchanged
  # core). It is never enabled by the normal or visualization entry points.
  source "${RACER_DEBUG_OVERLAY_SETUP}"
fi
set -u

adapter_share="$(ros2 pkg prefix racer_isaac_adapter)/share/racer_isaac_adapter"
communication_share="$(ros2 pkg prefix racer_sionna_comm)/share/racer_sionna_comm"
repo_root="$(realpath "${workspace_dir}/..")"
isaac_root="${ISAAC_SIM_ROOT:-/home/jiazheng/software/isaacsim}"
communication_mode="${RACER_COMMUNICATION_MODE:-sionna}"
network_topology="${RACER_NETWORK_TOPOLOGY:-distributed}"
sionna_runtime="${SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"
duration="${RACER_FIDELITY_DURATION:-900}"
wall_time_multiplier="${RACER_WALL_TIME_MULTIPLIER:-20}"
wall_time_grace="${RACER_WALL_TIME_GRACE_SECONDS:-300}"
drone_count="${RACER_FIDELITY_DRONE_COUNT:-5}"
scenario="${RACER_FIDELITY_SCENARIO:-warehouse_simple}"
headless="${RACER_FIDELITY_HEADLESS:-1}"
visualize="${RACER_FIDELITY_VISUALIZE:-0}"
require_completion="${RACER_REQUIRE_COMPLETION:-1}"
stop_on_completion="${RACER_STOP_ON_COMPLETION:-1}"
coverage_target="${RACER_MAPPING_COVERAGE_TARGET:-0}"
# 640x480 with the upstream skip_pixel=2 produces at most 76,800 samples.
# Keeping them all is necessary for 0.1 m SDF free-space connectivity; a low
# diagnostic budget can leave unknown voxel curtains that the unchanged
# non-optimistic A* correctly refuses to cross.
ray_budget="${RACER_CAMERA_RAY_BUDGET:-76800}"
physics_hz="${RACER_PHYSICS_RATE_HZ:-1000}"
sensor_hz="${RACER_SENSOR_RATE_HZ:-30}"
depth_width="${RACER_DEPTH_WIDTH:-640}"
depth_height="${RACER_DEPTH_HEIGHT:-480}"
interactive_hz="${RACER_INTERACTIVE_RENDER_HZ:-30}"
map_points="${RACER_VISUALIZATION_MAX_MAP_POINTS:-12000}"
record_trajectory_history="${RACER_RECORD_TRAJECTORY_HISTORY:-0}"
result_dir="${RACER_RESULT_DIR:-${workspace_dir}/validation}"
if [[ "${network_topology}" != "distributed" && "${network_topology}" != "ap_assisted" ]]; then
  printf 'RACER_NETWORK_TOPOLOGY must be distributed or ap_assisted.\n' >&2
  exit 2
fi
if [[ ! "${wall_time_multiplier}" =~ ^[1-9][0-9]*$ || ! "${wall_time_grace}" =~ ^[0-9]+$ ]]; then
  printf 'RACER_WALL_TIME_MULTIPLIER must be a positive integer and grace must be non-negative.\n' >&2
  exit 2
fi
mkdir -p "${result_dir}" /tmp/racer_original_fidelity_sionna_lkh
run_tag="${scenario}_${network_topology}"
launch_log="${result_dir}/${run_tag}_launch.log"
isaac_log="${result_dir}/${run_tag}_isaac.log"
result_file="${result_dir}/${run_tag}_result.json"
: > "${launch_log}"
: > "${isaac_log}"

if [[ "${scenario}" == "warehouse_loaded" || "${scenario}" == "warehouse_loaded_center" ]]; then
  # This layer lives next to its relative warehouse.usd dependency.
  default_scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_loaded.usd"
  default_sionna_scene_xml="${workspace_dir}/src/racer_sionna_comm/assets/warehouse_loaded_sionna/warehouse.xml"
  default_radio_map_cache="${workspace_dir}/src/racer_sionna_comm/assets/warehouse_loaded_sionna/hybrid_radio_cache.npz"
elif [[ "${scenario}" == "warehouse_simple" ]]; then
  default_scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_simple_with_industrial_ap.usda"
  default_sionna_scene_xml="${workspace_dir}/src/racer_sionna_comm/assets/warehouse_simple_with_ap_sionna/warehouse.xml"
  default_radio_map_cache="${workspace_dir}/src/racer_sionna_comm/assets/warehouse_simple_with_ap_sionna/hybrid_radio_cache.npz"
else
  printf 'Unsupported RACER_FIDELITY_SCENARIO: %s\n' "${scenario}" >&2
  exit 2
fi
scene_usd="${RACER_SCENE_USD:-${default_scene_usd}}"
sionna_scene_xml="${RACER_SIONNA_SCENE_XML:-${default_sionna_scene_xml}}"
radio_map_cache="${RACER_SIONNA_RADIO_MAP_CACHE:-${default_radio_map_cache}}"
vehicle_usd="${RACER_VEHICLE_USD:-${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd}"
if [[ ! -x "${isaac_root}/python.sh" || ! -f "${scene_usd}" || ! -f "${vehicle_usd}" ]]; then
  printf 'Missing Isaac Python, scene USD, or vehicle USD.\n' >&2
  exit 2
fi
if [[ "${communication_mode}" != "ideal" ]]; then
  if [[ ! -d "${sionna_runtime}/sionna" || ! -f "${sionna_scene_xml}" ]]; then
    printf 'Missing isolated Sionna RT runtime or Warehouse XML scene.\n' >&2
    printf 'Run ./setup_sionna_env.sh and ./prepare_warehouse_simple_sionna_scene.sh first.\n' >&2
    exit 2
  fi
  export PYTHONPATH="${sionna_runtime}${PYTHONPATH:+:${PYTHONPATH}}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-176}"
launch_communication_args=(
  communication_mode:="${communication_mode}"
  network_topology:="${network_topology}"
  require_sionna:=$([[ "${communication_mode}" == "ideal" ]] && printf false || printf true)
  sionna_scene_xml:="${sionna_scene_xml}"
)
if [[ -f "${radio_map_cache}" ]]; then
  launch_communication_args+=(radio_map_cache:="${radio_map_cache}")
fi
launch_debug_args=()
if [[ -n "${RACER_DEBUG_LAUNCH_PREFIX:-}" ]]; then
  # Diagnostic-only wrapper used to obtain native backtraces without changing
  # any planner callback, parameter, or normal launch behavior.
  launch_debug_args+=(--launch-prefix "${RACER_DEBUG_LAUNCH_PREFIX}")
  if [[ -n "${RACER_DEBUG_LAUNCH_PREFIX_FILTER:-}" ]]; then
    launch_debug_args+=(--launch-prefix-filter "${RACER_DEBUG_LAUNCH_PREFIX_FILTER}")
  fi
fi
setsid ros2 launch "${launch_debug_args[@]}" \
  racer_sionna_comm original_racer_warehouse_sionna.launch.py \
  drone_count:="${drone_count}" \
  scenario:="${scenario}" \
  lkh_dir:=/tmp/racer_original_fidelity_sionna_lkh \
  "${launch_communication_args[@]}" \
  >"${launch_log}" 2>&1 &
launch_pid=$!
completion_monitor_pid=""

cleanup() {
  trap - EXIT INT TERM HUP
  if [[ -n "${completion_monitor_pid}" ]]; then
    kill "${completion_monitor_pid}" 2>/dev/null || true
    wait "${completion_monitor_pid}" 2>/dev/null || true
  fi
  # The ros2 launch group leader may exit before every child (notably a
  # trajectory server handling an interrupted callback).  Address the already
  # resolved process group unconditionally so an interrupted diagnostic cannot
  # leak DDS participants into the next domain/run.
  kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

if [[ "${stop_on_completion}" == "1" ]]; then
  # Incrementally follow the launch log.  Long source-faithful runs generate
  # hundreds of MB of upstream diagnostics, so re-reading the whole file once
  # per second would consume substantial I/O without changing the criterion.
  python3 "${adapter_share}/scripts/monitor_completion.py" \
    "${launch_log}" "${drone_count}" "${launch_pid}" &
  completion_monitor_pid=$!
fi

sleep 3
isaac_args=(
  --scenario "${scenario}"
  --scene-usd "${scene_usd}"
  --vehicle-model racer_so3
  --vehicle-usd "${vehicle_usd}"
  --duration "${duration}"
  --drone-count "${drone_count}"
  --camera-ray-budget "${ray_budget}"
  --physics-rate-hz "${physics_hz}"
  --sensor-rate-hz "${sensor_hz}"
  --depth-width "${depth_width}"
  --depth-height "${depth_height}"
  --diagnostics
  --mapping-coverage-target "${coverage_target}"
)
if [[ "${headless}" == "1" && "${visualize}" != "1" ]]; then
  isaac_args+=(--headless --no-animate-propellers)
else
  isaac_args+=(
    --visualize-exploration
    --visualization-max-map-points "${map_points}"
    --interactive-render-hz "${interactive_hz}"
    --propeller-visual-hz 20
    --animate-propellers
  )
fi
if [[ "${record_trajectory_history}" == "1" ]]; then
  isaac_args+=(--record-trajectory-history)
fi

duration_ceiling="$(python3 -c 'import math,sys; print(math.ceil(float(sys.argv[1])))' "${duration}")"
set +e
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "$((duration_ceiling * wall_time_multiplier + wall_time_grace))" \
  "${isaac_root}/python.sh" "${adapter_share}/isaac_sim/original_racer_isaac.py" \
  "${isaac_args[@]}" 2>&1 | tee "${isaac_log}"
isaac_status=${PIPESTATUS[0]}
set -e

if [[ -n "${completion_monitor_pid}" ]]; then
  kill "${completion_monitor_pid}" 2>/dev/null || true
  wait "${completion_monitor_pid}" 2>/dev/null || true
  completion_monitor_pid=""
fi

sleep 2
kill -KILL -- "-${launch_pid}" 2>/dev/null || true
wait "${launch_pid}" 2>/dev/null || true

python3 - "${isaac_log}" "${launch_log}" "${result_file}" \
  "${drone_count}" "${require_completion}" "${isaac_status}" \
  "${communication_mode}" "${scenario}" "${network_topology}" \
  "${scene_usd}" "${sionna_scene_xml}" <<'PY'
import json
import math
from pathlib import Path
import re
import sys

isaac_log, launch_log, result_file = map(Path, sys.argv[1:4])
drone_count = int(sys.argv[4])
require_completion = bool(int(sys.argv[5]))
isaac_status = int(sys.argv[6])
communication_mode = sys.argv[7]
scenario = sys.argv[8]
network_topology = sys.argv[9]
scene_usd = sys.argv[10]
sionna_scene_xml = sys.argv[11]
lines = isaac_log.read_text(errors="replace").splitlines()
prefix = "RACER_3D_ISAAC_RESULT "
matches = [line[len(prefix):] for line in lines if line.startswith(prefix)]
if not matches:
    raise SystemExit("Isaac result line is missing")
metrics = json.loads(matches[-1])
launch_text = launch_log.read_text(errors="replace")
comm_matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", launch_text)
communication_statistics = json.loads(comm_matches[-1]) if comm_matches else {}
sionna_ready = (
    communication_mode == "ideal"
    or (
        "loaded Sionna RT scene" in launch_text
        and "Sionna RT initialization failed" not in launch_text
    )
)
exact_link_samples = (
    communication_statistics.get("sionna_exact_samples", 0)
    + communication_statistics.get("sionna_cache_corrected_samples", 0)
)
communication_active = (
    communication_statistics.get("delivered_packets", 0) > 0
    and (communication_mode == "ideal" or (sionna_ready and exact_link_samples > 0))
)
topology_active = (
    communication_statistics.get("network_topology") == network_topology
    and (
        network_topology == "distributed"
        or communication_statistics.get("ap_global_updates_received", 0) > 0
    )
)
finished = sorted({
    int(value)
    for value in re.findall(
        r"racer_original_exploration_(\d+).*(?:finish exploration|state: FINISH)",
        launch_text,
    )
})
returned = sorted({
    int(value)
    for value in re.findall(
        r"racer_original_exploration_(\d+).*Go back to", launch_text
    )
})
executed = sorted({
    int(value)
    for value in re.findall(
        r"racer_original_exploration_(\d+).*from PUB_TRAJ to EXEC_TRAJ",
        launch_text,
    )
})
def count(pattern):
    return len(re.findall(pattern, launch_text))

evidence = {
    "hgrid_tours": count(r"Grid tour:"),
    "frontier_updates": count(r"Frontier num:"),
    "lkh_atsp_solutions": count(r"Best ATSP solution:"),
    "lkh_acvrp_solutions": count(r"Best ACVRP solution:"),
    "pair_requests": count(r"send opt request"),
    "pair_responses": count(r"get response"),
    "kinodynamic_mid_goals": count(r"Mid goal"),
    "nlopt_trajectory_runs": count(r"Traj opt iter num:"),
    "yaw_plans": count(r"Traj: .*yaw:"),
    "lkh_call_failures": count(r"Fail to solve (?:ATSP|ACVRP)"),
    "process_crashes": count(r"process has died|exit code -11|Segmentation"),
    "tracking_recoveries": count(r"\[trackingLostCallback\]"),
}
normal_completion = len(finished) == drone_count and len(returned) == drone_count
algorithm_pipeline_ok = (
    len(executed) == drone_count
    and evidence["hgrid_tours"] > 0
    and evidence["frontier_updates"] > 0
    and evidence["lkh_atsp_solutions"] > 0
    and (drone_count < 2 or evidence["lkh_acvrp_solutions"] > 0)
    and (drone_count < 2 or evidence["pair_responses"] > 0)
    and evidence["kinodynamic_mid_goals"] > 0
    and evidence["nlopt_trajectory_runs"] > 0
    and evidence["yaw_plans"] > 0
)
min_distance = metrics.get("min_inter_drone")
min_clearance = metrics.get("min_obstacle_clearance")
starts = metrics.get("start_positions", [])
positions = metrics.get("positions", [])
return_errors = [
    math.dist(start, position)
    for start, position in zip(starts, positions)
]
physical_return_ok = (
    len(return_errors) == drone_count
    and all(error < 1.0 for error in return_errors)
)
acceptance = {
    "isaac_exit_ok": isaac_status == 0,
    "zero_collisions": metrics.get("collision_events") == 0,
    "minimum_inter_drone_distance_ok": (
        drone_count < 2 or (min_distance is not None and min_distance >= 1.0)
    ),
    "positive_obstacle_clearance": (
        min_clearance is not None and min_clearance > 0.0
    ),
    "all_agents_executed_original_fsm": len(executed) == drone_count,
    "original_algorithm_pipeline_observed": algorithm_pipeline_ok,
    "lkh_services_healthy": (
        evidence["lkh_call_failures"] == 0
        and evidence["process_crashes"] == 0
    ),
    "normal_completion": normal_completion,
    "physical_return_within_original_1m_threshold": physical_return_ok,
    "communication_proxy_active": communication_active,
    "requested_network_topology_active": topology_active,
    "sionna_rt_active": communication_mode == "ideal" or (
        sionna_ready and exact_link_samples > 0
    ),
}
passed = all(
    value for name, value in acceptance.items()
    if require_completion or name not in (
        "normal_completion",
        "physical_return_within_original_1m_threshold",
    )
)
result = {
    "algorithm": "original_racer_ros1_source_faithful_ros2_cpp_sionna_rt",
    "scene": scene_usd,
    "sionna_scene": sionna_scene_xml,
    "vehicle": "crazyflie_with_racer_dynamics.usd",
    "drone_count": drone_count,
    "require_completion": require_completion,
    "finished_drone_ids": finished,
    "returned_drone_ids": returned,
    "executed_drone_ids": executed,
    "algorithm_evidence": evidence,
    "return_position_errors_m": return_errors,
    "acceptance": acceptance,
    "communication": {
        "mode": communication_mode,
        "network_topology": network_topology,
        "sionna_ready": sionna_ready,
        "exact_link_samples": exact_link_samples,
        "statistics": communication_statistics,
    },
    "passed": passed,
    "metrics": metrics,
}
result_file.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if passed else 1)
PY
