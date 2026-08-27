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
  printf 'Cannot locate ros2_c2_explorer_ws; build the workspace first.\n' >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
if [[ -n "${C2_DEBUG_OVERLAY_SETUP:-}" ]]; then
  # Optional diagnostic overlay (for example an ASan build of the unchanged
  # core). It is never enabled by the normal or visualization entry points.
  source "${C2_DEBUG_OVERLAY_SETUP}"
fi
set -u

package_share="$(ros2 pkg prefix c2_explorer_isaac)/share/c2_explorer_isaac"
repo_root="$(realpath "${workspace_dir}/..")"
isaac_root="${ISAAC_SIM_ROOT:-${HOME}/software/isaacsim}"
duration="${C2_DURATION:-900}"
drone_count="${C2_DRONE_COUNT:-5}"
scenario="${C2_SCENARIO:-warehouse_simple}"
communication_mode="${C2_COMMUNICATION_MODE:-direct}"
random_seed="${C2_RANDOM_SEED:-42}"
max_retries="${C2_COMMUNICATION_MAX_RETRIES:-0}"
debug_opt_output="${C2_DEBUG_OPT_OUTPUT:-0}"
headless="${C2_HEADLESS:-1}"
visualize="${C2_VISUALIZE:-0}"
require_completion="${C2_REQUIRE_COMPLETION:-1}"
stop_on_completion="${C2_STOP_ON_COMPLETION:-1}"
coverage_target="${C2_MAPPING_COVERAGE_TARGET:-0}"
# 640x480 with the upstream skip_pixel=2 produces at most 76,800 samples.
# Keeping them all is necessary for 0.1 m SDF free-space connectivity; a low
# diagnostic budget can leave unknown voxel curtains that the unchanged
# non-optimistic A* correctly refuses to cross.
ray_budget="${C2_CAMERA_RAY_BUDGET:-76800}"
physics_hz="${C2_PHYSICS_RATE_HZ:-1000}"
sensor_hz="${C2_SENSOR_RATE_HZ:-30}"
depth_width="${C2_DEPTH_WIDTH:-640}"
depth_height="${C2_DEPTH_HEIGHT:-480}"
interactive_hz="${C2_INTERACTIVE_RENDER_HZ:-30}"
map_points="${C2_VISUALIZATION_MAX_MAP_POINTS:-12000}"
result_dir="${C2_RESULT_DIR:-${workspace_dir}/validation}"
lkh_dir="${C2_LKH_DIR:-/tmp/c2_explorer_lkh}"
mkdir -p "${result_dir}" "${lkh_dir}"
launch_log="${result_dir}/${scenario}_launch.log"
isaac_log="${result_dir}/${scenario}_isaac.log"
result_file="${result_dir}/${scenario}_result.json"
: > "${launch_log}"
: > "${isaac_log}"

if [[ "${scenario}" == "warehouse_loaded" || "${scenario}" == "warehouse_loaded_center" ]]; then
  # This layer lives next to its relative warehouse.usd dependency.
  default_scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_loaded.usd"
elif [[ "${scenario}" == "warehouse_simple" ]]; then
  default_scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_simple.usd"
elif [[ "${scenario}" == "warehouse_full" ]]; then
  default_scene_usd="${repo_root}/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
else
  printf 'Unsupported C2_SCENARIO: %s\n' "${scenario}" >&2
  exit 2
fi
scene_usd="${C2_SCENE_USD:-${default_scene_usd}}"
default_sionna_scene="${repo_root}/warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_scene="${C2_SIONNA_SCENE_XML:-${default_sionna_scene}}"
radio_map_cache="${C2_SIONNA_RADIO_MAP_CACHE:-}"
vehicle_usd="${C2_VEHICLE_USD:-${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd}"
if [[ ! -x "${isaac_root}/python.sh" || ! -f "${scene_usd}" || ! -f "${vehicle_usd}" ]]; then
  printf 'Missing Isaac Python, scene USD, or vehicle USD.\n' >&2
  exit 2
fi
if [[ "${communication_mode}" != "direct" && "${communication_mode}" != "ideal" && "${communication_mode}" != "sionna" ]]; then
  printf 'C2_COMMUNICATION_MODE must be direct, ideal, or sionna.\n' >&2
  exit 2
fi
if [[ "${communication_mode}" == "sionna" ]]; then
  sionna_runtime="${C2_SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"
  channel_source="${C2_SIONNA_CHANNEL_SOURCE:-${repo_root}/ros2_pairwise_robust_racer_ws/src/racer_sionna_comm/scripts/sionna_channel_node.py}"
  if [[ ! -d "${sionna_runtime}/sionna" || ! -f "${sionna_scene}" || ! -f "${channel_source}" ]]; then
    printf 'Missing Sionna runtime, scene XML, or validated channel source.\n' >&2
    exit 2
  fi
  export PYTHONPATH="${sionna_runtime}${PYTHONPATH:+:${PYTHONPATH}}"
  export C2_SIONNA_CHANNEL_SOURCE="${channel_source}"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-176}"
launch_debug_args=()
if [[ -n "${C2_DEBUG_LAUNCH_PREFIX:-}" ]]; then
  # Diagnostic-only wrapper used to obtain native backtraces without changing
  # any planner callback, parameter, or normal launch behavior.
  launch_debug_args+=(--launch-prefix "${C2_DEBUG_LAUNCH_PREFIX}")
  if [[ -n "${C2_DEBUG_LAUNCH_PREFIX_FILTER:-}" ]]; then
    launch_debug_args+=(--launch-prefix-filter "${C2_DEBUG_LAUNCH_PREFIX_FILTER}")
  fi
fi
launch_radio_cache_args=()
if [[ -n "${radio_map_cache}" ]]; then
  launch_radio_cache_args+=(radio_map_cache:="${radio_map_cache}")
fi
setsid ros2 launch "${launch_debug_args[@]}" \
  c2_explorer_isaac c2_explorer_warehouse.launch.py \
  drone_count:="${drone_count}" \
  scenario:="${scenario}" \
  communication_mode:="${communication_mode}" \
  sionna_scene_xml:="${sionna_scene}" \
  "${launch_radio_cache_args[@]}" \
  random_seed:="${random_seed}" \
  max_retries:="${max_retries}" \
  debug_opt_output:="${debug_opt_output}" \
  lkh_dir:="${lkh_dir}" \
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
  python3 "${package_share}/scripts/monitor_completion.py" \
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
if [[ -n "${C2_START_POSITIONS:-}" ]]; then
  read -r -a start_values <<<"${C2_START_POSITIONS}"
  isaac_args+=(--starts "${start_values[@]}")
fi
if [[ -n "${C2_TRAJECTORY_OUTPUT:-}" ]]; then
  isaac_args+=(
    --trajectory-output "${C2_TRAJECTORY_OUTPUT}"
    --trajectory-record-hz "${C2_TRAJECTORY_RECORD_HZ:-60}"
  )
fi
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

duration_ceiling="$(python3 -c 'import math,sys; print(math.ceil(float(sys.argv[1])))' "${duration}")"
wall_timeout_factor="${C2_WALL_TIMEOUT_FACTOR:-20}"
wall_timeout_grace="${C2_WALL_TIMEOUT_GRACE:-300}"
set +e
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "$((duration_ceiling * wall_timeout_factor + wall_timeout_grace))" \
  "${isaac_root}/python.sh" "${package_share}/isaac_sim/c2_explorer_isaac.py" \
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
  "${scenario}" "${communication_mode}" "${sionna_scene}" <<'PY'
import json
from pathlib import Path
import re
import sys

isaac_log, launch_log, result_file = map(Path, sys.argv[1:4])
drone_count = int(sys.argv[4])
require_completion = bool(int(sys.argv[5]))
isaac_status = int(sys.argv[6])
scenario = sys.argv[7]
communication_mode = sys.argv[8]
sionna_scene = sys.argv[9]
lines = isaac_log.read_text(errors="replace").splitlines()
prefix = "C2_EXPLORER_ISAAC_RESULT "
matches = [line[len(prefix):] for line in lines if line.startswith(prefix)]
if not matches:
    raise SystemExit("Isaac result line is missing")
metrics = json.loads(matches[-1])
launch_text = launch_log.read_text(errors="replace")
finished = sorted({
    int(value)
    for value in re.findall(
        r"c2_explorer_(\d+).*from .* to FINISH",
        launch_text,
    )
})
executed = sorted({
    int(value)
    for value in re.findall(
        r"c2_explorer_(\d+).*from PUB_TRAJ to EXEC_TRAJ",
        launch_text,
    )
})
def count(pattern):
    return len(re.findall(pattern, launch_text))

evidence = {
    "connectivity_graph_constructions": count(r"\[ConnectivityGraph L\d+\] construction time"),
    "hgrid_tours": count(r"Grid tour:"),
    "frontier_tours": count(r"Find frontier tour"),
    "grid_frontier_timing": count(r"Grid tour t:"),
    "acvrp_allocations": count(r"ACVRP total_demand="),
    "meeting_proposals": count(r"(?:sending|send) proposal to"),
    "meeting_commits": count(r"\[Meeting Opt\] Commit drone"),
    "meeting_commit_acks": count(r"recv commit-ack from"),
    "trajectory_plans": count(r"Traj plan time:"),
    "lkh_call_failures": count(r"Fail to solve (?:ATSP|ACVRP)"),
    "process_crashes": count(r"exit code -(?:6|11)|Segmentation"),
    "tracking_recoveries": count(r"\[trackingLostCallback\]"),
}
communication_matches = re.findall(
    r"C2_COMMUNICATION_STATS (\{[^\n]+\})", launch_text
)
communication_statistics = (
    json.loads(communication_matches[-1]) if communication_matches else {}
)
communication_drops = sum(
    int(communication_statistics.get(name, 0))
    for name in (
        "dropped_no_link", "dropped_per", "dropped_queue", "dropped_ttl"
    )
)
sionna_exact_samples = (
    int(communication_statistics.get("sionna_exact_samples", 0))
    + int(communication_statistics.get("sionna_cache_corrected_samples", 0))
)
sionna_ready = "loaded Sionna RT scene" in launch_text
if communication_mode == "direct":
    communication_ok = True
elif communication_mode == "ideal":
    communication_ok = (
        int(communication_statistics.get("attempted_packets", 0)) > 0
        and int(communication_statistics.get("delivered_packets", -1))
        == int(communication_statistics.get("attempted_packets", 0))
        and communication_drops == 0
    )
else:
    communication_ok = (
        sionna_ready
        and sionna_exact_samples > 0
        and int(communication_statistics.get("attempted_packets", 0)) > 0
    )
normal_completion = len(finished) == drone_count
fsm_participants = sorted(set(executed) | set(finished))
algorithm_pipeline_ok = (
    len(fsm_participants) == drone_count
    and len(executed) > 0
    and evidence["connectivity_graph_constructions"] > 0
    and evidence["hgrid_tours"] > 0
    and evidence["frontier_tours"] > 0
    and evidence["trajectory_plans"] > 0
)
min_distance = metrics.get("min_inter_drone")
min_clearance = metrics.get("min_obstacle_clearance")
starts = metrics.get("start_positions", [])
positions = metrics.get("positions", [])
acceptance = {
    "isaac_exit_ok": isaac_status == 0,
    "zero_collisions": metrics.get("collision_events") == 0,
    "minimum_inter_drone_distance_ok": (
        drone_count < 2 or (min_distance is not None and min_distance >= 1.0)
    ),
    "positive_obstacle_clearance": (
        min_clearance is not None and min_clearance > 0.0
    ),
    # Reaching FINISH without publishing a trajectory is a valid upstream FSM
    # outcome when an agent has no reachable task.  It still entered and ran
    # the C2 planning FSM, so count it together with trajectory executors.
    "all_agents_executed_c2_fsm": len(fsm_participants) == drone_count,
    "c2_algorithm_pipeline_observed": algorithm_pipeline_ok,
    "lkh_services_healthy": (
        evidence["lkh_call_failures"] == 0
        and evidence["process_crashes"] == 0
    ),
    "normal_completion": normal_completion,
    "communication_transport_ok": communication_ok,
}
passed = all(
    value for name, value in acceptance.items()
    if require_completion or name != "normal_completion"
)
result = {
    "algorithm": "c2_explorer_ros1_source_faithful_ros2_cpp",
    "scene": f"{scenario}.usd",
    "vehicle": "crazyflie_with_racer_dynamics.usd",
    "drone_count": drone_count,
    "require_completion": require_completion,
    "finished_drone_ids": finished,
    "executed_drone_ids": executed,
    "fsm_participating_drone_ids": fsm_participants,
    "algorithm_evidence": evidence,
    "communication": {
        "mode": communication_mode,
        "sionna_ready": sionna_ready,
        "sionna_scene": sionna_scene if communication_mode == "sionna" else None,
        "statistics": communication_statistics,
    },
    "acceptance": acceptance,
    "passed": passed,
    "metrics": metrics,
}
result_file.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if passed else 1)
PY
