#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"
scenario="${RACER_3D_PY_SCENARIO:-acceptance_15x9x2}"
scene_usd="${RACER_3D_PY_SCENE_USD:-}"
if [[ -z "${scene_usd}" && "${scenario}" == "warehouse_simple" ]]; then
  scene_usd="${workspace_dir}/warehouse_simple.usd"
fi
if [[ -z "${scene_usd}" && "${scenario}" == "warehouse_loaded" ]]; then
  scene_usd="${workspace_dir}/warehouse_loaded.usd"
fi
scene_arguments=()
if [[ -n "${scene_usd}" ]]; then
  if [[ ! -f "${scene_usd}" ]]; then
    printf 'External USD does not exist: %s\n' "${scene_usd}" >&2
    exit 2
  fi
  scene_arguments=(--scene-usd "${scene_usd}")
fi
result_name="ISAAC_${scenario#acceptance_}_RESULT.json"
result_file="${1:-${package_dir}/test_results/${result_name^^}}"
duration="${RACER_3D_PY_DURATION:-120}"
duration_seconds="$(python3 -c \
  'import math,sys; v=float(sys.argv[1]); assert v > 0; print(math.ceil(v))' \
  "${duration}")"
timeout_seconds="$((duration_seconds * 2 + 180))"
drone_count="${RACER_3D_PY_DRONE_COUNT:-3}"
headless="${RACER_3D_PY_HEADLESS:-1}"
visualize="${RACER_3D_PY_VISUALIZE:-0}"
visualization_max_map_points="${RACER_3D_PY_VISUALIZATION_MAX_MAP_POINTS:-40000}"
if (( drone_count <= 0 )); then
  printf 'RACER_3D_PY_DRONE_COUNT must be positive\n' >&2
  exit 2
fi
if [[ "${headless}" != "0" && "${headless}" != "1" ]]; then
  printf 'RACER_3D_PY_HEADLESS must be 0 or 1\n' >&2
  exit 2
fi
if [[ "${visualize}" != "0" && "${visualize}" != "1" ]]; then
  printf 'RACER_3D_PY_VISUALIZE must be 0 or 1\n' >&2
  exit 2
fi
if (( visualization_max_map_points <= 0 )); then
  printf 'RACER_3D_PY_VISUALIZATION_MAX_MAP_POINTS must be positive\n' >&2
  exit 2
fi
if [[ "${visualize}" == "1" ]]; then
  headless=0
fi
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-63}"
export ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  printf 'Build the workspace first: colcon build --symlink-install\n' >&2
  exit 2
fi
source "${workspace_dir}/install/setup.bash"
set -u
agent_executable="${workspace_dir}/install/racer_3d_py/lib/racer_3d_py/racer_3d_py_agent"
mapfile -t existing_agent_pids < <(
  pgrep -f "^${agent_executable}( |$)" || true
)
if (( ${#existing_agent_pids[@]} > 0 )); then
  printf 'Refusing to start: stale RACER agents are still running: %s\n' \
    "${existing_agent_pids[*]}" >&2
  printf 'Stop the previous run with Ctrl+C before starting another one.\n' >&2
  exit 3
fi

rm -f -- "${result_file}"

setsid ros2 launch racer_3d_py swarm_3d_py.launch.py \
  backend:=isaac \
  scenario:="${scenario}" \
  drone_count:="${drone_count}" \
  duration:="${duration}" \
  result_file:="${result_file}" &
launch_pid=$!
cleanup() {
  exit_status=$?
  trap - EXIT INT TERM HUP
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..50}; do
      if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
      sleep 1
    fi
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    fi
  fi
  wait "${launch_pid}" 2>/dev/null || true
  exit "${exit_status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

sleep 2
headless_arguments=()
if [[ "${headless}" == "1" ]]; then
  headless_arguments=(--headless)
fi
visualization_arguments=()
if [[ "${visualize}" == "1" ]]; then
  visualization_arguments=(
    --visualize-exploration
    --visualization-max-map-points "${visualization_max_map_points}"
  )
fi
env \
  -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "${timeout_seconds}" \
  "${isaac_root}/python.sh" \
  "${package_dir}/isaac_sim/isaac_sim_racer_3d_py.py" \
  "${headless_arguments[@]}" \
  "${visualization_arguments[@]}" \
  --scenario "${scenario}" \
  "${scene_arguments[@]}" \
  --duration "${duration}" \
  --drone-count "${drone_count}" \
  --diagnostics

deadline=$((SECONDS + 20))
while [[ ! -s "${result_file}" && ${SECONDS} -lt ${deadline} ]]; do
  sleep 1
done

python3 - "${result_file}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
if not path.exists():
    raise SystemExit(f"missing result file: {path}")
result = json.loads(path.read_text(encoding="utf-8"))
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result.get("passed") else 1)
PY
