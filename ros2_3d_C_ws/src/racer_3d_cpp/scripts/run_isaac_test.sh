#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
search_dir="${script_dir}"
workspace_dir=""
while [[ "${search_dir}" != "/" ]]; do
  if [[ -f "${search_dir}/install/setup.bash" ]]; then
    workspace_dir="${search_dir}"
    break
  fi
  if [[ "$(basename "${search_dir}")" == "install" && -f "${search_dir}/setup.bash" ]]; then
    workspace_dir="$(dirname "${search_dir}")"
    break
  fi
  search_dir="$(dirname "${search_dir}")"
done
if [[ -z "${workspace_dir}" ]]; then
  printf 'Cannot locate this colcon workspace; build it first\n' >&2
  exit 2
fi
isaac_root="${ISAAC_SIM_ROOT:-${HOME}/isaacsim}"
scenario="${RACER_3D_SCENARIO:-warehouse_simple}"
duration="${RACER_3D_DURATION:-900}"
drone_count="${RACER_3D_DRONE_COUNT:-3}"
vehicle_model="${RACER_3D_VEHICLE_MODEL:-racer_so3}"
vehicle_usd="${RACER_SO3_VEHICLE_USD:-${workspace_dir}/../isaac_assets/racer_so3_quadrotor/usd/racer_so3_quadrotor_flattened.usd}"
camera_ray_budget="${RACER_3D_CAMERA_RAY_BUDGET:-2400}"
headless="${RACER_3D_HEADLESS:-1}"
visualize="${RACER_3D_VISUALIZE:-0}"
visualization_max_map_points="${RACER_3D_VISUALIZATION_MAX_MAP_POINTS:-40000}"
animate_propellers="${RACER_3D_ANIMATE_PROPELLERS:-auto}"
propeller_visual_hz="${RACER_3D_PROPELLER_VISUAL_HZ:-60}"
duration_seconds="$(python3 -c 'import math,sys; v=float(sys.argv[1]); assert v > 0; print(math.ceil(v))' "${duration}")"
timeout_scale="${RACER_3D_TIMEOUT_SCALE:-2}"
if [[ -z "${RACER_3D_TIMEOUT_SCALE:-}" && "${vehicle_model}" == "racer_so3" ]]; then
  # The upstream-equivalent plant runs at 1 kHz with three 640x480 depth
  # cameras at 30 Hz. A measured 0.079 real-time factor needs about 12.7x
  # wall time, so leave additional headroom for a full 900 s simulation.
  timeout_scale=16
fi
if ! [[ "${timeout_scale}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_3D_TIMEOUT_SCALE must be a positive integer\n' >&2
  exit 2
fi
if (( drone_count <= 0 )); then
  printf 'RACER_3D_DRONE_COUNT must be positive\n' >&2
  exit 2
fi
if (( camera_ray_budget <= 0 )); then
  printf 'RACER_3D_CAMERA_RAY_BUDGET must be positive\n' >&2
  exit 2
fi
if [[ "${headless}" != "0" && "${headless}" != "1" ]]; then
  printf 'RACER_3D_HEADLESS must be 0 or 1\n' >&2
  exit 2
fi
if [[ "${visualize}" != "0" && "${visualize}" != "1" ]]; then
  printf 'RACER_3D_VISUALIZE must be 0 or 1\n' >&2
  exit 2
fi
if (( visualization_max_map_points <= 0 )); then
  printf 'RACER_3D_VISUALIZATION_MAX_MAP_POINTS must be positive\n' >&2
  exit 2
fi
if [[ "${animate_propellers}" != "auto" && "${animate_propellers}" != "0" && "${animate_propellers}" != "1" ]]; then
  printf 'RACER_3D_ANIMATE_PROPELLERS must be auto, 0 or 1\n' >&2
  exit 2
fi
if ! python3 -c 'import sys; assert float(sys.argv[1]) > 0' "${propeller_visual_hz}"; then
  printf 'RACER_3D_PROPELLER_VISUAL_HZ must be positive\n' >&2
  exit 2
fi
if [[ "${visualize}" == "1" ]]; then
  headless=0
fi
if [[ "${vehicle_model}" != "racer_so3" && "${vehicle_model}" != "crazyflie" ]]; then
  printf 'RACER_3D_VEHICLE_MODEL must be racer_so3 or crazyflie\n' >&2
  exit 2
fi
if [[ "${vehicle_model}" == "racer_so3" && ! -f "${vehicle_usd}" ]]; then
  printf 'RACER SO3 vehicle USD does not exist: %s\n' "${vehicle_usd}" >&2
  exit 2
fi
scene_usd="${RACER_3D_SCENE_USD:-}"
if [[ -z "${scene_usd}" && "${scenario}" == "warehouse_simple" ]]; then
  scene_usd="${workspace_dir}/warehouse_simple.usd"
  if [[ ! -f "${scene_usd}" ]]; then
    scene_usd="${workspace_dir}/../ros2_3d_ws/warehouse_simple.usd"
  fi
fi
if [[ -z "${scene_usd}" && "${scenario}" == "warehouse_loaded" ]]; then
  scene_usd="${workspace_dir}/../ros2_3d_py_ws/warehouse_loaded.usd"
fi
result_file="${1:-${package_dir}/test_results/ISAAC_RESULT.json}"
if [[ "${result_file}" != /* ]]; then
  result_file="$(realpath -m -- "${result_file}")"
fi
if [[ -n "${scene_usd}" && ! -f "${scene_usd}" ]]; then
  printf 'External USD does not exist: %s\n' "${scene_usd}" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-153}"

agent_executable="${workspace_dir}/install/racer_3d_cpp/lib/racer_3d_cpp/racer_3d_cpp_agent"
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
setsid ros2 launch racer_3d_cpp swarm_3d_cpp.launch.py \
  backend:=isaac scenario:="${scenario}" drone_count:="${drone_count}" \
  vehicle_model:="${vehicle_model}" \
  duration:="${duration}" result_file:="${result_file}" &
launch_pid=$!

cleanup_launch() {
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
trap cleanup_launch EXIT
trap 'exit 130' INT TERM HUP

scene_args=()
if [[ -n "${scene_usd}" ]]; then scene_args=(--scene-usd "${scene_usd}"); fi
vehicle_args=(--vehicle-model "${vehicle_model}")
if [[ "${vehicle_model}" == "racer_so3" ]]; then
  vehicle_args+=(--vehicle-usd "${vehicle_usd}")
fi
headless_args=()
if [[ "${headless}" == "1" ]]; then headless_args=(--headless); fi
visualization_args=()
if [[ "${visualize}" == "1" ]]; then
  visualization_args=(
    --visualize-exploration
    --visualization-max-map-points "${visualization_max_map_points}"
  )
fi
propeller_args=(--propeller-visual-hz "${propeller_visual_hz}")
if [[ "${animate_propellers}" == "1" ]]; then
  propeller_args+=(--animate-propellers)
elif [[ "${animate_propellers}" == "0" ]]; then
  propeller_args+=(--no-animate-propellers)
fi
sleep 2
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "$(( duration_seconds * timeout_scale + 180 ))" \
  "${isaac_root}/python.sh" \
  "${package_dir}/isaac_sim/isaac_sim_racer_3d_cpp.py" \
  "${headless_args[@]}" "${visualization_args[@]}" \
  "${propeller_args[@]}" \
  --scenario "${scenario}" "${scene_args[@]}" \
  "${vehicle_args[@]}" \
  --duration "${duration}" --drone-count "${drone_count}" \
  --camera-ray-budget "${camera_ray_budget}" --diagnostics 2>&1 | \
  stdbuf -oL awk '
    /PxShape::getMaterialFromInternalFaceIndex received/ { next }
    /^RACER_3D_CONTACT / {
      ++contact_lines
      if (contact_lines <= 10 || contact_lines % 500 == 0) print
      next
    }
    { print }
  '

deadline=$((SECONDS + 20))
while [[ ! -s "${result_file}" && ${SECONDS} -lt ${deadline} ]]; do sleep 1; done
python3 - "${result_file}" <<'PY'
import json
from pathlib import Path
import sys
p = Path(sys.argv[1])
if not p.is_file():
    raise SystemExit(f"missing result file: {p}")
r = json.loads(p.read_text())
print(json.dumps(r, indent=2, sort_keys=True))
raise SystemExit(0 if r.get("passed") else 1)
PY
