#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
root_dir="$(cd "${workspace_dir}/.." && pwd)"

isaac_root="${ISAAC_SIM_ROOT:-${root_dir}/../software/isaacsim}"
if [[ ! -x "${isaac_root}/python.sh" ]]; then
  if [[ -x "${HOME}/software/isaacsim/python.sh" ]]; then
    isaac_root="${HOME}/software/isaacsim"
  else
    printf 'Cannot find Isaac Sim; set ISAAC_SIM_ROOT\n' >&2
    exit 2
  fi
fi

duration="${RACER_3D_DURATION:-300}"
drone_count="${RACER_3D_DRONE_COUNT:-3}"
viewer_fps="${RACER_3D_VIEWER_FPS:-30}"
domain_id="${ROS_DOMAIN_ID:-172}"
scene_usd="${RACER_3D_SCENE_USD:-${root_dir}/ros2_3d_py_ws/warehouse_simple.usd}"
vehicle_usd="${RACER_SO3_VEHICLE_USD:-${root_dir}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd}"
viewer_script="${package_dir}/isaac_sim/isaac_sim_racer_viewer.py"
mkdir -p "${root_dir}/results"
result_file="${root_dir}/results/warehouse_simple_decoupled_result.json"
sim_log="${root_dir}/results/warehouse_simple_decoupled_sim.log"

if [[ ! -f "${scene_usd}" ]]; then
  printf 'Warehouse Simple USD does not exist: %s\n' "${scene_usd}" >&2
  exit 2
fi
if [[ ! -f "${vehicle_usd}" ]]; then
  printf 'Vehicle USD does not exist: %s\n' "${vehicle_usd}" >&2
  exit 2
fi

printf 'Starting headless RACER pipeline with model: %s\n' "${vehicle_usd}"
setsid env \
  ISAAC_SIM_ROOT="${isaac_root}" \
  RACER_3D_SCENARIO=warehouse_simple \
  RACER_3D_SCENE_USD="${scene_usd}" \
  RACER_SO3_VEHICLE_USD="${vehicle_usd}" \
  RACER_3D_DURATION="${duration}" \
  RACER_3D_DRONE_COUNT="${drone_count}" \
  RACER_3D_HEADLESS=1 \
  RACER_3D_VISUALIZE=0 \
  RACER_3D_ANIMATE_PROPELLERS=0 \
  ROS_DOMAIN_ID="${domain_id}" \
  "${script_dir}/run_isaac_test.sh" "${result_file}" \
  >"${sim_log}" 2>&1 &
sim_pid=$!

cleanup() {
  local status=$?
  local child pgid
  trap - EXIT INT TERM HUP
  if kill -0 "${sim_pid}" 2>/dev/null; then
    # run_isaac_test deliberately gives ros2 launch and the Isaac timeout
    # pipeline their own process groups. Signal every direct-child group so
    # closing the display-only viewer also stops physics and all ROS agents.
    while read -r child; do
      pgid="$(ps -o pgid= -p "${child}" 2>/dev/null | tr -d ' ')"
      if [[ -n "${pgid}" ]]; then
        kill -INT -- "-${pgid}" 2>/dev/null || true
      fi
    done < <(pgrep -P "${sim_pid}" || true)
    kill -INT "${sim_pid}" 2>/dev/null || true
  fi
  for _ in {1..50}; do
    if ! kill -0 "${sim_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  if kill -0 "${sim_pid}" 2>/dev/null; then
    while read -r child; do
      pgid="$(ps -o pgid= -p "${child}" 2>/dev/null | tr -d ' ')"
      if [[ -n "${pgid}" ]]; then
        kill -TERM -- "-${pgid}" 2>/dev/null || true
      fi
    done < <(pgrep -P "${sim_pid}" || true)
    kill -TERM "${sim_pid}" 2>/dev/null || true
  fi
  wait "${sim_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

# Let the ROS agents claim their names before the viewer joins the domain.
sleep 3
if ! kill -0 "${sim_pid}" 2>/dev/null; then
  printf 'Headless simulator exited during startup; see %s\n' "${sim_log}" >&2
  tail -80 "${sim_log}" >&2 || true
  exit 1
fi

printf 'Opening independent %s FPS Isaac viewer; close its window to stop.\n' \
  "${viewer_fps}"
set +e
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  ROS_DOMAIN_ID="${domain_id}" ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  "${isaac_root}/python.sh" "${viewer_script}" \
  --scene-usd "${scene_usd}" \
  --vehicle-usd "${vehicle_usd}" \
  --drone-count "${drone_count}" \
  --fps "${viewer_fps}" \
  --interpolation-delay 0.25 \
  --max-map-points 8000
viewer_status=$?
set -e
exit "${viewer_status}"
