#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
repo_root="$(cd "${workspace_dir}/.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/jiazheng/software/isaacsim}"
run_dir="${1:-${workspace_dir}/validation/record_replay_900_or_finish_20260812_v1}"
trajectory="${2:-${run_dir}/warehouse_simple_original_racer_trajectory_20260812_151514.npz}"
output="${run_dir}/warehouse_simple_pointcloud_reconstruction.npz"
log_file="${run_dir}/warehouse_simple_pointcloud_reconstruction.log"
scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_simple.usd"
vehicle_usd="${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
reconstruction_script="${package_dir}/isaac_sim/original_racer_pointcloud_reconstruction.py"
rviz_config="${package_dir}/config/warehouse_simple_pointcloud_reconstruction.rviz"

for required in "${trajectory}" "${scene_usd}" "${vehicle_usd}" \
  "${reconstruction_script}" "${rviz_config}" "${isaac_root}/python.sh"; do
  if [[ ! -e "${required}" ]]; then
    printf 'Required input does not exist: %s\n' "${required}" >&2
    exit 2
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-213}"

rviz2 -d "${rviz_config}" &
rviz_pid=$!
reconstruction_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${reconstruction_pid}" ]] && kill -0 "${reconstruction_pid}" 2>/dev/null; then
    kill -INT "${reconstruction_pid}" 2>/dev/null || true
  fi
  kill -TERM "${rviz_pid}" 2>/dev/null || true
  if [[ -n "${reconstruction_pid}" ]]; then
    wait "${reconstruction_pid}" 2>/dev/null || true
  fi
  wait "${rviz_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

sleep 3
printf 'Regenerating Warehouse Simple depth point clouds from the recorded flight.\n'
printf 'The growing 0.1 m surface-voxel map will be saved to %s\n' "${output}"
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  "${isaac_root}/python.sh" "${reconstruction_script}" \
  --trajectory "${trajectory}" \
  --scene-usd "${scene_usd}" \
  --vehicle-usd "${vehicle_usd}" \
  --output "${output}" \
  --speed "${RACER_POINTCLOUD_REPLAY_SPEED:-1.0}" \
  --sensor-rate-hz "${RACER_POINTCLOUD_SENSOR_HZ:-30}" \
  --publish-rate-hz "${RACER_POINTCLOUD_PUBLISH_HZ:-5}" \
  --depth-width "${RACER_POINTCLOUD_DEPTH_WIDTH:-320}" \
  --depth-height "${RACER_POINTCLOUD_DEPTH_HEIGHT:-240}" \
  --ray-budget "${RACER_POINTCLOUD_RAY_BUDGET:-16000}" \
  --voxel-size 0.1 \
  2>&1 | tee "${log_file}" &
reconstruction_pid=$!

wait "${rviz_pid}"
