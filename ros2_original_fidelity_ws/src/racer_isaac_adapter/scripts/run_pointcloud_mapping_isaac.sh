#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
repo_root="$(cd "${workspace_dir}/.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/jiazheng/software/isaacsim}"
run_dir="${1:-${workspace_dir}/validation/record_replay_900_or_finish_20260812_v1}"
trajectory="${2:-${run_dir}/warehouse_simple_original_racer_trajectory_20260812_151514.npz}"
pointcloud_map="${3:-${run_dir}/warehouse_simple_pointcloud_reconstruction.npz}"
scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_simple.usd"
vehicle_usd="${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
base_station_usd="${repo_root}/isaac_assets/industrial_wifi_ap/industrial_wifi_ap.usda"
replay_script="${package_dir}/isaac_sim/original_racer_trajectory_replay.py"
log_file="${run_dir}/warehouse_simple_pointcloud_isaac_replay.log"

for required in "${trajectory}" "${pointcloud_map}" "${scene_usd}" \
  "${vehicle_usd}" "${base_station_usd}" "${replay_script}" \
  "${isaac_root}/python.sh"; do
  if [[ ! -f "${required}" ]]; then
    printf 'Required input does not exist: %s\n' "${required}" >&2
    exit 2
  fi
done

replay_args=(
  --trajectory "${trajectory}"
  --pointcloud-map "${pointcloud_map}"
  --scene-usd "${scene_usd}"
  --vehicle-usd "${vehicle_usd}"
  --base-station-usd "${base_station_usd}"
  --fps "${RACER_POINTCLOUD_ISAAC_FPS:-60}"
  --speed "${RACER_POINTCLOUD_ISAAC_SPEED:-1.0}"
  --point-size "${RACER_POINTCLOUD_ISAAC_POINT_SIZE:-3.0}"
  --follow-drone "${RACER_POINTCLOUD_ISAAC_FOLLOW_DRONE:--1}"
  --camera-mode "${RACER_POINTCLOUD_ISAAC_CAMERA_MODE:-chase}"
  --initial-view "${RACER_POINTCLOUD_ISAAC_INITIAL_VIEW:-ap}"
  --propeller-visual-hz "${RACER_POINTCLOUD_ISAAC_PROPELLER_HZ:-60}"
)
if [[ "${RACER_POINTCLOUD_ISAAC_LOOP:-1}" == "1" ]]; then
  replay_args+=(--loop)
fi

printf 'Opening Isaac Sim 3-D mapping replay at %sx.\n' \
  "${RACER_POINTCLOUD_ISAAC_SPEED:-1.0}"
printf 'Map source: %s\n' "${pointcloud_map}"
printf 'Industrial Wi-Fi AP: ceiling center at (-0.50, 2.85, 8.55) m.\n'
printf 'Use Overview or D0-D4 Chase/Onboard controls in the Isaac window.\n'

env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  "${isaac_root}/python.sh" "${replay_script}" "${replay_args[@]}" \
  2>&1 | tee "${log_file}"
