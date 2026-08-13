#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
repo_root="$(cd "${workspace_dir}/.." && pwd)"

run_dir="${1:-${workspace_dir}/validation/record_replay_900_or_finish_20260812_v1}"
trajectory="${2:-}"
if [[ -z "${trajectory}" ]]; then
  trajectory="$(find "${run_dir}" -maxdepth 1 -type f -name '*trajectory*.npz' -print | sort | tail -n 1)"
fi
launch_log="${run_dir}/warehouse_simple_launch.log"
result_json="${run_dir}/warehouse_simple_result.json"
vehicle_urdf="${repo_root}/isaac_assets/racer_so3_quadrotor/urdf/racer_so3_quadrotor_crazyflie_style.urdf"
rviz_config="${package_dir}/config/warehouse_simple_recorded_mapping.rviz"
replay_script="${script_dir}/replay_recorded_mapping_rviz.py"

for required in "${trajectory}" "${launch_log}" "${result_json}" "${vehicle_urdf}" "${rviz_config}"; do
  if [[ ! -f "${required}" ]]; then
    printf 'Required replay input does not exist: %s\n' "${required}" >&2
    exit 2
  fi
done

set +u
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-211}"

python3 "${replay_script}" \
  --trajectory "${trajectory}" \
  --launch-log "${launch_log}" \
  --result-json "${result_json}" \
  --vehicle-urdf "${vehicle_urdf}" \
  --speed 1.0 --fps 30 --loop &
replay_pid=$!

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if kill -0 "${replay_pid}" 2>/dev/null; then
    kill -INT "${replay_pid}" 2>/dev/null || true
  fi
  wait "${replay_pid}" 2>/dev/null || true
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

printf 'Opening RViz 1x loop: %s\n' "${trajectory}"
printf 'Coverage values are exact log diagnostics; voxel geometry was not recorded.\n'
rviz2 -d "${rviz_config}"
