#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"

export RACER_3D_SCENARIO="${RACER_3D_SCENARIO:-warehouse_loaded}"
export RACER_3D_DRONE_COUNT="${RACER_3D_DRONE_COUNT:-3}"
export RACER_3D_DURATION="${RACER_3D_DURATION:-900}"
export RACER_3D_VEHICLE_MODEL="${RACER_3D_VEHICLE_MODEL:-racer_so3}"
export RACER_3D_CAMERA_RAY_BUDGET="${RACER_3D_CAMERA_RAY_BUDGET:-2400}"
export RACER_3D_VISUALIZATION_MAX_MAP_POINTS="${RACER_3D_VISUALIZATION_MAX_MAP_POINTS:-40000}"
export RACER_SO3_VEHICLE_USD="${RACER_SO3_VEHICLE_USD:-$(realpath -m "${package_dir}/../../../isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd")}"
export RACER_3D_ANIMATE_PROPELLERS="${RACER_3D_ANIMATE_PROPELLERS:-1}"
export RACER_3D_PROPELLER_VISUAL_HZ="${RACER_3D_PROPELLER_VISUAL_HZ:-60}"
export RACER_3D_HEADLESS=0
export RACER_3D_VISUALIZE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-172}"

result_file="${1:-${package_dir}/test_results/ISAAC_WAREHOUSE_LOADED_SO3_CPP_VISUALIZATION_RESULT.json}"

printf 'Starting RACER Isaac visualization: scenario=%s drones=%s duration=%ss ROS_DOMAIN_ID=%s vehicle=%s propeller_visual_hz=%s\n' \
  "${RACER_3D_SCENARIO}" "${RACER_3D_DRONE_COUNT}" \
  "${RACER_3D_DURATION}" "${ROS_DOMAIN_ID}" \
  "${RACER_SO3_VEHICLE_USD}" "${RACER_3D_PROPELLER_VISUAL_HZ}"
printf 'Legend: occupied map=cyan; drone trails/plans=red, green, yellow\n'

exec "${script_dir}/run_isaac_test.sh" "${result_file}"
