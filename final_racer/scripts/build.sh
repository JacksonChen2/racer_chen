#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
base_workspace="${final_root}/ros2_ws"
overlay_workspace="${final_root}/passive_metrics_overlay_ws"
sionna_overlay_workspace="${final_root}/sionna_distributed_overlay_ws"
build_workers="${RACER_BUILD_WORKERS:-8}"

if [[ ! "${build_workers}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_BUILD_WORKERS must be a positive integer\n' >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble is unavailable: /opt/ros/humble/setup.bash\n' >&2
  exit 2
fi

if [[ ! -f "${base_workspace}/third_party/install/lib/libnlopt.so" ]]; then
  "${base_workspace}/build_nlopt.sh"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${base_workspace}"
colcon build \
  --symlink-install \
  --executor parallel \
  --parallel-workers "${build_workers}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

set +u
source "${base_workspace}/install/setup.bash"
set -u

cd "${overlay_workspace}"
colcon build \
  --symlink-install \
  --executor parallel \
  --parallel-workers "${build_workers}" \
  --packages-select racer_sionna_comm \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

set +u
source "${base_workspace}/install/setup.bash"
set -u

cd "${sionna_overlay_workspace}"
colcon build \
  --symlink-install \
  --executor parallel \
  --parallel-workers "${build_workers}" \
  --packages-select racer_sionna_comm \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF

printf 'FINAL_RACER_BUILD_OK base=%s passive_overlay=%s sionna_overlay=%s\n' \
  "${base_workspace}/install" "${overlay_workspace}/install" \
  "${sionna_overlay_workspace}/install"
