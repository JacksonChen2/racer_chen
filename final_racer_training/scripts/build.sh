#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
final_root="${RACER_FINAL_RACER_ROOT:-$(realpath "${training_root}/../final_racer")}" 
overlay="${training_root}/training_overlay_ws"

if [[ ! -f "${final_root}/ros2_ws/install/setup.bash" ]]; then
  printf 'Missing final_racer ROS2 install: %s\n' "${final_root}/ros2_ws/install/setup.bash" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${final_root}/ros2_ws/install/setup.bash"
set -u

cd "${overlay}"
colcon --log-base log build \
  --base-paths src \
  --build-base build \
  --install-base install \
  --packages-select racer_sionna_comm racer_isaac_adapter \
  --allow-overriding racer_sionna_comm racer_isaac_adapter \
  --symlink-install \
  --event-handlers console_direct+

printf 'BUILD_OK overlay=%s\n' "${overlay}/install/setup.bash"
