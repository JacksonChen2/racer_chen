#!/usr/bin/env bash
set -eo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"

source /opt/ros/humble/setup.bash
set -u
cd "${workspace}"
colcon build \
  --packages-select racer_sionna_comm \
  --symlink-install \
  --executor sequential \
  --cmake-args -DBUILD_TESTING=ON
set +u
source "${workspace}/install/setup.bash"
set -u
colcon test \
  --packages-select racer_sionna_comm \
  --event-handlers console_direct+
colcon test-result --test-result-base "${workspace}/build/racer_sionna_comm" \
  --verbose
