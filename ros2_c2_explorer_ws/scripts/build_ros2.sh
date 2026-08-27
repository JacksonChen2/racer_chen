#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
if [[ ! -f "${ros_setup}" ]]; then
  printf 'ROS 2 setup not found: %s\n' "${ros_setup}" >&2
  exit 2
fi

nlopt_source="${workspace_dir}/third_party/nlopt-2.7.1"
nlopt_build="${workspace_dir}/third_party/nlopt-build"
nlopt_install="${workspace_dir}/third_party/install"
if [[ ! -f "${nlopt_install}/lib/libnlopt.so" ]]; then
  cmake -S "${nlopt_source}" -B "${nlopt_build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${nlopt_install}" \
    -DNLOPT_PYTHON=OFF -DNLOPT_OCTAVE=OFF -DNLOPT_MATLAB=OFF \
    -DNLOPT_GUILE=OFF -DNLOPT_SWIG=OFF
  cmake --build "${nlopt_build}" --parallel "${C2_BUILD_JOBS:-8}"
  cmake --install "${nlopt_build}"
fi

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
set +u
source "${ros_setup}"
set -u
cd "${workspace_dir}"
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DC2_ENABLE_SANITIZERS=OFF
