#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(realpath "${workspace_dir}/..")"
runtime_dir="${C2_SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"
requirements="${repo_root}/ros2_pairwise_robust_racer_ws/src/racer_sionna_comm/requirements-sionna.txt"

if [[ ! -f "${requirements}" ]]; then
  printf 'Missing Sionna requirements: %s\n' "${requirements}" >&2
  exit 2
fi

mkdir -p "${runtime_dir}"
python3 -m pip install --upgrade --target "${runtime_dir}" -r "${requirements}"

PYTHONPATH="${runtime_dir}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
import drjit
import mitsuba
import rclpy
import sionna.rt
print("Sionna RT, Mitsuba, Dr.Jit and ROS 2 rclpy imports succeeded")
PY

printf 'C2-Explorer Sionna runtime: %s\n' "${runtime_dir}"
