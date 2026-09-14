#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runtime_dir="${SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"

mkdir -p "${runtime_dir}"
python3 -m pip install --upgrade --target "${runtime_dir}" \
  -r "${workspace_dir}/src/racer_sionna_comm/requirements-sionna.txt"

PYTHONPATH="${runtime_dir}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
import drjit
import mitsuba
import rclpy
import sionna.rt
print("Sionna RT, Mitsuba, Dr.Jit and ROS 2 rclpy imports succeeded")
PY

printf 'Sionna runtime: %s\n' "${runtime_dir}"
