#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_DIR="${SIONNA_RUNTIME_DIR:-${WORKSPACE_DIR}/.sionna_runtime}"

mkdir -p "${RUNTIME_DIR}"
python3 -m pip install --upgrade --target "${RUNTIME_DIR}" \
  -r "${WORKSPACE_DIR}/src/racer_3d_sionna_comm/requirements-sionna.txt"

PYTHONPATH="${RUNTIME_DIR}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
import rclpy
import sionna.rt
import mitsuba
import drjit
print("Sionna RT and ROS 2 rclpy imports succeeded")
PY

echo "export SIONNA_PYTHON=python3"
echo "export SIONNA_PYTHONPATH=${RUNTIME_DIR}"
