#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACER_DIR="$(cd "${WORKSPACE_DIR}/.." && pwd)"
ISAAC_DIR="${ISAAC_SIM_DIR:-${ISAAC_SIM_ROOT:-${HOME}/isaacsim}}"
WAREHOUSE_BASE_USD="${WAREHOUSE_BASE_USD:-${RACER_DIR}/ros2_3d_py_ws/warehouse.usd}"
OUTPUT_DIR="${WORKSPACE_DIR}/src/racer_3d_sionna_comm/assets/warehouse_loaded_sionna"

if [[ ! -f "${RACER_DIR}/warehouse_loaded.usd" ]]; then
  echo "Missing ${RACER_DIR}/warehouse_loaded.usd" >&2
  exit 2
fi
if [[ ! -f "${WAREHOUSE_BASE_USD}" ]]; then
  echo "Missing base warehouse layer: ${WAREHOUSE_BASE_USD}" >&2
  echo "Set WAREHOUSE_BASE_USD to the warehouse.usd used by the overlay." >&2
  exit 2
fi

"${ISAAC_DIR}/python.sh" \
  "${WORKSPACE_DIR}/src/racer_3d_sionna_comm/scripts/convert_usd_to_sionna.py" \
  "${RACER_DIR}/warehouse_loaded.usd" \
  --base-usd "${WAREHOUSE_BASE_USD}" \
  --geometry-mode oriented_bbox_proxy \
  --output-dir "${OUTPUT_DIR}"
