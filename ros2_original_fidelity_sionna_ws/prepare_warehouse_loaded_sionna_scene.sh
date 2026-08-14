#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(realpath "${workspace_dir}/..")"
isaac_root="${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}"
scene_usd="${RACER_SCENE_USD:-${repo_root}/ros2_3d_py_ws/warehouse_loaded_with_industrial_ap.usda}"
output_dir="${RACER_SIONNA_SCENE_OUTPUT_DIR:-${workspace_dir}/src/racer_sionna_comm/assets/warehouse_loaded_sionna}"

if [[ ! -x "${isaac_root}/python.sh" || ! -f "${scene_usd}" ]]; then
  printf 'Missing Isaac Python or Warehouse Loaded BS USD.\n' >&2
  exit 2
fi

"${isaac_root}/python.sh" \
  "${workspace_dir}/src/racer_sionna_comm/scripts/convert_usd_to_sionna.py" \
  "${scene_usd}" \
  --output-dir "${output_dir}" \
  --geometry-mode oriented_bbox_proxy \
  --load-timeout 300

printf 'Sionna Warehouse Loaded BS scene: %s/warehouse.xml\n' "${output_dir}"
