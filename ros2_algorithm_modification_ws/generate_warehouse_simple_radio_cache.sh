#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
runtime_dir="${SIONNA_RUNTIME_DIR:-${workspace_dir}/.sionna_runtime}"
asset_dir="${workspace_dir}/src/racer_sionna_comm/assets/warehouse_simple_sionna"

if [[ ! -d "${runtime_dir}/sionna" || ! -f "${asset_dir}/warehouse.xml" ]]; then
  printf 'Install Sionna and prepare the Warehouse scene first.\n' >&2
  exit 2
fi

PYTHONPATH="${runtime_dir}${PYTHONPATH:+:${PYTHONPATH}}" python3 \
  "${workspace_dir}/src/racer_sionna_comm/scripts/generate_hybrid_radio_cache.py" \
  --scene-xml "${asset_dir}/warehouse.xml" \
  --output "${asset_dir}/hybrid_radio_cache.npz" \
  --x-min -10.0 --x-max 9.0 --y-min -11.9 --y-max 17.6 \
  --tx-spacing 5.0 --rx-cell 1.0 \
  --tx-heights 1.0 3.0 6.0 --rx-heights 0.75 1.5 3.0 5.0 7.5 \
  --samples-per-tx 100000 --max-depth 2 --seed 42
