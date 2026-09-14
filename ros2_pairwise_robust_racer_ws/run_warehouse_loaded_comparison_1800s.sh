#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
pair_dir="${RACER_COMPARISON_DIR:-${workspace_dir}/experiments/warehouse_loaded_bs_vs_distributed_${run_id}}"
bs_dir="${pair_dir}/bs_round_robin"
distributed_dir="${pair_dir}/distributed"
plot_dir="${pair_dir}/comparison"

mkdir -p "${bs_dir}" "${distributed_dir}" "${plot_dir}"

RACER_RUN_ID="${run_id}" RACER_RESULT_DIR="${bs_dir}" \
  "${workspace_dir}/run_warehouse_loaded_bs_round_robin_1800s.sh"

RACER_RUN_ID="${run_id}" RACER_RESULT_DIR="${distributed_dir}" \
  "${workspace_dir}/run_warehouse_loaded_distributed_1800s.sh"

python3 "${workspace_dir}/scripts/plot_bs_vs_distributed_coverage.py" \
  --bs "${bs_dir}/warehouse_loaded_bs_round_robin_result.json" \
  --distributed "${distributed_dir}/warehouse_loaded_distributed_result.json" \
  --output-dir "${plot_dir}" \
  --deadline 1800 \
  --title "Warehouse Loaded 多无人机建图：联合覆盖率随运行时间变化"

printf 'Warehouse Loaded paired comparison: %s\n' "${pair_dir}"
