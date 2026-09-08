#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${final_root}/results/perfect_vs_sionna_distributed_10uav_5sites_300s_${run_id}}"
perfect_suite="${suite}/perfect"
sionna_suite="${suite}/sionna_distributed"

mkdir -p "${suite}"
printf '%s\n' "running:perfect" >"${suite}/run_state.txt"
printf '%s\n' "$$" >"${suite}/supervisor.pid"

RACER_SUITE_DIR="${perfect_suite}" \
RACER_COVERAGE_UPDATE_RATE_HZ=10.0 \
RACER_EXPECTED_CPU_THREADS="${RACER_EXPECTED_CPU_THREADS:-18}" \
RACER_ALLOW_SHARED_GPU="${RACER_ALLOW_SHARED_GPU:-1}" \
  "${final_root}/scripts/run_perfect_75_2913.sh" --run

printf '%s\n' "cooldown:perfect_to_sionna" >"${suite}/run_state.txt"
sleep 30

printf '%s\n' "running:sionna_distributed" >"${suite}/run_state.txt"
RACER_SUITE_DIR="${sionna_suite}" \
RACER_RANDOM_SEED=42 \
RACER_DISCOVERY_DOMAIN_ID=84 \
RACER_BANDWIDTH_HZ=100000000.0 \
RACER_RESOURCE_BLOCKS=66 \
  "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh"

python3 - \
  "${perfect_suite}/formal_300s/warehouse_full_distributed_result.json" \
  "${sionna_suite}/formal_300s/warehouse_full_distributed_result.json" \
  "${suite}/comparison_summary.json" <<'PY'
import json
from pathlib import Path
import sys

perfect_path = Path(sys.argv[1])
sionna_path = Path(sys.argv[2])
output_path = Path(sys.argv[3])

def summarize(path):
    result = json.loads(path.read_text())
    metrics = result["metrics"]
    communication = result["communication"]
    statistics = communication.get("statistics", {})
    attempted = int(statistics.get("attempted_packets", 0))
    delivered = int(statistics.get("delivered_packets", 0))
    return {
        "result": str(path),
        "coverage": float(metrics["mapping_coverage_joint"]),
        "elapsed_s": float(metrics["elapsed"]),
        "collision_events": int(metrics["collision_events"]),
        "communication_mode": communication["mode"],
        "attempted_packets": attempted,
        "delivered_packets": delivered,
        "packet_success_rate": delivered / attempted if attempted else None,
        "task_quality_samples": len(statistics.get("task_quality_history", [])),
        "final_redundancy": statistics.get("redundant_exploration_ratio"),
        "final_bs_global_map_iou": statistics.get("bs_global_map_iou"),
    }

perfect = summarize(perfect_path)
sionna = summarize(sionna_path)
summary = {
    "experiment": "final_racer_perfect_vs_sionna_distributed_10uav_5sites_300s",
    "seed": 42,
    "perfect": perfect,
    "sionna_distributed": sionna,
    "coverage_difference_perfect_minus_sionna": (
        perfect["coverage"] - sionna["coverage"]
    ),
}
output_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
print(json.dumps(summary, indent=2, sort_keys=True))
PY

printf '%s\n' "completed" >"${suite}/run_state.txt"
