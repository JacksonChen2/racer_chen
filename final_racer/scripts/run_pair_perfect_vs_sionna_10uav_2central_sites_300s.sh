#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
layout="${final_root}/config/warehouse_full_10uav_two_center_sites_layout.json"
suite="${RACER_SUITE_DIR:-${final_root}/results/perfect_vs_sionna_distributed_10uav_2central_sites_300s_${run_id}}"
perfect_suite="${suite}/perfect"
sionna_suite="${suite}/sionna_distributed"
starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${layout}")"

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

python3 - "${suite}/experiment_manifest.json" "${layout}" "${run_id}" <<'PY'
import json
from pathlib import Path
import sys

layout_path = Path(sys.argv[2])
layout = json.loads(layout_path.read_text())
manifest = {
    "experiment": "final_racer_perfect_vs_sionna_distributed_10uav_2central_sites_300s",
    "run_id": sys.argv[3],
    "layout_file": str(layout_path),
    "layout": layout["layout"],
    "regions": layout["regions"],
    "start_positions": layout["start_positions"],
    "common": {
        "drone_count": 10,
        "duration_s": 300,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "random_seed": 42,
        "network_topology": "distributed",
    },
    "cases": {
        "perfect": {"communication_mode": "ideal"},
        "sionna_distributed": {
            "communication_mode": "sionna",
            "uav_tx_power_dbm": 23,
            "fixed_mcs_index": 14,
            "bandwidth_hz": 100000000.0,
            "resource_blocks": 66,
            "max_retries": 0,
            "initial_assignment_perfect_delivery": False,
            "bs_enabled": False,
        },
    },
}
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

printf '%s\n' "running:perfect" >"${suite}/run_state.txt"
set +e
RACER_RUN_ID="${run_id}" \
RACER_SUITE_DIR="${perfect_suite}" \
RACER_START_POSITIONS="${starts}" \
RACER_ALGORITHM_LABEL="final_racer_10uav_2central_sites_ideal_no_loss_100hz_76800rays_300s" \
RACER_COVERAGE_UPDATE_RATE_HZ=10.0 \
RACER_PREFLIGHT_DOMAIN_ID="${RACER_PERFECT_PREFLIGHT_DOMAIN_ID:-87}" \
RACER_FORMAL_DOMAIN_ID="${RACER_PERFECT_FORMAL_DOMAIN_ID:-88}" \
RACER_EXPECTED_CPU_THREADS="${RACER_EXPECTED_CPU_THREADS:-18}" \
RACER_ALLOW_SHARED_GPU="${RACER_ALLOW_SHARED_GPU:-1}" \
  "${final_root}/scripts/run_perfect_75_2913.sh" --run
perfect_status=$?
set -e
printf '%s\n' "${perfect_status}" >"${perfect_suite}/case_status.txt"

printf '%s\n' "cooldown:perfect_to_sionna" >"${suite}/run_state.txt"
sleep 30

printf '%s\n' "running:sionna_distributed" >"${suite}/run_state.txt"
set +e
RACER_RUN_ID="${run_id}" \
RACER_SUITE_DIR="${sionna_suite}" \
RACER_START_LAYOUT_FILE="${layout}" \
RACER_ALGORITHM_LABEL="final_racer_10uav_2central_sites_no_bs_sionna_23dbm_mcs14_100mhz_66rb_300s" \
RACER_RANDOM_SEED=42 \
RACER_DISCOVERY_DOMAIN_ID="${RACER_SIONNA_DOMAIN_ID:-89}" \
RACER_BANDWIDTH_HZ=100000000.0 \
RACER_RESOURCE_BLOCKS=66 \
RACER_UAV_TX_POWER_DBM=23 \
RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
  "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh"
sionna_status=$?
set -e
printf '%s\n' "${sionna_status}" >"${sionna_suite}/case_status.txt"

python3 - \
  "${perfect_suite}/formal_300s/warehouse_full_distributed_result.json" \
  "${sionna_suite}/formal_300s/warehouse_full_distributed_result.json" \
  "${suite}/comparison_summary.json" <<'PY'
import json
from pathlib import Path
import sys

def summarize(path_value):
    path = Path(path_value)
    if not path.is_file():
        return {"result": str(path), "available": False}
    result = json.loads(path.read_text())
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    statistics = communication.get("statistics", {})
    attempted = int(statistics.get("attempted_packets", 0))
    delivered = int(statistics.get("delivered_packets", 0))
    return {
        "result": str(path),
        "available": True,
        "coverage": metrics.get("mapping_coverage_joint"),
        "elapsed_s": metrics.get("elapsed"),
        "collision_events": metrics.get("collision_events"),
        "executed_drone_ids": result.get("executed_drone_ids", []),
        "communication_mode": communication.get("mode"),
        "attempted_packets": attempted,
        "delivered_packets": delivered,
        "packet_success_rate": delivered / attempted if attempted else None,
        "sionna_exact_samples": communication.get("exact_link_samples", 0),
    }

perfect = summarize(sys.argv[1])
sionna = summarize(sys.argv[2])
summary = {"perfect": perfect, "sionna_distributed": sionna}
if perfect.get("available") and sionna.get("available"):
    summary["coverage_difference_perfect_minus_sionna"] = (
        float(perfect["coverage"]) - float(sionna["coverage"])
    )
Path(sys.argv[3]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
PY

if [[ "${perfect_status}" -eq 0 && "${sionna_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error perfect=%s sionna=%s\n' \
    "${perfect_status}" "${sionna_status}" >"${suite}/run_state.txt"
fi
printf 'SUITE_FINISH time=%s perfect=%s sionna=%s\n' \
  "$(date --iso-8601=seconds)" "${perfect_status}" "${sionna_status}"
