#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
duration="${RACER_EXPERIMENT_DURATION:-900}"
source_layout="${RACER_START_LAYOUT_FILE:-${final_root}/config/warehouse_full_10uav_five_sites_layout.json}"
suite="${RACER_SUITE_DIR:-${final_root}/results/perfect_vs_sionna_distributed_10uav_new_starts_${duration}s_${run_id}}"
perfect_suite="${suite}/perfect"
sionna_suite="${suite}/sionna_distributed_original"
layout_snapshot="${suite}/start_layout_snapshot.json"
active_child=""
mode="${1:---run}"

if [[ "${mode}" != "--check" && "${mode}" != "--run" ]]; then
  printf 'Usage: %s [--check|--run]\n' "$0" >&2
  exit 2
fi
if ! [[ "${duration}" =~ ^[1-9][0-9]*$ ]] || [[ "${duration}" -ne 900 ]]; then
  printf 'This experiment requires RACER_EXPERIMENT_DURATION=900.\n' >&2
  exit 2
fi
if [[ ! -f "${source_layout}" ]]; then
  printf 'Missing start layout: %s\n' "${source_layout}" >&2
  exit 2
fi

mkdir -p "${suite}"
cp "${source_layout}" "${layout_snapshot}"
starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${layout_snapshot}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf 'Expected exactly 10 XYZ start positions in %s\n' "${layout_snapshot}" >&2
  exit 2
fi

stop_suite() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM -- "-${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' stopped >"${suite}/run_state.txt"
  exit 130
}
trap stop_suite INT TERM HUP

python3 - "${suite}/experiment_manifest.json" "${layout_snapshot}" \
  "${source_layout}" "${duration}" "${run_id}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys

output = Path(sys.argv[1])
snapshot = Path(sys.argv[2])
source = Path(sys.argv[3])
duration = int(sys.argv[4])
layout = json.loads(snapshot.read_text())
manifest = {
    "experiment": "final_racer_perfect_vs_sionna_distributed_original_new_starts_900s",
    "run_id": sys.argv[5],
    "layout_source": str(source),
    "layout_snapshot": str(snapshot),
    "layout_sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest(),
    "layout": layout["layout"],
    "start_positions": layout["start_positions"],
    "common": {
        "algorithm": "final_racer",
        "exploration_assignment_mode": "original",
        "initial_assignment_coordinator_uav_id": 1,
        "initial_assignment_perfect_delivery": False,
        "duration_s": duration,
        "drone_count": 10,
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "coverage_update_rate_hz": 0.5,
        "random_seed": 42,
    },
    "cases": {
        "perfect": {
            "communication_mode": "ideal",
            "network_topology": "distributed",
        },
        "sionna_distributed_original": {
            "communication_mode": "sionna",
            "network_topology": "distributed",
            "bs_enabled": False,
            "uav_tx_power_dbm": 23,
            "fixed_mcs_index": 14,
            "bandwidth_hz": 100000000.0,
            "resource_blocks": 66,
            "max_retries": 0,
        },
    },
}
output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

if [[ "${mode}" == "--check" ]]; then
  starts="${starts}" final_root="${final_root}" suite="${suite}" \
    bash -c '
      RACER_SUITE_DIR="${suite}/check/perfect" \
      RACER_EXPERIMENT_DURATION=900 \
      RACER_START_POSITIONS="${starts}" \
      RACER_EXPLORATION_ASSIGNMENT_MODE=original \
      RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
      RACER_EXPECTED_CPU_THREADS="${RACER_EXPECTED_CPU_THREADS:-18}" \
      RACER_ALLOW_SHARED_GPU=1 \
        "${final_root}/scripts/run_perfect_75_2913.sh" --check
      RACER_SUITE_DIR="${suite}/check/sionna_distributed_original" \
      RACER_EXPERIMENT_DURATION=900 \
      RACER_START_LAYOUT_FILE="${suite}/start_layout_snapshot.json" \
      RACER_EXPLORATION_ASSIGNMENT_MODE=original \
      RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
      RACER_COVERAGE_UPDATE_RATE_HZ=0.5 \
      RACER_BANDWIDTH_HZ=100000000.0 \
      RACER_RESOURCE_BLOCKS=66 \
      RACER_UAV_TX_POWER_DBM=23 \
        "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh" --check
    '
  printf '%s\n' checked >"${suite}/run_state.txt"
  exit 0
fi

printf '%s\n' running:perfect >"${suite}/run_state.txt"
set +e
setsid env \
  RACER_RUN_ID="${run_id}" \
  RACER_SUITE_DIR="${perfect_suite}" \
  RACER_EXPERIMENT_DURATION="${duration}" \
  RACER_START_POSITIONS="${starts}" \
  RACER_EXPLORATION_ASSIGNMENT_MODE=original \
  RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
  RACER_ALGORITHM_LABEL="final_racer_original_10uav_new_starts_ideal_${duration}s" \
  RACER_COVERAGE_UPDATE_RATE_HZ=0.5 \
  RACER_PREFLIGHT_DOMAIN_ID="${RACER_PERFECT_PREFLIGHT_DOMAIN_ID:-221}" \
  RACER_FORMAL_DOMAIN_ID="${RACER_PERFECT_FORMAL_DOMAIN_ID:-222}" \
  RACER_EXPECTED_CPU_THREADS="${RACER_EXPECTED_CPU_THREADS:-18}" \
  RACER_ALLOW_SHARED_GPU="${RACER_ALLOW_SHARED_GPU:-1}" \
  "${final_root}/scripts/run_perfect_75_2913.sh" --run \
  >"${suite}/perfect.log" 2>&1 &
active_child=$!
wait "${active_child}"
perfect_status=$?
active_child=""
set -e
printf '%s\n' "${perfect_status}" >"${perfect_suite}/case_status.txt"

printf '%s\n' cooldown:perfect_to_sionna >"${suite}/run_state.txt"
sleep 30

printf '%s\n' running:sionna_distributed_original >"${suite}/run_state.txt"
set +e
setsid env \
  RACER_RUN_ID="${run_id}" \
  RACER_SUITE_DIR="${sionna_suite}" \
  RACER_EXPERIMENT_DURATION="${duration}" \
  RACER_START_LAYOUT_FILE="${layout_snapshot}" \
  RACER_EXPLORATION_ASSIGNMENT_MODE=original \
  RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false \
  RACER_ALGORITHM_LABEL="final_racer_original_10uav_new_starts_no_bs_sionna_23dbm_mcs14_100mhz_66rb_${duration}s" \
  RACER_RANDOM_SEED=42 \
  RACER_DISCOVERY_DOMAIN_ID="${RACER_SIONNA_DOMAIN_ID:-223}" \
  RACER_COVERAGE_UPDATE_RATE_HZ=0.5 \
  RACER_BANDWIDTH_HZ=100000000.0 \
  RACER_RESOURCE_BLOCKS=66 \
  RACER_UAV_TX_POWER_DBM=23 \
  "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh" \
  >"${suite}/sionna_distributed_original.log" 2>&1 &
active_child=$!
wait "${active_child}"
sionna_status=$?
active_child=""
set -e
printf '%s\n' "${sionna_status}" >"${sionna_suite}/case_status.txt"

python3 - \
  "${perfect_suite}/formal_${duration}s/warehouse_full_distributed_result.json" \
  "${sionna_suite}/formal_${duration}s/warehouse_full_distributed_result.json" \
  "${suite}/comparison_summary.json" <<'PY'
import json
from pathlib import Path
import sys

def summarize(value):
    path = Path(value)
    if not path.is_file():
        return {"available": False, "result": str(path)}
    result = json.loads(path.read_text())
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    statistics = communication.get("statistics", {})
    attempted = int(statistics.get("attempted_packets", 0))
    delivered = int(statistics.get("delivered_packets", 0))
    return {
        "available": True,
        "result": str(path),
        "elapsed_s": metrics.get("elapsed"),
        "coverage": metrics.get("mapping_coverage_joint"),
        "collision_events": metrics.get("collision_events"),
        "total_path_length_m": metrics.get("total_path_length"),
        "communication_mode": communication.get("mode"),
        "network_topology": statistics.get("network_topology"),
        "attempted_packets": attempted,
        "delivered_packets": delivered,
        "packet_success_rate": delivered / attempted if attempted else None,
        "initial_assignment_perfect_delivery_enabled": statistics.get(
            "initial_assignment_perfect_delivery_enabled"
        ),
        "sionna_exact_samples": communication.get("exact_link_samples", 0),
    }

perfect = summarize(sys.argv[1])
sionna = summarize(sys.argv[2])
summary = {
    "perfect": perfect,
    "sionna_distributed_original": sionna,
}
if perfect.get("available") and sionna.get("available"):
    summary["coverage_difference_perfect_minus_sionna"] = (
        float(perfect["coverage"]) - float(sionna["coverage"])
    )
Path(sys.argv[3]).write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
PY

if [[ "${perfect_status}" -eq 0 && "${sionna_status}" -eq 0 ]]; then
  printf '%s\n' completed >"${suite}/run_state.txt"
  exit 0
fi
printf 'completed_with_error perfect=%s sionna=%s\n' \
  "${perfect_status}" "${sionna_status}" >"${suite}/run_state.txt"
exit 1
