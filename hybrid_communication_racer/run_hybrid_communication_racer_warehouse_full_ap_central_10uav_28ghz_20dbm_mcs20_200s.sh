#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/hybrid_communication_racer_warehouse_full_with_ap_central_10uav_sionna_distributed_28ghz_20dbm_mcs20_200s_${run_id}}"
case_dir="${suite}/sionna_distributed_28ghz_20dbm_mcs20"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
starts="-14.49 14.91 0.80 -13.20 14.91 1.50 -11.91 14.91 2.20 -11.91 16.20 1.15 -13.20 16.20 1.85 -7.80 15.00 0.80 -6.60 15.00 1.50 -7.80 16.20 2.20 -6.60 16.20 1.15 -7.80 17.40 1.85"

mkdir -p "${case_dir}"
printf '%s\n' running >"${suite}/run_state.txt"
python3 - "${suite}/experiment_manifest.json" "${scene_usd}" "${scene_xml}" "${starts}" <<'PY'
import json
from pathlib import Path
import sys

values = [float(value) for value in sys.argv[4].split()]
starts = [values[index:index + 3] for index in range(0, len(values), 3)]
manifest = {
    "algorithm": "hybrid_communication_racer",
    "exploration_assignment_mode": "global_cooperative",
    "scene": "warehouse_full_with_industrial_ap",
    "scene_usd": str(Path(sys.argv[2]).resolve()),
    "sionna_scene_xml": str(Path(sys.argv[3]).resolve()),
    "drone_count": 10,
    "duration_s": 200,
    "start_layout": "central_rack_two_sides_5_plus_5",
    "start_positions": starts,
    "communication": {
        "mode": "sionna",
        "network_topology": "distributed",
        "carrier_frequency_hz": 28.0e9,
        "uav_tx_power_dbm": 20.0,
        "fixed_mcs_index": 20,
        "fixed_mcs_modulation": "64QAM",
        "fixed_mcs_code_rate": 567.0 / 1024.0,
        "transport": "UDP",
        "shared_uav_ofdma_prbs": 66,
        "shared_uav_ofdma_scheduler": "round_robin_equal_share",
        "half_duplex": True,
        "max_retries": 0,
        "ap_relay_enabled": False,
        "lossless_link_overrides": False,
        "radio_map_cache_enabled": False,
    },
    "hybrid_global_state_channel": "direct_perfect_global_information",
    "controlled_parameters": {
        "physics_rate_hz": 100,
        "sensor_rate_hz": 10,
        "camera_ray_budget": 76800,
        "sensor_worker_count": 8,
        "scene_query_rate_hz": 50,
        "random_seed": 42,
    },
}
Path(sys.argv[1]).write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
PY

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${RACER_DOMAIN_ID:-97}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=200
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_SCENE_QUERY_RATE_HZ=50
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=1
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${starts}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${scene_xml}"
export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_FIXED_MCS_INDEX=20
export RACER_CARRIER_FREQUENCY_HZ=28000000000.0
export RACER_UAV_TX_POWER_DBM=20.0
export RACER_EXPLORATION_ASSIGNMENT_MODE=global_cooperative
export RACER_ALGORITHM_LABEL=hybrid_global_cooperative_sionna_distributed_28ghz_20dbm_mcs20_200s
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/hybrid_28ghz_20dbm_mcs20_${run_id}_lkh"

set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1
runner_status=$?
set -e
printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

python3 - "${case_dir}" <<'PY' >"${case_dir}/case_validation.log" 2>&1
import json
from pathlib import Path
import sys

case_dir = Path(sys.argv[1])
files = list(case_dir.glob("*_result.json"))
if len(files) != 1:
    raise SystemExit(f"expected one result JSON, found {len(files)}")
result = json.loads(files[0].read_text())
metrics = result.get("metrics", {})
communication = result.get("communication", {})
stats = communication.get("statistics", {})
phy = stats.get("phy", {})
errors = []
if float(metrics.get("elapsed", 0.0)) < 199.0:
    errors.append("simulation did not reach 200 seconds")
if sorted(result.get("executed_drone_ids", [])) != list(range(1, 11)):
    errors.append("not all ten UAV algorithms executed")
if communication.get("mode") != "sionna" or communication.get("network_topology") != "distributed":
    errors.append("communication mode/topology mismatch")
if communication.get("sionna_ready") is not True or int(communication.get("exact_link_samples", 0)) <= 0:
    errors.append("Sionna RT did not provide exact link samples")
if abs(float(phy.get("carrier_frequency_hz", 0.0)) - 28.0e9) > 1.0:
    errors.append("proxy carrier frequency is not 28 GHz")
if abs(float(phy.get("uav_tx_power_dbm", 0.0)) - 20.0) > 1.0e-6:
    errors.append("proxy UAV TX power is not 20 dBm")
if int(phy.get("fixed_mcs_index", -999)) != 20:
    errors.append("proxy fixed MCS index is not 20")
if phy.get("fixed_mcs_modulation") != "64QAM":
    errors.append("proxy fixed MCS modulation is not 64QAM")
if abs(float(phy.get("fixed_mcs_code_rate", 0.0)) - 567.0 / 1024.0) > 1.0e-6:
    errors.append("proxy fixed MCS code rate is not 567/1024")
if stats.get("shared_uav_ofdma_enabled") is not True:
    errors.append("shared UAV OFDMA scheduler was not enabled")
if stats.get("uav_transport") != "UDP":
    errors.append("distributed UAV transport is not UDP")
if int(stats.get("uav_ofdma_total_prbs", 0)) != 66:
    errors.append("shared UAV OFDMA pool is not 66 PRBs")
if int(stats.get("uav_ofdma_max_allocated_prbs_per_slot", 0)) > 66:
    errors.append("a UAV OFDMA slot allocated more than 66 PRBs")
if int(stats.get("uav_udp_retransmissions", -1)) != 0:
    errors.append("UDP radio retransmission was observed")
if int(stats.get("uav_physical_transmissions_started", -1)) != int(
    stats.get("uav_tx_power_applications", -2)
):
    errors.append("UAV TX power was not applied exactly once per physical transmission")
if int(result.get("algorithm_evidence", {}).get("process_crashes", 0)) != 0:
    errors.append("a ROS process crashed")
if errors:
    raise SystemExit("\n".join(errors))
print("INTEGRITY_OK")
PY
validation_status=$?
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"
if [[ "${validation_status}" -eq 0 ]]; then
  printf '%s\n' completed >"${suite}/run_state.txt"
else
  printf '%s\n' completed_with_error >"${suite}/run_state.txt"
fi
printf 'SUITE=%s runner=%s validation=%s\n' "${suite}" "${runner_status}" "${validation_status}"
