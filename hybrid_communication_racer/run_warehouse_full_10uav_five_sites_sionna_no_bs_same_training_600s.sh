#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
random_seed="${RACER_RANDOM_SEED:-42}"
experiment_bandwidth_hz="${RACER_EXPERIMENT_BANDWIDTH_HZ:-50000000}"
experiment_resource_blocks="${RACER_EXPERIMENT_RESOURCE_BLOCKS:-33}"
initial_assignment_perfect_delivery="${RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY:-false}"
comparison_basis="${RACER_COMPARISON_BASIS:-latest completed Qwen8 training episode; BS removed while preserving its U2U radio allocation}"
algorithm_label="${RACER_ALGORITHM_LABEL:-racer_10uav_sionna_distributed_no_bs_same_training_600s}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/five_sites_same_aisle_10uav_sionna_distributed_no_bs_same_training_600s_${run_id}}"
case_dir="${suite}/sim"
selection="${workspace}/config/warehouse_full_10uav_five_sites_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/warehouse.xml"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
expected_scene_sha256="e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e"
expected_sionna_sha256="b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168"
active_child=""

mkdir -p "${case_dir}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

stop_experiment() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${suite}/run_state.txt"
  printf 'STOPPED time=%s suite=%s\n' "$(date --iso-8601=seconds)" "${suite}"
  exit 130
}
trap stop_experiment INT TERM HUP

actual_scene_sha256="$(sha256sum "${scene_usd}" | awk '{print $1}')"
actual_sionna_sha256="$(sha256sum "${sionna_scene_xml}" | awk '{print $1}')"
if [[ "${actual_scene_sha256}" != "${expected_scene_sha256}" ||
      "${actual_sionna_sha256}" != "${expected_sionna_sha256}" ]]; then
  printf '%s\n' "blocked:scene_changed" >"${suite}/run_state.txt"
  printf 'Scene hash mismatch: USD=%s Sionna=%s\n' \
    "${actual_scene_sha256}" "${actual_sionna_sha256}" >&2
  exit 2
fi
if [[ ! -d "${sionna_runtime}/sionna" ]]; then
  printf '%s\n' "blocked:missing_sionna_runtime" >"${suite}/run_state.txt"
  printf 'Missing Sionna RT runtime: %s\n' "${sionna_runtime}" >&2
  exit 2
fi
if [[ "${initial_assignment_perfect_delivery}" != "true" &&
      "${initial_assignment_perfect_delivery}" != "false" ]]; then
  printf '%s\n' "blocked:invalid_initial_assignment_mode" >"${suite}/run_state.txt"
  printf 'RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY must be true or false.\n' >&2
  exit 2
fi

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf '%s\n' "blocked:invalid_start_layout" >"${suite}/run_state.txt"
  printf 'Expected ten XYZ start positions in %s\n' "${selection}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
if [[ "$(ros2 pkg prefix racer_original_core)" != "${workspace}/install/racer_original_core" ||
      "$(ros2 pkg prefix racer_fidelity_msgs)" != "${workspace}/install/racer_fidelity_msgs" ]]; then
  printf '%s\n' "blocked:wrong_ros_overlay" >"${suite}/run_state.txt"
  printf 'Wrong ROS overlay is active.\n' >&2
  exit 2
fi

jq -n \
  --arg run_id "${run_id}" \
  --arg scene_usd "${scene_usd}" \
  --arg scene_sha256 "${actual_scene_sha256}" \
  --arg sionna_xml "${sionna_scene_xml}" \
  --arg sionna_sha256 "${actual_sionna_sha256}" \
  --arg comparison_basis "${comparison_basis}" \
  --argjson starts "$(jq '.start_positions' "${selection}")" \
  --argjson seed "${random_seed}" \
  --argjson bandwidth_hz "${experiment_bandwidth_hz}" \
  --argjson resource_blocks "${experiment_resource_blocks}" \
  --argjson initial_assignment_perfect_delivery "${initial_assignment_perfect_delivery}" \
  '{
    run_id: $run_id,
    comparison_basis: $comparison_basis,
    duration_s: 600,
    drone_count: 10,
    random_seed: $seed,
    scene_usd: $scene_usd,
    scene_usd_sha256: $scene_sha256,
    sionna_scene_xml: $sionna_xml,
    sionna_scene_xml_sha256: $sionna_sha256,
    start_positions: $starts,
    physics_rate_hz: 100,
    sensor_rate_hz: 10,
    camera_ray_budget: 76800,
    sensor_worker_count: 8,
    scene_query_rate_hz: 20,
    task_metric_observer_mode: "async",
    communication: {
      mode: "sionna",
      network_topology: "distributed",
      bs_enabled: false,
      rl_bs_scheduler_enabled: false,
      uav_tx_power_dbm: 23,
      bandwidth_hz: $bandwidth_hz,
      resource_blocks: $resource_blocks,
      fixed_mcs_index: 14,
      max_retries: 3,
      initial_assignment_perfect_delivery: $initial_assignment_perfect_delivery,
      shared_uav_ofdma: true,
      transport: "UDP directed unicast"
    }
  }' >"${suite}/experiment_manifest.json"

export ROS_DOMAIN_ID="${RACER_DISCOVERY_DOMAIN_ID:-104}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION=600
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_TRIGGER_DELAY_S=5.0
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_STARTUP_FREE_SPACE_YAW=0
export RACER_STARTUP_SCAN_DURATION=0.0
export RACER_STARTUP_UNKNOWN_CORRIDOR_DISTANCE=0.0
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-300}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"
export RACER_RANDOM_SEED="${random_seed}"
export RACER_START_POSITIONS="${starts}"
export RACER_COVERAGE_UPDATE_RATE_HZ=10.0
export RACER_TASK_METRIC_OBSERVER_MODE=async
export RACER_EXPLORATION_ASSIGNMENT_MODE=original

export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_SIONNA_RADIO_MAP_CACHE="${suite}/no_radio_map_cache.npz"
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_COMMUNICATION_MODE=sionna
export RACER_REQUIRE_SIONNA=true
export RACER_RL_BS_SCHEDULER_ENABLED=false
export RACER_RL_SYNC_ENABLED=false
export RACER_PRESERVE_IDEAL_DIRECT_WITH_BS=false
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY="${initial_assignment_perfect_delivery}"
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_DIRECTED_MESSAGE_UNICAST=false
export RACER_CHUNK_DATA_PRE_ENQUEUE_DEDUP=false
export RACER_CHUNK_DATA_MAX_PENDING_PER_LINK=0
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20.0
export RACER_UAV_TX_POWER_DBM=23
export RACER_BANDWIDTH_HZ="${experiment_bandwidth_hz}"
export RACER_RESOURCE_BLOCKS="${experiment_resource_blocks}"
export RACER_MAX_RETRIES=3
export RACER_BS_MAX_RETRIES=3
export RACER_FIXED_MCS_INDEX=14
export RACER_RESULT_DIR="${case_dir}"
export RACER_LKH_DIR="/tmp/racer_sionna_distributed_no_bs_same_training_600s_${run_id}_lkh"
export RACER_ALGORITHM_LABEL="${algorithm_label}"

printf '%s\n' "running" >"${suite}/run_state.txt"
printf 'START time=%s suite=%s mode=sionna topology=distributed bs=off duration=600 uav_dbm=23 mcs=14 retries=3 bandwidth_hz=%s resource_blocks=%s initial_assignment_perfect=%s domain=%s seed=%s\n' \
  "$(date --iso-8601=seconds)" "${suite}" "${experiment_bandwidth_hz}" \
  "${experiment_resource_blocks}" "${initial_assignment_perfect_delivery}" \
  "${ROS_DOMAIN_ID}" "${random_seed}"

set +e
"${workspace}/run_warehouse_simple_sionna.sh" >"${case_dir}/runner.log" 2>&1 &
active_child=$!
printf '%s\n' "${active_child}" >"${case_dir}/runner.pid"
wait "${active_child}"
runner_status=$?
active_child=""
set -e
printf '%s\n' "${runner_status}" >"${case_dir}/runner_exit_status.txt"

mapfile -t results < <(find "${case_dir}" -maxdepth 1 -type f -name '*_result.json' -print)
validation_status=1
if [[ "${#results[@]}" -eq 1 ]]; then
  result="${results[0]}"
  set +e
  jq -e --slurpfile layout "${selection}" \
    --argjson bandwidth_hz "${experiment_bandwidth_hz}" \
    --argjson resource_blocks "${experiment_resource_blocks}" \
    --argjson initial_assignment_perfect_delivery "${initial_assignment_perfect_delivery}" '
    .random_seed == 42 and
    .drone_count == 10 and
    .metrics.elapsed >= 599.0 and
    .metrics.stop_reason == "duration" and
    .metrics.physics_rate_hz == 100 and
    .metrics.sensor_rate_hz == 10 and
    .metrics.camera_ray_budget == 76800 and
    .metrics.scene_query_rate_hz == 20 and
    .metrics.start_positions == $layout[0].start_positions and
    .metrics.startup_recovery.enabled == false and
    .metrics.collision_events == 0 and
    .metrics.physics_contact_events == 0 and
    .acceptance.isaac_exit_ok == true and
    .communication.mode == "sionna" and
    .communication.network_topology == "distributed" and
    .communication.sionna_ready == true and
    .communication.exact_link_samples > 0 and
    .communication.statistics.network_topology == "distributed" and
    .communication.statistics.ap_enabled == false and
    .communication.statistics.rl_bs_scheduler_enabled == false and
    .communication.statistics.shared_uav_ofdma_enabled == true and
    .communication.statistics.uav_transport == "UDP" and
    .communication.statistics.uav_udp_directed_unicast == true and
    .communication.statistics.task_metric_observer_mode == "async" and
    .communication.statistics.phy.uav_tx_power_dbm == 23 and
    .communication.statistics.phy.bandwidth_hz == $bandwidth_hz and
    .communication.statistics.phy.resource_blocks == $resource_blocks and
    .communication.statistics.phy.fixed_mcs_index == 14 and
    .communication.statistics.phy.max_retries == 3 and
    ((($initial_assignment_perfect_delivery | not)) or
      (.communication.statistics.initial_assignment_perfect_delivery_enabled == true and
       .communication.statistics.initial_assignment_perfect_delivery_complete == true and
       .communication.statistics.initial_assignment_perfect_epoch == 1 and
       .communication.statistics.initial_assignment_perfect_acks == 10 and
       .communication.statistics.initial_assignment_perfect_completed_at_s > 0 and
       .communication.statistics.initial_assignment_perfect_global_assignment_messages > 0 and
       .communication.statistics.initial_assignment_perfect_forwarded_packets > 0 and
       .communication.statistics.sionna_direct_attempted_packets > 0)) and
    (.metrics.path_lengths | length) == 10 and
    (.metrics.mapping_coverage_per_agent | length) == 10 and
    .algorithm_evidence.process_crashes == 0
  ' "${result}" >"${case_dir}/case_validation.log" 2>&1
  validation_status=$?
  set -e
  jq '{
    result_file: input_filename,
    algorithm_passed: .passed,
    algorithm_acceptance: .acceptance,
    executed_drone_ids: .executed_drone_ids,
    elapsed_s: .metrics.elapsed,
    coverage: .metrics.mapping_coverage_joint,
    collisions: .metrics.collision_events,
    topology: .communication.statistics.network_topology,
    ap_enabled: .communication.statistics.ap_enabled,
    exact_link_samples: .communication.exact_link_samples,
    attempted_packets: .communication.statistics.attempted_packets,
    delivered_packets: .communication.statistics.delivered_packets,
    delivery_ratio: .communication.statistics.logical_delivery_ratio,
    initial_assignment: {
      enabled: .communication.statistics.initial_assignment_perfect_delivery_enabled,
      complete: .communication.statistics.initial_assignment_perfect_delivery_complete,
      epoch: .communication.statistics.initial_assignment_perfect_epoch,
      acknowledgements: .communication.statistics.initial_assignment_perfect_acks,
      completed_at_s: .communication.statistics.initial_assignment_perfect_completed_at_s,
      forwarded_packets: .communication.statistics.initial_assignment_perfect_forwarded_packets
    },
    phy: .communication.statistics.phy
  }' "${result}" >"${case_dir}/result_summary.json"
else
  printf 'Expected one result JSON, found %s\n' "${#results[@]}" >"${case_dir}/case_validation.log"
fi
printf '%s\n' "${validation_status}" >"${case_dir}/case_validation_status.txt"

if [[ "${validation_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error runner=%s validation=%s\n' \
    "${runner_status}" "${validation_status}" >"${suite}/run_state.txt"
fi
printf 'FINISH time=%s suite=%s runner=%s validation=%s\n' \
  "$(date --iso-8601=seconds)" "${suite}" "${runner_status}" "${validation_status}"
exit "${validation_status}"
