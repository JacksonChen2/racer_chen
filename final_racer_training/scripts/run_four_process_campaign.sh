#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
final_root="${RACER_FINAL_RACER_ROOT:-$(realpath "${training_root}/../final_racer")}" 
python_env="${RACER_CRPO_PYTHON:-/home/jiazheng/ai_envs/racer-crpo/bin/python}"
config="${RACER_QWEN8_CONFIG:-${training_root}/config/qwen8b_fp8_four_process.yaml}"
campaign="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/four_process_$(date +%Y%m%d_%H%M%S)}"
episodes="${RACER_QWEN8_EPISODES:-1}"
duration="${RACER_QWEN8_DURATION:-300}"
domain_base="${RACER_QWEN8_DOMAIN_BASE:-118}"
reference="${RACER_QWEN8_PERFECT_RESULT:-${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json}"
selection="${final_root}/config/warehouse_full_10uav_five_sites_layout.json"
scene_usd="${final_root}/assets/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_xml="${final_root}/assets/sionna_scene/warehouse.xml"
gt_path="${RACER_QWEN8_GT_PATH:-${final_root}/data/gt_occupied_voxels.txt}"

if ! [[ "${episodes}" =~ ^[1-9][0-9]*$ ]] ||
   ! [[ "${duration}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Episode count and duration must be positive integers.\n' >&2
  exit 2
fi
if ! [[ "${domain_base}" =~ ^[0-9]+$ ]] ||
   (( domain_base < 1 || domain_base + episodes > 232 )); then
  printf 'RACER_QWEN8_DOMAIN_BASE must leave all episode domains in [1, 232].\n' >&2
  exit 2
fi
for required in "${python_env}" "${config}" "${reference}" "${selection}" \
                "${scene_usd}" "${sionna_xml}" "${gt_path}"; do
  if [[ ! -e "${required}" ]]; then
    printf 'Missing runtime input: %s\n' "${required}" >&2
    exit 2
  fi
done

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf 'Five-site layout must contain 10 three-dimensional starts.\n' >&2
  exit 2
fi

mkdir -p "${campaign}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${final_root}/ros2_ws/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${final_root}/ros2_ws/.sionna_runtime"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${duration}"
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_COVERAGE_UPDATE_RATE_HZ="${RACER_QWEN8_COVERAGE_UPDATE_RATE_HZ:-10}"
export RACER_CAMERA_RAY_BUDGET=76800
export RACER_SENSOR_WORKER_COUNT=8
export RACER_TRIGGER_MINIMUM_CLOUD_FRAMES=0
export RACER_SCENE_QUERY_RATE_HZ=20
export RACER_FIDELITY_HEADLESS=1
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=0
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${starts}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_xml}"
export RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${gt_path}"
export RACER_REQUIRE_GROUND_TRUTH_MAP=true
export RACER_COMMUNICATION_MODE="${RACER_QWEN8_COMMUNICATION_MODE:-sionna}"
export RACER_NETWORK_TOPOLOGY=bs_round_robin
if [[ "${RACER_COMMUNICATION_MODE}" == "ideal" ]]; then
  export RACER_REQUIRE_SIONNA=false
  export RACER_PRESERVE_IDEAL_DIRECT_WITH_BS=true
  unset RACER_SIONNA_RADIO_MAP_CACHE || true
else
  export RACER_REQUIRE_SIONNA=true
  export RACER_PRESERVE_IDEAL_DIRECT_WITH_BS=false
  export RACER_SIONNA_RADIO_MAP_CACHE="${campaign}/no_radio_map_cache.npz"
fi
export RACER_RL_BS_SCHEDULER_ENABLED=true
export RACER_FORCE_BS_PERFECT_DELIVERY="${RACER_QWEN8_FORCE_BS_PERFECT_DELIVERY:-false}"
export RACER_PAIR_CONTROL_RESERVATION_ENABLED=false
if [[ "${RACER_COMMUNICATION_MODE}" == "ideal" ]]; then
  export RACER_EXPLORATION_ASSIGNMENT_MODE="${RACER_EXPLORATION_ASSIGNMENT_MODE:-original}"
  export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY="${RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY:-false}"
  export RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS="${RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS:-false}"
else
  export RACER_EXPLORATION_ASSIGNMENT_MODE="${RACER_EXPLORATION_ASSIGNMENT_MODE:-bs_event_global}"
  export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY="${RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY:-true}"
  export RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS="${RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS:-true}"
fi
export RACER_BS_TX_POWER_DBM="${RACER_QWEN8_BS_TX_POWER_DBM:-40}"
export RACER_UAV_TX_POWER_DBM=23
export RACER_BANDWIDTH_HZ="${RACER_QWEN8_BANDWIDTH_HZ:-100000000}"
export RACER_RESOURCE_BLOCKS="${RACER_QWEN8_RESOURCE_BLOCKS:-66}"
export RACER_UAV_BROADCAST_BANDWIDTH_HZ="${RACER_QWEN8_UAV_BROADCAST_BANDWIDTH_HZ:-50000000}"
export RACER_UAV_BROADCAST_RESOURCE_BLOCKS="${RACER_QWEN8_UAV_BROADCAST_RESOURCE_BLOCKS:-33}"
export RACER_BS_BANDWIDTH_HZ="${RACER_QWEN8_BS_BANDWIDTH_HZ:-50000000}"
export RACER_BS_RESOURCE_BLOCKS="${RACER_QWEN8_BS_RESOURCE_BLOCKS:-33}"
# -1 selects SNR-adaptive MCS.  BS-assisted traffic already uses the
# dedicated adaptive BS link model; keeping the campaign-level model adaptive
# makes the requested PHY mode explicit in the launch manifest as well.
export RACER_FIXED_MCS_INDEX="${RACER_QWEN8_FIXED_MCS_INDEX:--1}"
export RACER_MAX_RETRIES=3
export RACER_BS_MAX_RETRIES=3
export RACER_TASK_METRIC_OBSERVER_MODE=async

resume="${RACER_QWEN8_RESUME:-}"
start_episode="${RACER_QWEN8_START_EPISODE:-1}"
if ! [[ "${start_episode}" =~ ^[1-9][0-9]*$ ]] || (( start_episode > episodes )); then
  printf 'Invalid start episode.\n' >&2
  exit 2
fi
if [[ -n "${resume}" && ! -f "${resume}" ]]; then
  printf 'Missing resume checkpoint: %s\n' "${resume}" >&2
  exit 2
fi
for ((episode = start_episode; episode <= episodes; ++episode)); do
  episode_dir="${campaign}/episode_$(printf '%02d' "${episode}")"
  if [[ -e "${episode_dir}" ]]; then
    printf 'Refusing to overwrite existing episode: %s\n' "${episode_dir}" >&2
    exit 2
  fi
  domain="$((domain_base + episode))"
  command=(
    "${python_env}" -m agentic_crpo.process_supervisor
    --config "${config}"
    --output-dir "${episode_dir}"
    --perfect-reference "${reference}"
    --total-timesteps 100000
    --learning-rate "${RACER_QWEN8_LEARNING_RATE:-0.0002}"
    --n-steps "${RACER_QWEN8_N_STEPS:-384}"
    --batch-size "${RACER_QWEN8_BATCH_SIZE:-64}"
    --n-epochs "${RACER_QWEN8_N_EPOCHS:-10}"
    --activation-fn "${RACER_QWEN8_ACTIVATION_FN:-silu}"
    --gamma-task "${RACER_QWEN8_GAMMA_TASK:-0.25}"
  )
  if [[ -n "${resume}" ]]; then command+=(--resume "${resume}"); fi
  ROS_DOMAIN_ID="${domain}" \
  RACER_LKH_DIR="/tmp/final_racer_four_process_${episode}_lkh" \
  RACER_OBSERVED_OCCUPIED_VOXELS_PATH="${episode_dir}/bs_global_map_occupied_voxels.txt" \
  RACER_ALGORITHM_LABEL="${RACER_QWEN8_ALGORITHM_LABEL:-qwen8b_fp8_crpo_four_process}_episode_${episode}" \
  PYTHONPATH="${training_root}/agentic_crpo${PYTHONPATH:+:${PYTHONPATH}}" \
    "${command[@]}"
  resume="${episode_dir}/training/crpo_final.zip"
  if [[ ! -f "${resume}" ]]; then
    printf 'Episode %d did not produce a checkpoint.\n' "${episode}" >&2
    exit 3
  fi
done

printf 'FOUR_PROCESS_CAMPAIGN_OK output=%s episodes=%s\n' \
  "${campaign}" "${episodes}"
