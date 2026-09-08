#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
final_root="${RACER_FINAL_RACER_ROOT:-$(realpath "${training_root}/../final_racer")}" 
workspace="${final_root}/ros2_ws"
communication_overlay="${final_root}/sionna_distributed_overlay_ws/install/setup.bash"
adapter_overlay="${training_root}/training_overlay_ws/install/setup.bash"
runner="${training_root}/runtime/run_final_racer_training.sh"
campaign="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/qwen8b_fp8_latest_$(date +%Y%m%d_%H%M%S)}"
config="${RACER_QWEN8_CONFIG:-${training_root}/config/qwen8b_fp8_latest.yaml}"
reference_root="${RACER_QWEN8_REFERENCE_ROOT:-${campaign}/reference}"
gt_path="${RACER_QWEN8_GT_PATH:-${final_root}/data/gt_occupied_voxels.txt}"
perfect_result="${RACER_QWEN8_PERFECT_RESULT:-${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json}"
auxiliary_result="${RACER_QWEN8_AUXILIARY_RESULT:-}"
training_dir="${campaign}/training"
bridge_dir="${RACER_QWEN8_BRIDGE_DIR:-/tmp/racer_agentic_crpo_qwen8b_taskloss_single_gpu}"
pause_request="${bridge_dir}/pause_request.json"
pause_ack="${bridge_dir}/pause_ack.json"
sync_release="${bridge_dir}/rl_sync_release.json"
sync_ack="${bridge_dir}/rl_sync_ack.json"
selection="${final_root}/config/warehouse_full_10uav_five_sites_layout.json"
scene_usd="${final_root}/assets/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${final_root}/assets/sionna_scene/warehouse.xml"
sionna_runtime="${final_root}/ros2_ws/.sionna_runtime"
python_env="/home/jiazheng/ai_envs/racer-crpo/bin/python"
episodes="${RACER_QWEN8_EPISODES:-30}"
episode_duration="${RACER_QWEN8_DURATION:-300}"
train_learning_rate="${RACER_QWEN8_LEARNING_RATE:-0.0002}"
train_n_steps="${RACER_QWEN8_N_STEPS:-384}"
train_batch_size="${RACER_QWEN8_BATCH_SIZE:-64}"
train_n_epochs="${RACER_QWEN8_N_EPOCHS:-10}"
train_activation_fn="${RACER_QWEN8_ACTIVATION_FN:-silu}"
train_gamma_task="${RACER_QWEN8_GAMMA_TASK:-0.15}"
training_label="${RACER_QWEN8_TRAINING_LABEL:-qwen8b_taskloss_single_gpu_gamma015}"
communication_mode="${RACER_QWEN8_COMMUNICATION_MODE:-sionna}"
network_topology="${RACER_QWEN8_NETWORK_TOPOLOGY:-bs_round_robin}"
require_sionna="${RACER_QWEN8_REQUIRE_SIONNA:-true}"
preserve_ideal_direct_with_bs="${RACER_QWEN8_PRESERVE_IDEAL_DIRECT_WITH_BS:-false}"
domain_base="${RACER_QWEN8_DOMAIN_BASE:-119}"
minimum_coverage="${RACER_QWEN8_MINIMUM_COVERAGE:-}"
require_qwen_model="${RACER_REQUIRE_QWEN_MODEL:-true}"
enable_single_gpu_pause="${RACER_SINGLE_GPU_PAUSE_ENABLED:-true}"
active_sim=""
active_train=""

if ! [[ "${episodes}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_QWEN8_EPISODES must be a positive integer.\n' >&2
  exit 2
fi
if ! [[ "${episode_duration}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RACER_QWEN8_DURATION must be a positive integer.\n' >&2
  exit 2
fi
if ! [[ "${domain_base}" =~ ^[0-9]+$ ]] ||
   (( domain_base + episodes > 232 )); then
  printf 'RACER_QWEN8_DOMAIN_BASE must leave all episode domains in [1, 232].\n' >&2
  exit 2
fi
if [[ "${network_topology}" != "bs_round_robin" ]]; then
  printf 'CRPO training requires RACER_QWEN8_NETWORK_TOPOLOGY=bs_round_robin.\n' >&2
  exit 2
fi
if [[ "${communication_mode}" == "ideal" ]]; then
  if [[ "${require_sionna}" != "false" ||
        "${preserve_ideal_direct_with_bs}" != "true" ]]; then
    printf 'Ideal CRPO training requires Sionna disabled and preserved ideal direct communication.\n' >&2
    exit 2
  fi
elif [[ "${communication_mode}" != "sionna" &&
        "${communication_mode}" != "sionna_hybrid" ]]; then
  printf 'Unsupported RACER_QWEN8_COMMUNICATION_MODE.\n' >&2
  exit 2
fi
if [[ -n "${minimum_coverage}" ]] &&
   ! "${python_env}" -c 'import math,sys; x=float(sys.argv[1]); raise SystemExit(not (math.isfinite(x) and 0.0 <= x <= 1.0))' "${minimum_coverage}"; then
  printf 'RACER_QWEN8_MINIMUM_COVERAGE must be in [0, 1].\n' >&2
  exit 2
fi

mkdir -p "${campaign}" "${reference_root}" "${training_dir}" "${bridge_dir}"
printf '%s\n' "$$" >"${campaign}/supervisor.pid"
printf '%s\n' "starting" >"${campaign}/run_state.txt"

stop_children() {
  trap - INT TERM HUP
  if [[ -n "${active_train}" ]]; then
    kill -TERM "${active_train}" 2>/dev/null || true
    wait "${active_train}" 2>/dev/null || true
  fi
  if [[ -n "${active_sim}" ]]; then
    kill -TERM "${active_sim}" 2>/dev/null || true
    wait "${active_sim}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${campaign}/run_state.txt"
  exit 130
}
trap stop_children INT TERM HUP

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
source "${communication_overlay}"
source "${adapter_overlay}"
set -u

if [[ ! -x "${python_env}" || ! -f "${config}" ||
      ! -f "${scene_usd}" || ! -x "${runner}" ||
      ! -f "${communication_overlay}" || ! -f "${adapter_overlay}" ]]; then
  printf '%s\n' "blocked:missing_runtime_input" >"${campaign}/run_state.txt"
  exit 2
fi
if [[ "${require_sionna}" == "true" ]] &&
   { [[ ! -f "${sionna_scene_xml}" ]] ||
     [[ ! -d "${sionna_runtime}/sionna" ]]; }; then
  printf '%s\n' "blocked:missing_sionna_runtime_input" >"${campaign}/run_state.txt"
  exit 2
fi
if [[ "${require_qwen_model}" == "true" &&
      ! -d "/home/jiazheng/ai_models/Qwen3-8B-FP8" ]]; then
  printf '%s\n' "blocked:missing_qwen_model" >"${campaign}/run_state.txt"
  exit 2
fi

starts="$(jq -r '.start_positions | flatten | map(tostring) | join(" ")' "${selection}")"
if [[ "$(wc -w <<<"${starts}")" -ne 30 ]]; then
  printf '%s\n' "blocked:invalid_start_layout" >"${campaign}/run_state.txt"
  exit 2
fi

export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/config/fastdds_large_scale.xml"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE}"
export SIONNA_RUNTIME_DIR="${sionna_runtime}"
export CUDA_VISIBLE_DEVICES=0

export RACER_FIDELITY_SCENARIO=warehouse_full
export RACER_FIDELITY_DURATION="${episode_duration}"
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
export RACER_COVERAGE_UPDATE_RATE_HZ=10
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
export RACER_WALL_TIME_MULTIPLIER=300
export RACER_WALL_TIME_GRACE_SECONDS=600
export RACER_RANDOM_SEED=42
export RACER_START_POSITIONS="${starts}"
export RACER_SCENE_USD="${scene_usd}"
export RACER_SIONNA_SCENE_XML="${sionna_scene_xml}"
export RACER_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT=0
export RACER_LOSSLESS_COMMUNICATION_RANGE_M=0.0
export RACER_LOSSLESS_CONTROL_ONLY=false
export RACER_COMMUNICATION_RANGE_M=4.0
export RACER_IDEAL_COALESCE_WINDOW_MS=20
export RACER_BS_TX_POWER_DBM="${RACER_QWEN8_BS_TX_POWER_DBM:-40}"
export RACER_UAV_TX_POWER_DBM=23
export RACER_FIXED_MCS_INDEX=14
export RACER_MAX_RETRIES=3
export RACER_BS_MAX_RETRIES=3
export RACER_TASK_METRIC_OBSERVER_MODE=async

write_progress() {
  local phase="$1"
  local completed="$2"
  local episode="$3"
  "${python_env}" - "${campaign}/progress.json" "${phase}" \
    "${completed}" "${episode}" "${episodes}" <<'PY'
import json
from datetime import datetime, timezone
from pathlib import Path
import sys

output = Path(sys.argv[1])
payload = {
    "phase": sys.argv[2],
    "completed_episodes": int(sys.argv[3]),
    "current_episode": int(sys.argv[4]),
    "target_episodes": int(sys.argv[5]),
    "updated_at": datetime.now(timezone.utc).astimezone().isoformat(),
}
temporary = output.with_suffix(".tmp")
temporary.write_text(json.dumps(payload, indent=2) + "\n")
temporary.replace(output)
PY
}

validate_reference() {
  "${python_env}" - "${perfect_result}" "${auxiliary_result}" <<'PY'
import json
from pathlib import Path
import sys

source = Path(sys.argv[1])
auxiliary = Path(sys.argv[2]) if sys.argv[2] else None
if not source.is_file():
    raise SystemExit(1)
result = json.loads(source.read_text())
metrics = result.get("metrics", {})
history = result.get("communication", {}).get("statistics", {}).get(
    "task_quality_history", []
)
if float(metrics.get("elapsed", 0.0)) < 299.0:
    raise SystemExit(1)
if not metrics.get("trajectory_history") or not metrics.get(
    "mapping_coverage_history"
):
    raise SystemExit(1)
if not history and auxiliary is not None and auxiliary.is_file():
    auxiliary_result = json.loads(auxiliary.read_text())
    auxiliary_metrics = auxiliary_result.get("metrics", {})
    if float(auxiliary_metrics.get("elapsed", 0.0)) < 299.0:
        raise SystemExit(1)
    history = auxiliary_result.get("communication", {}).get(
        "statistics", {}
    ).get("task_quality_history", [])
if not history or not all(
    float(item.get("bs_global_map_iou", -1.0)) >= 0.0
    and float(item.get("redundant_exploration_ratio", -1.0)) >= 0.0
    for item in history
):
    raise SystemExit(1)
PY
}

run_reference_case() {
  local phase="$1"
  local output_dir="$2"
  local domain="$3"
  mkdir -p "${output_dir}"
  write_progress "${phase}" 0 0
  printf '%s\n' "${phase}" >"${campaign}/run_state.txt"
  ROS_DOMAIN_ID="${domain}" \
  RACER_COMMUNICATION_MODE=ideal \
  RACER_NETWORK_TOPOLOGY=distributed \
  RACER_REQUIRE_SIONNA=false \
  RACER_RL_BS_SCHEDULER_ENABLED=false \
  RACER_RESULT_DIR="${output_dir}" \
  RACER_LKH_DIR="/tmp/racer_qwen8b_taskloss_single_gpu_${phase}_lkh" \
  RACER_ALGORITHM_LABEL="pairwise_robust_10uav_${phase}" \
    "${runner}" \
      >"${output_dir}/runner.log" 2>&1 &
  active_sim=$!
  printf '%s\n' "${active_sim}" >"${output_dir}/simulator.pid"

  # The generic runner can report a non-zero acceptance status when its
  # legacy log-string checks do not match this newer instrumentation.  The
  # strict artifact checks below are authoritative, so preserve the status
  # for diagnosis without aborting the campaign before validating output.
  set +e
  wait "${active_sim}"
  local reference_status=$?
  set -e
  active_sim=""
  printf '%s\n' "${reference_status}" >"${output_dir}/simulator_exit_status.txt"
}

if [[ ! -s "${gt_path}" ]]; then
  gt_dir="${reference_root}/gt_pass"
  export RACER_OBSERVED_OCCUPIED_VOXELS_PATH="${gt_path}"
  unset RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH || true
  export RACER_REQUIRE_GROUND_TRUTH_MAP=false
  run_reference_case "collecting_gt" "${gt_dir}" 116
  unset RACER_OBSERVED_OCCUPIED_VOXELS_PATH
  if [[ ! -s "${gt_path}" ]]; then
    printf '%s\n' "failed:empty_gt" >"${campaign}/run_state.txt"
    exit 3
  fi
fi

if ! validate_reference 2>/dev/null; then
  perfect_dir="${reference_root}/perfect_reference"
  export RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${gt_path}"
  export RACER_REQUIRE_GROUND_TRUTH_MAP=true
  run_reference_case "collecting_perfect_reference" "${perfect_dir}" 117
  if ! validate_reference; then
    printf '%s\n' "failed:invalid_perfect_reference" >"${campaign}/run_state.txt"
    exit 4
  fi
fi

export RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="${gt_path}"
export RACER_REQUIRE_GROUND_TRUTH_MAP=true
export RACER_COMMUNICATION_MODE="${communication_mode}"
export RACER_NETWORK_TOPOLOGY="${network_topology}"
export RACER_REQUIRE_SIONNA="${require_sionna}"
export RACER_PRESERVE_IDEAL_DIRECT_WITH_BS="${preserve_ideal_direct_with_bs}"
export RACER_RL_BS_SCHEDULER_ENABLED=true
export RACER_PAIR_CONTROL_RESERVATION_ENABLED=false
if [[ "${communication_mode}" == "ideal" ]]; then
  export RACER_EXPLORATION_ASSIGNMENT_MODE=original
  export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false
  export RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS=false
else
  export RACER_EXPLORATION_ASSIGNMENT_MODE=bs_event_global
  export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=true
  export RACER_INITIAL_ASSIGNMENT_PERFECT_VIA_BS=true
fi
export RACER_RL_BS_ACTION_PATH="${bridge_dir}/action.txt"
export RACER_RL_BS_STATE_PATH="${bridge_dir}/communication_state.json"
export RACER_RL_BS_MISSION_STATE_PATH="${bridge_dir}/mission_state.json"
export RACER_RL_BS_COMMUNICATION_SLOT_MS=20.0
export RACER_RL_BS_DECISION_PERIOD_MS=100.0
export RACER_RL_SYNC_ENABLED=true
export RACER_RL_SYNC_RELEASE_PATH="${sync_release}"
export RACER_RL_SYNC_ACK_PATH="${sync_ack}"
export RACER_RL_SYNC_MAXIMUM_WAIT_S=1200.0
export RACER_RL_SYNC_POLL_S=0.002
if [[ "${require_sionna}" == "true" ]]; then
  export RACER_SIONNA_RADIO_MAP_CACHE="${campaign}/no_radio_map_cache.npz"
else
  unset RACER_SIONNA_RADIO_MAP_CACHE || true
fi
if [[ "${enable_single_gpu_pause}" == "true" ]]; then
  export RACER_SINGLE_GPU_PAUSE_REQUEST_PATH="${pause_request}"
  export RACER_SINGLE_GPU_PAUSE_ACK_PATH="${pause_ack}"
  export RACER_SINGLE_GPU_MAXIMUM_PAUSE_S=600.0
  export RACER_SINGLE_GPU_PAUSE_POLL_S=0.01
else
  unset RACER_SINGLE_GPU_PAUSE_REQUEST_PATH || true
  unset RACER_SINGLE_GPU_PAUSE_ACK_PATH || true
  unset RACER_SINGLE_GPU_MAXIMUM_PAUSE_S || true
  unset RACER_SINGLE_GPU_PAUSE_POLL_S || true
fi

completed=0
if [[ -f "${campaign}/completed_episodes.txt" ]]; then
  completed="$(cat "${campaign}/completed_episodes.txt")"
fi
if ! [[ "${completed}" =~ ^[0-9]+$ ]] || (( completed < 0 || completed > episodes )); then
  printf '%s\n' "blocked:invalid_completed_episode_marker" >"${campaign}/run_state.txt"
  exit 5
fi

for ((episode = completed + 1; episode <= episodes; ++episode)); do
  episode_tag="episode_$(printf '%02d' "${episode}")"
  episode_dir="${campaign}/${episode_tag}"
  sim_dir="${episode_dir}/sim"
  mkdir -p "${sim_dir}" "${episode_dir}/final_bridge"
  rm -f "${bridge_dir}/action.txt" \
        "${bridge_dir}/communication_state.json" \
        "${bridge_dir}/mission_state.json" \
        "${pause_request}" "${pause_ack}" \
        "${sync_release}" "${sync_ack}"
  write_progress "training" "$((episode - 1))" "${episode}"
  printf 'training:%s/%s\n' "${episode}" "${episodes}" \
    >"${campaign}/run_state.txt"

  domain="$((domain_base + episode))"
  ROS_DOMAIN_ID="${domain}" \
  RACER_RESULT_DIR="${sim_dir}" \
  RACER_LKH_DIR="/tmp/racer_qwen8b_taskloss_single_gpu_${episode_tag}_lkh" \
  RACER_ALGORITHM_LABEL="${training_label}_${episode_tag}" \
    "${runner}" \
      >"${sim_dir}/runner.log" 2>&1 &
  active_sim=$!
  printf '%s\n' "${active_sim}" >"${episode_dir}/simulator.pid"

  ready=0
  for ((attempt = 1; attempt <= 360; ++attempt)); do
    if ! kill -0 "${active_sim}" 2>/dev/null; then
      break
    fi
    if "${python_env}" - "${bridge_dir}/communication_state.json" \
      "${bridge_dir}/mission_state.json" <<'PY' 2>/dev/null
import json
from pathlib import Path
import sys
for name in sys.argv[1:]:
    value = json.loads(Path(name).read_text())
    if not isinstance(value, dict):
        raise SystemExit(1)
PY
    then
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "${ready}" -ne 1 ]]; then
    wait "${active_sim}" || true
    active_sim=""
    printf 'failed:%s:bridge_not_ready\n' "${episode_tag}" \
      >"${campaign}/run_state.txt"
    exit 6
  fi

  train_args=(
    -m agentic_crpo.train_crpo
    --config "${config}"
    --perfect-reference "${perfect_result}"
    --total-timesteps 100000
    --learning-rate "${train_learning_rate}"
    --n-steps "${train_n_steps}"
    --batch-size "${train_batch_size}"
    --n-epochs "${train_n_epochs}"
    --activation-fn "${train_activation_fn}"
    --gamma-task "${train_gamma_task}"
    --output-dir "${training_dir}"
  )
  if [[ -f "${training_dir}/crpo_final.zip" ]]; then
    train_args+=(--resume "${training_dir}/crpo_final.zip")
  fi
  PYTHONPATH="${training_root}/agentic_crpo${PYTHONPATH:+:${PYTHONPATH}}" \
  PYTHONUNBUFFERED=1 "${python_env}" "${train_args[@]}" \
    >"${episode_dir}/train.log" 2>&1 &
  active_train=$!
  printf '%s\n' "${active_train}" >"${episode_dir}/trainer.pid"

  set +e
  wait "${active_train}"
  train_status=$?
  active_train=""
  wait "${active_sim}"
  sim_status=$?
  active_sim=""
  set -e
  printf '%s\n' "${train_status}" >"${episode_dir}/trainer_exit_status.txt"
  printf '%s\n' "${sim_status}" >"${episode_dir}/simulator_exit_status.txt"
  cp "${bridge_dir}/action.txt" "${episode_dir}/final_bridge/action.txt" 2>/dev/null || true
  cp "${bridge_dir}/communication_state.json" \
    "${episode_dir}/final_bridge/communication_state.json" 2>/dev/null || true
  cp "${bridge_dir}/mission_state.json" \
    "${episode_dir}/final_bridge/mission_state.json" 2>/dev/null || true
  cp "${pause_ack}" \
    "${episode_dir}/final_bridge/pause_ack.json" 2>/dev/null || true
  cp "${sync_ack}" \
    "${episode_dir}/final_bridge/rl_sync_ack.json" 2>/dev/null || true

  result_json="${sim_dir}/warehouse_full_${network_topology}_result.json"
  if [[ -n "${minimum_coverage}" ]] &&
     ! "${python_env}" - "${result_json}" "${minimum_coverage}" \
       "${episode_duration}" "${preserve_ideal_direct_with_bs}" <<'PY'
import json
from pathlib import Path
import sys

source = Path(sys.argv[1])
minimum_coverage = float(sys.argv[2])
expected_duration = float(sys.argv[3])
require_preserved_direct = sys.argv[4] == "true"
if not source.is_file():
    raise SystemExit(1)
result = json.loads(source.read_text())
metrics = result.get("metrics", {})
statistics = result.get("communication", {}).get("statistics", {})
coverage = float(metrics.get("mapping_coverage_joint", -1.0))
elapsed = float(metrics.get("elapsed", 0.0))
if elapsed < expected_duration - 1.0 or coverage < minimum_coverage:
    raise SystemExit(1)
if require_preserved_direct and not (
    statistics.get("ideal_direct_enabled") is True
    and statistics.get("preserve_ideal_direct_with_bs") is True
    and statistics.get("rl_bs_scheduler_enabled") is True
):
    raise SystemExit(1)
PY
  then
    printf 'failed:%s:coverage_or_isolation_guard\n' "${episode_tag}" \
      >"${campaign}/run_state.txt"
    exit 8
  fi

  if [[ "${train_status}" -ne 0 || ! -f "${training_dir}/crpo_final.zip" ||
        ! -f "${training_dir}/training_state.json" ]]; then
    printf 'failed:%s:trainer=%s:simulator=%s\n' \
      "${episode_tag}" "${train_status}" "${sim_status}" \
      >"${campaign}/run_state.txt"
    exit 7
  fi
  cp "${training_dir}/crpo_final.zip" "${episode_dir}/crpo_after_episode.zip"
  cp "${training_dir}/training_state.json" \
    "${episode_dir}/training_state_after_episode.json"
  printf '%s\n' "${episode}" >"${campaign}/completed_episodes.txt"
  write_progress "training" "${episode}" "${episode}"
done

write_progress "completed" "${episodes}" "${episodes}"
printf '%s\n' "completed" >"${campaign}/run_state.txt"
