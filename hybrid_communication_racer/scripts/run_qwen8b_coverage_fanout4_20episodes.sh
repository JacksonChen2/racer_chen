#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/hybrid_communication_racer"
campaign="${workspace}/experiments/five_sites_same_aisle_10uav_300s_20260831/qwen8b_coverage_fanout4_noretry_besteffort_20ep_lr2e4_b64_s384_tanh_20260901"
config="${workspace}/agentic_crpo/config_qwen8b_coverage_fanout4.yaml"
training_dir="${campaign}/training"
bridge_dir="/tmp/racer_agentic_crpo_qwen8b_fanout4"
selection="${workspace}/config/warehouse_full_10uav_five_sites_layout.json"
scene_usd="${workspace}/../warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"
sionna_scene_xml="${workspace}/../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/warehouse.xml"
sionna_runtime="${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime"
python_env="/home/jiazheng/ai_envs/racer-crpo/bin/python"
episodes=20
active_sim=""
active_train=""

mkdir -p "${campaign}" "${training_dir}" "${bridge_dir}"
printf '%s\n' "$$" >"${campaign}/supervisor.pid"
printf '%s\n' "starting" >"${campaign}/run_state.txt"

terminate_group() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "${pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

stop_children() {
  trap - INT TERM HUP
  terminate_group "${active_train}"
  terminate_group "${active_sim}"
  printf '%s\n' "stopped" >"${campaign}/run_state.txt"
  exit 130
}
trap stop_children INT TERM HUP

set +u
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u

if [[ ! -x "${python_env}" || ! -f "${config}" ||
      ! -f "${scene_usd}" || ! -f "${sionna_scene_xml}" ||
      ! -d "${sionna_runtime}/sionna" ||
      ! -d "/home/jiazheng/ai_models/Qwen3-8B-FP8" ]]; then
  printf '%s\n' "blocked:missing_runtime_input" >"${campaign}/run_state.txt"
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
export RACER_FIDELITY_DURATION=300
export RACER_FIDELITY_DRONE_COUNT=10
export RACER_PHYSICS_RATE_HZ=100
export RACER_SENSOR_RATE_HZ=10
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
export RACER_BS_TX_POWER_DBM=33
export RACER_UAV_TX_POWER_DBM=23
export RACER_FIXED_MCS_INDEX=14
# Best-effort modeled wireless service: one PHY attempt and no ARQ on either
# the fixed-MCS UAV broadcast route or adaptive-MCS UAV-BS/BS-UAV routes.
export RACER_MAX_RETRIES=0
export RACER_BS_MAX_RETRIES=0
export RACER_REQUIRE_GROUND_TRUTH_MAP=false
unset RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH || true
unset RACER_OBSERVED_OCCUPIED_VOXELS_PATH || true

export RACER_COMMUNICATION_MODE=sionna
export RACER_NETWORK_TOPOLOGY=bs_round_robin
export RACER_REQUIRE_SIONNA=true
export RACER_RL_BS_SCHEDULER_ENABLED=true
export RACER_RL_BS_ACTION_PATH="${bridge_dir}/action.txt"
export RACER_RL_BS_STATE_PATH="${bridge_dir}/communication_state.json"
export RACER_RL_BS_MISSION_STATE_PATH="${bridge_dir}/mission_state.json"
export RACER_RL_BS_DECISION_PERIOD_MS=20.0
export RACER_SIONNA_RADIO_MAP_CACHE="${campaign}/no_radio_map_cache.npz"

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

validate_episode_result() {
  local result_path="$1"
  "${python_env}" - "${result_path}" <<'PY'
import json
from pathlib import Path
import sys

source = Path(sys.argv[1])
if not source.is_file():
    raise SystemExit(1)
payload = json.loads(source.read_text())
metrics = payload.get("metrics", {})
coverage = float(metrics.get("mapping_coverage_joint", -1.0))
elapsed = float(metrics.get("elapsed", 0.0))
if elapsed < 299.0 or not 0.0 <= coverage <= 1.0:
    raise SystemExit(1)
PY
}

"${python_env}" - "${campaign}/run_manifest.json" "${config}" "${selection}" <<'PY'
import json
from datetime import datetime, timezone
from pathlib import Path
import sys

output, config, selection = map(Path, sys.argv[1:])
payload = {
    "algorithm_variant": "qwen8b_coverage_fanout4",
    "episodes": 20,
    "duration_s": 300,
    "n_uavs": 10,
    "seed": 42,
    "model_path": "/home/jiazheng/ai_models/Qwen3-8B-FP8",
    "reward": "coverage",
    "communication_delivery": {
        "service": "best_effort",
        "uav_max_retries": 0,
        "bs_max_retries": 0,
    },
    "constraint": {
        "type": "relay_fanout",
        "max_recipients_per_source_uav": 4,
        "hard_enforcement": True,
    },
    "ppo": {
        "learning_rate": 2e-4,
        "batch_size": 64,
        "n_steps": 384,
        "activation_fn": "tanh",
        "net_arch": {"pi": [512, 256, 256], "vf": [512, 256, 256]},
    },
    "config": str(config.resolve()),
    "start_layout": str(selection.resolve()),
    "started_at": datetime.now(timezone.utc).astimezone().isoformat(),
}
temporary = output.with_suffix(".tmp")
temporary.write_text(json.dumps(payload, indent=2) + "\n")
temporary.replace(output)
PY

completed=0
if [[ -f "${campaign}/completed_episodes.txt" ]]; then
  completed="$(cat "${campaign}/completed_episodes.txt")"
fi
if ! [[ "${completed}" =~ ^[0-9]+$ ]] || (( completed < 0 || completed > episodes )); then
  printf '%s\n' "blocked:invalid_completed_episode_marker" >"${campaign}/run_state.txt"
  exit 3
fi

for ((episode = completed + 1; episode <= episodes; ++episode)); do
  episode_tag="episode_$(printf '%02d' "${episode}")"
  episode_dir="${campaign}/${episode_tag}"
  sim_dir="${episode_dir}/sim"
  mkdir -p "${sim_dir}" "${episode_dir}/final_bridge"
  rm -f "${bridge_dir}/action.txt" \
        "${bridge_dir}/communication_state.json" \
        "${bridge_dir}/mission_state.json"
  write_progress "training" "$((episode - 1))" "${episode}"
  printf 'training:%s/%s\n' "${episode}" "${episodes}" \
    >"${campaign}/run_state.txt"

  domain="$((180 + episode))"
  ROS_DOMAIN_ID="${domain}" \
  RACER_RESULT_DIR="${sim_dir}" \
  RACER_LKH_DIR="/tmp/racer_qwen8b_fanout4_noretry_20ep_${episode_tag}_lkh" \
  RACER_ALGORITHM_LABEL="qwen8b_coverage_fanout4_noretry_besteffort_${episode_tag}" \
    setsid "${workspace}/run_warehouse_simple_sionna.sh" \
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
    terminate_group "${active_sim}"
    active_sim=""
    printf 'failed:%s:bridge_not_ready\n' "${episode_tag}" \
      >"${campaign}/run_state.txt"
    exit 4
  fi

  train_args=(
    -m agentic_crpo.train_crpo
    --config "${config}"
    --total-timesteps 100000
    --learning-rate 0.0002
    --n-steps 384
    --batch-size 64
    --activation-fn tanh
    --gamma-task 0.0
    --output-dir "${training_dir}"
  )
  if [[ -f "${training_dir}/crpo_final.zip" ]]; then
    train_args+=(--resume "${training_dir}/crpo_final.zip")
  fi
  PYTHONUNBUFFERED=1 setsid "${python_env}" "${train_args[@]}" \
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

  result_path="${sim_dir}/warehouse_full_bs_round_robin_result.json"
  if [[ "${train_status}" -ne 0 || ! -f "${training_dir}/crpo_final.zip" ||
        ! -f "${training_dir}/training_state.json" ]] ||
        ! validate_episode_result "${result_path}"; then
    printf 'failed:%s:trainer=%s:simulator=%s\n' \
      "${episode_tag}" "${train_status}" "${sim_status}" \
      >"${campaign}/run_state.txt"
    exit 5
  fi
  cp "${training_dir}/crpo_final.zip" "${episode_dir}/crpo_after_episode.zip"
  cp "${training_dir}/training_state.json" \
    "${episode_dir}/training_state_after_episode.json"
  printf '%s\n' "${episode}" >"${campaign}/completed_episodes.txt"
  write_progress "training" "${episode}" "${episode}"
done

write_progress "completed" "${episodes}" "${episodes}"
printf '%s\n' "completed" >"${campaign}/run_state.txt"
