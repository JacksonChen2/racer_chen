#!/usr/bin/env bash
set -euo pipefail

# Run the requested total-PRB experiment first, then start the independent
# BS-only experiment only after all 40 checkpoints have completed.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
training_root="$(realpath "${script_dir}/..")"
schedule_runner="${script_dir}/run_event_driven_one_shot_constraint_schedule.sh"
series_id="${RACER_QWEN8_SERIES_ID:-$(date +%Y%m%d_%H%M%S)}"

total_prb_campaign="${RACER_TOTAL_PRB_CAMPAIGN:-${training_root}/results/event_one_shot_total_prb_nsteps512_40ep_${series_id}}"
bs_only_campaign="${RACER_BS_ONLY_CAMPAIGN:-${training_root}/results/event_one_shot_bs_only_norm5000_nsteps512_30ep_${series_id}}"
bs_only_config="${training_root}/config/qwen8b_fp8_llm_action_prior_event_driven_one_shot_bs_only_norm5000.yaml"

RACER_QWEN8_TOTAL_EPISODES=40 \
RACER_QWEN8_CAMPAIGN="${total_prb_campaign}" \
RACER_QWEN8_CONFIG="${training_root}/config/qwen8b_fp8_llm_action_prior_event_driven_one_shot_reference752763_traj020_red020_constraint012.yaml" \
RACER_QWEN8_START_EPISODE=1 \
RACER_QWEN8_RESUME= \
  "${schedule_runner}"

RACER_QWEN8_TOTAL_EPISODES=30 \
RACER_QWEN8_CAMPAIGN="${bs_only_campaign}" \
RACER_QWEN8_CONFIG="${bs_only_config}" \
RACER_QWEN8_START_EPISODE=1 \
RACER_QWEN8_RESUME= \
  "${schedule_runner}"

printf 'EVENT_ONE_SHOT_TWO_STAGE_OK total_prb=%s bs_only=%s\n' \
  "${total_prb_campaign}" "${bs_only_campaign}"
