#!/usr/bin/env bash
set -euo pipefail

# Dedicated event-driven experiment requested for the 50-episode constraint
# schedule.  The fixed-clock launcher and the reusable event-driven launcher
# retain their existing defaults.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
training_root="$(realpath "${script_dir}/..")"
runner="${script_dir}/run_event_driven_one_shot_campaign.sh"

campaign="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/event_one_shot_nsteps512_constraint_schedule_$(date +%Y%m%d_%H%M%S)}"
start_episode="${RACER_QWEN8_START_EPISODE:-1}"
resume="${RACER_QWEN8_RESUME:-}"

if ! [[ "${start_episode}" =~ ^[1-9][0-9]*$ ]] ||
   (( start_episode > 50 )); then
  printf 'RACER_QWEN8_START_EPISODE must be in [1, 50].\n' >&2
  exit 2
fi

if (( start_episode == 1 )); then
  if [[ -e "${campaign}" ]]; then
    printf 'Refusing to overwrite an existing campaign: %s\n' "${campaign}" >&2
    exit 2
  fi
  if [[ -n "${resume}" ]]; then
    printf 'Episode 1 must start from scratch without RACER_QWEN8_RESUME.\n' >&2
    exit 2
  fi
  mkdir -p "${campaign}"
  printf 'episode_start\tepisode_end\tgamma_task\tn_steps\n1\t10\t0.14\t512\n11\t20\t0.12\t512\n21\t50\t0.10\t512\n' \
    >"${campaign}/constraint_schedule.tsv"
else
  if [[ ! -d "${campaign}" ]]; then
    printf 'Cannot resume missing campaign: %s\n' "${campaign}" >&2
    exit 2
  fi
  if [[ -z "${resume}" ]]; then
    resume="${campaign}/episode_$(printf '%02d' "$((start_episode - 1))")/training/llm_prior_crpo_final.zip"
  fi
  if [[ ! -f "${resume}" ]]; then
    printf 'Cannot resume without the preceding checkpoint: %s\n' "${resume}" >&2
    exit 2
  fi
fi

run_segment() {
  local segment_start="$1"
  local segment_end="$2"
  local gamma_task="$3"
  if (( start_episode > segment_end )); then
    return
  fi
  if (( segment_start < start_episode )); then
    segment_start="${start_episode}"
  fi

  RACER_QWEN8_CAMPAIGN="${campaign}" \
  RACER_QWEN8_EPISODES="${segment_end}" \
  RACER_QWEN8_START_EPISODE="${segment_start}" \
  RACER_QWEN8_RESUME="${resume}" \
  RACER_QWEN8_N_STEPS=512 \
  RACER_QWEN8_GAMMA_TASK="${gamma_task}" \
    "${runner}"

  resume="${campaign}/episode_$(printf '%02d' "${segment_end}")/training/llm_prior_crpo_final.zip"
  start_episode="$((segment_end + 1))"
}

run_segment 1 10 0.14
run_segment 11 20 0.12
run_segment 21 50 0.10

printf 'EVENT_ONE_SHOT_50EP_CONSTRAINT_SCHEDULE_OK output=%s\n' "${campaign}"
