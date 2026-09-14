#!/usr/bin/env bash
set -euo pipefail

# Reusable event-driven schedule:
#   episodes  1-10: Gamma_task=0.14
#   episodes 11-20: Gamma_task=0.12
#   episodes 21-N : Gamma_task=0.10
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
training_root="$(realpath "${script_dir}/..")"
runner="${script_dir}/run_event_driven_one_shot_campaign.sh"

total_episodes="${RACER_QWEN8_TOTAL_EPISODES:-40}"
campaign="${RACER_QWEN8_CAMPAIGN:-${training_root}/results/event_one_shot_nsteps512_constraint_schedule_$(date +%Y%m%d_%H%M%S)}"
start_episode="${RACER_QWEN8_START_EPISODE:-1}"
resume="${RACER_QWEN8_RESUME:-}"

if ! [[ "${total_episodes}" =~ ^[1-9][0-9]*$ ]] ||
   (( total_episodes > 100 )); then
  printf 'RACER_QWEN8_TOTAL_EPISODES must be in [1, 100].\n' >&2
  exit 2
fi
if ! [[ "${start_episode}" =~ ^[1-9][0-9]*$ ]] ||
   (( start_episode > total_episodes )); then
  printf 'RACER_QWEN8_START_EPISODE must be in [1, %d].\n' \
    "${total_episodes}" >&2
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
  {
    printf 'episode_start\tepisode_end\tgamma_task\tn_steps\n'
    if (( total_episodes <= 10 )); then
      printf '1\t%d\t0.14\t512\n' "${total_episodes}"
    elif (( total_episodes <= 20 )); then
      printf '1\t10\t0.14\t512\n'
      printf '11\t%d\t0.12\t512\n' "${total_episodes}"
    else
      printf '1\t10\t0.14\t512\n'
      printf '11\t20\t0.12\t512\n'
      printf '21\t%d\t0.10\t512\n' "${total_episodes}"
    fi
  } >"${campaign}/constraint_schedule.tsv"
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
  if (( segment_start > total_episodes || start_episode > segment_end )); then
    return
  fi
  if (( segment_end > total_episodes )); then
    segment_end="${total_episodes}"
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
run_segment 21 "${total_episodes}" 0.10

printf 'EVENT_ONE_SHOT_CONSTRAINT_SCHEDULE_OK output=%s episodes=%s\n' \
  "${campaign}" "${total_episodes}"
