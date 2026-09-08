#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
training_campaign="${RACER_WAIT_TRAINING_CAMPAIGN:?set RACER_WAIT_TRAINING_CAMPAIGN}"
training_pid="${RACER_WAIT_TRAINING_PID:?set RACER_WAIT_TRAINING_PID}"
suite="${RACER_SUITE_DIR:?set RACER_SUITE_DIR}"
campaign_log="${training_campaign}/campaign.log"
expected_checkpoint="${training_campaign}/episode_40/training/crpo_final.zip"

if ! [[ "${training_pid}" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Invalid training PID: %s\n' "${training_pid}" >&2
  exit 2
fi
if [[ ! -r "/proc/${training_pid}/stat" ]]; then
  printf 'Training process is not running: %s\n' "${training_pid}" >&2
  exit 2
fi

mkdir -p "${suite}"
expected_start_ticks="$(awk '{print $22}' "/proc/${training_pid}/stat")"
printf 'queued training_pid=%s training_start_ticks=%s time=%s\n' \
  "${training_pid}" "${expected_start_ticks}" "$(date --iso-8601=seconds)" \
  >"${suite}/queue_state.txt"

while [[ -r "/proc/${training_pid}/stat" ]]; do
  current_start_ticks="$(awk '{print $22}' "/proc/${training_pid}/stat" 2>/dev/null || true)"
  if [[ "${current_start_ticks}" != "${expected_start_ticks}" ]]; then
    break
  fi
  sleep 20
done

if [[ ! -f "${expected_checkpoint}" ]] ||
   ! rg -q 'FOUR_PROCESS_CAMPAIGN_OK .*episodes=40' "${campaign_log}"; then
  printf 'blocked:training_did_not_complete time=%s checkpoint=%s\n' \
    "$(date --iso-8601=seconds)" "${expected_checkpoint}" \
    >"${suite}/queue_state.txt"
  exit 3
fi

printf 'running_pair time=%s checkpoint=%s\n' \
  "$(date --iso-8601=seconds)" "${expected_checkpoint}" \
  >"${suite}/queue_state.txt"
set +e
RACER_SUITE_DIR="${suite}" RACER_EXPERIMENT_DURATION=900 \
  "${final_root}/scripts/run_pair_perfect_vs_sionna_10uav_new_starts_900s.sh"
status=$?
set -e
printf 'finished status=%s time=%s\n' \
  "${status}" "$(date --iso-8601=seconds)" >"${suite}/queue_state.txt"
exit "${status}"
