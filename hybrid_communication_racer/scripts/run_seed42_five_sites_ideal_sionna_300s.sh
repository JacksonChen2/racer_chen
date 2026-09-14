#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
suite="${RACER_SUITE_DIR:-${workspace}/experiments/five_sites_same_aisle_10uav_300s_seed42_baselines_${run_id}}"
sionna_runtime="${RACER_SIONNA_RUNTIME_DIR:-${workspace}/../ros2_original_fidelity_sionna_ws/.sionna_runtime}"
ideal_suite="${suite}/ideal_300s"
sionna_suite="${suite}/sionna_300s"
active_child=""

mkdir -p "${suite}"
printf '%s\n' "$$" >"${suite}/supervisor.pid"
printf '%s\n' "starting" >"${suite}/run_state.txt"

stop_experiment() {
  trap - INT TERM HUP
  if [[ -n "${active_child}" ]]; then
    kill -TERM "${active_child}" 2>/dev/null || true
    wait "${active_child}" 2>/dev/null || true
  fi
  printf '%s\n' "stopped" >"${suite}/run_state.txt"
  exit 130
}
trap stop_experiment INT TERM HUP

run_baseline() {
  local label="$1"
  local script="$2"
  local output_dir="$3"
  local log_file="$4"

  printf 'running:%s\n' "${label}" >"${suite}/run_state.txt"
  set +e
  env \
    RACER_RUN_ID="${run_id}_${label}" \
    RACER_RANDOM_SEED=42 \
    RACER_SUITE_DIR="${output_dir}" \
    RACER_SIONNA_RUNTIME_DIR="${sionna_runtime}" \
    bash "${script}" >"${log_file}" 2>&1 &
  active_child=$!
  printf '%s\n' "${active_child}" >"${suite}/${label}.pid"
  wait "${active_child}"
  local status=$?
  active_child=""
  printf '%s\n' "${status}" >"${suite}/${label}_exit_status.txt"
  return "${status}"
}

set +e
run_baseline \
  ideal \
  "${workspace}/run_warehouse_full_10uav_five_sites_ideal_takeoff_fixed_300s.sh" \
  "${ideal_suite}" \
  "${suite}/ideal_supervisor.log"
ideal_status=$?
set -e

printf '%s\n' "cleanup_between_cases" >"${suite}/run_state.txt"
sleep 30

set +e
run_baseline \
  sionna \
  "${workspace}/run_warehouse_full_10uav_five_sites_sionna_300s.sh" \
  "${sionna_suite}" \
  "${suite}/sionna_supervisor.log"
sionna_status=$?
set -e

if [[ "${ideal_status}" -eq 0 && "${sionna_status}" -eq 0 ]]; then
  printf '%s\n' "completed" >"${suite}/run_state.txt"
else
  printf 'completed_with_error ideal=%s sionna=%s\n' \
    "${ideal_status}" "${sionna_status}" >"${suite}/run_state.txt"
fi

exit "$((ideal_status != 0 || sionna_status != 0))"
