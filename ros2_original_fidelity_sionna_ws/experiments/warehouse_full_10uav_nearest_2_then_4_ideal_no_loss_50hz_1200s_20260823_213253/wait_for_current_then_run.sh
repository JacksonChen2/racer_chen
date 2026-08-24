#!/usr/bin/env bash
set -euo pipefail

current_suite="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws/experiments/warehouse_full_10uav_center_racer_layout_ideal_vs_sionna_no_retries_50hz_1200s_20260823_203213"
current_script="${current_suite}/run_comparison.sh"
current_supervisor_pid=3090459
queued_suite="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws/experiments/warehouse_full_10uav_nearest_2_then_4_ideal_no_loss_50hz_1200s_20260823_213253"

printf '%s\n' "$$" >"${queued_suite}/queue.pid"
printf '%s\n' "waiting_for_current_suite" >"${queued_suite}/queue_state.txt"
printf 'QUEUE_START time=%s dependency_pid=%s\n' \
  "$(date --iso-8601=seconds)" "${current_supervisor_pid}"

while kill -0 "${current_supervisor_pid}" 2>/dev/null; do
  cmdline="$(tr '\0' ' ' <"/proc/${current_supervisor_pid}/cmdline" 2>/dev/null || true)"
  if [[ "${cmdline}" != *"${current_script}"* ]]; then
    printf 'QUEUE_BLOCKED reason=dependency_pid_identity_changed time=%s\n' \
      "$(date --iso-8601=seconds)"
    printf '%s\n' "blocked:dependency_pid_identity_changed" >"${queued_suite}/queue_state.txt"
    exit 1
  fi
  sleep 30
done

if [[ "$(<"${current_suite}/active_case.txt")" != "complete" ]]; then
  printf 'QUEUE_BLOCKED reason=current_suite_not_complete state=%s time=%s\n' \
    "$(<"${current_suite}/active_case.txt")" "$(date --iso-8601=seconds)"
  printf '%s\n' "blocked:current_suite_not_complete" >"${queued_suite}/queue_state.txt"
  exit 1
fi

set +e
python3 - "${current_suite}" >"${queued_suite}/dependency_validation.log" 2>&1 <<'PY'
import json
from pathlib import Path
import re
import sys

suite = Path(sys.argv[1])
cases = (
    "ideal_no_loss_original_broadcast",
    "sionna_distributed_original_comm_no_link_retries",
)
errors = []
for case_name in cases:
    case_dir = suite / case_name
    isaac_log = case_dir / "warehouse_full_distributed_isaac.log"
    launch_log = case_dir / "warehouse_full_distributed_launch.log"
    metrics = None
    if isaac_log.is_file():
        prefix = "RACER_3D_ISAAC_RESULT "
        for line in isaac_log.read_text(errors="replace").splitlines():
            if line.startswith(prefix):
                metrics = json.loads(line[len(prefix):])
    if metrics is None:
        errors.append(f"{case_name}: missing Isaac result")
        continue
    elapsed = float(metrics.get("elapsed", 0.0))
    collisions = int(metrics.get("collision_events", -1))
    if elapsed < 1199.0:
        errors.append(f"{case_name}: elapsed={elapsed}")
    if collisions != 0:
        errors.append(f"{case_name}: collisions={collisions}")
    launch_text = launch_log.read_text(errors="replace") if launch_log.is_file() else ""
    crashes = len(re.findall(r"process has died|exit code -11|Segmentation", launch_text))
    if crashes:
        errors.append(f"{case_name}: process_crashes={crashes}")
    print(
        f"DEPENDENCY_CASE_OK case={case_name} elapsed={elapsed} "
        f"collisions={collisions} process_crashes={crashes}"
    )
if errors:
    for error in errors:
        print(f"DEPENDENCY_VALIDATION_ERROR {error}", file=sys.stderr)
    raise SystemExit(1)
print("DEPENDENCY_VALIDATION_OK user_authorized_scientific_acceptance_override=true")
PY
dependency_status=$?
set -e
if (( dependency_status != 0 )); then
  printf 'QUEUE_BLOCKED reason=dependency_integrity_validation_failed time=%s\n' \
    "$(date --iso-8601=seconds)"
  printf '%s\n' "blocked:dependency_integrity_validation_failed" \
    >"${queued_suite}/queue_state.txt"
  exit 1
fi

printf 'DEPENDENCY_OK time=%s\n' "$(date --iso-8601=seconds)"

printf '%s\n' "building_and_testing_mixed_transport" >"${queued_suite}/queue_state.txt"
set +e
"${queued_suite}/build_and_test_mixed_transport.sh" \
  >"${queued_suite}/build_and_test.log" 2>&1
build_status=$?
set -e
printf '%s\n' "${build_status}" >"${queued_suite}/build_and_test_exit_status.txt"
if (( build_status != 0 )); then
  printf 'QUEUE_BLOCKED reason=mixed_transport_build_or_test_failed status=%s time=%s\n' \
    "${build_status}" "$(date --iso-8601=seconds)"
  printf 'blocked:mixed_transport_build_or_test_failed:%s\n' "${build_status}" \
    >"${queued_suite}/queue_state.txt"
  exit 1
fi

printf '%s\n' "running_mixed_transport_suite" >"${queued_suite}/queue_state.txt"
set +e
"${queued_suite}/run_nearest_2_then_4.sh"
status=$?
set -e

printf '%s\n' "${status}" >"${queued_suite}/queue_exit_status.txt"
if (( status == 0 )); then
  printf '%s\n' "complete" >"${queued_suite}/queue_state.txt"
else
  printf 'failed:%s\n' "${status}" >"${queued_suite}/queue_state.txt"
fi
printf 'QUEUE_END status=%s time=%s\n' "${status}" "$(date --iso-8601=seconds)"
exit "${status}"
