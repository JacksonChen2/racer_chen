#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
search_dir="${script_dir}"
workspace_dir=""
while [[ "${search_dir}" != "/" ]]; do
  if [[ -f "${search_dir}/install/setup.bash" ]]; then
    workspace_dir="${search_dir}"
    break
  fi
  search_dir="$(dirname "${search_dir}")"
done
if [[ -z "${workspace_dir}" ]]; then
  printf 'Cannot locate ros2_original_fidelity_ws; build the workspace first.\n' >&2
  exit 2
fi

repo_root="$(realpath "${workspace_dir}/..")"
isaac_root="${ISAAC_SIM_ROOT:-/home/jiazheng/software/isaacsim}"
scene_usd="${repo_root}/ros2_3d_py_ws/warehouse_simple.usd"
vehicle_usd="${repo_root}/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"
replay_script="${workspace_dir}/src/racer_isaac_adapter/isaac_sim/original_racer_trajectory_replay.py"
duration="${RACER_REPLAY_RECORD_DURATION:-900}"
drone_count="${RACER_FIDELITY_DRONE_COUNT:-5}"
record_hz="${RACER_TRAJECTORY_RECORD_HZ:-60}"
replay_fps="${RACER_REPLAY_FPS:-60}"
follow_drone="${RACER_REPLAY_FOLLOW_DRONE:--1}"
run_stamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RACER_REPLAY_RUN_DIR:-${workspace_dir}/validation/record_replay_${run_stamp}}"
trajectory="${run_dir}/warehouse_simple_original_racer_trajectory_${run_stamp}.npz"

if [[ ! -x "${isaac_root}/python.sh" ]]; then
  printf 'Isaac Sim Python does not exist: %s\n' "${isaac_root}/python.sh" >&2
  exit 2
fi
if [[ ! -f "${scene_usd}" || ! -f "${vehicle_usd}" ]]; then
  printf 'Warehouse Simple scene or Crazyflie/RACER vehicle USD is missing.\n' >&2
  exit 2
fi
mkdir -p "${run_dir}"

printf 'Phase 1/2: headless original RACER + 1 kHz PhysX dynamics\n'
printf '  scene: %s\n' "${scene_usd}"
printf '  vehicle: %s\n' "${vehicle_usd}"
printf '  trajectory: %s\n' "${trajectory}"
set +e
env \
  ISAAC_SIM_ROOT="${isaac_root}" \
  RACER_FIDELITY_SCENARIO=warehouse_simple \
  RACER_SCENE_USD="${scene_usd}" \
  RACER_VEHICLE_USD="${vehicle_usd}" \
  RACER_FIDELITY_DURATION="${duration}" \
  RACER_FIDELITY_DRONE_COUNT="${drone_count}" \
  RACER_FIDELITY_HEADLESS=1 \
  RACER_FIDELITY_VISUALIZE=0 \
  RACER_REQUIRE_COMPLETION="${RACER_REQUIRE_COMPLETION:-0}" \
  RACER_STOP_ON_COMPLETION="${RACER_STOP_ON_COMPLETION:-1}" \
  RACER_WALL_TIMEOUT_FACTOR="${RACER_WALL_TIMEOUT_FACTOR:-120}" \
  RACER_WALL_TIMEOUT_GRACE="${RACER_WALL_TIMEOUT_GRACE:-600}" \
  RACER_RESULT_DIR="${run_dir}" \
  RACER_TRAJECTORY_OUTPUT="${trajectory}" \
  RACER_TRAJECTORY_RECORD_HZ="${record_hz}" \
  "${script_dir}/run_warehouse_simple.sh"
record_status=$?
set -e

if [[ ! -s "${trajectory}" ]]; then
  printf 'Headless phase did not create a trajectory (status %s).\n' \
    "${record_status}" >&2
  exit "${record_status}"
fi
python3 - "${trajectory}" "${duration}" "${drone_count}" <<'PY'
import json
from pathlib import Path
import sys

import numpy as np

path = Path(sys.argv[1])
requested_duration = float(sys.argv[2])
requested_drones = int(sys.argv[3])
with np.load(path, allow_pickle=False) as archive:
    metadata = json.loads(str(archive["metadata_json"].item()))
    times = np.asarray(archive["times"], dtype=float)
    positions = np.asarray(archive["positions"], dtype=float)
    motor_rpm = np.asarray(archive["motor_rpm"], dtype=float)
if metadata.get("scenario") != "warehouse_simple":
    raise SystemExit("recorded trajectory is not Warehouse Simple")
if not str(metadata.get("vehicle_usd", "")).endswith(
    "/crazyflie_with_racer_dynamics.usd"
):
    raise SystemExit("recorded trajectory uses the wrong vehicle USD")
if positions.shape != (len(times), requested_drones, 3):
    raise SystemExit(f"invalid position shape: {positions.shape}")
if motor_rpm.shape != (len(times), requested_drones, 4):
    raise SystemExit(f"invalid motor RPM shape: {motor_rpm.shape}")
if len(times) < 2 or not np.all(np.diff(times) > 0.0):
    raise SystemExit("trajectory timestamps are not strictly increasing")
duration_tolerance = max(0.05, 2.0 / float(metadata["sample_rate_hz"]))
deadline_reached = abs(float(times[-1]) - requested_duration) <= duration_tolerance
mission_complete = metadata.get("stop_reason") == "original_fsm_completion"
if not deadline_reached and not mission_complete:
    raise SystemExit(
        f"trajectory ended unexpectedly at {times[-1]:.3f}s "
        f"with stop_reason={metadata.get('stop_reason')!r}"
    )
print(
    "Trajectory validation passed: "
    f"{len(times)} samples, {requested_drones} drones, "
    f"{times[-1]:.3f}s, stop={metadata.get('stop_reason')}, "
    f"RPM {motor_rpm.min():.0f}..{motor_rpm.max():.0f}"
)
PY
if [[ "${record_status}" != "0" ]]; then
  printf 'Headless acceptance status was %s; replaying the saved test data.\n' \
    "${record_status}" >&2
fi

printf 'Phase 2/2: independent wall-clock 1x Isaac Sim replay\n'
printf '  mouse camera/zoom no longer shares the RACER simulation loop\n'
replay_args=(
  --trajectory "${trajectory}"
  --scene-usd "${scene_usd}"
  --vehicle-usd "${vehicle_usd}"
  --fps "${replay_fps}"
  --speed 1.0
  --follow-drone "${follow_drone}"
  --propeller-visual-hz "${replay_fps}"
)
if [[ "${RACER_REPLAY_LOOP:-0}" == "1" ]]; then
  replay_args+=(--loop)
fi

env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  "${isaac_root}/python.sh" "${replay_script}" "${replay_args[@]}"
