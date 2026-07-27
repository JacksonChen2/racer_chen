#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd "${script_dir}/.." && pwd)"
workspace_dir="$(cd "${package_dir}/../.." && pwd)"
isaac_root="${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}"
scenario="${RACER_3D_SCENARIO:-acceptance_15x9x2}"
scene_usd="${RACER_3D_SCENE_USD:-}"
if [[ -z "${scene_usd}" && "${scenario}" == "warehouse_simple" ]]; then
  scene_usd="${workspace_dir}/warehouse_simple.usd"
fi
scene_arguments=()
if [[ -n "${scene_usd}" ]]; then
  if [[ ! -f "${scene_usd}" ]]; then
    printf 'External USD does not exist: %s\n' "${scene_usd}" >&2
    exit 2
  fi
  scene_arguments=(--scene-usd "${scene_usd}")
fi
result_name="ISAAC_${scenario#acceptance_}_RESULT.json"
result_file="${1:-${package_dir}/test_results/${result_name^^}}"
duration="${RACER_3D_DURATION:-120}"
duration_seconds="${duration%%.*}"
timeout_seconds="$((duration_seconds * 2 + 180))"
drone_count="${RACER_3D_DRONE_COUNT:-3}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-63}"
export ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  printf 'Build the workspace first: colcon build --symlink-install\n' >&2
  exit 2
fi
source "${workspace_dir}/install/setup.bash"
set -u
if [[ -f "${result_file}" ]]; then
  rm -- "${result_file}"
fi

ros2 launch racer_3d swarm_3d.launch.py \
  backend:=isaac \
  scenario:="${scenario}" \
  drone_count:="${drone_count}" \
  duration:="${duration}" \
  result_file:="${result_file}" &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2
env \
  -u AMENT_PREFIX_PATH \
  -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH \
  -u PYTHONPATH \
  ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_root}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "${timeout_seconds}" \
  "${isaac_root}/python.sh" \
  "${package_dir}/isaac_sim/isaac_sim_racer_3d.py" \
  --headless \
  --scenario "${scenario}" \
  "${scene_arguments[@]}" \
  --duration "${duration}" \
  --drone-count "${drone_count}" \
  --diagnostics

deadline=$((SECONDS + 20))
while [[ ! -s "${result_file}" && ${SECONDS} -lt ${deadline} ]]; do
  sleep 1
done

python3 - "${result_file}" <<'PY'
import json
from pathlib import Path
import sys

path = Path(sys.argv[1])
if not path.exists():
    raise SystemExit(f"missing result file: {path}")
result = json.loads(path.read_text(encoding="utf-8"))
print(json.dumps(result, indent=2, sort_keys=True))
raise SystemExit(0 if result.get("passed") else 1)
PY
