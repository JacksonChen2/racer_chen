#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
ISAAC_ROOT="${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}"
RESULT_FILE="${1:-/tmp/racer_isaac_result.json}"
DURATION="${RACER_TEST_DURATION:-45}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-47}"
export ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  printf 'Build the workspace first: colcon build --symlink-install\n' >&2
  exit 2
fi
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
if [[ -f "${RESULT_FILE}" ]]; then
  rm -- "${RESULT_FILE}"
fi

ros2 launch racer_ros2 swarm_exploration.launch.py \
  backend:=isaac \
  duration:="${DURATION}" \
  result_file:="${RESULT_FILE}" &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
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
  LD_LIBRARY_PATH="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/humble/lib" \
  timeout "$((DURATION + 45))" \
  "${ISAAC_ROOT}/python.sh" \
  "${PACKAGE_DIR}/isaac_sim/isaac_sim_racer.py" \
  --headless \
  --duration "${DURATION}" \
  --drone-count 3

deadline=$((SECONDS + 15))
while [[ ! -s "${RESULT_FILE}" && ${SECONDS} -lt ${deadline} ]]; do
  sleep 1
done

python3 - "${RESULT_FILE}" <<'PY'
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
