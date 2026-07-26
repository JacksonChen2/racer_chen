#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
RESULT_FILE="${1:-/tmp/racer_mock_result.json}"
DURATION="${RACER_TEST_DURATION:-45}"
SCENARIO="${RACER_SCENARIO:-small}"
DRONE_COUNT="${RACER_DRONE_COUNT:-3}"
MINIMUM_COVERAGE="${RACER_MINIMUM_COVERAGE:-0.70}"

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
  backend:=mock \
  scenario:="${SCENARIO}" \
  drone_count:="${DRONE_COUNT}" \
  duration:="${DURATION}" \
  minimum_coverage:="${MINIMUM_COVERAGE}" \
  result_file:="${RESULT_FILE}" &
LAUNCH_PID=$!
cleanup() {
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + DURATION + 20))
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
