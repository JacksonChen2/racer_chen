#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
scenario="${RACER_3D_SCENARIO:-acceptance_15x9x2}"
if [[ "${scenario}" == "warehouse_simple" ]]; then
  printf 'warehouse_simple requires the Isaac external-USD backend\n' >&2
  exit 2
fi
result_name="MOCK_${scenario#acceptance_}_RESULT.json"
result_file="${1:-${workspace_dir}/src/racer_3d/test_results/${result_name^^}}"
duration="${RACER_3D_DURATION:-120}"
duration_seconds="${duration%%.*}"

source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"

if [[ -f "${result_file}" ]]; then
  rm -- "${result_file}"
fi

ros2 launch racer_3d swarm_3d.launch.py \
  backend:=mock \
  scenario:="${scenario}" \
  drone_count:=3 \
  duration:="${duration}" \
  result_file:="${result_file}" &
launch_pid=$!
cleanup() {
  kill "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + duration_seconds + 20))
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
