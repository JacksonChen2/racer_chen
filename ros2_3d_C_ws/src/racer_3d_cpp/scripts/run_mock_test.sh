#!/usr/bin/env bash
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
search_dir="${script_dir}"
workspace_dir=""
while [[ "${search_dir}" != "/" ]]; do
  if [[ -f "${search_dir}/install/setup.bash" ]]; then
    workspace_dir="${search_dir}"
    break
  fi
  if [[ "$(basename "${search_dir}")" == "install" && -f "${search_dir}/setup.bash" ]]; then
    workspace_dir="$(dirname "${search_dir}")"
    break
  fi
  search_dir="$(dirname "${search_dir}")"
done
if [[ -z "${workspace_dir}" ]]; then
  printf 'Cannot locate this colcon workspace; build it first\n' >&2
  exit 2
fi
scenario="${RACER_3D_SCENARIO:-acceptance_15x9x2}"
duration="${RACER_3D_DURATION:-120}"
result_file="${1:-${workspace_dir}/src/racer_3d_cpp/test_results/MOCK_RESULT.json}"
duration_seconds="$(python3 -c 'import math,sys; v=float(sys.argv[1]); assert v > 0; print(math.ceil(v))' "${duration}")"

if [[ "${scenario}" == "warehouse_simple" ]]; then
  printf 'warehouse_simple requires the Isaac external-USD backend\n' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "${workspace_dir}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-143}"

agent_executable="${workspace_dir}/install/racer_3d_cpp/lib/racer_3d_cpp/racer_3d_cpp_agent"
mapfile -t existing_agent_pids < <(
  pgrep -f "^${agent_executable}( |$)" || true
)
if (( ${#existing_agent_pids[@]} > 0 )); then
  printf 'Refusing to start: stale RACER agents are still running: %s\n' \
    "${existing_agent_pids[*]}" >&2
  printf 'Stop the previous run with Ctrl+C before starting another one.\n' >&2
  exit 3
fi

rm -f -- "${result_file}"

setsid ros2 launch racer_3d_cpp swarm_3d_cpp.launch.py \
  backend:=mock scenario:="${scenario}" drone_count:=3 \
  duration:="${duration}" result_file:="${result_file}" &
launch_pid=$!

cleanup_launch() {
  exit_status=$?
  trap - EXIT INT TERM HUP
  if kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -INT -- "-${launch_pid}" 2>/dev/null || true
    for _ in {1..50}; do
      if ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -TERM -- "-${launch_pid}" 2>/dev/null || true
      sleep 1
    fi
    if kill -0 -- "-${launch_pid}" 2>/dev/null; then
      kill -KILL -- "-${launch_pid}" 2>/dev/null || true
    fi
  fi
  wait "${launch_pid}" 2>/dev/null || true
  exit "${exit_status}"
}
trap cleanup_launch EXIT
trap 'exit 130' INT TERM HUP

deadline=$((SECONDS + duration_seconds + 30))
while [[ ! -s "${result_file}" && ${SECONDS} -lt ${deadline} ]]; do sleep 1; done
python3 - "${result_file}" <<'PY'
import json
from pathlib import Path
import sys
p = Path(sys.argv[1])
if not p.is_file():
    raise SystemExit(f"missing result file: {p}")
r = json.loads(p.read_text())
print(json.dumps(r, indent=2, sort_keys=True))
raise SystemExit(0 if r.get("passed") else 1)
PY
