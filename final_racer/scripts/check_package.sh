#!/usr/bin/env bash
set -euo pipefail

final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
result="${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json"

bash -n "${final_root}/scripts/build.sh"
bash -n "${final_root}/scripts/run_perfect_75_2913.sh"
bash -n "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh"
bash -n "${final_root}/scripts/run_pair_perfect_vs_sionna_10uav_5sites_300s.sh"
test -f "${final_root}/ros2_ws/install/setup.bash"
test -f "${final_root}/passive_metrics_overlay_ws/install/setup.bash"
test -x "${final_root}/ros2_ws/build/racer_original_core/racer_original_exploration_node"
test -x "${final_root}/passive_metrics_overlay_ws/install/racer_sionna_comm/lib/racer_sionna_comm/racer_sionna_communication_proxy"
test -x "${final_root}/sionna_distributed_overlay_ws/install/racer_sionna_comm/lib/racer_sionna_comm/racer_sionna_communication_proxy"

python3 - "${result}" <<'PY'
import json
import math
from pathlib import Path
import sys

result = json.loads(Path(sys.argv[1]).read_text())
metrics = result["metrics"]
statistics = result["communication"]["statistics"]
errors = []
if not math.isclose(float(metrics["mapping_coverage_joint"]), 0.752913131313, abs_tol=1e-12):
    errors.append("reference coverage is not 0.752913131313")
if not math.isclose(float(metrics["elapsed"]), 300.01, abs_tol=0.02):
    errors.append("reference duration is not 300 seconds")
if result["communication"]["mode"] != "ideal":
    errors.append("reference communication is not ideal")
if statistics["attempted_packets"] != statistics["delivered_packets"]:
    errors.append("reference ideal communication was not lossless")
if len(statistics["task_quality_history"]) != 3001:
    errors.append("reference does not contain 3001 passive metric samples")
if metrics["start_positions"] != [
    [-20.75, 26, 0.75], [-19.25, 26, 0.75],
    [-18.75, 5, 0.75], [-17.25, 5, 0.75],
    [-11.75, 14.25, 0.75], [-11.75, 15.75, 0.75],
    [-0.75, 27, 0.75], [0.75, 27, 0.75],
    [-3.75, 5, 0.75], [-2.25, 5, 0.75],
]:
    errors.append("reference takeoff positions changed")
if errors:
    raise SystemExit("\n".join(f"FINAL_RACER_CHECK_ERROR {item}" for item in errors))
print(
    "FINAL_RACER_REFERENCE_OK"
    f" coverage={100.0 * metrics['mapping_coverage_joint']:.4f}%"
    f" elapsed={metrics['elapsed']:.2f}s"
    f" passive_samples={len(statistics['task_quality_history'])}"
)
PY

if rg -q -- '--fixed-exploration-horizon' \
    "${final_root}/reference_result/formal_300s/warehouse_full_distributed_isaac.log"; then
  printf 'FINAL_RACER_CHECK_ERROR reference unexpectedly used fixed exploration horizon\n' >&2
  exit 1
fi

if rg -n '/racer_chen/hybrid_communication_racer( copy)?/' \
    "${final_root}/scripts" \
    "${final_root}/ros2_ws/run_warehouse_simple_sionna.sh" \
    "${final_root}/ros2_ws/config" >/dev/null; then
  printf 'FINAL_RACER_CHECK_ERROR runnable scripts still depend on the old hybrid directory\n' >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${final_root}/ros2_ws/install/setup.bash"
base_core_prefix="$(ros2 pkg prefix racer_original_core)"
source "${final_root}/passive_metrics_overlay_ws/install/setup.bash"
overlay_comm_prefix="$(ros2 pkg prefix racer_sionna_comm)"
source "${final_root}/sionna_distributed_overlay_ws/install/setup.bash"
sionna_comm_prefix="$(ros2 pkg prefix racer_sionna_comm)"
set -u

case "${base_core_prefix}" in
  "${final_root}"/*) ;;
  *) printf 'FINAL_RACER_CHECK_ERROR core resolves outside final_racer: %s\n' "${base_core_prefix}" >&2; exit 1 ;;
esac
case "${overlay_comm_prefix}" in
  "${final_root}"/*) ;;
  *) printf 'FINAL_RACER_CHECK_ERROR overlay resolves outside final_racer: %s\n' "${overlay_comm_prefix}" >&2; exit 1 ;;
esac
case "${sionna_comm_prefix}" in
  "${final_root}"/*) ;;
  *) printf 'FINAL_RACER_CHECK_ERROR Sionna overlay resolves outside final_racer: %s\n' "${sionna_comm_prefix}" >&2; exit 1 ;;
esac

printf 'FINAL_RACER_INDEPENDENCE_OK core=%s passive_overlay=%s sionna_overlay=%s\n' \
  "${base_core_prefix}" "${overlay_comm_prefix}" "${sionna_comm_prefix}"
