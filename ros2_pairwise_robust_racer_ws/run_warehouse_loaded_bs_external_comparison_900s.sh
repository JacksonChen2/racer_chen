#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
run_id="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
pair_dir="${RACER_COMPARISON_DIR:-${workspace_dir}/experiments/warehouse_loaded_bs_external_vs_distributed_900s_${run_id}}"
bs_dir="${pair_dir}/bs_round_robin"
distributed_dir="${pair_dir}/distributed"
mkdir -p "${bs_dir}" "${distributed_dir}"

export SIONNA_RUNTIME_DIR="${SIONNA_RUNTIME_DIR:-/home/jiazheng/RACER_warehouse_loaded_portable_20260805/ros2_original_fidelity_sionna_ws/.sionna_runtime}"
export RACER_COMMUNICATION_MODE=sionna
export RACER_FIDELITY_DURATION=900
export RACER_FIDELITY_DRONE_COUNT="${RACER_FIDELITY_DRONE_COUNT:-5}"
export RACER_FIDELITY_SCENARIO=warehouse_loaded
export RACER_FIDELITY_HEADLESS="${RACER_FIDELITY_HEADLESS:-1}"
export RACER_FIDELITY_VISUALIZE=0
export RACER_REQUIRE_COMPLETION=0
export RACER_STOP_ON_COMPLETION=1
export RACER_MAPPING_COVERAGE_TARGET=0
export RACER_PHYSICS_RATE_HZ=200
export RACER_SENSOR_RATE_HZ=15
export RACER_DEPTH_WIDTH=640
export RACER_DEPTH_HEIGHT=480
export RACER_CAMERA_RAY_BUDGET=19200
export RACER_RECORD_TRAJECTORY_HISTORY=1
export RACER_WALL_TIME_MULTIPLIER="${RACER_WALL_TIME_MULTIPLIER:-40}"
export RACER_WALL_TIME_GRACE_SECONDS="${RACER_WALL_TIME_GRACE_SECONDS:-600}"

export RACER_NETWORK_TOPOLOGY=bs_round_robin
export RACER_LAUNCH_PACKAGE=racer_bs_recovery
export RACER_LAUNCH_FILE=warehouse_sionna_with_bs_recovery.launch.py
export RACER_ALGORITHM_LABEL=racer_bs_external_recovery_v1
export RACER_LKH_DIR=/tmp/racer_bs_external_recovery_900s_lkh
export RACER_RESULT_DIR="${bs_dir}"
"${workspace_dir}/run_warehouse_simple_sionna.sh"

# No-BS baseline: keep the same 28 GHz Sionna channel and original distributed
# UAV broadcasts, but do not launch the BS supervisor or command mux.
export RACER_NETWORK_TOPOLOGY=distributed
export RACER_LAUNCH_PACKAGE=racer_sionna_comm
export RACER_LAUNCH_FILE=original_racer_warehouse_sionna.launch.py
export RACER_ALGORITHM_LABEL=original_racer_distributed_no_bs
export RACER_LKH_DIR=/tmp/racer_distributed_no_bs_900s_lkh
export RACER_RESULT_DIR="${distributed_dir}"
"${workspace_dir}/run_warehouse_simple_sionna.sh"

python3 - "${pair_dir}" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
paths = {
    "bs_round_robin": root / "bs_round_robin" / "warehouse_loaded_bs_round_robin_result.json",
    "distributed_no_bs": root / "distributed" / "warehouse_loaded_distributed_result.json",
}
summary = {
    "experiment": {
        "scene": "warehouse_loaded",
        "duration_limit_s": 900,
        "physics_rate_hz": 200,
        "depth_camera": "640x480@15Hz",
        "camera_ray_budget": 19200,
        "carrier_frequency_hz": 28.0e9,
    }
}
for name, path in paths.items():
    result = json.loads(path.read_text())
    metrics = result["metrics"]
    topology = result["communication"]["network_topology"]
    launch_path = path.parent / f"warehouse_loaded_{topology}_launch.log"
    launch_text = launch_path.read_text(errors="replace")
    summary[name] = {
        "result": str(path.resolve()),
        "passed": result["passed"],
        "algorithm": result["algorithm"],
        "elapsed_s": metrics["elapsed"],
        "stop_reason": metrics.get("stop_reason"),
        "joint_coverage": metrics["mapping_coverage_joint"],
        "path_lengths_m": metrics["path_lengths"],
        "collision_events": metrics["collision_events"],
        "finished_drone_ids": result["finished_drone_ids"],
        "no_path_count": launch_text.count("No path to next viewpoint"),
        "bs_recovery_start_count": launch_text.count("BS_RECOVERY_START"),
        "bs_recovery_release_count": launch_text.count("BS_RECOVERY_RELEASE"),
        "communication": result["communication"],
    }
(root / "comparison_summary.json").write_text(
    json.dumps(summary, indent=2, ensure_ascii=False) + "\n"
)
print(root / "comparison_summary.json")
PY
