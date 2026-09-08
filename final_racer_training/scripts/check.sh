#!/usr/bin/env bash
set -euo pipefail

training_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
final_root="${RACER_FINAL_RACER_ROOT:-$(realpath "${training_root}/../final_racer")}" 
python_env="/home/jiazheng/ai_envs/racer-crpo/bin/python"
config="${training_root}/config/qwen8b_fp8_four_process.yaml"

required=(
  "${final_root}/ros2_ws/install/setup.bash"
  "${final_root}/sionna_distributed_overlay_ws/install/setup.bash"
  "${training_root}/training_overlay_ws/install/setup.bash"
  "${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json"
  "${final_root}/data/gt_occupied_voxels.txt"
  "/home/jiazheng/ai_models/Qwen3-8B-FP8"
  "${config}"
)
for path in "${required[@]}"; do
  if [[ ! -e "${path}" ]]; then
    printf 'CHECK_ERROR missing=%s\n' "${path}" >&2
    exit 2
  fi
done

bash -n "${training_root}/runtime/run_final_racer_training.sh"
bash -n "${training_root}/scripts/run_training_campaign.sh"
bash -n "${training_root}/scripts/run_four_process_campaign.sh"
bash -n "${training_root}/scripts/run_qwen8b_fp8_training.sh"

rg -q 'Shared-memory four-process mode forbids RACER_RL_SYNC_ENABLED=true' \
  "${training_root}/runtime/run_final_racer_training.sh"
rg -q 'four-process shared-memory mode cannot enable either Isaac freeze gate' \
  "${training_root}/training_overlay_ws/src/racer_isaac_adapter/isaac_sim/original_racer_isaac.py"

if rg -n 'hybrid_communication_racer|qwen14b_crpo|reproduce_7565' \
  --glob '!check.sh' \
  "${training_root}/config" "${training_root}/runtime" \
  "${training_root}/scripts" "${training_root}/agentic_crpo/agentic_crpo"; then
  printf 'CHECK_ERROR legacy runtime dependency found\n' >&2
  exit 3
fi

set +u
source /opt/ros/humble/setup.bash
source "${final_root}/ros2_ws/install/setup.bash"
source "${final_root}/sionna_distributed_overlay_ws/install/setup.bash"
source "${training_root}/training_overlay_ws/install/setup.bash"
set -u

core_prefix="$(ros2 pkg prefix racer_original_core)"
comm_prefix="$(ros2 pkg prefix racer_sionna_comm)"
adapter_prefix="$(ros2 pkg prefix racer_isaac_adapter)"
[[ "${core_prefix}" == "${final_root}/ros2_ws/install/racer_original_core" ]]
[[ "${comm_prefix}" == "${training_root}/training_overlay_ws/install/racer_sionna_comm" ]]
[[ "${adapter_prefix}" == "${training_root}/training_overlay_ws/install/racer_isaac_adapter" ]]

PYTHONPATH="${training_root}/agentic_crpo" "${python_env}" - \
  "${config}" \
  "${final_root}/reference_result/formal_300s/warehouse_full_distributed_result.json" <<'PY'
import json
from pathlib import Path
import sys

import vllm
from agentic_crpo.config import load_config

config = load_config(Path(sys.argv[1]))
reference = json.loads(Path(sys.argv[2]).read_text())
assert config["qwen"]["backend"] == "vllm"
assert config["qwen"]["vllm"]["quantization"] == "fp8"
assert config["qwen"]["model_path"] == "/home/jiazheng/ai_models/Qwen3-8B-FP8"
assert config["environment"]["backend"]["kind"] == "shared_memory"
assert config["qwen"]["single_gpu_pause"]["enabled"] is False
assert config["crpo"]["ppo"]["n_steps"] == 384
assert reference["metrics"]["mapping_coverage_joint"] == 0.752913131313
assert len(reference["communication"]["statistics"]["task_quality_history"]) == 3001
print(f"PYTHON_OK vllm={vllm.__version__} reference_coverage=75.2913%")
PY

PYTHONPATH="${training_root}/agentic_crpo" "${python_env}" -m pytest \
  -q -p no:cacheprovider \
  "${training_root}/agentic_crpo/tests"

printf 'CHECK_OK core=%s comm=%s adapter=%s\n' \
  "${core_prefix}" "${comm_prefix}" "${adapter_prefix}"
