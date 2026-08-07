#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RACER_DIR="$(cd "${WORKSPACE_DIR}/.." && pwd)"
ISAAC_DIR="${ISAAC_SIM_DIR:-${ISAAC_SIM_ROOT:-${HOME}/isaacsim}}"
ISAAC_ROS2_DIR="${ISAAC_DIR}/exts/isaacsim.ros2.bridge/humble"
WAREHOUSE_BASE_USD="${WAREHOUSE_BASE_USD:-${RACER_DIR}/ros2_3d_py_ws/warehouse.usd}"
SIONNA_RUNTIME_PYTHON="${SIONNA_PYTHON:-python3}"
SIONNA_RUNTIME_PYTHONPATH="${SIONNA_PYTHONPATH:-${WORKSPACE_DIR}/.sionna_runtime}"
COMMUNICATION_MODE="${COMMUNICATION_MODE:-sionna_hybrid}"
DURATION="${DURATION:-900.0}"
DRONE_COUNT="${DRONE_COUNT:-3}"
RANDOM_SEED="${RANDOM_SEED:-42}"
RESULT_FILE="${RESULT_FILE:-${WORKSPACE_DIR}/results/warehouse_loaded_sionna.json}"
HEADLESS="${HEADLESS:-1}"

if ! command -v "${SIONNA_RUNTIME_PYTHON}" >/dev/null 2>&1; then
  echo "Sionna Python not found: ${SIONNA_RUNTIME_PYTHON}" >&2
  echo "Run ${WORKSPACE_DIR}/setup_sionna_env.sh first." >&2
  exit 2
fi
if [[ ! -d "${SIONNA_RUNTIME_PYTHONPATH}/sionna" ]]; then
  echo "Sionna runtime not found: ${SIONNA_RUNTIME_PYTHONPATH}" >&2
  echo "Run ${WORKSPACE_DIR}/setup_sionna_env.sh first." >&2
  exit 2
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ${WORKSPACE_DIR}/build.sh first." >&2
  exit 2
fi
if [[ ! -f "${RACER_DIR}/warehouse_loaded.usd" ]]; then
  echo "Missing ${RACER_DIR}/warehouse_loaded.usd" >&2
  exit 2
fi
if [[ ! -f "${WAREHOUSE_BASE_USD}" ]]; then
  echo "Missing base warehouse layer: ${WAREHOUSE_BASE_USD}" >&2
  echo "Set WAREHOUSE_BASE_USD to the warehouse.usd used by the overlay." >&2
  exit 2
fi
if [[ ! -f "${ISAAC_ROS2_DIR}/rclpy/rclpy/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so" ]]; then
  echo "Isaac Sim Humble Python 3.11 bridge not found: ${ISAAC_ROS2_DIR}" >&2
  exit 2
fi

mkdir -p "$(dirname "${RESULT_FILE}")" "${WORKSPACE_DIR}/logs"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export SIONNA_PYTHON="${SIONNA_RUNTIME_PYTHON}"
export SIONNA_PYTHONPATH="${SIONNA_RUNTIME_PYTHONPATH}"

ros2 launch racer_3d_sionna_comm warehouse_loaded_sionna.launch.py \
  backend:=isaac \
  drone_count:="${DRONE_COUNT}" \
  duration:="${DURATION}" \
  communication_mode:="${COMMUNICATION_MODE}" \
  require_sionna:=true \
  random_seed:="${RANDOM_SEED}" \
  result_file:="${RESULT_FILE}" \
  >"${WORKSPACE_DIR}/logs/ros2_hybrid.log" 2>&1 &
ROS_LAUNCH_PID=$!

cleanup() {
  if kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    kill "${ROS_LAUNCH_PID}" 2>/dev/null || true
    wait "${ROS_LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ISAAC_ARGUMENTS=(
  "${WORKSPACE_DIR}/src/racer_3d_sionna_comm/isaac_sim/isaac_sim_racer_3d_cpp.py"
  --duration "${DURATION}"
  --drone-count "${DRONE_COUNT}"
  --scenario warehouse_loaded
  --scene-usd "${RACER_DIR}/warehouse_loaded.usd"
  --base-scene-usd "${WAREHOUSE_BASE_USD}"
  --vehicle-model racer_so3
  --external-swarm-safety disabled
)
if [[ "${HEADLESS}" == "1" ]]; then
  ISAAC_ARGUMENTS+=(--headless)
else
  ISAAC_ARGUMENTS+=(--visualize-exploration)
fi

set +e
env \
  ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  PYTHONPATH="${ISAAC_ROS2_DIR}/rclpy" \
  LD_LIBRARY_PATH="${ISAAC_ROS2_DIR}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" \
  "${ISAAC_DIR}/python.sh" "${ISAAC_ARGUMENTS[@]}" \
  2>&1 | tee "${WORKSPACE_DIR}/logs/isaac_hybrid.log"
ISAAC_STATUS=${PIPESTATUS[0]}
set -e

if (( ISAAC_STATUS != 0 )); then
  exit "${ISAAC_STATUS}"
fi
for _ in $(seq 1 40); do
  if ! kill -0 "${ROS_LAUNCH_PID}" 2>/dev/null; then
    wait "${ROS_LAUNCH_PID}" || true
    exit 0
  fi
  sleep 0.25
done
echo "ROS monitor did not exit after Isaac stopped; shutting it down." >&2
