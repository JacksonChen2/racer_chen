#!/usr/bin/env bash
set -euo pipefail
final_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RACER_RUN_ID="${RACER_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
export RACER_SUITE_DIR="${RACER_SUITE_DIR:-${final_root}/results/sionna_uav1_blocking_10uav_5sites_300s_${RACER_RUN_ID}}"
export RACER_BLOCKING_INITIAL_ASSIGNMENT=true
export RACER_EXPLORATION_ASSIGNMENT_MODE=original
export RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=false
export RACER_EXPERIMENT_DURATION=300
export RACER_RANDOM_SEED=42
export RACER_DISCOVERY_DOMAIN_ID=86
export RACER_START_LAYOUT_FILE="${final_root}/config/warehouse_full_10uav_five_sites_layout.json"
export RACER_ALGORITHM_LABEL=final_racer_sionna_uav1_blocking_initial_assignment_300s
mkdir -p "${RACER_SUITE_DIR}"
python3 - "${RACER_SUITE_DIR}" "${RACER_START_LAYOUT_FILE}" <<'PY'
import json,sys,shutil
from pathlib import Path
out=Path(sys.argv[1]); layout=Path(sys.argv[2])
shutil.copyfile(layout,out/'start_layout.json')
(out/'blocking_assignment_config.json').write_text(json.dumps({
 'coordinator_uav_id':1, 'blocking_initial_assignment':True,
 'startup_assignment_transport':'normal Sionna; no perfect-delivery bypass',
 'waiting_timeout_s':None, 'start_on_local_assignment_receipt':True,
 'duration_s':300, 'duration_scope':'whole experiment; late starters use remaining time',
 'post_start_assignment_mode':'original pairwise with existing global recovery',
 'initial_coordinator_policy':'wait for fresh DroneState from all 10 UAVs; retransmit initial assignment every 0.5 simulation seconds until all ACKs',
 'network_topology':'distributed', 'bs_enabled':False,
 'start_positions':json.loads(layout.read_text())['start_positions']},indent=2)+'\n')
PY
export RACER_START_LAYOUT_FILE="${RACER_SUITE_DIR}/start_layout.json"
exec "${final_root}/scripts/run_sionna_distributed_10uav_5sites_300s.sh" "${1:---run}"
