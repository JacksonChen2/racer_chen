#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RACER_EXPERIMENT_COMMUNICATION_MODE=ideal
export RACER_FULL3_SIONNA_DOMAIN_ID="${RACER_FULL3_IDEAL_DOMAIN_ID:-198}"
exec "${workspace}/run_warehouse_full3_rf_blocking_racks_10uav_sionna_200s.sh"
