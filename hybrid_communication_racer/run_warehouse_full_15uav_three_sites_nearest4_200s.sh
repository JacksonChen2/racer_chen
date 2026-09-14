#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RACER_EXPERIMENT_SCENE_PROFILE=warehouse_full_with_industrial_ap
export RACER_15UAV_NEAREST4_DOMAIN_ID="${RACER_WAREHOUSE_FULL_15UAV_NEAREST4_DOMAIN_ID:-200}"
exec "${workspace}/run_warehouse_full3_rf_blocking_racks_15uav_three_sites_nearest4_200s.sh"
