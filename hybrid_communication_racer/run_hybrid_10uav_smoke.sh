#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RACER_SMOKE_DURATION_S="${1:-${RACER_SMOKE_DURATION_S:-30}}"
exec "${workspace}/run_hybrid_communication_racer_10uav_smoke.sh"
