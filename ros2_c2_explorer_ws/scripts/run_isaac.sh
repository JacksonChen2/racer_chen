#!/usr/bin/env bash
set -euo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${workspace_dir}/src/c2_explorer_isaac/scripts/run_c2_warehouse.sh"

