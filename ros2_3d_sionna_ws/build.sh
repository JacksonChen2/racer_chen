#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
colcon build --symlink-install --event-handlers console_direct+
