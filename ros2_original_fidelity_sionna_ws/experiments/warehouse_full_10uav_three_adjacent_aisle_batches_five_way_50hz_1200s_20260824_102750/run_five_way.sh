#!/usr/bin/env bash
set -euo pipefail

workspace="/home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_original_fidelity_sionna_ws"
export RACER_SUITE_DIR="${workspace}/experiments/warehouse_full_10uav_three_adjacent_aisle_batches_five_way_50hz_1200s_20260824_102750"
export RACER_LAYOUT_TAG="three_adjacent_aisles_4_3_3"

# UAV 1-4: central aisle; UAV 5-7: adjacent aisle on the left;
# UAV 8-10: adjacent aisle on the right.  Heights are staggered, while the
# minimum horizontal spacing inside each batch remains at least 1.2 m.
export RACER_START_POSITIONS="\
-7.8 15.0 0.8   -6.6 15.0 1.5   -7.8 16.2 2.2   -6.6 16.2 1.15 \
-14.49 14.91 0.8  -13.2 14.91 1.5  -11.91 14.91 2.2 \
-2.45 14.0 0.8  -2.45 15.3 1.5  -2.45 16.6 2.2"

exec "${workspace}/experiments/warehouse_full_10uav_two_wall_batches_five_way_50hz_1200s_20260824_101523/run_five_way.sh"
