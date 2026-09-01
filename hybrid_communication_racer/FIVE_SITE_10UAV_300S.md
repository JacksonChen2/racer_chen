# Five-site 10-UAV 300-second experiment

This workspace contains the source and launch tooling used for the matched
300-second Ideal/Sionna warehouse experiments. Generated builds, Python
runtimes, logs, and experiment outputs are intentionally excluded from Git.

## Fixed experiment configuration

- 10 UAVs at five sites, two UAVs per site
- U5/U6 in the same vertical aisle
- 300 seconds of simulation time
- 100 Hz physics and 10 Hz depth sensing
- 76,800 rays per UAV depth frame
- Pairwise Robust RACER with the original planning FSM
- Ideal case: lossless distributed transport
- Sionna case: distributed 28 GHz, 20 dBm UAV power, fixed MCS 14, no retries

The exact start coordinates are stored in
`config/warehouse_full_10uav_five_sites_layout.json`.

## Required repository assets

The launch scripts use these files from the parent repository:

- `warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda`
- `warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/`
- `isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd`

The scripts verify the Isaac and Sionna scene hashes before a formal run.

## Build

Prerequisites are Ubuntu 22.04, ROS 2 Humble, a working Isaac Sim installation,
`jq`, CMake, a C++ compiler, and the ROS/PCL/Eigen/Armadillo dependencies used
by RACER. Set `ISAAC_SIM_ROOT` if Isaac Sim is not installed in
`~/isaacsim`.

```bash
cd /path/to/racer_chen/hybrid_communication_racer
./build_nlopt.sh
./setup_sionna_env.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
```

## Run a matched seed

Use a new output directory for every repetition. The result and manifest both
record the selected seed.

```bash
cd /path/to/racer_chen/hybrid_communication_racer

RACER_RANDOM_SEED=42 \
RACER_RUN_ID=seed42_repeat1 \
RACER_SUITE_DIR="$PWD/experiments/seed42_repeat1/ideal_300s" \
./run_warehouse_full_10uav_five_sites_ideal_takeoff_fixed_300s.sh

RACER_RANDOM_SEED=42 \
RACER_RUN_ID=seed42_repeat1 \
RACER_DISCOVERY_DOMAIN_ID=84 \
RACER_SUITE_DIR="$PWD/experiments/seed42_repeat1/sionna_300s" \
./run_warehouse_full_10uav_five_sites_sionna_300s.sh
```

Each case writes `run_state.txt`, `experiment_manifest.json`, raw launch/Isaac
logs, `warehouse_full_distributed_result.json`, and a dedicated validation log.
The Sionna case is valid only when all ten original RACER FSMs execute, Isaac
exits normally, Sionna RT is active, and no collision/contact event is recorded.

## Plot coverage and trajectories

```bash
python3 scripts/plot_five_site_trajectory_coverage.py \
  --ideal experiments/seed42_repeat1/ideal_300s/ideal_no_loss_takeoff_fixed_300s/warehouse_full_distributed_result.json \
  --sionna experiments/seed42_repeat1/sionna_300s/sionna_distributed_no_retries_uav20dbm_fixed_mcs14/warehouse_full_distributed_result.json \
  --mesh-dir ../warehouse_scenes/sionna/warehouse_full_with_industrial_ap_20260827_101239/meshes \
  --output experiments/seed42_repeat1/trajectory_coverage.png \
  --seed 42
```

The plotter reconstructs trajectories from the saved 0.5-second position
history and synchronizes the asynchronous per-UAV map snapshots into the joint
cumulative coverage curve.

## Reproducibility note

The seed fixes the explicit planner and channel RNGs. ROS 2 message ordering,
thread scheduling, and Isaac/communication timing are asynchronous, so repeated
runs with an identical seed are not guaranteed to be bit-for-bit identical.
