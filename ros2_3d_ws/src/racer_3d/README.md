# RACER 3-D for ROS 2 Humble and Isaac Sim

This package is an independent ROS 2/Isaac Sim reproduction of the
three-dimensional RACER exploration pipeline. It does not depend on the older
planar compatibility package in `RACER/ros2_ws`.

## Implemented pipeline

- probabilistic `(x,y,z)` log-odds voxel occupancy map and signed ESDF;
- standard `sensor_msgs/PointCloud2` input from an Isaac
  `RotatingLidarPhysX` with 360° azimuth and 120° vertical field of view;
- six-connected volumetric frontier extraction, clustering, spherical
  viewpoint sampling, visibility checks and information gain;
- online 3-D octree-like HGrid decomposition, persistent cell ownership,
  pairwise interaction and capacity-constrained open-route allocation;
- 26-connected 3-D A*, strict edge/corner collision checks, path shortening,
  cubic B-spline control-point optimization and dynamic time scaling;
- ESDF obstacle barriers with rigid-body stopping distance, predicted
  trajectory conflict checks, 3-D inter-UAV CBF separation and emergency
  separation;
- a 27 g Crazyflie 2.x six-DOF PhysX body, visual USD model, gravity, local
  rotor thrust, motor mixing, attitude torque and geometric velocity control;
- measured volume coverage, free/occupied map quality, obstacle surface
  recall, flight distance, inter-UAV distance, obstacle clearance and raw
  PhysX contact events.

Two deterministic acceptance worlds are included. The 15 m × 9 m × 2 m
baseline contains a low partition and a suspended partition. The
20 m × 50 m × 3 m long-range world adds full-width overflight/underflight
walls, a central gate, mixed-height barriers, columns and hanging/low blocks.
Both require genuine vertical motion, so a planar solution cannot pass.
The supplied `warehouse_simple.usd` is also registered as an external-scene
test profile with measured flight bounds and three collision-free launch
poses.

## Tested platform

- Ubuntu 22.04.5 LTS;
- ROS 2 Humble;
- Isaac Sim 5.1;
- NVIDIA GeForce RTX 4060 Laptop GPU.

The default Isaac installation path is `/home/jackson/isaacsim`. Set
`ISAAC_SIM_ROOT` when Isaac is installed elsewhere.

## Build and test

```bash
cd RACER/ros2_3d_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Fast ROS-only integration test:

```bash
RACER_3D_DURATION=60 ROS_DOMAIN_ID=43 \
  src/racer_3d/scripts/run_mock_test.sh
```

The mock plant is only a development check. It cannot qualify as the Isaac
physics acceptance test.

Formal three-Crazyflie Isaac test:

```bash
RACER_3D_DURATION=120 ROS_DOMAIN_ID=63 \
  src/racer_3d/scripts/run_isaac_test.sh \
  src/racer_3d/test_results/ISAAC_15X9X2_RESULT.json
```

The script exits nonzero unless all acceptance thresholds pass, including zero
reported contacts and zero collision events.

Large-scene three-Crazyflie Isaac test:

```bash
RACER_3D_SCENARIO=acceptance_20x50x3 \
RACER_3D_DURATION=600 ROS_DOMAIN_ID=77 \
  src/racer_3d/scripts/run_isaac_test.sh \
  src/racer_3d/test_results/ISAAC_20X50X3_RESULT.json
```

`RACER_3D_DURATION` is simulated mission time. A successful test stops as soon
as the truth evaluator first reaches 90% free-space volume coverage. The
recorded large-scene pass reached 90% in 402.70 simulated seconds with zero
PhysX contacts.

Supplied Warehouse test:

```bash
RACER_3D_SCENARIO=warehouse_simple \
RACER_3D_DURATION=900 ROS_DOMAIN_ID=53 \
  src/racer_3d/scripts/run_isaac_test.sh \
  src/racer_3d/test_results/ISAAC_WAREHOUSE_SIMPLE_RESULT.json
```

The recorded Warehouse pass reached 90.0025% bounded observed-volume coverage
in 842.62 simulated seconds with three vehicles and zero PhysX contacts.

## Using a custom Isaac USD scene

The USD must contain collision-enabled geometry. Add a profile to
`racer_3d/scenario.py` so its map bounds, launch positions, HGrid scale and
configured safety bounds describe the new world, then rebuild. Use
`isaac_sim/audit_usd_scene.py` to check dependencies, collision prims, world
bounds and candidate launch poses before flight.

Launch the ROS agents without the acceptance monitor:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=64 ros2 launch racer_3d swarm_3d.launch.py \
  backend:=isaac drone_count:=3 run_monitor:=false
```

In another terminal, start the Isaac bridge with the same ROS domain and
matching launch positions:

```bash
cd RACER/ros2_3d_ws
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
  -u PYTHONPATH ROS_DOMAIN_ID=64 ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}/exts/isaacsim.ros2.bridge/humble/lib" \
  "${ISAAC_SIM_ROOT:-/home/jackson/isaacsim}/python.sh" \
  src/racer_3d/isaac_sim/isaac_sim_racer_3d.py \
  --scene-usd /absolute/path/to/scene.usd \
  --drone-count 3 --duration 120 \
  --starts -6.4 -3.2 0.45 -6.4 0.0 1.0 -6.4 3.2 1.55
```

For a custom mesh profile, use `truth_mode="observed_volume"` unless an
independent ground-truth voxel map is available. This measures bounded known
volume and PhysX safety but intentionally leaves free/occupied accuracy and
surface-recall fields unset.

The deterministic worlds also expose their AABB collision primitives to the
50 Hz Isaac flight-controller safety barrier. This is the final
stopping-distance guard used by their formal contact test. External USD
profiles use 10 Hz local lidar returns plus configured six-face flight-volume
barriers, in addition to the agent's point-cloud occupancy/ESDF planning and
execution barriers. Safety bounds must be measured for each new scene rather
than copied from the Warehouse profile.

## ROS interfaces

For vehicle `N`:

- input sensor/state: `/drone_N/points`, `/drone_N/odom`;
- flight command: `/drone_N/cmd_vel_3d`;
- outputs: `/drone_N/planned_path_3d`,
  `/drone_N/occupied_voxels`, `/drone_N/status`;
- distributed coordination: `/racer_3d/map_share`,
  `/racer_3d/swarm_state`, `/racer_3d/pairwise`;
- simulation/acceptance: `/racer_3d/sim_metrics`.

The Isaac adapter is a realistic rotor-wrench simulation interface, not a
radio driver for physical Crazyflie hardware. A real vehicle needs an external
bridge that converts `/drone_N/cmd_vel_3d` to the selected Crazyflie firmware
or Crazyswarm2 command interface and returns synchronized odometry/point
clouds.

RACER does not define an automatic return-to-home phase after exploration.
This reproduction therefore stops and hovers when its coverage threshold is
reached; returning to the launch point should be implemented as a separate
mission state when required.
