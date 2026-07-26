# RACER ROS 2 Humble / Isaac Sim compatibility implementation

This package reimplements the main decentralized RACER workflow described in
`racer.pdf` and the adjacent ROS 1 source tree for Ubuntu 22.04, ROS 2 Humble
and Isaac Sim 5.1.

It is not a source-identical port of the original RACER. The original project
uses ROS 1/catkin, a custom UAV simulator, 3-D occupancy/ESDF mapping, LKH-3
and NLopt. It has no native ROS 2 or Isaac Sim launch path. This package is a
Python/ROS 2 compatibility implementation with a fixed-altitude 2.5-D map.
See [test_results/FIDELITY_AUDIT.md](test_results/FIDELITY_AUDIT.md) for the
component-by-component comparison.

## Implemented workflow

Each UAV runs an independent `racer_agent` process. Agents exchange only maps,
states, trajectories and pairwise allocation messages.

| RACER concept | ROS 2 implementation |
|---|---|
| Online hierarchical-grid decomposition | `racer_ros2/hgrid.py` |
| Asynchronous pairwise request/response | `RacerAgent._pairwise_timer()` and `_pairwise_callback()` |
| Capacity-limited two-UAV allocation | exact open CVRP for small instances, deterministic insertion fallback |
| Frontier viewpoints and CP guidance | frontier clusters, information gain and hgrid coverage routes |
| Local path and trajectory generation | 8-connected A*, corridor-safe cubic B-spline and time scaling |
| Decentralized collision avoidance | shared trajectories, priority yielding, CBF separation and lidar braking |
| Map sharing | per-UAV log-odds maps on `/racer/map_share`, occupied evidence wins |

## Isaac Sim integration

The Isaac adapter uses:

- real PhysX rigid bodies, not Python-integrated positions;
- a 27 g, 0.16 m Crazyflie-like collision proxy with the bundled Crazyflie
  visual asset;
- `RotatingLidarPhysX` depth and azimuth buffers for mapping;
- PhysX contact sensors for collision acceptance;
- synchronized odometry and scans, including the measured two-step PhysX
  sensor latency.

The vehicle is velocity-controlled at a fixed altitude. It is not a
motor/propeller/aerodynamic Crazyflie model. Obstacles are floor-to-ceiling,
so each physical scene is mapped as a horizontal 2.5-D slice.

## Build

```bash
cd RACER/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Tests

The script defaults to `/home/jackson/isaacsim`. Set `ISAAC_SIM_ROOT` when
Isaac Sim is installed elsewhere.

Small 8 m x 6 m x 2 m scene:

```bash
cd RACER/ros2_ws
RACER_SCENARIO=small \
RACER_TEST_DURATION=30 \
src/racer_ros2/scripts/run_isaac_test.sh /tmp/racer_small.json
```

Requested 20 m x 10 m x 2 m scene:

```bash
cd RACER/ros2_ws
RACER_SCENARIO=large \
RACER_TEST_DURATION=60 \
src/racer_ros2/scripts/run_isaac_test.sh /tmp/racer_large.json
```

Long 20 m x 50 m x 3 m scene with nine internal obstacles:

```bash
cd RACER/ros2_ws
RACER_SCENARIO=long \
RACER_TEST_DURATION=300 \
RACER_MINIMUM_COVERAGE=0.95 \
src/racer_ros2/scripts/run_isaac_test.sh /tmp/racer_long.json
```

Isaac Sim 5.x and system ROS 2 Humble use different Python installations.
The test script starts Isaac with its bundled Humble bridge and the agents
with system Humble; they communicate through Fast DDS.

For an interactive Isaac window, first launch the ROS side:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch racer_ros2 swarm_exploration.launch.py \
  backend:=isaac scenario:=large drone_count:=3 duration:=120
```

Then run the adapter in another terminal:

```bash
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/jackson/isaacsim/exts/isaacsim.ros2.bridge/humble/lib \
  /home/jackson/isaacsim/python.sh \
  src/racer_ros2/isaac_sim/isaac_sim_racer.py \
  --duration 120 --scenario large --drone-count 3
```

## Acceptance checks

`racer_monitor` fails the Isaac test unless all of the following hold:

- all UAV maps are received and requested coverage is reached;
- the backend, odometry and scan sources are real Isaac PhysX components;
- at least two UAVs move at least 0.5 m;
- at least one pairwise allocation is accepted;
- PhysX contact events and simulator safety interventions are both zero;
- minimum inter-UAV distance is at least 0.50 m;
- UAV-body obstacle clearance is at least 0.05 m;
- the map contains obstacle evidence, false-free obstacle cells are at most
  5%, and occupied-cell precision is at least 80%.

Measured results are in
[test_results/TEST_REPORT.md](test_results/TEST_REPORT.md).
