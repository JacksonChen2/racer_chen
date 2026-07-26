# RACER ROS 2 / Isaac Sim reproduction

This package is a clean ROS 2 Humble reproduction of the RACER pipeline in
`racer.pdf` and the adjacent ROS 1 source tree. The original source remains
unchanged. Each UAV runs a separate `racer_agent` process and only exchanges
maps, state, trajectories and pairwise allocation messages.

## Paper-to-code mapping

| RACER component | ROS 2 implementation |
|---|---|
| Online hgrid decomposition (Algorithm 1) | `racer_ros2/hgrid.py` |
| Pairwise request/response (Algorithm 2) | `RacerAgent._pairwise_timer()` and `_pairwise_callback()` |
| Capacity-constrained open VRP | `racer_ros2/allocation.py` |
| Incremental frontiers and viewpoints (Algorithm 3) | `OccupancyMap.frontier_clusters()` and `plan_exploration()` |
| CP-guided local TSP / path search | `coverage_route()`, viewpoint scoring and 8-connected A* |
| Minimum-time B-spline | `UniformBSpline` and `minimum_time_trajectory()` |
| Inter-UAV trajectory avoidance (Equations 13–14) | trajectory conflict yielding plus continuous control-barrier projection |
| Volumetric map exchange | fixed-height occupancy chunks over `/racer/map_share` |

The provided acceptance environment is 2.5-D: all obstacles extend above the
fixed flight altitude. The mapping/planning interface is intentionally isolated,
so a 3-D point-cloud mapper can replace `OccupancyMap` without changing the
decentralized allocation protocol.

## Build

```bash
cd RACER/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Fast ROS 2 integration test

```bash
cd RACER/ros2_ws
RACER_TEST_DURATION=45 src/racer_ros2/scripts/run_mock_test.sh
```

## Isaac Sim test

The script defaults to `/home/jackson/isaacsim`; set `ISAAC_SIM_ROOT` if the
installation is elsewhere.

```bash
cd RACER/ros2_ws
RACER_TEST_DURATION=45 src/racer_ros2/scripts/run_isaac_test.sh
```

Isaac Sim 5.x uses Python 3.11 while Ubuntu's Humble installation uses Python
3.10. `run_isaac_test.sh` deliberately starts Isaac with its bundled Humble
bridge and starts the exploration nodes with the system Humble installation.
They communicate through Fast DDS.

For an interactive window, launch the ROS side with `backend:=isaac`, then run
the adapter without `--headless`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch racer_ros2 swarm_exploration.launch.py backend:=isaac

env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/home/jackson/isaacsim/exts/isaacsim.ros2.bridge/humble/lib \
  /home/jackson/isaacsim/python.sh \
  src/racer_ros2/isaac_sim/isaac_sim_racer.py --duration 120
```

## Acceptance criteria

`racer_monitor` writes a JSON result containing:

- merged-map known coverage;
- collision event count;
- minimum center-to-center UAV distance;
- minimum UAV-body-to-obstacle clearance;
- simulator safety-kernel interventions and final positions.

The default test passes only when all UAV maps are received, coverage is at
least 55%, at least one pairwise allocation is acknowledged, collision and
plant safety-intervention counts are both zero, center distance never drops
below 1.00 m, and UAV-body obstacle clearance never drops below 0.10 m.

The results produced on the supplied machine are recorded in
[`test_results/TEST_REPORT.md`](test_results/TEST_REPORT.md).
