# RACER ROS 2 / Isaac Sim acceptance report

Test date: 2026-07-26

## Platform

- Ubuntu 22.04.5 LTS
- ROS 2 Humble with `rmw_fastrtps_cpp`
- Isaac Sim 5.1
- NVIDIA GeForce RTX 4060 Laptop GPU, driver 580.159.03
- three Crazyflie-like 27 g PhysX rigid-body proxies

## Final results

### Build and unit tests

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racer_ros2
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  colcon test --packages-select racer_ros2
colcon test-result --verbose
```

Result: 14 tests, 0 errors, 0 failures, 0 skipped.

The tests cover log-odds scan fusion, non-finite sensor input, map-boundary
clipping, hit-surface voxel placement, peer-map merge, hierarchical-grid
subdivision, exact small-instance open CVRP allocation, A*, B-splines, CBF
separation, lidar braking and shared-trajectory conflicts.

### PhysX sensor-coordinate probe

A body was translated at 0.8 m/s and rotated at 1.5 rad/s while measured lidar
ranges were compared against scene geometry.

- PhysX buffer: 180 azimuth columns x 2 elevation rows
- azimuth coverage: -pi to 3.1067 rad
- detected sensor delay: 2 physics steps, 0.10 s
- mean range error after synchronized-pose correction: 0.000261 m
- mean error when incorrectly treating azimuth as world-frame: 1.2097 m

### Contact-sensor positive control

The test body was intentionally driven into a wall.

- collision events: 1
- maximum contact force: 0.424882 N
- obstacle penetration: 0.0400 m

This confirms that a zero event count in the exploration runs is not caused by
a disabled detector. Zero-force startup states are excluded from collisions.

### Small scene: 8 m x 6 m x 2 m, 30 seconds

Result file: `/tmp/racer_small_range35.json`

- passed: yes
- map coverage: 97.92%
- moving UAVs: 2
- pairwise proposals / accepted: 10 / 8
- PhysX lidar frames: 897
- PhysX contact events: 0
- minimum UAV center distance: 2.794 m
- minimum UAV-body obstacle clearance: 0.351 m
- false-free obstacle cells: 0
- occupied-cell precision: 100%

### Large scene: 20 m x 10 m x 2 m, 60 seconds

Result file: `/tmp/racer_large_final.json`

- passed: yes
- map coverage: 98.38%
- moving UAVs: 3
- pairwise proposals / accepted: 26 / 20
- PhysX lidar frames: 1794
- PhysX contact events: 0
- minimum UAV center distance: 4.035 m
- minimum UAV-body obstacle clearance: 0.291 m
- false-free obstacle cells: 7 / 416, 1.68%
- occupied-cell precision: 88.50%

Both acceptance runs used actual Isaac rigid-body poses, actual
`RotatingLidarPhysX` ranges and actual contact-sensor state. The adapter's
simulator-side safety intervention count remained zero, so the no-collision
result came from the ROS 2 planners/controllers rather than hidden teleporting
or position clipping.
