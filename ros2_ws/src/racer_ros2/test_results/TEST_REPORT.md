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

Result: 18 tests, 0 errors, 0 failures, 0 skipped.

The tests cover log-odds scan fusion, non-finite sensor input, map-boundary
clipping, hit-surface voxel placement, peer-map merge, hierarchical-grid
subdivision, exact small-instance open CVRP allocation, A*, B-splines, CBF
separation, lidar braking, metric obstacle inflation, reachable-component
frontier selection, safe stand-off viewpoints and shared-trajectory conflicts.

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

Result file: `/tmp/racer_small_regression_after_long_run2.json`

- passed: yes
- first 95% completion time: 7.65 s
- map coverage: 98.83%
- moving UAVs: 3
- pairwise proposals / accepted: 13 / 12
- PhysX lidar frames: 897
- PhysX contact events: 0
- minimum UAV center distance: 0.828 m
- minimum UAV-body obstacle clearance: 0.397 m
- false-free obstacle cells: 0
- occupied-cell precision: 89.30%

### Large scene: 20 m x 10 m x 2 m, 60 seconds

Result file: `/tmp/racer_large_regression_after_long.json`

- passed: yes
- first 95% completion time: 27.15 s
- map coverage: 98.16%
- moving UAVs: 3
- pairwise proposals / accepted: 26 / 21
- PhysX lidar frames: 1794
- PhysX contact events: 0
- minimum UAV center distance: 4.671 m
- minimum UAV-body obstacle clearance: 0.184 m
- false-free obstacle cells: 14 / 416, 3.37%
- occupied-cell precision: 86.98%

### Long scene: 20 m x 50 m x 3 m, 300 seconds

Result file: `/tmp/racer_long_20x50_run6.json`

The connected scene contains four offset 8 m barriers, five compact box
obstacles and four boundary walls. The three 27 g Crazyflie-like bodies start
from separate safe pads in the south, centre and north thirds. Completion is
the first instant at which all three shared maps are at least 95% known.

- passed: yes
- first 95% completion time: 199.40 s
- coverage at completion: 95.025%
- path distance at completion:
  - UAV 0: 89.940 m
  - UAV 1: 107.801 m
  - UAV 2: 112.863 m
  - fleet total: 310.604 m
- final map coverage after the 300 s observation window: 97.881%
- final per-UAV coverage: 97.838%, 97.881%, 97.756%
- final path distance: 103.903 m, 124.923 m, 126.601 m
- moving UAVs: 3
- pairwise proposals / accepted: 127 / 106
- PhysX lidar frames: 8961
- collision events / PhysX contact events: 0 / 0
- maximum contact force: 0 N
- minimum UAV center distance: 4.639 m
- minimum UAV-body obstacle clearance: 0.253 m
- simulator-side safety interventions: 0
- false-free obstacle cells: 0
- occupied-cell precision: 84.44%

Earlier long-scene attempts exposed and were used to correct three defects:
grid clearance was rounded from 0.60 m to 0.75 m, disconnected frontier
candidates could hide reachable viewpoints, and replacing the trajectory every
0.8 s reset progress toward longer goals. A deliberately maze-like preliminary
scene was also replaced with the requested open venue containing several
obstacles. One rejected intermediate run recorded one 0.038 N contact; it is
not used as the acceptance result.

All acceptance runs used actual Isaac rigid-body poses, actual
`RotatingLidarPhysX` ranges and actual contact-sensor state. The adapter's
simulator-side safety intervention count remained zero, so the no-collision
result came from the ROS 2 planners/controllers rather than hidden teleporting
or position clipping.

The vehicle is a Crazyflie 2.x mass/size rigid-body proxy with velocity
setpoints and acceleration limits. It is not a motor-, propeller- and
aerodynamics-level Crazyflie model. Mapping and planning are performed at a
fixed flight altitude in 2.5-D.
