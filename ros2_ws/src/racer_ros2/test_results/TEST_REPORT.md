# RACER ROS 2 acceptance report

Test date: 2026-07-26

## Platform

- Ubuntu 22.04.5 LTS
- ROS 2 Humble (`rmw_fastrtps_cpp`)
- Isaac Sim `5.1.0-rc.19+release.26219.9c81211b.gl`
- NVIDIA GeForce RTX 4060 Laptop GPU, driver 580.159.03, 8 GB VRAM

Isaac Sim's compatibility checker warns that 8 GB is below its recommended
10 GB VRAM. The lightweight headless RACER scene nevertheless completed every
test below. Larger RTX-sensor scenes may need a GPU with more VRAM.

## Results

### Unit and algorithm tests

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select racer_ros2
colcon test-result --verbose
```

Result: 10 tests, 0 errors, 0 failures, 0 skipped.

Covered modules include scan fusion, peer-map merge, hgrid subdivision,
capacity partition completeness/balance, A* safety, CP-guided frontier
planning, B-spline corridor validation, CBF separation, lidar braking and
trajectory conflict prediction.

### ROS 2 deterministic integration, 12 seconds

- Passed: yes
- Merged-map coverage: 94.14%
- Pairwise proposals / accepted: 5 / 4
- Collision events: 0
- Simulator safety interventions: 0
- Minimum UAV center distance: 3.064 m
- Minimum UAV-body obstacle clearance: 0.347 m

### Isaac Sim + ROS 2 Humble integration, 12 seconds

- Passed: yes
- Merged-map coverage: 94.44%
- Pairwise proposals / accepted: 4 / 3
- Collision events: 0
- Simulator safety interventions: 0
- Minimum UAV center distance: 2.972 m
- Minimum UAV-body obstacle clearance: 0.362 m

### Isaac Sim endurance run, 45 seconds

- Passed: yes
- Merged-map coverage: 96.68%
- Collision events: 0
- Simulator safety interventions: 0
- Minimum UAV center distance: 1.413 m
- Minimum UAV-body obstacle clearance: 0.303 m

An earlier 45-second run exposed two agents converging on a residual frontier:
their distance reached 0.668 m and failed the then 0.70 m acceptance limit.
The implementation was changed to publish trajectories at 5 Hz, stop pursuing
residual frontiers after the completion threshold, apply an explicit
close-range separation layer, and use 0.60 m obstacle inflation. The repeated
45-second run produced the passing result above. Current acceptance limits are
stricter: 1.00 m inter-UAV distance, 0.10 m obstacle clearance, zero collision
events, zero plant safety interventions, and at least one acknowledged
pairwise allocation.
