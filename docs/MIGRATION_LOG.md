# Migration log

## Baseline

- Upstream: `Robotics-STAR-Lab/RACER`
- Baseline branch: `main`
- Baseline commit: `049c332e3634ef72d8beb155b4c13dc91ca52916`
- Baseline tree: `e9717c6ba63381c943776757f9a4441e48a9ffcf`
- Tracked files checked against the downloaded tree: 1,749
- Missing, extra, or content-different files: 0

## ROS 2 migration branch

Branch: `migration/ros2-python-isaac`

Changes are recorded by source mapping. Target execution results and remaining
limits are maintained in `VALIDATION_STATUS.md`.

## 2026-07-23 — source migration

- Created a five-package colcon workspace under `ros2_ws/src`.
- Consolidated the upstream custom ROS messages and services into
  `racer_interfaces`.
- Ported occupancy/ESDF mapping, ray casting, geometric and kinodynamic
  search, topological PRM, polynomial/B-spline trajectories, optimization,
  perception, frontier extraction, hierarchical partitioning, heading
  planning, LKH invocation, multi-map chunks and position control to
  ROS-independent Python.
- Ported the exploration FSM, trajectory server, LKH compatibility service,
  swarm/map exchange and visualization adapters to `rclpy`.
- Added single- and multi-UAV ROS 2 launch files, centralized parameters and
  an RViz2 configuration.
- Added an Isaac Sim ROS 2 Bridge graph builder and an Isaac Lab command
  buffer; external ROS 2 and Isaac's internal Python remain isolated.
- Preserved the original ROS 1/C++ source trees as reference-only content.
- Per the owner's instruction, no build, test, ROS graph, simulation or
  numerical-equivalence run was performed.

## 2026-07-24 — logic repair and target validation

- Connected hierarchical partitioning, frontier tour/refinement, geometric and
  kinodynamic search, B-spline generation, yaw planning and trajectory
  feasibility into the active planning path.
- Corrected A* timing/indexing, kinodynamic hover expansion, voxel ray-casting
  termination, polynomial waypoint continuity, dynamic/swarm clearance
  gradients, partial map-chunk flushing and ROS message/time contracts.
- Added deterministic multi-drone grid allocation, stale-state handling,
  shared-trajectory collision checks and FSM safety/periodic/idle replanning.
- Added lidar-frame-to-world conversion and a `geometry_msgs/Twist` Isaac
  command contract with a default rigid-body velocity callback.
- Built all five packages in a clean ROS 2 Humble workspace.
- Passed 14 Python algorithm and ROS contract tests.
- Started and cleanly stopped both the single-drone graph and a three-drone,
  nine-node graph; verified swarm publishers/subscribers and injected odometry,
  point cloud and trigger messages through a successful planning attempt.
- Created the Isaac 5.1 OmniGraph successfully. The local Isaac ROS bridge
  runtime and a concrete UAV Stage remain external validation items; see
  `VALIDATION_STATUS.md`.
