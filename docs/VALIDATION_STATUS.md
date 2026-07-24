# Validation status

Status: **PARTIALLY VALIDATED — ROS 2/Python path passes; cross-version and
Isaac closed-loop validation remain**

Validation date: 2026-07-24

Target used: Ubuntu 22.04, ROS 2 Humble, Python 3.10, Isaac Sim 5.1.0

## Passed

- A clean `colcon build --symlink-install` built all five packages:
  `racer_interfaces`, `racer_core`, `racer_ros`, `racer_bringup` and
  `racer_isaac`.
- Fourteen regression tests passed. They cover geometric A*, kinodynamic
  search, finite voxel ray casting, continuous waypoint trajectories,
  independent position/yaw B-spline timing, hierarchical-grid activation,
  dynamic-obstacle gradients, deterministic TSP fallback, partial map chunks,
  the synthetic frontier-to-trajectory planning chain, ROS message contracts
  and ROS time conversion.
- The single-drone launch brought up the exploration node, trajectory server
  and Isaac command adapter. The Isaac command topic is
  `geometry_msgs/msg/Twist`, and all processes stopped cleanly on Ctrl-C.
- The three-drone launch brought up nine nodes. `/swarm_expl/drone_state` had
  three publishers and three subscribers, and shared trajectory/map exchange
  endpoints were present. All nine processes stopped cleanly.
- A live ROS input smoke test injected odometry, a 900-point lidar cloud and a
  trigger. The node updated the voxel map, produced 134 map chunks, assigned
  hierarchical grid IDs and completed a planning attempt without crashing.
- Isaac Sim 5.1 loaded the bridge script and created the OmniGraph nodes. This
  validates node/port names used by the script for the installed version.

## Not validated

- ROS Noetic is not installed, so the original ROS 1/C++ executable and the
  Python port were not run on the same bag/map. Exact frontier sets, cost
  matrices, tours, B-spline samples, completion time and CPU performance are
  therefore not claimed numerically equivalent.
- No concrete Isaac UAV Stage was supplied. Sensor publication, DDS traffic,
  vehicle actuation and collision-free closed-loop exploration have not been
  demonstrated in Isaac.
- In the first headless Isaac run the graph was created, but the ROS bridge
  could not load because system Humble's Python 3.10 `rclpy` was visible to
  Isaac's Python 3.11. Subsequent headless restarts terminated in Kit before
  application initialization. The README now documents starting Isaac with
  the bridge extension's bundled Humble libraries; this still needs a stable
  GUI/headless session and a real Stage to verify.
- LKH was not available during these tests. The Python fallback is exact for
  small tours and deterministic for larger tours, but it is not a performance
  substitute for upstream LKH.
- The Python implementation reproduces the active algorithm stages and
  constraints, but it is not a line-for-line numerical translation: the
  waypoint initializer, small-tour fallback and decentralized allocation are
  Python-native equivalents. Upstream pairwise optimization and published
  benchmark parity remain future comparison gates.

## Environment warning

The machine has NumPy 1.26.4 in the user Python path while Ubuntu's SciPy 1.8
expects NumPy below 1.25. All tests passed, but this mismatch should be removed
before performance or long-duration runs, preferably by using the Ubuntu
`python3-numpy` and `python3-scipy` packages in a clean ROS terminal.

## Remaining acceptance gates

1. Start Isaac with the documented bundled-Humble environment and verify
   `/clock`, odometry, point cloud and velocity-command DDS traffic.
2. Replace or tune the default rigid-body velocity callback for the selected
   multirotor asset, then run a single-UAV closed loop.
3. Replay one identical dataset through ROS 1/C++ and ROS 2/Python and compare
   frontier centroids, grid allocation, tours and sampled trajectories using
   explicit tolerances.
4. Run three UAVs with packet loss/replay and assert map convergence, minimum
   inter-UAV clearance, dynamic limits, coverage and completion time.
