# ROS 2 architecture

## Runtime boundary

```text
Isaac Sim 5.1 (bundled Python)
  ROS 2 Bridge: /clock, odometry, point cloud, velocity command
                    │ DDS
Ubuntu 22.04 / ROS 2 Humble (Python 3.10)
  racer_ros adapters
                    │ ordinary Python calls
  racer_core algorithms
```

The simulator process never imports the Humble Python installation. DDS is the
only boundary, avoiding the Python 3.10/3.11 ABI conflict.

## Package ownership

- `racer_core`: deterministic data structures and algorithms with no ROS
  import. This makes the mathematical migration independently testable.
- `racer_interfaces`: the one necessary `ament_cmake` package because ROS 2
  interface generation is CMake-based.
- `racer_ros`: all `rclpy` subscriptions, publishers, services, QoS and
  message conversion.
- `racer_bringup`: launch descriptions, the parameter set and RViz2.
- `racer_isaac`: scripts installed for execution by Isaac's own Python.

Each UAV is placed in `/drone_<id>`. Mapping, planning and commands are
relative topics in that namespace. Decentralized state, trajectories and
multi-map chunks retain shared absolute topics. No central planner is added.

## Planning flow

1. Isaac publishes odometry and a world-frame point cloud.
2. The probabilistic voxel map integrates rays, inflates obstacles and updates
   the ESDF.
3. Map chunks are exchanged asynchronously with interval stamps and replay of
   missing chunks.
4. Frontier cells are clustered and split; collision-free viewpoints are
   sampled and scored by visibility.
5. Viewpoint costs retain path, velocity direction and yaw-transition terms.
6. LKH selects a tour when installed; the deterministic nearest-neighbour
   path is only a source-level fallback.
7. A* produces the geometric path. The B-spline optimizer applies smoothness,
   ESDF clearance and dynamic-feasibility terms.
8. A layered graph maximizes information gain while penalizing yaw-rate
   violations.
9. The trajectory server evaluates position, velocity, acceleration, yaw and
   yaw rate at the configured command period.
10. The Isaac adapter emits standard Pose, Twist and Accel messages for the
    vehicle-specific controller in the stage.

