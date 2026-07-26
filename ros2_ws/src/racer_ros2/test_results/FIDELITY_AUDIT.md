# RACER fidelity and platform audit

## Conclusion

The original repository is not a ROS 2 or Isaac Sim implementation. It is a
ROS 1/catkin C++ system documented for Ubuntu 18.04/ROS Melodic and Ubuntu
20.04/ROS Noetic, with its own map generator and UAV simulation stack.

`racer_ros2` is now a working ROS 2 Humble / Isaac Sim compatibility
implementation of RACER's decentralized exploration architecture. It preserves
the high-level coordination flow, but it is not algorithmically identical to
the original 3-D C++ system.

## Component comparison

| Component | Original RACER source | Current ROS 2 package | Fidelity |
|---|---|---|---|
| Middleware | ROS 1, catkin, custom messages | ROS 2 Humble, `rclpy`, standard messages plus JSON state | platform adaptation |
| World model | probabilistic 3-D voxel occupancy and ESDF from depth/point clouds | shared 2-D log-odds grid at fixed flight altitude | simplified |
| Frontier representation | 3-D frontier information structure and viewpoints | 2-D frontier clusters with information-gain candidates | concept preserved, geometry simplified |
| Hgrid | online multilevel decomposition | two-level online decomposition with demand and ownership | structural reproduction |
| Pairwise coordination | asynchronous request/response optimization | decentralized request/response with timeouts and acceptance | structural reproduction |
| Global allocation | LKH-3 asymmetric capacitated VRP/TSP | exact two-vehicle open CVRP for up to 12 cells, insertion fallback | objective approximated |
| Local coverage guidance | CP-guided local TSP and viewpoint refinement | hgrid coverage route, frontier score and A* | concept preserved |
| Trajectory optimization | NLopt minimum-time B-spline with collision costs | corridor-checked cubic B-spline with speed/acceleration time scaling | simplified |
| Inter-UAV avoidance | shared trajectories and optimization penalties | shared trajectories, priority yielding, CBF projection and emergency separation | behavior preserved with different controller |
| Simulation | original custom simulator/map generator | Isaac Sim 5.1 PhysX plant and sensors | new integration |
| Vehicle | quadrotor dynamics in original stack | fixed-height, velocity-controlled 27 g Crazyflie-like rigid-body proxy | not dynamics-identical |

## Problems found and fixed

1. The earlier Isaac adapter integrated poses itself and synthesized ranges.
   It was replaced with PhysX rigid bodies, `RotatingLidarPhysX`, contact
   sensors and odometry read from the simulated bodies.
2. The PhysX `linear_depth` buffer was unsuitable for this lidar path. The
   adapter now uses the official normalized `depth` and `azimuth` buffers.
3. Completed lidar frames lag rigid-body poses by two 50 ms physics steps.
   Odometry is delayed to the true acquisition pose, and ROS agents pair scan
   and odometry by the exact common timestamp.
4. Raw surface hits on grid boundaries could be placed on the free side.
   The inverse sensor model now stops free carving before a hit and places
   occupied evidence half a voxel behind the surface.
5. Free ray carving was made conservative around diagonal grid cells, and weak
   one-ray evidence remains unknown.
6. Allocation capacity was aligned with the paper's 0.75 ratio, and small
   pairwise problems now use an exact open-CVRP solver.
7. Acceptance was extended to reject mock backends, synthetic sensor sources,
   stationary “multi-UAV” runs, poor obstacle maps, physical contact and hidden
   simulator safety corrections.
8. Metric obstacle inflation no longer rounds 0.60 m up to 0.75 m at 0.25 m
   map resolution. Frontier candidates are filtered by the UAV's reachable
   known-free component and use safe stand-off viewpoints at wall corners.
9. Safe trajectories are retained across planning cycles until completion,
   invalidation or timeout. This restores receding-horizon continuity instead
   of resetting progress every 0.8 s.
10. Lidar braking now includes the acceleration-limited PhysX stopping margin.
    The 20 m x 50 m x 3 m three-UAV run reached 95% at 199.4 s with zero
    contacts.

## Remaining gap to source-identical RACER

Achieving source-identical behavior would require a full ROS 2 port of the
original C++ packages, the 3-D occupancy/ESDF and FIS pipeline, LKH-3 ACVRP,
NLopt trajectory objectives, the original custom messages and communication
semantics, and a dynamic quadrotor/flight-controller interface in Isaac Sim.
Those components are not present in this compatibility package.

The completed tests therefore establish that this implementation performs
decentralized, collision-free multi-UAV exploration and fixed-altitude mapping
in Isaac Sim with ROS 2 Humble. They do not establish numerical equivalence to
the original paper's full 3-D experiments.
