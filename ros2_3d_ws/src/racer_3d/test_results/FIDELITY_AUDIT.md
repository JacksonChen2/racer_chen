# RACER 3-D fidelity audit

## Conclusion

The original RACER paper and the C++ source in
`RACER/swarm_exploration` are three-dimensional. They use volumetric
occupancy/ESDF data, `Eigen::Vector3d`, `(dx,dy,dz)` A* expansion, 3-D
frontiers, 3-D HGrid cells and B-spline position control points.

The original repository is a ROS 1 implementation and its simulation uses a
custom quadrotor/map/depth-camera simulator. It is not an original Isaac Sim
or ROS 2 implementation. `ros2_3d_ws` is therefore a clean ROS 2 Humble port
and functional reproduction rather than an unchanged build of the original
C++ nodes.

## Component comparison

| RACER component | Original reference | ROS 2/Isaac reproduction | Fidelity |
|---|---|---|---|
| Volumetric map and ESDF | `plan_env/src/sdf_map.cpp`, paper Sect. VII-A | `voxel_map.py`: probabilistic 3-D occupancy, free/occupied/unknown state, signed EDT ESDF and gradients | Same role and 3-D semantics |
| Sensor fusion | RealSense depth images or custom depth-camera simulation | Isaac `RotatingLidarPhysX` to standard `PointCloud2`, 3-D ray integration and hit/miss evidence | Equivalent volumetric input; sensor type differs |
| Frontier information | `active_perception/src/frontier_finder.cpp`, paper Sect. VI-A | Six-connected free/unknown boundary extraction, 3-D clustering, spherical viewpoints, visibility and gain | Same exploration primitive |
| HGrid | `active_perception/src/hgrid.cpp`, paper Sect. IV | Online 3-D coarse-to-fine, eight-child subdivision, unknown centroids/demand and inherited persistent ownership | Same hierarchy and task unit |
| Pairwise allocation | `fast_exploration_fsm.cpp`, paper Alg. 2 | Asynchronous ROS topic state, deterministic non-conflicting pair slots and persistent complementary ownership updates | Same decentralized pairwise behavior; transport/scheduling implementation differs |
| CVRP/coverage path | `fast_exploration_manager.cpp`, LKH3, paper Sect. V | Capacity-balanced two-vehicle open paths; exact Held-Karp subset routes for up to ten cells and bounded fallback above that; HGrid visibility detour costs | Same objective; solver differs from LKH3 |
| Local path | `path_searching/src/astar2.cpp` | 26-connected `(x,y,z)` A*, unknown-space policy, occupied inflation and strict diagonal corner checks | Same 3-D path-search role |
| Viewpoint/path hierarchy | paper Sect. VI-B | HGrid route rank guides frontier choice; top viewpoints are ranked by visible unknown gain and travel cost | Same hierarchy, lighter graph implementation |
| B-spline trajectory | `bspline`, `bspline_opt`, paper Sect. VI-B3 | Clamped cubic 3-D B-spline, iterative smoothness/ESDF control-point optimization, safe polyline fallback and velocity/acceleration time scaling | Same trajectory representation and constraints; optimizer differs from NLopt |
| Obstacle avoidance | original ESDF B-spline collision term | ESDF-constrained planning plus execution CBF with measured stopping distance | Equivalent safety purpose with an additional feedback layer |
| Inter-UAV avoidance | shared B-spline penalty `Jc,q`, paper Eq. 13-14 | time-stamped trajectory conflict yield, 3-D CBF projection and emergency separation | Same decentralized safety purpose; mathematical implementation differs |
| Vehicle control | original geometric controller on custom quadrotor | 27 g Crazyflie 2.x PhysX rigid body, gravity, rotor mixing, thrust/torque saturation and geometric attitude/velocity controller | Real six-DOF simulated dynamics; not a hardware radio link |
| Map exchange | original chunk ledger over UDP/LCM | compressed voxel-state broadcasts over ROS 2 topics | Same shared-map outcome; reliability protocol differs |
| State estimation | Omni-Swarm in real experiments | perfect common Isaac world frame/odometry | Not reproduced; simulation supplies ground truth |

## Important scope statements

- The implementation is architecture- and behavior-faithful, but it is not
  bit-for-bit or solver-for-solver identical to the ROS 1 source.
- The original RACER simulation is not Isaac Sim. Isaac Sim 5.1, ROS 2 Humble,
  standard ROS messages and the Crazyflie plant are additions in this port.
- The acceptance scenario requires vertical motion: one route requires flying
  above a low wall and another requires flying below a suspended wall.
- Neither the RACER paper nor the original exploration FSM defines a mandatory
  return-to-home phase. The reproduced agents hover after completion.
- Physical Crazyflie deployment still requires a hardware bridge and a
  real-world localization pipeline; the included controller is the Isaac
  six-degree-of-freedom plant interface.

## Validation evidence

The formal 120-second Isaac/ROS 2 run is recorded in
`ISAAC_15X9X2_RESULT.json`. It used three vehicles, achieved 97.11% free-space
volume coverage, reached 90% in 37.10 simulated seconds, reported zero PhysX
contacts and maintained 0.639 m minimum inter-UAV distance and 0.153 m minimum
obstacle clearance.
