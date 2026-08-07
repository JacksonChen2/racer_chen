# RACER 3-D C++ verification report

This file records tests of `ros2_3d_C_ws` only.  Historical Python/Isaac
results from `ros2_3d_ws` are not presented as C++ results.

## Build and algorithm tests

- Platform: Ubuntu 22.04.5, ROS 2 Humble, GCC 11.4 / C++17
- `rosdep check --from-paths src --ignore-src`: PASS
- `colcon build --symlink-install`: PASS
- `colcon test-result --verbose`: **12 tests, 0 errors, 0 failures**
- SO3 asset validation: **4 tests passed**, plus model/URDF consistency PASS
- Python -> C++ -> Python zlib/Base64 map payload round trip: PASS
- Warehouse-size 650,385-voxel initial ESDF benchmark: about 40.7 ms

The unit tests cover volumetric ray integration, exact 3-D ESDF, frontier
extraction, Warehouse launch-pose separation, vertical A*, corner checks,
3-D B-spline endpoints/timing, HGrid allocation, CBF response and
space-time trajectory conflicts.

## Three-UAV ROS integration

- Backend: C++ analytic 3-D lidar and acceleration-limited point-mass plant
- Scene: obstacle-containing 15 m x 9 m x 2 m acceptance scene
- Verdict: **PASS**
- Aggregate free-space coverage: 91.3006%
- Time to 90%: 40.50 s simulated
- Collision/contact events: 0 / 0
- Minimum inter-UAV distance: 0.6784 m
- Minimum obstacle clearance: 0.1819 m
- Free-space accuracy: 99.9389%
- Occupied precision: 99.8162%
- Obstacle-surface recall: 84.3272%
- Fleet flight distance: 30.0147 m
- All three C++ agents reported completion and cleared their active plan.

The machine-readable result is `MOCK_15X9X2_CPP_RESULT.json`.

This mock result demonstrates ROS topic and algorithm integration and is
supplemented by the formal Isaac run below.

## Isaac Warehouse 60-second smoke test

- ROS graph: three C++ RACER agents, C++ monitor and Isaac 5.1 ROS bridge
- Sensor: three PhysX rotating 3-D lidars, 357 point-cloud frames
- Motion: three 27 g six-DOF Crazyflie-like PhysX bodies
- Bounded known-volume coverage after 60 s: 20.3043%
- Collision/contact events: 0 / 0
- Minimum inter-UAV distance: 0.5478 m
- Minimum obstacle clearance: 0.3485 m
- Fleet flight distance: 59.9636 m
- All three C++ agents were mapping, planning and reporting.

The smoke result is intentionally `passed=false`: 60 s is far below the
900 s Warehouse acceptance window and did not reach 90% coverage.  It confirms
the Isaac/ROS/C++ data and control loop plus early-flight safety; it is not a
completed exploration result.  The machine-readable record is
`ISAAC_WAREHOUSE_CPP_SMOKE_RESULT.json`.

## Initial RACER SO3 loaded-Warehouse smoke test

- Scene: `warehouse_loaded.usd`
- ROS graph: three C++ RACER agents, C++ monitor and Isaac 5.1 bridge
- Vehicle: generated 0.98 kg RACER SO3 plus-quadrotor USD
- Dynamics: source `kf`/`km` mixer, motor RPM lag/limits, SO3 attitude torque
  and quadratic drag at 200 Hz
- Sensor/contact model: three PhysX 3-D lidars and 21 shape-level contact
  sensors
- Duration: 20.0 s simulated
- Bounded known-volume coverage: 14.8597%
- Fleet flight distance: 19.1268 m
- Physics contacts / collision events: 0 / 0
- Minimum inter-UAV distance: 11.1031 m
- Minimum vehicle-surface obstacle clearance: 0.1547 m
- Final positions: `(-23.125, 12.079, 2.441)`,
  `(-8.752, 8.598, 3.174)`, `(2.777, 10.755, 4.147)` m
- All three agents were reporting and retained active exploration plans.

The vehicle mass read back from PhysX was 0.980000019 kg.  All three vehicles
responded to nonzero C++ commands through rotor forces and attitude torques;
the result contains zero contacts/collisions.  As with the earlier short run,
`passed=false` only means that 20 s did not satisfy the formal 90% exploration
threshold.  The machine-readable record is
`ISAAC_WAREHOUSE_LOADED_SO3_CPP_SMOKE_RESULT.json`.

## RACER SO3 600-second migration baseline

This was an unlowered 90% acceptance attempt with the generated model. It ran
before the Isaac `/clock` correction described below.

| Metric | Requirement | Measured |
|---|---:|---:|
| Bounded observed-volume coverage | >= 90% | 77.2037% |
| Simulated duration | <= 600 s | 600.005 s |
| Physics contacts | 0 | 0 |
| Collision events | 0 | 0 |
| Minimum inter-UAV distance | >= 0.60 m | 1.5440 m |
| Minimum vehicle-surface obstacle clearance | >= 0.02 m | 0.0660 m |
| PhysX 3-D point-cloud frames | report | 3,600 |
| Fleet flight distance | report | 530.4249 m |

All agents continued reporting `planning=true` with 91--95 frontier clusters,
but coverage plateaued near 77%. This is a useful migration failure, not a
passing result: dynamics, mapping transport, planning execution and physical
safety remained active, while exploration completion was not met. The
machine-readable record is
`ISAAC_WAREHOUSE_LOADED_SO3_CPP_RESULT.json`.

The first hypothesis was self-occlusion by the larger collision model. A
same-scene probe rejected it: the SO3 and Crazyflie profiles produced 2,137
and 2,127 usable points respectively from the same 3,000-ray PhysX buffer.
The important difference was time. The 200 Hz SO3 simulation took 1,511.98
wall seconds for 600 simulated seconds, while the C++ agents originally
timestamped trajectories and peer state with wall time. Trajectory time
therefore advanced about 2.52 times faster than the actual vehicle.

## Physics-clock regression after the migration fix

The Isaac bridge now publishes physics time on `/clock`; only the C++ agents
enable `use_sim_time`, while the monitor retains independent wall timing.

- Duration: 20.0 s simulated / 49.35 s wall
- Coverage: 11.5938%
- Fleet flight distance: 18.5727 m
- Physics contacts / collision events: 0 / 0
- Minimum inter-UAV distance: 7.7806 m
- Minimum vehicle-surface obstacle clearance: 0.1261 m
- All agents reporting with `planning=true`

This regression verifies the corrected clock/control/map path but is too
short to validate 90% completion. Its machine-readable record is
`ISAAC_WAREHOUSE_LOADED_SO3_CPP_CLOCK_SMOKE_RESULT.json`. A new formal
physics-clock run is still required before marking the generated SO3 vehicle
fully accepted in `warehouse_loaded`.

## ROS1 simulation-profile migration and 10-second performance test

The current profile deliberately uses the original RACER simulation values,
not the paper's real-aircraft device settings:

There are two upstream simulation launch paths. The documented multi-UAV
example uses `simulator_light.xml` (100 Hz kinematic command-to-odometry and
10 Hz depth) and therefore has no SO3 dynamics to transfer. Since this task
requires the generated SO3 vehicle, the table below takes dynamics and sensor
rates from the full `simulator.xml` branch, then applies the current multi-UAV
RACER planner/map settings. The upstream repository has no single launch that
combines those two pieces, so this is an explicit simulation-only composition,
not a claim that such a launch existed originally.

| Subsystem | ROS1 simulation source | Isaac/ROS2 value |
|---|---|---:|
| Rigid-body/motor update | SO3 simulator default | 1,000 Hz |
| Odometry and IMU | RACER simulator launch | 200 Hz |
| Depth sensing | ideal `pcl_render_node` | 30 Hz |
| Renderer image/intrinsics | `camera.yaml` | 640 x 480; 387.229/387.229/321.046/243.449 px |
| Mapper intrinsics | `single_drone_exploration.xml` | 385.69794/385.69794/324.08798/239.10362 px |
| Render/depth-filter/raycast range | renderer + `sdf_map` | 5.0 m / 0.2--4.6 m / 0.5--4.5 m |
| Image filtering | `sdf_map` | border 2 px; `skip_pixel=2` |
| Voxel resolution | simulation launch | 0.1 m |
| Motion limits | simulation launch | 1.5 m/s; 1.0 m/s^2 |
| Obstacle inflation | simulation launch | 0.199 m |
| Swarm safe distance | simulation launch | 1.0 m |
| Frontier/planning period | exploration FSM | 0.5 s |

Isaac renders each complete calibrated image. To keep the test feasible on
the 8 GB RTX 4060, a separate adapter then selects at most 2,400 uniformly
distributed image rays from up to 75,684 candidates after the original
border/skip pattern, for ROS2 point-cloud transport and C++ ray integration.
This cap is not an upstream parameter and is the main fidelity/performance
compromise in this test. The old 360-degree PhysX lidar remains available only
in the explicitly selected legacy Crazyflie profile.

The renderer and mapper intrinsics above are intentionally different: that is
how the checked-in ROS1 launch files are configured. The Isaac camera uses
the renderer values, while depth-to-map projection uses the mapper values.
The small upstream calibration mismatch is therefore preserved rather than
silently corrected.

Vehicle geometry also required two environment adaptations. The ideal ROS1
renderer has no self geometry, so Isaac returns inside the generated
0.332 m visual envelope are removed. The historical Warehouse starts at
`y=7.5` were only 0.10--0.224 m from captured occupied surfaces and overlapped
the generated vehicle's 0.284 m collision radius. Moving them to `y=8.0`
preserved their `x/z` positions and increased the captured-map nearest
surface distances to 0.60--0.63 m.

The resulting three-UAV, 10 s physics-time test measured:

| Metric | Measured |
|---|---:|
| Observed-volume coverage | 5.2499% / 276,403 voxels |
| Fleet flight distance | 22.9078 m |
| Physics contacts / collisions | 0 / 0 |
| Minimum inter-UAV distance | 11.3605 m |
| Minimum vehicle-surface obstacle clearance | 0.1234 m |
| Depth point-cloud frames | 885 / about 900 (98.32%) |
| Physics-time / measured run wall-time | 10.001 s / 126.70 s |
| Real-time factor | 0.0789x |
| Total wall-time including Isaac/RTX startup | 168.32 s |
| Peak process RSS | 7,496,256 KiB (about 7.15 GiB) |
| Low-level safety intervention fraction | 15.00% of fleet control steps |

All agents reported and were actively planning at the final snapshot. The
minimum clearances exceeded their acceptance thresholds and no contact
occurred. `passed=false` is expected because a 10 s performance smoke test
cannot reach the formal 90% full-volume coverage target. The machine-readable
record is
`ISAAC_WAREHOUSE_LOADED_ROS1_SIM_PROFILE_10S_RESULT.json`.

This test is suitable for checking migration correctness, early exploration,
safety and throughput. It is not a new formal acceptance result and should
not be compared directly with the earlier 0.2 m/lidar runs because both the
sensor FoV and voxel count changed.

## Five-UAV ROS1 simulation-profile test in `warehouse_simple`

This is the full-duration follow-up using the simulation-only composite profile
defined above, the generated 0.98 kg RACER SO3 vehicle and five C++ RACER
agents. It used the supplied `warehouse_simple.usd`, stopped automatically at
90% observed-volume coverage, and retained the 900 s simulated-time ceiling.
The only deliberate sensor migration compromise remained the 2,400-ray cap per
rendered 640 x 480 depth frame.

- Verdict: **FAIL** (coverage and inter-UAV separation passed; physical safety
  did not)
- Stop reason: 90% coverage reached before the 900 s ceiling
- Completion time: 252.087 s simulated / 3,679.05 s wall
- Full process wall time including startup and shutdown: 3,708 s (1:01:48)
- Real-time factor: 0.06857x
- Peak process RSS: 5,787,748 KiB (about 5.52 GiB)

| Metric | Requirement | Measured |
|---|---:|---:|
| Bounded observed-volume coverage | >= 90% | 90.0691% |
| Physics contacts / collision events | 0 / 0 | 388 / 388 |
| Maximum contact force | report | 3,048.59 N |
| Minimum inter-UAV distance | >= 0.60 m | 0.9651 m |
| Minimum vehicle-surface obstacle clearance | >= 0.02 m | 0.00457 m |
| Depth point-cloud delivery | report | 37,120 / 37,860 (98.05%) |
| Fleet flight distance | report | 1,028.7521 m |
| Low-level safety intervention fraction | report | 7.386% |

| Vehicle | Flight distance | State at final status snapshot |
|---|---:|---|
| drone_0 | 285.5231 m | planning |
| drone_1 | 8.4196 m | inactive |
| drone_2 | 170.4767 m | inactive |
| drone_3 | 286.1374 m | planning |
| drone_4 | 278.1952 m | planning |

The fleet met the exploration target despite two agents becoming inactive, but
the result is not physically acceptable. Contacts began around 57.4 s and the
worst contact force reached 3.05 kN. The remaining three agents continued to
increase coverage, while the contact count stopped at 388. The final per-agent
status messages are slightly behind the monitor's global map integration at
shutdown (89.82--89.92% versus the final 90.0691%); the monitor's global
coverage is the stopping and acceptance value.

This run also exposes a significant throughput limitation: five calibrated
30 Hz depth streams and 1,000 Hz SO3 physics ran at only 0.06857x real time on
the test machine. The machine-readable source of truth is
`ISAAC_WAREHOUSE_SIMPLE_ROS1_SIM_PROFILE_5UAV_SO3_CPP_900S_RESULT.json`.

## Formal three-UAV Isaac Warehouse acceptance

- Platform: Ubuntu 22.04.5, ROS 2 Humble, Isaac Sim 5.1, RTX 4060 Laptop
- Scene: supplied `warehouse_simple.usd`
- Exploration volume: 19.4 m x 29.8 m x 9.0 m
- Vehicles: three 27 g six-DOF Crazyflie-like PhysX bodies
- Sensor: one 360 x 120 degree PhysX rotating lidar per vehicle
- Requested maximum duration: 900 s
- Verdict: **PASS**

| Metric | Requirement | Measured |
|---|---:|---:|
| Bounded observed-volume coverage | >= 90% | 90.1026% |
| Time to 90% | report | 458.86 s simulated |
| Physics contacts | 0 | 0 |
| Collision events | 0 | 0 |
| Minimum inter-UAV distance | >= 0.35 m | 0.5047 m |
| Minimum obstacle clearance | >= 0.02 m | 0.2494 m |
| 3-D lidar frames | report | 2,754 |
| Fleet flight distance | report | 470.6827 m |

| Vehicle | Flight distance | Final position `(x,y,z)` |
|---|---:|---|
| drone_0 | 156.6839 m | (1.919, 7.816, 3.100) m |
| drone_1 | 156.3766 m | (2.227, 7.170, 3.454) m |
| drone_2 | 157.6223 m | (6.102, 14.350, 4.137) m |

All three C++ agents reported `completed=true`, cleared their plan and
published hover commands after the monitor broadcast mission completion.
They do not return to their launch positions because return-to-home is not an
exploration phase in RACER.

The machine-readable source of truth is
`ISAAC_WAREHOUSE_CPP_RESULT.json`.  Warehouse has no independent labelled
voxel truth, so free-space accuracy, occupied precision and surface recall are
intentionally `null`; deterministic acceptance scenes remain the map-quality
tests.

## Formal five-UAV Isaac Warehouse acceptance

- Platform, scene, vehicle model and sensor: same as the three-UAV run
- Requested maximum duration: 900 s
- Verdict: **PASS**

| Metric | Requirement | Five UAVs | Three UAVs |
|---|---:|---:|---:|
| Bounded observed-volume coverage | >= 90% | 90.2522% | 90.1026% |
| Time to 90% | report | 330.42 s | 458.86 s |
| Wall elapsed time | report | 343.71 s | 466.20 s |
| Physics contacts | 0 | 0 | 0 |
| Collision events | 0 | 0 | 0 |
| Minimum inter-UAV distance | >= 0.35 m | 0.5296 m | 0.5047 m |
| Minimum obstacle clearance | >= 0.02 m | 0.1897 m | 0.2494 m |
| 3-D lidar frames | report | 3,305 | 2,754 |
| Fleet flight distance | report | 557.1724 m | 470.6827 m |

| Vehicle | Flight distance | Final position `(x,y,z)` |
|---|---:|---|
| drone_0 | 112.0785 m | (7.419, 9.998, 1.983) m |
| drone_1 | 112.4014 m | (6.901, 13.706, 7.008) m |
| drone_2 | 110.8719 m | (1.895, 14.374, 6.355) m |
| drone_3 | 111.8168 m | (-2.574, 9.861, 5.856) m |
| drone_4 | 110.0039 m | (-0.564, 8.926, 3.367) m |

Five vehicles reduced completion time by 27.99%, a 1.389x speedup over the
three-vehicle C++ run. Fleet travel increased by 18.38%, while mean travel per
vehicle fell from 156.894 m to 111.434 m (28.97% lower). Wall elapsed time
fell by 26.27%. All five agents reported `completed=true`,
`planning=false` after mission completion.

The machine-readable source of truth is
`ISAAC_WAREHOUSE_CPP_5UAV_RESULT.json`.
