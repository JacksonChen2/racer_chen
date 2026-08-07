# RACER 3-D C++ / ROS 2 Humble / Isaac Sim

`racer_3d_cpp` is the C++17 port of the three-dimensional exploration stack
in `RACER/ros2_3d_ws`.  The mapping, frontier processing, hierarchical
allocation, path planning, trajectory generation and agent-side safety code
run in native ROS 2 C++ nodes.

## What is implemented in C++

- probabilistic `(x,y,z)` log-odds voxels and an exact O(N) 3-D ESDF;
- 3-D lidar/depth point-cloud ray integration;
- volumetric frontier extraction, clustering, spherical viewpoint sampling,
  visibility and information gain;
- octree-like 3-D HGrid cells, persistent ownership and pairwise
  capacity-constrained open-route allocation;
- 26-connected 3-D A* with edge/corner checks, path shortening, cubic
  B-splines, ESDF control-point optimization and dynamic time scaling;
- predicted trajectory conflict detection, 3-D inter-UAV CBF separation,
  emergency separation and stopping-distance ESDF filtering;
- C++ ROS agents, analytic mock backend and acceptance monitor.

The ROS contracts remain compatible with the existing Isaac adapter:

- `/drone_N/odom`, `/drone_N/points` -> C++ agent;
- `/drone_N/cmd_vel_3d`, `/drone_N/planned_path_3d`,
  `/drone_N/occupied_voxels`, `/drone_N/status` <- C++ agent;
- `/racer_3d/map_share`, `/racer_3d/swarm_state`,
  `/racer_3d/pairwise` for decentralized coordination;
- `/racer_3d/sim_metrics`, `/racer_3d/mission_complete` for acceptance.

`Twist.linear` is a velocity in the shared `map` frame.  For compatibility
with the validated Isaac bridge, `Twist.angular.z` is an absolute yaw target,
not a ROS-conventional yaw rate.

## Why one Python launcher remains

Isaac Sim 5.1 exposes the used SimulationApp/PhysX sensor API through Python.
`isaac_sim_racer_3d_cpp.py` only runs that simulator/ROS bridge.  It does not
run the RACER exploration algorithm.  The bridge and its four small support
modules are stored inside this package; all RACER agent callbacks are C++.

The default vehicle is the generated RACER SO3 plus-quadrotor USD.  The bridge
checks its 0.98 kg imported mass, uses the source simulator's diagonal inertia,
0.26 m arm, `kf`/`km` mixer, 1/30 s motor lag, 1,200--35,000 RPM limits,
attitude/rate gains and quadratic drag, and applies the resulting four-rotor
wrench to the imported `base_link`.  The C++ velocity command is converted to
the desired force and attitude expected by that low-level controller.

`RACER_3D_VEHICLE_MODEL=crazyflie` selects the older 27 g comparison plant.
Neither profile is firmware SITL or a physical flight-controller interface.

The SO3 rigid-body/motor plant runs at the upstream simulator's 1,000 Hz.
Odometry and IMU are published at 200 Hz, matching the RACER simulation
launch, and the depth camera runs at 30 Hz. Isaac publishes physics time on
`/clock`, and launch enables `use_sim_time` on the C++ agents so trajectory
timing, plan reuse and peer expiry do not advance faster than the vehicle.
The acceptance monitor deliberately remains on wall time and also records
simulator-reported elapsed time.

The upstream repository contains two simulation paths. Its README launches
`simulator_light.xml`, which replaces vehicle physics with a 100 Hz
position-command-to-odometry shortcut and renders depth at 10 Hz. Because
this integration explicitly tests the requested SO3 vehicle, it instead
projects the upstream full `simulator.xml` branch: 1,000 Hz dynamics, 200 Hz
odometry/IMU and 30 Hz depth. Planner/map limits come from the current
multi-UAV RACER launch. This choice uses only simulation-side parameters, but
it is necessarily a composition: upstream does not provide a multi-UAV launch
that combines the current RACER planner with the full SO3 plant.

For `racer_so3`, the sensor is not a lidar. It is a forward pinhole depth
camera using the ROS1 renderer calibration: 640 x 480, `fx=fy=387.229`,
`cx=321.046`, `cy=243.449`, and a 5.0 m render horizon. The original mapper
uses its own default intrinsics (`fx=fy=385.69794`, `cx=324.08798`,
`cy=239.10362`), a 0.2--4.6 m depth filter, 0.5--4.5 m ray lengths, a
2-pixel border and `skip_pixel=2`. The voxel resolution is 0.1 m. Isaac
renders the full image, then the adapter uniformly limits the ROS point cloud
to 2,400 rays per frame by default so three cameras and 0.1 m ray integration
fit the available 8 GB GPU/host processing budget. The original border/skip
grid contains up to 75,684 candidates, so that 2,400-ray cap is an explicit
migration/performance adaptation, not an upstream RACER parameter; override
it with `RACER_3D_CAMERA_RAY_BUDGET`.

## Build and unit test

```bash
cd RACER/ros2_3d_C_ws
source /opt/ros/humble/setup.bash
rosdep check --from-paths src --ignore-src
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test
colcon test-result --verbose
```

ROS-only three-UAV integration regression:

```bash
RACER_3D_DURATION=60 ROS_DOMAIN_ID=143 \
  src/racer_3d_cpp/scripts/run_mock_test.sh
```

The mock backend is an engineering regression, not an Isaac/PhysX acceptance
result.

## Isaac Warehouse scenes and RACER SO3 vehicle

`warehouse_simple.usd` and `warehouse_loaded.usd` are small USD layers that
reference Isaac 5.1 Warehouse content and are not self-contained.  The C++
runner resolves the selected scene and the flattened, portable RACER vehicle
asset automatically.

Short SO3 smoke run in the requested loaded Warehouse:

```bash
cd RACER/ros2_3d_C_ws
RACER_3D_SCENARIO=warehouse_loaded \
RACER_3D_DRONE_COUNT=3 \
RACER_3D_DURATION=20 \
RACER_3D_VEHICLE_MODEL=racer_so3 \
ROS_DOMAIN_ID=153 \
  src/racer_3d_cpp/scripts/run_isaac_test.sh \
  src/racer_3d_cpp/test_results/ISAAC_WAREHOUSE_LOADED_SO3_CPP_CLOCK_SMOKE_RESULT.json
```

A smoke result remains `passed=false` unless it reaches the formal 90%
coverage threshold; use its agent, motion, map, contact and clearance fields
to validate the short data/control loop.

Formal SO3 run:

```bash
cd RACER/ros2_3d_C_ws
RACER_3D_SCENARIO=warehouse_loaded \
RACER_3D_DRONE_COUNT=3 \
RACER_3D_DURATION=600 \
RACER_3D_VEHICLE_MODEL=racer_so3 \
ROS_DOMAIN_ID=153 \
  src/racer_3d_cpp/scripts/run_isaac_test.sh \
  src/racer_3d_cpp/test_results/ISAAC_WAREHOUSE_LOADED_SO3_CPP_CLOCK_RESULT.json
```

Override `RACER_SO3_VEHICLE_USD` only when testing a different generated USD.
The default is
`isaac_assets/racer_so3_quadrotor/usd/racer_so3_quadrotor_flattened.usd`.

Interactive Isaac visualization:

```bash
cd RACER/ros2_3d_C_ws
ROS_DOMAIN_ID=172 \
  src/racer_3d_cpp/scripts/run_isaac_visualization.sh
```

This opens a non-headless Isaac Sim window with an overview camera. The
visualization launcher selects
`isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd` by
default. Its four render-only propeller Xforms are driven from the same four
`motor_rpm` states used for RACER thrust and moment. Rotors 0--1 turn clockwise
and rotors 2--3 counter-clockwise; this animation does not add joints,
collisions, mass, thrust or torque. RPM angles are integrated at the 1 kHz
physics rate and USD transforms are refreshed at 60 Hz by default.

Set `RACER_3D_PROPELLER_VISUAL_HZ` to change the transform refresh cap. Set
`RACER_3D_ANIMATE_PROPELLERS=0` to disable it, or `=1` to force it on during a
headless diagnostic. The default `auto` behavior enables it interactively and
disables it headlessly, preserving benchmark performance.

The shared occupied map is cyan; the three vehicles' flown trails and current
plans are red, green, and yellow. The display consumes
`/drone_0/occupied_voxels` and each `/drone_N/planned_path_3d` topic without
changing the map or planner. Only the display is capped, at 40,000 occupied
points by default; override it with
`RACER_3D_VISUALIZATION_MAX_MAP_POINTS` when GPU memory permits. Close the
window or interrupt the launch script to stop the demo and its ROS nodes.

In headless mode only its rendered vehicle geometry is hidden to avoid
material compilation pressure on 8 GB GPUs; rigid body, inertia, collision
shapes, contact sensors and calibrated depth cameras remain active. Returns
inside the generated 0.332 m visual envelope are also removed because the
upstream ideal renderer does not render the carrying vehicle.

The recorded 600 s migration baseline made the larger vehicle fleet fly
530.425 m without a contact or collision, but reached only 77.204% observed
volume and therefore correctly failed the 90% requirement. That run exposed
a wall-time/physics-time mismatch in the initial port. The subsequent
`/clock` fix passed a 20 s three-UAV regression; the formal 600 s result must
be rerun before claiming full loaded-Warehouse acceptance. See
`test_results/TEST_REPORT.md` and the machine-readable SO3 result files.

For the legacy five-UAV Crazyflie comparison:

```bash
RACER_3D_SCENARIO=warehouse_simple \
RACER_3D_DRONE_COUNT=5 \
RACER_3D_DURATION=900 \
RACER_3D_VEHICLE_MODEL=crazyflie \
ROS_DOMAIN_ID=156 \
  src/racer_3d_cpp/scripts/run_isaac_test.sh \
  src/racer_3d_cpp/test_results/ISAAC_WAREHOUSE_CPP_5UAV_RESULT.json
```

The recorded formal three-UAV C++ run reached 90.1026% bounded observed
volume in 458.86 simulated seconds, travelled 470.683 m as a fleet, reported
zero contacts/collisions, maintained 0.505 m minimum inter-UAV distance and
0.249 m minimum obstacle clearance.  See `test_results/TEST_REPORT.md`.

The recorded five-UAV run reached 90.2522% in 330.42 simulated seconds with
zero contacts/collisions. It was 27.99% faster than the three-UAV run. Fleet
travel was 557.172 m, minimum inter-UAV distance was 0.530 m and minimum
obstacle clearance was 0.190 m.

The acceptance monitor requires at least 90% bounded known-volume coverage,
zero collision/contact events, at least 0.60 m inter-UAV separation for the
larger SO3 model (0.35 m for Crazyflie), at least 0.02 m reported obstacle
clearance, odometry from every configured vehicle and status from every C++
agent.  It publishes mission completion to every agent, which then clears its
plan and commands hover.

Warehouse uses `observed_volume` because the referenced mesh has no independent
ground-truth voxel label set.  This verifies coverage and physical safety, but
does not by itself prove free/occupied classification accuracy.  The
deterministic 15x9x2 and 20x50x3 scenes retain analytic map-quality checks.

The current source-calibrated 10 s `warehouse_loaded` performance record is
`test_results/ISAAC_WAREHOUSE_LOADED_ROS1_SIM_PROFILE_10S_RESULT.json`.
It uses three SO3 vehicles and the documented 2,400-ray adapter budget.
It covered 5.2499% (276,403 voxels), flew 22.9078 m, delivered 885 depth
clouds, and had zero contacts/collisions. 10.001 s of physics took 126.70 s
after backend connection (0.0789x real time); total process time including
Isaac/RTX startup was 168.32 s with about 7.15 GiB peak RSS. The short result
is intentionally `passed=false` because it does not claim 90% full-Warehouse
completion.

## Fidelity boundary

The original repository is ROS 1 and does not contain Isaac Sim or ROS 2
nodes.  This package is a ROS 2/Isaac functional reproduction and C++ port of
the existing 3-D implementation; it is not a bit-identical rebuild of every
original solver.  In particular, its small pairwise CVRP solver replaces
LKH3, its B-spline optimizer replaces the original NLopt objective, and ROS 2
map messages replace the original communication transport.

The C++ port currently commands map-frame velocity plus absolute yaw; its
Isaac wrapper synthesizes the SO3 attitude/thrust request. The original ROS1
trajectory server instead sends full position, velocity, acceleration, yaw
and yaw-rate commands, so controller-interface fidelity remains a known
migration boundary even though the plant parameters are source-faithful.
