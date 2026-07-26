# RACER 3-D Isaac Sim / ROS 2 test report

## 20 m x 50 m x 3 m large-scene acceptance

- Date: 2026-07-26
- Platform: Ubuntu 22.04.5, ROS 2 Humble, Isaac Sim 5.1
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU
- Scene: 20 m x 50 m x 3 m enclosed volume
- Obstacles: full-width low and suspended walls, a central gate, mixed-height
  north barrier, five full-height columns and five partial-height blocks
- Vehicles: three 27 g Crazyflie 2.x six-DOF PhysX rigid bodies
- Sensor: one 360-degree x 120-degree PhysX rotating 3-D lidar per vehicle
- Map: 375,000 voxels at 0.20 m resolution
- Verdict: **PASS**

| Metric | Requirement | Measured |
|---|---:|---:|
| Free-space volume coverage | >= 90% | 90.337% |
| Time to 90% coverage | report | 402.70 s simulated |
| Free-space classification accuracy | >= 95% | 99.928% |
| Occupied precision | >= 75% | 99.677% |
| Obstacle surface recall | >= 35% | 95.532% |
| Physics contacts | 0 | 0 |
| Collision events | 0 | 0 |
| Minimum inter-UAV distance | >= 0.35 m | 0.798 m |
| Minimum obstacle clearance | >= 0.02 m | 0.127 m |

| Vehicle | Distance |
|---|---:|
| drone_0 | 132.427 m |
| drone_1 | 121.813 m |
| drone_2 | 131.429 m |
| Total | 385.668 m |

The mission reached the threshold after 402.70 simulator seconds and 410.02
wall-clock seconds. The result was finalized at 90.337% after a two-second
metrics-settling interval. It contains 339,254 known voxels and 2,415 raw
point-cloud frames. The source of truth is
`ISAAC_20X50X3_RESULT.json`.

The long-scene run exposed two faults that were not visible in the smaller
world. A low-rate command could outlive an expensive planning callback and
overshoot into a floor or suspended wall, and a connected corridor frontier
could span several HGrid regions while all agents selected the same centroid.
The corrected version applies a stopping-distance rigid-body barrier and
inter-UAV CBF at every 50 Hz physics step, and splits connected frontiers at
active HGrid boundaries before applying region ownership. A failed 356.24 s
regression after the safety fix reached 75.02% with zero contacts; it was not
counted as a pass. The final run above is the first full pass.

Reproduction command:

```bash
cd RACER/ros2_3d_ws
RACER_3D_SCENARIO=acceptance_20x50x3 \
RACER_3D_DURATION=600 ROS_DOMAIN_ID=77 \
  src/racer_3d/scripts/run_isaac_test.sh \
  src/racer_3d/test_results/ISAAC_20X50X3_RESULT.json
```

The 600 s value is a maximum simulated mission time. The monitor stops the
run as soon as 90% truth coverage is reached.

## 15 m x 9 m x 2 m baseline acceptance

- Date: 2026-07-26
- Platform: Ubuntu 22.04.5, ROS 2 Humble, Isaac Sim 5.1
- GPU: NVIDIA GeForce RTX 4060 Laptop GPU
- Scene: 15 m × 9 m × 2 m, floor/ceiling/walls, low partition,
  suspended partition, columns and blocks
- Vehicles: three 27 g Crazyflie 2.x six-DOF PhysX rigid bodies
- Sensor: one 360° × 120° PhysX rotating 3-D lidar per vehicle
- Requested run duration: 120 s
- Simulated duration recorded: 118.76 s
- Raw point-cloud frames: 711
- Verdict: PASS

## Metrics

| Metric | Requirement | Measured |
|---|---:|---:|
| Free-space volume coverage | ≥ 90% | 97.11% |
| Time to 90% coverage | report | 37.10 s simulated |
| Free-space classification accuracy | ≥ 95% | 99.83% |
| Occupied precision | ≥ 75% | 99.54% |
| Obstacle surface recall | ≥ 35% | 97.23% |
| Physics contacts | 0 | 0 |
| Collision events | 0 | 0 |
| Minimum inter-UAV distance | ≥ 0.35 m | 0.639 m |
| Minimum obstacle clearance | ≥ 0.02 m | 0.153 m |

## Flight distance

| Vehicle | Distance |
|---|---:|
| drone_0 | 13.045 m |
| drone_1 | 12.403 m |
| drone_2 | 12.049 m |
| Total | 37.497 m |

The reported completion time is the simulator mission clock. Wall time includes
Isaac startup and the monitor guard interval and is intentionally reported
separately in the JSON.

## Verification commands

```bash
cd RACER/ros2_3d_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose

RACER_3D_DURATION=120 ROS_DOMAIN_ID=94 \
  src/racer_3d/scripts/run_isaac_test.sh \
  src/racer_3d/test_results/ISAAC_15X9X2_RESULT.json
```

Unit/algorithm result after the large-scene changes: 14 tests passed. Static
Python style result:
18 files checked, no errors.

The final 60-second ROS-only integration regression also passed:
92.82% volume coverage, 28.50 simulated seconds to 90%, zero collisions,
0.712 m minimum inter-UAV distance and 0.224 m minimum obstacle clearance.
Its raw result is stored in `MOCK_15X9X2_RESULT.json`; it supplements but does
not replace the PhysX acceptance result.

## Safety regression history

Early tests exposed three real faults: the lidar detected the vehicle body,
map-boundary wall hits were discarded, and world-frame angular velocity was
used directly in a body-frame rate controller. Later tests also exposed
insufficient rigid-body braking margin at 0.7 m/s. The final version masks
self returns, preserves boundary hits, transforms angular rate into the body
frame, preserves collective rotor thrust during saturation, limits exploration
speed to 0.35 m/s and uses a 0.45 m velocity-dependent execution clearance.

The formal result file is the machine-readable source of truth. A short test
that does not reach the coverage threshold is expected to fail acceptance and
must not be interpreted as an algorithm failure.
