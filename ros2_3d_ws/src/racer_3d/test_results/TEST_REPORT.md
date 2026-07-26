# RACER 3-D Isaac Sim / ROS 2 test report

## Formal acceptance result

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

Unit/algorithm result: 12 tests passed. Static Python style result:
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
