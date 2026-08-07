# Validation record

Validated on 2026-08-04 with ROS 2 Humble, Isaac Sim 5.1, Sionna RT 2.0.1,
and the local `warehouse_loaded.usd` overlay.

## Completed checks

- `colcon build --symlink-install`: both packages built successfully.
- `colcon test`: 16 tests passed, including sparse-map serialization,
  manifest range validation, mapping, planning, allocation, and safety tests.
- USD conversion: the visual scene was composed with its explicit
  `warehouse.usd` base and converted to a six-material, 90,048-triangle RF
  proxy scene. Sionna RT loaded the generated XML successfully.
- Offline cache generator: the installed default cache contains 162 mobile-TX
  anchors at three heights, a 0.5 m receiver grid at five heights, 200,000 ray
  samples per solve, and maximum path depth two. Its shape is
  `[162, 5, 38, 66]` and its compressed size is 1.9 MB.
- Exact Sionna mock run: the monitor observed `sionna_exact` directed links and
  packets were delivered through the C++ emulator.
- Hybrid mock run: the monitor observed `sionna_cache_corrected`, proving that
  cache interpolation and low-rate exact correction were both active.
- Isaac integration smoke run: two RACER SO(3) drones, original-simulator depth
  profile, PhysX, ROS 2 C++ RACER, and exact Sionna communication ran together
  for two seconds of simulation time in the composed warehouse scene.

The Isaac smoke run produced 118 point-cloud frames, 200 Hz odometry, 30 Hz
sensing, 1000 Hz physics, no collision/contact events, 13.518 m minimum
inter-drone separation, and 0.931 m minimum obstacle clearance. The network
attempted 194 packets and delivered 62 before the short run ended; both agents
reported an active peer. The detailed summary is in
`validation/isaac_2s_smoke.json`.

The smoke run reports `passed: false` by design because a two-second run cannot
reach the 90% coverage acceptance threshold. A 900-second run has not been
claimed here; use `./run_warehouse_loaded_hybrid.sh` for that experiment after
generating the desired-resolution radio-map cache and calibrating RF values.
