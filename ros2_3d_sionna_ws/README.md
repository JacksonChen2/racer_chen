# Communication-aware RACER workspace

This standalone ROS 2 Humble workspace adds a Sionna RT hybrid communication
channel to the C++ RACER/Isaac Sim implementation for
`../warehouse_loaded.usd`.

Quick start:

```bash
./setup_sionna_env.sh
./prepare_warehouse_sionna_scene.sh
./build.sh
./run_warehouse_loaded_hybrid.sh
```

Architecture, parameter provenance, radio-map generation, and comparison modes
are documented in
`src/racer_3d_sionna_comm/README.md`.

Build, Sionna, hybrid-cache, and Isaac smoke-test evidence is recorded in
`VALIDATION.md`.
