# RACER ROS 2 + Isaac Sim + Sionna RT hybrid communication

This package is an isolated, communication-aware variant of the C++ ROS 2
RACER port. It targets `warehouse_loaded.usd`; it does not modify or depend on
the behavior of the original `ros2_3d_C_ws` package.

## What changed

The agents no longer subscribe to global peer state, full peer maps, or direct
pairwise-allocation topics. Every inter-agent message is serialized as a
`racer_3d_interfaces/msg/CommPacket` and must pass through the communication
emulator before it reaches `/drone_<id>/comm/rx`.

The hybrid channel has two layers:

1. `sionna_channel_node.py` obtains the true drone poses from Isaac odometry and
   uses Sionna RT to estimate directed link path gain, RSS, SNR, Doppler, and
   LOS state. A precomputed radio-map cache may supply the spatial baseline;
   low-rate exact path solves update a correction term while drones move.
2. `communication_emulator_node.cpp` turns link quality into finite throughput,
   queueing delay, jitter, MTU-dependent packet error probability, packet loss,
   TTL expiry, and bounded reliable retries.

RACER map exchange uses 200-voxel incremental chunks plus compact manifests.
Missing chunks are requested indirectly by rebroadcasting stored chunks when a
manifest exposes a gap. Task allocation uses proposal/ACK/commit/commit-ACK so
one lost packet cannot silently leave two drones with inconsistent ownership.
The full fused map is published only on an evaluator topic; other agents never
subscribe to that topic.

## Configuration provenance

`config/warehouse_loaded_agent.yaml` migrates the original RACER simulator
profile used by this repository: 0.10 m voxels, 0.5--4.5 m sensing range,
2400 rays, 1.5 m/s maximum velocity, 1.0 m/s^2 maximum acceleration, and
200-voxel map messages. These are simulation parameters, not real-vehicle
parameters.

The initial RF assumptions are deliberately separate in
`config/warehouse_loaded_communication.yaml`: 2.4 GHz carrier, 20 MHz
bandwidth, 20 dBm transmit power, and 7 dB receiver noise figure. RACER did not
define a radio/electronics model, so these values must be calibrated for the
radio and antenna being studied. The USD-to-Sionna converter assigns ITU radio
materials heuristically from USD names; inspect `conversion_report.json` and
override wrong material assignments before treating RF results as measured
performance.

## Build and run

From `RACER/ros2_3d_sionna_ws`:

```bash
./setup_sionna_env.sh
./prepare_warehouse_sionna_scene.sh
./build.sh
./run_warehouse_loaded_hybrid.sh
```

The runner uses three drones, a 900-second limit, headless Isaac Sim, the RACER
SO(3) vehicle, and `sionna_hybrid` communication. Override settings with, for
example:

```bash
HEADLESS=0 DURATION=120 DRONE_COUNT=2 ./run_warehouse_loaded_hybrid.sh
```

Logs are written below `logs/`; the monitor result defaults to
`results/warehouse_loaded_sionna.json`. The monitor stops at the duration limit
or 90% coverage. Isaac's external swarm safety is disabled so a global-truth
safety filter does not hide communication failures; local stale-peer projection
and uncertainty inflation remain active in each RACER agent.

The supplied `warehouse_loaded.usd` is an overlay with a relative
`warehouse.usd` dependency. The preparation and run scripts therefore compose
it with `../ros2_3d_py_ws/warehouse.usd` by default; set `WAREHOUSE_BASE_USD`
when that base layer lives elsewhere. The Sionna conversion uses per-mesh
oriented bounding-box RF proxies to keep ray tracing tractable. Isaac continues
to load the original visual/collision geometry.

## Offline radio-map cache

For repeated experiments, generate a baseline cache once:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=$PWD/.sionna_runtime${PYTHONPATH:+:$PYTHONPATH}
python3 install/racer_3d_sionna_comm/lib/racer_3d_sionna_comm/generate_hybrid_radio_cache.py \
  --scene-xml src/racer_3d_sionna_comm/assets/warehouse_loaded_sionna/warehouse.xml \
  --output src/racer_3d_sionna_comm/assets/warehouse_loaded_sionna/hybrid_radio_cache.npz
```

The default grid is intentionally high fidelity and expensive. Use coarser TX
spacing, RX cell size, fewer heights, or fewer ray samples for a smoke test.
This workspace already includes a generated default cache with 162 TX anchors,
five RX height slices, a 0.5 m RX grid, 200,000 samples per solve, path depth
two, and random seed 42. Regenerate it after changing RF materials, scene
geometry, carrier frequency, or mission bounds.

## Comparison modes

`communication_mode` accepts:

- `ideal`: no channel loss, useful as the upper-bound regression.
- `range_only`: deterministic distance gate, useful for logic tests.
- `sionna`: exact/current Sionna link quality without cache correction.
- `sionna_hybrid`: cached spatial baseline plus periodic exact Sionna updates.

For a controlled experiment, keep the scene, seeds, starts, dynamics, and
sensing parameters fixed and compare at least `ideal`, `range_only`, and
`sionna_hybrid`. Report coverage, completion time, minimum separation, collision
contacts, delivered-byte ratio, drop causes, queue delay, retries, and link
availability.
