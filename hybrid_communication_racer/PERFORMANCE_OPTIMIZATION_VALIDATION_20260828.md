# RACER + Isaac Sim + Sionna performance optimization validation

Date: 2026-08-28

## Invariants

- RACER map resolution/range, inflation, ESDF implementation, planner, controller, collision thresholds, and replanning logic were not changed.
- Sionna `max_depth`, `samples_per_source`, propagation options, array/power handling, radio-map correction, noise, MCS, PER, retry, and packet queue behavior remain unchanged in non-ideal modes.
- Existing RACER sensor, map, planner, and communication payload topics remain unchanged. `/racer_sionna/active_links` is an internal demand hint between the proxy and channel node.

## Implementation

- ESDF: occupancy and inflated occupancy still update for every accepted sensor cloud. ESDF is marked dirty, refreshed by a 2 Hz maximum-rate background timer, and refreshed synchronously at the common planning entry if dirty.
- Perfect communication: serialized logical messages are published directly, in callback order, without `DeliveryFlow`, `PendingPacket`, queue, PER, retry, or serialization-delay state. Existing per-receiver packet/byte counters remain, with separate logical-message and equivalent 1200-byte transport-block counters.
- Sionna: queued communication identifies active directed links. The worker retains inactive cache entries, chooses one canonical geometry for each active reciprocal pair, configures only the required endpoint sets, and updates a locked timestamped cache. Direction-specific Tx/Rx power and gain, radio-map correction, RSS/SNR, MCS, and PER remain downstream and separate.
- Async execution: `PathSolver` runs only on `sionna-path-solver`; the ROS executor publishes the newest cache state and requests stale refreshes without joining or waiting for the worker.
- Profiling: ESDF, perfect forwarding, PathSolver, cache lookup/update/publication, active/reciprocal link counts, and Isaac main/world-step wall time are emitted and included in result JSON where applicable.

Sionna RT 2.0.1 exposes endpoint matrices rather than a pair-subset argument. The retained implementation therefore minimizes the active Tx/Rx endpoint matrix in one batched solve and extracts only canonical reciprocal active results. For the dense 10-UAV all-to-all case this is 9 x 9 endpoint entries and 45 retained geometries instead of 10 x 10 entries and 90 directed outputs. A diagnostic decomposition into nine exact one-Tx calls was not retained because it increased mean refresh time from about 135.9 ms to about 1166.8 ms.

## Verification

Build and tests:

- `racer_original_core`, `racer_sionna_comm`, and `racer_isaac_adapter` build successfully.
- 18 `racer_sionna_comm` tests pass: six LinkModel tests, ten live ROS2 proxy interface/order/statistics/active-hint tests, and two active-link/reciprocity layout tests.
- A standalone 10-node live Sionna channel smoke delivered 90 links per publication and converged from `unavailable` to `sionna_exact` with 45 cached reciprocal geometries.

Matched short simulations used Warehouse Full, 10 UAVs, five start sites, 100 Hz physics, 10 Hz 76,800-ray Warp sensing, startup free-yaw/scan/corridor recovery, seed 42, and 12 seconds of simulation.

| Metric | Perfect fast path | Sionna async |
|---|---:|---:|
| Joint known-voxel coverage | 9.84% | 7.29% |
| Original FSMs that entered execution | 10/10 | 1/10 |
| Physical collision events | 0 | 0 |
| Process crashes | 0 | 0 |
| Main-loop wall time | 74.30 s | 74.65 s |
| Main-loop mean step | 61.82 ms | 62.11 ms |
| Real-time factor | 0.1616 | 0.1609 |
| ESDF mean update | 1.64 ms | 1.56 ms |
| ESDF max update | 3.58 ms | 6.59 ms |
| ESDF timer / planner-forced updates | 168 / 138 | 225 / 89 |

The different short-run behavior is expected from communication delivery: perfect delivered all 442,215 attempted receiver packets, while Sionna delivered 187,360 of 625,131 attempts under the unchanged fixed-MCS/PER/no-retry test settings. The 12-second tests are sanity/profiling tests, not completion tests. The acceptance wrapper reports `passed=false` because the complete original pipeline cannot be established in that duration (and only 1/10 Sionna FSMs entered execution); both cases also record the same startup minimum-clearance diagnostic of -0.0558 m despite zero physical collision events, so that pre-existing startup/clearance condition is not attributed to the communication or ESDF changes.

Perfect fast-path profiling:

- 49,135 logical forwarding calls to 442,215 receiver deliveries.
- 443,517,552 attempted and delivered bytes; the fast-path statistical-byte counter is identical.
- 442,269 equivalent 1200-byte transport blocks.
- 0.0171 ms mean and 3.3167 ms maximum forwarding time; zero queued packets and zero PER drops.

Sionna profiling:

- 90 active directed links, 45 reciprocal geometries, and 45 exact-cache entries.
- 16 asynchronous refreshes in the matched simulation.
- 135.94 ms mean / 169.62 ms maximum PathSolver time.
- 0.00769 ms mean cache lookup, 0.0289 ms mean cache update, and 2.61 ms mean link-array construction/publication.
- 9,720 exact-link samples reached the unchanged packet model; the first 1,080 lookups occurred before the first asynchronous result and used the existing unavailable/stale behavior without blocking physics.

Artifacts:

- `experiments/optimization_smoke_10uav_12s_20260828/ideal/warehouse_full_distributed_result.json`
- `experiments/optimization_smoke_10uav_12s_20260828/sionna/warehouse_full_distributed_result.json`
