# hybrid_communication_racer

This workspace is an independent copy of `ros2_pairwise_robust_racer_ws`. It adds a
perfect-communication exploration assignment mode without changing RACER's map,
ESDF, kinodynamic A*, B-spline optimization, collision checking, safety supervisor,
trajectory execution, sensors, or mapping pipeline.

## Modes

- `exploration_assignment_mode=original`: current pairwise-robust RACER allocation,
  original local no-grid/finish behavior, and no hybrid DDS endpoints.
- `exploration_assignment_mode=global_cooperative`: direct perfect-communication
  state exchange, a coordinator-owned de-duplicated global HGrid pool, spatial
  clustering, coverage-gain-per-time matching, HELP/reassignment, and a global
  finish decision.

Set the mode for the standard runner with:

```bash
RACER_COMMUNICATION_MODE=ideal \
RACER_EXPLORATION_ASSIGNMENT_MODE=global_cooperative \
./run_warehouse_simple_sionna.sh
```

## Utility and assignment

For candidate cluster `f` and UAV `i`, the matcher uses:

```text
U(i,f) = gain_scale * unknown_voxels(f) / (path_cost(i,f) / nominal_velocity + epsilon)
         - lambda_overlap * overlap_penalty(i,f)
         + lambda_age * age(f)
```

It performs deterministic greedy maximum-weight one-to-one matching. The overlap
term is recomputed after every accepted match, so later UAVs are discouraged from
selecting a cluster close to either a committed target or a target selected earlier
in the same joint epoch.

The adaptive priority radius is:

```text
R(C) = distance_min + (distance_max - distance_min) * C ^ distance_gamma
```

`C` is the monotonic envelope of a shared-HGrid unknown-count coverage proxy, so
newly exposed frontier regions cannot make the distance limit contract later in a
mission. If the priority radius leaves a UAV idle while eligible work remains, the
optional fallback retries that unmatched row against unmatched tasks up to
`distance_max`.

## Default ROS parameters

| Parameter | Default |
|---|---:|
| `global_cooperative.assignment_interval` | `0.5 s` |
| `global_cooperative.state_freshness` | `0.75 s` |
| `global_cooperative.nominal_velocity` | `1.5 m/s` |
| `global_cooperative.gain_scale` | `0.001` |
| `global_cooperative.utility_epsilon` | `0.1 s` |
| `global_cooperative.lambda_overlap` | `2.0` |
| `global_cooperative.lambda_age` | `0.05` |
| `global_cooperative.overlap_distance` | `6.0 m` |
| `global_cooperative.cluster_distance` | `3.0 m` |
| `global_cooperative.distance_min` | `8.0 m` |
| `global_cooperative.distance_max` | `45.0 m` |
| `global_cooperative.distance_gamma` | `0.7` |
| `global_cooperative.distance_fallback_when_idle` | `true` |
| `global_cooperative.finish_confirmation_cycles` | `6` |
| `global_cooperative.local_failures_before_report` | `2` |
| `global_cooperative.unreachable_distinct_uavs` | `2` |
| `global_cooperative.unreachable_global_failures` | `4` |
| `global_cooperative.unreachable_retry_s` | `20.0 s` |

## Diagnostics

The launch log contains structured prefixes `RACER_HYBRID_POOL`,
`RACER_HYBRID_TARGET`, `RACER_HYBRID_STATS`, `RACER_HYBRID_HELP`,
`RACER_HYBRID_REASSIGN`, `RACER_HYBRID_UNREACHABLE_REPORT`, and
`RACER_HYBRID_GLOBAL_UNREACHABLE`. Together they report pool/cluster sizes,
targets, utility components, radius, idle time, reassignment/help/takeover counts,
and per-UAV marginal unknown-voxel contribution.

## Implementation map

- `src/hybrid_communication_racer`: dedicated messages and the deterministic
  clustering/utility/matching library. Its tests do not depend on the planner.
- `fast_exploration_fsm.cpp`: mode gate, direct perfect-communication state,
  coordinator pool, matching, HELP/reassignment, failure recovery, global finish,
  and structured statistics.
- `fast_exploration_manager.cpp`: two read-only adapters to refresh the existing
  HGrid pool and reuse the existing grid path cost.
- `original_racer_warehouse_sionna.launch.py`: declares the mode and all
  `global_cooperative.*` ROS parameters.
- `run_warehouse_simple_sionna.sh`: validates and forwards
  `RACER_EXPLORATION_ASSIGNMENT_MODE`.

The coordinator and utility layer do not call or replace kinodynamic A*, the
B-spline optimizer, ESDF, collision checking, safety, or trajectory execution.

Run the reproducible short 10-UAV A/B smoke test with:

```bash
RACER_SMOKE_DURATION_S=30 ./run_hybrid_communication_racer_10uav_smoke.sh
```

It uses the verified five-site takeoff layout in
`config/hybrid_communication_racer_10uav_smoke_layout.json` and stores both raw
result/log pairs plus `comparison.json` under `experiments/hybrid_communication_racer_10uav_smoke_*`.

Two 30-second current-source repeats are summarized in
`experiments/hybrid_communication_racer_10uav_smoke_30s_current_source_aggregate.json`.
Every case executed all 10 UAVs without a process crash or ideal-link loss. Mean
observed joint coverage was 18.562% in `original` and 20.576% in
`global_cooperative` (+2.014 percentage points). Mean 5--20 second growth rose
from 0.642 to 0.675 percentage points/second; one repeat was -0.021 and one was
+0.086, so the short-window slope is directionally positive but noisy. The two
cooperative runs made 27/20 new joint-target decisions, with no selected target
pair under 3 m and no FINISH before global confirmation. This is a short smoke
trend, not a claim about converged final coverage; formal final-coverage and
late-mission HELP comparison still requires full-duration repeated-seed runs.
The fair runs contained 4/1 unreachable reports but no takeover, so a longer or
fault-injected run is also required to validate cross-UAV takeover end to end.
