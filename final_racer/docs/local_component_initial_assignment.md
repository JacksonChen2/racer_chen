# Local-component initial assignment

Select the algorithm with:

```bash
RACER_EXPLORATION_ASSIGNMENT_MODE=local_component
```

The dedicated 10-UAV, five-takeoff-site, 300-second entry point is:

```bash
scripts/run_sionna_local_component_10uav_5sites_300s.sh --run
```

It fixes UAV transmit power at the experiment default of 23 dBm unless
`RACER_UAV_TX_POWER_DBM` is explicitly set, and always disables the former
perfect-delivery startup window.

## Startup protocol

1. Each UAV periodically advertises which UAV state messages it received
   recently through the normal Sionna communication proxy.
2. UAV A admits an A-B edge only after receiving B's state and seeing A in
   B's recent-neighbor list. This is evidence that both directions worked.
3. Member IDs and positions are gossiped over admitted edges until the local
   connected component has been stable for the configured interval.
4. The lowest ID in that component partitions its own local active HGrid
   using only the component members' positions.
5. The assignment is scoped by an explicit sorted member list. Every member
   relays the same assignment until ACK IDs have propagated across that
   component. No fleet-wide 10/10 ACK is used.
6. A one-member component assigns its local HGrid to itself and immediately
   starts. After component startup completes, the existing versioned
   two-phase pairwise transactions perform all later redistribution.

The mode is intentionally rejected when
`RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY=true`, because that would change
the graph observed during startup.

Relevant parameters in `original_warehouse_simple.yaml` are:

```yaml
fsm.local_component_neighbor_freshness: 0.5
fsm.local_component_stability_duration: 1.0
fsm.local_component_assignment_interval: 0.5
```

Runtime evidence is emitted with the prefixes
`RACER_LOCAL_COMPONENT_DISCOVERY`, `RACER_LOCAL_COMPONENT_ASSIGN`,
`RACER_LOCAL_COMPONENT_APPLIED`, and `RACER_LOCAL_COMPONENT_COMMITTED`.
