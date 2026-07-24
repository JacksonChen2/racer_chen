"""Decentralized workload and trajectory bookkeeping for RACER."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .graph import ViewGraph
from .types import VehicleState, Viewpoint


@dataclass(slots=True)
class SwarmCoordinator:
    drone_id: int
    drone_count: int
    states: dict[int, VehicleState] = field(default_factory=dict)

    def update_state(self, drone_id: int, state: VehicleState) -> None:
        if drone_id != self.drone_id:
            self.states[int(drone_id)] = state

    def active_states(self, own_state: VehicleState) -> dict[int, VehicleState]:
        result = {self.drone_id: own_state}
        result.update(self.states)
        return result

    def allocate(
        self,
        own_state: VehicleState,
        viewpoints: list[Viewpoint],
        graph: ViewGraph,
    ) -> list[int]:
        """Return frontier IDs won by this drone.

        Every agent evaluates the same pairwise cost rule. Stable drone-ID
        tie-breaking makes the result decentralized and deterministic under
        asynchronous but eventually consistent state/map exchange.
        """
        states = self.active_states(own_state)
        assigned: list[int] = []
        loads = {drone: 0.0 for drone in states}
        for frontier_id, viewpoint in enumerate(viewpoints):
            winner, winner_cost = self.drone_id, math.inf
            for drone_id in sorted(states):
                state = states[drone_id]
                cost, _ = graph.compute_cost(
                    state.position, viewpoint.position, state.yaw, viewpoint.yaw,
                    state.velocity, state.yaw_rate,
                )
                cost += loads[drone_id]
                if cost < winner_cost:
                    winner, winner_cost = drone_id, cost
            loads[winner] += winner_cost
            if winner == self.drone_id:
                assigned.append(frontier_id)
        return assigned

