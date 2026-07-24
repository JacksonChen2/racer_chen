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
    grid_assignments: dict[int, list[int]] = field(default_factory=dict)
    state_timeout: float = 2.0

    def update_state(
        self, drone_id: int, state: VehicleState, grid_ids: list[int] | None = None
    ) -> None:
        if drone_id != self.drone_id:
            self.states[int(drone_id)] = state
            if grid_ids is not None:
                self.grid_assignments[int(drone_id)] = [
                    int(value) for value in grid_ids
                ]

    def active_states(self, own_state: VehicleState) -> dict[int, VehicleState]:
        result = {self.drone_id: own_state}
        result.update(
            {
                drone_id: state
                for drone_id, state in self.states.items()
                if own_state.stamp <= 0.0
                or state.stamp <= 0.0
                or own_state.stamp - state.stamp <= self.state_timeout
            }
        )
        return result

    def allocate(
        self,
        own_state: VehicleState,
        viewpoints: list[Viewpoint],
        graph: ViewGraph,
        item_ids: list[int] | None = None,
        consistency_bonus: float = 0.0,
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
                if (
                    item_ids is not None
                    and frontier_id < len(item_ids)
                    and item_ids[frontier_id]
                    in self.grid_assignments.get(drone_id, [])
                ):
                    cost -= consistency_bonus
                if cost < winner_cost:
                    winner, winner_cost = drone_id, cost
            loads[winner] = winner_cost
            if winner == self.drone_id:
                assigned.append(frontier_id)
        return assigned
