"""Pairwise, capacity-constrained workload partitioning."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Sequence, Tuple

from .hgrid import HGridCell


@dataclass
class Partition:
    first: List[str]
    second: List[str]
    first_route: List[str]
    second_route: List[str]
    first_demand: int
    second_demand: int


def _route_length(
    route: Sequence[HGridCell], start: Tuple[float, float]
) -> float:
    total = 0.0
    cursor = start
    for cell in route:
        total += math.hypot(
            cell.centroid[0] - cursor[0],
            cell.centroid[1] - cursor[1],
        )
        cursor = cell.centroid
    return total


def _best_insertion(
    route: Sequence[HGridCell],
    cell: HGridCell,
    start: Tuple[float, float],
) -> Tuple[float, List[HGridCell]]:
    best_cost = math.inf
    best_route: List[HGridCell] = []
    for index in range(len(route) + 1):
        candidate = list(route[:index]) + [cell] + list(route[index:])
        cost = _route_length(candidate, start)
        if cost < best_cost:
            best_cost, best_route = cost, candidate
    return best_cost, best_route


def capacity_partition(
    cells: Iterable[HGridCell],
    first_position: Tuple[float, float],
    second_position: Tuple[float, float],
    previous_owner: Dict[str, int] | None = None,
    capacity_ratio: float = 0.60,
    consistency_penalty: float = 0.25,
) -> Partition:
    """Approximate the paper's two-vehicle open ACVRP deterministically.

    The small online problem is solved with demand-first route insertion.
    Capacity is a hard upper bound whenever both vehicles can remain feasible.
    The objective is the sum of both open route lengths plus a consistency term.
    """

    all_cells = sorted(list(cells), key=lambda cell: (-cell.demand, cell.id))
    total_demand = sum(max(1, cell.demand) for cell in all_cells)
    capacity = max(1, int(math.ceil(capacity_ratio * total_demand)))
    routes: List[List[HGridCell]] = [[], []]
    loads = [0, 0]
    starts = [first_position, second_position]
    previous_owner = previous_owner or {}

    for cell in all_cells:
        demand = max(1, cell.demand)
        choices = []
        for owner in (0, 1):
            route_cost, candidate = _best_insertion(
                routes[owner], cell, starts[owner]
            )
            other_cost = _route_length(routes[1 - owner], starts[1 - owner])
            overload = max(0, loads[owner] + demand - capacity)
            penalty = overload * 1000.0
            if cell.id in previous_owner and previous_owner[cell.id] != owner:
                penalty += consistency_penalty
            balance = abs(
                (loads[owner] + demand) - loads[1 - owner]
            ) / max(1, total_demand)
            choices.append(
                (route_cost + other_cost + penalty + 0.15 * balance, owner, candidate)
            )
        _, selected, new_route = min(choices, key=lambda item: (item[0], item[1]))
        routes[selected] = new_route
        loads[selected] += demand

    return Partition(
        first=[cell.id for cell in routes[0]],
        second=[cell.id for cell in routes[1]],
        first_route=[cell.id for cell in routes[0]],
        second_route=[cell.id for cell in routes[1]],
        first_demand=loads[0],
        second_demand=loads[1],
    )
