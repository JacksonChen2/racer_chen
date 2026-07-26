"""Pairwise, capacity-constrained open-route workload partitioning.

For the small pairwise problems normally produced by the active hgrid this
module solves the two-vehicle open CVRP exactly with Held-Karp dynamic
programming. Larger instances use deterministic insertion as a bounded-time
fallback. RACER upstream delegates the same objective to LKH3.
"""

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


def _open_route_tables(
    cells: Sequence[HGridCell],
    start: Tuple[float, float],
) -> Tuple[List[float], List[List[int]], List[int]]:
    """Return optimal open-route cost and predecessor tables for every subset."""

    count = len(cells)
    subset_count = 1 << count
    infinity = math.inf
    costs = [[infinity] * count for _ in range(subset_count)]
    parents = [[-1] * count for _ in range(subset_count)]
    best_cost = [infinity] * subset_count
    best_last = [-1] * subset_count
    best_cost[0] = 0.0
    for index, cell in enumerate(cells):
        mask = 1 << index
        costs[mask][index] = math.hypot(
            cell.centroid[0] - start[0],
            cell.centroid[1] - start[1],
        )
        best_cost[mask] = costs[mask][index]
        best_last[mask] = index
    for mask in range(1, subset_count):
        for last in range(count):
            if not mask & (1 << last):
                continue
            previous_mask = mask ^ (1 << last)
            if previous_mask == 0:
                continue
            for previous in range(count):
                if not previous_mask & (1 << previous):
                    continue
                candidate = costs[previous_mask][previous] + math.hypot(
                    cells[last].centroid[0] - cells[previous].centroid[0],
                    cells[last].centroid[1] - cells[previous].centroid[1],
                )
                if candidate + 1.0e-12 < costs[mask][last]:
                    costs[mask][last] = candidate
                    parents[mask][last] = previous
            if costs[mask][last] < best_cost[mask]:
                best_cost[mask] = costs[mask][last]
                best_last[mask] = last
    return best_cost, parents, best_last


def _restore_route(
    cells: Sequence[HGridCell],
    mask: int,
    parents: Sequence[Sequence[int]],
    best_last: Sequence[int],
) -> List[HGridCell]:
    if mask == 0:
        return []
    route: List[HGridCell] = []
    last = best_last[mask]
    while last >= 0:
        route.append(cells[last])
        previous = parents[mask][last]
        mask ^= 1 << last
        last = previous
    route.reverse()
    return route


def _exact_partition(
    cells: Sequence[HGridCell],
    first_position: Tuple[float, float],
    second_position: Tuple[float, float],
    capacity: int,
    previous_owner: Dict[str, int],
    consistency_penalty: float,
) -> Partition:
    count = len(cells)
    all_mask = (1 << count) - 1
    demands = [max(1, cell.demand) for cell in cells]
    subset_demands = [0] * (1 << count)
    moved_penalties = [0.0] * (1 << count)
    for mask in range(1, 1 << count):
        bit = mask & -mask
        index = bit.bit_length() - 1
        previous_mask = mask ^ bit
        subset_demands[mask] = subset_demands[previous_mask] + demands[index]
        moved_penalties[mask] = moved_penalties[previous_mask]
        if previous_owner.get(cells[index].id) == 1:
            moved_penalties[mask] += consistency_penalty

    first_costs, first_parents, first_last = _open_route_tables(
        cells, first_position
    )
    second_costs, second_parents, second_last = _open_route_tables(
        cells, second_position
    )
    best = (math.inf, 0)
    for first_mask in range(1 << count):
        second_mask = all_mask ^ first_mask
        if (
            subset_demands[first_mask] > capacity
            or subset_demands[second_mask] > capacity
        ):
            continue
        penalty = moved_penalties[first_mask]
        for index, cell in enumerate(cells):
            if (
                second_mask & (1 << index)
                and previous_owner.get(cell.id) == 0
            ):
                penalty += consistency_penalty
        objective = (
            first_costs[first_mask]
            + second_costs[second_mask]
            + penalty
        )
        if objective + 1.0e-12 < best[0]:
            best = (objective, first_mask)
    first_mask = best[1]
    second_mask = all_mask ^ first_mask
    first_route = _restore_route(
        cells, first_mask, first_parents, first_last
    )
    second_route = _restore_route(
        cells, second_mask, second_parents, second_last
    )
    return Partition(
        first=[cell.id for cell in first_route],
        second=[cell.id for cell in second_route],
        first_route=[cell.id for cell in first_route],
        second_route=[cell.id for cell in second_route],
        first_demand=subset_demands[first_mask],
        second_demand=subset_demands[second_mask],
    )


def capacity_partition(
    cells: Iterable[HGridCell],
    first_position: Tuple[float, float],
    second_position: Tuple[float, float],
    previous_owner: Dict[str, int] | None = None,
    capacity_ratio: float = 0.75,
    consistency_penalty: float = 0.25,
    exact_limit: int = 12,
) -> Partition:
    """Solve the paper's two-vehicle capacity-constrained open-route objective.

    The 75% capacity ratio follows the upstream implementation. Dynamic
    programming is exact up to ``exact_limit`` cells; insertion is retained for
    larger online problems so the decentralized callback stays responsive.
    """

    all_cells = sorted(list(cells), key=lambda cell: (-cell.demand, cell.id))
    total_demand = sum(max(1, cell.demand) for cell in all_cells)
    capacity = max(
        max((max(1, cell.demand) for cell in all_cells), default=1),
        int(math.ceil(capacity_ratio * total_demand)),
    )
    previous_owner = previous_owner or {}
    if len(all_cells) <= exact_limit:
        return _exact_partition(
            all_cells,
            first_position,
            second_position,
            capacity,
            previous_owner,
            consistency_penalty,
        )

    routes: List[List[HGridCell]] = [[], []]
    loads = [0, 0]
    starts = [first_position, second_position]
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
