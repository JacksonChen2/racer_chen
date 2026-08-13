"""Pairwise capacity-constrained 3-D region allocation."""

from dataclasses import dataclass
import math
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .hgrid import HGridCell3D


DistanceFunction = Callable[[Sequence[float], Sequence[float]], float]


@dataclass
class Partition3D:
    first: List[str]
    second: List[str]
    first_route: List[str]
    second_route: List[str]
    first_demand: int
    second_demand: int
    cost: float


def _route_cost(
    route: Sequence[HGridCell3D],
    start: Sequence[float],
    distance_function: Optional[DistanceFunction] = None,
) -> float:
    cursor = np.asarray(start, dtype=float)
    result = 0.0
    for cell in route:
        target = np.asarray(cell.centroid)
        result += _distance(cursor, target, distance_function)
        cursor = target
    return result


def _distance(
    first: Sequence[float],
    second: Sequence[float],
    distance_function: Optional[DistanceFunction],
) -> float:
    if distance_function is not None:
        value = float(distance_function(first, second))
        if math.isfinite(value):
            return value
    return float(np.linalg.norm(np.asarray(second) - np.asarray(first)))


def _nearest_route(
    cells: Sequence[HGridCell3D],
    start: Sequence[float],
    distance_function: Optional[DistanceFunction] = None,
) -> List[HGridCell3D]:
    remaining = list(cells)
    result = []
    cursor = np.asarray(start, dtype=float)
    while remaining:
        item = min(
            remaining,
            key=lambda cell: _distance(
                cursor, cell.centroid, distance_function
            ),
        )
        result.append(item)
        remaining.remove(item)
        cursor = np.asarray(item.centroid)
    return result


def _all_exact_open_routes(
    cells: Sequence[HGridCell3D],
    start: Sequence[float],
    distance_function: Optional[DistanceFunction],
) -> Dict[int, Tuple[float, List[HGridCell3D]]]:
    """Held-Karp solutions for every open route subset.

    RACER's pairwise problem is small.  Computing all subset routes once for
    each vehicle gives exact open-path costs for every CVRP partition, rather
    than evaluating a nearest-neighbour approximation.
    """

    count = len(cells)
    routes: Dict[int, Tuple[float, List[HGridCell3D]]] = {0: (0.0, [])}
    if not count:
        return routes
    pair_cost = np.zeros((count, count), dtype=float)
    start_cost = np.zeros(count, dtype=float)
    for first in range(count):
        start_cost[first] = _distance(
            start, cells[first].centroid, distance_function
        )
        for second in range(first + 1, count):
            value = _distance(
                cells[first].centroid,
                cells[second].centroid,
                distance_function,
            )
            pair_cost[first, second] = value
            pair_cost[second, first] = value
    dynamic: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], int] = {}
    for mask in range(1, 1 << count):
        for last in range(count):
            bit = 1 << last
            if not mask & bit:
                continue
            previous_mask = mask ^ bit
            if previous_mask == 0:
                dynamic[mask, last] = float(start_cost[last])
                parent[mask, last] = -1
                continue
            previous = min(
                (
                    dynamic[previous_mask, candidate]
                    + pair_cost[candidate, last],
                    candidate,
                )
                for candidate in range(count)
                if previous_mask & (1 << candidate)
            )
            dynamic[mask, last] = float(previous[0])
            parent[mask, last] = int(previous[1])
        cost, final = min(
            (dynamic[mask, last], last)
            for last in range(count)
            if mask & (1 << last)
        )
        indices = []
        cursor_mask, cursor = mask, final
        while cursor >= 0:
            indices.append(cursor)
            previous = parent[cursor_mask, cursor]
            cursor_mask ^= 1 << cursor
            cursor = previous
        indices.reverse()
        routes[mask] = (float(cost), [cells[index] for index in indices])
    return routes


def capacity_partition(
    cells: Sequence[HGridCell3D],
    first_start: Sequence[float],
    second_start: Sequence[float],
    imbalance: float = 0.20,
    distance_function: Optional[DistanceFunction] = None,
) -> Partition3D:
    """Solve small pairwise CVRP partitions exactly, then route in 3-D."""

    values = list(cells)
    total_demand = sum(cell.demand for cell in values)
    if not values:
        return Partition3D([], [], [], [], 0, 0, 0.0)
    best = None
    # Pairwise HGrid interactions are deliberately bounded. Exact enumeration
    # reproduces RACER's capacity objective for up to 10 active cells.
    masks = range(1 << len(values)) if len(values) <= 10 else ()
    first_routes = (
        _all_exact_open_routes(values, first_start, distance_function)
        if len(values) <= 10
        else {}
    )
    second_routes = (
        _all_exact_open_routes(values, second_start, distance_function)
        if len(values) <= 10
        else {}
    )
    full_mask = (1 << len(values)) - 1
    for mask in masks:
        first = [
            cell for index, cell in enumerate(values) if mask & (1 << index)
        ]
        second = [cell for cell in values if cell not in first]
        first_demand = sum(cell.demand for cell in first)
        second_demand = total_demand - first_demand
        allowed = max(1.0, imbalance * total_demand)
        balance_error = abs(first_demand - second_demand)
        first_route_cost, first_route = first_routes[mask]
        second_route_cost, second_route = second_routes[full_mask ^ mask]
        route_cost = first_route_cost + second_route_cost
        objective = route_cost + 0.02 * max(0.0, balance_error - allowed)
        candidate = (
            objective,
            [cell.id for cell in first],
            [cell.id for cell in second],
            [cell.id for cell in first_route],
            [cell.id for cell in second_route],
            first_demand,
            second_demand,
        )
        if best is None or candidate[0] < best[0]:
            best = candidate
    if best is None:
        # Bounded deterministic fallback for an unusually large active set.
        first, second = [], []
        demand = [0, 0]
        for cell in sorted(values, key=lambda item: -item.demand):
            distances = (
                float(np.linalg.norm(np.asarray(cell.centroid) - first_start)),
                float(np.linalg.norm(np.asarray(cell.centroid) - second_start)),
            )
            owner = min(
                (0, 1),
                key=lambda index: distances[index] + 0.01 * demand[index],
            )
            (first if owner == 0 else second).append(cell)
            demand[owner] += cell.demand
        first_route = _nearest_route(
            first, first_start, distance_function
        )
        second_route = _nearest_route(
            second, second_start, distance_function
        )
        best = (
            _route_cost(first_route, first_start, distance_function)
            + _route_cost(second_route, second_start, distance_function),
            [cell.id for cell in first],
            [cell.id for cell in second],
            [cell.id for cell in first_route],
            [cell.id for cell in second_route],
            demand[0],
            demand[1],
        )
    return Partition3D(
        first=best[1],
        second=best[2],
        first_route=best[3],
        second_route=best[4],
        first_demand=best[5],
        second_demand=best[6],
        cost=best[0],
    )
