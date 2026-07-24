"""Topological probabilistic-roadmap path search port."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math

import numpy as np
from numpy.typing import NDArray

from .voxel_map import VoxelMap


Array = NDArray[np.float64]


@dataclass(slots=True)
class TopologyConfig:
    sample_count: int = 400
    reserve_count: int = 6
    ratio_to_short: float = 1.5
    clearance: float = 0.2
    connection_radius: float = 3.0
    seed: int = 0


class TopologicalPRM:
    def __init__(self, voxel_map: VoxelMap, config: TopologyConfig | None = None) -> None:
        self.map = voxel_map
        self.config = config or TopologyConfig()
        self.random = np.random.default_rng(self.config.seed)

    def visible(self, start: Array, end: Array) -> bool:
        for position in self.map.ray_caster.positions(start, end):
            if (
                not self.map.is_in_box(position)
                or self.map.get_inflated_occupancy(position) == 1
                or self.map.get_distance(position) < self.config.clearance
            ):
                return False
        return True

    @staticmethod
    def _length(path: list[Array]) -> float:
        return sum(float(np.linalg.norm(b - a)) for a, b in zip(path[:-1], path[1:]))

    def _shortest(self, points: list[Array], graph: list[list[tuple[int, float]]]) -> list[Array]:
        queue, serial = [(0.0, 0, 0)], itertools.count(1)
        costs, parent = {0: 0.0}, {0: -1}
        while queue:
            cost, _, node = heapq.heappop(queue)
            if node == 1:
                break
            if cost > costs[node]:
                continue
            for neighbor, edge in graph[node]:
                tentative = cost + edge
                if tentative < costs.get(neighbor, math.inf):
                    costs[neighbor], parent[neighbor] = tentative, node
                    heapq.heappush(queue, (tentative, next(serial), neighbor))
        if 1 not in parent:
            return []
        ids, node = [], 1
        while node >= 0:
            ids.append(node)
            node = parent[node]
        return [points[index].copy() for index in ids[::-1]]

    def search(self, start: Array, goal: Array) -> list[list[Array]]:
        if self.visible(start, goal):
            return [[np.asarray(start).copy(), np.asarray(goal).copy()]]
        center, direction = 0.5 * (start + goal), goal - start
        length = max(float(np.linalg.norm(direction)), 1.0)
        points = [np.asarray(start).copy(), np.asarray(goal).copy()]
        lower, upper = self.map.box()
        for _ in range(self.config.sample_count):
            sample = center + self.random.normal(size=3) * np.asarray((0.55 * length, 0.4 * length, 0.25 * length))
            sample = np.clip(sample, lower, upper)
            if self.map.get_distance(sample) > self.config.clearance:
                points.append(sample)
        graph: list[list[tuple[int, float]]] = [[] for _ in points]
        for first in range(len(points)):
            distances = sorted(
                ((float(np.linalg.norm(points[first] - points[second])), second)
                 for second in range(len(points)) if second != first),
                key=lambda value: value[0],
            )
            for distance, second in distances[:12]:
                if distance <= self.config.connection_radius and self.visible(points[first], points[second]):
                    graph[first].append((second, distance))
        paths: list[list[Array]] = []
        shortest = self._shortest(points, graph)
        if shortest:
            paths.append(shortest)
        # RACER retains geometrically distinct alternatives. Block each internal
        # shortest-path vertex in turn and search again.
        for blocked in range(1, max(1, len(shortest) - 1)):
            if len(paths) >= self.config.reserve_count:
                break
            graph_copy = [edges.copy() for edges in graph]
            nearest = min(range(2, len(points)), key=lambda i: np.linalg.norm(points[i] - shortest[blocked]))
            graph_copy[nearest] = []
            for edges in graph_copy:
                edges[:] = [edge for edge in edges if edge[0] != nearest]
            candidate = self._shortest(points, graph_copy)
            if candidate and self._length(candidate) <= self.config.ratio_to_short * self._length(shortest):
                paths.append(candidate)
        return paths

