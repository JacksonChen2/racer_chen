"""Geometric A* search port of ``path_searching/astar2.cpp``."""

from __future__ import annotations

import heapq
import itertools
import math
import time
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .environment import EDTEnvironment
from .voxel_map import Occupancy


Vector = NDArray[np.float64]
IndexTuple = tuple[int, int, int]


@dataclass(order=True, slots=True)
class _QueueEntry:
    score: float
    serial: int
    index: IndexTuple = field(compare=False)


@dataclass(slots=True)
class _Node:
    index: IndexTuple
    position: Vector
    g_score: float
    f_score: float
    parent: IndexTuple | None


class AStar:
    REACH_END = 1
    NO_PATH = 2

    def __init__(
        self,
        environment: EDTEnvironment,
        resolution: float = 0.3,
        lambda_heuristic: float = 10000.0,
        max_search_time: float = 0.001,
        allocate_num: int = 1_000_000,
    ) -> None:
        self.environment = environment
        self.resolution = float(resolution)
        self.inverse_resolution = 1.0 / self.resolution
        self.lambda_heuristic = float(lambda_heuristic)
        self.max_search_time = float(max_search_time)
        self.allocate_num = int(allocate_num)
        self.tie_breaker = 1.0 + 1.0 / 1000.0
        self.origin, self.map_size = environment.voxel_map.region()
        self.path: list[Vector] = []
        self.visited: list[Vector] = []
        self.early_terminate_cost = 0.0

    def reset(self) -> None:
        self.path.clear()
        self.visited.clear()
        self.early_terminate_cost = 0.0

    def set_resolution(self, resolution: float) -> None:
        self.resolution = float(resolution)
        self.inverse_resolution = 1.0 / self.resolution

    def position_to_index(self, point: Vector) -> IndexTuple:
        index = np.floor((np.asarray(point) - self.origin) * self.inverse_resolution).astype(int)
        return int(index[0]), int(index[1]), int(index[2])

    def diagonal_heuristic(self, first: Vector, second: Vector) -> float:
        dx, dy, dz = np.abs(np.asarray(first) - np.asarray(second))
        diagonal = min(dx, dy, dz)
        dx -= diagonal
        dy -= diagonal
        dz -= diagonal
        if dx < 1.0e-4:
            value = math.sqrt(3.0) * diagonal + math.sqrt(2.0) * min(dy, dz) + abs(dy - dz)
        elif dy < 1.0e-4:
            value = math.sqrt(3.0) * diagonal + math.sqrt(2.0) * min(dx, dz) + abs(dx - dz)
        else:
            value = math.sqrt(3.0) * diagonal + math.sqrt(2.0) * min(dx, dy) + abs(dx - dy)
        return self.tie_breaker * float(value)

    @staticmethod
    def path_length(path: list[Vector]) -> float:
        return float(
            sum(np.linalg.norm(second - first) for first, second in zip(path, path[1:]))
        )

    def _segment_safe(self, start: Vector, end: Vector, optimistic: bool) -> bool:
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length < 1.0e-12:
            return True
        direction /= length
        distance = 0.1
        while distance <= length + 1.0e-2:
            point = start + distance * direction
            if self.environment.voxel_map.get_inflated_occupancy(point) == 1:
                return False
            if not optimistic and self.environment.voxel_map.get_occupancy(point) == Occupancy.UNKNOWN:
                return False
            distance += 0.1
        return True

    def search(self, start: Vector, end: Vector, optimistic: bool = True) -> int:
        self.reset()
        start_position = np.asarray(start, dtype=np.float64)
        end_position = np.asarray(end, dtype=np.float64)
        start_index = self.position_to_index(start_position)
        end_index = self.position_to_index(end_position)
        start_node = _Node(
            start_index,
            start_position.copy(),
            0.0,
            self.lambda_heuristic * self.diagonal_heuristic(start_position, end_position),
            None,
        )
        nodes: dict[IndexTuple, _Node] = {start_index: start_node}
        closed: set[IndexTuple] = set()
        serial = itertools.count()
        queue = [_QueueEntry(start_node.f_score, next(serial), start_index)]
        start_time = time.monotonic()

        while queue:
            entry = heapq.heappop(queue)
            current = nodes[entry.index]
            if entry.index in closed or entry.score > current.f_score + 1.0e-12:
                continue
            if all(abs(entry.index[axis] - end_index[axis]) <= 1 for axis in range(3)):
                self._backtrack(nodes, current.index, end_position)
                self.visited = [node.position.copy() for node in nodes.values()]
                return self.REACH_END
            if time.monotonic() - start_time > self.max_search_time:
                self.early_terminate_cost = current.g_score + self.diagonal_heuristic(
                    current.position, end_position
                )
                self.visited = [node.position.copy() for node in nodes.values()]
                return self.NO_PATH
            closed.add(current.index)
            for offset in itertools.product((-1, 0, 1), repeat=3):
                if offset == (0, 0, 0):
                    continue
                step = self.resolution * np.asarray(offset, dtype=np.float64)
                neighbor_position = current.position + step
                voxel_map = self.environment.voxel_map
                if not voxel_map.is_in_box(neighbor_position):
                    continue
                if voxel_map.get_inflated_occupancy(neighbor_position) == 1:
                    continue
                if not optimistic and voxel_map.get_occupancy(neighbor_position) == Occupancy.UNKNOWN:
                    continue
                if not self._segment_safe(current.position, neighbor_position, optimistic):
                    continue
                neighbor_index = self.position_to_index(neighbor_position)
                if neighbor_index in closed:
                    continue
                g_score = current.g_score + float(np.linalg.norm(step))
                existing = nodes.get(neighbor_index)
                if existing is not None and g_score >= existing.g_score:
                    continue
                if existing is None and len(nodes) >= self.allocate_num:
                    self.visited = [node.position.copy() for node in nodes.values()]
                    return self.NO_PATH
                f_score = g_score + self.lambda_heuristic * self.diagonal_heuristic(
                    neighbor_position, end_position
                )
                nodes[neighbor_index] = _Node(
                    neighbor_index,
                    neighbor_position,
                    g_score,
                    f_score,
                    current.index,
                )
                heapq.heappush(queue, _QueueEntry(f_score, next(serial), neighbor_index))

        self.visited = [node.position.copy() for node in nodes.values()]
        return self.NO_PATH

    def _backtrack(
        self, nodes: dict[IndexTuple, _Node], final_index: IndexTuple, exact_end: Vector
    ) -> None:
        result = [np.asarray(exact_end, dtype=np.float64).copy()]
        index: IndexTuple | None = final_index
        while index is not None:
            node = nodes[index]
            result.append(node.position.copy())
            index = node.parent
        result.reverse()
        self.path = result
