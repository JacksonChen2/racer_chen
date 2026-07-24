"""Viewpoint graph costs ported from ``active_perception/graph_node.cpp``."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .raycast import RayCaster
from .search import AStar
from .voxel_map import VoxelMap


Vector = NDArray[np.float64]


@dataclass(slots=True)
class ViewNode:
    position: Vector
    yaw: float
    velocity: Vector = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    yaw_rate: float = 0.0
    id: int = -1


class ViewGraph:
    def __init__(
        self,
        voxel_map: VoxelMap,
        astar: AStar,
        max_velocity: float,
        max_acceleration: float,
        max_yaw_rate: float,
        max_yaw_acceleration: float,
        direction_weight: float,
    ) -> None:
        self.map = voxel_map
        self.astar = astar
        self.ray_caster = RayCaster(voxel_map.resolution, voxel_map.map_origin)
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_yaw_rate = max_yaw_rate
        self.max_yaw_acceleration = max_yaw_acceleration
        self.direction_weight = direction_weight

    def search_path(self, start: Vector, end: Vector) -> tuple[float, list[Vector]]:
        safe = True
        for index in self.ray_caster.indices(start, end):
            if not self.map.is_in_box(index) or self.map.get_inflated_occupancy(index) == 1:
                safe = False
                break
        if safe:
            return float(np.linalg.norm(start - end)), [start.copy(), end.copy()]
        self.astar.reset()
        previous_resolution = self.astar.resolution
        self.astar.set_resolution(0.4)
        try:
            if self.astar.search(start, end) == AStar.REACH_END:
                return AStar.path_length(self.astar.path), [
                    point.copy() for point in self.astar.path
                ]
            return 100.0, [start.copy(), end.copy()]
        finally:
            self.astar.set_resolution(previous_resolution)

    def compute_cost(
        self,
        first_position: Vector,
        second_position: Vector,
        first_yaw: float,
        second_yaw: float,
        first_velocity: Vector,
        first_yaw_rate: float,
    ) -> tuple[float, list[Vector]]:
        distance, path = self.search_path(first_position, second_position)
        position_cost = distance / self.max_velocity
        velocity_norm = float(np.linalg.norm(first_velocity))
        direction = second_position - first_position
        if velocity_norm > 1.0e-3 and np.linalg.norm(direction) > 1.0e-12:
            dot = float(
                np.clip(
                    np.dot(first_velocity / velocity_norm, direction / np.linalg.norm(direction)),
                    -1.0,
                    1.0,
                )
            )
            position_cost += self.direction_weight * math.acos(dot)
        yaw_difference = abs(second_yaw - first_yaw)
        yaw_difference = min(yaw_difference, 2.0 * math.pi - yaw_difference)
        yaw_cost = yaw_difference / self.max_yaw_rate
        return max(position_cost, yaw_cost), path

    def cost(self, first: ViewNode, second: ViewNode) -> tuple[float, list[Vector]]:
        return self.compute_cost(
            first.position,
            second.position,
            first.yaw,
            second.yaw,
            first.velocity,
            first.yaw_rate,
        )
