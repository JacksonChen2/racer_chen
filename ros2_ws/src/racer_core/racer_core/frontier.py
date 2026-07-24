"""Frontier extraction and viewpoint sampling port of ``frontier_finder.cpp``."""

from __future__ import annotations

import itertools
import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .environment import EDTEnvironment
from .graph import ViewGraph, ViewNode
from .math_utils import safe_normalize, wrap_yaw
from .perception import PerceptionConfig, PerceptionUtils
from .raycast import RayCaster
from .types import Frontier, Viewpoint
from .voxel_map import Occupancy


Vector = NDArray[np.float64]
IndexTuple = tuple[int, int, int]


@dataclass(slots=True)
class FrontierConfig:
    cluster_min: int = 100
    cluster_size_xy: float = 2.0
    cluster_size_z: float = 10.0
    min_candidate_distance: float = 0.5
    min_candidate_clearance: float = 0.21
    candidate_delta_yaw: float = math.radians(15.0)
    candidate_radius_count: int = 3
    candidate_radius_min: float = 1.0
    candidate_radius_max: float = 1.5
    downsample: int = 3
    min_visible_count: int = 30
    min_view_finish_fraction: float = 0.2


class FrontierFinder:
    def __init__(
        self,
        environment: EDTEnvironment,
        view_graph: ViewGraph,
        config: FrontierConfig = FrontierConfig(),
        perception_config: PerceptionConfig = PerceptionConfig(),
    ) -> None:
        self.environment = environment
        self.map = environment.voxel_map
        self.view_graph = view_graph
        self.config = config
        self.resolution = self.map.resolution
        self.frontier_flags = np.zeros(tuple(self.map.voxel_count), dtype=np.uint8)
        self.frontiers: list[Frontier] = []
        self.dormant_frontiers: list[Frontier] = []
        self.temporary_frontiers: list[Frontier] = []
        self.removed_ids: list[int] = []
        self.perception = PerceptionUtils(perception_config)
        self.ray_caster = RayCaster(self.resolution, self.map.map_origin)
        self.costs = np.empty((0, 0), dtype=np.float64)
        self.paths: dict[tuple[int, int], list[Vector]] = {}
        self.next_frontier: Frontier | None = None

    @staticmethod
    def six_neighbors(index: IndexTuple) -> list[IndexTuple]:
        x, y, z = index
        return [
            (x - 1, y, z),
            (x + 1, y, z),
            (x, y - 1, z),
            (x, y + 1, z),
            (x, y, z - 1),
            (x, y, z + 1),
        ]

    @staticmethod
    def all_neighbors(index: IndexTuple) -> list[IndexTuple]:
        return [
            (index[0] + dx, index[1] + dy, index[2] + dz)
            for dx, dy, dz in itertools.product((-1, 0, 1), repeat=3)
            if (dx, dy, dz) != (0, 0, 0)
        ]

    def _known_free(self, index: IndexTuple) -> bool:
        return self.map.get_occupancy(np.asarray(index, dtype=np.int64)) == Occupancy.FREE

    def _neighbor_unknown(self, index: IndexTuple) -> bool:
        return any(
            self.map.get_occupancy(np.asarray(neighbor, dtype=np.int64)) == Occupancy.UNKNOWN
            for neighbor in self.six_neighbors(index)
        )

    def _is_frontier_cell(self, index: IndexTuple) -> bool:
        array = np.asarray(index, dtype=np.int64)
        return self.map.is_in_box(array) and self._known_free(index) and self._neighbor_unknown(index)

    def _clear_flags(self, frontier: Frontier) -> None:
        for cell in frontier.cells:
            index = self.map.position_to_index(cell)
            if self.map.is_in_map(index):
                self.frontier_flags[tuple(index)] = 0

    def search_frontiers(self) -> None:
        self.temporary_frontiers.clear()
        updated_min, updated_max = self.map.updated_box()
        if not np.all(np.isfinite(updated_min)) or not np.all(np.isfinite(updated_max)):
            updated_min, updated_max = self.map.box()

        retained: list[Frontier] = []
        self.removed_ids.clear()
        for index, frontier in enumerate(self.frontiers):
            overlaps = bool(
                np.all(np.maximum(frontier.box_min, updated_min) <= np.minimum(frontier.box_max, updated_max) + 1.0e-3)
            )
            changed = overlaps and any(
                not self._is_frontier_cell(
                    tuple(int(value) for value in self.map.position_to_index(cell))
                )
                for cell in frontier.cells
            )
            if changed:
                self._clear_flags(frontier)
                self.removed_ids.append(index)
            else:
                retained.append(frontier)
        self.frontiers = retained

        search_min = np.maximum(updated_min - np.asarray((1.0, 1.0, 0.2)), self.map.box_min)
        search_max = np.minimum(updated_max + np.asarray((1.0, 1.0, 0.2)), self.map.box_max)
        minimum = self.map.bound_index(self.map.position_to_index(search_min))
        maximum = self.map.bound_index(self.map.position_to_index(search_max))
        for z in range(int(minimum[2]), int(maximum[2]) + 1):
            for x in range(int(minimum[0]), int(maximum[0]) + 1):
                for y in range(int(minimum[1]), int(maximum[1]) + 1):
                    seed = (x, y, z)
                    if self.frontier_flags[seed] == 0 and self._is_frontier_cell(seed):
                        frontier = self._expand_frontier(seed)
                        if frontier is not None:
                            self.temporary_frontiers.extend(self._split_frontier(frontier))

    def _expand_frontier(self, seed: IndexTuple) -> Frontier | None:
        queue: deque[IndexTuple] = deque([seed])
        self.frontier_flags[seed] = 1
        cells: list[Vector] = []
        while queue:
            current = queue.popleft()
            position = self.map.index_to_position(np.asarray(current, dtype=np.int64))
            if position[2] >= 0.2:
                cells.append(position)
            for neighbor in self.all_neighbors(current):
                index = np.asarray(neighbor, dtype=np.int64)
                if not self.map.is_in_map(index) or not self.map.is_in_box(index):
                    continue
                if self.frontier_flags[neighbor] or not self._is_frontier_cell(neighbor):
                    continue
                self.frontier_flags[neighbor] = 1
                queue.append(neighbor)
        if len(cells) <= self.config.cluster_min:
            return None
        return self._frontier_info(cells)

    def _frontier_info(self, cells: list[Vector]) -> Frontier:
        matrix = np.vstack(cells)
        leaf = self.resolution * self.config.downsample
        keys = np.floor(matrix / leaf).astype(np.int64)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        filtered = matrix[np.sort(unique_indices)]
        return Frontier(
            cells=[point.copy() for point in matrix],
            filtered_cells=[point.copy() for point in filtered],
            average=np.mean(matrix, axis=0),
            box_min=np.min(matrix, axis=0),
            box_max=np.max(matrix, axis=0),
        )

    def _split_frontier(self, frontier: Frontier) -> list[Frontier]:
        points = np.vstack(frontier.filtered_cells)
        xy_mean = frontier.average[:2]
        if not np.any(np.linalg.norm(points[:, :2] - xy_mean, axis=1) > self.config.cluster_size_xy):
            return [frontier]
        covariance = np.cov(points[:, :2] - xy_mean, rowvar=False, bias=True)
        values, vectors = np.linalg.eigh(covariance)
        principal = vectors[:, int(np.argmax(values))]
        left: list[Vector] = []
        right: list[Vector] = []
        for cell in frontier.cells:
            (left if np.dot(cell[:2] - xy_mean, principal) >= 0.0 else right).append(cell)
        result: list[Frontier] = []
        for group in (left, right):
            if group:
                result.extend(self._split_frontier(self._frontier_info(group)))
        return result

    def compute_frontiers_to_visit(self) -> None:
        for frontier in self.temporary_frontiers:
            self._sample_viewpoints(frontier)
            if frontier.viewpoints:
                frontier.viewpoints.sort(key=lambda viewpoint: viewpoint.visible_count, reverse=True)
                self.frontiers.append(frontier)
            else:
                self.dormant_frontiers.append(frontier)
        for index, frontier in enumerate(self.frontiers):
            frontier.id = index
            for viewpoint in frontier.viewpoints:
                viewpoint.frontier_id = index

    def _near_unknown(self, position: Vector) -> bool:
        voxel_count = math.floor(self.config.min_candidate_clearance / self.resolution)
        for x, y, z in itertools.product(
            range(-voxel_count, voxel_count + 1),
            range(-voxel_count, voxel_count + 1),
            range(-1, 2),
        ):
            sample = position + self.resolution * np.asarray((x, y, z), dtype=np.float64)
            if self.map.get_occupancy(sample) == Occupancy.UNKNOWN:
                return True
        return False

    def _visible_cells(self, position: Vector, yaw: float, cluster: list[Vector]) -> int:
        self.perception.set_pose(position, yaw)
        visible = 0
        for cell in cluster:
            if not self.perception.inside_fov(cell):
                continue
            blocked = False
            for index in self.ray_caster.indices(cell, position):
                if (
                    self.map.get_inflated_occupancy(index) == 1
                    or self.map.get_occupancy(index) == Occupancy.UNKNOWN
                ):
                    blocked = True
                    break
            if not blocked:
                visible += 1
        return visible

    def _sample_viewpoints(self, frontier: Frontier) -> None:
        denominator = max(self.config.candidate_radius_count, 1)
        delta_radius = (
            self.config.candidate_radius_max - self.config.candidate_radius_min
        ) / denominator
        radii = np.arange(
            self.config.candidate_radius_min,
            self.config.candidate_radius_max + 1.0e-3,
            delta_radius if delta_radius > 0.0 else 1.0,
        )
        for radius in radii:
            phi = -math.pi
            while phi < math.pi:
                sample = frontier.average + radius * np.asarray(
                    (math.cos(phi), math.sin(phi), 0.0)
                )
                phi += self.config.candidate_delta_yaw
                if (
                    not self.map.is_in_box(sample)
                    or self.map.get_inflated_occupancy(sample) == 1
                    or self._near_unknown(sample)
                ):
                    continue
                reference = safe_normalize(frontier.filtered_cells[0] - sample)
                average_yaw = 0.0
                for cell in frontier.filtered_cells[1:]:
                    direction = safe_normalize(cell - sample)
                    angle = math.acos(float(np.clip(np.dot(direction, reference), -1.0, 1.0)))
                    if np.cross(reference, direction)[2] < 0.0:
                        angle = -angle
                    average_yaw += angle
                average_yaw = wrap_yaw(
                    average_yaw / len(frontier.filtered_cells)
                    + math.atan2(reference[1], reference[0])
                )
                visible = self._visible_cells(sample, average_yaw, frontier.filtered_cells)
                if visible > self.config.min_visible_count:
                    frontier.viewpoints.append(
                        Viewpoint(sample, average_yaw, visible, frontier.id)
                    )

    def update_cost_matrix(self) -> None:
        count = len(self.frontiers)
        self.costs = np.zeros((count, count), dtype=np.float64)
        self.paths.clear()
        for first in range(count):
            for second in range(first + 1, count):
                first_view = self.frontiers[first].viewpoints[0]
                second_view = self.frontiers[second].viewpoints[0]
                cost, path = self.view_graph.compute_cost(
                    first_view.position,
                    second_view.position,
                    first_view.yaw,
                    second_view.yaw,
                    np.zeros(3),
                    0.0,
                )
                self.costs[first, second] = self.costs[second, first] = cost
                self.paths[first, second] = path
                self.paths[second, first] = list(reversed(path))

    def top_viewpoints(
        self, current_position: Vector
    ) -> tuple[list[Vector], list[float], list[Vector]]:
        positions: list[Vector] = []
        yaws: list[float] = []
        averages: list[Vector] = []
        for frontier in self.frontiers:
            selected = frontier.viewpoints[0]
            for viewpoint in frontier.viewpoints:
                if np.linalg.norm(viewpoint.position - current_position) >= self.config.min_candidate_distance:
                    selected = viewpoint
                    break
            positions.append(selected.position.copy())
            yaws.append(selected.yaw)
            averages.append(frontier.average.copy())
        return positions, yaws, averages

    def full_cost_matrix(
        self, current_position: Vector, current_velocity: Vector, current_yaw: float
    ) -> NDArray[np.float64]:
        count = len(self.frontiers)
        matrix = np.zeros((count + 1, count + 1), dtype=np.float64)
        if self.costs.shape == (count, count):
            matrix[1:, 1:] = self.costs
        for index, frontier in enumerate(self.frontiers):
            viewpoint = frontier.viewpoints[0]
            matrix[0, index + 1], _ = self.view_graph.compute_cost(
                current_position,
                viewpoint.position,
                current_yaw,
                viewpoint.yaw,
                current_velocity,
                0.0,
            )
        return matrix

    def swarm_cost_matrix(
        self, positions: list[Vector], velocities: list[Vector], yaws: list[float]
    ) -> NDArray[np.float64]:
        drone_count = len(positions)
        frontier_count = len(self.frontiers)
        dimension = 1 + drone_count + frontier_count
        matrix = np.zeros((dimension, dimension), dtype=np.float64)
        for drone in range(drone_count):
            matrix[0, 1 + drone] = -1000.0
            matrix[1 + drone, 0] = 1000.0
            matrix[1 : 1 + drone_count, 1 : 1 + drone_count] = 10000.0
            for frontier_index, frontier in enumerate(self.frontiers):
                view = frontier.viewpoints[0]
                matrix[1 + drone, 1 + drone_count + frontier_index], _ = (
                    self.view_graph.compute_cost(
                        positions[drone],
                        view.position,
                        yaws[drone],
                        view.yaw,
                        velocities[drone],
                        0.0,
                    )
                )
        matrix[0, 1 + drone_count :] = 1000.0
        if self.costs.shape == (frontier_count, frontier_count):
            matrix[1 + drone_count :, 1 + drone_count :] = self.costs
        np.fill_diagonal(matrix, 1000.0)
        return matrix

    def information_gain(self, position: Vector, yaw: float) -> int:
        self.perception.set_pose(position, yaw)
        minimum, maximum = self.perception.fov_bounding_box()
        gain = 0
        for x in np.arange(minimum[0], maximum[0] + 1.0e-6, 0.8):
            for y in np.arange(minimum[1], maximum[1] + 1.0e-6, 0.8):
                for z in np.arange(minimum[2], maximum[2] + 1.0e-6, 0.8):
                    point = np.asarray((x, y, z))
                    if self.perception.inside_fov(point) and self.map.get_occupancy(point) == Occupancy.UNKNOWN:
                        gain += 1
        return gain

    def set_next_frontier(self, frontier_id: int) -> None:
        self.next_frontier = self.frontiers[frontier_id]

    def is_frontier_covered(self) -> bool:
        for frontier in [*self.frontiers, *self.dormant_frontiers]:
            threshold = int(self.config.min_view_finish_fraction * len(frontier.cells))
            changed = 0
            for cell in frontier.cells:
                index = tuple(int(value) for value in self.map.position_to_index(cell))
                if not self._is_frontier_cell(index):
                    changed += 1
                    if changed >= threshold:
                        return True
        return False
