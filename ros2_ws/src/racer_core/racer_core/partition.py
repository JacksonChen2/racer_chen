"""Two-level exploration partition port of ``uniform_grid`` and ``hgrid``."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .environment import EDTEnvironment
from .search import AStar
from .voxel_map import Occupancy


Vector = NDArray[np.float64]


@dataclass(slots=True)
class PartitionConfig:
    minimum_unknown: int = 4000
    minimum_frontier: int = 100
    minimum_free: int = 3000
    consistent_cost: float = -5.0
    consistent_cost2: float = 8.0
    unknown_weight: float = 0.0
    grid_size: float = 5.0
    first_weight: float = 1.0


@dataclass(slots=True)
class GridInfo:
    center: Vector
    minimum: Vector
    maximum: Vector
    unknown_count: int
    frontier_ids: set[int] = field(default_factory=set)
    updated: bool = False
    need_divide: bool = False
    active: bool = False
    previously_relevant: bool = True
    currently_relevant: bool = True


class UniformGrid:
    def __init__(
        self,
        environment: EDTEnvironment,
        config: PartitionConfig,
        level: int,
    ) -> None:
        self.environment = environment
        self.map = environment.voxel_map
        self.config = config
        self.level = level
        self.minimum, self.maximum = self.map.box()
        size = self.maximum - self.minimum
        resolution = np.empty(3, dtype=np.float64)
        for axis in range(2):
            count = math.ceil(size[axis] / config.grid_size)
            resolution[axis] = size[axis] / count / (2 ** (level - 1))
        resolution[2] = size[2]
        self.resolution = resolution
        self.grid_count = np.ceil(size / resolution).astype(np.int64)
        self.grids: list[GridInfo] = []
        self.relevant_ids: list[int] = []
        self.initialized = False
        total_voxels = int(np.prod(np.ceil(resolution / self.map.resolution)))
        for address in range(int(np.prod(self.grid_count))):
            index = self.address_to_index(address)
            lower = self.index_to_position(index, 0.0)
            upper = self.index_to_position(index, 1.0)
            self.grids.append(
                GridInfo(
                    center=self.index_to_position(index, 0.5),
                    minimum=lower,
                    maximum=upper,
                    unknown_count=total_voxels,
                    active=level == 1,
                )
            )

    def to_address(self, index: NDArray[np.int64]) -> int:
        return int(
            index[0] * self.grid_count[1] * self.grid_count[2]
            + index[1] * self.grid_count[2]
            + index[2]
        )

    def address_to_index(self, address: int) -> NDArray[np.int64]:
        yz = int(self.grid_count[1] * self.grid_count[2])
        x, remainder = divmod(address, yz)
        y, z = divmod(remainder, int(self.grid_count[2]))
        return np.asarray((x, y, z), dtype=np.int64)

    def position_to_index(self, position: Vector) -> NDArray[np.int64]:
        return np.floor((position - self.minimum) / self.resolution).astype(np.int64)

    def index_to_position(self, index: NDArray[np.int64], increment: float) -> Vector:
        return (index.astype(np.float64) + increment) * self.resolution + self.minimum

    def inside(self, index: NDArray[np.int64]) -> bool:
        return bool(np.all(index >= 0) and np.all(index < self.grid_count))

    def input_frontiers(self, averages: list[Vector]) -> None:
        for grid in self.grids:
            grid.frontier_ids.clear()
        for frontier_id, average in enumerate(averages):
            index = self.position_to_index(average)
            if self.inside(index):
                self.grids[self.to_address(index)].frontier_ids.add(frontier_id)

    def relevant(self, grid: GridInfo) -> bool:
        return (
            grid.unknown_count >= self.config.minimum_unknown
            or bool(grid.frontier_ids)
        )

    def update_grid(self, address: int) -> None:
        grid = self.grids[address]
        grid.previously_relevant = grid.currently_relevant
        minimum_index = self.map.bound_index(self.map.position_to_index(grid.minimum))
        maximum_index = self.map.bound_index(self.map.position_to_index(grid.maximum))
        occupancy = self.map.occupancy[
            minimum_index[0] : maximum_index[0] + 1,
            minimum_index[1] : maximum_index[1] + 1,
            minimum_index[2] : maximum_index[2] + 1,
        ]
        unknown_mask = occupancy < self.map.clamp_min_log - 1.0e-3
        free_mask = np.logical_and(
            occupancy >= self.map.clamp_min_log - 1.0e-3,
            occupancy <= self.map.occupied_log,
        )
        grid.unknown_count = int(np.count_nonzero(unknown_mask))
        if grid.unknown_count:
            indices = np.argwhere(unknown_mask) + minimum_index
            positions = (indices + 0.5) * self.map.resolution + self.map.map_origin
            grid.center = np.mean(positions, axis=0)
        grid.currently_relevant = self.relevant(grid)
        grid.updated = True
        if self.level == 1 and grid.active and int(np.count_nonzero(free_mask)) > self.config.minimum_free:
            grid.need_divide = True

    def update(self, drone_id: int, assigned: list[int]) -> tuple[list[int], list[int]]:
        partitioned: list[int] = []
        partitioned_all: list[int] = []
        for address, grid in enumerate(self.grids):
            if not grid.active:
                continue
            self.update_grid(address)
            if grid.need_divide:
                partitioned_all.append(address)
        self.relevant_ids = [
            index for index, grid in enumerate(self.grids) if self.relevant(grid)
        ]
        if not self.initialized:
            if drone_id == 1 and self.level == 1:
                assigned = self.relevant_ids.copy()
            self.initialized = True
        else:
            result: list[int] = []
            for address in assigned:
                if address not in self.relevant_ids:
                    continue
                if self.grids[address].need_divide:
                    partitioned.append(address)
                else:
                    result.append(address)
            assigned = result
        return assigned, partitioned

    def activate(self, ids: list[int]) -> None:
        for address in ids:
            self.grids[address].active = True


class HierarchicalGrid:
    def __init__(
        self,
        environment: EDTEnvironment,
        path_finder: AStar,
        config: PartitionConfig = PartitionConfig(),
    ) -> None:
        self.environment = environment
        self.path_finder = path_finder
        self.config = config
        self.coarse = UniformGrid(environment, config, 1)
        self.fine = UniformGrid(environment, config, 2)

    @property
    def coarse_count(self) -> int:
        return len(self.coarse.grids)

    def coarse_to_fine(self, coarse_id: int) -> list[int]:
        index = self.coarse.address_to_index(coarse_id)
        return [
            self.fine.to_address(
                np.asarray((index[0] * 2 + dx, index[1] * 2 + dy, index[2]))
            )
            for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1))
        ]

    def fine_to_coarse(self, fine_id: int) -> int:
        index = self.fine.address_to_index(fine_id)
        return self.coarse.to_address(
            np.asarray((index[0] // 2, index[1] // 2, index[2]))
        )

    def get_grid(self, grid_id: int) -> GridInfo:
        if grid_id < self.coarse_count:
            return self.coarse.grids[grid_id]
        return self.fine.grids[grid_id - self.coarse_count]

    def input_frontiers(self, averages: list[Vector]) -> None:
        self.coarse.input_frontiers(averages)
        self.fine.input_frontiers(averages)

    def update(self, drone_id: int, grid_ids: list[int]) -> list[int]:
        coarse_ids = [grid_id for grid_id in grid_ids if grid_id < self.coarse_count]
        fine_ids = [
            grid_id - self.coarse_count
            for grid_id in grid_ids
            if grid_id >= self.coarse_count
        ]
        coarse_ids, partitioned = self.coarse.update(drone_id, coarse_ids)
        activated: list[int] = []
        for coarse_id in partitioned:
            children = self.coarse_to_fine(coarse_id)
            fine_ids.extend(children)
            activated.extend(children)
        self.fine.activate(activated)
        fine_ids, _ = self.fine.update(drone_id, fine_ids)
        return coarse_ids + [grid_id + self.coarse_count for grid_id in fine_ids]

    def active_grids(self) -> list[int]:
        result = [
            index
            for index, grid in enumerate(self.coarse.grids)
            if grid.active and grid.currently_relevant
        ]
        result.extend(
            index + self.coarse_count
            for index, grid in enumerate(self.fine.grids)
            if grid.active and grid.currently_relevant
        )
        return result

    def drone_to_grid_cost(
        self, position: Vector, grid_id: int, previous_first: list[int]
    ) -> float:
        grid = self.get_grid(grid_id)
        straight = float(np.linalg.norm(position - grid.center))
        if straight < 5.0:
            self.path_finder.reset()
            if self.path_finder.search(position, grid.center) == AStar.REACH_END:
                cost = AStar.path_length(self.path_finder.path)
            else:
                cost = straight + self.config.consistent_cost2
        else:
            cost = 1.5 * straight + self.config.consistent_cost2
        if grid_id in previous_first:
            cost += self.config.consistent_cost
        return cost

    def grid_to_grid_cost(self, first_id: int, second_id: int, drone_count: int) -> float:
        first = self.get_grid(first_id)
        second = self.get_grid(second_id)
        first_coarse = (
            first_id if first_id < self.coarse_count else self.fine_to_coarse(first_id - self.coarse_count)
        )
        second_coarse = (
            second_id if second_id < self.coarse_count else self.fine_to_coarse(second_id - self.coarse_count)
        )
        first_index = self.coarse.address_to_index(first_coarse)
        second_index = self.coarse.address_to_index(second_coarse)
        close = bool(np.all(np.abs(first_index - second_index) <= 1))
        straight = float(np.linalg.norm(first.center - second.center))
        if close:
            self.path_finder.reset()
            if self.path_finder.search(first.center, second.center) == AStar.REACH_END:
                cost = AStar.path_length(self.path_finder.path)
            else:
                cost = straight + self.config.consistent_cost2
            if (
                drone_count <= 1
                and first_id >= self.coarse_count
                and second_id >= self.coarse_count
                and first_coarse == second_coarse
            ):
                cost += self.config.consistent_cost
            return cost
        return 1.5 * straight + self.config.consistent_cost2

    def cost_matrix(
        self,
        positions: list[Vector],
        grid_ids: list[int],
        previous_first: list[list[int]],
    ) -> NDArray[np.float64]:
        drone_count = len(positions)
        grid_count = len(grid_ids)
        dimension = 1 + drone_count + grid_count
        matrix = np.zeros((dimension, dimension), dtype=np.float64)
        for drone in range(drone_count):
            matrix[0, 1 + drone] = -1000.0
            matrix[1 + drone, 0] = 1000.0
            matrix[1 : 1 + drone_count, 1 : 1 + drone_count] = 10000.0
            for grid_index, grid_id in enumerate(grid_ids):
                matrix[1 + drone, 1 + drone_count + grid_index] = self.drone_to_grid_cost(
                    positions[drone], grid_id, previous_first[drone]
                )
        matrix[0, 1 + drone_count :] = 1000.0
        for first in range(grid_count):
            for second in range(first + 1, grid_count):
                cost = self.grid_to_grid_cost(
                    grid_ids[first], grid_ids[second], drone_count
                )
                matrix[1 + drone_count + first, 1 + drone_count + second] = cost
                matrix[1 + drone_count + second, 1 + drone_count + first] = cost
        np.fill_diagonal(matrix, 1000.0)
        return matrix

    def frontiers_in_grids(self, grid_ids: list[int]) -> list[int]:
        result: set[int] = set()
        if not grid_ids:
            return []
        first = grid_ids[0]
        if first < self.coarse_count:
            result.update(self.coarse.grids[first].frontier_ids)
        else:
            coarse = self.fine_to_coarse(first - self.coarse_count)
            allocated = {
                grid_id - self.coarse_count
                for grid_id in grid_ids
                if grid_id >= self.coarse_count
            }
            for child in self.coarse_to_fine(coarse):
                if child in allocated:
                    result.update(self.fine.grids[child].frontier_ids)
        return sorted(result)

    def next_grid(self, grid_ids: list[int]) -> tuple[Vector, float] | None:
        if len(grid_ids) < 2:
            return None
        first_coarse = (
            grid_ids[0]
            if grid_ids[0] < self.coarse_count
            else self.fine_to_coarse(grid_ids[0] - self.coarse_count)
        )
        for candidate in grid_ids[1:]:
            coarse = (
                candidate
                if candidate < self.coarse_count
                else self.fine_to_coarse(candidate - self.coarse_count)
            )
            if coarse != first_coarse:
                first = self.get_grid(grid_ids[0]).center
                second = self.get_grid(candidate).center
                direction = second - first
                return second.copy(), math.atan2(direction[1], direction[0])
        return None
