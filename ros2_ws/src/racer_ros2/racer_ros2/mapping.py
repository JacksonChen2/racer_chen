"""Occupancy mapping and incremental frontier extraction."""

from collections import deque
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)
GridIndex = Tuple[int, int]


class OccupancyMap:
    """Small log-odds occupancy grid in a fixed shared world frame."""

    def __init__(
        self,
        resolution: float,
        origin: Tuple[float, float],
        size: Tuple[float, float],
    ) -> None:
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width = int(math.ceil(size[0] / resolution))
        self.height = int(math.ceil(size[1] / resolution))
        self.log_odds = np.zeros((self.height, self.width), dtype=np.int16)
        self.observations = np.zeros((self.height, self.width), dtype=np.uint16)

    def copy(self) -> "OccupancyMap":
        result = OccupancyMap(
            self.resolution,
            self.origin,
            (self.width * self.resolution, self.height * self.resolution),
        )
        result.log_odds = self.log_odds.copy()
        result.observations = self.observations.copy()
        return result

    def in_bounds(self, cell: GridIndex) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def world_to_grid(self, x: float, y: float) -> Optional[GridIndex]:
        gx = int(math.floor((x - self.origin[0]) / self.resolution))
        gy = int(math.floor((y - self.origin[1]) / self.resolution))
        if not self.in_bounds((gx, gy)):
            return None
        return gx, gy

    def grid_to_world(self, cell: GridIndex) -> Tuple[float, float]:
        return (
            self.origin[0] + (cell[0] + 0.5) * self.resolution,
            self.origin[1] + (cell[1] + 0.5) * self.resolution,
        )

    @staticmethod
    def bresenham(start: GridIndex, end: GridIndex) -> List[GridIndex]:
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx - dy
        result: List[GridIndex] = []
        while True:
            result.append((x0, y0))
            if x0 == x1 and y0 == y1:
                return result
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy

    def _observe(self, cells: Iterable[GridIndex], delta: int) -> None:
        for x, y in cells:
            if not self.in_bounds((x, y)):
                continue
            self.log_odds[y, x] = np.int16(
                np.clip(int(self.log_odds[y, x]) + delta, -20, 20)
            )
            if self.observations[y, x] < np.iinfo(np.uint16).max:
                self.observations[y, x] += 1

    def update_scan(
        self,
        position: Tuple[float, float],
        yaw: float,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_max: float,
    ) -> None:
        start = self.world_to_grid(*position)
        if start is None:
            return
        self._observe([start], -5)
        for index, measured in enumerate(ranges):
            if not math.isfinite(measured) or measured <= 0.0:
                continue
            hit = measured < range_max - 0.05
            distance = min(measured, range_max)
            angle = yaw + angle_min + index * angle_increment
            endpoint = (
                position[0] + distance * math.cos(angle),
                position[1] + distance * math.sin(angle),
            )
            end = self.world_to_grid(*endpoint)
            if end is None:
                # Clamp a ray endpoint just inside the map.
                endpoint = (
                    min(
                        self.origin[0] + self.width * self.resolution - 1e-4,
                        max(self.origin[0] + 1e-4, endpoint[0]),
                    ),
                    min(
                        self.origin[1] + self.height * self.resolution - 1e-4,
                        max(self.origin[1] + 1e-4, endpoint[1]),
                    ),
                )
                end = self.world_to_grid(*endpoint)
            if end is None:
                continue
            cells = self.bresenham(start, end)
            free_cells = cells[:-1] if hit else cells
            self._observe(free_cells, -2)
            if hit:
                self._observe([cells[-1]], 5)

    def states(self) -> np.ndarray:
        state = np.full((self.height, self.width), UNKNOWN, dtype=np.int8)
        known = self.observations > 0
        state[known & (self.log_odds <= 1)] = FREE
        state[known & (self.log_odds > 1)] = OCCUPIED
        return state

    def set_states(self, values: np.ndarray) -> None:
        if values.shape != (self.height, self.width):
            raise ValueError(
                f"map shape {values.shape} != {(self.height, self.width)}"
            )
        known = values >= 0
        self.observations[known] = np.maximum(self.observations[known], 1)
        self.log_odds[values == FREE] = -10
        self.log_odds[values >= OCCUPIED] = 10

    def merge(self, values: np.ndarray) -> None:
        """Merge a peer map; occupied evidence wins over free evidence."""

        if values.shape != (self.height, self.width):
            return
        known = values >= 0
        occupied = values >= OCCUPIED
        free = values == FREE
        self.observations[known] = np.maximum(self.observations[known], 1)
        self.log_odds[free & (self.log_odds <= 1)] = -10
        self.log_odds[occupied] = 10

    def coverage(self) -> float:
        return float(np.count_nonzero(self.observations)) / self.observations.size

    def inflated_blocked(
        self, clearance: float, unknown_is_blocked: bool = True
    ) -> np.ndarray:
        state = self.states()
        source = state != FREE if unknown_is_blocked else state == OCCUPIED
        radius = int(math.ceil(clearance / self.resolution))
        blocked = source.copy()
        occupied_cells = np.argwhere(source)
        for y, x in occupied_cells:
            y0, y1 = max(0, y - radius), min(self.height, y + radius + 1)
            x0, x1 = max(0, x - radius), min(self.width, x + radius + 1)
            for yy in range(y0, y1):
                for xx in range(x0, x1):
                    if (xx - x) ** 2 + (yy - y) ** 2 <= radius**2:
                        blocked[yy, xx] = True
        return blocked

    def frontier_clusters(self, minimum_size: int = 2) -> List[List[GridIndex]]:
        """Known-free cells adjacent to unknown cells, grouped in 8-connectivity."""

        state = self.states()
        frontier = np.zeros_like(state, dtype=bool)
        for y in range(1, self.height - 1):
            for x in range(1, self.width - 1):
                if state[y, x] != FREE:
                    continue
                if (
                    state[y - 1, x] == UNKNOWN
                    or state[y + 1, x] == UNKNOWN
                    or state[y, x - 1] == UNKNOWN
                    or state[y, x + 1] == UNKNOWN
                ):
                    frontier[y, x] = True
        visited = np.zeros_like(frontier)
        clusters: List[List[GridIndex]] = []
        for seed_y, seed_x in np.argwhere(frontier):
            if visited[seed_y, seed_x]:
                continue
            queue = deque([(int(seed_x), int(seed_y))])
            visited[seed_y, seed_x] = True
            cluster: List[GridIndex] = []
            while queue:
                x, y = queue.popleft()
                cluster.append((x, y))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if (
                            0 <= nx < self.width
                            and 0 <= ny < self.height
                            and frontier[ny, nx]
                            and not visited[ny, nx]
                        ):
                            visited[ny, nx] = True
                            queue.append((nx, ny))
            if len(cluster) >= minimum_size:
                clusters.append(cluster)
        return clusters

    def information_gain(
        self, cell: GridIndex, radius_m: float = 2.5
    ) -> int:
        radius = int(math.ceil(radius_m / self.resolution))
        state = self.states()
        gain = 0
        for y in range(max(0, cell[1] - radius), min(self.height, cell[1] + radius + 1)):
            for x in range(max(0, cell[0] - radius), min(self.width, cell[0] + radius + 1)):
                if (
                    (x - cell[0]) ** 2 + (y - cell[1]) ** 2 <= radius**2
                    and state[y, x] == UNKNOWN
                ):
                    gain += 1
        return gain
