"""Probabilistic 3-D voxel occupancy, ESDF, and frontier extraction."""

from collections import deque
import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np


UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(100)
GridIndex3 = Tuple[int, int, int]
Point3 = Tuple[float, float, float]


class VoxelMap:
    """Shared-frame log-odds voxel map stored in ``(z, y, x)`` order."""

    def __init__(
        self,
        resolution: float,
        origin: Sequence[float],
        size: Sequence[float],
    ) -> None:
        self.resolution = float(resolution)
        self.origin = tuple(float(value) for value in origin)
        self.size = tuple(float(value) for value in size)
        self.nx, self.ny, self.nz = (
            int(math.ceil(value / self.resolution)) for value in self.size
        )
        shape = (self.nz, self.ny, self.nx)
        self.log_odds = np.zeros(shape, dtype=np.int16)
        self.observations = np.zeros(shape, dtype=np.uint16)
        self._esdf_cache: dict[bool, np.ndarray] = {}

    @property
    def shape(self) -> Tuple[int, int, int]:
        return self.log_odds.shape

    def copy(self) -> "VoxelMap":
        result = VoxelMap(self.resolution, self.origin, self.size)
        result.log_odds = self.log_odds.copy()
        result.observations = self.observations.copy()
        return result

    def in_bounds(self, cell: GridIndex3) -> bool:
        x, y, z = cell
        return 0 <= x < self.nx and 0 <= y < self.ny and 0 <= z < self.nz

    def world_to_grid(
        self, point: Sequence[float]
    ) -> Optional[GridIndex3]:
        cell = tuple(
            int(math.floor((point[index] - self.origin[index]) / self.resolution))
            for index in range(3)
        )
        return cell if self.in_bounds(cell) else None

    def grid_to_world(self, cell: GridIndex3) -> Point3:
        return tuple(
            self.origin[index] + (cell[index] + 0.5) * self.resolution
            for index in range(3)
        )

    @staticmethod
    def line_cells(
        start: Sequence[float],
        end: Sequence[float],
        world_to_grid,
        resolution: float,
    ) -> List[GridIndex3]:
        """Conservative sampled 3-D DDA with duplicate removal."""

        first = np.asarray(start, dtype=float)
        second = np.asarray(end, dtype=float)
        distance = float(np.linalg.norm(second - first))
        samples = max(1, int(math.ceil(distance / (0.35 * resolution))))
        result: List[GridIndex3] = []
        previous = None
        for index in range(samples + 1):
            point = first + (second - first) * (index / samples)
            cell = world_to_grid(point)
            if cell is not None and cell != previous:
                result.append(cell)
                previous = cell
        return result

    def ray_cells(
        self, start: Sequence[float], end: Sequence[float]
    ) -> List[GridIndex3]:
        return self.line_cells(
            start, end, self.world_to_grid, self.resolution
        )

    def _observe(self, cells: Iterable[GridIndex3], delta: int) -> None:
        changed = False
        for x, y, z in cells:
            if not self.in_bounds((x, y, z)):
                continue
            self.log_odds[z, y, x] = np.int16(
                np.clip(int(self.log_odds[z, y, x]) + delta, -30, 30)
            )
            if self.observations[z, y, x] < np.iinfo(np.uint16).max:
                self.observations[z, y, x] += 1
            changed = True
        if changed:
            self._esdf_cache.clear()

    def _observe_bulk(
        self, cells: Sequence[GridIndex3], delta: int
    ) -> None:
        """Vectorized evidence accumulation for a complete point-cloud frame."""

        if not cells:
            return
        values = np.asarray(cells, dtype=np.int32).reshape((-1, 3))
        valid = (
            (values[:, 0] >= 0)
            & (values[:, 0] < self.nx)
            & (values[:, 1] >= 0)
            & (values[:, 1] < self.ny)
            & (values[:, 2] >= 0)
            & (values[:, 2] < self.nz)
        )
        values = values[valid]
        if not len(values):
            return
        flat = (
            values[:, 2] * self.ny * self.nx
            + values[:, 1] * self.nx
            + values[:, 0]
        )
        unique, counts = np.unique(flat, return_counts=True)
        log_flat = self.log_odds.reshape(-1)
        observation_flat = self.observations.reshape(-1)
        log_flat[unique] = np.clip(
            log_flat[unique].astype(np.int32) + delta * counts,
            -30,
            30,
        ).astype(np.int16)
        observation_flat[unique] = np.minimum(
            np.iinfo(np.uint16).max,
            observation_flat[unique].astype(np.uint32) + counts,
        ).astype(np.uint16)
        self._esdf_cache.clear()

    def update_point_cloud(
        self,
        sensor_origin: Sequence[float],
        points_world: np.ndarray,
        maximum_range: float,
        hit_mask: Optional[np.ndarray] = None,
        maximum_rays: int = 2400,
    ) -> None:
        """Integrate hit/miss rays from a depth camera or 3-D lidar cloud."""

        origin = np.asarray(sensor_origin, dtype=float)
        points = np.asarray(points_world, dtype=float).reshape((-1, 3))
        if hit_mask is None:
            hit_mask = np.ones(len(points), dtype=bool)
        else:
            hit_mask = np.asarray(hit_mask, dtype=bool).reshape(-1)
        if len(points) > maximum_rays:
            indices = np.linspace(
                0, len(points) - 1, maximum_rays, dtype=np.int64
            )
            points, hit_mask = points[indices], hit_mask[indices]
        start_cell = self.world_to_grid(origin)
        free_evidence: List[GridIndex3] = []
        occupied_evidence: List[GridIndex3] = []
        if start_cell is not None:
            free_evidence.extend([start_cell] * 3)
        for point, is_hit in zip(points, hit_mask):
            if not np.all(np.isfinite(point)):
                continue
            vector = point - origin
            distance = float(np.linalg.norm(vector))
            if distance < 0.03:
                continue
            clipped = min(distance, maximum_range)
            endpoint = origin + vector * (clipped / distance)
            cells = self.ray_cells(origin, endpoint)
            if not cells:
                continue
            actual_hit = bool(is_hit and distance < maximum_range - 0.03)
            free_cells = cells[:-1] if actual_hit else cells
            free_evidence.extend(free_cells)
            if actual_hit:
                # Move half a cell behind the measured surface. This prevents
                # floor operations at a voxel boundary selecting its free side.
                occupied_point = origin + vector * (
                    (min(maximum_range, distance + 0.5 * self.resolution))
                    / distance
                )
                occupied = self.world_to_grid(occupied_point)
                if occupied is None and cells:
                    # A hit exactly on a configured map boundary (wall,
                    # floor, ceiling) has its half-voxel continuation outside
                    # the array. Preserve the last in-bounds surface voxel.
                    occupied = cells[-1]
                if occupied is not None:
                    occupied_evidence.append(occupied)
        self._observe_bulk(free_evidence, -2)
        self._observe_bulk(occupied_evidence, 8)

    def states(self) -> np.ndarray:
        result = np.full(self.shape, UNKNOWN, dtype=np.int8)
        known = self.observations > 0
        result[known & (self.log_odds <= -2)] = FREE
        result[known & (self.log_odds >= 2)] = OCCUPIED
        return result

    def set_states(self, states: np.ndarray) -> None:
        values = np.asarray(states, dtype=np.int8)
        if values.shape != self.shape:
            raise ValueError(f"map shape {values.shape} != {self.shape}")
        known = values >= 0
        self.observations.fill(0)
        self.log_odds.fill(0)
        self.observations[known] = 1
        self.log_odds[values == FREE] = -10
        self.log_odds[values == OCCUPIED] = 10
        self._esdf_cache.clear()

    def merge(self, states: np.ndarray) -> None:
        values = np.asarray(states, dtype=np.int8)
        if values.shape != self.shape:
            return
        known = values >= 0
        free = values == FREE
        occupied = values == OCCUPIED
        self.observations[known] = np.maximum(self.observations[known], 1)
        self.log_odds[free & (self.log_odds < 2)] = np.minimum(
            self.log_odds[free & (self.log_odds < 2)], -8
        )
        self.log_odds[occupied] = np.maximum(
            self.log_odds[occupied], 10
        )
        self._esdf_cache.clear()

    def coverage(self) -> float:
        return float(np.count_nonzero(self.states() != UNKNOWN)) / float(
            self.observations.size
        )

    def esdf(self, unknown_is_occupied: bool = True) -> np.ndarray:
        """Return a signed Euclidean distance field in metres."""

        cached = self._esdf_cache.get(unknown_is_occupied)
        if cached is not None:
            return cached
        state = self.states()
        source = state == OCCUPIED
        if unknown_is_occupied:
            source |= state == UNKNOWN
        try:
            import warnings

            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                from scipy.ndimage import distance_transform_edt

            outside = distance_transform_edt(~source) * self.resolution
            inside = distance_transform_edt(source) * self.resolution
            distance = outside.astype(np.float32)
            distance[source] = -inside[source]
        except (ImportError, ValueError):
            distance = self._dijkstra_distance(source)
            distance[source] = -0.5 * self.resolution
        self._esdf_cache[unknown_is_occupied] = distance
        return distance

    def _dijkstra_distance(self, source: np.ndarray) -> np.ndarray:
        import heapq

        distance = np.full(self.shape, np.inf, dtype=np.float32)
        queue = []
        for z, y, x in np.argwhere(source):
            distance[z, y, x] = 0.0
            heapq.heappush(queue, (0.0, int(x), int(y), int(z)))
        moves = [
            (dx, dy, dz, self.resolution * math.sqrt(dx * dx + dy * dy + dz * dz))
            for dz in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
            if dx or dy or dz
        ]
        while queue:
            cost, x, y, z = heapq.heappop(queue)
            if cost > float(distance[z, y, x]) + 1.0e-6:
                continue
            for dx, dy, dz, step in moves:
                cell = x + dx, y + dy, z + dz
                if not self.in_bounds(cell):
                    continue
                candidate = cost + step
                if candidate + 1.0e-6 < distance[cell[2], cell[1], cell[0]]:
                    distance[cell[2], cell[1], cell[0]] = candidate
                    heapq.heappush(queue, (candidate, *cell))
        return distance

    def distance_at(
        self, point: Sequence[float], unknown_is_occupied: bool = True
    ) -> float:
        value = np.asarray(point, dtype=float)
        coordinates = (
            (value - np.asarray(self.origin)) / self.resolution - 0.5
        )
        if np.any(coordinates < 0.0) or np.any(
            coordinates > np.asarray((self.nx - 1, self.ny - 1, self.nz - 1))
        ):
            return -math.inf
        lower = np.floor(coordinates).astype(int)
        upper = np.minimum(
            lower + 1, np.asarray((self.nx - 1, self.ny - 1, self.nz - 1))
        )
        ratio = coordinates - lower
        field = self.esdf(unknown_is_occupied)
        result = 0.0
        for dz in (0, 1):
            z = lower[2] if dz == 0 else upper[2]
            wz = 1.0 - ratio[2] if dz == 0 else ratio[2]
            for dy in (0, 1):
                y = lower[1] if dy == 0 else upper[1]
                wy = 1.0 - ratio[1] if dy == 0 else ratio[1]
                for dx in (0, 1):
                    x = lower[0] if dx == 0 else upper[0]
                    wx = 1.0 - ratio[0] if dx == 0 else ratio[0]
                    result += float(field[z, y, x]) * wx * wy * wz
        return result

    def esdf_gradient(
        self, point: Sequence[float], unknown_is_occupied: bool = True
    ) -> np.ndarray:
        value = np.asarray(point, dtype=float)
        if self.world_to_grid(value) is None:
            return np.zeros(3)
        gradient = np.zeros(3)
        step = 0.35 * self.resolution
        for axis in range(3):
            lower, upper = value.copy(), value.copy()
            lower[axis] -= step
            upper[axis] += step
            low = self.distance_at(lower, unknown_is_occupied)
            high = self.distance_at(upper, unknown_is_occupied)
            if math.isfinite(low) and math.isfinite(high):
                gradient[axis] = (high - low) / (2.0 * step)
        return gradient

    def inflated_blocked(
        self, clearance: float, unknown_is_blocked: bool = True
    ) -> np.ndarray:
        return self.esdf(unknown_is_blocked) < float(clearance)

    def frontier_mask(self) -> np.ndarray:
        """Free voxels with a six-connected unknown neighbour."""

        state = self.states()
        free = state == FREE
        unknown = state == UNKNOWN
        adjacent = np.zeros_like(free)
        adjacent[1:, :, :] |= unknown[:-1, :, :]
        adjacent[:-1, :, :] |= unknown[1:, :, :]
        adjacent[:, 1:, :] |= unknown[:, :-1, :]
        adjacent[:, :-1, :] |= unknown[:, 1:, :]
        adjacent[:, :, 1:] |= unknown[:, :, :-1]
        adjacent[:, :, :-1] |= unknown[:, :, 1:]
        return free & adjacent

    def frontier_clusters(
        self, minimum_size: int = 4
    ) -> List[List[GridIndex3]]:
        frontier = self.frontier_mask()
        visited = np.zeros_like(frontier)
        clusters: List[List[GridIndex3]] = []
        for seed_z, seed_y, seed_x in np.argwhere(frontier):
            if visited[seed_z, seed_y, seed_x]:
                continue
            queue = deque([(int(seed_x), int(seed_y), int(seed_z))])
            visited[seed_z, seed_y, seed_x] = True
            cluster = []
            while queue:
                x, y, z = queue.popleft()
                cluster.append((x, y, z))
                for dz in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if not (dx or dy or dz):
                                continue
                            cell = x + dx, y + dy, z + dz
                            if (
                                self.in_bounds(cell)
                                and frontier[cell[2], cell[1], cell[0]]
                                and not visited[cell[2], cell[1], cell[0]]
                            ):
                                visited[cell[2], cell[1], cell[0]] = True
                                queue.append(cell)
            if len(cluster) >= minimum_size:
                clusters.append(cluster)
        return clusters

    def information_gain(
        self, point: Sequence[float], radius_m: float = 2.5
    ) -> int:
        center = self.world_to_grid(point)
        if center is None:
            return 0
        radius = int(math.ceil(radius_m / self.resolution))
        state = self.states()
        gain = 0
        for z in range(max(0, center[2] - radius),
                       min(self.nz, center[2] + radius + 1)):
            for y in range(max(0, center[1] - radius),
                           min(self.ny, center[1] + radius + 1)):
                for x in range(max(0, center[0] - radius),
                               min(self.nx, center[0] + radius + 1)):
                    if (
                        (x - center[0]) ** 2
                        + (y - center[1]) ** 2
                        + (z - center[2]) ** 2
                        <= radius**2
                        and state[z, y, x] == UNKNOWN
                    ):
                        gain += 1
        return gain

    def visible_unknown_gain(
        self,
        viewpoint: Sequence[float],
        cluster: Sequence[GridIndex3],
        maximum_rays: int = 48,
    ) -> int:
        """Count unknown voxels visible behind a sampled frontier surface."""

        state = self.states()
        if not cluster:
            return 0
        stride = max(1, len(cluster) // maximum_rays)
        gain_cells = set()
        for cell in cluster[::stride]:
            target = np.asarray(self.grid_to_world(cell))
            ray = target - np.asarray(viewpoint)
            distance = float(np.linalg.norm(ray))
            if distance < 1.0e-6:
                continue
            endpoint = target + ray / distance * 2.0
            crossed = self.ray_cells(viewpoint, endpoint)
            blocked = False
            for candidate in crossed:
                value = state[candidate[2], candidate[1], candidate[0]]
                if value == OCCUPIED:
                    blocked = True
                    break
                if value == UNKNOWN:
                    gain_cells.add(candidate)
            if blocked:
                continue
        return len(gain_cells)
