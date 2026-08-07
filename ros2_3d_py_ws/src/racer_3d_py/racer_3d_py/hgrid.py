"""RACER-style online hierarchical grid decomposition in three dimensions."""

from dataclasses import dataclass
import math
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .voxel_map import UNKNOWN, VoxelMap


@dataclass(frozen=True)
class HGridCell3D:
    level: int
    ix: int
    iy: int
    iz: int
    bounds: Tuple[int, int, int, int, int, int]
    centroid: Tuple[float, float, float]
    unknown: int
    total: int

    @property
    def id(self) -> str:
        return f"{self.level}:{self.ix}:{self.iy}:{self.iz}"

    @property
    def demand(self) -> int:
        return self.unknown


class HierarchicalGrid3D:
    """Coarse-to-fine octree-like active-cell decomposition."""

    def __init__(
        self,
        voxel_map: VoxelMap,
        coarse_size: Sequence[float] = (5.0, 4.5, 2.0),
        levels: int = 2,
        subdivide_known_ratio: float = 0.35,
        minimum_unknown: int = 4,
    ) -> None:
        self.map = voxel_map
        self.coarse_size = tuple(float(value) for value in coarse_size)
        self.levels = int(levels)
        self.subdivide_known_ratio = float(subdivide_known_ratio)
        self.minimum_unknown = int(minimum_unknown)
        if self.levels < 1:
            raise ValueError("hgrid needs at least one level")
        self._cells: Dict[str, HGridCell3D] = {}

    def _span(self, level: int) -> Tuple[int, int, int]:
        factor = 2 ** (level - 1)
        return tuple(
            max(1, int(round(value / factor / self.map.resolution)))
            for value in self.coarse_size
        )

    def _cell_from_index(
        self, level: int, ix: int, iy: int, iz: int
    ) -> HGridCell3D:
        span_x, span_y, span_z = self._span(level)
        x0, y0, z0 = ix * span_x, iy * span_y, iz * span_z
        x1 = min(self.map.nx, x0 + span_x)
        y1 = min(self.map.ny, y0 + span_y)
        z1 = min(self.map.nz, z0 + span_z)
        values = self.map.states()[z0:z1, y0:y1, x0:x1]
        unknown_mask = values == UNKNOWN
        unknown = int(np.count_nonzero(unknown_mask))
        if unknown:
            zz, yy, xx = np.argwhere(unknown_mask).mean(axis=0)
            index = (
                x0 + int(round(float(xx))),
                y0 + int(round(float(yy))),
                z0 + int(round(float(zz))),
            )
        else:
            index = (
                min(self.map.nx - 1, (x0 + x1) // 2),
                min(self.map.ny - 1, (y0 + y1) // 2),
                min(self.map.nz - 1, (z0 + z1) // 2),
            )
        return HGridCell3D(
            level=level,
            ix=ix,
            iy=iy,
            iz=iz,
            bounds=(x0, y0, z0, x1, y1, z1),
            centroid=self.map.grid_to_world(index),
            unknown=unknown,
            total=max(1, int(values.size)),
        )

    def update(self) -> Dict[str, HGridCell3D]:
        active: Dict[str, HGridCell3D] = {}

        def visit(level: int, ix: int, iy: int, iz: int) -> None:
            cell = self._cell_from_index(level, ix, iy, iz)
            x0, y0, z0, x1, y1, z1 = cell.bounds
            if x0 >= x1 or y0 >= y1 or z0 >= z1:
                return
            known_ratio = 1.0 - cell.unknown / cell.total
            if (
                level < self.levels
                and known_ratio >= self.subdivide_known_ratio
                and cell.unknown >= self.minimum_unknown
            ):
                for dz in (0, 1):
                    for dy in (0, 1):
                        for dx in (0, 1):
                            visit(
                                level + 1,
                                ix * 2 + dx,
                                iy * 2 + dy,
                                iz * 2 + dz,
                            )
                return
            if cell.unknown >= self.minimum_unknown:
                active[cell.id] = cell

        span_x, span_y, span_z = self._span(1)
        for iz in range(int(math.ceil(self.map.nz / span_z))):
            for iy in range(int(math.ceil(self.map.ny / span_y))):
                for ix in range(int(math.ceil(self.map.nx / span_x))):
                    visit(1, ix, iy, iz)
        self._cells = active
        return dict(active)

    @property
    def cells(self) -> Dict[str, HGridCell3D]:
        return dict(self._cells)

    def containing(
        self,
        point: Sequence[float],
        cells: Optional[Iterable[HGridCell3D]] = None,
    ) -> Optional[HGridCell3D]:
        index = self.map.world_to_grid(point)
        if index is None:
            return None
        candidates = list(cells) if cells is not None else list(self._cells.values())
        for cell in sorted(candidates, key=lambda item: item.level, reverse=True):
            x0, y0, z0, x1, y1, z1 = cell.bounds
            if (
                x0 <= index[0] < x1
                and y0 <= index[1] < y1
                and z0 <= index[2] < z1
            ):
                return cell
        return None

    def initial_owners(
        self, starts: Sequence[Sequence[float]]
    ) -> Dict[str, int]:
        """Demand-balanced deterministic distributed initialization."""

        owners: Dict[str, int] = {}
        load = [0 for _ in starts]
        for cell in sorted(self._cells.values(), key=lambda item: -item.demand):
            scores = [
                float(np.linalg.norm(np.asarray(cell.centroid) - np.asarray(start)))
                + 0.004 * load[drone_id]
                for drone_id, start in enumerate(starts)
            ]
            owner = min(range(len(scores)), key=scores.__getitem__)
            owners[cell.id] = owner
            load[owner] += cell.demand
        return owners

    def coverage_route(
        self,
        cell_ids: Sequence[str],
        start: Sequence[float],
        distance_function: Optional[
            Callable[[Sequence[float], Sequence[float]], float]
        ] = None,
    ) -> List[str]:
        remaining = [
            self._cells[item] for item in cell_ids if item in self._cells
        ]
        route: List[HGridCell3D] = []
        cursor = np.asarray(start, dtype=float)

        def distance(first, second) -> float:
            if distance_function is not None:
                value = float(distance_function(first, second))
                if math.isfinite(value):
                    return value
            return float(np.linalg.norm(np.asarray(second) - np.asarray(first)))

        while remaining:
            selected = min(
                remaining,
                key=lambda cell: distance(cursor, cell.centroid),
            )
            route.append(selected)
            remaining.remove(selected)
            cursor = np.asarray(selected.centroid)
        if len(route) > 3:
            route = self._two_opt(route, start, distance_function)
        return [cell.id for cell in route]

    @staticmethod
    def _two_opt(
        route: List[HGridCell3D],
        start: Sequence[float],
        distance_function=None,
    ) -> List[HGridCell3D]:
        def distance(first, second) -> float:
            if distance_function is not None:
                value = float(distance_function(first, second))
                if math.isfinite(value):
                    return value
            return float(np.linalg.norm(np.asarray(second) - np.asarray(first)))

        def cost(items: Sequence[HGridCell3D]) -> float:
            cursor = np.asarray(start, dtype=float)
            total = 0.0
            for item in items:
                target = np.asarray(item.centroid)
                total += distance(cursor, target)
                cursor = target
            return total

        best = list(route)
        best_cost = cost(best)
        changed = True
        while changed:
            changed = False
            for first in range(len(best) - 1):
                for last in range(first + 2, len(best) + 1):
                    candidate = (
                        best[:first]
                        + list(reversed(best[first:last]))
                        + best[last:]
                    )
                    candidate_cost = cost(candidate)
                    if candidate_cost + 1.0e-9 < best_cost:
                        best, best_cost = candidate, candidate_cost
                        changed = True
        return best
