"""Online hierarchical grid decomposition from RACER Section IV."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .mapping import FREE, UNKNOWN, OccupancyMap


@dataclass(frozen=True)
class HGridCell:
    level: int
    ix: int
    iy: int
    bounds: Tuple[int, int, int, int]
    centroid: Tuple[float, float]
    unknown: int
    total: int

    @property
    def id(self) -> str:
        return f"{self.level}:{self.ix}:{self.iy}"

    @property
    def demand(self) -> int:
        return self.unknown


class HierarchicalGrid:
    """Array-addressable 2.5-D hgrid with coarse-to-fine active cells."""

    def __init__(
        self,
        occupancy_map: OccupancyMap,
        coarse_size: float = 5.0,
        levels: int = 2,
        subdivide_known_ratio: float = 0.35,
        minimum_unknown: int = 3,
    ) -> None:
        if levels < 1:
            raise ValueError("hgrid needs at least one level")
        self.map = occupancy_map
        self.coarse_size = coarse_size
        self.levels = levels
        self.subdivide_known_ratio = subdivide_known_ratio
        self.minimum_unknown = minimum_unknown
        self._cells: Dict[str, HGridCell] = {}

    def _cell_from_index(self, level: int, ix: int, iy: int) -> HGridCell:
        factor = 2 ** (level - 1)
        span_m = self.coarse_size / factor
        span = max(1, int(round(span_m / self.map.resolution)))
        x0, y0 = ix * span, iy * span
        x1 = min(self.map.width, x0 + span)
        y1 = min(self.map.height, y0 + span)
        states = self.map.states()[y0:y1, x0:x1]
        unknown_mask = states == UNKNOWN
        unknown = int(np.count_nonzero(unknown_mask))
        if unknown:
            yy, xx = np.argwhere(unknown_mask).mean(axis=0)
            centroid = self.map.grid_to_world(
                (x0 + int(round(float(xx))), y0 + int(round(float(yy))))
            )
        else:
            centroid = self.map.grid_to_world(
                (min(self.map.width - 1, (x0 + x1) // 2),
                 min(self.map.height - 1, (y0 + y1) // 2))
            )
        return HGridCell(
            level,
            ix,
            iy,
            (x0, y0, x1, y1),
            centroid,
            unknown,
            max(1, states.size),
        )

    def update(self) -> Dict[str, HGridCell]:
        """Recompute the active decomposition using Algorithm 1 semantics."""

        active: Dict[str, HGridCell] = {}

        def visit(level: int, ix: int, iy: int) -> None:
            cell = self._cell_from_index(level, ix, iy)
            if cell.bounds[0] >= cell.bounds[2] or cell.bounds[1] >= cell.bounds[3]:
                return
            known_ratio = 1.0 - cell.unknown / cell.total
            if (
                level < self.levels
                and known_ratio >= self.subdivide_known_ratio
                and cell.unknown >= self.minimum_unknown
            ):
                for dy in (0, 1):
                    for dx in (0, 1):
                        visit(level + 1, ix * 2 + dx, iy * 2 + dy)
                return
            if level == self.levels and cell.unknown < self.minimum_unknown:
                return
            if cell.unknown > 0:
                active[cell.id] = cell

        coarse_span = max(1, int(round(self.coarse_size / self.map.resolution)))
        nx = int(math.ceil(self.map.width / coarse_span))
        ny = int(math.ceil(self.map.height / coarse_span))
        for iy in range(ny):
            for ix in range(nx):
                visit(1, ix, iy)
        self._cells = active
        return dict(active)

    @property
    def cells(self) -> Dict[str, HGridCell]:
        return dict(self._cells)

    def containing(
        self, point: Tuple[float, float], cells: Optional[Iterable[HGridCell]] = None
    ) -> Optional[HGridCell]:
        index = self.map.world_to_grid(*point)
        if index is None:
            return None
        candidates = list(cells) if cells is not None else list(self._cells.values())
        candidates.sort(key=lambda item: item.level, reverse=True)
        for cell in candidates:
            x0, y0, x1, y1 = cell.bounds
            if x0 <= index[0] < x1 and y0 <= index[1] < y1:
                return cell
        return None

    def initial_owners(
        self,
        starts: Sequence[Tuple[float, float]],
    ) -> Dict[str, int]:
        """Deterministic distributed bootstrap before pairwise interactions."""

        owners: Dict[str, int] = {}
        load = [0 for _ in starts]
        ordered = sorted(self._cells.values(), key=lambda item: -item.demand)
        for cell in ordered:
            scores = []
            for drone_id, start in enumerate(starts):
                distance = math.hypot(
                    cell.centroid[0] - start[0], cell.centroid[1] - start[1]
                )
                scores.append(distance + 0.015 * load[drone_id])
            owner = min(range(len(scores)), key=scores.__getitem__)
            owners[cell.id] = owner
            load[owner] += cell.demand
        return owners

    def coverage_route(
        self,
        cell_ids: Sequence[str],
        start: Tuple[float, float],
    ) -> List[str]:
        """Open nearest-insertion route followed by 2-opt."""

        remaining = [self._cells[item] for item in cell_ids if item in self._cells]
        route: List[HGridCell] = []
        cursor = start
        while remaining:
            next_cell = min(
                remaining,
                key=lambda cell: math.hypot(
                    cell.centroid[0] - cursor[0],
                    cell.centroid[1] - cursor[1],
                ),
            )
            route.append(next_cell)
            remaining.remove(next_cell)
            cursor = next_cell.centroid
        if len(route) > 3:
            route = self._two_opt(route, start)
        return [cell.id for cell in route]

    @staticmethod
    def _two_opt(
        route: List[HGridCell], start: Tuple[float, float]
    ) -> List[HGridCell]:
        def length(items: Sequence[HGridCell]) -> float:
            total = 0.0
            cursor = start
            for item in items:
                total += math.hypot(
                    item.centroid[0] - cursor[0],
                    item.centroid[1] - cursor[1],
                )
                cursor = item.centroid
            return total

        best = list(route)
        best_cost = length(best)
        changed = True
        while changed:
            changed = False
            for i in range(len(best) - 1):
                for j in range(i + 2, len(best) + 1):
                    candidate = best[:i] + list(reversed(best[i:j])) + best[j:]
                    cost = length(candidate)
                    if cost + 1.0e-9 < best_cost:
                        best, best_cost, changed = candidate, cost, True
        return best
