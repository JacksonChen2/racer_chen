"""Trajectory visibility constraints used by active-perception optimization."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

from .voxel_map import VoxelMap


Array = NDArray[np.float64]


@dataclass(slots=True)
class VisibilityConstraint:
    control_index: int
    target: Array
    direction: Array
    minimum_clearance: float


class TrajectoryVisibility:
    def __init__(self, voxel_map: VoxelMap, clearance: float = 0.2) -> None:
        self.map = voxel_map
        self.clearance = clearance

    def line_visible(self, position: Array, target: Array) -> bool:
        return all(
            self.map.is_in_box(point)
            and self.map.get_inflated_occupancy(point) == 0
            and self.map.get_distance(point) >= self.clearance
            for point in self.map.ray_caster.positions(position, target)
        )

    def constraints(self, control_points: Array, targets: list[Array]) -> list[VisibilityConstraint]:
        points = np.asarray(control_points, dtype=np.float64)
        result: list[VisibilityConstraint] = []
        for target in targets:
            nearest = int(np.argmin(np.linalg.norm(points - target, axis=1)))
            if self.line_visible(points[nearest], target):
                continue
            direction = target - points[nearest]
            norm = float(np.linalg.norm(direction))
            if norm > 1.0e-9:
                direction /= norm
            result.append(VisibilityConstraint(nearest, np.asarray(target).copy(), direction, self.clearance))
        return result

