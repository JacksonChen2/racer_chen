"""Euclidean-distance environment used by path and trajectory optimization."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from .voxel_map import VoxelMap


Vector = NDArray[np.float64]


@dataclass(slots=True)
class PredictedBox:
    position: Vector
    velocity: Vector
    scale: Vector
    reference_time: float = 0.0
    drone_id: int = -1

    def evaluate_constant_velocity(self, time: float) -> Vector:
        return self.position + (float(time) - self.reference_time) * self.velocity


@dataclass(slots=True)
class EDTEnvironment:
    voxel_map: VoxelMap
    predicted_boxes: list[PredictedBox] = field(default_factory=list)

    def distance_to_box(self, box: PredictedBox, position: Vector, time: float) -> float:
        center = box.evaluate_constant_velocity(time)
        maximum = center + 0.5 * box.scale
        minimum = center - 0.5 * box.scale
        point = np.asarray(position, dtype=np.float64)
        distance = np.where(
            np.logical_and(point >= minimum, point <= maximum),
            0.0,
            np.minimum(np.abs(point - minimum), np.abs(point - maximum)),
        )
        return float(np.linalg.norm(distance))

    def evaluate_with_gradient(self, position: Vector, time: float = -1.0) -> tuple[float, Vector]:
        return self.voxel_map.get_distance_with_gradient(position)

    def evaluate_coarse(self, position: Vector, time: float = -1.0) -> float:
        static_distance = self.voxel_map.get_distance(position)
        if time < 0.0 or not self.predicted_boxes:
            return static_distance
        dynamic_distance = min(
            self.distance_to_box(box, position, time) for box in self.predicted_boxes
        )
        return min(static_distance, dynamic_distance)
