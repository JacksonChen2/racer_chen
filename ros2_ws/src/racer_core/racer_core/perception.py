"""Camera field-of-view geometry port of ``active_perception/perception_utils``."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .math_utils import rotation_from_yaw


Vector = NDArray[np.float64]


@dataclass(slots=True)
class PerceptionConfig:
    top_angle: float = 0.56125
    left_angle: float = 0.69222
    right_angle: float = 0.68901
    max_distance: float = 4.5
    visualization_distance: float = 1.0


class PerceptionUtils:
    def __init__(self, config: PerceptionConfig) -> None:
        self.config = config
        self.position = np.zeros(3, dtype=np.float64)
        self.yaw = 0.0
        self.normals: list[Vector] = []
        self.camera_normals = [
            np.asarray((0.0, math.sin(math.pi / 2 - config.top_angle), math.cos(math.pi / 2 - config.top_angle))),
            np.asarray((0.0, -math.sin(math.pi / 2 - config.top_angle), math.cos(math.pi / 2 - config.top_angle))),
            np.asarray((math.sin(math.pi / 2 - config.left_angle), 0.0, math.cos(math.pi / 2 - config.left_angle))),
            np.asarray((-math.sin(math.pi / 2 - config.right_angle), 0.0, math.cos(math.pi / 2 - config.right_angle))),
        ]
        self.camera_to_body = np.asarray(
            ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
        )
        distance = config.visualization_distance
        horizontal = distance * math.tan(config.left_angle)
        vertical = distance * math.tan(config.top_angle)
        origin = np.zeros(3)
        left_up = np.asarray((distance, horizontal, vertical))
        left_down = np.asarray((distance, horizontal, -vertical))
        right_up = np.asarray((distance, -horizontal, vertical))
        right_down = np.asarray((distance, -horizontal, -vertical))
        self.edges = [
            (origin, left_up),
            (origin, left_down),
            (origin, right_up),
            (origin, right_down),
            (left_up, right_up),
            (right_up, right_down),
            (right_down, left_down),
            (left_down, left_up),
        ]
        self.set_pose(self.position, 0.0)

    def set_pose(self, position: Vector, yaw: float) -> None:
        self.position = np.asarray(position, dtype=np.float64).copy()
        self.yaw = float(yaw)
        world_from_body = rotation_from_yaw(yaw)
        world_from_camera = world_from_body @ self.camera_to_body.T
        self.normals = [world_from_camera @ normal for normal in self.camera_normals]

    def fov_edges(self) -> tuple[list[Vector], list[Vector]]:
        rotation = rotation_from_yaw(self.yaw)
        starts = [rotation @ first + self.position for first, _ in self.edges]
        ends = [rotation @ second + self.position for _, second in self.edges]
        return starts, ends

    def inside_fov(self, point: Vector) -> bool:
        direction = np.asarray(point, dtype=np.float64) - self.position
        norm = float(np.linalg.norm(direction))
        if norm > self.config.max_distance:
            return False
        if norm < 1.0e-12:
            return True
        direction /= norm
        return all(float(np.dot(direction, normal)) >= 0.0 for normal in self.normals)

    def fov_bounding_box(self) -> tuple[Vector, Vector]:
        left = self.yaw + self.config.left_angle
        right = self.yaw - self.config.right_angle
        candidates = [
            self.position
            + self.config.max_distance * np.asarray((math.cos(left), math.sin(left), 0.0)),
            self.position
            + self.config.max_distance * np.asarray((math.cos(right), math.sin(right), 0.0)),
        ]
        if left > 0.0 > right:
            candidates.append(self.position + self.config.max_distance * np.asarray((1.0, 0.0, 0.0)))
        elif left > math.pi / 2 > right:
            candidates.append(self.position + self.config.max_distance * np.asarray((0.0, 1.0, 0.0)))
        elif left > -math.pi / 2 > right:
            candidates.append(self.position + self.config.max_distance * np.asarray((0.0, -1.0, 0.0)))
        elif (left > math.pi > right) or (left > -math.pi > right):
            candidates.append(self.position + self.config.max_distance * np.asarray((-1.0, 0.0, 0.0)))
        points = np.vstack([self.position, *candidates])
        return np.min(points, axis=0), np.max(points, axis=0)
