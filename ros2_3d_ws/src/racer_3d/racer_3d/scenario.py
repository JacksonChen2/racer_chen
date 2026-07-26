"""Deterministic three-dimensional Isaac/ROS acceptance scene."""

from dataclasses import dataclass
import math
from typing import Iterable, Sequence, Tuple

import numpy as np


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Box3D:
    """Axis-aligned solid box in the shared map frame."""

    center: Point3
    size: Point3
    name: str = "box"

    @property
    def minimum(self) -> Point3:
        return tuple(
            self.center[index] - 0.5 * self.size[index] for index in range(3)
        )

    @property
    def maximum(self) -> Point3:
        return tuple(
            self.center[index] + 0.5 * self.size[index] for index in range(3)
        )

    def contains(self, point: Sequence[float], margin: float = 0.0) -> bool:
        return all(
            lower - margin <= value <= upper + margin
            for value, lower, upper in zip(point, self.minimum, self.maximum)
        )


@dataclass(frozen=True)
class Scenario3D:
    name: str
    map_min: Point3
    map_max: Point3
    starts: Tuple[Point3, ...]
    obstacles: Tuple[Box3D, ...]

    @property
    def map_size(self) -> Point3:
        return tuple(
            self.map_max[index] - self.map_min[index] for index in range(3)
        )


DRONE_RADIUS = 0.12


def _room_boundaries(
    half_x: float, half_y: float, height: float, thickness: float = 0.12
) -> Tuple[Box3D, ...]:
    return (
        Box3D((0.0, 0.0, -0.5 * thickness),
              (2.0 * half_x, 2.0 * half_y, thickness), "floor"),
        Box3D((0.0, 0.0, height + 0.5 * thickness),
              (2.0 * half_x, 2.0 * half_y, thickness), "ceiling"),
        Box3D((-half_x - 0.5 * thickness, 0.0, 0.5 * height),
              (thickness, 2.0 * half_y, height), "west_wall"),
        Box3D((half_x + 0.5 * thickness, 0.0, 0.5 * height),
              (thickness, 2.0 * half_y, height), "east_wall"),
        Box3D((0.0, -half_y - 0.5 * thickness, 0.5 * height),
              (2.0 * half_x, thickness, height), "south_wall"),
        Box3D((0.0, half_y + 0.5 * thickness, 0.5 * height),
              (2.0 * half_x, thickness, height), "north_wall"),
    )


def acceptance_scene() -> Scenario3D:
    """15 x 9 x 2 m room containing genuinely height-dependent routes."""

    height = 2.0
    obstacles = _room_boundaries(7.5, 4.5, height) + (
        # A low partition can be crossed only by climbing.
        Box3D((-1.8, -2.0, 0.52), (0.40, 4.6, 1.04), "low_partition"),
        # A suspended partition forces a low-altitude route.
        Box3D((1.8, 2.0, 1.55), (0.40, 4.6, 0.90), "high_partition"),
        # Distributed clutter for 3-D surface reconstruction.
        Box3D((-4.2, 2.6, 0.75), (1.00, 1.00, 1.50), "west_column"),
        Box3D((0.0, 0.0, 1.00), (1.10, 1.10, 2.00), "center_column"),
        Box3D((4.4, -2.6, 0.60), (1.20, 1.00, 1.20), "east_low_block"),
        Box3D((5.2, 2.4, 1.30), (0.90, 1.10, 1.40), "east_hanging_block"),
    )
    return Scenario3D(
        name="acceptance_15x9x2",
        map_min=(-7.5, -4.5, 0.0),
        map_max=(7.5, 4.5, 2.0),
        starts=((-6.4, -3.2, 0.45), (-6.4, 0.0, 1.00), (-6.4, 3.2, 1.55)),
        obstacles=obstacles,
    )


DEFAULT_SCENARIO = acceptance_scene()


def point_box_signed_clearance(point: Sequence[float], box: Box3D) -> float:
    """Euclidean clearance outside a box and negative penetration inside."""

    minimum = box.minimum
    maximum = box.maximum
    delta = [
        max(minimum[index] - point[index], 0.0, point[index] - maximum[index])
        for index in range(3)
    ]
    if any(value > 0.0 for value in delta):
        return float(np.linalg.norm(delta))
    return -min(
        min(point[index] - minimum[index], maximum[index] - point[index])
        for index in range(3)
    )


def obstacle_clearance(
    point: Sequence[float], obstacles: Sequence[Box3D]
) -> float:
    return min(point_box_signed_clearance(point, box) for box in obstacles)


def ray_box_distance(
    origin: Sequence[float], direction: Sequence[float], box: Box3D
) -> float:
    t_min, t_max = -math.inf, math.inf
    for value, axis, lower, upper in zip(
        origin, direction, box.minimum, box.maximum
    ):
        if abs(axis) < 1.0e-12:
            if value < lower or value > upper:
                return math.inf
            continue
        first, second = (lower - value) / axis, (upper - value) / axis
        t_min = max(t_min, min(first, second))
        t_max = min(t_max, max(first, second))
        if t_min > t_max:
            return math.inf
    if t_max < 0.0:
        return math.inf
    return max(0.0, t_min)


def simulate_point_cloud(
    position: Sequence[float],
    azimuth_count: int = 90,
    elevation_count: int = 15,
    vertical_fov: float = math.radians(100.0),
    maximum_range: float = 7.0,
    obstacles: Sequence[Box3D] = DEFAULT_SCENARIO.obstacles,
) -> np.ndarray:
    """Ray-cast a dense local-frame 3-D cloud for the ROS-only backend."""

    points = []
    for elevation in np.linspace(-0.5 * vertical_fov, 0.5 * vertical_fov,
                                 elevation_count):
        cos_elevation = math.cos(float(elevation))
        for azimuth in np.linspace(-math.pi, math.pi, azimuth_count,
                                   endpoint=False):
            direction = (
                cos_elevation * math.cos(float(azimuth)),
                cos_elevation * math.sin(float(azimuth)),
                math.sin(float(elevation)),
            )
            distance = min(
                ray_box_distance(position, direction, obstacle)
                for obstacle in obstacles
            )
            distance = min(maximum_range, distance)
            points.append(tuple(distance * component for component in direction))
    return np.asarray(points, dtype=np.float32)


def pairwise_distances(points: Iterable[Sequence[float]]) -> Iterable[float]:
    values = list(points)
    for index, first in enumerate(values):
        for second in values[index + 1:]:
            yield float(np.linalg.norm(np.asarray(first) - np.asarray(second)))
