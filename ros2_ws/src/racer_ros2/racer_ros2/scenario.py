"""Deterministic exploration scene shared by mock and Isaac Sim adapters."""

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class Box2D:
    """Axis-aligned, floor-to-ceiling obstacle."""

    cx: float
    cy: float
    sx: float
    sy: float
    height: float = 3.0

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        return (
            self.cx - self.sx * 0.5,
            self.cx + self.sx * 0.5,
            self.cy - self.sy * 0.5,
            self.cy + self.sy * 0.5,
        )


MAP_MIN = (-10.0, -7.0)
MAP_MAX = (10.0, 7.0)
FLIGHT_Z = 1.2
DRONE_RADIUS = 0.30
STARTS = ((-8.2, -4.8), (-8.2, 4.8), (7.8, 0.0))


def default_obstacles() -> List[Box2D]:
    """Return a connected indoor scene with multiple passages."""

    return [
        # Outer boundary.
        Box2D(0.0, -6.85, 20.0, 0.30),
        Box2D(0.0, 6.85, 20.0, 0.30),
        Box2D(-9.85, 0.0, 0.30, 14.0),
        Box2D(9.85, 0.0, 0.30, 14.0),
        # Interior walls and pillars. Gaps are deliberately wider than the
        # inflated planning radius.
        Box2D(-3.3, -3.8, 0.55, 5.3),
        Box2D(-3.3, 4.2, 0.55, 4.8),
        Box2D(3.0, -4.1, 0.55, 4.6),
        Box2D(3.0, 3.7, 0.55, 5.5),
        Box2D(0.0, 0.0, 2.0, 1.2),
        Box2D(6.4, -3.5, 1.0, 1.0),
        Box2D(6.2, 3.6, 1.0, 1.0),
        Box2D(-6.4, 0.0, 1.1, 1.1),
    ]


def point_box_clearance(x: float, y: float, box: Box2D) -> float:
    """Unsigned distance to a box, or a negative penetration depth."""

    xmin, xmax, ymin, ymax = box.bounds
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return -min(x - xmin, xmax - x, y - ymin, ymax - y)


def obstacle_clearance(
    x: float, y: float, obstacles: Sequence[Box2D]
) -> float:
    return min(point_box_clearance(x, y, box) for box in obstacles)


def ray_box_distance(
    ox: float, oy: float, dx: float, dy: float, box: Box2D
) -> float:
    """Return first positive ray/AABB intersection, or infinity."""

    xmin, xmax, ymin, ymax = box.bounds
    tmin = -math.inf
    tmax = math.inf
    for origin, direction, lower, upper in (
        (ox, dx, xmin, xmax),
        (oy, dy, ymin, ymax),
    ):
        if abs(direction) < 1.0e-12:
            if origin < lower or origin > upper:
                return math.inf
            continue
        t1 = (lower - origin) / direction
        t2 = (upper - origin) / direction
        tmin = max(tmin, min(t1, t2))
        tmax = min(tmax, max(t1, t2))
        if tmin > tmax:
            return math.inf
    if tmax < 0.0:
        return math.inf
    return max(0.0, tmin)


def simulate_scan(
    position: Tuple[float, float],
    yaw: float,
    obstacles: Sequence[Box2D],
    ray_count: int = 180,
    max_range: float = 6.0,
    fov: float = 2.0 * math.pi,
) -> List[float]:
    """Ray-cast a planar lidar against scene geometry."""

    ox, oy = position
    start = -0.5 * fov
    step = fov / max(1, ray_count - 1)
    ranges: List[float] = []
    for index in range(ray_count):
        angle = yaw + start + index * step
        dx, dy = math.cos(angle), math.sin(angle)
        distance = min(
            ray_box_distance(ox, oy, dx, dy, obstacle)
            for obstacle in obstacles
        )
        ranges.append(min(max_range, distance))
    return ranges


def pairwise_distances(
    positions: Iterable[Tuple[float, float]]
) -> Iterable[float]:
    points = list(positions)
    for i, first in enumerate(points):
        for second in points[i + 1 :]:
            yield math.hypot(first[0] - second[0], first[1] - second[1])
