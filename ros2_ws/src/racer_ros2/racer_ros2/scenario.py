"""Deterministic exploration scenes shared by the test backends.

The ``small`` scene is intentionally cheap enough for rapid integration tests.
``large`` is 20 m x 10 m x 2 m, while ``long`` is the 20 m x 50 m x 3 m
endurance scene. Obstacles are floor-to-ceiling because the mapper is planar.
"""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Sequence, Tuple


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


@dataclass(frozen=True)
class Scenario:
    """A bounded fixed-altitude exploration volume."""

    name: str
    map_min: Tuple[float, float]
    map_max: Tuple[float, float]
    height: float
    flight_z: float
    starts: Tuple[Tuple[float, float], ...]
    obstacles: Tuple[Box2D, ...]

    @property
    def map_size(self) -> Tuple[float, float]:
        return (
            self.map_max[0] - self.map_min[0],
            self.map_max[1] - self.map_min[1],
        )


# The collision proxy is a 0.16 m square Crazyflie-like body. A conservative
# circular radius is used by the planner and acceptance metrics.
DRONE_RADIUS = 0.12


def _boundary(
    half_x: float, half_y: float, height: float, thickness: float = 0.25
) -> Tuple[Box2D, ...]:
    return (
        Box2D(0.0, -half_y + thickness * 0.5, 2.0 * half_x, thickness, height),
        Box2D(0.0, half_y - thickness * 0.5, 2.0 * half_x, thickness, height),
        Box2D(-half_x + thickness * 0.5, 0.0, thickness, 2.0 * half_y, height),
        Box2D(half_x - thickness * 0.5, 0.0, thickness, 2.0 * half_y, height),
    )


def _small_scene() -> Scenario:
    height = 2.0
    obstacles = _boundary(4.0, 3.0, height) + (
        Box2D(-0.8, -1.55, 0.35, 2.25, height),
        Box2D(-0.8, 1.75, 0.35, 1.75, height),
        Box2D(1.7, 0.0, 0.75, 0.75, height),
    )
    return Scenario(
        name="small",
        map_min=(-4.0, -3.0),
        map_max=(4.0, 3.0),
        height=height,
        flight_z=1.0,
        starts=((-3.1, -2.1), (-3.1, 2.1), (2.9, 0.0)),
        obstacles=obstacles,
    )


def _large_scene() -> Scenario:
    """20 m x 10 m x 2 m indoor scene with connected passages."""

    height = 2.0
    obstacles = _boundary(10.0, 5.0, height) + (
        # Two offset partitions create three connected work regions.
        Box2D(-3.3, -3.15, 0.40, 3.45, height),
        Box2D(-3.3, 2.55, 0.40, 4.65, height),
        Box2D(3.0, -2.80, 0.40, 4.15, height),
        Box2D(3.0, 3.05, 0.40, 3.65, height),
        # Central and side clutter.
        Box2D(0.0, 0.0, 1.60, 0.85, height),
        Box2D(6.7, -2.9, 0.80, 0.80, height),
        Box2D(6.3, 2.8, 0.80, 0.80, height),
        Box2D(-6.5, 0.0, 0.90, 0.90, height),
    )
    return Scenario(
        name="large",
        map_min=(-10.0, -5.0),
        map_max=(10.0, 5.0),
        height=height,
        flight_z=1.0,
        starts=((-8.4, -3.6), (-8.4, 3.6), (7.8, 0.0)),
        obstacles=obstacles,
    )


def _long_scene() -> Scenario:
    """Open 20 m x 50 m x 3 m arena with staggered obstacles."""

    height = 3.0
    obstacles = _boundary(10.0, 25.0, height) + (
        # Four offset barriers require detours but leave broad routes around
        # both ends, keeping the requested obstacle field fully connected.
        Box2D(-3.0, -15.0, 8.00, 0.50, height),
        Box2D(3.5, -5.0, 8.00, 0.50, height),
        Box2D(-3.5, 5.0, 8.00, 0.50, height),
        Box2D(3.0, 15.0, 8.00, 0.50, height),
        # Compact clutter distributed over the entire venue.
        Box2D(-6.0, -20.0, 1.00, 1.00, height),
        Box2D(5.5, -10.0, 1.20, 1.20, height),
        Box2D(-5.2, 0.0, 1.10, 1.10, height),
        Box2D(5.8, 10.0, 1.20, 1.20, height),
        Box2D(-5.8, 20.0, 1.00, 1.00, height),
    )
    return Scenario(
        name="long",
        map_min=(-10.0, -25.0),
        map_max=(10.0, 25.0),
        height=height,
        flight_z=1.5,
        # Three separated safe launch pads let the distributed allocation
        # begin in the south, centre, and north thirds of the long venue.
        starts=((-6.5, -22.0), (0.0, 0.0), (5.0, 22.0)),
        obstacles=obstacles,
    )


SCENARIOS: Dict[str, Scenario] = {
    item.name: item
    for item in (_small_scene(), _large_scene(), _long_scene())
}


def get_scenario(name: str = "large") -> Scenario:
    try:
        return SCENARIOS[name]
    except KeyError as error:
        choices = ", ".join(sorted(SCENARIOS))
        raise ValueError(f"unknown scenario {name!r}; choose {choices}") from error


DEFAULT_SCENARIO = get_scenario("large")
MAP_MIN = DEFAULT_SCENARIO.map_min
MAP_MAX = DEFAULT_SCENARIO.map_max
FLIGHT_Z = DEFAULT_SCENARIO.flight_z
STARTS = DEFAULT_SCENARIO.starts


def default_obstacles(name: str = "large") -> List[Box2D]:
    """Return a mutable copy of one scene's collision geometry."""

    return list(get_scenario(name).obstacles)


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
        for second in points[i + 1:]:
            yield math.hypot(first[0] - second[0], first[1] - second[1])
