"""Deterministic three-dimensional Isaac/ROS acceptance scene."""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

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
    coarse_grid_size: Point3 = (5.0, 4.5, 2.0)
    truth_mode: str = "analytic_boxes"
    safety_min: Optional[Point3] = None
    safety_max: Optional[Point3] = None

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


def large_acceptance_scene() -> Scenario3D:
    """20 x 50 x 3 m long-range scene with height-dependent passages."""

    height = 3.0
    obstacles = _room_boundaries(10.0, 25.0, height) + (
        # Full-width barriers require genuinely vertical route choices.
        Box3D(
            (0.0, -15.0, 0.65),
            (20.0, 0.45, 1.30),
            "south_low_overflight_wall",
        ),
        Box3D(
            (0.0, -5.0, 2.20),
            (20.0, 0.45, 1.60),
            "south_high_underflight_wall",
        ),
        # A conventional narrow central gate.
        Box3D((-5.5, 5.0, 1.50), (9.0, 0.45, 3.0), "center_gate_left"),
        Box3D((5.5, 5.0, 1.50), (9.0, 0.45, 3.0), "center_gate_right"),
        # The north barrier offers a high route on the west and a low route
        # on the east, encouraging the fleet to use distinct 3-D passages.
        Box3D((-5.0, 15.0, 0.70), (10.0, 0.45, 1.40), "north_low_half"),
        Box3D((5.0, 15.0, 2.25), (10.0, 0.45, 1.50), "north_high_half"),
        # Distributed full-height and partial-height clutter.
        Box3D((-6.2, -20.0, 1.50), (1.20, 1.20, 3.0), "column_south_west"),
        Box3D((6.0, -10.0, 1.50), (1.10, 1.10, 3.0), "column_south_east"),
        Box3D((-5.8, 0.0, 1.50), (1.20, 1.20, 3.0), "column_center_west"),
        Box3D((5.8, 10.0, 1.50), (1.20, 1.20, 3.0), "column_north_east"),
        Box3D((-6.0, 20.0, 1.50), (1.10, 1.10, 3.0), "column_north_west"),
        Box3D((3.0, -20.0, 0.60), (1.8, 1.6, 1.20), "south_low_block"),
        Box3D((-2.5, -10.0, 2.20), (1.8, 1.6, 1.60), "south_hanging_block"),
        Box3D((3.0, 0.0, 0.75), (1.8, 1.8, 1.50), "center_low_block"),
        Box3D((-3.0, 10.0, 2.15), (1.8, 1.8, 1.70), "north_hanging_block"),
        Box3D((3.5, 21.0, 0.70), (2.0, 1.6, 1.40), "north_low_block"),
    )
    return Scenario3D(
        name="acceptance_20x50x3",
        map_min=(-10.0, -25.0, 0.0),
        map_max=(10.0, 25.0, 3.0),
        starts=(
            (-6.0, -23.0, 0.60),
            (0.0, -23.0, 1.50),
            (6.0, -23.0, 2.40),
        ),
        obstacles=obstacles,
        coarse_grid_size=(5.0, 10.0, 3.0),
    )


def warehouse_simple_scene() -> Scenario3D:
    """Flight volume inside the supplied multi-shelf Isaac warehouse USD."""

    return Scenario3D(
        name="warehouse_simple",
        # The rendered asset extends farther because of exterior floor tiles.
        # These limits select the enclosed warehouse flight volume bounded by
        # its inner collision walls and ceiling.
        # Keep the voxel centres inside the measured inner wall safety faces.
        # The west/east wall faces are approximately -10.37/+9.37 m. With the
        # 0.28 m flight barrier, centres at -10.10/+9.10 m remain valid and
        # must not fall outside the planner map due to normal tracking error.
        map_min=(-10.2, -12.0, 0.0),
        map_max=(9.2, 17.8, 9.0),
        starts=(
            (-6.0, -10.0, 0.60),
            (0.0, -10.0, 1.50),
            (6.0, -10.0, 2.40),
            (-3.0, -10.0, 1.05),
            (3.0, -10.0, 1.95),
        ),
        # Geometry is supplied by the external USD and discovered by lidar.
        obstacles=(),
        coarse_grid_size=(4.85, 7.45, 4.50),
        truth_mode="observed_volume",
        # Collision-face bounds measured from the supplied USD. They form a
        # configured geofence in addition to obstacle returns from lidar.
        safety_min=(-10.37, -12.24, 0.0),
        safety_max=(9.37, 17.96, 9.0),
    )


def warehouse_loaded_scene() -> Scenario3D:
    """Rack-zone flight volume in the cargo-populated Warehouse asset.

    The bounds include every generated cargo collision shape recorded in
    ``warehouse_loaded_cargo_report.json`` while excluding the remote loading
    apron and exterior floor tiles.  Candidate starts were verified with a
    0.25 m PhysX overlap box against the composed USD.
    """

    return Scenario3D(
        name="warehouse_loaded",
        map_min=(-26.8, 7.2, 0.0),
        map_max=(5.8, 26.2, 8.5),
        starts=(
            (-24.0, 7.5, 0.80),
            (-10.5, 7.5, 1.50),
            (3.0, 7.5, 2.20),
        ),
        obstacles=(),
        # Four by four by two top-level regions, matching the decomposition
        # density used by the formally tested warehouse_simple profile.
        coarse_grid_size=(8.15, 4.75, 4.25),
        truth_mode="observed_volume",
        # A software geofence around the selected rack zone complements the
        # collision returns from the external USD lidar.
        safety_min=(-27.05, 6.95, 0.0),
        safety_max=(6.05, 26.45, 8.8),
    )


DEFAULT_SCENARIO = acceptance_scene()
LARGE_SCENARIO = large_acceptance_scene()
WAREHOUSE_SCENARIO = warehouse_simple_scene()
WAREHOUSE_LOADED_SCENARIO = warehouse_loaded_scene()
SCENARIOS: Dict[str, Scenario3D] = {
    DEFAULT_SCENARIO.name: DEFAULT_SCENARIO,
    LARGE_SCENARIO.name: LARGE_SCENARIO,
    WAREHOUSE_SCENARIO.name: WAREHOUSE_SCENARIO,
    WAREHOUSE_LOADED_SCENARIO.name: WAREHOUSE_LOADED_SCENARIO,
}


def get_scenario(name: str) -> Scenario3D:
    try:
        return SCENARIOS[str(name)]
    except KeyError as error:
        raise ValueError(
            f"unknown scenario {name!r}; choose one of {sorted(SCENARIOS)}"
        ) from error


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
    if not obstacles:
        return math.inf
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
    obstacles: Optional[Sequence[Box3D]] = None,
) -> np.ndarray:
    """Ray-cast a dense local-frame 3-D cloud for the ROS-only backend."""

    if obstacles is None:
        obstacles = DEFAULT_SCENARIO.obstacles
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
