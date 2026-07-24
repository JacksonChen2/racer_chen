"""Shared data structures corresponding to RACER's C++ planning containers."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

import numpy as np
from numpy.typing import NDArray


Vector = NDArray[np.float64]


def vec3(value: object = (0.0, 0.0, 0.0)) -> Vector:
    """Create the exact three-component floating vector used throughout RACER."""
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,):
        raise ValueError(f"expected a three-dimensional vector, got {result.shape}")
    return result.copy()


class PlannerStatus(IntEnum):
    FAIL = 0
    SUCCEED = 1
    NO_FRONTIER = 2
    NO_PATH = 3
    NO_GRID = 4


@dataclass(slots=True)
class VehicleState:
    position: Vector = field(default_factory=vec3)
    velocity: Vector = field(default_factory=vec3)
    acceleration: Vector = field(default_factory=vec3)
    yaw: float = 0.0
    yaw_rate: float = 0.0
    stamp: float = 0.0


@dataclass(slots=True)
class Viewpoint:
    position: Vector
    yaw: float
    visible_count: int = 0
    frontier_id: int = -1

    def __post_init__(self) -> None:
        self.position = vec3(self.position)


@dataclass(slots=True)
class Frontier:
    cells: list[Vector] = field(default_factory=list)
    filtered_cells: list[Vector] = field(default_factory=list)
    viewpoints: list[Viewpoint] = field(default_factory=list)
    average: Vector = field(default_factory=vec3)
    box_min: Vector = field(default_factory=vec3)
    box_max: Vector = field(default_factory=vec3)
    id: int = -1


@dataclass(slots=True)
class PlannerResult:
    status: PlannerStatus
    position_control_points: Optional[NDArray[np.float64]] = None
    yaw_control_points: Optional[NDArray[np.float64]] = None
    knot_span: float = 0.0
    yaw_knot_span: float = 0.0
    position_knots: Optional[NDArray[np.float64]] = None
    duration: float = 0.0
    path: list[Vector] = field(default_factory=list)
    goal: Optional[Vector] = None
    goal_yaw: float = 0.0


@dataclass(slots=True)
class SwarmTrajectory:
    drone_id: int
    trajectory_id: int
    start_time: float
    duration: float
    position_control_points: NDArray[np.float64]
    knots: NDArray[np.float64]
    yaw_control_points: NDArray[np.float64]
    yaw_dt: float
