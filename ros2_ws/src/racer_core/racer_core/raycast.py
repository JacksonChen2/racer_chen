"""Voxel traversal port of ``plan_env/raycast.cpp``."""

from __future__ import annotations

import math
from collections.abc import Iterator

import numpy as np
from numpy.typing import NDArray


IntVector = NDArray[np.int64]
Vector = NDArray[np.float64]


def signum(value: float) -> int:
    return 1 if value > 0.0 else (-1 if value < 0.0 else 0)


def modulus(value: float, divisor: float) -> float:
    result = math.fmod(value, divisor)
    return result + divisor if result < 0.0 else result


def intbound(position: float, direction: float) -> float:
    if direction < 0.0:
        return intbound(-position, -direction)
    if direction > 0.0:
        return (1.0 - modulus(position, 1.0)) / direction
    return math.inf


class RayCaster:
    """Amanatides-Woo traversal with RACER's voxel-center convention."""

    def __init__(self, resolution: float, origin: Vector) -> None:
        if resolution <= 0.0:
            raise ValueError("resolution must be positive")
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=np.float64).copy()

    def _grid_coordinate(self, point: Vector) -> Vector:
        return (np.asarray(point, dtype=np.float64) - self.origin) / self.resolution

    def indices(self, start: Vector, end: Vector, include_end: bool = True) -> Iterator[IntVector]:
        start_grid = self._grid_coordinate(start)
        end_grid = self._grid_coordinate(end)
        current = np.floor(start_grid).astype(np.int64)
        target = np.floor(end_grid).astype(np.int64)
        # RACER's C++ implementation steps according to the difference
        # between the integer endpoint voxels.  Using the raw floating ray
        # direction can step along an axis whose start and end are already in
        # the same voxel, overshoot the target, and loop forever.
        direction = (target - current).astype(np.float64)
        step = np.sign(direction).astype(np.int64)
        t_max = np.asarray(
            [intbound(start_grid[axis], direction[axis]) for axis in range(3)],
            dtype=np.float64,
        )
        t_delta = np.asarray(
            [abs(1.0 / value) if value != 0.0 else math.inf for value in direction],
            dtype=np.float64,
        )
        yield current.copy()
        maximum_steps = int(np.sum(np.abs(target - current))) + 1
        for _ in range(maximum_steps):
            if np.array_equal(current, target):
                break
            axis = int(np.argmin(t_max))
            current[axis] += step[axis]
            t_max[axis] += t_delta[axis]
            if include_end or not np.array_equal(current, target):
                yield current.copy()
        else:
            raise RuntimeError("ray traversal failed to reach its target voxel")

    def positions(self, start: Vector, end: Vector, include_end: bool = True) -> Iterator[Vector]:
        half = np.full(3, 0.5, dtype=np.float64)
        for index in self.indices(start, end, include_end=include_end):
            yield (index.astype(np.float64) + half) * self.resolution + self.origin


def raycast(
    start: Vector,
    end: Vector,
    origin: Vector,
    resolution: float,
    include_end: bool = True,
) -> list[IntVector]:
    return list(RayCaster(resolution, origin).indices(start, end, include_end))
