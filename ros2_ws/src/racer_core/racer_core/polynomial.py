"""Minimum-jerk polynomial trajectory port of ``poly_traj``."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from numpy.typing import NDArray


Array = NDArray[np.float64]


@dataclass(slots=True)
class Polynomial:
    coefficients: Array
    duration: float

    def __post_init__(self) -> None:
        self.coefficients = np.asarray(self.coefficients, dtype=np.float64)
        if self.coefficients.shape != (3, 6):
            raise ValueError("coefficients must have shape (3, 6)")
        if self.duration <= 0.0:
            raise ValueError("polynomial duration must be positive")

    @staticmethod
    def time_basis(time: float, power: int, derivative: int) -> float:
        if power < derivative:
            return 0.0
        coefficient = 1
        for value in range(power, power - derivative, -1):
            coefficient *= value
        return float(coefficient) * float(time) ** (power - derivative)

    def evaluate(self, time: float, derivative: int = 0) -> Array:
        basis = np.zeros(6, dtype=np.float64)
        for power in range(derivative, 6):
            basis[power] = self.time_basis(time, power, derivative)
        return self.coefficients @ basis


class PolynomialTrajectory:
    def __init__(self) -> None:
        self.segments: list[Polynomial] = []
        self.times: list[float] = []
        self.sample_points: list[Array] = []
        self.time_sum = -1.0
        self.length = -1.0

    def reset(self) -> None:
        self.segments.clear()
        self.times.clear()
        self.sample_points.clear()
        self.time_sum = -1.0
        self.length = -1.0

    def add_segment(self, polynomial: Polynomial) -> None:
        self.segments.append(polynomial)
        self.times.append(polynomial.duration)
        self.time_sum = -1.0
        self.length = -1.0

    def evaluate(self, time: float, derivative: int = 0) -> Array:
        if not self.segments:
            raise ValueError("trajectory has no segments")
        remaining = min(max(float(time), 0.0), self.get_total_time())
        index = 0
        while index < len(self.times) - 1 and self.times[index] + 1.0e-4 < remaining:
            remaining -= self.times[index]
            index += 1
        return self.segments[index].evaluate(remaining, derivative)

    def get_total_time(self) -> float:
        self.time_sum = float(sum(self.times))
        return self.time_sum

    def get_sample_points(self, step: float = 0.01) -> list[Array]:
        total = self.get_total_time()
        self.sample_points = [
            self.evaluate(time, 0) for time in np.arange(0.0, total, step, dtype=np.float64)
        ]
        return [point.copy() for point in self.sample_points]

    def get_length(self) -> float:
        if not self.sample_points:
            self.get_sample_points()
        self.length = float(
            sum(
                np.linalg.norm(current - previous)
                for previous, current in zip(self.sample_points, self.sample_points[1:])
            )
        )
        return self.length

    def get_mean_speed(self) -> float:
        return self.get_length() / self.get_total_time()

    def integral_cost(self, derivative: int, step: float = 0.01) -> float:
        total = self.get_total_time()
        return float(
            sum(
                np.dot(value, value) * step
                for value in (
                    self.evaluate(time, derivative)
                    for time in np.arange(0.0, total, step, dtype=np.float64)
                )
            )
        )

    def derivative_statistics(self, derivative: int, step: float = 0.01) -> tuple[float, float]:
        values = [
            float(np.linalg.norm(self.evaluate(time, derivative)))
            for time in np.arange(0.0, self.get_total_time(), step, dtype=np.float64)
        ]
        return float(np.mean(values)), float(np.max(values))

    @classmethod
    def through_waypoints(
        cls,
        positions: Iterable[Iterable[float]] | Array,
        start_velocity: Iterable[float] | Array,
        end_velocity: Iterable[float] | Array,
        start_acceleration: Iterable[float] | Array,
        end_acceleration: Iterable[float] | Array,
        times: Iterable[float] | Array,
    ) -> "PolynomialTrajectory":
        positions_array = np.asarray(positions, dtype=np.float64)
        durations = np.asarray(times, dtype=np.float64)
        segment_count = durations.size
        if positions_array.shape != (segment_count + 1, 3):
            raise ValueError("positions must contain one more row than segment durations")
        if np.any(durations <= 0.0):
            raise ValueError("all segment durations must be positive")

        # The original implementation solves for shared waypoint derivatives
        # and then constructs a minimum-jerk polynomial.  Use shared centered
        # derivatives here as the stable closed-form equivalent; position,
        # velocity and acceleration remain continuous at every waypoint.
        velocities = np.zeros_like(positions_array)
        accelerations = np.zeros_like(positions_array)
        velocities[0] = np.asarray(start_velocity, dtype=np.float64)
        velocities[-1] = np.asarray(end_velocity, dtype=np.float64)
        accelerations[0] = np.asarray(start_acceleration, dtype=np.float64)
        accelerations[-1] = np.asarray(end_acceleration, dtype=np.float64)
        for waypoint in range(1, segment_count):
            before = (
                positions_array[waypoint] - positions_array[waypoint - 1]
            ) / durations[waypoint - 1]
            after = (
                positions_array[waypoint + 1] - positions_array[waypoint]
            ) / durations[waypoint]
            velocities[waypoint] = 0.5 * (before + after)
            accelerations[waypoint] = (
                2.0 * (after - before)
                / max(durations[waypoint - 1] + durations[waypoint], 1.0e-6)
            )

        trajectory = cls()
        for segment, duration in enumerate(durations):
            time = float(duration)
            mapping = np.asarray(
                [
                    [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    [1.0, time, time**2, time**3, time**4, time**5],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 2.0 * time, 3.0 * time**2, 4.0 * time**3, 5.0 * time**4],
                    [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 2.0, 6.0 * time, 12.0 * time**2, 20.0 * time**3],
                ],
                dtype=np.float64,
            )
            boundary = np.vstack(
                (
                    positions_array[segment],
                    positions_array[segment + 1],
                    velocities[segment],
                    velocities[segment + 1],
                    accelerations[segment],
                    accelerations[segment + 1],
                )
            )
            coefficients = np.linalg.solve(mapping, boundary).T
            trajectory.add_segment(Polynomial(coefficients, time))
        return trajectory
