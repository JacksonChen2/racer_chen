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

        derivative_vectors = np.zeros((3, segment_count * 6), dtype=np.float64)
        for segment in range(segment_count):
            derivative_vectors[:, segment * 6] = positions_array[segment]
            derivative_vectors[:, segment * 6 + 1] = positions_array[segment + 1]
            if segment == 0:
                derivative_vectors[:, segment * 6 + 2] = np.asarray(start_velocity)
                derivative_vectors[:, segment * 6 + 4] = np.asarray(start_acceleration)
            elif segment == segment_count - 1:
                derivative_vectors[:, segment * 6 + 3] = np.asarray(end_velocity)
                derivative_vectors[:, segment * 6 + 5] = np.asarray(end_acceleration)

        mapping = np.zeros((segment_count * 6, segment_count * 6), dtype=np.float64)
        for segment, duration in enumerate(durations):
            block = np.zeros((6, 6), dtype=np.float64)
            for derivative in range(3):
                block[2 * derivative, derivative] = math.factorial(derivative)
                for power in range(derivative, 6):
                    block[2 * derivative + 1, power] = (
                        math.factorial(power)
                        / math.factorial(power - derivative)
                        * duration ** (power - derivative)
                    )
            begin = segment * 6
            mapping[begin : begin + 6, begin : begin + 6] = block

        fixed_count = 2 * segment_count + 4
        free_count = 2 * segment_count - 2
        selection_t = np.zeros(
            (6 * segment_count, fixed_count + free_count), dtype=np.float64
        )
        selection_t[0, 0] = selection_t[2, 1] = selection_t[4, 2] = 1.0
        selection_t[1, 3] = 1.0
        selection_t[3, 2 * segment_count + 4] = 1.0
        selection_t[5, 2 * segment_count + 5] = 1.0
        last = 6 * (segment_count - 1)
        selection_t[last, 2 * segment_count] = 1.0
        selection_t[last + 1, 2 * segment_count + 1] = 1.0
        selection_t[last + 2, 4 * segment_count] = 1.0
        selection_t[last + 3, 2 * segment_count + 2] = 1.0
        selection_t[last + 4, 4 * segment_count + 1] = 1.0
        selection_t[last + 5, 2 * segment_count + 3] = 1.0
        for waypoint in range(2, segment_count):
            offset = 6 * (waypoint - 1)
            selection_t[offset, 2 + 2 * (waypoint - 1)] = 1.0
            selection_t[offset + 1, 2 + 2 * (waypoint - 1) + 1] = 1.0
            selection_t[offset + 2, 2 * segment_count + 4 + 2 * (waypoint - 2)] = 1.0
            selection_t[offset + 3, 2 * segment_count + 4 + 2 * (waypoint - 1)] = 1.0
            selection_t[offset + 4, 2 * segment_count + 5 + 2 * (waypoint - 2)] = 1.0
            selection_t[offset + 5, 2 * segment_count + 5 + 2 * (waypoint - 1)] = 1.0

        selection = selection_t.T
        transformed = derivative_vectors @ selection.T
        jerk = np.zeros_like(mapping)
        for segment, duration in enumerate(durations):
            for row in range(3, 6):
                for column in range(3, 6):
                    jerk[segment * 6 + row, segment * 6 + column] = (
                        row
                        * (row - 1)
                        * (row - 2)
                        * column
                        * (column - 1)
                        * (column - 2)
                        / (row + column - 5)
                        * duration ** (row + column - 5)
                    )

        inverse_mapping = np.linalg.inv(mapping)
        reduced = selection @ inverse_mapping.T @ jerk @ inverse_mapping @ selection_t
        fixed = transformed[:, :fixed_count]
        if free_count:
            r_fp = reduced[:fixed_count, fixed_count:]
            r_pp = reduced[fixed_count:, fixed_count:]
            free = -(np.linalg.solve(r_pp, r_fp.T) @ fixed.T).T
            transformed[:, fixed_count:] = free
        coefficients = transformed @ selection_t.T @ inverse_mapping.T

        trajectory = cls()
        for segment, duration in enumerate(durations):
            begin = segment * 6
            trajectory.add_segment(
                Polynomial(coefficients[:, begin : begin + 6], float(duration))
            )
        return trajectory
