"""Non-uniform B-spline port of ``bspline/non_uniform_bspline.cpp``."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from numpy.typing import NDArray


Array = NDArray[np.float64]


class NonUniformBspline:
    """B-spline with the same knot and control-point conventions as RACER."""

    def __init__(
        self,
        points: Iterable[Iterable[float]] | Array | None = None,
        order: int = 3,
        interval: float = 1.0,
    ) -> None:
        self.control_points = np.empty((0, 0), dtype=np.float64)
        self.degree = int(order)
        self.knot_span = float(interval)
        self.knots = np.empty(0, dtype=np.float64)
        self.n = -1
        self.m = -1
        self.limit_vel = math.inf
        self.limit_acc = math.inf
        self.limit_ratio = 1.1
        self.start_time = 0.0
        self.duration = 0.0
        if points is not None:
            self.set_uniform_bspline(points, order, interval)

    def set_uniform_bspline(
        self, points: Iterable[Iterable[float]] | Array, order: int, interval: float
    ) -> None:
        control = np.asarray(points, dtype=np.float64)
        if control.ndim != 2:
            raise ValueError("control points must be a two-dimensional matrix")
        if control.shape[0] <= order:
            raise ValueError("a B-spline needs more control points than its degree")
        if interval <= 0.0:
            raise ValueError("knot interval must be positive")
        self.control_points = control.copy()
        self.degree = int(order)
        self.knot_span = float(interval)
        self.n = control.shape[0] - 1
        self.m = self.n + self.degree + 1
        self.knots = np.zeros(self.m + 1, dtype=np.float64)
        for index in range(self.m + 1):
            if index <= self.degree:
                self.knots[index] = (-self.degree + index) * self.knot_span
            else:
                self.knots[index] = self.knots[index - 1] + self.knot_span
        self.duration = self.get_time_sum()

    def set_knot(self, knot: Iterable[float] | Array) -> None:
        values = np.asarray(knot, dtype=np.float64)
        if values.shape != (self.m + 1,):
            raise ValueError(f"expected {self.m + 1} knots, got {values.shape}")
        if np.any(np.diff(values) < 0.0):
            raise ValueError("knots must be non-decreasing")
        self.knots = values.copy()
        self.duration = self.get_time_sum()

    def get_knot(self) -> Array:
        return self.knots.copy()

    def get_control_point(self) -> Array:
        return self.control_points.copy()

    def get_time_span(self) -> tuple[float, float]:
        return float(self.knots[self.degree]), float(self.knots[self.m - self.degree])

    def evaluate_de_boor(self, parameter: float) -> Array:
        lower, upper = self.get_time_span()
        u = min(max(float(parameter), lower), upper)
        k = self.degree
        while k + 1 < self.knots.size and self.knots[k + 1] < u:
            k += 1
        points = [
            self.control_points[k - self.degree + index].copy()
            for index in range(self.degree + 1)
        ]
        for recursion in range(1, self.degree + 1):
            for index in range(self.degree, recursion - 1, -1):
                left_index = index + k - self.degree
                denominator = (
                    self.knots[index + 1 + k - recursion] - self.knots[left_index]
                )
                alpha = 0.0 if abs(denominator) < 1.0e-12 else (
                    u - self.knots[left_index]
                ) / denominator
                points[index] = (1.0 - alpha) * points[index - 1] + alpha * points[index]
        return points[self.degree]

    def evaluate(self, time_from_start: float) -> Array:
        return self.evaluate_de_boor(float(time_from_start) + self.knots[self.degree])

    def derivative_control_points(self) -> Array:
        result = np.zeros(
            (self.control_points.shape[0] - 1, self.control_points.shape[1]),
            dtype=np.float64,
        )
        for index in range(result.shape[0]):
            denominator = self.knots[index + self.degree + 1] - self.knots[index + 1]
            result[index] = (
                self.degree
                * (self.control_points[index + 1] - self.control_points[index])
                / denominator
            )
        return result

    def derivative(self) -> "NonUniformBspline":
        derivative = NonUniformBspline(
            self.derivative_control_points(), self.degree - 1, self.knot_span
        )
        derivative.set_knot(self.knots[1:-1])
        return derivative

    def derivatives(self, order: int) -> list["NonUniformBspline"]:
        if order < 1:
            return []
        result = [self.derivative()]
        for _ in range(2, order + 1):
            result.append(result[-1].derivative())
        return result

    def boundary_states(self, start_order: int, end_order: int) -> tuple[list[Array], list[Array]]:
        derivatives = self.derivatives(max(start_order, end_order))
        duration = self.get_time_sum()
        start = [self.evaluate(0.0)]
        end = [self.evaluate(duration)]
        start.extend(item.evaluate(0.0) for item in derivatives[:start_order])
        end.extend(item.evaluate(duration) for item in derivatives[:end_order])
        return start, end

    @staticmethod
    def parameterize_to_bspline(
        time_step: float,
        points: Iterable[Iterable[float]] | Array,
        start_end_derivative: Iterable[Iterable[float]] | Array,
        degree: int = 3,
    ) -> Array:
        point_matrix = np.asarray(points, dtype=np.float64)
        derivatives = np.asarray(start_end_derivative, dtype=np.float64)
        if time_step <= 0.0:
            raise ValueError("time step must be positive")
        if point_matrix.ndim != 2 or point_matrix.shape[0] < 2 or point_matrix.shape[1] != 3:
            raise ValueError("point set must have shape (K, 3), K >= 2")
        if derivatives.shape != (4, 3):
            raise ValueError("boundary derivatives must have shape (4, 3)")
        if degree not in (3, 4, 5):
            raise ValueError("the original RACER implementation supports degree 3, 4, or 5")

        count = point_matrix.shape[0]
        matrix = np.zeros((count + 4, count + degree - 1), dtype=np.float64)
        if degree == 3:
            pos_map = np.asarray((1.0, 4.0, 1.0)) / 6.0
            vel_map = np.asarray((-1.0, 0.0, 1.0)) / (2.0 * time_step)
            acc_map = np.asarray((1.0, -2.0, 1.0)) / (time_step**2)
        elif degree == 4:
            pos_map = np.asarray((1.0, 11.0, 11.0, 1.0)) / 24.0
            vel_map = np.asarray((-1.0, -3.0, 3.0, 1.0)) / (6.0 * time_step)
            acc_map = np.asarray((1.0, -1.0, -1.0, 1.0)) / (2.0 * time_step**2)
        else:
            pos_map = np.asarray((1.0, 26.0, 66.0, 26.0, 1.0)) / 120.0
            vel_map = np.asarray((-1.0, -10.0, 0.0, 10.0, 1.0)) / (24.0 * time_step)
            acc_map = np.asarray((1.0, 2.0, -6.0, 2.0, 1.0)) / (6.0 * time_step**2)
        width = degree
        for index in range(count):
            matrix[index, index : index + width] = pos_map
        matrix[count, :width] = vel_map
        matrix[count + 1, count - 1 : count - 1 + width] = vel_map
        matrix[count + 2, :width] = acc_map
        matrix[count + 3, count - 1 : count - 1 + width] = acc_map
        target = np.vstack((point_matrix, derivatives))
        control, *_ = np.linalg.lstsq(matrix, target, rcond=None)
        return control

    def set_physical_limits(self, velocity: float, acceleration: float) -> None:
        self.limit_vel = float(velocity)
        self.limit_acc = float(acceleration)
        self.limit_ratio = 1.1

    def check_ratio(self) -> float:
        velocity = self.derivative_control_points()
        acceleration = self.derivative().derivative_control_points()
        max_velocity = float(np.max(np.abs(velocity)))
        max_acceleration = float(np.max(np.abs(acceleration)))
        return max(
            max_velocity / self.limit_vel,
            math.sqrt(abs(max_acceleration) / self.limit_acc),
        )

    def lengthen_time(self, ratio: float) -> None:
        first = 2 * self.degree - 1
        second = (self.knots.size - 1) - 2 * self.degree + 1
        if first >= second:
            return
        delta = (float(ratio) - 1.0) * (self.knots[second] - self.knots[first])
        increment = delta / (second - first)
        for index in range(first + 1, second + 1):
            self.knots[index] += (index - first) * increment
        self.knots[second + 1 :] += delta
        self.duration = self.get_time_sum()

    def check_feasibility(self) -> bool:
        velocity = self.derivative_control_points()
        acceleration = self.derivative().derivative_control_points()
        return bool(
            np.all(np.abs(velocity) <= self.limit_vel + 1.0e-4)
            and np.all(np.abs(acceleration) <= self.limit_acc + 1.0e-4)
        )

    def reallocate_time(self) -> bool:
        feasible = True
        points = self.control_points
        for index in range(points.shape[0] - 1):
            velocity = self.degree * (points[index + 1] - points[index]) / (
                self.knots[index + self.degree + 1] - self.knots[index + 1]
            )
            if np.any(np.abs(velocity) > self.limit_vel + 1.0e-4):
                feasible = False
                ratio = min(float(np.max(np.abs(velocity))) / self.limit_vel + 1.0e-4, self.limit_ratio)
                original = self.knots[index + self.degree + 1] - self.knots[index + 1]
                delta = (ratio - 1.0) * original
                increment = delta / self.degree
                for knot_index in range(index + 2, index + self.degree + 2):
                    self.knots[knot_index] += (knot_index - index - 1) * increment
                self.knots[index + self.degree + 2 :] += delta
        for index in range(points.shape[0] - 2):
            acceleration = self.degree * (self.degree - 1) * (
                (points[index + 2] - points[index + 1])
                / (self.knots[index + self.degree + 2] - self.knots[index + 2])
                - (points[index + 1] - points[index])
                / (self.knots[index + self.degree + 1] - self.knots[index + 1])
            ) / (self.knots[index + self.degree + 1] - self.knots[index + 2])
            if np.any(np.abs(acceleration) > self.limit_acc + 1.0e-4):
                feasible = False
                ratio = min(
                    math.sqrt(float(np.max(np.abs(acceleration))) / self.limit_acc) + 1.0e-4,
                    self.limit_ratio,
                )
                original = self.knots[index + self.degree + 1] - self.knots[index + 2]
                delta = (ratio - 1.0) * original
                increment = delta / (self.degree - 1)
                if index in (1, 2):
                    for knot_index in range(2, 6):
                        self.knots[knot_index] += (knot_index - 1) * increment
                    self.knots[6:] += 4.0 * increment
                else:
                    for knot_index in range(index + 3, index + self.degree + 2):
                        self.knots[knot_index] += (knot_index - index - 2) * increment
                    self.knots[index + self.degree + 2 :] += delta
        self.duration = self.get_time_sum()
        return feasible

    def get_time_sum(self) -> float:
        return float(self.knots[self.m - self.degree] - self.knots[self.degree])

    def get_length(self, resolution: float = 0.01) -> float:
        duration = self.get_time_sum()
        previous = self.evaluate(0.0)
        length = 0.0
        time = resolution
        while time <= duration + 1.0e-4:
            current = self.evaluate(min(time, duration))
            length += float(np.linalg.norm(current - previous))
            previous = current
            time += resolution
        return length

    def get_jerk(self) -> float:
        jerk = self.derivative().derivative().derivative()
        intervals = np.diff(jerk.knots[: jerk.control_points.shape[0] + 1])
        return float(np.sum(intervals[:, None] * jerk.control_points**2))

    def derivative_statistics(self, derivative_order: int, step: float = 0.01) -> tuple[float, float]:
        trajectory = self
        for _ in range(derivative_order):
            trajectory = trajectory.derivative()
        lower, upper = trajectory.get_time_span()
        values: list[float] = []
        parameter = lower
        while parameter <= upper + 1.0e-9:
            values.append(float(np.linalg.norm(trajectory.evaluate_de_boor(parameter))))
            parameter += step
        return float(np.mean(values)), float(np.max(values))
