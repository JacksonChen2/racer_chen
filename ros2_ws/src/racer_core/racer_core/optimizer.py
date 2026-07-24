"""B-spline optimizer retaining RACER's smoothness, clearance and feasibility terms."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize

from .environment import EDTEnvironment


Array = NDArray[np.float64]


@dataclass(slots=True)
class OptimizerConfig:
    lambda_smoothness: float = 1.0
    lambda_distance: float = 5.0
    lambda_feasibility: float = 0.1
    safe_distance: float = 0.5
    max_velocity: float = 2.0
    max_acceleration: float = 2.0
    max_iterations: int = 200
    lambda_swarm: float = 10.0
    swarm_clearance: float = 0.7


class BsplineOptimizer:
    def __init__(self, environment: EDTEnvironment, config: OptimizerConfig | None = None) -> None:
        self.environment = environment
        self.config = config or OptimizerConfig()

    def _cost_gradient(
        self, flattened: Array, original: Array, knot_span: float, fixed_start: int, fixed_end: int
    ) -> tuple[float, Array]:
        points = original.copy()
        points[fixed_start : len(points) - fixed_end] = flattened.reshape((-1, 3))
        gradient = np.zeros_like(points)
        cost, cfg = 0.0, self.config
        for i in range(len(points) - 3):
            jerk = points[i + 3] - 3.0 * points[i + 2] + 3.0 * points[i + 1] - points[i]
            cost += cfg.lambda_smoothness * float(np.dot(jerk, jerk))
            grad = 2.0 * cfg.lambda_smoothness * jerk
            gradient[i] -= grad
            gradient[i + 1] += 3.0 * grad
            gradient[i + 2] -= 3.0 * grad
            gradient[i + 3] += grad
        for i in range(1, len(points) - 1):
            sample_time = max(0.0, (i - fixed_start + 1) * knot_span)
            distance, distance_gradient = (
                self.environment.voxel_map.get_distance_with_gradient(points[i])
            )
            if distance < cfg.safe_distance:
                error = cfg.safe_distance - distance
                cost += cfg.lambda_distance * error**2
                gradient[i] -= 2.0 * cfg.lambda_distance * error * distance_gradient
            if self.environment.predicted_boxes:
                swarm_distance, swarm_gradient = min(
                    (
                        self.environment.distance_to_box_with_gradient(
                            box, points[i], sample_time
                        )
                        for box in self.environment.predicted_boxes
                    ),
                    key=lambda item: item[0],
                )
                if swarm_distance < cfg.swarm_clearance:
                    error = cfg.swarm_clearance - swarm_distance
                    cost += cfg.lambda_swarm * error**2
                    gradient[i] -= (
                        2.0 * cfg.lambda_swarm * error * swarm_gradient
                    )
        inv_dt, inv_dt2 = 1.0 / knot_span, 1.0 / knot_span**2
        for i in range(len(points) - 1):
            velocity = 3.0 * (points[i + 1] - points[i]) * inv_dt
            excess = np.maximum(np.abs(velocity) - cfg.max_velocity, 0.0)
            signed = np.sign(velocity) * excess
            cost += cfg.lambda_feasibility * float(np.dot(excess, excess))
            grad = 6.0 * cfg.lambda_feasibility * signed * inv_dt
            gradient[i] -= grad
            gradient[i + 1] += grad
        for i in range(len(points) - 2):
            acceleration = 6.0 * (points[i + 2] - 2.0 * points[i + 1] + points[i]) * inv_dt2
            excess = np.maximum(np.abs(acceleration) - cfg.max_acceleration, 0.0)
            signed = np.sign(acceleration) * excess
            cost += cfg.lambda_feasibility * float(np.dot(excess, excess))
            grad = 12.0 * cfg.lambda_feasibility * signed * inv_dt2
            gradient[i] += grad
            gradient[i + 1] -= 2.0 * grad
            gradient[i + 2] += grad
        return float(cost), gradient[fixed_start : len(points) - fixed_end].reshape(-1)

    def optimize(
        self, control_points: Array, knot_span: float, fixed_start: int = 3, fixed_end: int = 3
    ) -> Array:
        points = np.asarray(control_points, dtype=np.float64)
        if len(points) <= fixed_start + fixed_end:
            return points.copy()
        x0 = points[fixed_start : len(points) - fixed_end].reshape(-1)
        result = minimize(
            lambda value: self._cost_gradient(value, points, knot_span, fixed_start, fixed_end),
            x0, method="L-BFGS-B", jac=True,
            options={"maxiter": self.config.max_iterations, "ftol": 1.0e-6},
        )
        optimized = points.copy()
        optimized[fixed_start : len(points) - fixed_end] = result.x.reshape((-1, 3))
        return optimized
