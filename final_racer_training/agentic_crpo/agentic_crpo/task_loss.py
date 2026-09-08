"""The configured four-component task loss and telescoping CRPO cost."""

from __future__ import annotations

from dataclasses import dataclass

from .schemas import TaskMetrics


@dataclass(frozen=True)
class TaskLossWeights:
    alpha_traj: float = 0.30
    alpha_cov: float = 0.30
    alpha_red: float = 0.10
    alpha_map: float = 0.30

    def __post_init__(self) -> None:
        if min(
            self.alpha_traj,
            self.alpha_cov,
            self.alpha_red,
            self.alpha_map,
        ) < 0.0:
            raise ValueError("task-loss weights must be non-negative")
        total = self.alpha_traj + self.alpha_cov + self.alpha_red + self.alpha_map
        if abs(total - 1.0) > 1.0e-9:
            raise ValueError("task-loss weights must sum to 1")

    def total(self, metrics: TaskMetrics) -> float:
        return float(
            self.alpha_traj * metrics.d_traj
            + self.alpha_cov * metrics.d_cov
            + self.alpha_red * metrics.d_red
            + self.alpha_map * metrics.d_map
        )


class TaskLossTracker:
    """Return signed increments so costs telescope over the mission."""

    def __init__(self, weights: TaskLossWeights) -> None:
        self.weights = weights
        self.initial = 0.0
        self.previous = 0.0
        self.sum_cost = 0.0

    def reset(self, metrics: TaskMetrics) -> float:
        self.initial = self.weights.total(metrics)
        self.previous = self.initial
        self.sum_cost = 0.0
        return self.initial

    def update(self, metrics: TaskMetrics) -> tuple[float, float]:
        current = self.weights.total(metrics)
        cost = current - self.previous
        self.previous = current
        self.sum_cost += cost
        return current, cost

    @property
    def telescoping_error(self) -> float:
        return float(self.sum_cost - (self.previous - self.initial))
