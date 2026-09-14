"""Mission-level constraint estimates that survive rollout boundaries."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import warnings

import numpy as np


@dataclass(frozen=True)
class EpisodeRecord:
    final_task_loss: float
    initial_task_loss: float
    sum_cost: float
    total_bs_resource: float
    episode_length: int
    telescoping_error: float


class EpisodeCostTracker:
    def __init__(
        self,
        n_envs: int,
        window: int = 20,
        telescoping_tolerance: float = 1.0e-6,
        check_telescoping: bool = True,
    ) -> None:
        if n_envs < 1 or window < 1:
            raise ValueError("invalid episode tracker dimensions")
        self.n_envs = n_envs
        self.telescoping_tolerance = float(telescoping_tolerance)
        self.check_telescoping = bool(check_telescoping)
        if self.telescoping_tolerance < 0.0:
            raise ValueError("telescoping tolerance must be non-negative")
        self.records: deque[EpisodeRecord] = deque(maxlen=window)
        self.costs = np.zeros(n_envs, np.float64)
        self.resources = np.zeros(n_envs, np.float64)
        self.lengths = np.zeros(n_envs, np.int64)
        self.initial_losses = np.zeros(n_envs, np.float64)

    def observe(
        self,
        costs: np.ndarray,
        dones: np.ndarray,
        infos: list[dict],
    ) -> None:
        for index in range(self.n_envs):
            info = infos[index]
            if self.lengths[index] == 0:
                self.initial_losses[index] = float(info.get("L_task_initial", 0.0))
            self.costs[index] += float(costs[index])
            self.resources[index] += float(info.get("C_BS", 0.0))
            self.lengths[index] += 1
            if not dones[index]:
                continue
            final_loss = float(info.get("L_task", self.initial_losses[index] + self.costs[index]))
            error = (
                self.costs[index]
                - (final_loss - self.initial_losses[index])
                if self.check_telescoping
                else 0.0
            )
            if self.check_telescoping and abs(error) > self.telescoping_tolerance:
                warnings.warn(
                    "CRPO task cost failed the telescoping check: "
                    f"error={error:.3e}, tolerance={self.telescoping_tolerance:.3e}",
                    RuntimeWarning,
                    stacklevel=2,
                )
            self.records.append(
                EpisodeRecord(
                    final_task_loss=final_loss,
                    initial_task_loss=float(self.initial_losses[index]),
                    sum_cost=float(self.costs[index]),
                    total_bs_resource=float(self.resources[index]),
                    episode_length=int(self.lengths[index]),
                    telescoping_error=float(error),
                )
            )
            self.costs[index] = 0.0
            self.resources[index] = 0.0
            self.lengths[index] = 0
            self.initial_losses[index] = 0.0

    def estimate(
        self, cost_value_estimate: np.ndarray | None = None
    ) -> tuple[float, str]:
        if self.records:
            return (
                float(np.mean([item.final_task_loss for item in self.records])),
                "episode_return",
            )
        if cost_value_estimate is None:
            return float(np.mean(self.initial_losses + self.costs)), "partial_episode"
        estimate = self.initial_losses + self.costs + np.asarray(cost_value_estimate)
        return float(np.mean(estimate)), "critic_estimate"
