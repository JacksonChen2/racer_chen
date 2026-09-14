"""SB3 RolloutBuffer extension with an independent cost GAE."""

from __future__ import annotations

from typing import Generator, NamedTuple

import numpy as np
import torch as th
from stable_baselines3.common.buffers import RolloutBuffer


class CRPORolloutBufferSamples(NamedTuple):
    observations: th.Tensor
    actions: th.Tensor
    old_values: th.Tensor
    old_cost_values: th.Tensor
    old_log_prob: th.Tensor
    advantages: th.Tensor
    cost_advantages: th.Tensor
    returns: th.Tensor
    cost_returns: th.Tensor


class CRPORolloutBuffer(RolloutBuffer):
    def __init__(
        self,
        *args,
        gamma_cost: float = 1.0,
        gae_lambda_cost: float = 0.95,
        **kwargs,
    ) -> None:
        self.gamma_cost = gamma_cost
        self.gae_lambda_cost = gae_lambda_cost
        super().__init__(*args, **kwargs)

    def reset(self) -> None:
        super().reset()
        shape = (self.buffer_size, self.n_envs)
        self.costs = np.zeros(shape, dtype=np.float32)
        self.cost_values = np.zeros(shape, dtype=np.float32)
        self.cost_advantages = np.zeros(shape, dtype=np.float32)
        self.cost_returns = np.zeros(shape, dtype=np.float32)

    def add(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        reward: np.ndarray,
        cost: np.ndarray,
        episode_start: np.ndarray,
        value: th.Tensor,
        cost_value: th.Tensor,
        log_prob: th.Tensor,
    ) -> None:
        position = self.pos
        super().add(obs, action, reward, episode_start, value, log_prob)
        self.costs[position] = np.asarray(cost)
        self.cost_values[position] = cost_value.detach().cpu().numpy().flatten()

    @property
    def valid_size(self) -> int:
        """Number of collected transitions, including a partial final rollout."""

        return self.buffer_size if self.full else self.pos

    def compute_returns_and_advantage(
        self,
        last_values: th.Tensor,
        last_cost_values: th.Tensor,
        dones: np.ndarray,
    ) -> None:
        valid_size = self.valid_size
        if valid_size < 1:
            raise ValueError("cannot compute returns for an empty rollout")
        next_reward_values = last_values.detach().cpu().numpy().flatten()
        next_cost_values = last_cost_values.detach().cpu().numpy().flatten()
        last_reward_gae = 0.0
        last_cost_gae = 0.0
        for step in reversed(range(valid_size)):
            if step == valid_size - 1:
                next_non_terminal = 1.0 - dones.astype(np.float32)
                following_reward = next_reward_values
                following_cost = next_cost_values
            else:
                next_non_terminal = 1.0 - self.episode_starts[step + 1]
                following_reward = self.values[step + 1]
                following_cost = self.cost_values[step + 1]
            reward_delta = (
                self.rewards[step]
                + self.gamma * following_reward * next_non_terminal
                - self.values[step]
            )
            last_reward_gae = (
                reward_delta
                + self.gamma
                * self.gae_lambda
                * next_non_terminal
                * last_reward_gae
            )
            self.advantages[step] = last_reward_gae
            cost_delta = (
                self.costs[step]
                + self.gamma_cost * following_cost * next_non_terminal
                - self.cost_values[step]
            )
            last_cost_gae = (
                cost_delta
                + self.gamma_cost
                * self.gae_lambda_cost
                * next_non_terminal
                * last_cost_gae
            )
            self.cost_advantages[step] = last_cost_gae
        self.returns[:valid_size] = (
            self.advantages[:valid_size] + self.values[:valid_size]
        )
        self.cost_returns[:valid_size] = (
            self.cost_advantages[:valid_size] + self.cost_values[:valid_size]
        )

    def get(
        self, batch_size: int | None = None
    ) -> Generator[CRPORolloutBufferSamples, None, None]:
        valid_size = self.valid_size
        if valid_size < 1:
            raise ValueError("cannot sample an empty rollout")
        sample_count = valid_size * self.n_envs
        indices = np.random.permutation(sample_count)
        if not self.generator_ready:
            for name in (
                "observations",
                "actions",
                "values",
                "cost_values",
                "log_probs",
                "advantages",
                "cost_advantages",
                "returns",
                "cost_returns",
            ):
                self.__dict__[name] = self.swap_and_flatten(
                    self.__dict__[name][:valid_size]
                )
            self.generator_ready = True
        batch_size = batch_size or sample_count
        for start in range(0, sample_count, batch_size):
            batch = indices[start : start + batch_size]
            data = (
                self.observations[batch],
                self.actions[batch].astype(np.float32, copy=False),
                self.values[batch].flatten(),
                self.cost_values[batch].flatten(),
                self.log_probs[batch].flatten(),
                self.advantages[batch].flatten(),
                self.cost_advantages[batch].flatten(),
                self.returns[batch].flatten(),
                self.cost_returns[batch].flatten(),
            )
            yield CRPORolloutBufferSamples(*tuple(map(self.to_torch, data)))
