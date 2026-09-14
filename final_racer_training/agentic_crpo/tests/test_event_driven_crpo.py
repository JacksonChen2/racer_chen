import numpy as np
import pytest
import torch
from gymnasium import spaces

from agentic_crpo.crpo_ppo import CRPOPPO
from agentic_crpo.event_driven_crpo import (
    EventDrivenCRPORolloutBuffer,
    EventDrivenOneShotLLMPriorCRPOPPO,
)


def test_event_buffer_scales_gae_by_measured_simulated_delta_t():
    buffer = EventDrivenCRPORolloutBuffer(
        2,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=0.81,
        gae_lambda=0.64,
        gamma_cost=0.9,
        gae_lambda_cost=0.5,
        reference_delta_t_s=0.1,
        n_envs=1,
    )
    for index, (reward, cost, start, end) in enumerate(
        (
            (1.0, 0.1, 0.0, 0.2),
            (2.0, 0.2, 0.2, 0.3),
        )
    ):
        buffer.add(
            np.zeros((1, 3), np.float32),
            np.zeros((1, 2), np.int8),
            np.asarray([reward], np.float32),
            np.asarray([cost], np.float32),
            np.asarray([index == 0], dtype=bool),
            torch.zeros(1),
            torch.zeros(1),
            torch.zeros(1),
            sim_time=np.asarray([start]),
            sim_timestamp=np.asarray([end]),
        )
    buffer.compute_returns_and_advantage(
        torch.zeros(1), torch.zeros(1), np.ones(1, dtype=bool)
    )

    assert buffer.delta_times_s[:, 0].tolist() == pytest.approx([0.2, 0.1])
    # dt=0.2 is two baseline periods, so the first trace multiplier is
    # gamma^2 * lambda^2 rather than gamma * lambda.
    assert buffer.advantages[0, 0] == pytest.approx(
        1.0 + (0.81**2) * (0.64**2) * 2.0
    )
    assert buffer.advantages[1, 0] == pytest.approx(2.0)
    assert buffer.cost_advantages[0, 0] == pytest.approx(
        0.1 + (0.9**2) * (0.5**2) * 0.2
    )
    assert buffer.cost_advantages[1, 0] == pytest.approx(0.2)
    np.testing.assert_allclose(
        buffer.returns[:, 0], buffer.advantages[:, 0]
    )
    np.testing.assert_allclose(
        buffer.cost_returns[:, 0], buffer.cost_advantages[:, 0]
    )
    assert float(np.mean(buffer.normalized_advantages)) == pytest.approx(
        0.0, abs=1.0e-6
    )
    assert float(
        np.mean(buffer.normalized_cost_advantages)
    ) == pytest.approx(0.0, abs=1.0e-6)


def test_event_buffer_rejects_missing_or_nonpositive_delta_t():
    buffer = EventDrivenCRPORolloutBuffer(
        1,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=1.0,
        gae_lambda=0.98,
        n_envs=1,
    )
    with pytest.raises(FloatingPointError, match="delta_t"):
        buffer.add(
            np.zeros((1, 3), np.float32),
            np.zeros((1, 2), np.int8),
            np.zeros(1, np.float32),
            np.zeros(1, np.float32),
            np.zeros(1, dtype=bool),
            torch.zeros(1),
            torch.zeros(1),
            torch.zeros(1),
            sim_time=np.asarray([0.2]),
            sim_timestamp=np.asarray([0.2]),
        )


def test_only_event_variant_time_scales_terminal_bootstrap_discount():
    fixed = object.__new__(CRPOPPO)
    assert fixed._bootstrap_discount(
        0.9, {"transition_sim_time_s": 0.2}
    ) == pytest.approx(0.9)

    event = object.__new__(EventDrivenOneShotLLMPriorCRPOPPO)
    event.event_reference_delta_t_s = 0.1
    assert event._bootstrap_discount(
        0.9, {"transition_sim_time_s": 0.2}
    ) == pytest.approx(0.9**2)
