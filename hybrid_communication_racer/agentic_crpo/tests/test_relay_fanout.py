import numpy as np
import pytest

from agentic_crpo.backend import FileBridgeBackend, MockRacerBackend
from agentic_crpo.gym_env import RacerCRPOEnv
from agentic_crpo.qwen_global_agent import GuidanceManager
from agentic_crpo.relay_fanout import RelayFanoutConstraint
from agentic_crpo.state_builder import StateNormalization
from agentic_crpo.task_loss import TaskLossWeights


def test_projection_keeps_uploads_and_highest_priority_receivers():
    limit = RelayFanoutConstraint(n_uavs=6, max_recipients=4)
    matrix = np.ones((7, 6), dtype=np.int8)
    np.fill_diagonal(matrix[:6], 0)
    priority = np.zeros((6, 6), dtype=np.float32)
    priority[0] = [0.0, 0.9, 0.1, 0.8, 0.7, 0.6]

    proposed = limit.evaluate(matrix)
    projected = limit.project(matrix, priority)
    executed = limit.evaluate(projected)

    assert proposed.max_count == 5
    assert proposed.cost == pytest.approx(1.0)
    assert executed.max_count == 4
    assert projected[0, 2] == 0
    np.testing.assert_array_equal(projected[6], np.ones(6, dtype=np.int8))
    limit.validate_executed(projected)


def test_coverage_reward_and_fanout_cost_are_separate_from_task_loss():
    n_uavs = 6
    env = RacerCRPOEnv(
        MockRacerBackend(n_uavs, horizon=8),
        GuidanceManager(None, n_uavs),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
        reward_mode="coverage",
        constraint_mode="relay_fanout",
        relay_fanout=RelayFanoutConstraint(n_uavs, max_recipients=4),
        enforce_relay_fanout=True,
    )
    env.reset(seed=3)
    proposed_action = np.ones(env.action_space.shape, dtype=np.int8)

    _, reward, _, _, info = env.step(proposed_action)

    assert reward == pytest.approx(info["coverage"])
    assert info["cost"] == pytest.approx(1.0)
    assert info["constraint_value"] == pytest.approx(1.0)
    assert info["relay_fanout_proposed_max"] == 5
    assert info["relay_fanout_executed_max"] == 4
    assert info["relay_fanout_projection_drops"] == n_uavs
    assert np.max(np.sum(info["crpo_action_matrix"][:n_uavs], axis=1)) == 4
    assert info["L_task"] != info["constraint_value"]
    env.close()


def test_invalid_executed_fanout_is_rejected():
    limit = RelayFanoutConstraint(n_uavs=6, max_recipients=4)
    matrix = np.ones((7, 6), dtype=np.int8)
    np.fill_diagonal(matrix[:6], 0)
    with pytest.raises(ValueError, match="exceeds"):
        limit.validate_executed(matrix)


def test_file_bridge_refuses_unprojected_action(tmp_path):
    backend = FileBridgeBackend(
        6,
        action_path=str(tmp_path / "action.txt"),
        telemetry_path=str(tmp_path / "state.json"),
        require_mission_state=False,
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
        max_relay_recipients_per_uav=4,
    )
    matrix = np.ones((7, 6), dtype=np.int8)
    np.fill_diagonal(matrix[:6], 0)

    with pytest.raises(ValueError, match="refused"):
        backend._write_action(matrix)
    assert not (tmp_path / "action.txt").exists()
