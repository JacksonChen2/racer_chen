import numpy as np

from agentic_crpo.backend import MockRacerBackend
from agentic_crpo.gym_env import RacerCRPOEnv
from agentic_crpo.qwen_global_agent import GuidanceManager
from agentic_crpo.state_builder import StateNormalization
from agentic_crpo.task_loss import TaskLossTracker, TaskLossWeights
from agentic_crpo.schemas import TaskMetrics


def make_env(direct_resource=1234.0):
    return RacerCRPOEnv(
        MockRacerBackend(3, horizon=8, direct_u2u_resource=direct_resource),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=66.0,
    )


def test_direct_u2u_resource_never_enters_reward():
    env = make_env()
    env.reset(seed=3)
    action = np.zeros(env.action_space.shape, dtype=np.int8)
    _, reward, _, _, info = env.step(action)
    # One RL transition aggregates exactly five 20 ms communication slots.
    assert info["C_U2U"] == 5.0 * 1234.0
    assert info["C_BS"] == 0.0
    assert reward == 0.0


def test_signed_incremental_cost_telescopes():
    tracker = TaskLossTracker(TaskLossWeights(0.1, 0.2, 0.3, 0.4))
    initial = TaskMetrics(0.2, 0.3, 0.4, 0.5)
    tracker.reset(initial)
    sequence = [
        TaskMetrics(0.3, 0.2, 0.5, 0.4),
        TaskMetrics(0.1, 0.1, 0.2, 0.3),
        TaskMetrics(0.5, 0.4, 0.3, 0.2),
    ]
    costs = [tracker.update(metrics)[1] for metrics in sequence]
    assert any(cost < 0.0 for cost in costs)
    expected = tracker.weights.total(sequence[-1]) - tracker.weights.total(initial)
    assert abs(sum(costs) - expected) < 1.0e-12
    assert abs(tracker.telescoping_error) < 1.0e-12
