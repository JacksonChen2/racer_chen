import numpy as np

from agentic_crpo.backend import MockRacerBackend
from agentic_crpo.episode_tracker import EpisodeCostTracker
from agentic_crpo.gym_env import RacerCRPOEnv
from agentic_crpo.qwen_global_agent import GuidanceManager
from agentic_crpo.state_builder import StateNormalization
from agentic_crpo.task_loss import TaskLossTracker, TaskLossWeights
from agentic_crpo.schemas import TaskMetrics


def make_env(direct_resource=1234.0, reward_mode="negative_bs_resource"):
    return RacerCRPOEnv(
        MockRacerBackend(3, horizon=8, direct_u2u_resource=direct_resource),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=66.0,
        reward_mode=reward_mode,
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
    assert np.isclose(
        info["constraint_cost"],
        info["tilde_L_task"] * info["delta_t_s"] / 300.0,
    )
    assert info["cost"] == info["constraint_cost"]
    assert info["constraint_return_cumulative"] == info["constraint_cost"]


def test_total_prb_reward_includes_bs_and_direct_u2u_resources():
    env = make_env(reward_mode="negative_total_prb")
    env.reset(seed=3)
    action = np.zeros(env.action_space.shape, dtype=np.int8)
    _, reward, _, _, info = env.step(action)
    assert np.isclose(
        reward,
        -(info["C_BS_ul"] + info["C_BS_dl"] + info["C_U2U"]) / 66.0,
    )


def test_constraint_cost_sum_equals_time_weighted_mean_at_300_seconds():
    tracker = TaskLossTracker(TaskLossWeights(1.0, 0.0, 0.0, 0.0))
    initial = TaskMetrics(d_traj=1.0, task_time_s=0.0)
    tracker.reset(initial)
    sequence = [
        TaskMetrics(d_traj=0.5, task_time_s=100.0),
        TaskMetrics(d_traj=0.25, task_time_s=300.0),
    ]
    costs = [tracker.update(metrics)[1] for metrics in sequence]
    # Trapezoids: (1 + .5) / 2 * 100 + (.5 + .25) / 2 * 200 = 150.
    assert tracker.integrated_loss == 150.0
    assert tracker.elapsed_time_s == 300.0
    assert tracker.constraint_value == 0.5
    assert abs(sum(costs) - tracker.constraint_value) < 1.0e-12
    assert tracker.sum_cost == tracker.constraint_value
    assert abs(tracker.telescoping_error) < 1.0e-12


def test_time_weighted_mean_uses_unequal_delta_t():
    tracker = TaskLossTracker(TaskLossWeights(1.0, 0.0, 0.0, 0.0))
    tracker.reset(TaskMetrics(d_traj=0.2, task_time_s=10.0))
    first_cost = tracker.update(
        TaskMetrics(d_traj=0.6, task_time_s=11.0)
    )[1]
    second_cost = tracker.update(
        TaskMetrics(d_traj=0.3, task_time_s=14.0)
    )[1]
    # Areas: (0.2 + 0.6) / 2 * 1 + (0.6 + 0.3) / 2 * 3 = 1.75.
    assert tracker.interval_previous_loss == 0.6
    assert tracker.interval_delta_time_s == 3.0
    assert np.isclose(tracker.interval_mean_loss, 0.45)
    assert np.isclose(tracker.integrated_loss, 1.75)
    assert tracker.elapsed_time_s == 4.0
    assert np.isclose(tracker.constraint_value, 0.4375)
    assert np.isclose(
        first_cost + second_cost,
        tracker.integrated_loss / 300.0,
    )


def test_time_weighted_task_cost_rejects_a_backward_clock():
    tracker = TaskLossTracker(TaskLossWeights())
    tracker.reset(TaskMetrics(task_time_s=1.0))
    with np.testing.assert_raises_regex(ValueError, "must be monotonic"):
        tracker.update(TaskMetrics(task_time_s=0.5))


def test_episode_constraint_estimate_uses_current_time_weighted_loss():
    tracker = EpisodeCostTracker(n_envs=1)
    tracker.observe(
        np.asarray([0.7 / 3.0]),
        np.asarray([False]),
        [
            {
                "L_task_initial": 0.8,
                "L_task": 0.6,
                "L_task_time_weighted": 0.7,
                "constraint_return_cumulative": 0.7 / 3.0,
                "constraint_elapsed_time_s": 100.0,
                "constraint_episode_duration_s": 300.0,
            }
        ],
    )
    assert tracker.estimate() == (0.7, "partial_time_weighted_mean")

    tracker.observe(
        np.asarray([0.5 - 0.7 / 3.0]),
        np.asarray([True]),
        [
            {
                "L_task_initial": 0.8,
                "L_task": 0.3,
                "L_task_time_weighted": 0.5,
                "constraint_return_cumulative": 0.5,
                "constraint_elapsed_time_s": 300.0,
                "constraint_episode_duration_s": 300.0,
            }
        ],
    )
    estimate, source = tracker.estimate()
    assert estimate == 0.5
    assert source == "time_weighted_mean"
    record = tracker.records[-1]
    assert record.final_task_loss == 0.3
    assert record.time_weighted_task_loss == 0.5
    assert record.sum_cost == 0.5
    assert record.constraint_elapsed_time_s == 300.0
    assert record.constraint_episode_duration_s == 300.0
    assert record.telescoping_error == 0.0

    tracker.observe(
        np.asarray([0.1]),
        np.asarray([False]),
        [
            {
                "L_task_initial": 0.9,
                "L_task_time_weighted": 0.8,
                "constraint_elapsed_time_s": 1.0,
            }
        ],
    )
    tracker.reset_active()
    assert tracker.lengths.tolist() == [0]
    assert tracker.constraint_values.tolist() == [0.0]
    assert len(tracker.records) == 1
