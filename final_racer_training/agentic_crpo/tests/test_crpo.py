from dataclasses import replace
import json
import threading

import numpy as np
import pytest
import torch
from gymnasium import spaces
from stable_baselines3.common.save_util import (
    load_from_zip_file,
    save_to_zip_file,
)

from agentic_crpo.backend import MissionEndedError, MockRacerBackend
from agentic_crpo.crpo_ppo import (
    CONSTRAINT_COST_VERSION,
    CRPOPPO,
    _bernoulli_log_prob_from_masked_logits,
    select_crpo_mode,
)
from agentic_crpo.crpo_buffer import CRPORolloutBuffer
from agentic_crpo.crpo_policy import (
    DUAL_ENCODER_ARCHITECTURE,
    N10_GUIDANCE_DIM,
    N10_OBSERVATION_DIM,
    N10_PHYSICAL_STATE_DIM,
    TASK_PHASE_DUAL_CRITIC_ARCHITECTURE,
)
from agentic_crpo.gym_env import RacerCRPOEnv
from agentic_crpo.qwen_global_agent import GuidanceManager
from agentic_crpo.state_builder import StateNormalization
from agentic_crpo.task_loss import TaskLossWeights


def make_env(n_uavs=3, horizon=12):
    return RacerCRPOEnv(
        MockRacerBackend(n_uavs, horizon=horizon),
        GuidanceManager(None, n_uavs),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )


class OneShotMockBackend(MockRacerBackend):
    """Mirror a single Isaac process: terminal reset has no next episode."""

    def __init__(self, n_uavs: int, horizon: int):
        super().__init__(n_uavs, horizon=horizon)
        self.started = False

    def reset(self, seed=None):
        if self.started:
            return self._snapshot(coverage_delta=0.0)
        self.started = True
        return super().reset(seed)

    def step(self, action_matrix):
        if self.decision_index >= self.horizon:
            raise MissionEndedError("single Isaac episode ended")
        return super().step(action_matrix)


class StickyTerminalMockBackend(MockRacerBackend):
    """Expose the terminal snapshot again if a collector steps past it."""

    def __init__(self, n_uavs: int, horizon: int):
        super().__init__(n_uavs, horizon=horizon)
        self.started = False
        self.step_calls = 0

    def reset(self, seed=None):
        if self.started:
            return self._snapshot(coverage_delta=0.0)
        self.started = True
        return super().reset(seed)

    def step(self, action_matrix):
        self.step_calls += 1
        return super().step(action_matrix)


class ChannelMaskMockBackend(MockRacerBackend):
    """Exercise directional no-link handling and a weak usable BS link."""

    def _update_channel(self):
        super()._update_channel()
        bs = self.n_uavs
        # UAV 2 can upload but cannot receive a BS downlink. Its upload action
        # must remain feasible while relay actions involving it stay masked.
        self.channel[1, bs] = -50.0
        self.channel[bs, 1] = -120.0
        # A poor but usable link must not be confused with the no-link sentinel.
        self.channel[2, bs] = -50.0
        self.channel[bs, 2] = -50.0


class AsyncMetricMockBackend(MockRacerBackend):
    task_metrics_are_async = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.metrics = {}

    def reset(self, seed=None):
        snapshot = super().reset(seed)
        self.metrics[snapshot.step_id] = snapshot.task_metrics
        return snapshot

    def step(self, action_matrix):
        result = super().step(action_matrix)
        self.metrics[result.snapshot.step_id] = result.snapshot.task_metrics
        return result

    def resolve_task_metrics(self, step_ids):
        return {int(step): self.metrics[int(step)] for step in step_ids}


class AsyncSingleMissionMockBackend(AsyncMetricMockBackend):
    """Require terminal autoreset to preserve delayed metric state."""

    single_mission_terminal_reset_is_noop = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reset_calls = 0

    def reset(self, seed=None):
        self.reset_calls += 1
        return super().reset(seed)


class VersionedActionMockBackend(MockRacerBackend):
    """Expose the same action/step identity used by shared-memory runtime."""

    decoupled_transition_collection = True

    def __init__(self, *args, stale=False, **kwargs):
        super().__init__(*args, **kwargs)
        self.stale = bool(stale)
        self.published_action_version = 0
        self.policy_version = 0
        self.pending_action = None
        self.pending_source_step = 0

    def reset(self, seed=None):
        self.published_action_version = 0
        self.pending_action = None
        return super().reset(seed)

    def synchronization_status(self):
        return {
            "enabled": False,
            "task_step": self.decision_index,
            "rl_decision_index": self.decision_index,
            "sim_time_s": 0.1 * self.decision_index,
            "next_action_version": self.published_action_version + 1,
        }

    def set_action_context(self, *, guidance_id=0, policy_version=0):
        del guidance_id
        self.policy_version = int(policy_version)

    def publish_action(self, action_matrix, source_context=None):
        source = source_context or {}
        self.pending_source_step = int(
            source.get("rl_decision_index", self.decision_index)
        )
        self.published_action_version += 1
        self.pending_action = np.asarray(action_matrix, dtype=np.int8).copy()
        return self.published_action_version

    def collect_transition(self):
        # The production boundary ring waits independently while the policy
        # worker publishes.  Reproduce that scheduling opportunity instead
        # of requiring the action to exist before collection starts.
        import time

        deadline = time.perf_counter() + 1.0
        while self.pending_action is None and time.perf_counter() < deadline:
            time.sleep(0.001)
        if self.pending_action is None:
            raise RuntimeError("mock policy action was not published")
        time.sleep(0.001)
        executed_action = (
            np.zeros_like(self.pending_action)
            if self.stale
            else self.pending_action
        )
        result = super().step(executed_action)
        self.pending_action = None
        executed_version = (
            self.published_action_version - 1
            if self.stale
            else self.published_action_version
        )
        return replace(
            result,
            executed_action_matrix=np.asarray(executed_action, dtype=np.int8),
            action_version=executed_version,
            policy_version=self.policy_version,
            action_source_step_id=(
                max(0, self.pending_source_step - 1)
                if self.stale
                else self.pending_source_step
            ),
            action_source_sim_time_s=(
                0.1 * max(0, self.pending_source_step - 1)
                if self.stale
                else 0.1 * self.pending_source_step
            ),
        )

    def step(self, action_matrix):
        self.publish_action(action_matrix)
        return self.collect_transition()


def make_versioned_action_env(*, stale, horizon=4):
    return RacerCRPOEnv(
        VersionedActionMockBackend(3, horizon=horizon, stale=stale),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )


def make_channel_mask_env(horizon=12):
    return RacerCRPOEnv(
        ChannelMaskMockBackend(3, horizon=horizon),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )


def test_crpo_switching_rule():
    assert select_crpo_mode(0.19, 0.20, 0.0) == "reward"
    assert select_crpo_mode(0.20, 0.20, 0.0) == "reward"
    assert select_crpo_mode(0.22, 0.20, 0.01) == "cost"


def test_crpo_mode_uses_running_time_weighted_mean_not_cost_critic():
    model = CRPOPPO(
        make_env(horizon=4),
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        gamma_task=0.4,
        device="cpu",
        seed=41,
    )
    tracker = model.episode_cost_tracker
    tracker.lengths[0] = 1
    tracker.constraint_values[0] = 0.35
    model._last_cost_value_estimate = np.asarray([1000.0])
    estimate, source = model._constraint_estimate()
    assert estimate == 0.35
    assert source == "partial_time_weighted_mean"
    assert select_crpo_mode(estimate, model.gamma_task, model.eta) == "reward"

    tracker.constraint_values[0] = 0.45
    model._last_cost_value_estimate = np.asarray([-1000.0])
    estimate, source = model._constraint_estimate()
    assert estimate == 0.45
    assert source == "partial_time_weighted_mean"
    assert select_crpo_mode(estimate, model.gamma_task, model.eta) == "cost"


def test_n10_dual_encoder_forward_action_sampling_and_value_prediction():
    env = make_env(n_uavs=10, horizon=4)
    model = CRPOPPO(
        env,
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=5,
    )
    policy = model.policy
    extractor = policy.features_extractor
    assert env.observation_space.shape == (N10_OBSERVATION_DIM,)
    assert extractor.physical_state_dim == N10_PHYSICAL_STATE_DIM
    assert extractor.guidance_dim == N10_GUIDANCE_DIM

    assert (
        extractor.physical_encoder[0].in_features,
        extractor.physical_encoder[0].out_features,
    ) == (440, 256)
    assert (
        extractor.physical_encoder[3].in_features,
        extractor.physical_encoder[3].out_features,
    ) == (256, 128)
    assert (
        extractor.guidance_encoder[0].in_features,
        extractor.guidance_encoder[0].out_features,
    ) == (110, 128)
    assert (
        extractor.guidance_encoder[3].in_features,
        extractor.guidance_encoder[3].out_features,
    ) == (128, 64)
    assert (
        extractor.fusion_encoder[0].in_features,
        extractor.fusion_encoder[0].out_features,
    ) == (192, 256)
    assert (
        extractor.fusion_encoder[3].in_features,
        extractor.fusion_encoder[3].out_features,
    ) == (256, 128)
    assert (
        policy.mlp_extractor.policy_net[0].in_features,
        policy.mlp_extractor.policy_net[0].out_features,
    ) == (138, 128)
    assert (policy.action_net.in_features, policy.action_net.out_features) == (
        128,
        100,
    )
    assert (
        policy.mlp_extractor.value_net[0].in_features,
        policy.mlp_extractor.value_net[0].out_features,
    ) == (128, 128)
    assert (policy.value_net.in_features, policy.value_net.out_features) == (
        128,
        1,
    )

    observation, _ = env.reset(seed=5)
    observation_tensor = torch.as_tensor(observation).unsqueeze(0)
    h_physical, h_guidance, fused = extractor.encode_parts(
        observation_tensor
    )
    assert h_physical.shape == (1, 128)
    assert h_guidance.shape == (1, 64)
    assert fused.shape == (1, 128)

    encoder_calls = 0

    def count_encoder_calls(_module, _inputs, _output):
        nonlocal encoder_calls
        encoder_calls += 1

    encoder_hook = extractor.register_forward_hook(count_encoder_calls)
    with torch.no_grad():
        actions, reward_value, cost_value, log_prob = policy.forward_crpo(
            observation_tensor
        )
    encoder_hook.remove()
    with torch.no_grad():
        predicted_value = policy.predict_values(observation_tensor)
    assert encoder_calls == 1
    encoder_calls = 0
    encoder_hook = extractor.register_forward_hook(count_encoder_calls)
    with torch.no_grad():
        policy.evaluate_actions_crpo(observation_tensor, actions)
    encoder_hook.remove()
    assert encoder_calls == 1
    assert actions.shape == (1, 100)
    assert torch.all((actions == 0) | (actions == 1))
    assert reward_value.shape == cost_value.shape == predicted_value.shape == (
        1,
        1,
    )
    assert log_prob.shape == (1,)
    assert torch.allclose(reward_value, predicted_value)

    fusion_before = extractor.fusion_encoder[0].weight.detach().clone()
    model.learn(total_timesteps=2)
    assert not any(
        thread.name == "crpo-policy-inference" and thread.is_alive()
        for thread in threading.enumerate()
    )
    assert model.num_timesteps == 2
    assert model.reward_updates + model.cost_updates == 1
    assert not torch.equal(fusion_before, extractor.fusion_encoder[0].weight)
    for parameter in (
        extractor.physical_encoder[0].weight,
        extractor.guidance_encoder[0].weight,
        extractor.fusion_encoder[0].weight,
    ):
        assert parameter.grad is not None
        assert torch.all(torch.isfinite(parameter.grad))


def test_task_phase_is_critic_only_and_critics_are_parameter_independent():
    model = CRPOPPO(
        make_env(n_uavs=3, horizon=8),
        n_steps=4,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=101,
    )
    policy = model.policy
    observation, _ = model.env.envs[0].reset(seed=101)
    obs = torch.as_tensor(observation).unsqueeze(0)
    tau = torch.tensor([0.75])

    assert policy.critic_architecture_version == (
        TASK_PHASE_DUAL_CRITIC_ARCHITECTURE
    )
    assert policy.features_extractor.observation_dim == obs.shape[-1]
    assert policy.mlp_extractor.policy_net[0].in_features == 128 + 3
    assert policy.reward_critic.input_dim == obs.shape[-1] + 1
    assert policy.constraint_critic.input_dim == obs.shape[-1] + 1

    actor_inputs = []
    reward_phase_inputs = []
    constraint_phase_inputs = []
    actor_hook = policy.features_extractor.register_forward_pre_hook(
        lambda _module, inputs: actor_inputs.append(inputs[0].detach().clone())
    )
    reward_hook = policy.reward_critic.features_extractor.fusion_encoder[
        0
    ].register_forward_pre_hook(
        lambda _module, inputs: reward_phase_inputs.append(
            inputs[0].detach().clone()
        )
    )
    constraint_hook = (
        policy.constraint_critic.features_extractor.fusion_encoder[
            0
        ].register_forward_pre_hook(
            lambda _module, inputs: constraint_phase_inputs.append(
                inputs[0].detach().clone()
            )
        )
    )
    with torch.no_grad():
        actions, reward_value, constraint_value, _ = policy.forward_crpo(
            obs, critic_tau=tau
        )
    actor_hook.remove()
    reward_hook.remove()
    constraint_hook.remove()

    assert actions.shape == (1, 9)
    assert reward_value.shape == constraint_value.shape == (1, 1)
    assert actor_inputs and actor_inputs[0].shape[-1] == obs.shape[-1]
    assert reward_phase_inputs[0].shape[-1] == 193
    assert constraint_phase_inputs[0].shape[-1] == 193
    assert reward_phase_inputs[0][0, -1].item() == pytest.approx(0.75)
    assert constraint_phase_inputs[0][0, -1].item() == pytest.approx(0.75)

    actor_parameters = {
        parameter.data_ptr()
        for module in (
            policy.features_extractor,
            policy.mlp_extractor.policy_net,
            policy.action_net,
        )
        for parameter in module.parameters()
    }
    reward_parameters = {
        parameter.data_ptr() for parameter in policy.reward_critic.parameters()
    }
    constraint_parameters = {
        parameter.data_ptr()
        for parameter in policy.constraint_critic.parameters()
    }
    assert actor_parameters.isdisjoint(reward_parameters)
    assert actor_parameters.isdisjoint(constraint_parameters)
    assert reward_parameters.isdisjoint(constraint_parameters)


def test_episode_phase_is_clipped_stored_and_resets_to_zero():
    env = RacerCRPOEnv(
        MockRacerBackend(3, horizon=8),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
        constraint_episode_duration_s=0.4,
    )
    env.reset(seed=103)
    initial_tau = CRPOPPO._critic_tau_from_cycle_status(
        env.cycle_status(), 1
    )
    assert initial_tau.tolist() == [0.0]

    model = CRPOPPO(
        env,
        n_steps=4,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=103,
    )
    model.learn(total_timesteps=4)
    required_logs = {
        "train/reward_adv_mean",
        "train/reward_adv_std",
        "train/constraint_adv_mean",
        "train/constraint_adv_std",
        "train/reward_value_loss",
        "train/constraint_value_loss",
        "train/critic_tau_mean",
    }
    assert required_logs.issubset(model.logger.name_to_value)
    assert all(
        np.isfinite(model.logger.name_to_value[name])
        for name in required_logs
    )
    np.testing.assert_allclose(
        model.rollout_buffer.critic_taus.reshape(-1),
        [0.0, 0.25, 0.5, 0.75],
        atol=1.0e-6,
    )
    assert np.all(model.rollout_buffer.critic_taus >= 0.0)
    assert np.all(model.rollout_buffer.critic_taus <= 1.0)
    assert CRPOPPO._normalized_critic_tau(-1.0, 0.4) == 0.0
    assert CRPOPPO._normalized_critic_tau(1.0, 0.4) == 1.0

    env.reset(seed=104)
    reset_tau = CRPOPPO._critic_tau_from_cycle_status(env.cycle_status(), 1)
    assert reset_tau.tolist() == [0.0]


def test_pre_phase_policy_state_migrates_into_both_independent_critics():
    model = CRPOPPO(
        make_env(n_uavs=3, horizon=4),
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=107,
    )
    policy = model.policy
    legacy_state = {
        name: value
        for name, value in policy.state_dict().items()
        if not name.startswith(("reward_critic.", "constraint_critic."))
    }
    policy.load_state_dict(legacy_state, strict=True)

    for critic in (policy.reward_critic, policy.constraint_critic):
        fusion_weight = critic.features_extractor.fusion_encoder[0].weight
        assert torch.allclose(
            fusion_weight[:, :-1],
            policy.features_extractor.fusion_encoder[0].weight,
        )
        assert torch.count_nonzero(fusion_weight[:, -1]).item() == 0
    assert torch.allclose(
        policy.reward_critic.value_head.weight, policy.value_net.weight
    )
    assert torch.allclose(
        policy.constraint_critic.value_head.weight,
        policy.cost_value_net.weight,
    )


def test_bs_channel_mask_uses_direction_required_by_each_action():
    env = make_channel_mask_env(horizon=4)
    model = CRPOPPO(
        env,
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=17,
    )
    observation, _ = env.reset(seed=17)
    obs_tensor = torch.as_tensor(observation).unsqueeze(0)
    policy = model.policy

    uplink_availability = policy.bs_uplink_availability(obs_tensor)
    expected_uplink_availability = torch.tensor([[True, True, True]])
    assert torch.equal(uplink_availability, expected_uplink_availability)
    downlink_availability = policy.bs_downlink_availability(obs_tensor)
    expected_downlink_availability = torch.tensor([[True, False, True]])
    assert torch.equal(downlink_availability, expected_downlink_availability)
    availability = policy.bs_link_availability(obs_tensor)
    expected_availability = torch.tensor([[True, False, True]])
    assert torch.equal(availability, expected_availability)
    expected_matrix_mask = torch.tensor(
        [
            [False, False, True],
            [True, False, True],
            [True, False, False],
            [True, True, True],
        ]
    ).unsqueeze(0)
    assert torch.equal(policy.link_mask_matrix(obs_tensor), expected_matrix_mask)
    expected_action_mask = torch.tensor(
        [[False, True, True, True, True, False, True, True, True]]
    )
    assert torch.equal(policy.action_mask(obs_tensor), expected_action_mask)
    assert policy.mlp_extractor.policy_net[0].in_features == 128 + 3

    with torch.no_grad():
        policy.action_net.weight.zero_()
        policy.action_net.bias.fill_(20.0)
        distribution = policy.get_distribution(obs_tensor)
        probabilities = distribution.distribution.probs
        (
            actions,
            _,
            _,
            old_log_prob,
            masked_logits,
        ) = policy.forward_crpo_with_masked_logits(obs_tensor)
        _, _, new_log_prob, entropy = policy.evaluate_actions_crpo(
            obs_tensor, actions
        )
    assert torch.equal(
        probabilities[~expected_action_mask],
        torch.zeros_like(probabilities[~expected_action_mask]),
    )
    assert torch.all(probabilities[expected_action_mask] > 0.0)
    assert torch.all(actions[~expected_action_mask] == 0)
    assert torch.allclose(old_log_prob, new_log_prob)
    cached_log_prob = _bernoulli_log_prob_from_masked_logits(
        masked_logits.cpu().numpy(), actions.cpu().numpy()
    )
    assert np.allclose(
        cached_log_prob, old_log_prob.cpu().numpy(), atol=1.0e-6
    )
    assert entropy is not None and torch.all(torch.isfinite(entropy))

    predicted_action, _ = model.predict(observation, deterministic=False)
    assert np.all(predicted_action[~expected_action_mask.numpy()[0]] == 0)


def test_cached_masked_log_prob_is_finite_and_rejects_impossible_action():
    minimum = np.finfo(np.float32).min
    logits = np.asarray([[minimum, 0.25, minimum]], dtype=np.float32)
    compatible = _bernoulli_log_prob_from_masked_logits(
        logits, np.asarray([[0, 1, 0]], dtype=np.int8)
    )
    assert np.all(np.isfinite(compatible))
    with pytest.raises(FloatingPointError, match="behavior-policy mask"):
        _bernoulli_log_prob_from_masked_logits(
            logits, np.asarray([[1, 1, 0]], dtype=np.int8)
        )


def test_rollout_buffer_and_crpo_update_use_the_same_masked_distribution():
    model = CRPOPPO(
        make_channel_mask_env(horizon=8),
        learning_rate=0.0,
        n_steps=4,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=19,
    )
    model.learn(total_timesteps=4)
    assert model.reward_updates + model.cost_updates == 1

    observations = torch.as_tensor(
        model.rollout_buffer.observations, device=model.device
    )
    actions = torch.as_tensor(
        model.rollout_buffer.actions, device=model.device, dtype=torch.float32
    )
    old_log_prob = torch.as_tensor(
        model.rollout_buffer.log_probs, device=model.device
    ).flatten()
    mask = model.policy.action_mask(observations)
    assert not torch.any(actions.bool() & ~mask)

    with torch.no_grad():
        _, _, new_log_prob, entropy = model.policy.evaluate_actions_crpo(
            observations, actions
        )
        probabilities = model.policy.get_distribution(
            observations
        ).distribution.probs
        ratio = torch.exp(new_log_prob - old_log_prob)
    assert torch.allclose(new_log_prob, old_log_prob, atol=1.0e-6)
    assert torch.allclose(ratio, torch.ones_like(ratio), atol=1.0e-6)
    assert torch.equal(
        probabilities[~mask], torch.zeros_like(probabilities[~mask])
    )
    assert entropy is not None and torch.all(torch.isfinite(entropy))


def test_constraint_definition_does_not_change_ppo_update_schedule():
    model = CRPOPPO(
        make_channel_mask_env(horizon=20),
        learning_rate=0.0,
        n_steps=4,
        batch_size=2,
        n_epochs=3,
        target_kl=None,
        device="cpu",
        seed=43,
    )
    model.learn(total_timesteps=4)
    update = model.sync_update_records[-1]
    assert model.num_timesteps == 4
    assert len(model.sync_update_records) == 1
    assert update["rollout_buffer_size"] == 4
    assert update["epochs_executed"] == 3
    assert update["optimizer_steps"] == 6


@pytest.mark.parametrize(
    ("stale", "expected_kind"),
    ((False, "fresh"), (True, "stale")),
)
def test_transition_log_prob_never_runs_a_second_policy_evaluation(
    stale,
    expected_kind,
    monkeypatch,
    capsys,
):
    model = CRPOPPO(
        make_versioned_action_env(stale=stale),
        learning_rate=0.0,
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=23,
        verbose=1,
    )
    rollout_evaluations = 0
    original_evaluate = model.policy.evaluate_actions_crpo

    def counted_evaluate(observations, actions, *args, **kwargs):
        nonlocal rollout_evaluations
        if not model.policy.training:
            rollout_evaluations += 1
        return original_evaluate(observations, actions, *args, **kwargs)

    monkeypatch.setattr(
        model.policy, "evaluate_actions_crpo", counted_evaluate
    )
    original_add = model.rollout_buffer.add

    def deliberately_nontrivial_finalize(*args, **kwargs):
        # Releases the GIL while representing ordinary CPU-side bookkeeping,
        # making the expected inference/finalize overlap deterministic.
        import time

        time.sleep(0.01)
        return original_add(*args, **kwargs)

    monkeypatch.setattr(
        model.rollout_buffer, "add", deliberately_nontrivial_finalize
    )
    model.learn(total_timesteps=2)
    output_lines = capsys.readouterr().out.splitlines()
    transition_records = [
        json.loads(line.removeprefix("RACER_RL_TRANSITION "))
        for line in output_lines
        if line.startswith("RACER_RL_TRANSITION ")
    ]
    inference_records = [
        json.loads(line.removeprefix("RACER_RL_INFERENCE "))
        for line in output_lines
        if line.startswith("RACER_RL_INFERENCE ")
    ]
    rollout_inference_records = [
        record
        for record in inference_records
        if not record["update_in_progress"]
    ]
    update_inference_records = [
        record
        for record in inference_records
        if record["update_in_progress"]
    ]

    assert rollout_evaluations == 0
    assert len(transition_records) == 2
    assert len(rollout_inference_records) == 2
    assert update_inference_records
    assert all(
        not record["waited_for_action_ack"]
        for record in inference_records
    )
    assert all(
        not record["waited_for_next_state"]
        for record in inference_records
    )
    assert all(
        not record["request_ring_blocked"]
        for record in inference_records
    )
    assert all(
        record["transition_kind"] == expected_kind
        for record in transition_records
    )
    assert all(
        not record["second_policy_forward"]
        for record in transition_records
    )
    assert all(
        record["stale_policy_forward_latency_ms"] == 0.0
        for record in transition_records
    )
    assert all(
        record["transition_waited_for_inference_ms"] == 0.0
        for record in transition_records
    )
    assert all(
        record["decoupled_inference_collector"]
        for record in transition_records
    )
    assert all(
        not record["inference_blocked_by_transition_finalize"]
        for record in transition_records
    )
    assert transition_records[0]["next_inference_overlap_observed"]
    assert (
        rollout_inference_records[1]["inference_started_perf_ns"]
        < transition_records[0]["transition_finalize_completed_perf_ns"]
    )
    assert all(
        record["rl_cycle_latency_ms"] >= 0.0
        for record in transition_records
    )
    assert all(
        record["proposed_sim_timestamp"]
        == pytest.approx(0.1 * record["proposed_step_id"])
        for record in transition_records
    )
    timing_key = f"{expected_kind}_transition_processing_latency_ms"
    assert all(record[timing_key] >= 0.0 for record in transition_records)
    with torch.no_grad():
        observations = torch.as_tensor(
            model.rollout_buffer.observations, device=model.device
        )
        actions = torch.as_tensor(
            model.rollout_buffer.actions,
            device=model.device,
            dtype=torch.float32,
        )
        _, _, recomputed_log_prob, _ = model.policy.evaluate_actions_crpo(
            observations, actions
        )
    stored_log_prob = torch.as_tensor(
        model.rollout_buffer.log_probs, device=model.device
    ).flatten()
    assert torch.allclose(recomputed_log_prob, stored_log_prob, atol=1.0e-6)


def test_async_update_keeps_frozen_behavior_inference_on_latest_states():
    model = CRPOPPO(
        make_versioned_action_env(stale=False),
        n_steps=2,
        batch_size=2,
        n_epochs=12,
        device="cpu",
        seed=29,
    )
    # Keep the learner busy long enough to deterministically observe several
    # update-window states without changing any PPO/CRPO calculation.
    original_optimizer_step = model.policy.optimizer.step

    def slow_optimizer_step(*args, **kwargs):
        result = original_optimizer_step(*args, **kwargs)
        import time

        time.sleep(0.005)
        return result

    model.policy.optimizer.step = slow_optimizer_step
    old_behavior = model.behavior_policy
    frozen_parameters = [
        parameter.detach().clone() for parameter in old_behavior.parameters()
    ]

    assert model.behavior_policy is not model.learner_policy
    assert model._policy_parameters_are_disjoint(
        model.behavior_policy, model.learner_policy
    )
    model.learn(total_timesteps=2)

    update = model.sync_update_records[-1]
    assert update["async_learner"] is True
    assert update["actions_generated_during_update"] >= 2
    assert update["distinct_update_states_inferred"] >= 2
    assert update["update_transitions_excluded"] >= 1
    assert update["behavior_parameters_unchanged"] is True
    assert update["behavior_learner_storage_disjoint"] is True
    assert update["inference_used_updating_parameters"] is False
    assert update["policy_swap_success"] is True
    assert model.policy_version == 1
    assert model.behavior_policy is not old_behavior
    assert all(
        torch.equal(before, after)
        for before, after in zip(frozen_parameters, old_behavior.parameters())
    )
    assert all(
        not parameter.requires_grad
        for parameter in model.behavior_policy.parameters()
    )
    assert all(
        torch.equal(behavior, learner)
        for behavior, learner in zip(
            model.behavior_policy.parameters(), model.learner_policy.parameters()
        )
    )
    assert np.all(
        model.rollout_buffer.policy_versions[
            : model.rollout_buffer.valid_size
        ]
        == 0
    )
    assert not np.any(
        model.rollout_buffer.update_in_progress_flags[
            : model.rollout_buffer.valid_size
        ]
    )


def test_each_new_rollout_contains_only_the_swapped_behavior_version():
    model = CRPOPPO(
        make_versioned_action_env(stale=False, horizon=100),
        n_steps=2,
        batch_size=2,
        n_epochs=1,
        device="cpu",
        seed=31,
    )

    model.learn(total_timesteps=4)

    assert model.policy_version == 2
    assert len(model.sync_update_records) == 2
    assert np.all(
        model.rollout_buffer.policy_versions[
            : model.rollout_buffer.valid_size
        ]
        == 1
    )
    assert all(
        record["inference_used_updating_parameters"] is False
        for record in model.sync_update_records
    )


def test_mock_crpo_updates_and_checkpoint_roundtrip(tmp_path):
    env = make_env()
    model = CRPOPPO(
        env,
        n_steps=8,
        batch_size=4,
        n_epochs=2,
        gamma_task=1.5,
        device="cpu",
        seed=7,
    )
    reward_before = model.policy.reward_critic.value_head.weight.detach().clone()
    cost_before = (
        model.policy.constraint_critic.value_head.weight.detach().clone()
    )
    model.learn(24)
    assert not torch.equal(
        reward_before, model.policy.reward_critic.value_head.weight
    )
    assert not torch.equal(
        cost_before, model.policy.constraint_critic.value_head.weight
    )
    for parameter in (
        model.policy.physical_encoder[0].weight,
        model.policy.guidance_encoder[0].weight,
        model.policy.fusion_encoder[0].weight,
        model.policy.action_net.weight,
        model.policy.reward_critic.value_head.weight,
        model.policy.constraint_critic.value_head.weight,
    ):
        assert torch.all(torch.isfinite(parameter))
        assert parameter.grad is not None
        assert torch.all(torch.isfinite(parameter.grad))
    assert model.reward_updates + model.cost_updates > 0
    assert model.episode_cost_tracker.records
    assert max(
        abs(item.telescoping_error) for item in model.episode_cost_tracker.records
    ) < 1.0e-5

    checkpoint = tmp_path / "crpo_smoke"
    model.save(checkpoint)
    assert model.policy_architecture_version == DUAL_ENCODER_ARCHITECTURE
    assert model.constraint_cost_version == CONSTRAINT_COST_VERSION
    detached = CRPOPPO.load(checkpoint, device="cpu")
    assert detached.episode_cost_tracker.records
    assert detached.constraint_cost_version == CONSTRAINT_COST_VERSION
    restored = CRPOPPO.load(checkpoint, env=make_env(), device="cpu")
    assert restored.behavior_policy is not restored.learner_policy
    assert restored._policy_parameters_are_disjoint(
        restored.behavior_policy, restored.learner_policy
    )
    assert restored._behavior_policy_version == restored.policy_version
    observation, _ = restored.env.envs[0].reset(seed=11)
    action, _ = restored.predict(observation, deterministic=True)
    assert action.shape == (9,)
    assert np.all((action == 0) | (action == 1))

    data, params, pytorch_variables = load_from_zip_file(checkpoint)
    assert data is not None
    data.pop("policy_architecture_version")
    legacy_checkpoint = tmp_path / "legacy_single_encoder"
    save_to_zip_file(
        legacy_checkpoint,
        data=data,
        params=params,
        pytorch_variables=pytorch_variables,
    )
    with pytest.raises(ValueError, match="incompatible checkpoint architecture"):
        CRPOPPO.load(legacy_checkpoint, device="cpu")


def test_terminal_partial_rollout_is_trained_instead_of_discarded():
    env = RacerCRPOEnv(
        OneShotMockBackend(3, horizon=5),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )
    model = CRPOPPO(
        env,
        n_steps=8,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=13,
    )
    model.learn(100)
    assert model.num_timesteps == 5
    assert model.ended_on_mission_boundary is True
    assert model.partial_rollout_updates == 1
    assert model.reward_updates + model.cost_updates == 1
    assert model.sync_update_records[-1]["rollout_buffer_size"] == 5


def test_terminal_snapshot_is_not_recorded_as_repeated_one_step_episodes():
    backend = StickyTerminalMockBackend(3, horizon=5)
    env = RacerCRPOEnv(
        backend,
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )
    model = CRPOPPO(
        env,
        n_steps=8,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=17,
    )
    model.learn(100)
    assert model.num_timesteps == 5
    assert backend.step_calls == 5
    assert len(model.episode_cost_tracker.records) == 1
    assert model.episode_cost_tracker.records[0].episode_length == 5


def test_async_terminal_partial_rollout_preserves_metric_state_during_autoreset():
    backend = AsyncSingleMissionMockBackend(3, horizon=5)
    env = RacerCRPOEnv(
        backend,
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )
    model = CRPOPPO(
        env,
        n_steps=8,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=19,
    )

    model.learn(100)

    assert backend.reset_calls == 1
    assert model.num_timesteps == 5
    assert model.ended_on_mission_boundary is True
    assert model.partial_rollout_updates == 1
    assert model.rollout_buffer.valid_size == 5
    assert np.all(model.rollout_buffer.cost_ready[:5])


def test_crpo_buffer_refuses_pending_cost_until_step_backfill():
    buffer = CRPORolloutBuffer(
        2,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=1.0,
        gae_lambda=0.95,
        n_envs=1,
    )
    buffer.add(
        np.zeros((1, 3), np.float32),
        np.zeros((1, 2), np.int8),
        np.zeros(1, np.float32),
        np.zeros(1, np.float32),
        np.zeros(1, dtype=bool),
        torch.zeros(1),
        torch.zeros(1),
        torch.zeros(1),
        step_id=np.asarray([7]),
        sim_time=np.asarray([0.7]),
        cost_ready=np.asarray([False]),
    )
    with pytest.raises(RuntimeError, match="rollout costs are pending"):
        buffer.compute_returns_and_advantage(
            torch.zeros(1), torch.zeros(1), np.zeros(1, dtype=bool)
        )
    buffer.backfill_cost(0, 0, 0.125)
    buffer.compute_returns_and_advantage(
        torch.zeros(1), torch.zeros(1), np.zeros(1, dtype=bool)
    )
    assert buffer.costs[0, 0] == pytest.approx(0.125)
    assert bool(buffer.cost_ready[0, 0])


def test_constraint_cost_drives_cost_gae_and_value_targets():
    buffer = CRPORolloutBuffer(
        2,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=1.0,
        gae_lambda=1.0,
        gamma_cost=1.0,
        gae_lambda_cost=1.0,
        n_envs=1,
    )
    for cost in (0.1, 0.2):
        buffer.add(
            np.zeros((1, 3), np.float32),
            np.zeros((1, 2), np.int8),
            np.zeros(1, np.float32),
            np.asarray([cost], np.float32),
            np.zeros(1, dtype=bool),
            torch.zeros(1),
            torch.zeros(1),
            torch.zeros(1),
        )
    buffer.compute_returns_and_advantage(
        torch.zeros(1),
        torch.zeros(1),
        np.ones(1, dtype=bool),
    )
    np.testing.assert_allclose(buffer.cost_advantages[:, 0], [0.3, 0.2])
    np.testing.assert_allclose(buffer.cost_returns[:, 0], [0.3, 0.2])


def test_reward_and_constraint_advantages_normalize_once_and_independently():
    buffer = CRPORolloutBuffer(
        3,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=1.0,
        gae_lambda=1.0,
        gamma_cost=1.0,
        gae_lambda_cost=1.0,
        n_envs=1,
    )
    for index, (reward, cost) in enumerate(
        ((1.0, 0.1), (3.0, 0.2), (2.0, 0.8))
    ):
        buffer.add(
            np.zeros((1, 3), np.float32),
            np.zeros((1, 2), np.int8),
            np.asarray([reward], np.float32),
            np.asarray([cost], np.float32),
            np.zeros(1, dtype=bool),
            torch.zeros(1),
            torch.zeros(1),
            torch.zeros(1),
            critic_tau=np.asarray([0.25 * index], np.float32),
        )
    buffer.compute_returns_and_advantage(
        torch.zeros(1), torch.zeros(1), np.ones(1, dtype=bool)
    )

    reward_advantages = buffer.advantages.copy()
    constraint_advantages = buffer.cost_advantages.copy()
    reward_returns = buffer.returns.copy()
    constraint_returns = buffer.cost_returns.copy()
    expected_reward = (
        reward_advantages - reward_advantages.mean()
    ) / (reward_advantages.std(ddof=0) + 1.0e-8)
    expected_constraint = (
        constraint_advantages - constraint_advantages.mean()
    ) / (constraint_advantages.std(ddof=0) + 1.0e-8)
    np.testing.assert_allclose(
        buffer.normalized_advantages, expected_reward, atol=1.0e-6
    )
    np.testing.assert_allclose(
        buffer.normalized_cost_advantages,
        expected_constraint,
        atol=1.0e-6,
    )
    assert buffer.normalized_advantages.mean() == pytest.approx(0.0, abs=1e-6)
    assert buffer.normalized_advantages.std(ddof=0) == pytest.approx(
        1.0, abs=1e-6
    )
    assert buffer.normalized_cost_advantages.mean() == pytest.approx(
        0.0, abs=1e-6
    )
    assert buffer.normalized_cost_advantages.std(ddof=0) == pytest.approx(
        1.0, abs=1e-6
    )
    assert not np.allclose(
        buffer.normalized_advantages, buffer.normalized_cost_advantages
    )
    np.testing.assert_array_equal(buffer.advantages, reward_advantages)
    np.testing.assert_array_equal(
        buffer.cost_advantages, constraint_advantages
    )
    np.testing.assert_array_equal(buffer.returns, reward_returns)
    np.testing.assert_array_equal(buffer.cost_returns, constraint_returns)

    first_minibatch = next(buffer.get(batch_size=1))
    assert first_minibatch.normalized_advantages.numel() == 1
    assert first_minibatch.normalized_cost_advantages.numel() == 1


def test_crpo_buffer_refuses_nonfinite_behavior_log_probability():
    buffer = CRPORolloutBuffer(
        1,
        spaces.Box(0.0, 1.0, shape=(3,), dtype=np.float32),
        spaces.MultiBinary(2),
        device="cpu",
        gamma=1.0,
        gae_lambda=0.95,
        n_envs=1,
    )
    with pytest.raises(FloatingPointError, match="old log probability"):
        buffer.add(
            np.zeros((1, 3), np.float32),
            np.zeros((1, 2), np.int8),
            np.zeros(1, np.float32),
            np.zeros(1, np.float32),
            np.zeros(1, dtype=bool),
            torch.zeros(1),
            torch.zeros(1),
            np.asarray([-np.inf], dtype=np.float32),
        )


def test_async_task_costs_are_backfilled_before_crpo_update():
    env = RacerCRPOEnv(
        AsyncMetricMockBackend(3, horizon=20),
        GuidanceManager(None, 3),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )
    model = CRPOPPO(
        env,
        n_steps=4,
        batch_size=4,
        n_epochs=1,
        device="cpu",
        seed=17,
    )
    model.learn(4)
    assert model.rollout_buffer.valid_size == 4
    assert np.all(model.rollout_buffer.cost_ready[:4])
    np.testing.assert_array_equal(
        model.rollout_buffer.step_ids[:4, 0], np.arange(4)
    )
    np.testing.assert_allclose(
        model.rollout_buffer.sim_times[:4, 0], 0.1 * np.arange(4)
    )
    assert np.all(model.rollout_buffer.costs[:4, 0] >= 0.0)
    assert np.sum(model.rollout_buffer.costs[:4, 0]) == pytest.approx(
        env.task_tracker.sum_cost
    )
