import numpy as np
import pytest
import torch
from stable_baselines3.common.save_util import (
    load_from_zip_file,
    save_to_zip_file,
)

from agentic_crpo.backend import MissionEndedError, MockRacerBackend
from agentic_crpo.crpo_ppo import CRPOPPO, select_crpo_mode
from agentic_crpo.crpo_policy import (
    DUAL_ENCODER_ARCHITECTURE,
    N10_GUIDANCE_DIM,
    N10_OBSERVATION_DIM,
    N10_PHYSICAL_STATE_DIM,
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


class ChannelMaskMockBackend(MockRacerBackend):
    """Keep UAV 2 unavailable and UAV 3 available with a poor BS link."""

    def _update_channel(self):
        super()._update_channel()
        bs = self.n_uavs
        self.channel[1, bs] = -120.0
        self.channel[bs, 1] = -120.0
        self.channel[2, bs] = -50.0
        self.channel[bs, 2] = -50.0


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

    with torch.no_grad():
        actions, reward_value, cost_value, log_prob = policy.forward_crpo(
            observation_tensor
        )
        predicted_value = policy.predict_values(observation_tensor)
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


def test_bs_channel_mask_is_applied_before_sampling_and_log_probability():
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

    availability = policy.bs_link_availability(obs_tensor)
    expected_availability = torch.tensor([[True, False, True]])
    assert torch.equal(availability, expected_availability)
    expected_matrix_mask = torch.tensor(
        [
            [False, False, True],
            [False, False, False],
            [True, False, False],
            [True, False, True],
        ]
    ).unsqueeze(0)
    assert torch.equal(policy.link_mask_matrix(obs_tensor), expected_matrix_mask)
    expected_action_mask = torch.tensor(
        [[False, True, False, False, True, False, True, False, True]]
    )
    assert torch.equal(policy.action_mask(obs_tensor), expected_action_mask)
    assert policy.mlp_extractor.policy_net[0].in_features == 128 + 3

    with torch.no_grad():
        policy.action_net.weight.zero_()
        policy.action_net.bias.fill_(20.0)
        distribution = policy.get_distribution(obs_tensor)
        probabilities = distribution.distribution.probs
        actions, _, _, old_log_prob = policy.forward_crpo(obs_tensor)
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
    assert entropy is not None and torch.all(torch.isfinite(entropy))

    predicted_action, _ = model.predict(observation, deterministic=False)
    assert np.all(predicted_action[~expected_action_mask.numpy()[0]] == 0)


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
    reward_before = model.policy.value_net.weight.detach().clone()
    cost_before = model.policy.cost_value_net.weight.detach().clone()
    model.learn(24)
    assert not torch.equal(reward_before, model.policy.value_net.weight)
    assert not torch.equal(cost_before, model.policy.cost_value_net.weight)
    for parameter in (
        model.policy.physical_encoder[0].weight,
        model.policy.guidance_encoder[0].weight,
        model.policy.fusion_encoder[0].weight,
        model.policy.action_net.weight,
        model.policy.value_net.weight,
        model.policy.cost_value_net.weight,
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
    detached = CRPOPPO.load(checkpoint, device="cpu")
    assert detached.episode_cost_tracker.records
    restored = CRPOPPO.load(checkpoint, env=make_env(), device="cpu")
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
