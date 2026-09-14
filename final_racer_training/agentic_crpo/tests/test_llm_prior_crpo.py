from pathlib import Path

import numpy as np
import pytest
import torch
from gymnasium import spaces

from agentic_crpo.backend import MockRacerBackend
from agentic_crpo.crpo_policy import (
    CRPOActorCriticPolicy,
    DUAL_ENCODER_ARCHITECTURE,
)
from agentic_crpo.crpo_ppo import CRPOPPO
from agentic_crpo.gym_env import RacerCRPOEnv
from agentic_crpo.llm_prior_crpo_ppo import LLMPriorCRPOPPO
from agentic_crpo.llm_prior_policy import (
    LLM_PRIOR_ARCHITECTURE,
    LLMPriorCRPOActorCriticPolicy,
)
from agentic_crpo.llm_prior_process_supervisor import (
    _replace_rl_train_module,
)
from agentic_crpo.qwen_global_agent import GuidanceManager
from agentic_crpo.state_builder import StateNormalization
from agentic_crpo.task_loss import TaskLossWeights


def make_env(n_uavs: int = 3, horizon: int = 12) -> RacerCRPOEnv:
    return RacerCRPOEnv(
        MockRacerBackend(n_uavs, horizon=horizon),
        GuidanceManager(None, n_uavs),
        high_level_interval=4,
        channel_history=4,
        task_weights=TaskLossWeights(),
        normalization=StateNormalization(),
        bs_resource_normalizer=660.0,
    )


def make_policy(
    n_uavs: int = 3,
    *,
    relay_beta: float = 1.0,
    upload_beta: float = 1.0,
) -> LLMPriorCRPOActorCriticPolicy:
    return LLMPriorCRPOActorCriticPolicy(
        spaces.Box(
            0.0,
            1.0,
            shape=(5 * n_uavs * (n_uavs + 1),),
            dtype=np.float32,
        ),
        spaces.MultiBinary(n_uavs * n_uavs),
        lambda _: 2.0e-4,
        llm_prior_relay_beta=relay_beta,
        llm_prior_upload_beta=upload_beta,
        llm_prior_clip=2.0,
        llm_prior_initial_gate=0.5,
    )


def guided_observation(n_uavs: int = 3) -> torch.Tensor:
    physical_dim = 4 * n_uavs * (n_uavs + 1)
    obs = torch.zeros((1, 5 * n_uavs * (n_uavs + 1)))
    uplink_start = 3 * n_uavs + n_uavs * (n_uavs - 1)
    obs[..., uplink_start : uplink_start + 2 * n_uavs] = 0.5
    task = torch.tensor(
        [
            [0.0, 1.0, 0.0],
            [0.25, 0.0, 1.0],
            [0.75, 0.0, 0.0],
        ]
    )
    omega = torch.tensor([0.80, 0.15, 0.05])
    obs[..., physical_dim : physical_dim + n_uavs * n_uavs] = task.reshape(-1)
    obs[..., physical_dim + n_uavs * n_uavs :] = omega
    return obs


def test_prior_is_centered_bounded_and_aligned_with_action_bits() -> None:
    policy = make_policy()
    bias = policy.llm_prior_bias(guided_observation())
    relay = bias[..., :6].reshape(1, 3, 2)
    upload = bias[..., 6:]
    assert bias.shape == (1, 9)
    assert torch.all(torch.isfinite(bias))
    assert torch.max(torch.abs(bias)) <= 2.0
    assert torch.allclose(relay.mean(dim=-1), torch.zeros((1, 3)))
    assert torch.allclose(
        upload.mean(dim=-1), torch.zeros((1,)), atol=1.0e-6
    )
    # Row-major off-diagonal action order begins with 1->2 then 1->3.
    assert bias[0, 0] > bias[0, 1]


def test_unselected_llm_link_remains_a_soft_nonzero_probability() -> None:
    policy = make_policy()
    with torch.no_grad():
        policy.action_net.weight.zero_()
        policy.action_net.bias.zero_()
    observation = guided_observation()
    features = policy.extract_features(observation)
    latent = policy._actor_latent(
        features, policy.bs_link_availability(observation)
    )
    logits = policy._prior_masked_action_logits(
        observation,
        features,
        latent,
        policy.action_mask(observation),
    )
    # UAV 1 -> UAV 3 was not selected by W_task, but receives a finite bias.
    unselected_logit = logits[0, 1]
    probability = torch.sigmoid(unselected_logit)
    assert torch.isfinite(unselected_logit)
    assert 0.0 < probability < 0.5
    assert logits[0, 0] > unselected_logit


def test_physical_mask_is_the_only_hard_zero_probability() -> None:
    policy = make_policy()
    observation = guided_observation()
    n_uavs = 3
    uplink_start = 3 * n_uavs + n_uavs * (n_uavs - 1)
    downlink_start = uplink_start + n_uavs
    # Disable BS -> UAV 3. Both relay actions whose receiver is UAV 3 are
    # physically infeasible, including the LLM-unselected UAV 1 -> UAV 3 bit.
    observation[..., downlink_start + 2] = 0.0
    distribution = policy.get_distribution(observation).distribution
    assert distribution is not None
    probabilities = distribution.probs
    assert probabilities[0, 1] == 0.0
    assert probabilities[0, 3] == 0.0
    assert probabilities[0, 0] > 0.0


def test_crpo_training_scores_the_same_prior_distribution_it_sampled() -> None:
    torch.manual_seed(7)
    policy = make_policy()
    observation = guided_observation()
    actions, _reward, _cost, sampled_log_prob, masked_logits = (
        policy.forward_crpo_with_masked_logits(observation)
    )
    _reward2, _cost2, evaluated_log_prob, entropy = (
        policy.evaluate_actions_crpo(observation, actions)
    )
    assert entropy is not None
    assert torch.all(torch.isfinite(masked_logits))
    assert torch.allclose(sampled_log_prob, evaluated_log_prob, atol=1.0e-6)


def test_variant_does_not_replace_the_baseline_policy() -> None:
    n_uavs = 3
    baseline = CRPOActorCriticPolicy(
        spaces.Box(
            0.0,
            1.0,
            shape=(5 * n_uavs * (n_uavs + 1),),
            dtype=np.float32,
        ),
        spaces.MultiBinary(n_uavs * n_uavs),
        lambda _: 2.0e-4,
    )
    assert baseline.architecture_version == DUAL_ENCODER_ARCHITECTURE
    assert not hasattr(baseline, "llm_prior_gate_net")


def test_variant_checkpoint_is_isolated_from_baseline(tmp_path: Path) -> None:
    prior_env = make_env()
    prior_model = LLMPriorCRPOPPO(
        prior_env,
        n_steps=4,
        batch_size=4,
        n_epochs=1,
        device="cpu",
    )
    prior_model.experiment_variant = "test_llm_prior"
    prior_path = tmp_path / "prior"
    prior_model.save(prior_path)
    restored = LLMPriorCRPOPPO.load(
        prior_path.with_suffix(".zip"), env=make_env(), device="cpu"
    )
    assert restored.policy_architecture_version == LLM_PRIOR_ARCHITECTURE
    with pytest.raises(ValueError, match="incompatible checkpoint architecture"):
        CRPOPPO.load(prior_path.with_suffix(".zip"), device="cpu")
    prior_env.close()

    baseline_env = make_env()
    baseline_model = CRPOPPO(
        baseline_env,
        n_steps=4,
        batch_size=4,
        n_epochs=1,
        device="cpu",
    )
    baseline_path = tmp_path / "baseline"
    baseline_model.save(baseline_path)
    with pytest.raises(ValueError, match="incompatible checkpoint architecture"):
        LLMPriorCRPOPPO.load(
            baseline_path.with_suffix(".zip"), device="cpu"
        )
    baseline_env.close()


def test_separate_supervisor_rewrites_only_the_rl_module() -> None:
    command = [
        "/usr/bin/python",
        "-m",
        "agentic_crpo.train_crpo",
        "--config",
        "config.yaml",
    ]
    replaced = _replace_rl_train_module(command)
    assert command[2] == "agentic_crpo.train_crpo"
    assert replaced[2] == "agentic_crpo.train_llm_prior_crpo"
