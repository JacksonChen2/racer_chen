"""SB3 actor with independent task-phase reward and constraint critics."""

from __future__ import annotations

import math

from gymnasium import spaces
import numpy as np
import torch as th
from torch import nn

from stable_baselines3.common.distributions import (
    BernoulliDistribution,
    Distribution,
)
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.type_aliases import PyTorchObs, Schedule


DUAL_ENCODER_ARCHITECTURE = "physical_guidance_dual_encoder_bs_mask_v2"
# SB3 keeps the last distribution/value projections as ``action_net`` and
# ``value_net``.  With a 128-wide fused feature, this gives the requested
# independent heads: 128 -> 128 -> action_dim and 128 -> 128 -> 1.
DUAL_ENCODER_NET_ARCH = {"pi": [128], "vf": [128]}
N10_PHYSICAL_STATE_DIM = 440
N10_GUIDANCE_DIM = 110
N10_OBSERVATION_DIM = N10_PHYSICAL_STATE_DIM + N10_GUIDANCE_DIM
TASK_PHASE_DUAL_CRITIC_ARCHITECTURE = (
    "independent_reward_constraint_critics_with_episode_phase_v1"
)


class PhysicalGuidanceFeatureExtractor(BaseFeaturesExtractor):
    """Encode physical state and LLM guidance independently, then fuse them."""

    features_dim = 128

    def __init__(self, observation_space: spaces.Space) -> None:
        if not isinstance(observation_space, spaces.Box) or len(
            observation_space.shape
        ) != 1:
            raise ValueError(
                "dual-encoder policy requires a flat Box observation"
            )
        observation_dim = int(observation_space.shape[0])
        if observation_dim % 5 != 0:
            raise ValueError(
                "dual-encoder policy requires [physical, guidance] without "
                f"delta; got observation dimension {observation_dim}"
            )
        guidance_dim = observation_dim // 5
        discriminant = 1 + 4 * guidance_dim
        root = math.isqrt(discriminant)
        n_uavs = (root - 1) // 2
        if (
            root * root != discriminant
            or n_uavs < 1
            or n_uavs * (n_uavs + 1) != guidance_dim
        ):
            raise ValueError(
                "dual-encoder policy requires dimensions 5*N*(N+1): "
                "physical=4*N*(N+1), guidance=N*(N+1), with no delta; "
                f"got {observation_dim}"
            )

        super().__init__(observation_space, features_dim=self.features_dim)
        self.n_uavs = n_uavs
        self.physical_state_dim = 4 * guidance_dim
        self.guidance_dim = guidance_dim
        self.observation_dim = observation_dim

        self.physical_encoder = nn.Sequential(
            nn.Linear(self.physical_state_dim, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, 128),
            nn.LayerNorm(128),
            nn.SiLU(),
        )
        self.guidance_encoder = nn.Sequential(
            nn.Linear(self.guidance_dim, 128),
            nn.LayerNorm(128),
            nn.SiLU(),
            nn.Linear(128, 64),
            nn.LayerNorm(64),
            nn.SiLU(),
        )
        self.fusion_encoder = nn.Sequential(
            nn.Linear(192, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, self.features_dim),
            nn.SiLU(),
        )

    def encode_parts(
        self, observations: th.Tensor
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor]:
        if observations.shape[-1] != self.observation_dim:
            raise ValueError(
                f"expected observation width {self.observation_dim}, "
                f"got {observations.shape[-1]}"
            )
        physical_state = observations[..., : self.physical_state_dim]
        llm_guidance = observations[..., self.physical_state_dim :]
        h_physical = self.physical_encoder(physical_state)
        h_guidance = self.guidance_encoder(llm_guidance)
        fused = self.fusion_encoder(
            th.cat((h_physical, h_guidance), dim=-1)
        )
        return h_physical, h_guidance, fused

    def forward(self, observations: th.Tensor) -> th.Tensor:
        return self.encode_parts(observations)[2]


class TaskPhaseCriticFeatureExtractor(nn.Module):
    """Independent dual encoder whose fusion input is ``[h_obs, tau]``."""

    features_dim = PhysicalGuidanceFeatureExtractor.features_dim

    def __init__(self, observation_space: spaces.Space) -> None:
        super().__init__()
        observation_encoder = PhysicalGuidanceFeatureExtractor(
            observation_space
        )
        self.n_uavs = observation_encoder.n_uavs
        self.physical_state_dim = observation_encoder.physical_state_dim
        self.guidance_dim = observation_encoder.guidance_dim
        self.observation_dim = observation_encoder.observation_dim
        self.input_dim = self.observation_dim + 1
        self.physical_encoder = observation_encoder.physical_encoder
        self.guidance_encoder = observation_encoder.guidance_encoder
        self.fusion_encoder = nn.Sequential(
            nn.Linear(193, 256),
            nn.LayerNorm(256),
            nn.SiLU(),
            nn.Linear(256, self.features_dim),
            nn.SiLU(),
        )

    def forward(self, critic_input: th.Tensor) -> th.Tensor:
        if critic_input.shape[-1] != self.input_dim:
            raise ValueError(
                f"critic expected [obs, tau] width {self.input_dim}, "
                f"got {critic_input.shape[-1]}"
            )
        physical_state = critic_input[..., : self.physical_state_dim]
        guidance_end = self.physical_state_dim + self.guidance_dim
        llm_guidance = critic_input[..., self.physical_state_dim : guidance_end]
        phase = critic_input[..., -1:]
        h_physical = self.physical_encoder(physical_state)
        h_guidance = self.guidance_encoder(llm_guidance)
        return self.fusion_encoder(
            th.cat((h_physical, h_guidance, phase), dim=-1)
        )


class TaskPhaseValueEstimator(nn.Module):
    """An independent critic whose complete input is ``[obs, tau]``.

    Each instance owns its physical, guidance, and fusion encoders.  The
    actor's feature extractor is therefore not part of either value-learning
    path, and the reward and constraint critics share no parameter storage.
    """

    def __init__(self, observation_space: spaces.Space) -> None:
        super().__init__()
        self.features_extractor = TaskPhaseCriticFeatureExtractor(
            observation_space
        )
        self.observation_dim = self.features_extractor.observation_dim
        self.input_dim = self.observation_dim + 1
        self.value_mlp = nn.Sequential(
            nn.Linear(self.features_extractor.features_dim, 128),
            nn.SiLU(),
        )
        self.value_head = nn.Linear(128, 1)

    @staticmethod
    def _phase_tensor(obs: th.Tensor, tau: th.Tensor | float | None) -> th.Tensor:
        batch_shape = tuple(obs.shape[:-1])
        if tau is None:
            phase = obs.new_zeros((*batch_shape, 1))
        else:
            phase = th.as_tensor(tau, device=obs.device, dtype=obs.dtype)
            if tuple(phase.shape) == batch_shape:
                phase = phase.unsqueeze(-1)
            elif phase.ndim == 0:
                phase = phase.expand(*batch_shape, 1)
            elif tuple(phase.shape) != (*batch_shape, 1):
                raise ValueError(
                    "critic tau shape must match the observation batch: "
                    f"obs={tuple(obs.shape)}, tau={tuple(phase.shape)}"
                )
        if not bool(th.all(th.isfinite(phase))):
            raise FloatingPointError("critic tau contains NaN or Inf")
        return phase.clamp(0.0, 1.0)

    def forward(
        self, obs: th.Tensor, tau: th.Tensor | float | None
    ) -> th.Tensor:
        if obs.shape[-1] != self.observation_dim:
            raise ValueError(
                f"critic expected observation width {self.observation_dim}, "
                f"got {obs.shape[-1]}"
            )
        phase = self._phase_tensor(obs, tau)
        critic_input = th.cat((obs, phase), dim=-1)
        if critic_input.shape[-1] != self.input_dim:
            raise RuntimeError("critic [obs, tau] input has the wrong width")
        features = self.features_extractor(critic_input)
        latent = self.value_mlp(features)
        return self.value_head(latent)


class CRPOActorCriticPolicy(ActorCriticPolicy):
    cost_value_net: nn.Linear
    reward_critic: TaskPhaseValueEstimator
    constraint_critic: TaskPhaseValueEstimator

    def __init__(
        self,
        *args: object,
        net_arch: dict[str, list[int]] | None = None,
        activation_fn: type[nn.Module] = nn.SiLU,
        features_extractor_class: type[BaseFeaturesExtractor] = (
            PhysicalGuidanceFeatureExtractor
        ),
        features_extractor_kwargs: dict[str, object] | None = None,
        share_features_extractor: bool = True,
        **kwargs: object,
    ) -> None:
        selected_arch = DUAL_ENCODER_NET_ARCH if net_arch is None else net_arch
        if selected_arch != DUAL_ENCODER_NET_ARCH:
            raise ValueError(
                "dual-encoder actor/critic heads require "
                f"net_arch={DUAL_ENCODER_NET_ARCH}, got {selected_arch}"
            )
        if activation_fn is not nn.SiLU:
            raise ValueError("dual-encoder policy requires nn.SiLU activation")
        if features_extractor_class is not PhysicalGuidanceFeatureExtractor:
            raise ValueError(
                "dual-encoder policy requires PhysicalGuidanceFeatureExtractor"
            )
        if features_extractor_kwargs not in (None, {}):
            raise ValueError("dual-encoder feature extractor takes no kwargs")
        if not share_features_extractor:
            raise ValueError("actor and critic must share the fusion feature")
        super().__init__(
            *args,
            net_arch=selected_arch,
            activation_fn=nn.SiLU,
            features_extractor_class=PhysicalGuidanceFeatureExtractor,
            features_extractor_kwargs={},
            share_features_extractor=True,
            **kwargs,
        )
        self.architecture_version = DUAL_ENCODER_ARCHITECTURE
        self.critic_architecture_version = TASK_PHASE_DUAL_CRITIC_ARCHITECTURE
        if not isinstance(self.action_space, spaces.MultiBinary):
            raise ValueError(
                "BS-link feasibility masking requires a MultiBinary action space"
            )
        n_uavs = self.features_extractor.n_uavs
        action_dim = int(np.prod(self.action_space.shape))
        expected_action_dim = n_uavs**2
        if action_dim != expected_action_dim:
            raise ValueError(
                "dual-encoder policy expected an N*N MultiBinary action "
                f"space ({expected_action_dim}), got {action_dim}"
            )
        relay_receivers = [
            receiver
            for sender in range(n_uavs)
            for receiver in range(n_uavs)
            if sender != receiver
        ]
        self.register_buffer(
            "_relay_receiver_indices",
            th.tensor(relay_receivers, dtype=th.long),
            persistent=False,
        )
        self.register_buffer(
            "_pair_off_diagonal",
            ~th.eye(n_uavs, dtype=th.bool),
            persistent=False,
        )

    @property
    def physical_encoder(self) -> nn.Sequential:
        return self.features_extractor.physical_encoder

    @property
    def guidance_encoder(self) -> nn.Sequential:
        return self.features_extractor.guidance_encoder

    @property
    def fusion_encoder(self) -> nn.Sequential:
        return self.features_extractor.fusion_encoder

    def _build(self, lr_schedule: Schedule) -> None:
        super()._build(lr_schedule)
        actor_input = self.mlp_extractor.policy_net[0]
        if not isinstance(actor_input, nn.Linear):
            raise TypeError("expected the actor head to start with nn.Linear")
        self.mlp_extractor.policy_net[0] = nn.Linear(
            actor_input.in_features + self.features_extractor.n_uavs,
            actor_input.out_features,
            bias=actor_input.bias is not None,
            device=actor_input.weight.device,
            dtype=actor_input.weight.dtype,
        )
        if self.ortho_init:
            self.mlp_extractor.policy_net[0].apply(
                lambda module: self.init_weights(module, gain=np.sqrt(2))
            )
        self.cost_value_net = nn.Linear(self.mlp_extractor.latent_dim_vf, 1)
        if self.ortho_init:
            self.cost_value_net.apply(
                lambda module: self.init_weights(module, gain=1.0)
            )
        # Retain the legacy shared critic modules in the state dictionary so
        # existing checkpoints remain loadable, but never use or optimize them.
        # New checkpoints likewise retain those keys for bidirectional tooling
        # compatibility with existing checkpoint inspection code.
        for module in (
            self.mlp_extractor.value_net,
            self.value_net,
            self.cost_value_net,
        ):
            module.requires_grad_(False)

        self.reward_critic = TaskPhaseValueEstimator(self.observation_space)
        self.constraint_critic = TaskPhaseValueEstimator(self.observation_space)
        if self.ortho_init:
            for critic in (self.reward_critic, self.constraint_critic):
                critic.apply(
                    lambda module: self.init_weights(module, gain=np.sqrt(2))
                )
                critic.value_head.apply(
                    lambda module: self.init_weights(module, gain=1.0)
                )

        # ActorCriticPolicy created its optimizer before the independent
        # critics existed. Frozen legacy value modules are intentionally
        # omitted from optimization.
        self.optimizer = self.optimizer_class(
            (parameter for parameter in self.parameters() if parameter.requires_grad),
            lr=lr_schedule(1),
            **self.optimizer_kwargs,
        )

    def _migrate_legacy_critic_state_dict(
        self, state_dict: dict[str, th.Tensor]
    ) -> dict[str, th.Tensor]:
        """Seed new independent critics when loading a pre-phase checkpoint."""

        if any(key.startswith("reward_critic.") for key in state_dict):
            return state_dict
        migrated = state_dict.copy()
        current = self.state_dict()
        for critic_name, legacy_head in (
            ("reward_critic", "value_net"),
            ("constraint_critic", "cost_value_net"),
        ):
            prefix = f"{critic_name}."
            for target, default in current.items():
                if not target.startswith(prefix) or target in migrated:
                    continue
                relative = target[len(prefix) :]
                if relative.startswith("features_extractor."):
                    source = relative
                elif relative.startswith("value_mlp."):
                    source = "mlp_extractor.value_net." + relative.removeprefix(
                        "value_mlp."
                    )
                elif relative.startswith("value_head."):
                    source = legacy_head + "." + relative.removeprefix(
                        "value_head."
                    )
                else:
                    source = ""
                value = state_dict.get(source)
                if value is not None and value.shape == default.shape:
                    migrated[target] = value
                elif (
                    value is not None
                    and target.endswith("fusion_encoder.0.weight")
                    and value.shape[:-1] == default.shape[:-1]
                    and value.shape[-1] + 1 == default.shape[-1]
                ):
                    expanded = default.clone()
                    expanded[..., :-1] = value
                    expanded[..., -1] = 0.0
                    migrated[target] = expanded
                else:
                    migrated[target] = default
        return migrated

    def load_state_dict(
        self,
        state_dict: dict[str, th.Tensor],
        strict: bool = True,
        assign: bool = False,
    ):
        migrated = self._migrate_legacy_critic_state_dict(state_dict)
        return super().load_state_dict(migrated, strict=strict, assign=assign)

    def _bs_link_directions(
        self, obs: PyTorchObs
    ) -> tuple[th.Tensor, th.Tensor]:
        """Return the directional UAV-to-BS and BS-to-UAV link masks."""

        if not isinstance(obs, th.Tensor):
            raise TypeError("dual-encoder policy requires a tensor observation")
        if obs.shape[-1] != self.features_extractor.observation_dim:
            raise ValueError(
                "cannot build action mask from observation width "
                f"{obs.shape[-1]}"
            )
        n_uavs = self.features_extractor.n_uavs
        uplink_start = 3 * n_uavs + n_uavs * (n_uavs - 1)
        downlink_start = uplink_start + n_uavs
        uplink = obs[..., uplink_start:downlink_start]
        downlink = obs[..., downlink_start : downlink_start + n_uavs]
        return uplink > 0.0, downlink > 0.0

    def bs_uplink_availability(self, obs: PyTorchObs) -> th.Tensor:
        """Return whether each UAV can upload to the BS."""

        return self._bs_link_directions(obs)[0]

    def bs_downlink_availability(self, obs: PyTorchObs) -> th.Tensor:
        """Return whether the BS can forward data to each UAV."""

        return self._bs_link_directions(obs)[1]

    def bs_link_availability(self, obs: PyTorchObs) -> th.Tensor:
        """Return whether each UAV has usable BS links in both directions."""

        uplink, downlink = self._bs_link_directions(obs)
        return uplink & downlink

    def link_mask_matrix(self, obs: PyTorchObs) -> th.Tensor:
        """Return the boolean ``(N+1) x N`` feasibility mask."""

        downlink_available = self.bs_downlink_availability(obs)
        upload_available = self.bs_uplink_availability(obs)
        pair_mask = th.logical_and(
            downlink_available.unsqueeze(-2),
            self._pair_off_diagonal,
        )
        return th.cat((pair_mask, upload_available.unsqueeze(-2)), dim=-2)

    def action_mask(self, obs: PyTorchObs) -> th.Tensor:
        """Return the mask in ActionMapping's N*N flattened bit order."""

        downlink_available = self.bs_downlink_availability(obs)
        upload_available = self.bs_uplink_availability(obs)
        relay_mask = downlink_available[..., self._relay_receiver_indices]
        return th.cat((relay_mask, upload_available), dim=-1)

    def _actor_latent(
        self, features: th.Tensor, available: th.Tensor
    ) -> th.Tensor:
        actor_input = th.cat(
            (features, available.to(dtype=features.dtype)), dim=-1
        )
        return self.mlp_extractor.policy_net(actor_input)

    def _masked_action_distribution(
        self, latent_pi: th.Tensor, action_mask: th.Tensor
    ) -> Distribution:
        masked_logits = self._masked_action_logits(latent_pi, action_mask)
        return self._distribution_from_masked_logits(masked_logits)

    def _masked_action_logits(
        self, latent_pi: th.Tensor, action_mask: th.Tensor
    ) -> th.Tensor:
        if not isinstance(self.action_dist, BernoulliDistribution):
            raise TypeError(
                "BS-link action masking requires BernoulliDistribution"
            )
        raw_logits = self.action_net(latent_pi)
        if not bool(th.all(th.isfinite(raw_logits))):
            raise FloatingPointError("policy action logits contain NaN or Inf")
        if raw_logits.shape != action_mask.shape:
            raise ValueError(
                f"action logits {raw_logits.shape} and mask "
                f"{action_mask.shape} do not match"
            )
        masked_logits = raw_logits.masked_fill(
            ~action_mask, th.finfo(raw_logits.dtype).min
        )
        return masked_logits

    def _distribution_from_masked_logits(
        self, masked_logits: th.Tensor
    ) -> Distribution:
        if not isinstance(self.action_dist, BernoulliDistribution):
            raise TypeError(
                "BS-link action masking requires BernoulliDistribution"
            )
        return self.action_dist.proba_distribution(action_logits=masked_logits)

    def forward(
        self,
        obs: th.Tensor,
        deterministic: bool = False,
        critic_tau: th.Tensor | float | None = None,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor]:
        features = self.extract_features(obs)
        available = self.bs_link_availability(obs)
        latent_pi = self._actor_latent(features, available)
        values = self.predict_reward_values(obs, critic_tau)
        distribution = self._masked_action_distribution(
            latent_pi, self.action_mask(obs)
        )
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        actions = actions.reshape((-1, *self.action_space.shape))
        return actions, values, log_prob

    def evaluate_actions(
        self,
        obs: PyTorchObs,
        actions: th.Tensor,
        critic_tau: th.Tensor | float | None = None,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor | None]:
        if not isinstance(obs, th.Tensor):
            raise TypeError("dual-encoder policy requires a tensor observation")
        features = self.extract_features(obs)
        available = self.bs_link_availability(obs)
        latent_pi = self._actor_latent(features, available)
        distribution = self._masked_action_distribution(
            latent_pi, self.action_mask(obs)
        )
        return (
            self.predict_reward_values(obs, critic_tau),
            distribution.log_prob(actions),
            distribution.entropy(),
        )

    def get_distribution(self, obs: PyTorchObs) -> Distribution:
        features = super().extract_features(obs, self.pi_features_extractor)
        available = self.bs_link_availability(obs)
        latent_pi = self._actor_latent(features, available)
        return self._masked_action_distribution(latent_pi, self.action_mask(obs))

    def predict_reward_values(
        self, obs: PyTorchObs, critic_tau: th.Tensor | float | None = None
    ) -> th.Tensor:
        if not isinstance(obs, th.Tensor):
            raise TypeError("reward critic requires a tensor observation")
        return self.reward_critic(obs, critic_tau)

    def predict_values(
        self, obs: PyTorchObs, critic_tau: th.Tensor | float | None = None
    ) -> th.Tensor:
        """SB3-compatible reward-value entry point with optional phase."""

        return self.predict_reward_values(obs, critic_tau)

    def predict_cost_values(
        self, obs: PyTorchObs, critic_tau: th.Tensor | float | None = None
    ) -> th.Tensor:
        if not isinstance(obs, th.Tensor):
            raise TypeError("constraint critic requires a tensor observation")
        return self.constraint_critic(obs, critic_tau)

    def forward_crpo(
        self,
        obs: th.Tensor,
        deterministic: bool = False,
        critic_tau: th.Tensor | float | None = None,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor, th.Tensor]:
        actions, reward_values, cost_values, log_prob, _ = (
            self.forward_crpo_with_masked_logits(
                obs, deterministic, critic_tau=critic_tau
            )
        )
        return actions, reward_values, cost_values, log_prob

    def forward_crpo_with_masked_logits(
        self,
        obs: th.Tensor,
        deterministic: bool = False,
        critic_tau: th.Tensor | float | None = None,
    ) -> tuple[
        th.Tensor,
        th.Tensor,
        th.Tensor,
        th.Tensor,
        th.Tensor,
    ]:
        """Run one shared encoder pass and expose the sampled distribution.

        The returned masked logits are sufficient to score any actually
        executed MultiBinary action on CPU.  Keeping them with the decision
        record lets transition collection avoid a second policy evaluation.
        """

        features = self.extract_features(obs)
        available = self.bs_link_availability(obs)
        latent_pi = self._actor_latent(features, available)
        reward_values = self.predict_reward_values(obs, critic_tau)
        cost_values = self.predict_cost_values(obs, critic_tau)
        masked_logits = self._masked_action_logits(
            latent_pi, self.action_mask(obs)
        )
        distribution = self._distribution_from_masked_logits(masked_logits)
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        actions = actions.reshape((-1, *self.action_space.shape))
        return (
            actions,
            reward_values,
            cost_values,
            log_prob,
            masked_logits,
        )

    def evaluate_actions_crpo(
        self,
        obs: PyTorchObs,
        actions: th.Tensor,
        critic_tau: th.Tensor | float | None = None,
    ) -> tuple[th.Tensor, th.Tensor, th.Tensor, th.Tensor | None]:
        if not isinstance(obs, th.Tensor):
            raise TypeError("dual-encoder policy requires a tensor observation")
        features = self.extract_features(obs)
        available = self.bs_link_availability(obs)
        latent_pi = self._actor_latent(features, available)
        distribution = self._masked_action_distribution(
            latent_pi, self.action_mask(obs)
        )
        return (
            self.predict_reward_values(obs, critic_tau),
            self.predict_cost_values(obs, critic_tau),
            distribution.log_prob(actions),
            distribution.entropy(),
        )
