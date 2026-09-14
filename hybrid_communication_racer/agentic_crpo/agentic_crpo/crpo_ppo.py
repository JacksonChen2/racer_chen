"""Vanilla matrix-CRPO implemented as a local SB3 PPO subclass."""

from __future__ import annotations

from collections import defaultdict
import io
import json
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
import torch as th
import torch.nn.functional as F
from gymnasium import spaces

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.save_util import load_from_zip_file
from stable_baselines3.common.type_aliases import GymEnv, Schedule
from stable_baselines3.common.utils import explained_variance, obs_as_tensor
from stable_baselines3.common.vec_env import VecEnv

from .crpo_buffer import CRPORolloutBuffer
from .crpo_policy import CRPOActorCriticPolicy, DUAL_ENCODER_ARCHITECTURE
from .backend import MissionEndedError
from .episode_tracker import EpisodeCostTracker


def select_crpo_mode(j_cost_hat: float, gamma_task: float, eta: float) -> str:
    """Pure switching rule, kept separate for deterministic unit testing."""

    return "reward" if j_cost_hat <= gamma_task + eta else "cost"


class CRPOPPO(PPO):
    """PPO with reward/cost critics and CRPO's mode-switching actor update."""

    policy: CRPOActorCriticPolicy
    rollout_buffer: CRPORolloutBuffer

    @classmethod
    def load(
        cls,
        path: str | Path | io.BufferedIOBase,
        env: GymEnv | None = None,
        device: th.device | str = "auto",
        custom_objects: dict[str, Any] | None = None,
        print_system_info: bool = False,
        force_reset: bool = True,
        **kwargs: Any,
    ) -> "CRPOPPO":
        """Reject pre-dual-encoder checkpoints before SB3 restores weights.

        SB3 contains a compatibility fallback that may retry policy loading
        with ``exact_match=False``.  That behavior is unsafe across this
        deliberate feature-extractor change, so architecture compatibility is
        checked before delegating to SB3's normal loader.
        """

        stream_position: int | None = None
        if isinstance(path, io.BufferedIOBase):
            if not path.seekable():
                raise ValueError(
                    "checkpoint streams must be seekable for architecture "
                    "validation"
                )
            stream_position = path.tell()
        data, params, _ = load_from_zip_file(
            path,
            device=device,
            custom_objects=custom_objects,
            print_system_info=False,
        )
        if stream_position is not None:
            path.seek(stream_position)
        if data is None or params is None:
            raise ValueError("checkpoint does not contain model data/parameters")

        saved_architecture = data.get("policy_architecture_version")
        if saved_architecture != DUAL_ENCODER_ARCHITECTURE:
            raise ValueError(
                "incompatible checkpoint architecture: expected "
                f"{DUAL_ENCODER_ARCHITECTURE!r}, got "
                f"{saved_architecture!r}. Legacy single-encoder checkpoints "
                "cannot initialize the Physical State + LLM Guidance dual "
                "encoder; start a new training run."
            )
        policy_state = params.get("policy")
        required_encoder_parameters = {
            "features_extractor.physical_encoder.0.weight",
            "features_extractor.guidance_encoder.0.weight",
            "features_extractor.fusion_encoder.0.weight",
        }
        if not isinstance(policy_state, dict):
            raise ValueError("checkpoint is missing the policy state dictionary")
        missing = sorted(required_encoder_parameters.difference(policy_state))
        if missing:
            raise ValueError(
                "checkpoint architecture metadata names the dual encoder, "
                f"but encoder parameters are missing: {missing}"
            )

        return super().load(
            path,
            env=env,
            device=device,
            custom_objects=custom_objects,
            print_system_info=print_system_info,
            force_reset=force_reset,
            **kwargs,
        )

    def __init__(
        self,
        env: GymEnv | str,
        learning_rate: float | Schedule = 3e-4,
        n_steps: int = 2048,
        batch_size: int = 64,
        n_epochs: int = 10,
        gamma_reward: float = 1.0,
        gamma_cost: float = 1.0,
        gae_lambda_reward: float = 0.95,
        gae_lambda_cost: float = 0.95,
        gamma_task: float = 1.0,
        eta: float = 0.0,
        episode_cost_window: int = 20,
        telescoping_tolerance: float = 1.0e-6,
        constraint_estimator: str = "episode_return",
        cost_vf_coef: float = 0.5,
        policy_kwargs: dict[str, Any] | None = None,
        **kwargs: Any,
    ) -> None:
        if constraint_estimator not in (
            "episode_return",
            "critic_estimate",
            "mean_step_cost",
        ):
            raise ValueError(
                "constraint_estimator must be episode_return, "
                "critic_estimate, or mean_step_cost"
            )
        self.gamma_cost = float(gamma_cost)
        self.gae_lambda_cost = float(gae_lambda_cost)
        self.gamma_task = float(gamma_task)
        self.eta = float(eta)
        self.episode_cost_window = int(episode_cost_window)
        self.telescoping_tolerance = float(telescoping_tolerance)
        self.constraint_estimator = constraint_estimator
        self.cost_vf_coef = float(cost_vf_coef)
        self.policy_architecture_version = DUAL_ENCODER_ARCHITECTURE
        self.reward_updates = 0
        self.cost_updates = 0
        self.crpo_mode = "reward"
        self.constraint_estimator_source = "uninitialized"
        self.j_cost_hat = 0.0
        self._last_cost_value_estimate: np.ndarray | None = None
        self.ended_on_mission_boundary = False
        self.partial_rollout_updates = 0
        self.sync_update_records: list[dict[str, Any]] = []
        self._stop_before_next_rollout = False
        # BaseAlgorithm.load reconstructs subclasses with a serialized
        # ``policy=...`` keyword. This class always owns the CRPO policy.
        kwargs.pop("policy", None)
        super().__init__(
            CRPOActorCriticPolicy,
            env,
            learning_rate=learning_rate,
            n_steps=n_steps,
            batch_size=batch_size,
            n_epochs=n_epochs,
            gamma=gamma_reward,
            gae_lambda=gae_lambda_reward,
            rollout_buffer_class=CRPORolloutBuffer,
            rollout_buffer_kwargs={
                "gamma_cost": self.gamma_cost,
                "gae_lambda_cost": self.gae_lambda_cost,
            },
            policy_kwargs=policy_kwargs,
            **kwargs,
        )
        # SB3 ``load()`` first constructs with ``_init_setup_model=False`` and
        # restores serialized attributes afterwards. In that path ``n_envs``
        # does not exist yet and the saved tracker must remain authoritative.
        if hasattr(self, "n_envs"):
            self.episode_cost_tracker = EpisodeCostTracker(
                self.n_envs,
                self.episode_cost_window,
                self.telescoping_tolerance,
                check_telescoping=(
                    self.constraint_estimator != "mean_step_cost"
                ),
            )

    def collect_rollouts(
        self,
        env: VecEnv,
        callback: BaseCallback,
        rollout_buffer: CRPORolloutBuffer,
        n_rollout_steps: int,
    ) -> bool:
        if self._stop_before_next_rollout:
            return False
        assert self._last_obs is not None
        self.policy.set_training_mode(False)
        rollout_buffer.reset()
        n_steps = 0
        callback.on_rollout_start()
        metric_values: dict[str, list[float]] = defaultdict(list)
        partial_rollout = False

        while n_steps < n_rollout_steps:
            with th.no_grad():
                obs_tensor = obs_as_tensor(self._last_obs, self.device)
                actions, values, cost_values, log_probs = self.policy.forward_crpo(
                    obs_tensor
                )
            actions_np = actions.cpu().numpy()
            clipped_actions = actions_np
            if isinstance(self.action_space, spaces.Box):
                if self.policy.squash_output:
                    clipped_actions = self.policy.unscale_action(clipped_actions)
                else:
                    clipped_actions = np.clip(
                        actions_np, self.action_space.low, self.action_space.high
                    )
            try:
                new_obs, rewards, dones, infos = env.step(clipped_actions)
            except MissionEndedError:
                self.ended_on_mission_boundary = True
                if n_steps == 0:
                    callback.on_rollout_end()
                    return False
                # The final transition was already returned and inserted on
                # the preceding iteration.  Finalize the non-empty prefix and
                # train it instead of discarding it because n_steps < 384.
                partial_rollout = True
                self._stop_before_next_rollout = True
                break
            costs = np.asarray(
                [float(info.get("cost", 0.0)) for info in infos], np.float32
            )
            raw_costs = costs.copy()
            self.num_timesteps += env.num_envs
            callback.update_locals(locals())
            if not callback.on_step():
                return False
            self._update_info_buffer(infos, dones)
            self.episode_cost_tracker.observe(raw_costs, dones, infos)
            n_steps += 1

            for info_index, info in enumerate(infos):
                mapping = {
                    "training/reward": rewards[info_index],
                    "training/C_BS": info.get("C_BS", 0.0),
                    "training/L_task": info.get("L_task", 0.0),
                    "training/D_traj": info.get("D_traj", 0.0),
                    "training/D_cov": info.get("D_cov", 0.0),
                    "training/D_red": info.get("D_red", 0.0),
                    "training/D_map": info.get("D_map", 0.0),
                    "training/action_ones": info.get("action_ones", 0.0),
                    "training/relay_action_ones": info.get(
                        "relay_action_ones", 0.0
                    ),
                    "training/upload_action_ones": info.get(
                        "upload_action_ones", 0.0
                    ),
                    "training/coverage": info.get("coverage", 0.0),
                    "training/episode_length": info.get(
                        "episode_length", 0.0
                    ),
                    "qwen/inference_latency": info.get("qwen_latency", 0.0),
                    "qwen/W_task_mean": info.get("qwen_W_task_mean", 0.0),
                    "qwen/omega_sem_entropy": info.get(
                        "qwen_omega_sem_entropy", 0.0
                    ),
                    "qwen/parse_failures": info.get("qwen_parse_failures", 0.0),
                    "qwen/single_gpu_pause_count": info.get(
                        "qwen_single_gpu_pause_count", 0.0
                    ),
                    "qwen/single_gpu_pause_ack_wait_s": info.get(
                        "qwen_single_gpu_pause_ack_wait_s", 0.0
                    ),
                    "constraint/D_traj": info.get("D_traj", 0.0),
                    "constraint/D_cov": info.get("D_cov", 0.0),
                    "constraint/D_red": info.get("D_red", 0.0),
                    "constraint/D_map": info.get("D_map", 0.0),
                    "constraint/L_task": info.get("L_task", 0.0),
                    "constraint/Gamma_task": self.gamma_task,
                    "constraint/value": info.get("constraint_value", 0.0),
                    "constraint/step_cost": info.get("cost", 0.0),
                    "constraint/violation": info.get("constraint_value", 0.0)
                    - self.gamma_task,
                    "constraint/relay_fanout_proposed_max": info.get(
                        "relay_fanout_proposed_max", 0.0
                    ),
                    "constraint/relay_fanout_executed_max": info.get(
                        "relay_fanout_executed_max", 0.0
                    ),
                    "constraint/relay_fanout_violating_senders": info.get(
                        "relay_fanout_violating_senders", 0.0
                    ),
                    "constraint/relay_fanout_projection_drops": info.get(
                        "relay_fanout_projection_drops", 0.0
                    ),
                    "task/coverage": info.get("coverage", 0.0),
                    "task/coverage_pc": info.get("coverage_pc", 0.0),
                    "task/redundancy": info.get("redundancy", 0.0),
                    "task/redundancy_pc": info.get("redundancy_pc", 0.0),
                    "task/map_iou": info.get("map_iou", 0.0),
                    "task/map_iou_pc": info.get("map_iou_pc", 0.0),
                    "task/trajectory_deviation": info.get(
                        "trajectory_deviation_m", 0.0
                    ),
                    "communication/C_BS": info.get("C_BS", 0.0),
                    "communication/C_U2U": info.get("C_U2U", 0.0),
                }
                if dones[info_index]:
                    mapping.update(
                        {
                            "episode/L_task_final": info.get("L_task", 0.0),
                            "episode/sum_CRPO_cost": (
                                self.episode_cost_tracker.records[-1].sum_cost
                                if self.episode_cost_tracker.records
                                else 0.0
                            ),
                            "episode/C_BS_total": (
                                self.episode_cost_tracker.records[-1].total_bs_resource
                                if self.episode_cost_tracker.records
                                else 0.0
                            ),
                            "episode/constraint_mean_step_cost": (
                                self.episode_cost_tracker.records[-1].sum_cost
                                / max(
                                    1,
                                    self.episode_cost_tracker.records[-1].episode_length,
                                )
                                if self.episode_cost_tracker.records
                                else 0.0
                            ),
                        }
                    )
                for name, value in mapping.items():
                    metric_values[name].append(float(value))

            if isinstance(self.action_space, spaces.Discrete):
                actions_np = actions_np.reshape(-1, 1)
            for index, done in enumerate(dones):
                if (
                    done
                    and infos[index].get("terminal_observation") is not None
                    and infos[index].get("TimeLimit.truncated", False)
                ):
                    terminal_obs = self.policy.obs_to_tensor(
                        infos[index]["terminal_observation"]
                    )[0]
                    with th.no_grad():
                        rewards[index] += self.gamma * self.policy.predict_values(
                            terminal_obs
                        )[0]
                        costs[index] += (
                            self.gamma_cost
                            * self.policy.predict_cost_values(terminal_obs)[0]
                        )

            rollout_buffer.add(
                self._last_obs,
                actions_np,
                rewards,
                costs,
                self._last_episode_starts,
                values,
                cost_values,
                log_probs,
            )
            transition_log = {
                "simulation_time_s": float(
                    infos[0].get("simulation_time_s", 0.0)
                ),
                "communication_slot_index": int(
                    infos[0].get("communication_slot_index", 0)
                ),
                "rl_decision_index": int(
                    infos[0].get("rl_decision_index", 0)
                ),
                "action_held_slots": int(
                    infos[0].get("action_held_slots", 0)
                ),
                "rollout_buffer_size": rollout_buffer.valid_size,
                "rollout_buffer_capacity": n_rollout_steps,
                "transition_sim_time_s": float(
                    infos[0].get("transition_sim_time_s", 0.0)
                ),
                "done": bool(dones[0]),
            }
            print(
                "RACER_RL_TRANSITION "
                + json.dumps(transition_log, separators=(",", ":")),
                flush=True,
            )
            self._last_obs = new_obs
            self._last_episode_starts = dones

        with th.no_grad():
            last_obs = obs_as_tensor(new_obs, self.device)
            values = self.policy.predict_values(last_obs)
            cost_values = self.policy.predict_cost_values(last_obs)
        self._last_cost_value_estimate = cost_values.cpu().numpy().flatten()
        rollout_buffer.compute_returns_and_advantage(
            last_values=values,
            last_cost_values=cost_values,
            dones=dones,
        )
        if partial_rollout:
            self.partial_rollout_updates += 1
        self.logger.record("rollout/buffer_size", rollout_buffer.valid_size)
        self.logger.record("rollout/is_partial", int(partial_rollout))
        for name, values_list in metric_values.items():
            if values_list:
                self.logger.record(name, float(np.mean(values_list)))
        callback.update_locals(locals())
        callback.on_rollout_end()
        return True

    def _constraint_estimate(self) -> tuple[float, str]:
        tracker = self.episode_cost_tracker
        if self.constraint_estimator == "mean_step_cost":
            if tracker.records:
                values = [
                    item.sum_cost / max(1, item.episode_length)
                    for item in tracker.records
                ]
                return float(np.mean(values)), "mean_step_cost"
            active = tracker.lengths > 0
            if np.any(active):
                return float(
                    np.mean(tracker.costs[active] / tracker.lengths[active])
                ), "partial_mean_step_cost"
            return 0.0, "partial_mean_step_cost"
        if self.constraint_estimator == "episode_return" and tracker.records:
            return (
                float(np.mean([item.final_task_loss for item in tracker.records])),
                "episode_return",
            )
        estimate = tracker.initial_losses + tracker.costs
        if self._last_cost_value_estimate is not None:
            estimate = estimate + self._last_cost_value_estimate
        return float(np.mean(estimate)), "critic_estimate"

    def _synchronization_status(self) -> dict[str, Any]:
        if self.env is None:
            return {"enabled": False}
        try:
            values = self.env.env_method("synchronization_status")
        except (AttributeError, RuntimeError):
            return {"enabled": False}
        if not values or not isinstance(values[0], dict):
            return {"enabled": False}
        return dict(values[0])

    def train(self) -> None:
        rollout_size = self.rollout_buffer.valid_size
        update_index = self.reward_updates + self.cost_updates + 1
        sync_before = self._synchronization_status()
        update_begin = {
            "update_index": update_index,
            "rollout_buffer_size": rollout_size,
            "rollout_buffer_capacity": self.n_steps,
            "partial_rollout": rollout_size < self.n_steps,
            "simulation_time_s": sync_before.get(
                "telemetry_sim_time_s", sync_before.get("sim_time_s")
            ),
            "communication_slot_index": sync_before.get(
                "telemetry_communication_slot_index",
                sync_before.get("communication_slot_index"),
            ),
            "rl_decision_index": sync_before.get(
                "telemetry_rl_decision_index",
                sync_before.get("rl_decision_index"),
            ),
            "boundary_status": sync_before.get("status", "disabled"),
        }
        print(
            "RACER_PPO_UPDATE_BEGIN "
            + json.dumps(update_begin, separators=(",", ":")),
            flush=True,
        )
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        clip_range = self.clip_range(self._current_progress_remaining)
        clip_range_vf = (
            None
            if self.clip_range_vf is None
            else self.clip_range_vf(self._current_progress_remaining)
        )
        self.j_cost_hat, self.constraint_estimator_source = (
            self._constraint_estimate()
        )
        violation = self.j_cost_hat - (self.gamma_task + self.eta)
        self.crpo_mode = select_crpo_mode(
            self.j_cost_hat, self.gamma_task, self.eta
        )
        if self.crpo_mode == "reward":
            self.reward_updates += 1
        else:
            self.cost_updates += 1

        entropy_losses: list[float] = []
        policy_losses: list[float] = []
        reward_value_losses: list[float] = []
        cost_value_losses: list[float] = []
        clip_fractions: list[float] = []
        approx_kl_divs: list[float] = []
        continue_training = True
        final_loss = th.zeros((), device=self.device)

        for epoch in range(self.n_epochs):
            epoch_kls: list[float] = []
            for rollout_data in self.rollout_buffer.get(self.batch_size):
                actions = rollout_data.actions
                if isinstance(self.action_space, spaces.Discrete):
                    actions = actions.long().flatten()
                reward_values, cost_values, log_prob, entropy = (
                    self.policy.evaluate_actions_crpo(
                        rollout_data.observations, actions
                    )
                )
                reward_values = reward_values.flatten()
                cost_values = cost_values.flatten()
                advantages = (
                    rollout_data.advantages
                    if self.crpo_mode == "reward"
                    else -rollout_data.cost_advantages
                )
                if self.normalize_advantage and len(advantages) > 1:
                    advantages = (advantages - advantages.mean()) / (
                        advantages.std() + 1.0e-8
                    )
                ratio = th.exp(log_prob - rollout_data.old_log_prob)
                objective_1 = advantages * ratio
                objective_2 = advantages * th.clamp(
                    ratio, 1.0 - clip_range, 1.0 + clip_range
                )
                policy_loss = -th.min(objective_1, objective_2).mean()
                policy_losses.append(policy_loss.item())
                clip_fractions.append(
                    th.mean((th.abs(ratio - 1.0) > clip_range).float()).item()
                )

                if clip_range_vf is None:
                    reward_prediction = reward_values
                    cost_prediction = cost_values
                else:
                    reward_prediction = rollout_data.old_values + th.clamp(
                        reward_values - rollout_data.old_values,
                        -clip_range_vf,
                        clip_range_vf,
                    )
                    cost_prediction = rollout_data.old_cost_values + th.clamp(
                        cost_values - rollout_data.old_cost_values,
                        -clip_range_vf,
                        clip_range_vf,
                    )
                reward_value_loss = F.mse_loss(
                    rollout_data.returns, reward_prediction
                )
                cost_value_loss = F.mse_loss(
                    rollout_data.cost_returns, cost_prediction
                )
                reward_value_losses.append(reward_value_loss.item())
                cost_value_losses.append(cost_value_loss.item())
                entropy_loss = (
                    -th.mean(-log_prob)
                    if entropy is None
                    else -th.mean(entropy)
                )
                entropy_losses.append(entropy_loss.item())
                final_loss = (
                    policy_loss
                    + self.ent_coef * entropy_loss
                    + self.vf_coef * reward_value_loss
                    + self.cost_vf_coef * cost_value_loss
                )

                with th.no_grad():
                    log_ratio = log_prob - rollout_data.old_log_prob
                    approx_kl = th.mean(
                        (th.exp(log_ratio) - 1.0) - log_ratio
                    ).item()
                    epoch_kls.append(approx_kl)
                    approx_kl_divs.append(approx_kl)
                if self.target_kl is not None and approx_kl > 1.5 * self.target_kl:
                    continue_training = False
                    break
                self.policy.optimizer.zero_grad()
                final_loss.backward()
                th.nn.utils.clip_grad_norm_(
                    self.policy.parameters(), self.max_grad_norm
                )
                self.policy.optimizer.step()
            self._n_updates += 1
            if not continue_training:
                break

        reward_explained = explained_variance(
            self.rollout_buffer.values.flatten(),
            self.rollout_buffer.returns.flatten(),
        )
        cost_explained = explained_variance(
            self.rollout_buffer.cost_values.flatten(),
            self.rollout_buffer.cost_returns.flatten(),
        )
        mean = lambda values: float(np.mean(values)) if values else 0.0
        self.logger.record("train/entropy_loss", mean(entropy_losses))
        self.logger.record("train/policy_gradient_loss", mean(policy_losses))
        self.logger.record("train/reward_value_loss", mean(reward_value_losses))
        self.logger.record("train/cost_value_loss", mean(cost_value_losses))
        self.logger.record("train/approx_kl", mean(approx_kl_divs))
        self.logger.record("train/clip_fraction", mean(clip_fractions))
        self.logger.record("train/loss", final_loss.item())
        self.logger.record("train/reward_explained_variance", reward_explained)
        self.logger.record("train/cost_explained_variance", cost_explained)
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/clip_range", clip_range)
        self.logger.record("crpo/mode", 0 if self.crpo_mode == "reward" else 1)
        self.logger.record("crpo/mode_name", self.crpo_mode)
        self.logger.record("crpo/J_C_hat", self.j_cost_hat)
        self.logger.record("crpo/Gamma_task", self.gamma_task)
        self.logger.record("constraint/Gamma_task", self.gamma_task)
        self.logger.record("constraint/violation_estimate", violation)
        self.logger.record("crpo/constraint_violation", violation)
        self.logger.record("crpo/reward_updates", self.reward_updates)
        self.logger.record("crpo/cost_updates", self.cost_updates)
        self.logger.record(
            "crpo/estimator_is_episode_return",
            int(self.constraint_estimator_source == "episode_return"),
        )
        self.logger.record(
            "crpo/estimator_is_mean_step_cost",
            int(
                self.constraint_estimator_source
                in {"mean_step_cost", "partial_mean_step_cost"}
            ),
        )
        if self.episode_cost_tracker.records:
            errors = [
                abs(item.telescoping_error)
                for item in self.episode_cost_tracker.records
            ]
            self.logger.record("crpo/telescoping_error_max", max(errors))
        sync_after = self._synchronization_status()
        end_sim_time = sync_after.get(
            "telemetry_sim_time_s", sync_after.get("sim_time_s")
        )
        end_slot = sync_after.get(
            "telemetry_communication_slot_index",
            sync_after.get("communication_slot_index"),
        )
        end_decision = sync_after.get(
            "telemetry_rl_decision_index",
            sync_after.get("rl_decision_index"),
        )
        sync_enabled = bool(sync_before.get("enabled", False))
        simulation_time_frozen = bool(
            sync_enabled
            and update_begin["simulation_time_s"] == end_sim_time
            and update_begin["communication_slot_index"] == end_slot
            and update_begin["rl_decision_index"] == end_decision
            and sync_before.get("status") in {"paused", "terminal"}
            and sync_after.get("status") in {"paused", "terminal"}
        )
        update_end = {
            **update_begin,
            "simulation_time_s_start": update_begin["simulation_time_s"],
            "simulation_time_s_end": end_sim_time,
            "communication_slot_index_end": end_slot,
            "rl_decision_index_end": end_decision,
            "boundary_status_end": sync_after.get("status", "disabled"),
            "simulation_time_frozen": simulation_time_frozen,
        }
        self.sync_update_records.append(update_end)
        self.logger.record(
            "timing/ppo_update_sim_time_start",
            float(update_begin["simulation_time_s"] or 0.0),
        )
        self.logger.record(
            "timing/ppo_update_sim_time_end", float(end_sim_time or 0.0)
        )
        self.logger.record(
            "timing/ppo_update_sim_time_frozen", int(simulation_time_frozen)
        )
        self.logger.record("timing/rollout_buffer_size", rollout_size)
        print(
            "RACER_PPO_UPDATE_END "
            + json.dumps(update_end, separators=(",", ":")),
            flush=True,
        )
