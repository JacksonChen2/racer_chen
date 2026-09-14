"""Gymnasium environment for the two-timescale scheduling problem."""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .action_mapping import ActionMapping
from .backend import RacerBackend
from .qwen_global_agent import GuidanceManager
from .relay_fanout import RelayFanoutConstraint, RelayFanoutStatus
from .state_builder import LargeStateBuilder, SmallStateBuilder, StateNormalization
from .task_loss import TaskLossTracker, TaskLossWeights


class RacerCRPOEnv(gym.Env[np.ndarray, np.ndarray]):
    metadata = {"render_modes": []}

    def __init__(
        self,
        backend: RacerBackend,
        guidance_manager: GuidanceManager,
        high_level_interval: int,
        channel_history: int,
        task_weights: TaskLossWeights,
        normalization: StateNormalization,
        bs_resource_normalizer: float,
        include_global_guidance: bool = True,
        reward_mode: str = "negative_bs_resource",
        reward_scale: float = 1.0,
        constraint_mode: str = "task_loss",
        relay_fanout: RelayFanoutConstraint | None = None,
        enforce_relay_fanout: bool = True,
        fanout_priority_weights: dict[str, float] | None = None,
    ) -> None:
        super().__init__()
        if (
            high_level_interval < 1
            or bs_resource_normalizer <= 0.0
            or reward_scale <= 0.0
        ):
            raise ValueError("invalid environment interval or resource normalizer")
        if reward_mode not in {
            "negative_bs_resource",
            "coverage",
            "coverage_delta",
        }:
            raise ValueError(f"unsupported reward mode: {reward_mode}")
        if constraint_mode not in {"task_loss", "relay_fanout"}:
            raise ValueError(f"unsupported constraint mode: {constraint_mode}")
        if constraint_mode == "relay_fanout" and relay_fanout is None:
            raise ValueError("relay_fanout constraint mode requires a limit")
        self.backend = backend
        self.n_uavs = backend.n_uavs
        self.guidance_manager = guidance_manager
        self.high_level_interval = high_level_interval
        self.bs_resource_normalizer = float(bs_resource_normalizer)
        self.reward_mode = reward_mode
        self.reward_scale = float(reward_scale)
        self.constraint_mode = constraint_mode
        self.relay_fanout = relay_fanout
        self.enforce_relay_fanout = bool(enforce_relay_fanout)
        weights = {
            "task_dependency": 0.70,
            "relay_queue": 0.15,
            "aoi": 0.10,
            "downlink_snr": 0.05,
        }
        weights.update(fanout_priority_weights or {})
        if set(weights) != {
            "task_dependency",
            "relay_queue",
            "aoi",
            "downlink_snr",
        }:
            raise ValueError("unknown fanout projection priority weight")
        if any(float(value) < 0.0 for value in weights.values()):
            raise ValueError("fanout projection priority weights must be non-negative")
        total_weight = float(sum(weights.values()))
        if total_weight <= 0.0:
            raise ValueError("at least one fanout priority weight must be positive")
        self.fanout_priority_weights = {
            name: float(value) / total_weight for name, value in weights.items()
        }
        self.normalization = normalization
        self.mapping = ActionMapping(self.n_uavs)
        self.small_builder = SmallStateBuilder(
            self.n_uavs,
            normalization,
            include_global_guidance=include_global_guidance,
        )
        self.large_builder = LargeStateBuilder(self.n_uavs, channel_history)
        self.task_tracker = TaskLossTracker(task_weights)
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(self.small_builder.observation_dim,),
            dtype=np.float32,
        )
        self.action_space = spaces.MultiBinary(self.mapping.action_dim)
        self.snapshot = None
        self.initial_task_loss = 0.0
        self.episode_length = 0

    def _relay_priority(self) -> np.ndarray:
        assert self.snapshot is not None
        n = self.n_uavs
        weights = self.fanout_priority_weights
        guidance = self.guidance_manager.current.task_dependency
        relay_queue = np.clip(
            np.log1p(np.maximum(self.snapshot.relay_queue_bytes, 0.0))
            / np.log1p(max(self.normalization.max_queue_bytes, 1.0)),
            0.0,
            1.0,
        )
        aoi = np.clip(
            self.snapshot.pair_aoi_s
            / max(self.normalization.max_aoi_s, 1.0e-6),
            0.0,
            1.0,
        )
        downlink = np.clip(
            (
                self.snapshot.channel_snr_db[n, :n]
                - self.normalization.snr_min_db
            )
            / max(
                self.normalization.snr_max_db
                - self.normalization.snr_min_db,
                1.0e-6,
            ),
            0.0,
            1.0,
        )
        score = (
            weights["task_dependency"] * guidance
            + weights["relay_queue"] * relay_queue
            + weights["aoi"] * aoi
            + weights["downlink_snr"] * downlink[None, :]
        )
        np.fill_diagonal(score, 0.0)
        return score

    def _reward(self, result: Any) -> float:
        if self.reward_mode == "negative_bs_resource":
            value = -result.bs_resource / self.bs_resource_normalizer
        elif self.reward_mode == "coverage":
            value = self.snapshot.coverage
        else:
            value = self.snapshot.coverage_delta
        return float(self.reward_scale * value)

    def _observation(self) -> np.ndarray:
        assert self.snapshot is not None
        return self.small_builder.build(
            self.snapshot,
            self.guidance_manager.current,
        )

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        del options
        self.snapshot = self.backend.reset(seed)
        self.large_builder.reset()
        self.episode_length = 0
        self.initial_task_loss = self.task_tracker.reset(self.snapshot.task_metrics)
        if (
            self.guidance_manager.enabled
            and not (self.snapshot.terminated or self.snapshot.truncated)
        ):
            self.guidance_manager.request_update(
                self.large_builder.build(self.snapshot)
            )
        return self._observation(), {
            "L_task": self.initial_task_loss,
            "L_task_initial": self.initial_task_loss,
            "constraint_value": (
                self.initial_task_loss
                if self.constraint_mode == "task_loss"
                else 0.0
            ),
            "constraint_mode": self.constraint_mode,
            "reward_mode": self.reward_mode,
            "qwen_epoch": self.guidance_manager.epoch,
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        assert self.snapshot is not None
        transition_start_sim_time_s = self.snapshot.sim_time_s
        transition_start_slot = int(self.snapshot.communication_slot_index)
        transition_start_decision = int(self.snapshot.rl_decision_index)
        proposed_matrix = self.mapping.decode(action)
        proposed_fanout: RelayFanoutStatus | None = None
        executed_fanout: RelayFanoutStatus | None = None
        matrix = proposed_matrix
        if self.relay_fanout is not None:
            proposed_fanout = self.relay_fanout.evaluate(proposed_matrix)
            if self.enforce_relay_fanout:
                matrix = self.relay_fanout.project(
                    proposed_matrix, self._relay_priority()
                )
            executed_fanout = self.relay_fanout.evaluate(matrix)
            if self.enforce_relay_fanout:
                self.relay_fanout.validate_executed(matrix)
        result = self.backend.step(matrix)
        self.episode_length += 1
        self.snapshot = result.snapshot
        task_loss, task_cost = self.task_tracker.update(
            self.snapshot.task_metrics
        )
        if self.constraint_mode == "task_loss":
            cost = task_cost
            constraint_value = task_loss
        else:
            assert proposed_fanout is not None
            cost = proposed_fanout.cost
            constraint_value = proposed_fanout.cost
        request_guidance = (
            self.guidance_manager.enabled
            and
            not (self.snapshot.terminated or self.snapshot.truncated)
            and self.snapshot.slot % self.high_level_interval == 0
        )
        if request_guidance:
            self.guidance_manager.request_update(
                self.large_builder.build(self.snapshot)
            )
        elif (
            self.guidance_manager.enabled
            and not (self.snapshot.terminated or self.snapshot.truncated)
        ):
            self.large_builder.observe(self.snapshot)
        reward = self._reward(result)
        relay_ones = int(matrix[: self.n_uavs].sum())
        upload_ones = int(matrix[self.n_uavs].sum())
        guidance = self.guidance_manager.current
        omega = np.maximum(guidance.semantic_importance, 1.0e-12)
        omega_entropy = float(-np.sum(omega * np.log(omega)))
        metrics = self.snapshot.task_metrics
        info = {
            "cost": float(cost),
            "constraint_value": float(constraint_value),
            "constraint_mode": self.constraint_mode,
            "reward_mode": self.reward_mode,
            "L_task": float(task_loss),
            "L_task_initial": float(self.initial_task_loss),
            "D_traj": metrics.d_traj,
            "D_cov": metrics.d_cov,
            "D_red": metrics.d_red,
            "D_map": metrics.d_map,
            "C_BS": result.bs_resource,
            "C_BS_ul": float(result.bs_ul_resource),
            "C_BS_dl": float(result.bs_dl_resource),
            "C_U2U": float(result.direct_u2u_resource),
            "coverage": self.snapshot.coverage,
            "coverage_pc": metrics.coverage_pc,
            "redundancy": metrics.redundancy,
            "redundancy_pc": metrics.redundancy_pc,
            "map_iou": metrics.map_iou,
            "map_iou_pc": metrics.map_iou_pc,
            "trajectory_deviation_m": metrics.trajectory_deviation_m,
            "episode_length": self.episode_length,
            "qwen_epoch": self.guidance_manager.epoch,
            "qwen_latency": self.guidance_manager.latency_s,
            "qwen_parse_failures": self.guidance_manager.failures,
            "qwen_single_gpu_pause_count": (
                self.guidance_manager.single_gpu_pause_count
            ),
            "qwen_single_gpu_pause_ack_wait_s": (
                self.guidance_manager.single_gpu_pause_ack_wait_s
            ),
            "qwen_W_task_mean": float(guidance.task_dependency.mean()),
            "qwen_omega_sem_entropy": omega_entropy,
            "crpo_action_matrix": matrix.copy(),
            "crpo_proposed_action_matrix": proposed_matrix.copy(),
            "action_ones": relay_ones + upload_ones,
            "relay_action_ones": relay_ones,
            "upload_action_ones": upload_ones,
            "task_cost_telescoping_error": self.task_tracker.telescoping_error,
            "transition_start_sim_time_s": transition_start_sim_time_s,
            "simulation_time_s": self.snapshot.sim_time_s,
            "transition_sim_time_s": (
                self.snapshot.sim_time_s - transition_start_sim_time_s
            ),
            "transition_start_communication_slot_index": transition_start_slot,
            "communication_slot_index": int(
                self.snapshot.communication_slot_index
            ),
            "transition_start_rl_decision_index": transition_start_decision,
            "rl_decision_index": int(self.snapshot.rl_decision_index),
            "action_held_slots": int(self.snapshot.action_held_slots),
            "communication_slot_duration_s": (
                self.snapshot.communication_slot_duration_s
            ),
            "rl_decision_interval_s": self.snapshot.rl_decision_interval_s,
            "state_sequence": int(self.snapshot.state_sequence),
        }
        if proposed_fanout is not None and executed_fanout is not None:
            info.update(
                {
                    "relay_fanout_limit": self.relay_fanout.max_recipients,
                    "relay_fanout_proposed_counts": proposed_fanout.counts.copy(),
                    "relay_fanout_executed_counts": executed_fanout.counts.copy(),
                    "relay_fanout_proposed_max": proposed_fanout.max_count,
                    "relay_fanout_executed_max": executed_fanout.max_count,
                    "relay_fanout_violating_senders": (
                        proposed_fanout.violating_senders
                    ),
                    "relay_fanout_projection_drops": int(
                        proposed_fanout.counts.sum()
                        - executed_fanout.counts.sum()
                    ),
                    "relay_fanout_hard_enforced": self.enforce_relay_fanout,
                }
            )
        return (
            self._observation(),
            float(reward),
            self.snapshot.terminated,
            self.snapshot.truncated,
            info,
        )

    def close(self) -> None:
        self.guidance_manager.close()
        self.backend.close()

    def synchronization_status(self) -> dict[str, Any]:
        return self.backend.synchronization_status()
