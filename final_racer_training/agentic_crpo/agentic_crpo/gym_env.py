"""Gymnasium environment for the two-timescale scheduling problem."""

from __future__ import annotations

from collections import OrderedDict
import threading
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from .action_mapping import ActionMapping
from .backend import RacerBackend
from .qwen_global_agent import GuidanceManager
from .schemas import GlobalGuidance, TaskMetrics
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
        self.observation_guidance = self.guidance_manager.current
        self.observation_guidance_id = int(self.guidance_manager.epoch)
        self.policy_version = 0
        self.initial_task_loss = 0.0
        self.episode_length = 0
        self.task_tracker_initialized = False
        self.initial_metric_step_id = 0
        self._resolved_costs: dict[int, dict[str, Any]] = {}
        self._published_action_contexts: OrderedDict[
            int, tuple[np.ndarray, tuple[Any, ...]]
        ] = OrderedDict()
        self._action_context_lock = threading.Lock()

    def _relay_priority(self) -> np.ndarray:
        assert self.snapshot is not None
        n = self.n_uavs
        weights = self.fanout_priority_weights
        guidance = self.observation_guidance.task_dependency
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
        if (
            self.snapshot.guidance_id > 0
            and self.snapshot.guidance_task_dependency is not None
            and self.snapshot.guidance_semantic_importance is not None
        ):
            # Fast State embeds the fixed-size guidance arrays. Refresh the
            # low-frequency JSON facade only when its version changes so its
            # latency/source diagnostics stay current without entering the
            # 10 Hz observation hot path.
            if self.guidance_manager.epoch < self.snapshot.guidance_id:
                _ = self.guidance_manager.current
            self.observation_guidance = GlobalGuidance.validated(
                self.snapshot.guidance_task_dependency,
                self.snapshot.guidance_semantic_importance,
                self.n_uavs,
            )
            self.observation_guidance_id = int(self.snapshot.guidance_id)
        else:
            self.observation_guidance = self.guidance_manager.current
            self.observation_guidance_id = int(self.guidance_manager.epoch)
        return self.small_builder.build(
            self.snapshot,
            self.observation_guidance,
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
        with self._action_context_lock:
            self._published_action_contexts.clear()
        self.large_builder.reset()
        self.episode_length = 0
        self.task_tracker_initialized = False
        self.initial_metric_step_id = int(self.snapshot.step_id)
        self._resolved_costs.clear()
        externally_driven = bool(
            getattr(self.guidance_manager, "externally_driven", False)
        )
        if externally_driven and self.guidance_manager.enabled:
            self.guidance_manager.wait_until_ready()
            refreshed = self.backend.refresh_latest_snapshot()
            if refreshed is not None:
                self.snapshot = refreshed
        if self.backend.task_metrics_are_async:
            # Cost L_0 is resolved together with the first rollout batch. The
            # actor never waits for map/task work during reset or step().
            self.initial_task_loss = 0.0
        else:
            self.initial_task_loss = self.task_tracker.reset(
                self.snapshot.task_metrics
            )
            self.task_tracker_initialized = True
        if (
            self.guidance_manager.enabled
            and not externally_driven
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

    def supports_decoupled_transition_collection(self) -> bool:
        return bool(self.backend.decoupled_transition_collection)

    def _prepare_action(self, action: np.ndarray) -> tuple[Any, ...]:
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
        return (
            transition_start_sim_time_s,
            transition_start_slot,
            transition_start_decision,
            proposed_matrix,
            proposed_fanout,
            executed_fanout,
            matrix,
        )

    def publish_action(
        self,
        action: np.ndarray,
        policy_version: int,
        source_context: dict[str, Any] | None = None,
    ) -> dict[str, int | float]:
        """Publish only; the inference worker never waits for next state."""

        if not self.backend.decoupled_transition_collection:
            raise RuntimeError("backend does not support decoupled collection")
        prepared = self._prepare_action(action)
        if source_context is not None:
            prepared = (
                float(
                    source_context.get("sim_time_s", prepared[0])
                ),
                int(
                    source_context.get(
                        "communication_slot_index", prepared[1]
                    )
                ),
                int(
                    source_context.get(
                        "rl_decision_index", prepared[2]
                    )
                ),
                *prepared[3:],
            )
        matrix = prepared[-1]
        self.policy_version = max(0, int(policy_version))
        self.backend.set_action_context(
            guidance_id=self.observation_guidance_id,
            policy_version=self.policy_version,
        )
        action_version = self.backend.publish_action(matrix, source_context)
        with self._action_context_lock:
            self._published_action_contexts[int(action_version)] = (
                np.asarray(action).copy(),
                prepared,
            )
            # The runtime writes one action per decision.  This comfortably
            # spans a full 300 s episode while bounding a pathological held
            # action run; entries are retained because one version can govern
            # several zero-order-hold transitions.
            while len(self._published_action_contexts) > 4096:
                self._published_action_contexts.popitem(last=False)
        return {
            "action_version": int(action_version),
            "channel_version": int(
                (source_context or {}).get(
                    "channel_version",
                    (source_context or {}).get(
                        "consumed_communication_version",
                        self.snapshot.state_sequence,
                    ),
                )
            ),
            "step_id": int(prepared[2]),
            "sim_timestamp": float(prepared[0]),
        }

    def step(
        self, action: np.ndarray
    ) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        if not self.backend.decoupled_transition_collection:
            prepared = self._prepare_action(action)
            self.backend.set_action_context(
                guidance_id=self.observation_guidance_id,
                policy_version=self.policy_version,
            )
            result = self.backend.step(prepared[-1])
        else:
            # Transition collection is driven solely by the ordered 100 ms
            # boundary ring.  It neither waits for nor owns a policy result.
            # A short lookup after the boundary supplies diagnostics for the
            # action that the Proxy says was actually active.
            fallback_prepared = self._prepare_action(action)
            result = self.backend.collect_transition()
            with self._action_context_lock:
                published = self._published_action_contexts.get(
                    int(result.action_version)
                )
            prepared = fallback_prepared
            if published is not None:
                # The action's source state can precede the interval in which
                # zero-order hold executes it.  Keep boundary timing from the
                # collector and use only the matching action diagnostics from
                # the DecisionRecord context.
                prepared = (*fallback_prepared[:3], *published[1][3:])
        (
            transition_start_sim_time_s,
            transition_start_slot,
            transition_start_decision,
            proposed_matrix,
            proposed_fanout,
            executed_fanout,
            matrix,
        ) = prepared
        executed_matrix = (
            matrix
            if result.executed_action_matrix is None
            else np.asarray(result.executed_action_matrix, dtype=np.int8)
        )
        if self.relay_fanout is not None:
            executed_fanout = self.relay_fanout.evaluate(executed_matrix)
            if self.enforce_relay_fanout:
                self.relay_fanout.validate_executed(executed_matrix)
        self.episode_length += 1
        self.snapshot = result.snapshot
        cost_pending = bool(
            self.backend.task_metrics_are_async
            and self.constraint_mode == "task_loss"
        )
        if cost_pending:
            task_loss = self.task_tracker.previous
            cost = 0.0
            constraint_value = task_loss
        else:
            task_loss, task_cost = self.task_tracker.update(
                self.snapshot.task_metrics
            )
            if self.constraint_mode == "task_loss":
                cost = task_cost
                constraint_value = task_loss
            else:
                assert executed_fanout is not None
                cost = executed_fanout.cost
                constraint_value = executed_fanout.cost
        externally_driven = bool(
            getattr(self.guidance_manager, "externally_driven", False)
        )
        request_guidance = (
            self.guidance_manager.enabled
            and not externally_driven
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
            and not externally_driven
            and not (self.snapshot.terminated or self.snapshot.truncated)
        ):
            self.large_builder.observe(self.snapshot)
        reward = self._reward(result)
        relay_ones = int(executed_matrix[: self.n_uavs].sum())
        upload_ones = int(executed_matrix[self.n_uavs].sum())
        guidance = self.observation_guidance
        omega = np.maximum(guidance.semantic_importance, 1.0e-12)
        omega_entropy = float(-np.sum(omega * np.log(omega)))
        metrics = self.snapshot.task_metrics
        info = {
            "cost": float(cost),
            "cost_pending": cost_pending,
            "cost_step_id": int(result.transition_step_id),
            "metric_state_step_id": int(self.snapshot.step_id),
            "constraint_value": float(constraint_value),
            "constraint_mode": self.constraint_mode,
            "reward_mode": self.reward_mode,
            "L_task": float(task_loss),
            "L_task_initial": float(self.initial_task_loss),
            "D_traj": metrics.d_traj,
            "D_cov": metrics.d_cov,
            "D_red": metrics.d_red,
            "D_map": metrics.d_map,
            "C_joint_map": metrics.joint_coverage,
            "C_BS_map": metrics.bs_coverage,
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
            "bs_global_map_coverage": metrics.bs_coverage,
            "trajectory_deviation_m": metrics.trajectory_deviation_m,
            "task_step": metrics.task_step,
            "task_time_s": metrics.task_time_s,
            "episode_length": self.episode_length,
            "qwen_epoch": self.guidance_manager.epoch,
            "action_source_guidance_id": int(
                result.action_source_guidance_id
            ),
            "action_source_step_id": int(result.action_source_step_id),
            "action_source_sim_time_s": float(
                result.action_source_sim_time_s
            ),
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
            "crpo_action_matrix": executed_matrix.copy(),
            "executed_action": self.mapping.encode(executed_matrix),
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
            "action_version": int(result.action_version),
            "policy_version": int(result.policy_version),
            "action_age": float(result.action_age),
            "sim_timestamp": float(result.sim_timestamp),
            "communication_slot_duration_s": (
                self.snapshot.communication_slot_duration_s
            ),
            "rl_decision_interval_s": self.snapshot.rl_decision_interval_s,
            "state_sequence": int(self.snapshot.state_sequence),
            "step_id": int(result.transition_step_id),
            "next_state_step_id": int(self.snapshot.step_id),
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

    @staticmethod
    def _metric_info(metrics: TaskMetrics) -> dict[str, Any]:
        return {
            "D_traj": metrics.d_traj,
            "D_cov": metrics.d_cov,
            "D_red": metrics.d_red,
            "D_map": metrics.d_map,
            "C_joint_map": metrics.joint_coverage,
            "C_BS_map": metrics.bs_coverage,
            "coverage": metrics.coverage,
            "coverage_pc": metrics.coverage_pc,
            "redundancy": metrics.redundancy,
            "redundancy_pc": metrics.redundancy_pc,
            "map_iou": metrics.map_iou,
            "map_iou_pc": metrics.map_iou_pc,
            "bs_global_map_coverage": metrics.bs_coverage,
            "trajectory_deviation_m": metrics.trajectory_deviation_m,
            "task_step": metrics.task_step,
            "task_time_s": metrics.task_time_s,
        }

    def resolve_pending_costs(
        self, step_ids: list[int]
    ) -> list[dict[str, Any]]:
        """Backfill c_k after collection, immediately before CRPO GAE/update."""

        requested = [int(value) for value in step_ids]
        unresolved = [
            value for value in requested if value not in self._resolved_costs
        ]
        metric_steps = [value + 1 for value in unresolved]
        if not self.task_tracker_initialized:
            metric_steps.insert(0, self.initial_metric_step_id)
        metrics = self.backend.resolve_task_metrics(metric_steps)
        if not self.task_tracker_initialized:
            initial = metrics[self.initial_metric_step_id]
            self.initial_task_loss = self.task_tracker.reset(initial)
            self.task_tracker_initialized = True
        for step_id in unresolved:
            metric = metrics[step_id + 1]
            task_loss, cost = self.task_tracker.update(metric)
            value = {
                "cost": float(cost),
                "constraint_value": float(task_loss),
                "L_task": float(task_loss),
                "L_task_initial": float(self.initial_task_loss),
                "cost_pending": False,
                "cost_step_id": step_id,
                "task_cost_telescoping_error": (
                    self.task_tracker.telescoping_error
                ),
                **self._metric_info(metric),
            }
            self._resolved_costs[step_id] = value
        return [dict(self._resolved_costs[value]) for value in requested]

    def refresh_latest_observation(self) -> np.ndarray:
        """Use current shared state after PPO while preserving skipped effects."""

        refreshed = self.backend.refresh_latest_snapshot()
        if refreshed is not None:
            self.snapshot = refreshed
        return self._observation()

    def cycle_status(self) -> dict[str, Any]:
        status = self.backend.synchronization_status()
        # This counter labels local inference attempts only. Simulation time
        # and transition-ring sequence are the authoritative timesteps.
        status["rl_cycle_id"] = self.episode_length + 1
        status["guidance_id"] = self.observation_guidance_id
        return status

    def set_policy_version(self, policy_version: int) -> None:
        self.policy_version = max(0, int(policy_version))

    def close(self) -> None:
        self.guidance_manager.close()
        self.backend.close()

    def synchronization_status(self) -> dict[str, Any]:
        return self.backend.synchronization_status()
