"""Create matching mock/real environments from one experiment config."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .backend import FileBridgeBackend, MockRacerBackend, RacerBackend
from .gym_env import RacerCRPOEnv
from .qwen_global_agent import GuidanceManager, QwenConfig, QwenGlobalAgent
from .relay_fanout import RelayFanoutConstraint
from .single_gpu_pause import SingleGpuPauseConfig, SingleGpuPauseController
from .state_builder import StateNormalization
from .task_loss import TaskLossWeights


def make_backend(config: dict[str, Any], force_mock: bool = False) -> RacerBackend:
    backend = config["environment"]["backend"]
    kind = "mock" if force_mock else str(backend.get("kind", "mock"))
    n = int(config["n_uavs"])
    if kind == "mock":
        return MockRacerBackend(
            n,
            horizon=int(backend.get("horizon", 128)),
            seed=int(config["seed"]),
            direct_u2u_resource=float(
                backend.get("mock_direct_u2u_resource", 11.0)
            ),
        )
    if kind == "file_bridge":
        normalization = config["environment"]["normalization"]
        constraint = config.get("constraint", {})
        synchronous = backend.get("synchronous_online", {})
        fanout_limit = (
            int(constraint["max_relay_recipients_per_uav"])
            if constraint.get("type") == "relay_fanout"
            and bool(constraint.get("hard_enforcement", True))
            else None
        )
        return FileBridgeBackend(
            n,
            action_path=backend["action_path"],
            telemetry_path=backend["telemetry_path"],
            mission_state_path=backend.get("mission_state_path"),
            perfect_reference_result=backend.get("perfect_reference_result"),
            auxiliary_reference_result=backend.get(
                "auxiliary_reference_result"
            ),
            workspace_min=tuple(normalization["position_min"]),
            workspace_max=tuple(normalization["position_max"]),
            redundancy_cell_m=float(backend.get("redundancy_cell_m", 1.0)),
            fixed_exploration_time_s=float(
                backend.get("fixed_exploration_time_s", 300.0)
            ),
            require_auxiliary_reference_metrics=bool(
                backend.get("require_auxiliary_reference_metrics", True)
            ),
            require_mission_state=bool(
                backend.get("require_mission_state", True)
            ),
            require_perfect_reference=bool(
                backend.get("require_perfect_reference", True)
            ),
            max_relay_recipients_per_uav=fanout_limit,
            timeout_s=float(backend.get("timeout_s", 5.0)),
            poll_interval_s=float(backend.get("poll_interval_s", 0.002)),
            synchronous_online=bool(synchronous.get("enabled", False)),
            sync_acknowledgement_path=synchronous.get(
                "acknowledgement_path"
            ),
            sync_release_path=synchronous.get("release_path"),
            communication_slot_s=float(
                synchronous.get("communication_slot_s", 0.02)
            ),
            decision_interval_s=float(
                synchronous.get("decision_interval_s", 0.1)
            ),
            slots_per_decision=int(
                synchronous.get("slots_per_decision", 5)
            ),
        )
    raise ValueError(f"unknown backend kind: {kind}")


def make_env(
    config: dict[str, Any],
    *,
    force_mock: bool = False,
    disable_qwen: bool = False,
) -> RacerCRPOEnv:
    n = int(config["n_uavs"])
    qwen_config = config["qwen"]
    constraint_config = config.get("constraint", {})
    constraint_mode = str(constraint_config.get("type", "task_loss"))
    fanout_limit = (
        int(constraint_config["max_relay_recipients_per_uav"])
        if constraint_mode == "relay_fanout"
        else None
    )
    qwen_enabled = bool(qwen_config.get("enabled", True)) and not disable_qwen
    agent = None
    pause_controller = None
    if qwen_enabled:
        vllm_config = qwen_config.get("vllm", {})
        agent = QwenGlobalAgent(
            QwenConfig(
                model_path=str(qwen_config["model_path"]),
                backend=str(qwen_config.get("backend", "transformers")),
                device=str(qwen_config.get("device", "auto")),
                dtype=str(qwen_config.get("dtype", "bfloat16")),
                local_files_only=bool(qwen_config.get("local_files_only", True)),
                max_new_tokens=int(qwen_config.get("max_new_tokens", 1024)),
                normalize_omega=bool(qwen_config.get("normalize_omega", True)),
                dequantize_fp8=bool(qwen_config.get("dequantize_fp8", True)),
                load_in_8bit=bool(qwen_config.get("load_in_8bit", False)),
                load_in_4bit=bool(qwen_config.get("load_in_4bit", False)),
                prompt_variant=str(
                    qwen_config.get("prompt_variant", "legacy_full_schema")
                ),
                vllm_quantization=str(
                    vllm_config.get("quantization", "fp8")
                ),
                vllm_linear_backend=str(
                    vllm_config.get("linear_backend", "cutlass")
                ),
                vllm_gpu_memory_utilization=float(
                    vllm_config.get("gpu_memory_utilization", 0.25)
                ),
                vllm_max_model_len=int(
                    vllm_config.get("max_model_len", 8192)
                ),
                vllm_max_num_seqs=int(vllm_config.get("max_num_seqs", 1)),
                vllm_enforce_eager=bool(
                    vllm_config.get("enforce_eager", False)
                ),
                vllm_enable_prefix_caching=bool(
                    vllm_config.get("enable_prefix_caching", False)
                ),
                vllm_disable_deep_gemm=bool(
                    vllm_config.get("disable_deep_gemm", True)
                ),
                vllm_disable_flashinfer_sampler=bool(
                    vllm_config.get("disable_flashinfer_sampler", True)
                ),
                vllm_seed=int(vllm_config.get("seed", config["seed"])),
            ),
            n,
            max_relay_recipients_per_uav=fanout_limit,
        )
        pause_config = qwen_config.get("single_gpu_pause", {})
        if bool(pause_config.get("enabled", False)):
            pause_controller = SingleGpuPauseController(
                SingleGpuPauseConfig(
                    request_path=Path(pause_config["request_path"]),
                    acknowledgement_path=Path(
                        pause_config["acknowledgement_path"]
                    ),
                    timeout_s=float(pause_config.get("timeout_s", 600.0)),
                    poll_interval_s=float(
                        pause_config.get("poll_interval_s", 0.01)
                    ),
                )
            )
    manager = GuidanceManager(
        agent,
        n,
        asynchronous=bool(qwen_config.get("async_llm", True)),
        pause_controller=pause_controller,
    )
    environment = config["environment"]
    normalization = environment["normalization"]
    task = config["task_loss"]
    reward = config.get("reward", {})
    fanout = (
        RelayFanoutConstraint(n, fanout_limit)
        if fanout_limit is not None
        else None
    )
    return RacerCRPOEnv(
        make_backend(config, force_mock),
        manager,
        high_level_interval=int(environment["high_level_interval"]),
        channel_history=int(environment["channel_history"]),
        task_weights=TaskLossWeights(
            float(task["alpha_traj"]),
            float(task["alpha_cov"]),
            float(task["alpha_red"]),
            float(task["alpha_map"]),
        ),
        normalization=StateNormalization(
            position_min=tuple(normalization["position_min"]),
            position_max=tuple(normalization["position_max"]),
            snr_min_db=float(normalization["snr_min_db"]),
            snr_max_db=float(normalization["snr_max_db"]),
            max_aoi_s=float(normalization["max_aoi_s"]),
            max_missing_bytes=float(normalization["max_missing_bytes"]),
            max_queue_bytes=float(normalization["max_queue_bytes"]),
        ),
        bs_resource_normalizer=float(environment["bs_resource_normalizer"]),
        include_global_guidance=bool(
            environment.get("include_global_guidance", True)
        ),
        reward_mode=str(reward.get("type", "negative_bs_resource")),
        reward_scale=float(reward.get("scale", 1.0)),
        constraint_mode=constraint_mode,
        relay_fanout=fanout,
        enforce_relay_fanout=bool(
            constraint_config.get("hard_enforcement", True)
        ),
        fanout_priority_weights=constraint_config.get(
            "projection_priority_weights"
        ),
    )
