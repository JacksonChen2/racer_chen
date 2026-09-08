import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from agentic_crpo.qwen_global_agent import (
    QwenConfig,
    QwenGlobalAgent,
    parse_guidance,
)
from agentic_crpo.config import load_config
from agentic_crpo.schemas import GlobalGuidance


def test_qwen_ranked_links_expand_and_bs_priorities_normalize():
    text = """UAV links: 1:2,3 2:3,1 3:1,2
    BS priority: 1:4 2:1 3:2"""
    guidance = parse_guidance(text, 3)
    np.testing.assert_allclose(np.diag(guidance.task_dependency), 0.0)
    assert np.all((0 <= guidance.task_dependency) & (guidance.task_dependency <= 1))
    assert guidance.task_dependency[0, 1] == 1.0
    assert guidance.task_dependency[0, 2] == 0.5
    np.testing.assert_allclose(guidance.semantic_importance.sum(), 1.0)
    np.testing.assert_allclose(guidance.semantic_importance, [4 / 7, 1 / 7, 2 / 7])


def test_qwen_output_rejects_missing_sources_self_links_and_extra_text():
    with pytest.raises(ValueError, match="every source UAV"):
        parse_guidance(
            "UAV links: 1:2,3 2:3,1\nBS priority: 1:3 2:1 3:2", 3
        )
    with pytest.raises(ValueError, match="self-links"):
        parse_guidance(
            "UAV links: 1:1,2 2:3,1 3:1,2\nBS priority: 1:3 2:1 3:2", 3
        )
    with pytest.raises(ValueError, match="exactly UAV links"):
        parse_guidance(
            "explanation\nUAV links: 1:2 2:1\nBS priority: 1:3 2:1", 2
        )
    with pytest.raises(ValueError, match="0 to 4"):
        parse_guidance(
            "UAV links: 1:2 2:1\nBS priority: 1:5 2:1", 2
        )


def test_async_guidance_close_waits_for_inflight_generation():
    class FakeAgent:
        inference_latency_s = 0.01

        def infer(self, state):
            del state
            time.sleep(0.01)
            return GlobalGuidance.neutral(2)

    from agentic_crpo.qwen_global_agent import GuidanceManager

    manager = GuidanceManager(FakeAgent(), 2, asynchronous=True)
    assert manager.request_update({})
    manager.close()
    assert manager.epoch == 1


def test_latest_qwen8_fp8_config_and_compact_prompt():
    config = load_config(
        Path(__file__).parents[2] / "config" / "qwen8b_fp8_latest.yaml"
    )
    assert config["qwen"]["model_path"].endswith("Qwen3-8B-FP8")
    assert config["qwen"]["backend"] == "vllm"
    assert config["qwen"]["dequantize_fp8"] is False
    assert config["qwen"]["vllm"]["linear_backend"] == "cutlass"
    agent = QwenGlobalAgent(
        QwenConfig(
            model_path=config["qwen"]["model_path"],
            prompt_variant=config["qwen"]["prompt_variant"],
        ),
        n_uavs=10,
    )
    prompt = agent._prompt({})
    assert "BS priority 不允许全部为 0" in prompt
    assert "每架源 UAV 输出 top-4" in prompt
    assert "该源 UAV 的信息利用基站转发给各接收方 UAV 的任务重要性" in prompt
    assert "UAV links: 1:2,3,4,5 2:3,4,5,6" in prompt
    assert "BS priority: 1:4 2:3 3:2 4:1 5:0" in prompt
    for input_key in (
        "bs_map_summary",
        "bs_exploration_progress",
        "uav_bs_missing_bytes",
        "bs_uav_missing_bytes",
        "coarse_communication_summary",
        "peer_aoi_summary",
        "bs_aoi_s",
    ):
        assert input_key in prompt
    for forbidden_key in (
        "U2U_SNR_dB",
        "UAV_BS_SNR_dB",
        "BS_UAV_SNR_dB",
        "channel_outage_ratio",
        "pairwise_aoi_s",
        "information_version_gap",
    ):
        assert forbidden_key not in prompt
    assert "velocities" not in prompt
    assert "headings_cos_sin" not in prompt


def test_vllm_fp8_backend_uses_cutlass_and_native_sampler(
    tmp_path, monkeypatch
):
    model_path = tmp_path / "Qwen3-8B-FP8"
    model_path.mkdir()
    (model_path / "config.json").write_text(
        '{"quantization_config":{"quant_method":"fp8"}}',
        encoding="utf-8",
    )
    monkeypatch.delenv("VLLM_USE_DEEP_GEMM", raising=False)
    monkeypatch.delenv("VLLM_USE_FLASHINFER_SAMPLER", raising=False)

    class FakeTokenizer:
        def apply_chat_template(self, messages, **kwargs):
            assert messages[0]["role"] == "user"
            assert kwargs["enable_thinking"] is False
            return "rendered prompt"

    class FakeEngineCore:
        def __init__(self):
            self.shutdown_called = False

        def shutdown(self):
            self.shutdown_called = True

    class FakeLLM:
        instances = []

        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.tokenizer = FakeTokenizer()
            self.llm_engine = SimpleNamespace(engine_core=FakeEngineCore())
            self.generate_args = None
            self.__class__.instances.append(self)

        def get_tokenizer(self):
            return self.tokenizer

        def generate(self, prompts, sampling_params, use_tqdm):
            self.generate_args = (prompts, sampling_params, use_tqdm)
            text = (
                "UAV links: 1:2 2:1\n"
                "BS priority: 1:4 2:1"
            )
            return [SimpleNamespace(outputs=[SimpleNamespace(text=text)])]

    class FakeSamplingParams:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    monkeypatch.setitem(
        sys.modules,
        "vllm",
        SimpleNamespace(LLM=FakeLLM, SamplingParams=FakeSamplingParams),
    )
    agent = QwenGlobalAgent(
        QwenConfig(
            model_path=str(model_path),
            backend="vllm",
            dequantize_fp8=False,
            max_new_tokens=128,
            vllm_linear_backend="cutlass",
            vllm_max_model_len=4096,
            vllm_max_num_seqs=1,
            vllm_seed=42,
        ),
        n_uavs=2,
    )

    guidance = agent.infer({})
    instance = FakeLLM.instances[0]
    assert instance.kwargs["quantization"] == "fp8"
    assert instance.kwargs["linear_backend"] == "cutlass"
    assert instance.kwargs["max_num_seqs"] == 1
    assert instance.kwargs["seed"] == 42
    assert os.environ["VLLM_USE_DEEP_GEMM"] == "0"
    assert os.environ["VLLM_USE_FLASHINFER_SAMPLER"] == "0"
    assert instance.generate_args[2] is False
    assert instance.generate_args[1].kwargs["temperature"] == 0.0
    assert instance.generate_args[1].kwargs["max_tokens"] == 128
    assert guidance.task_dependency.shape == (2, 2)

    engine_core = instance.llm_engine.engine_core
    agent.close()
    assert engine_core.shutdown_called


def test_vllm_backend_rejects_transformers_dequantization(tmp_path):
    model_path = tmp_path / "Qwen3-8B-FP8"
    model_path.mkdir()
    (model_path / "config.json").write_text("{}", encoding="utf-8")
    with pytest.raises(ValueError, match="dequantize_fp8 must be false"):
        QwenConfig(model_path=str(model_path), backend="vllm").validate()
