from types import SimpleNamespace

import pytest
from torch import nn

from agentic_crpo.train_crpo import (
    _activation_class,
    _configure_rl_pytorch_threads,
    _learn_until_target_or_mission_end,
    _mission_has_ended,
    _prepare_resume_for_new_episode,
)


def test_rl_pytorch_threads_are_fixed_to_one(monkeypatch, capsys) -> None:
    counts = {"intra": 8, "inter": 4}

    monkeypatch.setattr(
        "agentic_crpo.train_crpo.torch.set_num_threads",
        lambda count: counts.__setitem__("intra", count),
    )
    monkeypatch.setattr(
        "agentic_crpo.train_crpo.torch.get_num_threads",
        lambda: counts["intra"],
    )
    monkeypatch.setattr(
        "agentic_crpo.train_crpo.torch.set_num_interop_threads",
        lambda count: counts.__setitem__("inter", count),
    )
    monkeypatch.setattr(
        "agentic_crpo.train_crpo.torch.get_num_interop_threads",
        lambda: counts["inter"],
    )

    _configure_rl_pytorch_threads()

    assert counts == {"intra": 1, "inter": 1}
    output = capsys.readouterr().out
    assert '"intra_op_threads":1' in output
    assert '"inter_op_threads":1' in output


class _TimeoutModel:
    def __init__(self, *, progress: int) -> None:
        self.num_timesteps = 10
        self.progress = progress

    def learn(self, **_kwargs) -> None:
        self.num_timesteps += self.progress
        raise TimeoutError("telemetry stopped")


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("silu", nn.SiLU),
        ("tanh", nn.Tanh),
        ("ReLU", nn.ReLU),
        ("gelu", nn.GELU),
    ],
)
def test_activation_function_selection(name: str, expected: type[nn.Module]) -> None:
    assert _activation_class(name) is expected


def test_unknown_activation_function_is_rejected() -> None:
    with pytest.raises(ValueError, match="unsupported PPO activation_fn"):
        _activation_class("swish")


def test_terminal_mission_file_covers_stale_in_memory_snapshot(tmp_path) -> None:
    mission_path = tmp_path / "mission_state.json"
    mission_path.write_text('{"terminated": false, "truncated": true}')
    env = SimpleNamespace(
        snapshot=SimpleNamespace(terminated=False, truncated=False),
        backend=SimpleNamespace(mission_state_path=mission_path),
    )

    assert _mission_has_ended(env) is True


def test_terminal_mission_timeout_preserves_completed_training() -> None:
    model = _TimeoutModel(progress=8)
    env = SimpleNamespace(
        snapshot=SimpleNamespace(terminated=False, truncated=True)
    )

    ended = _learn_until_target_or_mission_end(
        model,
        env,
        total_timesteps=100,
        callback=object(),
        reset_num_timesteps=False,
    )

    assert ended is True
    assert model.num_timesteps == 18


@pytest.mark.parametrize(
    ("progress", "terminated", "truncated"),
    [(0, False, True), (8, False, False)],
)
def test_nonterminal_or_no_progress_timeout_is_not_suppressed(
    progress: int, terminated: bool, truncated: bool
) -> None:
    model = _TimeoutModel(progress=progress)
    env = SimpleNamespace(
        snapshot=SimpleNamespace(
            terminated=terminated,
            truncated=truncated,
        )
    )

    with pytest.raises(TimeoutError, match="telemetry stopped"):
        _learn_until_target_or_mission_end(
            model,
            env,
            total_timesteps=100,
            callback=object(),
            reset_num_timesteps=False,
        )


def test_resume_discards_terminal_runtime_state_but_keeps_training_counts() -> None:
    model = SimpleNamespace(
        ended_on_mission_boundary=True,
        _stop_before_next_rollout=True,
        _last_obs="terminal observation",
        _last_original_obs="terminal original observation",
        _last_episode_starts=[False],
        num_timesteps=3000,
        reward_updates=8,
    )
    _prepare_resume_for_new_episode(model)
    assert model.ended_on_mission_boundary is False
    assert model._stop_before_next_rollout is False
    assert model._last_obs is None
    assert model._last_original_obs is None
    assert model._last_episode_starts is None
    assert model.num_timesteps == 3000
    assert model.reward_updates == 8
