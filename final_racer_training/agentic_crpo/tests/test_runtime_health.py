import os
import signal
from types import SimpleNamespace

from agentic_crpo.process_supervisor import _signal_group
from agentic_crpo.runtime_health import RuntimeHealth


def blocks(pid=None):
    result = {name: SimpleNamespace(version=0)
              for name in ("physical", "transition_ring", "action")}
    result["status_racer"] = SimpleNamespace(
        snapshot_json=lambda: None if pid is None else (None, {"pid": pid}))
    return result


def test_nested_proxy_death_is_detected_without_launch_exit():
    data = blocks(2147483647)
    assert RuntimeHealth(0).check(data, 1).startswith("communication_proxy_dead")


def test_live_proxy_but_no_states_times_out():
    data = blocks(os.getpid())
    data["physical"].version = 100
    health = RuntimeHealth(0)
    assert health.check(data, 179) is None
    assert health.check(data, 181) == "startup_timeout_transition_ring"


def test_model_warmup_then_stalled_actions():
    data = blocks(os.getpid())
    health = RuntimeHealth(0)
    data["physical"].version = data["transition_ring"].version = 1
    assert health.check(data, 1) is None
    assert health.check(data, 100) is None
    data["action"].version = 1
    data["physical"].version = data["transition_ring"].version = 2
    assert health.check(data, 101) is None
    data["physical"].version = data["transition_ring"].version = 100
    assert health.check(data, 162) == "progress_timeout_action"


def test_signal_group_reaps_descendants_after_leader_exit(monkeypatch):
    calls = []
    child = SimpleNamespace(
        process=SimpleNamespace(pid=12345, poll=lambda: 0)
    )
    monkeypatch.setattr(
        os, "killpg", lambda process_group, sig: calls.append((process_group, sig))
    )

    _signal_group(child, signal.SIGKILL)

    assert calls == [(12345, signal.SIGKILL)]
