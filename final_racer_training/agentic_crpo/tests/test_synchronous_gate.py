import json
from pathlib import Path
import sys
import threading
import time


ISAAC_SIM = (
    Path(__file__).resolve().parents[2]
    / "training_overlay_ws"
    / "src"
    / "racer_isaac_adapter"
    / "isaac_sim"
)
sys.path.insert(0, str(ISAAC_SIM))

from synchronous_rl_gate import SynchronousRlBoundaryGate


def _write(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    temporary.replace(path)


def test_gate_freezes_each_exact_simulation_boundary_until_matching_action(tmp_path):
    state = tmp_path / "state.json"
    action = tmp_path / "action.txt"
    release = tmp_path / "release.json"
    ack = tmp_path / "ack.json"
    gate = SynchronousRlBoundaryGate(
        state,
        action,
        release,
        ack,
        maximum_wait_s=2.0,
        poll_interval_s=0.001,
    )
    _write(
        state,
        {
            "sequence": 9,
            "rl_decision_index": 0,
            "communication_slot_index": 0,
            "sim_time_s": 0.0,
            "action_held_slots": 0,
        },
    )

    def release_boundary():
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if ack.exists() and json.loads(ack.read_text())["status"] == "paused":
                break
            time.sleep(0.001)
        action.write_text("7 2 0 0 0 0 0\n", encoding="ascii")
        _write(release, {"rl_decision_index": 0, "action_epoch": 7})

    worker = threading.Thread(target=release_boundary)
    worker.start()
    assert gate.wait_at_boundary(0.0, lambda: True) is True
    worker.join(timeout=1.0)
    final_ack = json.loads(ack.read_text())
    assert final_ack["status"] == "resumed"
    assert final_ack["sim_time_s"] == 0.0
    assert final_ack["simulation_time_unchanged"] is True
    assert final_ack["action_epoch"] == 7
    assert gate.next_decision_index == 1
    assert gate.boundary_due(0.099) is False
