import json
import threading
import time

import numpy as np
import pytest

from agentic_crpo.backend import FileBridgeBackend, MissionEndedError


def _write_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value), encoding="utf-8")
    temporary.replace(path)


def _sync_payload(decision, sequence, *, resources, interval):
    n = 2
    return {
        "synchronous_online": True,
        "sequence": sequence,
        "action_epoch": decision,
        "sim_time_s": 0.1 * decision,
        "communication_slot_index": 5 * decision,
        "rl_decision_index": decision,
        "action_held_slots": 0 if decision == 0 else 5,
        "communication_slot_duration_s": 0.02,
        "rl_decision_interval_s": 0.1,
        "channel_snr_db": [[-120.0, 10.0, 15.0], [10.0, -120.0, 16.0], [20.0, 21.0, -120.0]],
        "pair_aoi_s": [[0.0, 0.1], [0.1, 0.0]],
        "bs_aoi_s": [0.1, 0.1],
        "pair_missing_bytes": [[0.0, 1200.0], [1200.0, 0.0]],
        "bs_missing_bytes": [1200.0, 1200.0],
        "uplink_queue_bytes": [1200.0, 1200.0],
        "relay_queue_bytes": [[0.0, 0.0], [0.0, 0.0]],
        "information_version_gap": [[0.0, 1.0, 1.0], [1.0, 0.0, 1.0]],
        "positions": [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]],
        "velocities": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "yaws": [0.0, 0.0],
        "bs_uplink_prb_slots": resources[0],
        "bs_downlink_prb_slots": resources[1],
        "direct_u2u_prb_slots": resources[2],
        "interval_bs_uplink_prb_slots": interval[0],
        "interval_bs_downlink_prb_slots": interval[1],
        "interval_direct_u2u_prb_slots": interval[2],
        "bs_global_map_coverage": 0.06,
    }


def _mission(decision, *, terminal=False):
    return {
        "sim_time_s": 0.1 * decision,
        "positions": [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0]],
        "velocities": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "yaws": [0.0, 0.0],
        "fsm_states": ["FINISH" if terminal else "EXPLORE"] * 2,
        "coverage": 0.1 * decision,
        "coverage_delta": 0.1 if decision else 0.0,
        "terminated": False,
        "truncated": terminal,
        "task_metrics": {
            "D_traj": 0.1,
            "D_cov": 0.2,
            "D_red": 0.3,
            "D_map": 0.4,
            "coverage": 0.1 * decision,
            "coverage_pc": 0.2,
            "redundancy": 0.3,
            "redundancy_pc": 0.3,
            "map_iou": 0.6,
            "map_iou_pc": 0.7,
            "trajectory_deviation_m": 5.0,
        },
    }


def _ack(decision, sequence, *, terminal=False):
    return {
        "status": "terminal" if terminal else "paused",
        "rl_decision_index": decision,
        "communication_slot_index": 5 * decision,
        "state_sequence": sequence,
        "sim_time_s": 0.1 * decision,
    }


def test_file_bridge_stops_immediately_at_terminal_mission(tmp_path):
    telemetry_path = tmp_path / "communication_state.json"
    mission_path = tmp_path / "mission_state.json"
    telemetry_path.write_text(json.dumps({"sequence": 7}), encoding="utf-8")
    mission_path.write_text(
        json.dumps({"terminated": False, "truncated": True}),
        encoding="utf-8",
    )
    backend = FileBridgeBackend(
        2,
        action_path=str(tmp_path / "action.txt"),
        telemetry_path=str(telemetry_path),
        mission_state_path=str(mission_path),
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
        timeout_s=30.0,
        poll_interval_s=0.001,
    )

    started = time.monotonic()
    with pytest.raises(TimeoutError, match="mission ended"):
        backend._wait_payload(after_sequence=7)

    assert time.monotonic() - started < 0.5


def test_synchronous_file_bridge_consumes_exactly_one_100ms_interval(tmp_path):
    state_path = tmp_path / "communication_state.json"
    mission_path = tmp_path / "mission_state.json"
    action_path = tmp_path / "action.txt"
    ack_path = tmp_path / "ack.json"
    release_path = tmp_path / "release.json"
    _write_json(state_path, _sync_payload(0, 20, resources=(10, 20, 30), interval=(0, 0, 0)))
    _write_json(mission_path, _mission(0))
    _write_json(ack_path, _ack(0, 20))
    backend = FileBridgeBackend(
        2,
        action_path=str(action_path),
        telemetry_path=str(state_path),
        mission_state_path=str(mission_path),
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
        timeout_s=2.0,
        poll_interval_s=0.001,
        synchronous_online=True,
        sync_acknowledgement_path=str(ack_path),
        sync_release_path=str(release_path),
    )
    initial = backend.reset()
    assert (initial.sim_time_s, initial.communication_slot_index, initial.rl_decision_index) == (0.0, 0, 0)

    def publish_next_boundary():
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not release_path.exists():
            time.sleep(0.001)
        release = json.loads(release_path.read_text(encoding="utf-8"))
        assert release["rl_decision_index"] == 0
        assert release["communication_slot_index"] == 0
        _write_json(mission_path, _mission(1, terminal=True))
        _write_json(state_path, _sync_payload(1, 21, resources=(11, 22, 33), interval=(1, 2, 3)))
        _write_json(ack_path, _ack(1, 21, terminal=True))

    publisher = threading.Thread(target=publish_next_boundary)
    publisher.start()
    result = backend.step(np.zeros((3, 2), dtype=np.int8))
    publisher.join(timeout=1.0)
    assert not publisher.is_alive()
    assert result.snapshot.sim_time_s == pytest.approx(0.1)
    assert result.snapshot.communication_slot_index == 5
    assert result.snapshot.rl_decision_index == 1
    assert result.snapshot.action_held_slots == 5
    assert result.snapshot.truncated is True
    assert result.snapshot.task_metrics.d_map == pytest.approx(0.04)
    assert result.snapshot.task_metrics.joint_coverage == pytest.approx(0.1)
    assert result.snapshot.task_metrics.bs_coverage == pytest.approx(0.06)
    assert (result.bs_ul_resource, result.bs_dl_resource, result.direct_u2u_resource) == (1.0, 2.0, 3.0)
    action_fields = action_path.read_text(encoding="ascii").split()
    header = action_fields[:3]
    assert header == ["1", "2", "0"]
    assert action_fields[-2:] == ["20", "0"]
    with pytest.raises(MissionEndedError, match="already consumed"):
        backend.step(np.zeros((3, 2), dtype=np.int8))
