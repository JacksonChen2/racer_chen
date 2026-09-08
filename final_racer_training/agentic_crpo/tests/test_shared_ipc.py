import json
import multiprocessing
from pathlib import Path
import threading

import numpy as np
import pytest

from agentic_crpo.backend import SharedMemoryBackend
from agentic_crpo.fast_state import (
    FAST_STATE_ABI,
    FAST_STATE_HEADER,
    FAST_STATE_MAGIC,
)
from agentic_crpo.shared_ipc import (
    SharedMemoryLayout,
    create_layout,
    open_blocks,
    snapshot_consistent,
)


def _layout(tmp_path: Path):
    # SharedMemoryLayout intentionally accepts only a direct /dev/shm child.
    root = Path("/dev/shm") / f"fr_test_{tmp_path.name}"
    if root.exists():
        for path in root.iterdir():
            path.unlink()
        root.rmdir()
    layout = SharedMemoryLayout.from_value(root)
    blocks = create_layout(
        layout,
        capacities={name: 16 * 1024 for name in layout.BLOCKS},
    )
    return layout, blocks


def _close(layout, blocks):
    for block in blocks.values():
        block.close()
    for path in layout.root.iterdir():
        path.unlink()
    layout.root.rmdir()


def _publish_from_independent_process(root: str, count: int) -> None:
    """Process entry point: open, rather than inherit, the mmap block."""

    layout = SharedMemoryLayout.from_value(root)
    block = open_blocks(layout, ("physical",))["physical"]
    try:
        for sequence in range(1, count + 1):
            block.publish_json(
                {"sequence": sequence, "copy": sequence},
                sim_step=sequence,
                version=sequence,
            )
    finally:
        block.close()


def _physical(stamp=0.0):
    return {
        "sim_step": int(stamp * 1000),
        "sim_time_s": stamp,
        "positions": [[0, 0, 1], [1, 0, 1]],
        "velocities": [[0, 0, 0], [0, 0, 0]],
        "yaws": [0, 0],
        "fsm_states": ["EXPLORE", "EXPLORE"],
        "coverage": 0.1,
        "coverage_delta": 0.0,
        "map_summary": {},
        "terminated": False,
        "truncated": False,
    }


def _communication(sequence, stamp, resources):
    task_step = int(round(stamp / 0.1))
    return {
        "sequence": sequence,
        "sim_time_s": stamp,
        "task_step": task_step,
        "task_time_s": 0.1 * task_step,
        "task_step_duration_s": 0.1,
        "communication_slot_index": sequence,
        "rl_decision_index": sequence,
        "action_held_slots": 0,
        "communication_slot_duration_s": 0.02,
        "rl_decision_interval_s": 0.1,
        "positions": [[0, 0, 1], [1, 0, 1]],
        "velocities": [[0, 0, 0], [0, 0, 0]],
        "yaws": [0, 0],
        "fsm_states": ["EXPLORE", "EXPLORE"],
        "channel_snr_db": [[-120, 10, 15], [10, -120, 16], [20, 21, -120]],
        "pair_aoi_s": [[0, 0.1], [0.1, 0]],
        "bs_aoi_s": [0.1, 0.1],
        "pair_missing_bytes": [[0, 1200], [1200, 0]],
        "bs_missing_bytes": [1200, 1200],
        "uplink_queue_bytes": [1200, 1200],
        "relay_queue_bytes": [[0, 0], [0, 0]],
        "information_version_gap": [[0, 1, 1], [1, 0, 1]],
        "map_summary": {"bs_known_chunks": 1, "local_known_chunks": [2, 2]},
        "redundant_exploration_ratio": 0.1,
        "bs_global_map_iou": 0.5,
        "bs_global_map_coverage": 0.6,
        "bs_uplink_prb_slots": resources[0],
        "bs_downlink_prb_slots": resources[1],
        "direct_u2u_prb_slots": resources[2],
        "interval_start_sim_time_s": max(0.0, stamp - 0.1),
        "interval_end_sim_time_s": stamp,
        "interval_bs_uplink_prb_slots": 0,
        "interval_bs_downlink_prb_slots": 0,
        "interval_direct_u2u_prb_slots": 0,
        "has_completed_transition": stamp > 0.0,
        "executed_relay_action": [[0, 0], [0, 0]],
        "executed_upload_action": [0, 0],
        "action_version": 0,
        "policy_version": 0,
        "action_age": 0.0,
        "sim_timestamp": stamp,
    }


def _ring_record(communication, physical, communication_version, physical_version):
    del communication_version
    n = 2
    step_id = int(communication["task_step"])
    header = FAST_STATE_HEADER.pack(
        FAST_STATE_MAGIC,
        FAST_STATE_ABI,
        n,
        step_id,
        int(communication["sequence"]),
        int(communication["communication_slot_index"]),
        int(communication["rl_decision_index"]),
        float(communication["sim_time_s"]),
        0.02,
        0.1,
        int(communication.get("action_version", 0)),
        int(communication.get("policy_version", 0)),
        0,
        0,
        0,
        0,
        step_id,
        float(communication["sim_time_s"]),
        float(communication.get("action_age", 0.0)),
        int(communication.get("action_held_slots", 0)),
        int(physical_version),
        float(physical.get("coverage", 0.0)),
        float(physical.get("coverage_delta", 0.0)),
        float(communication.get("interval_bs_uplink_prb_slots", 0.0)),
        float(communication.get("interval_bs_downlink_prb_slots", 0.0)),
        float(communication.get("interval_direct_u2u_prb_slots", 0.0)),
        bool(physical.get("terminated", False)),
        bool(physical.get("truncated", False)),
        bool(communication.get("has_completed_transition", False)),
        0,
        0,
    )
    arrays = (
        np.asarray(communication["positions"], dtype="<f4"),
        np.asarray(communication["channel_snr_db"], dtype="<f4"),
        np.asarray(communication["pair_aoi_s"], dtype="<f4"),
        np.asarray(communication["bs_aoi_s"], dtype="<f4"),
        np.asarray(communication["pair_missing_bytes"], dtype="<u8"),
        np.asarray(communication["bs_missing_bytes"], dtype="<u8"),
        np.asarray(communication["uplink_queue_bytes"], dtype="<u8"),
        np.asarray(communication["relay_queue_bytes"], dtype="<u8"),
        np.asarray(communication["executed_relay_action"], dtype="u1"),
        np.asarray(communication["executed_upload_action"], dtype="u1"),
        np.zeros((n, n), dtype="<f4"),
        np.full(n, 1.0 / n, dtype="<f4"),
    )
    return header + b"".join(value.tobytes() for value in arrays)


def test_double_buffer_latest_state_and_consistent_snapshot(tmp_path):
    layout, blocks = _layout(tmp_path)
    try:
        for sequence in range(1, 101):
            blocks["physical"].publish_json(
                {"sequence": sequence, "copy": sequence},
                sim_step=sequence,
            )
        snapshot, payload = blocks["physical"].snapshot_json()
        assert snapshot.version == 100
        assert payload == {"sequence": 100, "copy": 100}

        blocks["communication"].publish_json({"sequence": 7})
        values = snapshot_consistent(
            {
                "physical": blocks["physical"],
                "communication": blocks["communication"],
            }
        )
        assert values["physical"][1]["sequence"] == 100
        assert values["communication"][1]["sequence"] == 7
    finally:
        _close(layout, blocks)


def test_transition_ring_retains_every_boundary_in_order(tmp_path):
    layout, blocks = _layout(tmp_path)
    try:
        ring = blocks["transition_ring"]
        for sequence in range(1, 8):
            ring.publish_json(
                {"boundary": sequence},
                sim_step=sequence * 5,
                sim_time_s=sequence * 0.1,
            )
        records = ring.read_after(0)
        assert [record.sequence for record in records] == list(range(1, 8))
        assert [record.json()["boundary"] for record in records] == list(
            range(1, 8)
        )
        assert [record.sim_step for record in records] == [
            sequence * 5 for sequence in range(1, 8)
        ]
    finally:
        _close(layout, blocks)


def test_shared_block_is_atomic_across_independent_processes(tmp_path):
    layout, blocks = _layout(tmp_path)
    process = multiprocessing.get_context("spawn").Process(
        target=_publish_from_independent_process,
        args=(str(layout.root), 200),
    )
    try:
        process.start()
        observed_versions = []
        version = 0
        # Read concurrently while the other interpreter is committing.  Use
        # non-blocking snapshots here so a child startup failure cannot strand
        # the test in an uninterruptible futex wait.
        while process.is_alive():
            snapshot = blocks["physical"].snapshot()
            if snapshot is not None and snapshot.version > version:
                payload = snapshot.json()
                assert payload["sequence"] == payload["copy"] == snapshot.version
                version = snapshot.version
                observed_versions.append(version)
        process.join(timeout=10)
        assert process.exitcode == 0
        snapshot = blocks["physical"].snapshot()
        assert snapshot is not None
        payload = snapshot.json()
        assert payload["sequence"] == payload["copy"] == snapshot.version == 200
        if version < snapshot.version:
            observed_versions.append(snapshot.version)
        assert observed_versions == sorted(set(observed_versions))
        assert observed_versions[-1] == 200
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=2)
        _close(layout, blocks)


def test_shared_backend_publishes_without_ack_and_consumes_ring(tmp_path):
    layout, owner = _layout(tmp_path)
    owner["shutdown"].publish_json({"shutdown": False})
    owner["physical"].publish_json(_physical(), sim_step=0, sim_time_s=0.0)
    initial_communication = _communication(1, 0.0, (10, 20, 30))
    initial_communication.update(
        {"communication_slot_index": 0, "rl_decision_index": 0}
    )
    owner["communication"].publish_json(
        initial_communication,
        sim_step=0,
        sim_time_s=0.0,
    )
    owner["transition_ring"].publish_bytes(
        _ring_record(initial_communication, _physical(), 1, 1),
        sim_step=0,
        sim_time_s=0.0,
    )
    backend = SharedMemoryBackend(
        2,
        str(layout.root),
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
    )
    try:
        backend.reset()
        status = backend.synchronization_status()
        assert status["task_step"] == 0
        assert status["channel_version"] == 1
        assert status["next_action_version"] == 1
        with pytest.raises(ValueError, match="channel_version"):
            backend.publish_action(
                np.zeros((3, 2), dtype=np.int8),
                {
                    "consumed_communication_version": 1,
                    "channel_version": 2,
                },
            )
        allow_first_boundary = threading.Event()

        def executor():
            action = owner["action"].wait_for_newer(0)
            assert action is not None
            fields = action.payload.decode("ascii").split()
            action_id = int(fields[0])
            assert int(fields[2]) == 1
            assert int(fields[3]) == 1
            assert int(fields[6]) == 0  # policy_version
            assert int(fields[7]) > 0  # generated wall timestamp
            assert allow_first_boundary.wait(timeout=1.0)
            owner["physical"].publish_json(
                _physical(0.04), sim_step=40, sim_time_s=0.04
            )
            next_communication = _communication(2, 0.1, (11, 22, 33))
            next_communication.update(
                {
                    "communication_slot_index": 5,
                    "rl_decision_index": 1,
                    "action_held_slots": 5,
                    "interval_bs_uplink_prb_slots": 1,
                    "interval_bs_downlink_prb_slots": 2,
                    "interval_direct_u2u_prb_slots": 3,
                    "action_version": action_id,
                    "policy_version": 0,
                    "action_age": 0.0,
                    "sim_timestamp": 0.1,
                }
            )
            owner["communication"].publish_json(
                next_communication,
                sim_step=1,
                sim_time_s=0.1,
            )
            owner["transition_ring"].publish_bytes(
                _ring_record(
                    next_communication, _physical(0.1), 2, 2
                ),
                sim_step=1,
                sim_time_s=0.1,
            )
            second_action = owner["action"].wait_for_newer(action_id)
            assert second_action is not None
            held_communication = _communication(3, 0.2, (15, 27, 39))
            held_communication.update(
                {
                    "communication_slot_index": 10,
                    "rl_decision_index": 2,
                    "action_held_slots": 5,
                    "interval_bs_uplink_prb_slots": 4,
                    "interval_bs_downlink_prb_slots": 5,
                    "interval_direct_u2u_prb_slots": 6,
                    "action_version": action_id,
                    "policy_version": 0,
                    "action_age": 0.1,
                    "sim_timestamp": 0.2,
                }
            )
            owner["communication"].publish_json(
                held_communication, sim_step=10, sim_time_s=0.2
            )
            owner["transition_ring"].publish_bytes(
                _ring_record(
                    held_communication, _physical(0.2), 3, 3
                ),
                sim_step=2,
                sim_time_s=0.2,
            )

        thread = threading.Thread(target=executor)
        thread.start()
        action_id = backend.publish_action(
            np.zeros((3, 2), dtype=np.int8)
        )
        assert action_id == 1
        # Publication completed before ACK or the next transition existed.
        allow_first_boundary.set()
        result = backend.collect_transition()
        assert result.snapshot.sim_time_s == 0.1
        # BS map quality is intentionally absent from Fast RL State and is
        # delivered through task_metric_ring for asynchronous cost backfill.
        assert result.snapshot.bs_global_map_coverage == 0.0
        assert (
            result.bs_ul_resource,
            result.bs_dl_resource,
            result.direct_u2u_resource,
        ) == (1.0, 2.0, 3.0)
        assert result.action_version == 1
        assert result.policy_version == 0
        assert backend.synchronization_status()["next_action_version"] == 2
        backend.publish_action(np.zeros((3, 2), dtype=np.int8))
        second = backend.collect_transition()
        thread.join(timeout=1)
        assert not thread.is_alive()
        assert second.snapshot.sim_time_s == 0.2
        assert second.action_version == result.action_version == 1
        assert second.action_age == 0.1
        assert owner["action_ack"].version == 0
        assert (
            second.bs_ul_resource,
            second.bs_dl_resource,
            second.direct_u2u_resource,
        ) == (4.0, 5.0, 6.0)
    finally:
        backend.close()
        _close(layout, owner)


def test_task_metrics_are_resolved_from_independent_ring_by_step_id(tmp_path):
    layout, owner = _layout(tmp_path)
    owner["shutdown"].publish_json({"shutdown": False})
    backend = SharedMemoryBackend(
        2,
        str(layout.root),
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
    )
    try:
        for step_id in (0, 1):
            sim_time = 0.1 * step_id
            owner["task_metric_ring"].publish_json(
                {
                    "step_id": step_id,
                    "state_step_id": step_id,
                    "transition_step_id": step_id - 1,
                    "sim_time": sim_time,
                    "sim_time_s": sim_time,
                    "task_step": step_id,
                    "task_time_s": sim_time,
                    "task_step_duration_s": 0.1,
                    "positions": [[0, 0, 1], [1, 0, 1]],
                    "coverage": 0.1 + 0.01 * step_id,
                    "redundant_exploration_ratio": 0.1,
                    "bs_global_map_iou": 0.5,
                    "bs_global_map_coverage": 0.6,
                },
                sim_step=step_id,
                sim_time_s=sim_time,
            )
        metrics = backend.resolve_task_metrics([0, 1])
        assert list(metrics) == [0, 1]
        assert metrics[0].task_step == 0
        assert metrics[1].task_step == 1
        assert metrics[1].task_time_s == 0.1
        assert metrics[1].d_map == 0.0
        assert metrics[1].joint_coverage == pytest.approx(0.11)
        assert metrics[1].bs_coverage == pytest.approx(0.6)
        assert backend.last_task_metric_sequence == 2
    finally:
        backend.close()
        _close(layout, owner)
