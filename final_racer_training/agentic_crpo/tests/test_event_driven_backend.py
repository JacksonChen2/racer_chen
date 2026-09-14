import threading
from pathlib import Path

import numpy as np

from agentic_crpo.event_driven_backend import (
    EventDrivenOneShotSharedMemoryBackend,
)
from agentic_crpo.fast_state import (
    FAST_STATE_ABI,
    FAST_STATE_HEADER,
    FAST_STATE_MAGIC,
)
from agentic_crpo.shared_ipc import SharedMemoryLayout, create_layout


def _layout(tmp_path: Path):
    root = Path("/dev/shm") / f"fr_event_test_{tmp_path.name}"
    if root.exists():
        for path in root.iterdir():
            path.unlink()
        root.rmdir()
    layout = SharedMemoryLayout.from_value(root)
    return layout, create_layout(
        layout,
        capacities={name: 16 * 1024 for name in layout.BLOCKS},
    )


def _close(layout, blocks):
    for block in blocks.values():
        block.close()
    for path in layout.root.iterdir():
        path.unlink()
    layout.root.rmdir()


def _event_record(
    *,
    step: int,
    sequence: int,
    slot: int,
    sim_time_s: float,
    delta_t_s: float,
    completed: bool,
    action_version: int = 0,
    source_sequence: int = 0,
    source_sim_time_s: float = 0.0,
    action_age_s: float = 0.0,
    held_slots: int = 0,
) -> bytes:
    n = 2
    header = FAST_STATE_HEADER.pack(
        FAST_STATE_MAGIC,
        FAST_STATE_ABI,
        n,
        step,
        sequence,
        slot,
        step,
        sim_time_s,
        0.02,
        delta_t_s,
        action_version,
        0,
        0,
        0,
        1,
        source_sequence,
        step - 1 if completed else 0,
        source_sim_time_s,
        action_age_s,
        held_slots,
        sequence,
        0.2,
        0.01,
        2.0 if completed else 0.0,
        3.0 if completed else 0.0,
        4.0 if completed else 0.0,
        False,
        False,
        completed,
        0,
        0,
    )
    relay = np.asarray(
        [[0, 1], [0, 0]] if completed else [[0, 0], [0, 0]],
        dtype="u1",
    )
    upload = np.asarray([1, 0] if completed else [0, 0], dtype="u1")
    arrays = (
        np.asarray([[0, 0, 1], [1, 0, 1]], dtype="<f4"),
        np.asarray(
            [[-120, 10, 15], [10, -120, 16], [20, 21, -120]],
            dtype="<f4",
        ),
        np.asarray([[0, 0.1], [0.1, 0]], dtype="<f4"),
        np.asarray([0.1, 0.1], dtype="<f4"),
        np.asarray([[0, 1200], [1200, 0]], dtype="<u8"),
        np.asarray([1200, 1200], dtype="<u8"),
        np.asarray([1200, 1200], dtype="<u8"),
        np.zeros((n, n), dtype="<u8"),
        relay,
        upload,
        np.zeros((n, n), dtype="<f4"),
        np.full(n, 0.5, dtype="<f4"),
    )
    return header + b"".join(value.tobytes() for value in arrays)


def test_event_backend_consumes_one_action_and_variable_delta_t(tmp_path):
    layout, owner = _layout(tmp_path)
    owner["shutdown"].publish_json({"shutdown": False})
    owner["transition_ring"].publish_bytes(
        _event_record(
            step=0,
            sequence=1,
            slot=3,
            sim_time_s=0.06,
            delta_t_s=0.02,
            completed=False,
        ),
        sim_step=0,
        sim_time_s=0.06,
    )
    backend = EventDrivenOneShotSharedMemoryBackend(
        2,
        str(layout.root),
        require_perfect_reference=False,
        require_auxiliary_reference_metrics=False,
    )
    try:
        initial = backend.reset()
        assert initial.step_id == 0
        assert initial.communication_slot_index == 3
        assert initial.action_held_slots == 0

        def proxy():
            action = owner["action"].wait_for_newer(0)
            assert action is not None
            fields = action.payload.decode("ascii").split()
            action_version = int(fields[0])
            assert int(fields[3]) == 1  # exact source communication version
            assert int(fields[9]) == 0  # source event step
            owner["transition_ring"].publish_bytes(
                _event_record(
                    step=1,
                    sequence=2,
                    slot=6,
                    sim_time_s=0.12,
                    delta_t_s=0.06,
                    completed=True,
                    action_version=action_version,
                    source_sequence=1,
                    source_sim_time_s=0.06,
                    action_age_s=0.04,
                    held_slots=1,
                ),
                sim_step=1,
                sim_time_s=0.12,
            )

        thread = threading.Thread(target=proxy)
        thread.start()
        action_version = backend.publish_action(
            np.asarray([[0, 1], [0, 0], [1, 0]], dtype=np.int8)
        )
        result = backend.collect_transition()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert result.action_version == action_version == 1
        assert result.snapshot.rl_decision_index == 1
        assert result.snapshot.communication_slot_index == 6
        assert result.snapshot.action_held_slots == 1
        assert result.snapshot.rl_decision_interval_s == 0.06
        assert result.action_age == 0.04
        assert result.transition_sim_time_s == 0.06
        assert np.array_equal(
            result.executed_action_matrix,
            np.asarray([[0, 1], [0, 0], [1, 0]], dtype=np.int8),
        )
        assert (
            result.bs_ul_resource,
            result.bs_dl_resource,
            result.direct_u2u_resource,
        ) == (2.0, 3.0, 4.0)
        for step_id, sim_time_s, delta_t_s in (
            (0, 0.06, 0.02),
            (1, 0.12, 0.06),
        ):
            owner["task_metric_ring"].publish_json(
                {
                    "step_id": step_id,
                    "state_step_id": step_id,
                    "transition_step_id": step_id - 1,
                    "sim_time": sim_time_s,
                    "sim_time_s": sim_time_s,
                    "task_step": step_id,
                    "task_time_s": sim_time_s,
                    "task_step_duration_s": delta_t_s,
                    "positions": [[0, 0, 1], [1, 0, 1]],
                    "coverage": 0.1 + 0.01 * step_id,
                    "redundant_exploration_ratio": 0.1,
                    "bs_global_map_iou": 0.5,
                    "bs_global_map_coverage": 0.6,
                },
                sim_step=step_id,
                sim_time_s=sim_time_s,
            )
        metrics = backend.resolve_task_metrics([0, 1])
        assert metrics[0].task_time_s == 0.06
        assert metrics[1].task_time_s == 0.12
        status = backend.synchronization_status()
        assert status["status"] == "event_driven_one_shot"
        assert status["one_shot"] is True
        assert status["variable_delta_t"] is True
    finally:
        backend.close()
        _close(layout, owner)
