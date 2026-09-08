import numpy as np

from agentic_crpo.backend import MockRacerBackend
from dataclasses import replace

from agentic_crpo.llm_worker import _snapshot as llm_snapshot
from agentic_crpo.schemas import GlobalGuidance, TaskMetrics
from agentic_crpo.state_builder import (
    MIN_AVAILABLE_CHANNEL_VALUE,
    LargeStateBuilder,
    SmallStateBuilder,
    StateNormalization,
)


def test_ten_uav_observation_dimension_and_finiteness():
    backend = MockRacerBackend(10)
    snapshot = backend.reset(1)
    builder = SmallStateBuilder(10, StateNormalization())
    observation = builder.build(snapshot, GlobalGuidance.neutral(10))
    assert builder.observation_dim == 550
    assert observation.shape == (550,)
    assert observation.dtype == np.float32
    assert np.all(np.isfinite(observation))
    assert np.all((0.0 <= observation) & (observation <= 1.0))


def test_small_model_only_observation_removes_qwen_outputs():
    backend = MockRacerBackend(10)
    snapshot = backend.reset(1)
    guided_builder = SmallStateBuilder(10, StateNormalization())
    small_only_builder = SmallStateBuilder(
        10, StateNormalization(), include_global_guidance=False
    )
    guidance = GlobalGuidance.neutral(10)

    guided = guided_builder.build(snapshot, guidance)
    small_only = small_only_builder.build(snapshot, guidance)

    assert small_only_builder.observation_dim == 440
    assert "guidance" not in small_only_builder.slices
    assert "delta" not in guided_builder.slices
    assert "delta" not in small_only_builder.slices
    assert small_only.shape == (440,)
    np.testing.assert_array_equal(small_only, guided[:440])


def test_channel_encoding_distinguishes_no_link_from_available_poor_link():
    backend = MockRacerBackend(3)
    snapshot = backend.reset(2)
    snapshot.channel_snr_db[0, 3] = -120.0
    snapshot.channel_snr_db[3, 0] = -120.0
    snapshot.channel_snr_db[1, 3] = -50.0
    snapshot.channel_snr_db[3, 1] = -50.0
    builder = SmallStateBuilder(3, StateNormalization())
    observation = builder.build(snapshot, GlobalGuidance.neutral(3))
    channel = observation[builder.slices["channels"]]
    uplink = channel[6:9]
    downlink = channel[9:12]

    assert uplink[0] == downlink[0] == 0.0
    assert uplink[1] == downlink[1] == MIN_AVAILABLE_CHANNEL_VALUE


def test_perfect_reference_and_gt_loss_metrics_do_not_enter_actor_states():
    backend = MockRacerBackend(3)
    snapshot = backend.reset(7)
    changed = replace(
        snapshot,
        task_metrics=TaskMetrics(1.0, 1.0, 1.0, 1.0),
        coverage=0.99,
        coverage_delta=0.8,
        map_summary={"oracle_local_known_voxels": [999, 999, 999]},
        information_version_gap=np.full((3, 4), 999.0, np.float32),
    )
    guidance = GlobalGuidance.neutral(3)
    small_a = SmallStateBuilder(3, StateNormalization()).build(
        snapshot, guidance
    )
    small_b = SmallStateBuilder(3, StateNormalization()).build(
        changed, guidance
    )
    large_a = LargeStateBuilder(3, 4).build(snapshot)
    large_b = LargeStateBuilder(3, 4).build(changed)
    np.testing.assert_array_equal(small_a, small_b)
    assert large_a == large_b


def test_large_state_uses_requested_qwen_inputs():
    snapshot = MockRacerBackend(3).reset(7)
    state = LargeStateBuilder(3, 1).build(snapshot)

    assert {"velocities", "headings_cos_sin"}.isdisjoint(state)
    assert len(state["coarse_communication_summary"]) == 3
    assert len(state["peer_aoi_summary"]) == 3
    assert len(state["bs_aoi_s"]) == 3
    assert len(state["uav_bs_missing_bytes"]) == 3
    assert len(state["bs_uav_missing_bytes"]) == 3
    assert "bs_map_summary" in state
    assert "bs_exploration_progress" in state
    assert {
        "map_summary",
        "exploration_progress",
        "information_version_gap",
        "U2U_SNR_dB",
        "UAV_BS_SNR_dB",
        "BS_UAV_SNR_dB",
        "channel_outage_ratio",
        "pairwise_aoi_s",
    }.isdisjoint(state)


def test_ten_uav_large_state_contains_ten_coarse_link_summaries():
    snapshot = MockRacerBackend(10).reset(7)
    state = LargeStateBuilder(10, 50).build(snapshot)

    assert len(state["coarse_communication_summary"]) == 10
    assert all(
        "reachable_u2u_neighbor_count" in item
        for item in state["coarse_communication_summary"]
    )


def test_large_state_channel_window_and_reset():
    snapshot = MockRacerBackend(2).reset(7)
    good = replace(snapshot, channel_snr_db=np.full((3, 3), 10.0))
    bad = replace(snapshot, channel_snr_db=np.full((3, 3), -120.0))
    builder = LargeStateBuilder(2, 2)

    builder.build(good)
    averaged = builder.build(bad)
    assert averaged["coarse_communication_summary"][0][
        "ul_bs_outage_ratio"
    ] == 0.5

    builder.reset()
    reset_state = builder.build(bad)
    assert reset_state["coarse_communication_summary"][0][
        "ul_bs_outage_ratio"
    ] == 1.0


def test_large_state_uses_only_bs_map_progress_and_missing_inventory():
    snapshot = MockRacerBackend(3).reset(7)
    snapshot = replace(
        snapshot,
        bs_map_summary={
            "bs_map_coverage": 0.25,
            "bs_known_voxels": 25,
            "bs_total_voxels": 100,
            "bs_unknown_ratio": 0.75,
            "bs_frontier_count": 7,
        },
        bs_global_map_coverage=0.25,
        bs_map_coverage_delta=0.03,
        uav_bs_missing_bytes=np.asarray([100, 200, 300], np.float32),
        bs_uav_missing_bytes=np.asarray([400, 500, 600], np.float32),
        region_summary=[{"region_id": 0, "unknown_ratio": 0.75}],
    )
    state = LargeStateBuilder(3, 1).build(snapshot)

    assert state["bs_map_summary"]["bs_known_voxels"] == 25
    assert state["bs_exploration_progress"] == [0.25, 0.03]
    assert state["uav_bs_missing_bytes"] == [100.0, 200.0, 300.0]
    assert state["bs_uav_missing_bytes"] == [400.0, 500.0, 600.0]
    assert state["region_summary"] == [{"region_id": 0, "unknown_ratio": 0.75}]


def test_llm_snapshot_does_not_read_physical_oracle_map_or_positions():
    communication = {
        "sequence": 8,
        "sim_time_s": 5.0,
        "positions": [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]],
        "velocities": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        "yaws": [0.1, 0.2],
        "fsm_states": ["ASSIGNED", "REQUESTING_WORK"],
        "channel_snr_db": np.zeros((3, 3)).tolist(),
        "pair_aoi_s": np.zeros((2, 2)).tolist(),
        "bs_aoi_s": [0.2, 1.0],
        "pair_missing_bytes": np.zeros((2, 2)).tolist(),
        "uav_bs_missing_bytes": [10, 20],
        "bs_uav_missing_bytes": [30, 40],
        "uplink_queue_bytes": [0, 0],
        "relay_queue_bytes": np.zeros((2, 2)).tolist(),
        "bs_map_summary": {"bs_map_coverage": 0.2},
        "bs_map_coverage_delta": 0.01,
        "bs_global_map_coverage": 0.2,
    }
    physical_a = {
        "positions": [[90, 90, 90], [91, 91, 91]],
        "coverage": 0.1,
        "coverage_delta": 0.01,
        "map_summary": {"local_known_voxels": [1, 2]},
    }
    physical_b = {
        "positions": [[-90, -90, -90], [-91, -91, -91]],
        "coverage": 0.99,
        "coverage_delta": 0.8,
        "map_summary": {"local_known_voxels": [999, 999]},
    }

    state_a = LargeStateBuilder(2, 1).build(
        llm_snapshot(physical_a, communication, 2)
    )
    state_b = LargeStateBuilder(2, 1).build(
        llm_snapshot(physical_b, communication, 2)
    )

    assert state_a == state_b
    assert state_a["positions"] == communication["positions"]
    assert state_a["bs_exploration_progress"] == [0.2, 0.01]
