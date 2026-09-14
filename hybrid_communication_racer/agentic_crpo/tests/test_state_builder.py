import numpy as np

from agentic_crpo.backend import MockRacerBackend
from dataclasses import replace

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
    assert state["U2U_SNR_dB"].count(":") == 6
    assert "1-2:" in state["U2U_SNR_dB"]
    assert "2-1:" in state["U2U_SNR_dB"]
    assert state["UAV_BS_SNR_dB"].count(":") == 3
    assert state["BS_UAV_SNR_dB"].count(":") == 3
    assert np.asarray(state["channel_outage_ratio"]).shape == (4, 4)
    assert np.asarray(state["pairwise_aoi_s"]).shape == (3, 3)
    assert len(state["bs_aoi_s"]) == 3


def test_ten_uav_large_state_contains_90_directed_u2u_links():
    snapshot = MockRacerBackend(10).reset(7)
    state = LargeStateBuilder(10, 50).build(snapshot)

    assert state["U2U_SNR_dB"].count(":") == 90


def test_large_state_channel_window_and_reset():
    snapshot = MockRacerBackend(2).reset(7)
    good = replace(snapshot, channel_snr_db=np.full((3, 3), 10.0))
    bad = replace(snapshot, channel_snr_db=np.full((3, 3), -10.0))
    builder = LargeStateBuilder(2, 2)

    builder.build(good)
    averaged = builder.build(bad)
    np.testing.assert_allclose(averaged["channel_outage_ratio"], 0.5)

    builder.reset()
    reset_state = builder.build(bad)
    np.testing.assert_allclose(reset_state["channel_outage_ratio"], 1.0)
