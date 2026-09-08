import json
import math

import numpy as np
import pytest

from agentic_crpo.reference_metrics import ReferenceTaskMetricEvaluator


def _reference(tmp_path, *, include_auxiliary=True):
    metrics = {
        "elapsed": 10.0,
        "trajectory_history": [
            {"time_s": 0.0, "positions": [[0, 0, 0], [0, 0, 0]]},
            {"time_s": 10.0, "positions": [[10, 0, 0], [10, 0, 0]]},
        ],
        "mapping_coverage_history": [
            {"time_s": 0.0, "drone_id": 0, "ratio": 0.2},
            {"time_s": 10.0, "drone_id": 1, "ratio": 0.6},
        ],
    }
    statistics = {}
    if include_auxiliary:
        statistics["task_quality_history"] = [
            {
                "time_s": 0.0,
                "redundant_exploration_ratio": 0.1,
                "bs_global_map_iou": 0.8,
            },
            {
                "time_s": 10.0,
                "redundant_exploration_ratio": 0.2,
                "bs_global_map_iou": 1.0,
            },
        ]
    path = tmp_path / "perfect.json"
    path.write_text(
        json.dumps({"metrics": metrics, "communication": {"statistics": statistics}})
    )
    return path


def test_metrics_are_time_aligned_normalized_and_coverage_is_hold_last(tmp_path):
    evaluator = ReferenceTaskMetricEvaluator(
        str(_reference(tmp_path)),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
        fixed_exploration_time_s=10.0,
    )
    evaluator.validate_reference()
    evaluator.evaluate(
        {
            "sim_time_s": 0.0,
            "positions": [[0, 0, 0], [0, 0, 0]],
            "redundant_exploration_ratio": 0.1,
            "bs_global_map_iou": 0.8,
            "bs_global_map_coverage": 0.08,
        },
        {"coverage": 0.1},
    )
    metrics = evaluator.evaluate(
        {
            "sim_time_s": 5.0,
            # Reference interpolation at t=5 is x=5, so deviation is 1 m.
            "positions": [[6, 0, 0], [6, 0, 0]],
            "redundant_exploration_ratio": 0.5,
            "bs_global_map_iou": 0.4,
            "bs_global_map_coverage": 0.04,
        },
        {"coverage": 0.1},
    )
    assert metrics.trajectory_deviation_m == pytest.approx(1.0)
    assert metrics.d_traj == pytest.approx(0.5 / math.sqrt(300.0))
    assert metrics.d_cov == pytest.approx(0.5)
    assert metrics.d_red == pytest.approx(1.0)
    assert metrics.d_map == pytest.approx(0.06)
    assert metrics.joint_coverage == pytest.approx(0.1)
    assert metrics.bs_coverage == pytest.approx(0.04)
    values = (metrics.d_traj, metrics.d_cov, metrics.d_red, metrics.d_map)
    assert np.all(np.isfinite(values))
    assert np.all((0.0 <= np.asarray(values)) & (np.asarray(values) <= 1.0))

    evaluator.reset()
    metrics = evaluator.evaluate(
        {
            "sim_time_s": 0.0,
            "positions": [[0, 0, 0], [0, 0, 0]],
            "redundant_exploration_ratio": 0.1,
            "bs_global_map_coverage": 0.8,
        },
        {"coverage": 0.3},
    )
    assert metrics.d_map == 0.0
    assert metrics.joint_coverage == pytest.approx(0.3)
    assert metrics.bs_coverage == pytest.approx(0.8)


def test_explicit_union_coverage_history_overrides_legacy_per_uav_max(tmp_path):
    path = _reference(tmp_path)
    payload = json.loads(path.read_text())
    payload["metrics"]["mapping_coverage_joint_history"] = [
        {"time_s": 0.0, "ratio": 0.3},
        {"time_s": 10.0, "ratio": 0.9},
    ]
    path.write_text(json.dumps(payload))
    evaluator = ReferenceTaskMetricEvaluator(
        str(path),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
        fixed_exploration_time_s=10.0,
    )
    assert evaluator.coverage_values.tolist() == pytest.approx([0.3, 0.9])


def test_strict_reference_rejects_reconstructed_auxiliary_metrics(tmp_path):
    evaluator = ReferenceTaskMetricEvaluator(
        str(_reference(tmp_path, include_auxiliary=False)),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
    )
    with pytest.raises(FileNotFoundError, match="redundancy_history"):
        evaluator.validate_reference()


def test_separate_auxiliary_reference_does_not_replace_primary_curves(tmp_path):
    primary = _reference(tmp_path, include_auxiliary=False)
    primary_payload = json.loads(primary.read_text())
    primary_payload["metrics"]["mapping_coverage_history"][-1]["ratio"] = 0.75
    primary.write_text(json.dumps(primary_payload))

    auxiliary_dir = tmp_path / "auxiliary"
    auxiliary_dir.mkdir()
    auxiliary = _reference(auxiliary_dir, include_auxiliary=True)
    evaluator = ReferenceTaskMetricEvaluator(
        str(primary),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
        fixed_exploration_time_s=10.0,
        auxiliary_reference_result=str(auxiliary),
    )

    evaluator.validate_reference()
    assert evaluator.reference_source == primary.resolve()
    assert evaluator.auxiliary_reference_source == auxiliary.resolve()
    assert evaluator.coverage_values[-1] == pytest.approx(0.75)
    assert evaluator.redundancy_values[-1] == pytest.approx(0.2)
    assert evaluator.map_iou_values[-1] == pytest.approx(1.0)


def test_auxiliary_reference_only_needs_redundancy_for_new_map_loss(tmp_path):
    primary = _reference(tmp_path, include_auxiliary=False)
    auxiliary = tmp_path / "redundancy_only.json"
    auxiliary.write_text(
        json.dumps(
            {
                "metrics": {"elapsed": 10.0},
                "communication": {
                    "statistics": {
                        "redundancy_history": [
                            {"time_s": 0.0, "redundancy": 0.1},
                            {"time_s": 10.0, "redundancy": 0.2},
                        ]
                    }
                },
            }
        )
    )
    evaluator = ReferenceTaskMetricEvaluator(
        str(primary),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
        fixed_exploration_time_s=10.0,
        auxiliary_reference_result=str(auxiliary),
    )
    evaluator.validate_reference()
    assert evaluator.redundancy_values[-1] == pytest.approx(0.2)


def test_shared_metrics_require_and_deduplicate_canonical_task_clock(tmp_path):
    evaluator = ReferenceTaskMetricEvaluator(
        str(_reference(tmp_path)),
        2,
        (0.0, 0.0, 0.0),
        (10.0, 10.0, 10.0),
        fixed_exploration_time_s=10.0,
        require_explicit_task_clock=True,
    )
    base = {
        "sim_time_s": 0.0,
        "positions": [[0, 0, 0], [0, 0, 0]],
        "redundant_exploration_ratio": 0.1,
        "bs_global_map_iou": 0.8,
        "bs_global_map_coverage": 0.08,
    }
    with pytest.raises(ValueError, match="explicit task_step"):
        evaluator.evaluate(base, {"coverage": 0.1})

    first = evaluator.evaluate(
        {
            **base,
            "task_step": 0,
            "task_time_s": 0.0,
            "task_step_duration_s": 0.1,
        },
        {"coverage": 0.1},
    )
    duplicate = evaluator.evaluate(
        {
            **base,
            "task_step": 0,
            "task_time_s": 0.0,
            "task_step_duration_s": 0.1,
            "positions": [[9, 0, 0], [9, 0, 0]],
        },
        {"coverage": 0.9},
    )
    assert duplicate is first
    assert evaluator.trajectory_position_count == 2

    second = evaluator.evaluate(
        {
            **base,
            "task_step": 1,
            "task_time_s": 0.1,
            "task_step_duration_s": 0.1,
        },
        {"coverage": 0.1},
    )
    assert second.task_step == 1
    assert second.task_time_s == pytest.approx(0.1)
    assert evaluator.trajectory_position_count == 4
