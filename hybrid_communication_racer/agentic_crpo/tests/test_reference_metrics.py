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
        },
        {"coverage": 0.1},
    )
    assert metrics.trajectory_deviation_m == pytest.approx(1.0)
    assert metrics.d_traj == pytest.approx(0.5 / math.sqrt(300.0))
    assert metrics.d_cov == pytest.approx(0.5)
    assert metrics.d_red == pytest.approx(1.0)
    assert metrics.d_map == pytest.approx(0.5)
    values = (metrics.d_traj, metrics.d_cov, metrics.d_red, metrics.d_map)
    assert np.all(np.isfinite(values))
    assert np.all((0.0 <= np.asarray(values)) & (np.asarray(values) <= 1.0))


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
