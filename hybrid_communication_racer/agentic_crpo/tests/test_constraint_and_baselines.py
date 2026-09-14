import json

import numpy as np
import pytest

from agentic_crpo.baseline import deterministic_baseline_action
from agentic_crpo.constraint import resolve_constraint


def test_deterministic_baseline_actions():
    distributed = deterministic_baseline_action("distributed_only", 3)
    full = deterministic_baseline_action("full_bs", 3)
    assert distributed.shape == (9,)
    assert not np.any(distributed)
    assert np.all(full == 1)


def test_manual_gamma_is_authoritative_without_baseline_files():
    result = resolve_constraint(
        {"constraint": {"gamma_task": 0.15, "rho": 0.25, "eta": 0.01}}
    )
    assert result.gamma_task == pytest.approx(0.15)
    assert result.source == "manual"


def test_baseline_gap_calibration(tmp_path):
    full = tmp_path / "full.json"
    dist = tmp_path / "dist.json"
    full.write_text(json.dumps({"episodes": [{"L_task_final": 0.1}]}))
    dist.write_text(json.dumps({"mean_final_task_loss": 0.5}))
    result = resolve_constraint(
        {
            "constraint": {
                "gamma_task": None,
                "rho": 0.25,
                "full_bs_results": [str(full)],
                "distributed_only_results": [str(dist)],
            }
        }
    )
    assert result.l_full == pytest.approx(0.1)
    assert result.l_dist == pytest.approx(0.5)
    assert result.gamma_task == pytest.approx(0.2)
