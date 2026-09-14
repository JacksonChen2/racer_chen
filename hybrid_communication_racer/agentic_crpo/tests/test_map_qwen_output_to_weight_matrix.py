import json
import subprocess
import sys
from pathlib import Path

import numpy as np


SCRIPT = (
    Path(__file__).parents[1]
    / "scripts"
    / "map_qwen_output_to_weight_matrix.py"
)


def test_cli_maps_ranked_links_and_bs_priorities_to_guidance_matrix():
    model_output = (
        "UAV links: 1:2,3 2:3,1 3:1,2\n"
        "BS priority: 1:4 2:2 3:0\n"
    )
    completed = subprocess.run(
        [sys.executable, str(SCRIPT), "--n-uavs", "3"],
        input=model_output,
        text=True,
        capture_output=True,
        check=True,
    )
    payload = json.loads(completed.stdout)

    assert payload["shape"] == [4, 3]
    matrix = np.asarray(payload["weight_matrix"], dtype=np.float32)
    np.testing.assert_allclose(
        matrix,
        [
            [0.0, 1.0, 0.5],
            [0.5, 0.0, 1.0],
            [1.0, 0.5, 0.0],
            [2.0 / 3.0, 1.0 / 3.0, 0.0],
        ],
        atol=1.0e-6,
    )
    np.testing.assert_allclose(
        payload["small_model_guidance_vector"], matrix.reshape(-1)
    )


def test_cli_can_keep_bs_priorities_scaled_without_sum_normalization():
    model_output = "UAV links: 1:2 2:1\nBS priority: 1:4 2:2\n"
    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--n-uavs",
            "2",
            "--no-normalize-bs-priority",
        ],
        input=model_output,
        text=True,
        capture_output=True,
        check=True,
    )
    matrix = np.asarray(
        json.loads(completed.stdout)["weight_matrix"], dtype=np.float32
    )
    np.testing.assert_allclose(matrix[-1], [1.0, 0.5])
