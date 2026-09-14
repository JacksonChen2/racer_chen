#!/usr/bin/env python3
"""Convert compact Qwen guidance text into small-model guidance weights."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from agentic_crpo.qwen_global_agent import parse_guidance  # noqa: E402


def map_output_to_weight_matrix(
    model_output: str,
    n_uavs: int,
    *,
    normalize_bs_priority: bool = True,
) -> np.ndarray:
    """Return the ``(N+1, N)`` guidance matrix consumed by the small model.

    Rows ``0..N-1`` contain ranked UAV-to-UAV link weights.  For top-4, the
    listed targets receive weights ``1.0, 0.75, 0.5, 0.25``.  The last row
    contains the BS upload-priority weights for UAVs ``1..N``.
    """

    if n_uavs < 1:
        raise ValueError("n_uavs must be positive")
    guidance = parse_guidance(
        model_output,
        n_uavs,
        normalize_omega=normalize_bs_priority,
    )
    return np.vstack(
        (guidance.task_dependency, guidance.semantic_importance[None, :])
    ).astype(np.float32, copy=False)


def _read_text(path: str) -> str:
    if path == "-":
        return sys.stdin.read()
    return Path(path).read_text(encoding="utf-8")


def _write_json(payload: dict[str, object], path: str) -> None:
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    if path == "-":
        sys.stdout.write(text)
    else:
        Path(path).write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Map 'UAV links' and 'BS priority' output to the weight matrix "
            "inserted into the small-model observation."
        )
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="-",
        help="Qwen output text file, or '-' to read stdin (default)",
    )
    parser.add_argument("--n-uavs", type=int, default=10)
    parser.add_argument(
        "--output",
        default="-",
        help="JSON output file, or '-' to write stdout (default)",
    )
    parser.add_argument(
        "--no-normalize-bs-priority",
        action="store_true",
        help="keep BS priority as priority/4 instead of normalizing its sum",
    )
    args = parser.parse_args()

    try:
        matrix = map_output_to_weight_matrix(
            _read_text(args.input),
            args.n_uavs,
            normalize_bs_priority=not args.no_normalize_bs_priority,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))

    rounded = np.round(matrix.astype(np.float64), decimals=6)
    payload: dict[str, object] = {
        "n_uavs": args.n_uavs,
        "shape": list(matrix.shape),
        "layout": (
            "rows 1..N: source-UAV to receiver-UAV weights; "
            "row N+1: BS upload-priority weights"
        ),
        "weight_matrix": rounded.tolist(),
        "small_model_guidance_vector": rounded.reshape(-1).tolist(),
    }
    _write_json(payload, args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
