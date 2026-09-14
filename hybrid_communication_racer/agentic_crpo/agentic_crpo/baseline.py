"""Deterministic BS-assisted scheduling baselines."""

from __future__ import annotations

import numpy as np

from .action_mapping import ActionMapping


BASELINE_POLICIES = ("distributed_only", "full_bs")


def deterministic_baseline_action(policy: str, n_uavs: int) -> np.ndarray:
    """Return policy bits; the backend remains responsible for feasibility."""

    mapping = ActionMapping(n_uavs)
    if policy == "distributed_only":
        return np.zeros(mapping.action_dim, dtype=np.int8)
    if policy == "full_bs":
        matrix = np.ones((n_uavs + 1, n_uavs), dtype=np.int8)
        np.fill_diagonal(matrix[:n_uavs], 0)
        return mapping.encode(matrix)
    raise ValueError(f"unknown deterministic baseline: {policy}")
