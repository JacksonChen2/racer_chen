"""Numerical helpers ported from the original Eigen utility code."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def wrap_yaw(yaw: float) -> float:
    while yaw < -math.pi:
        yaw += 2.0 * math.pi
    while yaw > math.pi:
        yaw -= 2.0 * math.pi
    return yaw


def yaw_difference(lhs: float, rhs: float) -> float:
    return abs(wrap_yaw(lhs - rhs))


def safe_normalize(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return np.zeros_like(vector, dtype=np.float64)
    return np.asarray(vector, dtype=np.float64) / norm


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_rotation(
    x: float, y: float, z: float, w: float
) -> NDArray[np.float64]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def rotation_from_yaw(yaw: float) -> NDArray[np.float64]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.asarray(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
