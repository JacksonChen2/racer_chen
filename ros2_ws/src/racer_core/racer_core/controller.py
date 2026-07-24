"""Position-command tracking laws shared by ROS 2 and Isaac adapters."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


Array = NDArray[np.float64]


@dataclass(slots=True)
class ControllerConfig:
    position_gain: Array = field(default_factory=lambda: np.asarray((5.7, 5.7, 6.2)))
    velocity_gain: Array = field(default_factory=lambda: np.asarray((3.4, 3.4, 4.0)))
    mass: float = 1.0
    gravity: float = 9.81
    max_acceleration: float = 10.0


@dataclass(slots=True)
class ControlSetpoint:
    acceleration: Array
    yaw: float
    yaw_rate: float


class PositionController:
    def __init__(self, config: ControllerConfig | None = None) -> None:
        self.config = config or ControllerConfig()

    def compute(
        self,
        position: Array,
        velocity: Array,
        target_position: Array,
        target_velocity: Array,
        target_acceleration: Array,
        target_yaw: float,
        target_yaw_rate: float,
    ) -> ControlSetpoint:
        acceleration = (
            np.asarray(target_acceleration)
            + self.config.position_gain * (np.asarray(target_position) - np.asarray(position))
            + self.config.velocity_gain * (np.asarray(target_velocity) - np.asarray(velocity))
        )
        norm = float(np.linalg.norm(acceleration))
        if norm > self.config.max_acceleration:
            acceleration *= self.config.max_acceleration / norm
        return ControlSetpoint(acceleration, float(target_yaw), float(target_yaw_rate))

