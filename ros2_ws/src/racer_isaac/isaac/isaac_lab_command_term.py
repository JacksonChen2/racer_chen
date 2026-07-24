"""Isaac Lab command-term adapter for RACER setpoints.

The ROS 2 Bridge graph owns DDS.  This helper accepts values obtained from its
Twist subscriber and converts them to the world-frame velocity command expected
by an Isaac Lab manager-based environment.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass
class RacerCommandBuffer:
    linear_velocity: np.ndarray
    yaw_rate: float
    acceleration: np.ndarray

    @classmethod
    def zeros(cls) -> "RacerCommandBuffer":
        return cls(np.zeros(3, dtype=np.float32), 0.0, np.zeros(3, dtype=np.float32))

    def update_twist(self, linear, angular) -> None:
        self.linear_velocity[:] = linear[0], linear[1], linear[2]
        self.yaw_rate = float(angular[2])

    def command_tensor(self, torch, device: str):
        """Return the ``(vx, vy, vz, yaw_rate)`` tensor used by Lab terms."""
        values = np.r_[self.linear_velocity, self.yaw_rate]
        return torch.as_tensor(values, dtype=torch.float32, device=device).unsqueeze(0)

