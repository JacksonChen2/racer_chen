from __future__ import annotations

import unittest

from geometry_msgs.msg import Twist
from racer_interfaces.msg import Bspline, DroneState

from racer_ros.conversions import seconds_to_time, time_to_seconds


class RosContractTests(unittest.TestCase):
    def test_drone_state_uses_original_interface(self) -> None:
        state = DroneState()
        state.drone_id = 1
        state.grid_ids = [1, 2]
        state.recent_attempt_time = 0.0
        self.assertFalse(hasattr(state, "trajectory_id"))

    def test_isaac_velocity_contract_is_twist(self) -> None:
        command = Twist()
        command.linear.x = 1.0
        self.assertEqual(command.linear.x, 1.0)

    def test_bspline_has_independent_yaw_interval(self) -> None:
        trajectory = Bspline()
        trajectory.yaw_dt = 0.2
        self.assertAlmostEqual(trajectory.yaw_dt, 0.2)

    def test_time_conversion_round_trip(self) -> None:
        value = 123.456789
        self.assertAlmostEqual(
            time_to_seconds(seconds_to_time(value)), value, places=8
        )


if __name__ == "__main__":
    unittest.main()
