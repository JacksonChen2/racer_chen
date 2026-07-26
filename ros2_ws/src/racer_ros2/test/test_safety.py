import math
import unittest

from racer_ros2.safety import (
    cbf_swarm_filter,
    emergency_separation,
    obstacle_brake,
    predicted_path_conflict,
)


class SafetyTest(unittest.TestCase):
    def test_cbf_removes_closing_velocity(self):
        safe = cbf_swarm_filter(
            (1.0, 0.0),
            (0.0, 0.0),
            [(1, (1.1, 0.0), (-1.0, 0.0))],
            safe_distance=1.0,
            gamma=1.5,
        )
        relative = (-1.1, 0.0)
        h_value = 1.1**2 - 1.0**2
        lhs = 2.0 * (
            relative[0] * (safe[0] - (-1.0))
            + relative[1] * safe[1]
        )
        self.assertGreaterEqual(lhs, -1.5 * h_value - 1.0e-7)

    def test_lidar_brake_stops_toward_close_obstacle(self):
        ranges = [10.0] * 9
        ranges[4] = 0.45
        result = obstacle_brake(
            (1.0, 0.0),
            ranges,
            -math.pi,
            math.pi / 4.0,
            0.0,
            robot_radius=0.30,
        )
        self.assertLessEqual(result[0], 1.0e-6)

    def test_predicted_trajectory_conflict(self):
        first = [(1.0, 0.0, 0.0), (2.0, 1.0, 0.0)]
        second = [(1.1, 3.0, 0.0), (2.1, 1.4, 0.0)]
        self.assertTrue(predicted_path_conflict(first, second, 0.6))
        self.assertFalse(predicted_path_conflict(first, second, 0.2))

    def test_emergency_separation_overrides_task_velocity(self):
        result = emergency_separation(
            (1.0, 0.0),
            (0.0, 0.0),
            [(1, (0.7, 0.0), (0.0, 0.0))],
            activation_distance=0.95,
            max_speed=1.2,
        )
        self.assertLess(result[0], 0.0)
