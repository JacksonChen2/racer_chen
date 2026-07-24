"""Small conversions kept separate so planner code never imports ROS."""

from __future__ import annotations

import numpy as np

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point


def time_to_seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def seconds_to_time(seconds: float) -> Time:
    message = Time()
    message.sec = int(seconds)
    message.nanosec = int(round((seconds - message.sec) * 1.0e9))
    if message.nanosec >= 1_000_000_000:
        message.sec += 1
        message.nanosec -= 1_000_000_000
    return message


def point_message(value: np.ndarray) -> Point:
    message = Point()
    message.x, message.y, message.z = map(float, value)
    return message


def point_array(message: Point) -> np.ndarray:
    return np.asarray((message.x, message.y, message.z), dtype=np.float64)

