"""ROS 2 trajectory server port of ``plan_manage/src/traj_server.cpp``."""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node

from racer_core import NonUniformBspline
from racer_interfaces.msg import Bspline, PositionCommand

from .conversions import time_to_seconds


class TrajectoryServer(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_server")
        self.declare_parameter("drone_id", 1)
        self.declare_parameter("command_period", 0.01)
        self.declare_parameter("kx", [5.7, 5.7, 6.2])
        self.declare_parameter("kv", [3.4, 3.4, 4.0])
        self.drone_id = self.get_parameter("drone_id").value
        self.position: NonUniformBspline | None = None
        self.velocity: NonUniformBspline | None = None
        self.acceleration: NonUniformBspline | None = None
        self.yaw: NonUniformBspline | None = None
        self.yaw_rate: NonUniformBspline | None = None
        self.start_time = 0.0
        self.duration = 0.0
        self.trajectory_id = 0
        self.publisher = self.create_publisher(PositionCommand, "position_cmd", 10)
        self.create_subscription(Bspline, "planning/bspline", self._trajectory_callback, 10)
        self.create_timer(self.get_parameter("command_period").value, self._command_tick)

    def _trajectory_callback(self, message: Bspline) -> None:
        if message.drone_id != self.drone_id:
            return
        control = np.asarray([(point.x, point.y, point.z) for point in message.pos_pts])
        self.position = NonUniformBspline(control, message.order, 1.0)
        self.position.set_knot(message.knots)
        derivatives = self.position.derivatives(2)
        self.velocity, self.acceleration = derivatives[0], derivatives[1]
        yaw_control = np.asarray(message.yaw_pts, dtype=np.float64).reshape((-1, 1))
        self.yaw = NonUniformBspline(yaw_control, 3, max(message.yaw_dt, 1.0e-3))
        self.yaw_rate = self.yaw.derivative()
        self.start_time = time_to_seconds(message.start_time)
        self.duration = self.position.get_time_sum()
        self.trajectory_id = message.traj_id

    def _command_tick(self) -> None:
        if self.position is None:
            return
        elapsed = self.get_clock().now().nanoseconds * 1.0e-9 - self.start_time
        stamp = float(np.clip(elapsed, 0.0, self.duration))
        position = self.position.evaluate(stamp)
        velocity = self.velocity.evaluate(stamp)
        acceleration = self.acceleration.evaluate(stamp)
        yaw_time = min(stamp, self.yaw.get_time_sum())
        yaw = float(self.yaw.evaluate(yaw_time)[0])
        yaw_rate = float(self.yaw_rate.evaluate(min(yaw_time, self.yaw_rate.get_time_sum()))[0])
        message = PositionCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.position.x, message.position.y, message.position.z = map(float, position)
        message.velocity.x, message.velocity.y, message.velocity.z = map(float, velocity)
        message.acceleration.x, message.acceleration.y, message.acceleration.z = map(float, acceleration)
        message.yaw, message.yaw_dot = yaw, yaw_rate
        message.kx = self.get_parameter("kx").value
        message.kv = self.get_parameter("kv").value
        message.trajectory_id = int(self.trajectory_id)
        message.trajectory_flag = (
            PositionCommand.TRAJECTORY_STATUS_COMPLETED
            if elapsed >= self.duration else PositionCommand.TRAJECTORY_STATUS_READY
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
