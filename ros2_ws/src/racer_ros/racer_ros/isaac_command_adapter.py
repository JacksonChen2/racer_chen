"""Translate RACER setpoints into standard messages consumed by an Isaac ROS 2 graph."""

from __future__ import annotations

import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from racer_core import ControllerConfig, PositionController
from racer_interfaces.msg import PositionCommand


class IsaacCommandAdapter(Node):
    def __init__(self) -> None:
        super().__init__("isaac_command_adapter")
        self.declare_parameter("position_gain", [5.7, 5.7, 6.2])
        self.declare_parameter("velocity_gain", [3.4, 3.4, 4.0])
        self.declare_parameter("max_acceleration", 10.0)
        self.controller = PositionController(
            ControllerConfig(
                np.asarray(self.get_parameter("position_gain").value),
                np.asarray(self.get_parameter("velocity_gain").value),
                max_acceleration=self.get_parameter("max_acceleration").value,
            )
        )
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.have_odom = False
        self.create_subscription(Odometry, "odometry", self._odom, qos_profile_sensor_data)
        self.create_subscription(PositionCommand, "position_cmd", self._command, 10)
        self.twist_publisher = self.create_publisher(TwistStamped, "isaac/velocity_command", 10)
        self.accel_publisher = self.create_publisher(AccelStamped, "isaac/acceleration_command", 10)
        self.pose_publisher = self.create_publisher(PoseStamped, "isaac/pose_command", 10)

    def _odom(self, message: Odometry) -> None:
        p, v = message.pose.pose.position, message.twist.twist.linear
        self.position[:] = p.x, p.y, p.z
        self.velocity[:] = v.x, v.y, v.z
        self.have_odom = True

    def _command(self, message: PositionCommand) -> None:
        if not self.have_odom:
            return
        target_position = np.asarray((message.position.x, message.position.y, message.position.z))
        target_velocity = np.asarray((message.velocity.x, message.velocity.y, message.velocity.z))
        target_acceleration = np.asarray(
            (message.acceleration.x, message.acceleration.y, message.acceleration.z)
        )
        control = self.controller.compute(
            self.position, self.velocity, target_position, target_velocity,
            target_acceleration, message.yaw, message.yaw_dot
        )
        now = self.get_clock().now().to_msg()
        twist = TwistStamped()
        twist.header.stamp, twist.header.frame_id = now, message.header.frame_id
        twist.twist.linear = message.velocity
        twist.twist.angular.z = message.yaw_dot
        acceleration = AccelStamped()
        acceleration.header = twist.header
        acceleration.accel.linear.x, acceleration.accel.linear.y, acceleration.accel.linear.z = map(
            float, control.acceleration
        )
        pose = PoseStamped()
        pose.header = twist.header
        pose.pose.position = message.position
        pose.pose.orientation.z = float(np.sin(message.yaw / 2.0))
        pose.pose.orientation.w = float(np.cos(message.yaw / 2.0))
        self.twist_publisher.publish(twist)
        self.accel_publisher.publish(acceleration)
        self.pose_publisher.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacCommandAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

