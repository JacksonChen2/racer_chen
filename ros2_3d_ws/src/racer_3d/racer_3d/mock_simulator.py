"""Fast ROS-only 3-D sensor backend used before launching heavy Isaac Sim."""

import json
import math
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .pointcloud import create_xyzi_cloud
from .scenario import (
    DEFAULT_SCENARIO,
    DRONE_RADIUS,
    get_scenario,
    obstacle_clearance,
    pairwise_distances,
    simulate_point_cloud,
)


class Racer3DMockSimulator(Node):
    """Acceleration-limited point-mass plant with true 3-D ray casting."""

    def __init__(self) -> None:
        super().__init__("racer_3d_mock_sim")
        self.declare_parameter("scenario_name", DEFAULT_SCENARIO.name)
        self.declare_parameter("drone_count", 3)
        self.declare_parameter("lidar_range", 7.0)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("max_acceleration", 1.4)
        self.drone_count = int(self.get_parameter("drone_count").value)
        self.maximum_range = float(self.get_parameter("lidar_range").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.scenario = get_scenario(
            str(self.get_parameter("scenario_name").value)
        )
        starts = list(self.scenario.starts[:self.drone_count])
        self.positions = [np.asarray(point, dtype=float) for point in starts]
        self.velocities = [np.zeros(3) for _ in starts]
        self.commands = [np.zeros(3) for _ in starts]
        self.yaw_commands = [0.0 for _ in starts]
        self.yaws = [0.0 for _ in starts]
        self.path_lengths = [0.0 for _ in starts]
        self.collision_events = 0
        self.contact_active = [False for _ in starts]
        self.peer_contact_active = False
        self.safety_interventions = 0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.elapsed = 0.0
        self.last_cloud = -math.inf
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_publishers: List = []
        self.cloud_publishers: List = []
        for drone_id in range(self.drone_count):
            namespace = f"/drone_{drone_id}"
            self.odom_publishers.append(
                self.create_publisher(Odometry, namespace + "/odom", qos)
            )
            self.cloud_publishers.append(
                self.create_publisher(PointCloud2, namespace + "/points", qos)
            )
            self.create_subscription(
                Twist,
                namespace + "/cmd_vel_3d",
                lambda message, index=drone_id: self._command(index, message),
                qos,
            )
        self.metrics_publisher = self.create_publisher(
            String, "/racer_3d/sim_metrics", qos
        )
        self.create_timer(0.05, self._step)
        self.get_logger().info("RACER 3D mock backend ready")

    def _command(self, drone_id: int, message: Twist) -> None:
        self.commands[drone_id] = np.asarray(
            (message.linear.x, message.linear.y, message.linear.z), dtype=float
        )
        self.yaw_commands[drone_id] = float(message.angular.z)

    def _step(self) -> None:
        dt = 0.05
        self.elapsed += dt
        for drone_id in range(self.drone_count):
            command = self.commands[drone_id]
            norm = float(np.linalg.norm(command))
            if norm > self.max_speed:
                command = command * self.max_speed / norm
            delta = command - self.velocities[drone_id]
            delta_norm = float(np.linalg.norm(delta))
            limit = self.max_acceleration * dt
            if delta_norm > limit:
                delta *= limit / delta_norm
            velocity = self.velocities[drone_id] + delta
            proposed = self.positions[drone_id] + velocity * dt
            clearance = obstacle_clearance(
                proposed, self.scenario.obstacles
            )
            contact = clearance <= DRONE_RADIUS
            if contact and not self.contact_active[drone_id]:
                self.collision_events += 1
            self.contact_active[drone_id] = contact
            if contact:
                self.safety_interventions += 1
                velocity[:] = 0.0
                proposed = self.positions[drone_id].copy()
            self.path_lengths[drone_id] += float(
                np.linalg.norm(proposed - self.positions[drone_id])
            )
            self.positions[drone_id] = proposed
            self.velocities[drone_id] = velocity
            error = (
                self.yaw_commands[drone_id] - self.yaws[drone_id] + math.pi
            ) % (2.0 * math.pi) - math.pi
            self.yaws[drone_id] += float(np.clip(error, -1.5 * dt, 1.5 * dt))
        distances = list(pairwise_distances(self.positions))
        if distances:
            current_minimum = min(distances)
            peer_contact = current_minimum < 2.0 * DRONE_RADIUS
            if peer_contact and not self.peer_contact_active:
                self.collision_events += 1
            self.peer_contact_active = peer_contact
            self.min_inter_drone = min(
                self.min_inter_drone, current_minimum
            )
        for position in self.positions:
            self.min_obstacle_clearance = min(
                self.min_obstacle_clearance,
                obstacle_clearance(position, self.scenario.obstacles)
                - DRONE_RADIUS,
            )
        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(stamp)
        if self.elapsed - self.last_cloud >= 0.30 - 1.0e-9:
            self.last_cloud = self.elapsed
            self._publish_clouds(stamp)
        if int(round(self.elapsed * 10.0)) % 5 == 0:
            self._publish_metrics()

    def _publish_odometry(self, stamp) -> None:
        for drone_id, (position, velocity) in enumerate(
            zip(self.positions, self.velocities)
        ):
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
            message.pose.pose.position.x = float(position[0])
            message.pose.pose.position.y = float(position[1])
            message.pose.pose.position.z = float(position[2])
            message.pose.pose.orientation.w = math.cos(
                0.5 * self.yaws[drone_id]
            )
            message.pose.pose.orientation.z = math.sin(
                0.5 * self.yaws[drone_id]
            )
            message.twist.twist.linear.x = float(velocity[0])
            message.twist.twist.linear.y = float(velocity[1])
            message.twist.twist.linear.z = float(velocity[2])
            self.odom_publishers[drone_id].publish(message)

    def _publish_clouds(self, stamp) -> None:
        for drone_id, position in enumerate(self.positions):
            local = simulate_point_cloud(
                position,
                azimuth_count=120,
                elevation_count=21,
                maximum_range=self.maximum_range,
                obstacles=self.scenario.obstacles,
            )
            ranges = np.linalg.norm(local, axis=1)
            points_world = local + position
            hit = ranges < self.maximum_range - 0.03
            self.cloud_publishers[drone_id].publish(
                create_xyzi_cloud(stamp, "map", points_world, hit)
            )

    def _publish_metrics(self) -> None:
        self.metrics_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "backend": "ros_3d_mock",
                        "scenario": self.scenario.name,
                        "elapsed": self.elapsed,
                        "collision_events": self.collision_events,
                        "physics_contact_events": self.collision_events,
                        "safety_interventions": self.safety_interventions,
                        "min_inter_drone": self.min_inter_drone,
                        "min_obstacle_clearance": self.min_obstacle_clearance,
                        "positions": [
                            position.tolist() for position in self.positions
                        ],
                        "path_lengths": self.path_lengths,
                        "sensor_source": "3-D analytic lidar",
                        "motion_source": "acceleration-limited 3-D point mass",
                    },
                    separators=(",", ":"),
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Racer3DMockSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
