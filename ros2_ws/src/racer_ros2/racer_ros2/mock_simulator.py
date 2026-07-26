"""Lightweight deterministic ROS 2 plant for fast integration tests."""

import json
import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .safety import limit_norm
from .scenario import (
    DRONE_RADIUS,
    FLIGHT_Z,
    STARTS,
    default_obstacles,
    obstacle_clearance,
    pairwise_distances,
    simulate_scan,
)


class MockSwarmSimulator(Node):
    """World-frame velocity plant using the exact Isaac scene geometry."""

    def __init__(self) -> None:
        super().__init__("racer_mock_sim")
        self.declare_parameter("drone_count", 3)
        self.declare_parameter("scenario", "small")
        self.declare_parameter(
            "start_positions", [-8.2, -4.8, -8.2, 4.8, 7.8, 0.0]
        )
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("scan_rate", 10.0)
        self.declare_parameter("scan_rays", 180)
        self.declare_parameter("scan_range", 6.0)
        self.declare_parameter("max_speed", 1.2)
        self.declare_parameter("max_acceleration", 1.5)
        self.declare_parameter("robot_radius", DRONE_RADIUS)
        self.declare_parameter("flight_z", FLIGHT_Z)

        self.drone_count = int(self.get_parameter("drone_count").value)
        flattened = [
            float(item) for item in self.get_parameter("start_positions").value
        ]
        starts = [
            (flattened[index], flattened[index + 1])
            for index in range(0, len(flattened) - 1, 2)
        ]
        while len(starts) < self.drone_count:
            starts.append(STARTS[len(starts) % len(STARTS)])
        self.positions = [list(item) for item in starts[: self.drone_count]]
        self.velocities = [[0.0, 0.0] for _ in range(self.drone_count)]
        self.commands = [[0.0, 0.0, 0.0] for _ in range(self.drone_count)]
        self.yaws = [0.0 for _ in range(self.drone_count)]
        self.scenario_name = str(self.get_parameter("scenario").value)
        self.obstacles = default_obstacles(self.scenario_name)
        self.flight_z = float(self.get_parameter("flight_z").value)
        self.radius = float(self.get_parameter("robot_radius").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.scan_rays = int(self.get_parameter("scan_rays").value)
        self.scan_range = float(self.get_parameter("scan_range").value)
        update_rate = float(self.get_parameter("update_rate").value)
        scan_rate = float(self.get_parameter("scan_rate").value)
        self.dt = 1.0 / update_rate
        self.scan_period = 1.0 / scan_rate
        self.last_scan = -math.inf
        self.elapsed = 0.0
        self.collision_events = 0
        self.safety_interventions = 0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_publishers = []
        self.scan_publishers = []
        for drone_id in range(self.drone_count):
            namespace = f"/drone_{drone_id}"
            self.odom_publishers.append(
                self.create_publisher(
                    Odometry, namespace + "/odom", qos
                )
            )
            self.scan_publishers.append(
                self.create_publisher(
                    LaserScan, namespace + "/scan", qos
                )
            )
            self.create_subscription(
                Twist,
                namespace + "/cmd_vel",
                lambda message, index=drone_id: self._command(index, message),
                qos,
            )
        self.metrics_publisher = self.create_publisher(
            String, "/racer/sim_metrics", qos
        )
        self.create_timer(self.dt, self._step)
        self.get_logger().info(
            f"mock exploration world running with {self.drone_count} UAVs"
        )

    def _command(self, drone_id: int, message: Twist) -> None:
        self.commands[drone_id] = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]

    def _safe_next(self, drone_id: int, candidate: Tuple[float, float]) -> bool:
        if (
            obstacle_clearance(*candidate, self.obstacles)
            < self.radius + 0.03
        ):
            return False
        for other_id, other in enumerate(self.positions):
            if other_id == drone_id:
                continue
            if math.hypot(candidate[0] - other[0], candidate[1] - other[1]) < (
                2.0 * self.radius + 0.05
            ):
                return False
        return True

    def _step(self) -> None:
        self.elapsed += self.dt
        next_positions: List[Tuple[float, float]] = []
        for drone_id in range(self.drone_count):
            desired = limit_norm(
                (self.commands[drone_id][0], self.commands[drone_id][1]),
                self.max_speed,
            )
            dv = (
                desired[0] - self.velocities[drone_id][0],
                desired[1] - self.velocities[drone_id][1],
            )
            dv = limit_norm(dv, self.max_acceleration * self.dt)
            velocity = [
                self.velocities[drone_id][0] + dv[0],
                self.velocities[drone_id][1] + dv[1],
            ]
            candidate = (
                self.positions[drone_id][0] + velocity[0] * self.dt,
                self.positions[drone_id][1] + velocity[1] * self.dt,
            )
            if not self._safe_next(drone_id, candidate):
                velocity = [0.0, 0.0]
                candidate = tuple(self.positions[drone_id])
                self.safety_interventions += 1
            self.velocities[drone_id] = velocity
            next_positions.append(candidate)
            self.yaws[drone_id] = math.atan2(
                math.sin(
                    self.yaws[drone_id]
                    + self.commands[drone_id][2] * self.dt
                ),
                math.cos(
                    self.yaws[drone_id]
                    + self.commands[drone_id][2] * self.dt
                ),
            )
        self.positions = [list(point) for point in next_positions]
        self._update_metrics()
        self._publish_odometry()
        if self.elapsed - self.last_scan >= self.scan_period - 1.0e-9:
            self.last_scan = self.elapsed
            self._publish_scans()
        if int(self.elapsed * 2) != int((self.elapsed - self.dt) * 2):
            self._publish_metrics()

    def _update_metrics(self) -> None:
        for distance in pairwise_distances(
            [tuple(point) for point in self.positions]
        ):
            self.min_inter_drone = min(self.min_inter_drone, distance)
            if distance < 2.0 * self.radius:
                self.collision_events += 1
        for point in self.positions:
            clearance = obstacle_clearance(*point, self.obstacles) - self.radius
            self.min_obstacle_clearance = min(
                self.min_obstacle_clearance, clearance
            )
            if clearance < 0.0:
                self.collision_events += 1

    def _publish_odometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for drone_id in range(self.drone_count):
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
            message.pose.pose.position.x = self.positions[drone_id][0]
            message.pose.pose.position.y = self.positions[drone_id][1]
            message.pose.pose.position.z = self.flight_z
            message.pose.pose.orientation.z = math.sin(
                0.5 * self.yaws[drone_id]
            )
            message.pose.pose.orientation.w = math.cos(
                0.5 * self.yaws[drone_id]
            )
            message.twist.twist.linear.x = self.velocities[drone_id][0]
            message.twist.twist.linear.y = self.velocities[drone_id][1]
            self.odom_publishers[drone_id].publish(message)

    def _publish_scans(self) -> None:
        stamp = self.get_clock().now().to_msg()
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / (self.scan_rays - 1)
        for drone_id in range(self.drone_count):
            message = LaserScan()
            message.header.stamp = stamp
            message.header.frame_id = f"drone_{drone_id}/lidar"
            message.angle_min = angle_min
            message.angle_max = math.pi
            message.angle_increment = angle_increment
            message.range_min = 0.05
            message.range_max = self.scan_range
            message.scan_time = self.scan_period
            message.ranges = simulate_scan(
                tuple(self.positions[drone_id]),
                self.yaws[drone_id],
                self.obstacles,
                ray_count=self.scan_rays,
                max_range=self.scan_range,
            )
            self.scan_publishers[drone_id].publish(message)

    def _publish_metrics(self) -> None:
        metrics = {
            "backend": "mock",
            "scenario": self.scenario_name,
            "elapsed": self.elapsed,
            "collision_events": self.collision_events,
            "safety_interventions": self.safety_interventions,
            "min_inter_drone": self.min_inter_drone,
            "min_obstacle_clearance": self.min_obstacle_clearance,
            "positions": self.positions,
        }
        self.metrics_publisher.publish(
            String(data=json.dumps(metrics, separators=(",", ":")))
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MockSwarmSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
