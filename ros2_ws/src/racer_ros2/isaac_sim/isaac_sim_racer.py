#!/usr/bin/env python3
"""Isaac Sim 5.x adapter for the ROS 2 Humble RACER agents.

The adapter creates a USD scene with collision geometry, visual quadrotors and
planar lidar measurements. Flight commands and sensor/state messages cross the
ROS 2 DDS boundary, so the exploration agents remain normal Humble processes.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import List, Tuple


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-every", type=int, default=3)
    return parser.parse_args()


ARGS = parse_arguments()
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if (PACKAGE_ROOT / "racer_ros2").is_dir():
    sys.path.insert(0, str(PACKAGE_ROOT))

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from racer_ros2.safety import limit_norm  # noqa: E402
from racer_ros2.scenario import (  # noqa: E402
    DRONE_RADIUS,
    FLIGHT_Z,
    STARTS,
    Box2D,
    default_obstacles,
    obstacle_clearance,
    pairwise_distances,
    simulate_scan,
)


def add_cube(
    stage,
    path: str,
    position: Tuple[float, float, float],
    scale: Tuple[float, float, float],
    color: Tuple[float, float, float],
    collision: bool = True,
):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*position))
    cube.AddScaleOp().Set(Gf.Vec3f(*scale))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def build_stage(obstacles: List[Box2D], starts: List[Tuple[float, float]]):
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    world = UsdGeom.Xform.Define(stage, "/World")
    world.GetPrim().SetMetadata("kind", "assembly")

    add_cube(
        stage,
        "/World/Ground",
        (0.0, 0.0, -0.05),
        (20.0, 14.0, 0.1),
        (0.16, 0.18, 0.21),
    )
    for index, obstacle in enumerate(obstacles):
        add_cube(
            stage,
            f"/World/Obstacles/box_{index}",
            (obstacle.cx, obstacle.cy, obstacle.height * 0.5),
            (obstacle.sx, obstacle.sy, obstacle.height),
            (0.32, 0.36, 0.42),
        )

    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(2500.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))

    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, -20.0, 19.0))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(48.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(24.0)

    palette = [
        (0.95, 0.15, 0.12),
        (0.12, 0.55, 1.00),
        (0.12, 0.85, 0.30),
        (0.95, 0.75, 0.10),
    ]
    transforms = []
    for drone_id, start in enumerate(starts):
        root = UsdGeom.Xform.Define(stage, f"/World/Drones/drone_{drone_id}")
        translation = root.AddTranslateOp()
        translation.Set(Gf.Vec3d(start[0], start[1], FLIGHT_Z))
        rotation = root.AddRotateXYZOp()
        rotation.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        color = palette[drone_id % len(palette)]
        add_cube(
            stage,
            f"/World/Drones/drone_{drone_id}/body",
            (0.0, 0.0, 0.0),
            (0.35, 0.22, 0.14),
            color,
            collision=True,
        )
        add_cube(
            stage,
            f"/World/Drones/drone_{drone_id}/arm_x",
            (0.0, 0.0, 0.02),
            (0.75, 0.05, 0.04),
            (0.08, 0.08, 0.08),
            collision=False,
        )
        add_cube(
            stage,
            f"/World/Drones/drone_{drone_id}/arm_y",
            (0.0, 0.0, 0.02),
            (0.05, 0.75, 0.04),
            (0.08, 0.08, 0.08),
            collision=False,
        )
        transforms.append((translation, rotation))
    return stage, transforms


class IsaacRacerBridge(Node):
    def __init__(self, drone_count: int, starts: List[Tuple[float, float]]):
        super().__init__("isaac_racer_bridge")
        self.drone_count = drone_count
        self.positions = [list(point) for point in starts]
        self.velocities = [[0.0, 0.0] for _ in starts]
        self.commands = [[0.0, 0.0, 0.0] for _ in starts]
        self.yaws = [0.0 for _ in starts]
        self.obstacles = default_obstacles()
        self.radius = DRONE_RADIUS
        self.max_speed = 1.2
        self.max_acceleration = 1.5
        self.collision_events = 0
        self.safety_interventions = 0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.elapsed = 0.0
        self.scan_rays = 180
        self.scan_range = 6.0
        self.scan_period = 0.1
        self.last_scan = -math.inf
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_publishers = []
        self.scan_publishers = []
        for drone_id in range(drone_count):
            namespace = f"/drone_{drone_id}"
            self.odom_publishers.append(
                self.create_publisher(Odometry, namespace + "/odom", qos)
            )
            self.scan_publishers.append(
                self.create_publisher(LaserScan, namespace + "/scan", qos)
            )
            self.create_subscription(
                Twist,
                namespace + "/cmd_vel",
                lambda message, index=drone_id: self.command(index, message),
                qos,
            )
        self.metrics_publisher = self.create_publisher(
            String, "/racer/sim_metrics", qos
        )

    def command(self, drone_id: int, message: Twist):
        self.commands[drone_id] = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]

    def safe_next(self, drone_id: int, candidate: Tuple[float, float]) -> bool:
        if obstacle_clearance(*candidate, self.obstacles) < self.radius + 0.03:
            return False
        for other_id, other in enumerate(self.positions):
            if other_id == drone_id:
                continue
            if math.hypot(candidate[0] - other[0], candidate[1] - other[1]) < (
                2.0 * self.radius + 0.05
            ):
                return False
        return True

    def step(self, dt: float):
        self.elapsed += dt
        next_positions = []
        for drone_id in range(self.drone_count):
            desired = limit_norm(
                (self.commands[drone_id][0], self.commands[drone_id][1]),
                self.max_speed,
            )
            delta_velocity = limit_norm(
                (
                    desired[0] - self.velocities[drone_id][0],
                    desired[1] - self.velocities[drone_id][1],
                ),
                self.max_acceleration * dt,
            )
            velocity = [
                self.velocities[drone_id][0] + delta_velocity[0],
                self.velocities[drone_id][1] + delta_velocity[1],
            ]
            candidate = (
                self.positions[drone_id][0] + velocity[0] * dt,
                self.positions[drone_id][1] + velocity[1] * dt,
            )
            if not self.safe_next(drone_id, candidate):
                velocity = [0.0, 0.0]
                candidate = tuple(self.positions[drone_id])
                self.safety_interventions += 1
            self.velocities[drone_id] = velocity
            next_positions.append(candidate)
            self.yaws[drone_id] = math.atan2(
                math.sin(self.yaws[drone_id] + self.commands[drone_id][2] * dt),
                math.cos(self.yaws[drone_id] + self.commands[drone_id][2] * dt),
            )
        self.positions = [list(point) for point in next_positions]
        self.update_metrics()
        self.publish_odometry()
        if self.elapsed - self.last_scan >= self.scan_period - 1.0e-9:
            self.last_scan = self.elapsed
            self.publish_scans()

    def update_metrics(self):
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

    def publish_odometry(self):
        stamp = self.get_clock().now().to_msg()
        for drone_id in range(self.drone_count):
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
            message.pose.pose.position.x = self.positions[drone_id][0]
            message.pose.pose.position.y = self.positions[drone_id][1]
            message.pose.pose.position.z = FLIGHT_Z
            message.pose.pose.orientation.z = math.sin(
                0.5 * self.yaws[drone_id]
            )
            message.pose.pose.orientation.w = math.cos(
                0.5 * self.yaws[drone_id]
            )
            message.twist.twist.linear.x = self.velocities[drone_id][0]
            message.twist.twist.linear.y = self.velocities[drone_id][1]
            self.odom_publishers[drone_id].publish(message)

    def publish_scans(self):
        stamp = self.get_clock().now().to_msg()
        for drone_id in range(self.drone_count):
            message = LaserScan()
            message.header.stamp = stamp
            message.header.frame_id = f"drone_{drone_id}/lidar"
            message.angle_min = -math.pi
            message.angle_max = math.pi
            message.angle_increment = 2.0 * math.pi / (self.scan_rays - 1)
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

    def publish_metrics(self):
        payload = {
            "backend": "isaac_sim",
            "elapsed": self.elapsed,
            "collision_events": self.collision_events,
            "safety_interventions": self.safety_interventions,
            "min_inter_drone": self.min_inter_drone,
            "min_obstacle_clearance": self.min_obstacle_clearance,
            "positions": self.positions,
        }
        self.metrics_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


def main():
    obstacles = default_obstacles()
    starts = list(STARTS[: ARGS.drone_count])
    while len(starts) < ARGS.drone_count:
        starts.append((-8.0, -5.0 + 2.5 * len(starts)))
    _, transforms = build_stage(obstacles, starts)
    simulation_app.update()

    rclpy.init()
    bridge = IsaacRacerBridge(ARGS.drone_count, starts)
    dt = 0.05
    frame = 0
    last_metrics = -math.inf
    wall_started = time.monotonic()
    print(
        f"RACER_ISAAC_READY drones={ARGS.drone_count} "
        f"duration={ARGS.duration:.1f}",
        flush=True,
    )
    try:
        while (
            simulation_app.is_running()
            and time.monotonic() - wall_started < ARGS.duration
        ):
            step_started = time.monotonic()
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.step(dt)
            for drone_id, (translation, rotation) in enumerate(transforms):
                position = bridge.positions[drone_id]
                translation.Set(Gf.Vec3d(position[0], position[1], FLIGHT_Z))
                rotation.Set(
                    Gf.Vec3f(
                        0.0, 0.0, math.degrees(bridge.yaws[drone_id])
                    )
                )
            if bridge.elapsed - last_metrics >= 0.5:
                bridge.publish_metrics()
                last_metrics = bridge.elapsed
            # Updating the app advances the real Isaac USD/rendering pipeline.
            simulation_app.update()
            frame += 1
            remaining = dt - (time.monotonic() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        bridge.publish_metrics()
        result = {
            "collision_events": bridge.collision_events,
            "safety_interventions": bridge.safety_interventions,
            "min_inter_drone": bridge.min_inter_drone,
            "min_obstacle_clearance": bridge.min_obstacle_clearance,
        }
        print("RACER_ISAAC_RESULT " + json.dumps(result, sort_keys=True), flush=True)
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
