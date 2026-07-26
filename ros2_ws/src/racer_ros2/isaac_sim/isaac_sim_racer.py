#!/usr/bin/env python3
"""Real Isaac Sim/PhysX plant and sensor bridge for the ROS 2 agents.

Unlike the original prototype adapter, this module never integrates position
itself and never synthesizes lidar ranges from the scenario description.
Commands drive Isaac rigid bodies, odometry is read back from PhysX, scans come
from ``RotatingLidarPhysX``, and collision events come from contact sensors.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import List, Sequence, Tuple


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument(
        "--scenario", choices=("small", "large", "long"), default="small"
    )
    parser.add_argument(
        "--contact-probe",
        action="store_true",
        help="drive one rigid body into a wall to validate contact reporting",
    )
    parser.add_argument(
        "--scan-diagnostics",
        action="store_true",
        help="print the first raw PhysX lidar buffer shape and angle range",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-every", type=int, default=4)
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
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from isaacsim.sensors.physics import ContactSensor  # noqa: E402
from isaacsim.sensors.physx import RotatingLidarPhysX  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from racer_ros2.safety import limit_norm  # noqa: E402
from racer_ros2.scenario import (  # noqa: E402
    DRONE_RADIUS,
    Scenario,
    get_scenario,
    obstacle_clearance,
    pairwise_distances,
    ray_box_distance,
)


PHYSICS_DT = 0.05
CRAZYFLIE_MASS = 0.027
BODY_SIZE = (0.16, 0.16, 0.06)
CRAZYFLIE_ASSET = Path(
    "/home/jackson/isaacsim/extscache/"
    "omni.warp.core-1.8.2+lx64/warp/examples/assets/crazyflie.usd"
)


def _add_fixed_cube(
    world: World,
    path: str,
    position: Sequence[float],
    scale: Sequence[float],
    color: Sequence[float],
) -> FixedCuboid:
    return world.scene.add(
        FixedCuboid(
            prim_path=path,
            name=path.rsplit("/", 1)[-1],
            position=np.asarray(position, dtype=float),
            scale=np.asarray(scale, dtype=float),
            color=np.asarray(color, dtype=float),
        )
    )


def _add_crazyflie(
    world: World,
    stage,
    drone_id: int,
    start: Tuple[float, float],
    flight_z: float,
    lidar_range: float,
) -> Tuple[SingleRigidPrim, RotatingLidarPhysX, ContactSensor]:
    root_path = f"/World/Drones/drone_{drone_id}"
    root = UsdGeom.Xform.Define(stage, root_path)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(root.GetPrim())
    mass_api.CreateMassAttr(CRAZYFLIE_MASS)
    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid_api.CreateDisableGravityAttr(True)
    rigid_api.CreateLinearDampingAttr(0.15)
    rigid_api.CreateAngularDampingAttr(0.20)
    root_contact_report = PhysxSchema.PhysxContactReportAPI.Apply(
        root.GetPrim()
    )
    root_contact_report.CreateThresholdAttr(0.0)

    collider = UsdGeom.Cube.Define(stage, root_path + "/body")
    collider.CreateSizeAttr(1.0)
    collider.AddScaleOp().Set(Gf.Vec3f(*BODY_SIZE))
    collider.CreateDisplayOpacityAttr([0.0])
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    contact_report = PhysxSchema.PhysxContactReportAPI.Apply(
        collider.GetPrim()
    )
    contact_report.CreateThresholdAttr(0.0)

    if CRAZYFLIE_ASSET.is_file():
        visual = UsdGeom.Xform.Define(stage, root_path + "/crazyflie_visual")
        visual.GetPrim().GetReferences().AddReference(str(CRAZYFLIE_ASSET))
        # The bundled Warp asset is Y-up. Rotate it into this Z-up world.
        visual.AddRotateXOp().Set(90.0)

    rigid_body = world.scene.add(
        SingleRigidPrim(
            prim_path=root_path,
            name=f"crazyflie_{drone_id}",
            position=np.asarray((start[0], start[1], flight_z), dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float),
            mass=CRAZYFLIE_MASS,
            reset_xform_properties=True,
        )
    )
    lidar = world.scene.add(
        RotatingLidarPhysX(
            prim_path=root_path + "/lidar",
            name=f"lidar_{drone_id}",
            translation=np.asarray((0.0, 0.0, 0.075), dtype=float),
            # Zero rotation rate asks the PhysX range sensor for a complete
            # 360-degree buffer every physics update (as in NVIDIA's tests).
            rotation_frequency=0.0,
            fov=(360.0, 2.0),
            resolution=(2.0, 2.0),
            valid_range=(0.05, lidar_range),
        )
    )
    contact = world.scene.add(
        ContactSensor(
            prim_path=root_path + "/body/contact_sensor",
            name=f"contact_{drone_id}",
            dt=PHYSICS_DT,
            min_threshold=1.0e-4,
            max_threshold=1.0e6,
            radius=-1.0,
        )
    )
    return rigid_body, lidar, contact


def build_world(
    scenario: Scenario, starts: List[Tuple[float, float]]
) -> Tuple[
    World,
    List[SingleRigidPrim],
    List[RotatingLidarPhysX],
    List[ContactSensor],
]:
    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    size = scenario.map_size
    _add_fixed_cube(
        world,
        "/World/Ground",
        (0.0, 0.0, -0.05),
        (size[0], size[1], 0.10),
        (0.16, 0.18, 0.21),
    )
    for index, obstacle in enumerate(scenario.obstacles):
        _add_fixed_cube(
            world,
            f"/World/Obstacles/box_{index}",
            (obstacle.cx, obstacle.cy, obstacle.height * 0.5),
            (obstacle.sx, obstacle.sy, obstacle.height),
            (0.32, 0.36, 0.42),
        )

    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(2500.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))
    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.AddTranslateOp().Set(
        Gf.Vec3d(0.0, -1.6 * size[1], max(9.0, 1.2 * size[1]))
    )
    camera.AddRotateXYZOp().Set(Gf.Vec3f(50.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(24.0)

    bodies: List[SingleRigidPrim] = []
    lidars: List[RotatingLidarPhysX] = []
    contacts: List[ContactSensor] = []
    for drone_id, start in enumerate(starts):
        body, lidar, contact = _add_crazyflie(
            world,
            stage,
            drone_id,
            start,
            scenario.flight_z,
            3.5 if scenario.name == "small" else 6.0,
        )
        bodies.append(body)
        lidars.append(lidar)
        contacts.append(contact)

    world.reset()
    for body in bodies:
        body._rigid_prim_view.disable_gravities()
    for lidar in lidars:
        lidar.add_depth_data_to_frame()
        lidar.add_azimuth_data_to_frame()
    for contact in contacts:
        contact.add_raw_contact_data_to_frame()
    # Prime physics-backed sensors before ROS consumers start mapping.
    for _ in range(4):
        world.step(render=False)
    return world, bodies, lidars, contacts


def _yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(item) for item in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _backend_array_to_numpy(value) -> np.ndarray:
    """Convert Isaac's NumPy, Torch, or Warp backend arrays safely."""

    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    if hasattr(value, "cpu"):
        host_value = value.cpu()
        if hasattr(host_value, "numpy"):
            return np.asarray(host_value.numpy())
    return np.asarray(value)


class IsaacRacerBridge(Node):
    """ROS interface around the actual Isaac rigid bodies and sensors."""

    def __init__(
        self,
        scenario: Scenario,
        bodies: List[SingleRigidPrim],
        lidars: List[RotatingLidarPhysX],
        contacts: List[ContactSensor],
    ):
        super().__init__("isaac_racer_bridge")
        self.scenario = scenario
        self.bodies = bodies
        self.lidars = lidars
        self.contacts = contacts
        self.drone_count = len(bodies)
        self.positions = [[0.0, 0.0] for _ in bodies]
        self.velocities = [[0.0, 0.0] for _ in bodies]
        self.commands = [[0.0, 0.0, 0.0] for _ in bodies]
        self.yaws = [0.0 for _ in bodies]
        self.radius = DRONE_RADIUS
        self.max_speed = 1.2
        self.max_acceleration = 1.5
        self.collision_events = 0
        self.contact_active = [False for _ in bodies]
        self.max_contact_force = 0.0
        self.safety_interventions = 0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.elapsed = 0.0
        self.scan_period = 0.1
        self.last_scan = -math.inf
        self.lidar_frames = 0
        self.scan_diagnostic_printed = False
        # RotatingLidarPhysX exposes the completed depth buffer two physics
        # callbacks after the rigid-body pose that generated it. ROS odometry
        # is delayed by the same amount so each LaserScan is projected from
        # its actual acquisition pose.
        self.sensor_latency_steps = 2
        self.pose_history = [[] for _ in bodies]
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_publishers = []
        self.scan_publishers = []
        for drone_id in range(self.drone_count):
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
        self.read_physics()

    def command(self, drone_id: int, message: Twist) -> None:
        self.commands[drone_id] = [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ]

    def apply_commands(self, dt: float) -> None:
        """Apply an acceleration-limited velocity setpoint to each rigid body."""

        for drone_id, body in enumerate(self.bodies):
            current = body.get_linear_velocity()
            desired = limit_norm(
                (self.commands[drone_id][0], self.commands[drone_id][1]),
                self.max_speed,
            )
            delta = limit_norm(
                (
                    desired[0] - float(current[0]),
                    desired[1] - float(current[1]),
                ),
                self.max_acceleration * dt,
            )
            _, pose_orientation = body.get_world_pose()
            yaw = _yaw_from_quaternion(pose_orientation)
            _, _, current_z = body.get_world_pose()[0]
            z_velocity = float(
                np.clip(
                    4.0 * (self.scenario.flight_z - float(current_z)),
                    -0.6,
                    0.6,
                )
            )
            body.set_linear_velocity(
                np.asarray(
                    (
                        float(current[0]) + delta[0],
                        float(current[1]) + delta[1],
                        z_velocity,
                    ),
                    dtype=float,
                )
            )
            yaw_rate = float(np.clip(self.commands[drone_id][2], -1.5, 1.5))
            body.set_angular_velocity(
                np.asarray((0.0, 0.0, yaw_rate), dtype=float)
            )
            self.yaws[drone_id] = yaw

    def read_physics(self) -> None:
        for drone_id, body in enumerate(self.bodies):
            position, orientation = body.get_world_pose()
            velocity = body.get_linear_velocity()
            self.positions[drone_id] = [
                float(position[0]),
                float(position[1]),
            ]
            self.velocities[drone_id] = [
                float(velocity[0]),
                float(velocity[1]),
            ]
            self.yaws[drone_id] = _yaw_from_quaternion(orientation)
            history = self.pose_history[drone_id]
            history.append(
                (
                    np.asarray(position, dtype=float).copy(),
                    np.asarray(orientation, dtype=float).copy(),
                    np.asarray(velocity, dtype=float).copy(),
                )
            )
            if len(history) > self.sensor_latency_steps + 4:
                del history[0]

    def sensor_pose_sample(
        self, drone_id: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        history = self.pose_history[drone_id]
        index = max(0, len(history) - self.sensor_latency_steps - 1)
        return history[index]

    def update_metrics(self) -> None:
        for distance in pairwise_distances(
            [tuple(point) for point in self.positions]
        ):
            self.min_inter_drone = min(self.min_inter_drone, distance)
        for point in self.positions:
            clearance = (
                obstacle_clearance(
                    point[0], point[1], self.scenario.obstacles
                )
                - self.radius
            )
            self.min_obstacle_clearance = min(
                self.min_obstacle_clearance, clearance
            )
        for drone_id, sensor in enumerate(self.contacts):
            frame = sensor.get_current_frame()
            force = float(frame.get("force", 0.0))
            active = bool(frame.get("in_contact", False)) and force > 1.0e-4
            self.max_contact_force = max(
                self.max_contact_force, force
            )
            if active and not self.contact_active[drone_id]:
                self.collision_events += 1
            self.contact_active[drone_id] = active

    def step_observations(self, dt: float) -> None:
        self.elapsed += dt
        self.read_physics()
        self.update_metrics()
        stamp = self.get_clock().now().to_msg()
        self.publish_odometry(stamp)
        if self.elapsed - self.last_scan >= self.scan_period - 1.0e-9:
            self.last_scan = self.elapsed
            self.publish_scans(stamp)

    def publish_odometry(self, stamp) -> None:
        for drone_id in range(self.drone_count):
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
            position, orientation, velocity = self.sensor_pose_sample(drone_id)
            message.pose.pose.position.x = float(position[0])
            message.pose.pose.position.y = float(position[1])
            message.pose.pose.position.z = float(position[2])
            message.pose.pose.orientation.w = float(orientation[0])
            message.pose.pose.orientation.x = float(orientation[1])
            message.pose.pose.orientation.y = float(orientation[2])
            message.pose.pose.orientation.z = float(orientation[3])
            message.twist.twist.linear.x = float(velocity[0])
            message.twist.twist.linear.y = float(velocity[1])
            message.twist.twist.linear.z = float(velocity[2])
            self.odom_publishers[drone_id].publish(message)

    @staticmethod
    def _scan_from_lidar(
        lidar: RotatingLidarPhysX,
    ) -> Tuple[np.ndarray, np.ndarray] | None:
        frame = lidar.get_current_frame()
        depth = frame.get("depth")
        azimuth = frame.get("azimuth")
        if depth is None or azimuth is None:
            return None
        ranges = _backend_array_to_numpy(depth).astype(float, copy=False)
        _, maximum_range = lidar.get_valid_range()
        ranges *= float(maximum_range) / 65535.0
        angles = _backend_array_to_numpy(azimuth).astype(
            float, copy=False
        ).reshape(-1)
        if ranges.size == 0 or angles.size == 0:
            return None
        # PhysX may expose an incomplete rotating column while a scan buffer is
        # being swapped. Treat invalid depth as a miss and drop invalid angles.
        ranges = np.where(np.isfinite(ranges), ranges, float(maximum_range))
        if ranges.ndim > 1:
            ranges = np.min(ranges, axis=tuple(range(1, ranges.ndim)))
        ranges = ranges.reshape(-1)
        count = min(ranges.size, angles.size)
        if count < 8:
            return None
        ranges = ranges[:count]
        angles = angles[:count]
        valid = np.isfinite(angles)
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size < 8:
            return None
        angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
        order = np.argsort(angles)
        return angles[order], np.clip(
            ranges[order], 0.05, float(maximum_range)
        )

    def publish_scans(self, stamp) -> None:
        for drone_id, lidar in enumerate(self.lidars):
            scan = self._scan_from_lidar(lidar)
            if scan is None:
                continue
            if (
                ARGS.scan_diagnostics
                and not self.scan_diagnostic_printed
                and self.elapsed >= 0.45
            ):
                raw = lidar.get_current_frame()
                raw_depth_value = raw.get("depth")
                raw_azimuth_value = raw.get("azimuth")
                raw_depth = _backend_array_to_numpy(raw_depth_value)
                raw_azimuth = _backend_array_to_numpy(raw_azimuth_value)
                angles, measured_ranges = scan
                position, orientation, _ = self.sensor_pose_sample(drone_id)
                yaw = _yaw_from_quaternion(orientation)
                _, current_orientation = self.bodies[
                    drone_id
                ].get_world_pose()
                current_yaw = _yaw_from_quaternion(current_orientation)

                def expected_ranges(yaw_offset: float | None) -> np.ndarray:
                    result = []
                    for angle in angles:
                        world_angle = float(angle) + (
                            yaw + yaw_offset
                            if yaw_offset is not None
                            else 0.0
                        )
                        result.append(
                            min(
                                float(lidar.get_valid_range()[1]),
                                min(
                                    ray_box_distance(
                                        float(position[0]),
                                        float(position[1]),
                                        math.cos(world_angle),
                                        math.sin(world_angle),
                                        obstacle,
                                    )
                                    for obstacle in self.scenario.obstacles
                                ),
                            )
                        )
                    return np.asarray(result)

                offset_errors = {
                    offset: float(
                        np.mean(
                            np.abs(
                                measured_ranges - expected_ranges(offset)
                            )
                        )
                    )
                    for offset in np.linspace(-0.20, 0.20, 17)
                }
                best_offset = min(offset_errors, key=offset_errors.get)
                expected_local = expected_ranges(0.0)
                expected_world = expected_ranges(None)
                print(
                    "RACER_LIDAR_RAW "
                    + json.dumps(
                        {
                            "depth_shape": raw_depth.shape,
                            "azimuth_shape": raw_azimuth.shape,
                            "depth_min": float(
                                np.nanmin(
                                    raw_depth.astype(float)
                                    * lidar.get_valid_range()[1]
                                    / 65535.0
                                )
                            ),
                            "depth_max": float(
                                np.nanmax(
                                    raw_depth.astype(float)
                                    * lidar.get_valid_range()[1]
                                    / 65535.0
                                )
                            ),
                            "azimuth_min": float(np.nanmin(raw_azimuth)),
                            "azimuth_max": float(np.nanmax(raw_azimuth)),
                            "rows": int(lidar.get_num_rows()),
                            "cols": int(lidar.get_num_cols()),
                            "depth_type": type(raw_depth_value).__name__,
                            "azimuth_type": type(raw_azimuth_value).__name__,
                            "sensor_pose_yaw": yaw,
                            "current_body_yaw": current_yaw,
                            "mean_error_if_local_angles": float(
                                np.mean(
                                    np.abs(measured_ranges - expected_local)
                                )
                            ),
                            "mean_error_if_world_angles": float(
                                np.mean(
                                    np.abs(measured_ranges - expected_world)
                                )
                            ),
                            "best_yaw_offset": float(best_offset),
                            "best_mean_error": offset_errors[best_offset],
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                self.scan_diagnostic_printed = True
            angles, ranges = scan
            increments = np.diff(angles)
            angle_increment = (
                float(np.median(increments))
                if increments.size
                else 2.0 * math.pi
            )
            message = LaserScan()
            message.header.stamp = stamp
            message.header.frame_id = f"drone_{drone_id}/lidar"
            message.angle_min = float(angles[0])
            message.angle_max = float(angles[-1])
            message.angle_increment = angle_increment
            message.range_min = 0.05
            message.range_max = float(lidar.get_valid_range()[1])
            message.scan_time = self.scan_period
            message.ranges = ranges.astype(float).tolist()
            self.scan_publishers[drone_id].publish(message)
            self.lidar_frames += 1

    def publish_metrics(self) -> None:
        payload = {
            "backend": "isaac_sim_physx",
            "scenario": self.scenario.name,
            "vehicle_model": "Crazyflie 2.x rigid-body proxy",
            "motion_source": "Isaac PhysX rigid body",
            "sensor_source": "Isaac RotatingLidarPhysX",
            "elapsed": self.elapsed,
            "collision_events": self.collision_events,
            "physics_contact_events": self.collision_events,
            "max_contact_force": self.max_contact_force,
            "safety_interventions": self.safety_interventions,
            "lidar_frames": self.lidar_frames,
            "min_inter_drone": self.min_inter_drone,
            "min_obstacle_clearance": self.min_obstacle_clearance,
            "positions": self.positions,
        }
        self.metrics_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


def main() -> None:
    scenario = get_scenario(ARGS.scenario)
    starts = list(scenario.starts[: ARGS.drone_count])
    if ARGS.contact_probe:
        ARGS.drone_count = 1
        starts = [
            (
                scenario.map_max[0] - 0.55,
                scenario.map_min[1] + 1.0,
            )
        ]
    while len(starts) < ARGS.drone_count:
        starts.append(
            (
                scenario.map_min[0] + 1.0,
                scenario.map_min[1] + 1.0 + 0.7 * len(starts),
            )
        )
    world, bodies, lidars, contacts = build_world(scenario, starts)

    rclpy.init()
    bridge = IsaacRacerBridge(scenario, bodies, lidars, contacts)
    if ARGS.contact_probe:
        bridge.commands[0] = [0.8, 0.0, 0.0]
    elif ARGS.scan_diagnostics:
        # Rotate during the coordinate-frame diagnostic so local and world
        # azimuth interpretations can be distinguished. Translate at the same
        # time to validate the complete acquisition pose.
        bridge.commands[0] = [0.8, 0.0, 1.5]
    frame = 0
    last_metrics = -math.inf
    wall_started = time.monotonic()
    print(
        f"RACER_ISAAC_READY drones={ARGS.drone_count} "
        f"scenario={scenario.name} duration={ARGS.duration:.1f} "
        "motion=physx sensor=RotatingLidarPhysX "
        f"contact_probe={ARGS.contact_probe}",
        flush=True,
    )
    try:
        while (
            simulation_app.is_running()
            and time.monotonic() - wall_started < ARGS.duration
        ):
            step_started = time.monotonic()
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.apply_commands(PHYSICS_DT)
            world.step(
                render=(
                    not ARGS.headless
                    or frame % max(1, ARGS.render_every) == 0
                )
            )
            bridge.step_observations(PHYSICS_DT)
            if bridge.elapsed - last_metrics >= 0.5:
                bridge.publish_metrics()
                last_metrics = bridge.elapsed
            frame += 1
            remaining = PHYSICS_DT - (time.monotonic() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        for body in bodies:
            body.set_linear_velocity(np.zeros(3, dtype=float))
            body.set_angular_velocity(np.zeros(3, dtype=float))
        bridge.publish_metrics()
        result = {
            "backend": "isaac_sim_physx",
            "scenario": scenario.name,
            "collision_events": bridge.collision_events,
            "physics_contact_events": bridge.collision_events,
            "max_contact_force": bridge.max_contact_force,
            "safety_interventions": bridge.safety_interventions,
            "lidar_frames": bridge.lidar_frames,
            "min_inter_drone": bridge.min_inter_drone,
            "min_obstacle_clearance": bridge.min_obstacle_clearance,
        }
        print(
            "RACER_ISAAC_RESULT " + json.dumps(result, sort_keys=True),
            flush=True,
        )
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
