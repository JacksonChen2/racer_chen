#!/usr/bin/env python3
"""Isaac Sim PhysX six-DOF Crazyflie plant and 3-D lidar ROS bridge."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence, Tuple


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument(
        "--scenario", default="acceptance_15x9x2"
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-every", type=int, default=5)
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument(
        "--scene-usd",
        type=Path,
        help=(
            "reference an external collision-enabled USD instead of building "
            "the deterministic acceptance obstacles"
        ),
    )
    parser.add_argument(
        "--starts",
        type=float,
        nargs="+",
        help="flat x y z launch positions; provide three values per vehicle",
    )
    parser.add_argument(
        "--control-probe",
        action="store_true",
        help="command one vehicle at 0.35 m/s for isolated 6-DOF testing",
    )
    return parser.parse_args()


ARGS = parse_arguments()
if ARGS.scene_usd is not None:
    ARGS.scene_usd = ARGS.scene_usd.expanduser().resolve()
    if not ARGS.scene_usd.is_file():
        raise SystemExit(f"scene USD does not exist: {ARGS.scene_usd}")
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if (PACKAGE_ROOT / "racer_3d").is_dir():
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
from sensor_msgs.msg import PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from racer_3d.crazyflie import MASS, velocity_wrench  # noqa: E402
from racer_3d.pointcloud import create_xyzi_cloud  # noqa: E402
from racer_3d.scenario import (  # noqa: E402
    DRONE_RADIUS,
    get_scenario,
    obstacle_clearance,
    pairwise_distances,
    point_box_signed_clearance,
)
from racer_3d.safety import (  # noqa: E402
    aabb_obstacle_filter,
    cbf_swarm_filter,
    flight_volume_filter,
    pointcloud_obstacle_filter,
)


SCENARIO = get_scenario(ARGS.scenario)
if ARGS.starts is None:
    STARTS = SCENARIO.starts[:ARGS.drone_count]
else:
    if len(ARGS.starts) != 3 * ARGS.drone_count:
        raise SystemExit(
            "--starts requires exactly three values per configured vehicle"
        )
    STARTS = tuple(
        tuple(float(value) for value in ARGS.starts[index:index + 3])
        for index in range(0, len(ARGS.starts), 3)
    )


PHYSICS_DT = 0.02
LIDAR_PERIOD = 0.50
SAFETY_LIDAR_PERIOD = 0.10
BODY_SIZE = (0.16, 0.16, 0.06)
LIDAR_TRANSLATION = np.asarray((0.0, 0.0, 0.075))
ISAAC_SIM_ROOT = Path(
    os.environ.get("ISAAC_SIM_ROOT", "/home/jackson/isaacsim")
)
CRAZYFLIE_ASSET = ISAAC_SIM_ROOT / (
    "extscache/"
    "omni.warp.core-1.8.2+lx64/warp/examples/assets/crazyflie.usd"
)
USE_REFERENCE_VISUAL = True


def _low_level_safety_description() -> str:
    if ARGS.scene_usd is None:
        return "AABB stopping-distance velocity barrier"
    if SCENARIO.safety_min is not None and SCENARIO.safety_max is not None:
        return "lidar-point plus flight-volume stopping-distance barriers"
    return "lidar-point stopping-distance velocity barrier"


def _external_obstacle_filter(
    preferred: Sequence[float],
    position: Sequence[float],
    points_world: Sequence[Sequence[float]],
    current_velocity: Sequence[float],
) -> np.ndarray:
    result = pointcloud_obstacle_filter(
        preferred,
        position,
        points_world,
        clearance=0.30,
        speed_limit=0.35,
        current_velocity=current_velocity,
    )
    if SCENARIO.safety_min is not None and SCENARIO.safety_max is not None:
        result = flight_volume_filter(
            result,
            position,
            SCENARIO.safety_min,
            SCENARIO.safety_max,
            clearance=0.32,
            speed_limit=0.35,
            current_velocity=current_velocity,
        )
    return np.asarray(result, dtype=float)


def _backend_array_to_numpy(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    if hasattr(value, "cpu"):
        host = value.cpu()
        if hasattr(host, "numpy"):
            return np.asarray(host.numpy())
    return np.asarray(value)


def _add_fixed_cube(
    world: World,
    path: str,
    center: Sequence[float],
    size: Sequence[float],
    color: Sequence[float],
) -> FixedCuboid:
    return world.scene.add(
        FixedCuboid(
            prim_path=path,
            name=path.rsplit("/", 1)[-1],
            position=np.asarray(center, dtype=float),
            scale=np.asarray(size, dtype=float),
            color=np.asarray(color, dtype=float),
        )
    )


def _add_crazyflie(
    world: World, stage, drone_id: int, start: Sequence[float]
) -> Tuple[SingleRigidPrim, RotatingLidarPhysX, ContactSensor]:
    root_path = f"/World/Drones/drone_{drone_id}"
    root = UsdGeom.Xform.Define(stage, root_path)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(MASS)
    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid_api.CreateDisableGravityAttr(False)
    rigid_api.CreateLinearDampingAttr(0.02)
    rigid_api.CreateAngularDampingAttr(0.02)
    PhysxSchema.PhysxContactReportAPI.Apply(
        root.GetPrim()
    ).CreateThresholdAttr(0.0)

    collider = UsdGeom.Cube.Define(stage, root_path + "/body")
    collider.CreateSizeAttr(1.0)
    collider.AddScaleOp().Set(Gf.Vec3f(*BODY_SIZE))
    collider.CreateDisplayOpacityAttr([0.0])
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(
        collider.GetPrim()
    ).CreateThresholdAttr(0.0)

    if USE_REFERENCE_VISUAL and CRAZYFLIE_ASSET.is_file():
        visual = UsdGeom.Xform.Define(stage, root_path + "/crazyflie_visual")
        visual.GetPrim().GetReferences().AddReference(str(CRAZYFLIE_ASSET))
        visual.AddRotateXOp().Set(90.0)

    body = world.scene.add(
        SingleRigidPrim(
            prim_path=root_path,
            name=f"crazyflie_3d_{drone_id}",
            position=np.asarray(start, dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0)),
            mass=MASS,
            reset_xform_properties=True,
        )
    )
    lidar = world.scene.add(
        RotatingLidarPhysX(
            prim_path=root_path + "/lidar",
            name=f"lidar_3d_{drone_id}",
            translation=LIDAR_TRANSLATION,
            rotation_frequency=0.0,
            fov=(360.0, 120.0),
            resolution=(3.0, 5.0),
            # Exclude the Crazyflie's own 0.16 m collision body. Real vehicle
            # drivers apply the same body/self point-cloud mask.
            valid_range=(0.25, 7.0),
        )
    )
    contact = world.scene.add(
        ContactSensor(
            prim_path=root_path + "/body/contact_sensor",
            name=f"contact_3d_{drone_id}",
            dt=PHYSICS_DT,
            min_threshold=1.0e-4,
            max_threshold=1.0e6,
            radius=-1.0,
        )
    )
    return body, lidar, contact


def build_world():
    world = World(
        physics_dt=PHYSICS_DT,
        # Keep one physics step per control update. A larger render dt makes
        # World.step() perform unactuated gravity substeps on rendered frames.
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    colors = (
        (0.16, 0.18, 0.21),
        (0.30, 0.35, 0.42),
        (0.38, 0.31, 0.28),
    )
    if ARGS.scene_usd is None:
        for index, obstacle in enumerate(SCENARIO.obstacles):
            _add_fixed_cube(
                world,
                f"/World/Obstacles/{obstacle.name}_{index}",
                obstacle.center,
                obstacle.size,
                colors[min(2, index // 6)],
            )
    else:
        external = UsdGeom.Xform.Define(stage, "/World/ExternalScene")
        external.GetPrim().GetReferences().AddReference(str(ARGS.scene_usd))
    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(2600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))
    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, -15.0, 12.0))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(24.0)

    bodies, lidars, contacts = [], [], []
    for drone_id, start in enumerate(STARTS):
        body, lidar, contact = _add_crazyflie(
            world, stage, drone_id, start
        )
        bodies.append(body)
        lidars.append(lidar)
        contacts.append(contact)
    world.reset()
    for body in bodies:
        body._rigid_prim_view.enable_gravities()
        # World.reset() advances initialization physics in Isaac Sim 5.1.
        # Clear that transient before the rotor controller starts.
        body.set_linear_velocity(np.zeros(3, dtype=float))
        body.set_angular_velocity(np.zeros(3, dtype=float))
    for lidar in lidars:
        lidar.add_point_cloud_data_to_frame()
    for contact in contacts:
        contact.add_raw_contact_data_to_frame()
    return world, bodies, lidars, contacts


def _yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class IsaacRacer3DBridge(Node):
    def __init__(self, bodies, lidars, contacts) -> None:
        super().__init__("isaac_racer_3d_bridge")
        self.bodies = bodies
        self.lidars = lidars
        self.contacts = contacts
        self.drone_count = len(bodies)
        self.commands = [np.zeros(3) for _ in bodies]
        self.applied_commands = [np.zeros(3) for _ in bodies]
        self.safety_points = [np.empty((0, 3), dtype=float) for _ in bodies]
        self.yaw_commands = [0.0 for _ in bodies]
        self.yaw_targets = [0.0 for _ in bodies]
        self.positions = [np.zeros(3) for _ in bodies]
        self.velocities = [np.zeros(3) for _ in bodies]
        self.path_lengths = [0.0 for _ in bodies]
        self.previous_positions = [None for _ in bodies]
        self.motor_thrusts = [np.zeros(4) for _ in bodies]
        self.elapsed = 0.0
        self.last_cloud = -math.inf
        self.last_safety_cloud = -math.inf
        self.collision_events = 0
        self.contact_active = [False for _ in bodies]
        self.max_contact_force = 0.0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.cloud_frames = 0
        self.raw_diagnostics_printed = False
        self.control_steps = 0
        self.safety_interventions = 0
        self.mission_complete = False
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_publishers = []
        self.cloud_publishers = []
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
        self.create_subscription(
            String,
            "/racer_3d/mission_complete",
            self._mission_complete,
            qos,
        )
        self._read_physics(count_distance=False)

    def _command(self, drone_id: int, message: Twist) -> None:
        self.commands[drone_id] = np.asarray(
            (message.linear.x, message.linear.y, message.linear.z), dtype=float
        )
        self.yaw_targets[drone_id] = float(message.angular.z)

    def _mission_complete(self, message: String) -> None:
        self.mission_complete = message.data.strip().lower() == "true"

    def apply_motor_wrenches(self) -> None:
        self.control_steps += 1
        states = []
        for body in self.bodies:
            position, orientation = body.get_world_pose()
            states.append(
                (
                    np.asarray(position, dtype=float),
                    orientation,
                    np.asarray(body.get_linear_velocity(), dtype=float),
                )
            )
        for drone_id, body in enumerate(self.bodies):
            yaw_error = (
                self.yaw_targets[drone_id]
                - self.yaw_commands[drone_id]
                + math.pi
            ) % (2.0 * math.pi) - math.pi
            self.yaw_commands[drone_id] += float(
                np.clip(yaw_error, -0.15 * PHYSICS_DT, 0.15 * PHYSICS_DT)
            )
            position, orientation, velocity = states[drone_id]
            applied_command = self.commands[drone_id]
            if ARGS.scene_usd is None:
                applied_command = np.asarray(
                    aabb_obstacle_filter(
                        applied_command,
                        position,
                        SCENARIO.obstacles,
                        clearance=0.28,
                        speed_limit=0.35,
                        current_velocity=velocity,
                    ),
                    dtype=float,
                )
            else:
                applied_command = _external_obstacle_filter(
                    applied_command,
                    position,
                    self.safety_points[drone_id],
                    velocity,
                )
            applied_command = np.asarray(
                cbf_swarm_filter(
                    applied_command,
                    position,
                    [
                        (peer_id, peer_position, peer_velocity)
                        for peer_id, (
                            peer_position,
                            _,
                            peer_velocity,
                        ) in enumerate(states)
                        if peer_id != drone_id
                    ],
                    safe_distance=0.55,
                    speed_limit=0.35,
                ),
                dtype=float,
            )
            # Pairwise projection can point toward a nearby wall; make the
            # obstacle barrier the final authority on the combined command.
            if ARGS.scene_usd is None:
                applied_command = np.asarray(
                    aabb_obstacle_filter(
                        applied_command,
                        position,
                        SCENARIO.obstacles,
                        clearance=0.28,
                        speed_limit=0.35,
                        current_velocity=velocity,
                    ),
                    dtype=float,
                )
            else:
                applied_command = _external_obstacle_filter(
                    applied_command,
                    position,
                    self.safety_points[drone_id],
                    velocity,
                )
            if float(
                np.linalg.norm(applied_command - self.commands[drone_id])
            ) > 1.0e-3:
                self.safety_interventions += 1
            self.applied_commands[drone_id] = applied_command
            wrench = velocity_wrench(
                applied_command,
                velocity,
                orientation,
                body.get_angular_velocity(),
                self.yaw_commands[drone_id],
            )
            body._rigid_prim_view.apply_forces(
                forces=wrench.local_force.reshape((1, 3)).astype(np.float32),
                is_global=False,
            )
            body._rigid_prim_view.apply_forces_and_torques_at_pos(
                torques=wrench.local_torque.reshape((1, 3)).astype(np.float32),
                is_global=False,
            )
            self.motor_thrusts[drone_id] = wrench.motor_thrusts
            if (
                ARGS.diagnostics
                and drone_id == 0
                and self.control_steps in (1, 10, 50, 250, 500, 1000, 1500)
            ):
                diag_position, diag_orientation = body.get_world_pose()
                print(
                    "RACER_3D_CONTROL "
                    + json.dumps(
                        {
                            "step": self.control_steps,
                            "mass": float(
                                _backend_array_to_numpy(
                                    body._rigid_prim_view.get_masses()
                                ).reshape(-1)[0]
                            ),
                            "position": np.asarray(diag_position).tolist(),
                            "orientation_wxyz": np.asarray(
                                diag_orientation
                            ).tolist(),
                            "velocity": np.asarray(
                                velocity
                            ).tolist(),
                            "requested_command": self.commands[
                                drone_id
                            ].tolist(),
                            "applied_command": applied_command.tolist(),
                            "angular_velocity": np.asarray(
                                body.get_angular_velocity()
                            ).tolist(),
                            "local_force": wrench.local_force.tolist(),
                            "local_torque": wrench.local_torque.tolist(),
                            "motors": wrench.motor_thrusts.tolist(),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )

    def _read_physics(self, count_distance: bool = True) -> None:
        for drone_id, body in enumerate(self.bodies):
            position, _ = body.get_world_pose()
            velocity = body.get_linear_velocity()
            position = np.asarray(position, dtype=float)
            if count_distance and self.previous_positions[drone_id] is not None:
                step = float(
                    np.linalg.norm(
                        position - self.previous_positions[drone_id]
                    )
                )
                if step < 0.20:
                    self.path_lengths[drone_id] += step
            self.previous_positions[drone_id] = position.copy()
            self.positions[drone_id] = position
            self.velocities[drone_id] = np.asarray(velocity, dtype=float)

    def _update_metrics(self) -> None:
        distances = list(pairwise_distances(self.positions))
        if distances:
            self.min_inter_drone = min(
                self.min_inter_drone, min(distances)
            )
        if ARGS.scene_usd is None:
            for position in self.positions:
                self.min_obstacle_clearance = min(
                    self.min_obstacle_clearance,
                    obstacle_clearance(
                        position, SCENARIO.obstacles
                    )
                    - DRONE_RADIUS,
                )
        else:
            for position, points in zip(
                self.positions, self.safety_points
            ):
                if len(points):
                    self.min_obstacle_clearance = min(
                        self.min_obstacle_clearance,
                        float(
                            np.min(
                                np.linalg.norm(points - position, axis=1)
                            )
                        )
                        - DRONE_RADIUS,
                    )
        for drone_id, sensor in enumerate(self.contacts):
            frame = sensor.get_current_frame()
            force = float(frame.get("force", 0.0))
            active = bool(frame.get("in_contact", False)) and force > 1.0e-4
            if active and not self.contact_active[drone_id]:
                self.collision_events += 1
                nearest_name = "external_usd"
                center_clearance = None
                body_clearance = None
                if ARGS.scene_usd is None:
                    clearances = [
                        point_box_signed_clearance(
                            self.positions[drone_id], obstacle
                        )
                        for obstacle in SCENARIO.obstacles
                    ]
                    nearest_index = int(np.argmin(clearances))
                    nearest_name = SCENARIO.obstacles[
                        nearest_index
                    ].name
                    center_clearance = clearances[nearest_index]
                    body_clearance = center_clearance - DRONE_RADIUS
                print(
                    "RACER_3D_CONTACT "
                    + json.dumps(
                        {
                            "drone_id": drone_id,
                            "elapsed": self.elapsed,
                            "position": self.positions[drone_id].tolist(),
                            "velocity": self.velocities[drone_id].tolist(),
                            "command": self.commands[drone_id].tolist(),
                            "applied_command": self.applied_commands[
                                drone_id
                            ].tolist(),
                            "force": force,
                            "nearest_obstacle": nearest_name,
                            "center_clearance": center_clearance,
                            "body_clearance": body_clearance,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            self.contact_active[drone_id] = active
            self.max_contact_force = max(self.max_contact_force, force)

    def step_observations(self) -> None:
        self.elapsed += PHYSICS_DT
        self._read_physics()
        if (
            self.elapsed - self.last_safety_cloud
            >= SAFETY_LIDAR_PERIOD - 1.0e-9
        ):
            self.last_safety_cloud = self.elapsed
            self._refresh_safety_points()
        self._update_metrics()
        stamp = self.get_clock().now().to_msg()
        self._publish_odometry(stamp)
        if self.elapsed - self.last_cloud >= LIDAR_PERIOD - 1.0e-9:
            self.last_cloud = self.elapsed
            self._publish_clouds(stamp)

    def _publish_odometry(self, stamp) -> None:
        for drone_id, body in enumerate(self.bodies):
            position, orientation = body.get_world_pose()
            velocity = body.get_linear_velocity()
            angular = body.get_angular_velocity()
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
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
            message.twist.twist.angular.x = float(angular[0])
            message.twist.twist.angular.y = float(angular[1])
            message.twist.twist.angular.z = float(angular[2])
            self.odom_publishers[drone_id].publish(message)

    @staticmethod
    def _world_points(
        raw_points: np.ndarray,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        from racer_3d.crazyflie import quaternion_matrix

        values = np.asarray(raw_points, dtype=float).reshape((-1, 3))
        finite = np.all(np.isfinite(values), axis=1)
        values = values[finite]
        ranges = np.linalg.norm(values, axis=1)
        valid = (ranges > 0.24) & (ranges <= 7.05)
        values, ranges = values[valid], ranges[valid]
        rotation = quaternion_matrix(orientation)
        sensor_origin = (
            np.asarray(position, dtype=float) + rotation @ LIDAR_TRANSLATION
        )
        world = values @ rotation.T + sensor_origin
        return world.astype(np.float32), ranges < 6.97

    def _refresh_safety_points(self) -> None:
        """Refresh local collision returns independently of ROS map traffic."""

        for drone_id, (body, lidar) in enumerate(
            zip(self.bodies, self.lidars)
        ):
            raw = lidar.get_current_frame().get("point_cloud")
            if raw is None:
                continue
            raw = _backend_array_to_numpy(raw)
            if raw.size < 12:
                continue
            position, orientation = body.get_world_pose()
            points, hit = self._world_points(raw, position, orientation)
            if len(points) >= 12:
                self.safety_points[drone_id] = np.asarray(
                    points[hit], dtype=float
                )

    def _publish_clouds(self, stamp) -> None:
        for drone_id, (body, lidar) in enumerate(
            zip(self.bodies, self.lidars)
        ):
            raw = lidar.get_current_frame().get("point_cloud")
            if raw is None:
                continue
            raw = _backend_array_to_numpy(raw)
            if raw.size < 12:
                continue
            position, orientation = body.get_world_pose()
            points, hit = self._world_points(raw, position, orientation)
            if len(points) < 12:
                continue
            self.safety_points[drone_id] = np.asarray(
                points[hit], dtype=float
            )
            if ARGS.diagnostics and not self.raw_diagnostics_printed:
                median_error = None
                if ARGS.scene_usd is None:
                    clearances = [
                        abs(
                            obstacle_clearance(
                                point, SCENARIO.obstacles
                            )
                        )
                        for point in points[::max(1, len(points) // 500)]
                    ]
                    median_error = float(np.median(clearances))
                print(
                    "RACER_3D_LIDAR_RAW "
                    + json.dumps(
                        {
                            "raw_shape": list(raw.shape),
                            "world_count": len(points),
                            "rows": int(lidar.get_num_rows()),
                            "cols": int(lidar.get_num_cols()),
                            "median_surface_error_m": median_error,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                self.raw_diagnostics_printed = True
            self.cloud_publishers[drone_id].publish(
                create_xyzi_cloud(stamp, "map", points, hit)
            )
            self.cloud_frames += 1

    def publish_metrics(self) -> None:
        payload = {
            "backend": "isaac_sim_physx_3d",
            "scenario": SCENARIO.name,
            "scene_usd": (
                None if ARGS.scene_usd is None else str(ARGS.scene_usd)
            ),
            "vehicle_model": "Crazyflie 2.x 27g six-DOF rigid body",
            "motion_source": "local rotor thrust and attitude torque",
            "sensor_source": "Isaac RotatingLidarPhysX point cloud",
            "elapsed": self.elapsed,
            "collision_events": self.collision_events,
            "physics_contact_events": self.collision_events,
            "max_contact_force": self.max_contact_force,
            "min_inter_drone": self.min_inter_drone,
            "min_obstacle_clearance": self.min_obstacle_clearance,
            "path_lengths": self.path_lengths,
            "positions": [point.tolist() for point in self.positions],
            "motor_thrusts_n": [
                values.tolist() for values in self.motor_thrusts
            ],
            "point_cloud_frames": self.cloud_frames,
            "safety_point_refresh_hz": 1.0 / SAFETY_LIDAR_PERIOD,
            "safety_interventions": self.safety_interventions,
            "low_level_safety": _low_level_safety_description(),
        }
        self.metrics_publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )


def main() -> None:
    world, bodies, lidars, contacts = build_world()
    rclpy.init()
    bridge = IsaacRacer3DBridge(bodies, lidars, contacts)
    # Prime range sensors while each gravity-enabled vehicle is held by the
    # same motor/attitude controller used during the actual run.
    for _ in range(8):
        bridge.apply_motor_wrenches()
        world.step(render=False)
    # Sensor prims need warm-up physics, but that warm-up is not part of the
    # experiment. Restore the exact launch state after callbacks are active.
    for drone_id, body in enumerate(bodies):
        body.set_world_pose(
            position=np.asarray(STARTS[drone_id], dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float),
        )
        body.set_linear_velocity(np.zeros(3, dtype=float))
        body.set_angular_velocity(np.zeros(3, dtype=float))
        bridge.previous_positions[drone_id] = np.asarray(
            STARTS[drone_id], dtype=float
        )
        bridge.contact_active[drone_id] = False
    bridge.collision_events = 0
    bridge.max_contact_force = 0.0
    bridge.elapsed = 0.0
    bridge.last_cloud = -math.inf
    bridge.last_safety_cloud = -math.inf
    bridge.cloud_frames = 0
    bridge.control_steps = 0
    bridge.safety_interventions = 0
    bridge.applied_commands = [np.zeros(3) for _ in bodies]
    bridge.positions = [
        np.asarray(point, dtype=float) for point in STARTS
    ]
    bridge.velocities = [np.zeros(3, dtype=float) for _ in bodies]
    bridge.min_inter_drone = math.inf
    bridge.min_obstacle_clearance = math.inf
    bridge.path_lengths = [0.0 for _ in bodies]
    if ARGS.control_probe:
        bridge.commands[0] = np.asarray((0.30, 0.20, 0.10), dtype=float)
        bridge.yaw_targets[0] = 2.0
    frame = 0
    last_metrics = -math.inf
    print(
        f"RACER_3D_ISAAC_READY drones={len(bodies)} "
        f"duration={ARGS.duration:.1f} motion=rotor_wrench "
        "sensor=RotatingLidarPhysX "
        f"scene={ARGS.scene_usd or SCENARIO.name}",
        flush=True,
    )
    try:
        while (
            simulation_app.is_running()
            and bridge.elapsed < ARGS.duration
            and not bridge.mission_complete
        ):
            step_started = time.monotonic()
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.apply_motor_wrenches()
            world.step(
                render=(
                    not ARGS.headless
                    or frame % max(1, ARGS.render_every) == 0
                )
            )
            bridge.step_observations()
            if bridge.elapsed - last_metrics >= 0.5:
                bridge.publish_metrics()
                last_metrics = bridge.elapsed
            frame += 1
            remaining = PHYSICS_DT - (time.monotonic() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        bridge.commands = [np.zeros(3) for _ in bridge.commands]
        bridge.publish_metrics()
        print(
            "RACER_3D_ISAAC_RESULT "
            + json.dumps(
                {
                    "backend": "isaac_sim_physx_3d",
                    "collision_events": bridge.collision_events,
                    "physics_contact_events": bridge.collision_events,
                    "max_contact_force": bridge.max_contact_force,
                    "min_inter_drone": bridge.min_inter_drone,
                    "min_obstacle_clearance": bridge.min_obstacle_clearance,
                    "path_lengths": bridge.path_lengths,
                    "positions": [
                        point.tolist() for point in bridge.positions
                    ],
                    "point_cloud_frames": bridge.cloud_frames,
                    "safety_point_refresh_hz": (
                        1.0 / SAFETY_LIDAR_PERIOD
                    ),
                    "safety_interventions": bridge.safety_interventions,
                    "low_level_safety": _low_level_safety_description(),
                },
                sort_keys=True,
            ),
            flush=True,
        )
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
