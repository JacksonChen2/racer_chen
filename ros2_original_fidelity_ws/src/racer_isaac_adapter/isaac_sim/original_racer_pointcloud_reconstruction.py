#!/usr/bin/env python3
"""Reconstruct a growing 3-D point-cloud map from a recorded RACER flight.

This is intentionally a sensor replay, not a planner rerun.  Recorded vehicle
poses drive the same Warehouse Simple USD and pinhole depth-camera model used
by the source-fidelity experiment.  Surface hits are accumulated in 0.1 m
voxels, published to RViz, and stored with their first-observed timestamps so
the reconstructed mapping process can be replayed later without Isaac Sim.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
from pathlib import Path
import time


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--scene-usd", type=Path, required=True)
    parser.add_argument("--vehicle-usd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--sensor-rate-hz", type=float, default=30.0)
    parser.add_argument("--publish-rate-hz", type=float, default=5.0)
    parser.add_argument("--depth-width", type=int, default=320)
    parser.add_argument("--depth-height", type=int, default=240)
    parser.add_argument("--ray-budget", type=int, default=16000)
    parser.add_argument("--voxel-size", type=float, default=0.1)
    parser.add_argument("--end-time", type=float)
    parser.add_argument(
        "--hold-final",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="keep the final transient-local cloud alive for RViz",
    )
    return parser.parse_args()


ARGS = parse_arguments()
for name in ("trajectory", "scene_usd", "vehicle_usd"):
    value = getattr(ARGS, name).expanduser().resolve()
    if not value.is_file():
        raise SystemExit(f"{name.replace('_', ' ')} does not exist: {value}")
    setattr(ARGS, name, value)
ARGS.output = ARGS.output.expanduser().resolve()
if ARGS.output.suffix != ".npz":
    ARGS.output = ARGS.output.with_suffix(".npz")
if min(ARGS.speed, ARGS.sensor_rate_hz, ARGS.publish_rate_hz, ARGS.voxel_size) <= 0:
    raise SystemExit("speed, rates, and voxel size must be positive")
if ARGS.depth_width < 64 or ARGS.depth_height < 48 or ARGS.ray_budget <= 0:
    raise SystemExit("invalid depth resolution or ray budget")

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.syntheticdata as syntheticdata  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Point, PoseStamped  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from nav_msgs.msg import Path as RosPath  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402


DEPTH_MIN_RANGE = 0.2
DEPTH_MAP_RANGE = 4.6
SELF_FILTER_RADIUS = 0.332
MAP_MIN = np.asarray((-10.0, -11.9, 0.4), dtype=np.float64)
MAP_MAX = np.asarray((9.0, 17.6, 8.6), dtype=np.float64)
FIELDS_XYZI = (
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
)
PATH_COLORS = (
    (0.95, 0.20, 0.18, 1.0),
    (0.20, 0.85, 0.30, 1.0),
    (0.18, 0.48, 1.00, 1.0),
    (1.00, 0.78, 0.12, 1.0),
    (0.92, 0.25, 0.92, 1.0),
)


def normalized_quaternion(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if norm < 1.0e-12:
        return np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64)
    return result / norm


def quaternion_slerp(first: np.ndarray, second: np.ndarray, fraction: float) -> np.ndarray:
    a = normalized_quaternion(first)
    b = normalized_quaternion(second)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return normalized_quaternion(a + fraction * (b - a))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return (
        math.sin((1.0 - fraction) * angle) / sine * a
        + math.sin(fraction * angle) / sine * b
    )


def quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = normalized_quaternion(quaternion)
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


class RecordedTrajectory:
    def __init__(self, path: Path) -> None:
        with np.load(path, allow_pickle=False) as archive:
            self.times = np.asarray(archive["times"], dtype=np.float64)
            self.positions = np.asarray(archive["positions"], dtype=np.float64)
            self.orientations = np.asarray(
                archive["orientations_wxyz"], dtype=np.float64
            )
            self.metadata = json.loads(str(archive["metadata_json"].item()))
        if len(self.times) < 2 or np.any(np.diff(self.times) <= 0.0):
            raise RuntimeError("trajectory timestamps must be strictly increasing")
        if self.positions.shape != (len(self.times), self.positions.shape[1], 3):
            raise RuntimeError("invalid trajectory position shape")
        if self.orientations.shape != (
            len(self.times), self.positions.shape[1], 4
        ):
            raise RuntimeError("invalid trajectory orientation shape")
        self.drone_count = int(self.positions.shape[1])
        self.start = float(self.times[0])
        self.end = float(self.times[-1])
        self.duration = self.end - self.start

    def interpolate(self, relative_time: float) -> tuple[np.ndarray, np.ndarray]:
        source_time = self.start + float(relative_time)
        if source_time <= self.start:
            return self.positions[0], self.orientations[0]
        if source_time >= self.end:
            return self.positions[-1], self.orientations[-1]
        after = int(np.searchsorted(self.times, source_time, side="right"))
        before = after - 1
        fraction = float(
            (source_time - self.times[before])
            / (self.times[after] - self.times[before])
        )
        positions = self.positions[before] + fraction * (
            self.positions[after] - self.positions[before]
        )
        orientations = np.asarray(
            [
                quaternion_slerp(
                    self.orientations[before, drone],
                    self.orientations[after, drone],
                    fraction,
                )
                for drone in range(self.drone_count)
            ],
            dtype=np.float64,
        )
        return positions, orientations


def create_xyzi_cloud(
    stamp, points: np.ndarray, intensity: np.ndarray
) -> PointCloud2:
    values = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    weights = np.asarray(intensity, dtype=np.float32).reshape((-1, 1))
    matrix = np.concatenate((values, weights), axis=1)
    return point_cloud2.create_cloud(
        Header(stamp=stamp, frame_id="world"), FIELDS_XYZI, matrix
    )


class PointCloudMapPublisher(Node):
    def __init__(self, trajectory: RecordedTrajectory) -> None:
        super().__init__("racer_pointcloud_reconstruction")
        self.trajectory = trajectory
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(
            PointCloud2, "/racer_pointcloud_reconstruction/map", latched
        )
        self.current_publisher = self.create_publisher(
            PointCloud2, "/racer_pointcloud_reconstruction/current_hits", reliable
        )
        self.status_publisher = self.create_publisher(
            MarkerArray, "/racer_pointcloud_reconstruction/status", reliable
        )
        self.path_publishers = [
            self.create_publisher(
                RosPath, f"/racer_pointcloud_reconstruction/drone_{drone}/path", reliable
            )
            for drone in range(trajectory.drone_count)
        ]
        self.clock_publisher = self.create_publisher(Clock, "/clock", reliable)
        self.lookup: dict[tuple[int, int, int], int] = {}
        self.indices: list[tuple[int, int, int]] = []
        self.first_seen: list[float] = []
        self.first_agent: list[int] = []
        self.agent_masks: list[int] = []
        self.last_publish = -math.inf

    @staticmethod
    def stamp(relative_time: float):
        seconds = int(math.floor(relative_time))
        nanoseconds = int(round((relative_time - seconds) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        clock = Clock()
        clock.clock.sec = seconds
        clock.clock.nanosec = nanoseconds
        return clock

    def add_hits(self, points: np.ndarray, drone: int, relative_time: float) -> int:
        if not len(points):
            return 0
        inside = np.all((points >= MAP_MIN) & (points < MAP_MAX), axis=1)
        points = points[inside]
        if not len(points):
            return 0
        voxels = np.floor(points / ARGS.voxel_size).astype(np.int32)
        voxels = np.unique(voxels, axis=0)
        added = 0
        bit = 1 << drone
        for x, y, z in voxels:
            key = (int(x), int(y), int(z))
            existing = self.lookup.get(key)
            if existing is None:
                self.lookup[key] = len(self.indices)
                self.indices.append(key)
                self.first_seen.append(float(relative_time))
                self.first_agent.append(int(drone))
                self.agent_masks.append(bit)
                added += 1
            else:
                self.agent_masks[existing] |= bit
        return added

    def _map_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if not self.indices:
            return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.float32)
        indices = np.asarray(self.indices, dtype=np.int32)
        points = (indices.astype(np.float32) + 0.5) * float(ARGS.voxel_size)
        intensity = np.asarray(self.first_agent, dtype=np.float32)
        return points, intensity

    def publish_paths(
        self,
        relative_time: float,
        positions: np.ndarray,
        orientations: np.ndarray,
    ) -> None:
        stride = max(
            1,
            int(round(float(self.trajectory.metadata.get("sample_rate_hz", 60.0)) / 5.0)),
        )
        source_time = self.trajectory.start + relative_time
        finish = max(
            0,
            int(np.searchsorted(self.trajectory.times, source_time, side="right") - 1),
        )
        sample_indices = list(range(0, finish + 1, stride))
        stamp = self.stamp(relative_time).clock
        for drone, publisher in enumerate(self.path_publishers):
            message = RosPath()
            message.header.frame_id = "world"
            message.header.stamp = stamp
            for sample in sample_indices:
                pose = PoseStamped()
                pose.header = message.header
                position = self.trajectory.positions[sample, drone]
                orientation = self.trajectory.orientations[sample, drone]
                pose.pose.position.x = float(position[0])
                pose.pose.position.y = float(position[1])
                pose.pose.position.z = float(position[2])
                pose.pose.orientation.w = float(orientation[0])
                pose.pose.orientation.x = float(orientation[1])
                pose.pose.orientation.y = float(orientation[2])
                pose.pose.orientation.z = float(orientation[3])
                message.poses.append(pose)
            final_pose = PoseStamped()
            final_pose.header = message.header
            final_pose.pose.position.x = float(positions[drone, 0])
            final_pose.pose.position.y = float(positions[drone, 1])
            final_pose.pose.position.z = float(positions[drone, 2])
            final_pose.pose.orientation.w = float(orientations[drone, 0])
            final_pose.pose.orientation.x = float(orientations[drone, 1])
            final_pose.pose.orientation.y = float(orientations[drone, 2])
            final_pose.pose.orientation.z = float(orientations[drone, 3])
            message.poses.append(final_pose)
            publisher.publish(message)

    def publish(
        self,
        relative_time: float,
        current_points: np.ndarray,
        current_agents: np.ndarray,
        positions: np.ndarray,
        orientations: np.ndarray,
        force: bool = False,
    ) -> None:
        clock = self.stamp(relative_time)
        self.clock_publisher.publish(clock)
        if len(current_points):
            self.current_publisher.publish(
                create_xyzi_cloud(clock.clock, current_points, current_agents)
            )
        if not force and relative_time - self.last_publish < 1.0 / ARGS.publish_rate_hz:
            return
        self.last_publish = relative_time
        points, intensity = self._map_arrays()
        self.map_publisher.publish(create_xyzi_cloud(clock.clock, points, intensity))
        self.publish_paths(relative_time, positions, orientations)

        status = Marker()
        status.header.frame_id = "world"
        status.header.stamp = clock.clock
        status.ns = "pointcloud_reconstruction"
        status.id = 0
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position = Point(x=-0.5, y=-12.5, z=9.2)
        status.pose.orientation.w = 1.0
        status.scale.z = 0.55
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = (
            "Warehouse Simple 3-D point-cloud reconstruction\n"
            f"Recorded flight t={relative_time:6.1f}/{self.trajectory.duration:.1f} s  "
            f"occupied surface voxels={len(self.indices):,}\n"
            "Isaac depth regenerated from recorded poses; 0.1 m endpoint voxels"
        )
        self.status_publisher.publish(MarkerArray(markers=[status]))

    def save(self, relative_time: float, complete: bool) -> None:
        ARGS.output.parent.mkdir(parents=True, exist_ok=True)
        metadata = {
            "format": "racer_pointcloud_first_seen_v1",
            "source_trajectory": str(ARGS.trajectory),
            "scene_usd": str(ARGS.scene_usd),
            "vehicle_usd": str(ARGS.vehicle_usd),
            "duration_s": float(relative_time),
            "source_duration_s": float(self.trajectory.duration),
            "complete": bool(complete),
            "sensor_rate_hz": float(ARGS.sensor_rate_hz),
            "depth_resolution": [int(ARGS.depth_width), int(ARGS.depth_height)],
            "ray_budget": int(ARGS.ray_budget),
            "voxel_size_m": float(ARGS.voxel_size),
            "map_min": MAP_MIN.tolist(),
            "map_max": MAP_MAX.tolist(),
            "semantics": (
                "regenerated depth-camera surface endpoints; not the unsaved original SDF buffer"
            ),
        }
        np.savez_compressed(
            ARGS.output,
            voxel_indices=np.asarray(self.indices, dtype=np.int32).reshape((-1, 3)),
            first_seen_times=np.asarray(self.first_seen, dtype=np.float32),
            first_agent=np.asarray(self.first_agent, dtype=np.uint8),
            agent_masks=np.asarray(self.agent_masks, dtype=np.uint8),
            metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
        )
        print(
            "RACER_POINTCLOUD_SAVED "
            + json.dumps(
                {
                    "output": str(ARGS.output),
                    "voxel_count": len(self.indices),
                    "duration_s": relative_time,
                    "complete": complete,
                },
                sort_keys=True,
            ),
            flush=True,
        )


def project_depth(
    depth_image: np.ndarray,
    position: np.ndarray,
    orientation: np.ndarray,
) -> np.ndarray:
    depth = np.asarray(depth_image, dtype=np.float32).squeeze()
    expected = (ARGS.depth_height, ARGS.depth_width)
    if depth.shape != expected:
        raise RuntimeError(f"depth shape {depth.shape}, expected {expected}")
    scale_x = ARGS.depth_width / 640.0
    scale_y = ARGS.depth_height / 480.0
    fx = 385.69793701171875 * scale_x
    fy = 385.69793701171875 * scale_y
    cx = 324.0879821777344 * scale_x
    cy = 239.10362243652344 * scale_y
    margin = max(1, round(2 * min(scale_x, scale_y)))
    skip = max(1, round(2 * min(scale_x, scale_y)))
    vv, uu = np.mgrid[
        margin:ARGS.depth_height - margin:skip,
        margin:ARGS.depth_width - margin:skip,
    ]
    measured = depth[vv, uu].reshape(-1)
    u = uu.reshape(-1).astype(np.float32)
    v = vv.reshape(-1).astype(np.float32)
    if len(measured) > ARGS.ray_budget:
        selected = np.linspace(0, len(measured) - 1, ARGS.ray_budget, dtype=np.int64)
        measured, u, v = measured[selected], u[selected], v[selected]
    hit = (
        np.isfinite(measured)
        & (measured >= DEPTH_MIN_RANGE)
        & (measured <= DEPTH_MAP_RANGE)
    )
    measured, u, v = measured[hit], u[hit], v[hit]
    optical_x = (u - cx) * measured / fx
    optical_y = (v - cy) * measured / fy
    body = np.column_stack((measured, -optical_x, -optical_y))
    body = body[np.linalg.norm(body, axis=1) > SELF_FILTER_RADIUS]
    rotation = quaternion_matrix(orientation)
    return (body @ rotation.T + position).astype(np.float32)


def set_pose(translate_op, orient_op, position: np.ndarray, orientation: np.ndarray) -> None:
    quaternion = normalized_quaternion(orientation)
    translate_op.Set(Gf.Vec3d(*(float(value) for value in position)))
    orient_op.Set(
        Gf.Quatf(
            float(quaternion[0]),
            Gf.Vec3f(*(float(value) for value in quaternion[1:])),
        )
    )


def build_stage(trajectory: RecordedTrajectory):
    context = omni.usd.get_context()
    context.new_stage()
    simulation_world = World(
        physics_dt=1.0 / ARGS.sensor_rate_hz,
        rendering_dt=1.0 / ARGS.sensor_rate_hz,
        stage_units_in_meters=1.0,
    )
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    environment = UsdGeom.Xform.Define(stage, "/World/Environment")
    environment.GetPrim().GetReferences().AddReference(str(ARGS.scene_usd))
    light = UsdLux.DistantLight.Define(stage, "/World/ReconstructionSun")
    light.CreateIntensityAttr(2600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))
    UsdGeom.Xform.Define(stage, "/World/ReconstructionDrones")

    translate_ops, orient_ops, cameras = [], [], []
    scale_x = ARGS.depth_width / 640.0
    scale_y = ARGS.depth_height / 480.0
    for drone in range(trajectory.drone_count):
        root_path = f"/World/ReconstructionDrones/drone_{drone}"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.GetPrim().GetReferences().AddReference(str(ARGS.vehicle_usd))
        translate_ops.append(root.AddTranslateOp())
        orient_ops.append(root.AddOrientOp())
        set_pose(
            translate_ops[-1],
            orient_ops[-1],
            trajectory.positions[0, drone],
            trajectory.orientations[0, drone],
        )
        visuals = stage.GetPrimAtPath(root_path + "/base_link/visuals")
        if visuals.IsValid():
            UsdGeom.Imageable(visuals).MakeInvisible()
        body_prim = stage.GetPrimAtPath(root_path + "/base_link")
        if not body_prim.IsValid() or not body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError(f"vehicle has no rigid body at {body_prim.GetPath()}")
        rigid_body = UsdPhysics.RigidBodyAPI(body_prim)
        kinematic = rigid_body.GetKinematicEnabledAttr()
        if not kinematic.IsValid():
            kinematic = rigid_body.CreateKinematicEnabledAttr()
        kinematic.Set(True)
        camera = simulation_world.scene.add(
            Camera(
                prim_path=root_path + "/base_link/reconstruction_depth_camera",
                name=f"reconstruction_depth_camera_{drone}",
                frequency=ARGS.sensor_rate_hz,
                resolution=(ARGS.depth_width, ARGS.depth_height),
                translation=np.zeros(3, dtype=np.float64),
                orientation=np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64),
            )
        )
        camera.set_opencv_pinhole_properties(
            cx=321.046 * scale_x,
            cy=243.449 * scale_y,
            fx=387.229 * scale_x,
            fy=387.229 * scale_y,
            pinhole=[0.0] * 12,
        )
        camera.set_clipping_range(near_distance=DEPTH_MIN_RANGE, far_distance=5.0)
        cameras.append(camera)
    simulation_world.reset()
    for camera in cameras:
        camera.add_distance_to_image_plane_to_frame()
    return simulation_world, translate_ops, orient_ops, cameras


def main() -> None:
    trajectory = RecordedTrajectory(ARGS.trajectory)
    if trajectory.drone_count != 5:
        raise RuntimeError(f"expected five recorded drones, got {trajectory.drone_count}")
    end_time = trajectory.duration
    if ARGS.end_time is not None:
        end_time = min(end_time, max(0.0, float(ARGS.end_time)))
    simulation_world, translate_ops, orient_ops, cameras = build_stage(trajectory)
    for _ in range(12):
        simulation_world.step(render=True)

    async def wait_for_depth() -> None:
        await asyncio.gather(
            *(
                syntheticdata.sensors.next_render_simulation_async(
                    camera.get_render_product_path(), 10
                )
                for camera in cameras
            )
        )

    simulation_app.run_coroutine(wait_for_depth())
    for drone, camera in enumerate(cameras):
        depth = camera.get_depth()
        if depth is None or np.asarray(depth).squeeze().shape != (
            ARGS.depth_height,
            ARGS.depth_width,
        ):
            raise RuntimeError(f"depth camera D{drone} did not initialize")

    rclpy.init()
    node = PointCloudMapPublisher(trajectory)
    print(
        "RACER_POINTCLOUD_RECONSTRUCTION_READY "
        + json.dumps(
            {
                "trajectory": str(ARGS.trajectory),
                "scene_usd": str(ARGS.scene_usd),
                "vehicle_usd": str(ARGS.vehicle_usd),
                "duration_s": end_time,
                "speed": ARGS.speed,
                "sensor_rate_hz": ARGS.sensor_rate_hz,
                "depth_resolution": [ARGS.depth_width, ARGS.depth_height],
                "ray_budget": ARGS.ray_budget,
                "voxel_size_m": ARGS.voxel_size,
                "semantics": "regenerated depth surface endpoints",
            },
            sort_keys=True,
        ),
        flush=True,
    )
    started = time.monotonic()
    sample_period = 1.0 / ARGS.sensor_rate_hz
    sample_time = 0.0
    last_report = -math.inf
    complete = False
    try:
        while simulation_app.is_running() and sample_time <= end_time + 1.0e-9:
            target_wall = started + sample_time / ARGS.speed
            remaining = target_wall - time.monotonic()
            if remaining > 0.0:
                time.sleep(min(remaining, 0.02))
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            positions, orientations = trajectory.interpolate(sample_time)
            for drone in range(trajectory.drone_count):
                set_pose(
                    translate_ops[drone],
                    orient_ops[drone],
                    positions[drone],
                    orientations[drone],
                )
            simulation_world.step(render=True)
            current_points = []
            current_agents = []
            new_voxels = 0
            for drone, camera in enumerate(cameras):
                depth = camera.get_depth()
                if depth is None:
                    continue
                points = project_depth(
                    np.asarray(depth), positions[drone], orientations[drone]
                )
                new_voxels += node.add_hits(points, drone, sample_time)
                if len(points):
                    current_points.append(points)
                    current_agents.append(
                        np.full(len(points), drone, dtype=np.float32)
                    )
            combined_points = (
                np.concatenate(current_points, axis=0)
                if current_points
                else np.empty((0, 3), dtype=np.float32)
            )
            combined_agents = (
                np.concatenate(current_agents, axis=0)
                if current_agents
                else np.empty(0, dtype=np.float32)
            )
            node.publish(
                sample_time,
                combined_points,
                combined_agents,
                positions,
                orientations,
            )
            rclpy.spin_once(node, timeout_sec=0.0)
            if sample_time - last_report >= 5.0 - 1.0e-9:
                last_report = sample_time
                print(
                    "RACER_POINTCLOUD_PROGRESS "
                    + json.dumps(
                        {
                            "time_s": sample_time,
                            "duration_s": end_time,
                            "voxel_count": len(node.indices),
                            "new_voxels": new_voxels,
                            "lag_s": max(
                                0.0,
                                time.monotonic()
                                - (started + sample_time / ARGS.speed),
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            sample_time += sample_period
        complete = sample_time >= end_time
    finally:
        final_time = min(end_time, max(0.0, sample_time - sample_period))
        positions, orientations = trajectory.interpolate(final_time)
        node.publish(
            final_time,
            np.empty((0, 3), dtype=np.float32),
            np.empty(0, dtype=np.float32),
            positions,
            orientations,
            force=True,
        )
        node.save(final_time, complete)
        # Keep the final transient-local point cloud alive while RViz remains open.
        while (
            complete
            and ARGS.hold_final
            and simulation_app.is_running()
            and rclpy.ok()
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
