#!/usr/bin/env python3
"""Render RACER ROS state in a UI-only Isaac Sim process.

The authoritative PhysX process runs headlessly.  This process owns no
physics scene and no camera sensors: it only consumes ROS odometry, map, and
path messages and updates display-only USD transforms.  A short wall-clock
buffer interpolates sparse odometry arrivals so the viewport remains smooth
even when the source simulation runs slower than real time.
"""

import argparse
from collections import deque
import math
from pathlib import Path
import time


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene-usd", type=Path, required=True)
    parser.add_argument("--vehicle-usd", type=Path, required=True)
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--interpolation-delay", type=float, default=0.25)
    parser.add_argument("--max-map-points", type=int, default=8000)
    return parser.parse_args()


ARGS = parse_arguments()
ARGS.scene_usd = ARGS.scene_usd.expanduser().resolve()
ARGS.vehicle_usd = ARGS.vehicle_usd.expanduser().resolve()
if not ARGS.scene_usd.is_file():
    raise SystemExit(f"scene USD does not exist: {ARGS.scene_usd}")
if not ARGS.vehicle_usd.is_file():
    raise SystemExit(f"vehicle USD does not exist: {ARGS.vehicle_usd}")
if ARGS.drone_count <= 0 or ARGS.drone_count > 5:
    raise SystemExit("--drone-count must be between 1 and 5")
if ARGS.fps <= 0.0:
    raise SystemExit("--fps must be positive")
if ARGS.interpolation_delay < 0.0:
    raise SystemExit("--interpolation-delay must be non-negative")
if ARGS.max_map_points <= 0:
    raise SystemExit("--max-map-points must be positive")

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": False,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.util.debug_draw")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.util.debug_draw import _debug_draw  # noqa: E402
from nav_msgs.msg import Odometry, Path as RosPath  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2  # noqa: E402

from pointcloud_cpp_bridge import read_xyzi_cloud  # noqa: E402


STARTS = (
    (-6.0, -10.0, 0.6),
    (0.0, -10.0, 1.5),
    (6.0, -10.0, 2.4),
    (-3.0, -10.0, 1.05),
    (3.0, -10.0, 1.95),
)
COLORS = (
    (1.0, 0.16, 0.10, 1.0),
    (0.15, 1.0, 0.25, 1.0),
    (1.0, 0.82, 0.08, 1.0),
    (0.75, 0.25, 1.0, 1.0),
    (1.0, 0.48, 0.05, 1.0),
)
MAP_COLOR = (0.0, 0.72, 1.0, 0.62)


def normalized_quaternion(values) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    norm = float(np.linalg.norm(result))
    if norm < 1.0e-9:
        return np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float)
    return result / norm


def quaternion_slerp(first, second, fraction: float) -> np.ndarray:
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


def line_segments(points):
    if len(points) < 2:
        return [], []
    values = np.asarray(points, dtype=float).reshape((-1, 3))
    return values[:-1].tolist(), values[1:].tolist()


class RacerViewer(Node):
    def __init__(self) -> None:
        super().__init__("racer_isaac_decoupled_viewer")
        self.samples = [deque(maxlen=64) for _ in range(ARGS.drone_count)]
        self.display_positions = [
            np.asarray(STARTS[index], dtype=float)
            for index in range(ARGS.drone_count)
        ]
        self.display_orientations = [
            np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float)
            for _ in range(ARGS.drone_count)
        ]
        self.paths = [
            np.empty((0, 3), dtype=np.float32)
            for _ in range(ARGS.drone_count)
        ]
        self.trails = [
            [STARTS[index]] for index in range(ARGS.drone_count)
        ]
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.last_trail_wall = -math.inf
        self.last_draw_wall = -math.inf
        self.draw_dirty = True
        self.debug_draw = _debug_draw.acquire_debug_draw_interface()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription_handles = []
        for drone_id in range(ARGS.drone_count):
            self.subscription_handles.append(
                self.create_subscription(
                    Odometry,
                    f"/drone_{drone_id}/odom",
                    lambda message, index=drone_id: self._odometry(
                        index, message
                    ),
                    qos,
                )
            )
            self.subscription_handles.append(
                self.create_subscription(
                    RosPath,
                    f"/drone_{drone_id}/planned_path_3d",
                    lambda message, index=drone_id: self._path(
                        index, message
                    ),
                    qos,
                )
            )
        self.subscription_handles.append(
            self.create_subscription(
                PointCloud2,
                "/drone_0/occupied_voxels",
                self._map,
                qos,
            )
        )

    def _odometry(self, drone_id: int, message: Odometry) -> None:
        pose = message.pose.pose
        position = np.asarray(
            (pose.position.x, pose.position.y, pose.position.z), dtype=float
        )
        orientation = normalized_quaternion(
            (
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            )
        )
        self.samples[drone_id].append(
            (time.monotonic(), position, orientation)
        )

    def _path(self, drone_id: int, message: RosPath) -> None:
        self.paths[drone_id] = np.asarray(
            [
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                )
                for pose in message.poses
            ],
            dtype=np.float32,
        ).reshape((-1, 3))
        self.draw_dirty = True

    def _map(self, message: PointCloud2) -> None:
        points, _ = read_xyzi_cloud(message)
        if len(points) > ARGS.max_map_points:
            indices = np.linspace(
                0, len(points) - 1, ARGS.max_map_points, dtype=np.int64
            )
            points = points[indices]
        self.map_points = np.asarray(points, dtype=np.float32)
        self.draw_dirty = True

    def interpolate(self, drone_id: int, now: float):
        samples = self.samples[drone_id]
        if not samples:
            return (
                self.display_positions[drone_id],
                self.display_orientations[drone_id],
            )
        target = now - ARGS.interpolation_delay
        if target <= samples[0][0]:
            return samples[0][1], samples[0][2]
        if target >= samples[-1][0]:
            return samples[-1][1], samples[-1][2]
        for index in range(1, len(samples)):
            after = samples[index]
            if after[0] < target:
                continue
            before = samples[index - 1]
            span = after[0] - before[0]
            fraction = 1.0 if span <= 1.0e-9 else (target - before[0]) / span
            position = before[1] + fraction * (after[1] - before[1])
            orientation = quaternion_slerp(before[2], after[2], fraction)
            return position, orientation
        return samples[-1][1], samples[-1][2]

    def update_trails(self, now: float) -> None:
        if now - self.last_trail_wall < 0.10:
            return
        for drone_id, position in enumerate(self.display_positions):
            point = tuple(float(value) for value in position)
            if np.linalg.norm(
                np.asarray(point) - np.asarray(self.trails[drone_id][-1])
            ) >= 0.01:
                self.trails[drone_id].append(point)
                if len(self.trails[drone_id]) > 1500:
                    self.trails[drone_id] = self.trails[drone_id][-1500:]
                self.draw_dirty = True
        self.last_trail_wall = now

    def update_debug_draw(self, now: float, force: bool = False) -> None:
        if not self.draw_dirty and not force:
            return
        if not force and now - self.last_draw_wall < 0.20:
            return
        self.debug_draw.clear_points()
        self.debug_draw.clear_lines()
        if len(self.map_points):
            points = self.map_points.tolist()
            self.debug_draw.draw_points(
                points,
                [MAP_COLOR] * len(points),
                [3.0] * len(points),
            )
        starts, ends, colors, widths = [], [], [], []
        for drone_id in range(ARGS.drone_count):
            color = COLORS[drone_id]
            trail_starts, trail_ends = line_segments(self.trails[drone_id])
            starts.extend(trail_starts)
            ends.extend(trail_ends)
            colors.extend(
                [(color[0], color[1], color[2], 0.55)]
                * len(trail_starts)
            )
            widths.extend([2.0] * len(trail_starts))
            path_starts, path_ends = line_segments(self.paths[drone_id])
            starts.extend(path_starts)
            ends.extend(path_ends)
            colors.extend([color] * len(path_starts))
            widths.extend([4.0] * len(path_starts))
        if starts:
            self.debug_draw.draw_lines(starts, ends, colors, widths)
        self.last_draw_wall = now
        self.draw_dirty = False


def build_stage():
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    environment = UsdGeom.Xform.Define(stage, "/World/Environment")
    environment.GetPrim().GetReferences().AddReference(str(ARGS.scene_usd))

    light = UsdLux.DistantLight.Define(stage, "/World/ViewerSun")
    light.CreateIntensityAttr(2600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))

    translate_ops, orient_ops = [], []
    UsdGeom.Xform.Define(stage, "/World/ViewerDrones")
    for drone_id in range(ARGS.drone_count):
        root_path = f"/World/ViewerDrones/drone_{drone_id}"
        root = UsdGeom.Xform.Define(stage, root_path)
        translate_ops.append(root.AddTranslateOp())
        orient_ops.append(root.AddOrientOp())
        translate_ops[-1].Set(Gf.Vec3d(*STARTS[drone_id]))
        orient_ops[-1].Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        model = UsdGeom.Xform.Define(stage, root_path + "/Model")
        model.GetPrim().GetReferences().AddReference(str(ARGS.vehicle_usd))
    return translate_ops, orient_ops


def main() -> None:
    translate_ops, orient_ops = build_stage()
    for _ in range(4):
        simulation_app.update()
    set_camera_view(
        eye=np.asarray((0.0, -32.0, 20.0), dtype=float),
        target=np.asarray((0.0, 3.0, 3.0), dtype=float),
    )
    simulation_app.update()

    rclpy.init()
    viewer = RacerViewer()
    print(
        "RACER_DECOUPLED_VIEWER_READY "
        f"fps={ARGS.fps:.1f} drones={ARGS.drone_count} "
        f"vehicle={ARGS.vehicle_usd} scene={ARGS.scene_usd}",
        flush=True,
    )
    frame_period = 1.0 / ARGS.fps
    report_start = time.monotonic()
    report_frames = 0
    try:
        while simulation_app.is_running():
            frame_start = time.monotonic()
            rclpy.spin_once(viewer, timeout_sec=0.0)
            now = time.monotonic()
            for drone_id in range(ARGS.drone_count):
                position, orientation = viewer.interpolate(drone_id, now)
                viewer.display_positions[drone_id] = position
                viewer.display_orientations[drone_id] = orientation
                translate_ops[drone_id].Set(
                    Gf.Vec3d(*(float(value) for value in position))
                )
                orient_ops[drone_id].Set(
                    Gf.Quatf(
                        float(orientation[0]),
                        Gf.Vec3f(
                            float(orientation[1]),
                            float(orientation[2]),
                            float(orientation[3]),
                        ),
                    )
                )
            viewer.update_trails(now)
            viewer.update_debug_draw(now)
            simulation_app.update()
            report_frames += 1
            report_now = time.monotonic()
            report_span = report_now - report_start
            if report_span >= 5.0:
                print(
                    "RACER_DECOUPLED_VIEWER_FPS "
                    f"measured={report_frames / report_span:.2f} "
                    f"target={ARGS.fps:.2f}",
                    flush=True,
                )
                report_start = report_now
                report_frames = 0
            remaining = frame_period - (time.monotonic() - frame_start)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        viewer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
