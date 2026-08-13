#!/usr/bin/env python3
"""Replay an authoritative headless RACER/PhysX trajectory at wall-clock 1x."""

import argparse
import json
import math
from pathlib import Path
import time


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--scene-usd", type=Path, required=True)
    parser.add_argument("--vehicle-usd", type=Path, required=True)
    parser.add_argument(
        "--base-station-usd",
        type=Path,
        help="optional ceiling-mounted industrial Wi-Fi AP asset",
    )
    parser.add_argument(
        "--pointcloud-map",
        type=Path,
        help=(
            "first-seen point-cloud NPZ produced by "
            "original_racer_pointcloud_reconstruction.py"
        ),
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=3.0,
        help="Isaac debug-draw point size in pixels",
    )
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument(
        "--loop",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="restart the 1x replay after its final recorded sample",
    )
    parser.add_argument(
        "--follow-drone",
        type=int,
        default=-1,
        help="chase-camera drone id; -1 leaves the overview camera user-controlled",
    )
    parser.add_argument(
        "--camera-mode",
        choices=("chase", "onboard"),
        default="chase",
        help="initial view used when a drone is selected",
    )
    parser.add_argument(
        "--initial-view",
        choices=("overview", "ap"),
        default="overview",
        help="initial viewport camera",
    )
    parser.add_argument("--propeller-visual-hz", type=float, default=60.0)
    return parser.parse_args()


ARGS = parse_arguments()
for attribute in (
    "trajectory",
    "scene_usd",
    "vehicle_usd",
    "pointcloud_map",
    "base_station_usd",
):
    if getattr(ARGS, attribute) is None:
        continue
    path = getattr(ARGS, attribute).expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"{attribute.replace('_', ' ')} does not exist: {path}")
    setattr(ARGS, attribute, path)
if ARGS.fps <= 0.0:
    raise SystemExit("--fps must be positive")
if ARGS.speed <= 0.0:
    raise SystemExit("--speed must be positive")
if ARGS.propeller_visual_hz <= 0.0:
    raise SystemExit("--propeller-visual-hz must be positive")
if ARGS.point_size <= 0.0:
    raise SystemExit("--point-size must be positive")
if ARGS.follow_drone < -1:
    raise SystemExit("--follow-drone must be -1 or a non-negative drone id")

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


enable_extension("isaacsim.util.debug_draw")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.ui as ui  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.util.debug_draw import _debug_draw  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdLux  # noqa: E402


COLORS = (
    (1.0, 0.16, 0.10, 0.72),
    (0.15, 1.0, 0.25, 0.72),
    (1.0, 0.82, 0.08, 0.72),
    (0.75, 0.25, 1.0, 0.72),
    (1.0, 0.48, 0.05, 0.72),
)
MAP_COLORS = (
    (0.15, 0.55, 1.00, 0.92),
    (0.15, 1.00, 0.40, 0.92),
    (1.00, 0.25, 0.18, 0.92),
    (1.00, 0.88, 0.12, 0.92),
    (0.92, 0.25, 1.00, 0.92),
)
BASE_STATION_POSITION = np.asarray((-0.5, 2.85, 8.55), dtype=np.float64)


def normalized_quaternion(values) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if norm < 1.0e-12:
        return np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64)
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


class RecordedTrajectory:
    REQUIRED_FIELDS = (
        "times",
        "positions",
        "orientations_wxyz",
        "linear_velocities",
        "angular_velocities",
        "motor_rpm",
        "metadata_json",
    )

    def __init__(self, path: Path) -> None:
        with np.load(path, allow_pickle=False) as archive:
            missing = [name for name in self.REQUIRED_FIELDS if name not in archive]
            if missing:
                raise RuntimeError(f"trajectory is missing fields: {missing}")
            self.times = np.asarray(archive["times"], dtype=np.float64)
            self.positions = np.asarray(archive["positions"], dtype=np.float64)
            self.orientations = np.asarray(
                archive["orientations_wxyz"], dtype=np.float64
            )
            self.linear_velocities = np.asarray(
                archive["linear_velocities"], dtype=np.float64
            )
            self.angular_velocities = np.asarray(
                archive["angular_velocities"], dtype=np.float64
            )
            self.motor_rpm = np.asarray(archive["motor_rpm"], dtype=np.float64)
            self.metadata = json.loads(str(archive["metadata_json"].item()))
        if len(self.times) < 2 or np.any(np.diff(self.times) <= 0.0):
            raise RuntimeError("trajectory needs at least two increasing timestamps")
        drone_count = self.positions.shape[1]
        expected = {
            "positions": (len(self.times), drone_count, 3),
            "orientations": (len(self.times), drone_count, 4),
            "linear_velocities": (len(self.times), drone_count, 3),
            "angular_velocities": (len(self.times), drone_count, 3),
            "motor_rpm": (len(self.times), drone_count, 4),
        }
        for name, shape in expected.items():
            if getattr(self, name).shape != shape:
                raise RuntimeError(
                    f"trajectory {name} shape {getattr(self, name).shape}, "
                    f"expected {shape}"
                )
        self.drone_count = drone_count
        self.start_time = float(self.times[0])
        self.end_time = float(self.times[-1])
        self.duration = self.end_time - self.start_time

    def interpolate(self, playback_time: float):
        if playback_time <= self.start_time:
            return (
                self.positions[0],
                self.orientations[0],
                self.linear_velocities[0],
                self.motor_rpm[0],
            )
        if playback_time >= self.end_time:
            return (
                self.positions[-1],
                self.orientations[-1],
                self.linear_velocities[-1],
                self.motor_rpm[-1],
            )
        after = int(np.searchsorted(self.times, playback_time, side="right"))
        before = after - 1
        span = self.times[after] - self.times[before]
        fraction = float((playback_time - self.times[before]) / span)
        positions = self.positions[before] + fraction * (
            self.positions[after] - self.positions[before]
        )
        velocities = self.linear_velocities[before] + fraction * (
            self.linear_velocities[after] - self.linear_velocities[before]
        )
        rpms = self.motor_rpm[before] + fraction * (
            self.motor_rpm[after] - self.motor_rpm[before]
        )
        orientations = np.asarray(
            [
                quaternion_slerp(
                    self.orientations[before, drone_id],
                    self.orientations[after, drone_id],
                    fraction,
                )
                for drone_id in range(self.drone_count)
            ],
            dtype=np.float64,
        )
        return positions, orientations, velocities, rpms


class RecordedPointCloudMap:
    REQUIRED_FIELDS = (
        "voxel_indices",
        "first_seen_times",
        "first_agent",
        "metadata_json",
    )

    def __init__(self, path: Path) -> None:
        with np.load(path, allow_pickle=False) as archive:
            missing = [name for name in self.REQUIRED_FIELDS if name not in archive]
            if missing:
                raise RuntimeError(f"point-cloud map is missing fields: {missing}")
            voxel_indices = np.asarray(archive["voxel_indices"], dtype=np.int32)
            first_seen = np.asarray(archive["first_seen_times"], dtype=np.float64)
            first_agent = np.asarray(archive["first_agent"], dtype=np.int32)
            self.metadata = json.loads(str(archive["metadata_json"].item()))
        if voxel_indices.ndim != 2 or voxel_indices.shape[1] != 3:
            raise RuntimeError(
                f"point-cloud voxel index shape {voxel_indices.shape}, expected (N, 3)"
            )
        if first_seen.shape != (len(voxel_indices),):
            raise RuntimeError("point-cloud first-seen timestamps have the wrong shape")
        if first_agent.shape != (len(voxel_indices),):
            raise RuntimeError("point-cloud first-agent values have the wrong shape")
        if np.any(~np.isfinite(first_seen)) or np.any(first_seen < 0.0):
            raise RuntimeError("point-cloud first-seen timestamps are invalid")
        voxel_size = float(self.metadata.get("voxel_size_m", 0.1))
        if voxel_size <= 0.0:
            raise RuntimeError("point-cloud voxel size must be positive")
        order = np.argsort(first_seen, kind="stable")
        self.points = (
            (voxel_indices[order].astype(np.float32) + 0.5) * voxel_size
        )
        self.first_seen = first_seen[order]
        self.first_agent = first_agent[order]
        self.voxel_size = voxel_size
        self.cursor = 0
        self.debug_draw = _debug_draw.acquire_debug_draw_interface()
        self.debug_draw.clear_points()

    @property
    def count(self) -> int:
        return int(len(self.points))

    def reset(self) -> None:
        self.debug_draw.clear_points()
        self.cursor = 0

    def update(self, relative_time: float) -> int:
        finish = int(np.searchsorted(self.first_seen, relative_time, side="right"))
        if finish <= self.cursor:
            return self.cursor
        # Draw only newly discovered voxels. Debug-draw retains previous calls,
        # so viewport work scales with the new observations instead of the full
        # accumulated map on every frame.
        chunk_size = 20000
        while self.cursor < finish:
            chunk_finish = min(finish, self.cursor + chunk_size)
            points = self.points[self.cursor:chunk_finish].tolist()
            colors = [
                MAP_COLORS[int(agent) % len(MAP_COLORS)]
                for agent in self.first_agent[self.cursor:chunk_finish]
            ]
            self.debug_draw.draw_points(
                points,
                colors,
                [float(ARGS.point_size)] * len(points),
            )
            self.cursor = chunk_finish
        return self.cursor


class PropellerAnimator:
    DEGREES_PER_RPM_SECOND = 6.0
    AXIS_INDICES = {"X": 0, "Y": 1, "Z": 2}

    def __init__(self, stage, drone_count: int, update_hz: float) -> None:
        self.update_period = 1.0 / float(update_hz)
        self.accumulated_time = 0.0
        self.angles = np.zeros((drone_count, 4), dtype=np.float64)
        self.spin_signs = np.zeros((drone_count, 4), dtype=np.float64)
        self.entries = []
        missing = []
        for drone_id in range(drone_count):
            visuals_path = f"/World/ReplayDrones/drone_{drone_id}/base_link/visuals"
            visuals = stage.GetPrimAtPath(visuals_path)
            if not visuals.IsValid():
                missing.append(visuals_path)
                continue
            candidates = []
            for prim in Usd.PrimRange(visuals):
                rotor_attr = prim.GetAttribute("racer:rotorId")
                if rotor_attr.IsValid() and rotor_attr.HasAuthoredValueOpinion():
                    candidates.append((int(rotor_attr.Get()), prim))
            candidates.sort(key=lambda value: value[0])
            if [value[0] for value in candidates] != [0, 1, 2, 3]:
                missing.append(
                    f"{visuals_path} rotor_ids={[value[0] for value in candidates]}"
                )
                continue
            for rotor_id, prim in candidates:
                sign = int(prim.GetAttribute("racer:spinDirectionSign").Get())
                axis = str(
                    prim.GetAttribute("racer:visualRotationAxis").Get()
                ).upper()
                operation_name = str(
                    prim.GetAttribute("racer:visualRotationOp").Get()
                )
                rotation_attribute = prim.GetAttribute(operation_name)
                base_rotation = rotation_attribute.Get()
                if (
                    sign not in (-1, 1)
                    or axis not in self.AXIS_INDICES
                    or base_rotation is None
                ):
                    raise RuntimeError(
                        f"invalid propeller metadata at {prim.GetPath()}"
                    )
                self.spin_signs[drone_id, rotor_id] = sign
                self.entries.append(
                    {
                        "drone_id": drone_id,
                        "rotor_id": rotor_id,
                        "attribute": rotation_attribute,
                        "axis_index": self.AXIS_INDICES[axis],
                        "base": tuple(float(value) for value in base_rotation),
                        "vector_type": type(base_rotation),
                    }
                )
        if missing:
            raise RuntimeError(
                "propeller animation metadata unavailable: " + "; ".join(missing)
            )
        self.write_angles()

    def write_angles(self) -> None:
        for entry in self.entries:
            values = list(entry["base"])
            values[entry["axis_index"]] += self.angles[
                entry["drone_id"], entry["rotor_id"]
            ]
            entry["attribute"].Set(entry["vector_type"](*values))

    def step(self, motor_rpm: np.ndarray, simulation_dt: float) -> None:
        if motor_rpm.shape != self.angles.shape:
            raise RuntimeError(
                f"RPM shape {motor_rpm.shape} does not match {self.angles.shape}"
            )
        self.angles = np.remainder(
            self.angles
            + self.spin_signs
            * motor_rpm
            * self.DEGREES_PER_RPM_SECOND
            * float(simulation_dt),
            360.0,
        )
        self.accumulated_time += float(simulation_dt)
        if self.accumulated_time + 1.0e-12 < self.update_period:
            return
        self.accumulated_time %= self.update_period
        self.write_angles()

    def reset(self) -> None:
        self.angles.fill(0.0)
        self.accumulated_time = 0.0
        self.write_angles()


def build_stage(trajectory: RecordedTrajectory):
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    environment = UsdGeom.Xform.Define(stage, "/World/Environment")
    environment.GetPrim().GetReferences().AddReference(str(ARGS.scene_usd))
    if ARGS.base_station_usd is not None:
        access_point = UsdGeom.Xform.Define(
            stage, "/World/IndustrialWifiAccessPoint"
        )
        access_point.GetPrim().GetReferences().AddReference(
            str(ARGS.base_station_usd)
        )
        access_point.AddTranslateOp().Set(
            Gf.Vec3d(*(float(value) for value in BASE_STATION_POSITION))
        )

    light = UsdLux.DistantLight.Define(stage, "/World/ReplaySun")
    light.CreateIntensityAttr(2600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))

    UsdGeom.Xform.Define(stage, "/World/ReplayDrones")
    translate_ops, orient_ops = [], []
    for drone_id in range(trajectory.drone_count):
        root_path = f"/World/ReplayDrones/drone_{drone_id}"
        root = UsdGeom.Xform.Define(stage, root_path)
        root.GetPrim().GetReferences().AddReference(str(ARGS.vehicle_usd))
        translate_ops.append(root.AddTranslateOp())
        orient_ops.append(root.AddOrientOp())
        translate_ops[-1].Set(
            Gf.Vec3d(*trajectory.positions[0, drone_id].tolist())
        )
        quaternion = normalized_quaternion(trajectory.orientations[0, drone_id])
        orient_ops[-1].Set(
            Gf.Quatf(
                float(quaternion[0]),
                Gf.Vec3f(*quaternion[1:].astype(float).tolist()),
            )
        )
    return stage, translate_ops, orient_ops


class RecordedPathRenderer:
    """Append flown path segments without revealing the future trajectory."""

    def __init__(self, trajectory: RecordedTrajectory) -> None:
        stride = max(1, int(round(len(trajectory.times) / 4000.0)))
        indices = np.arange(0, len(trajectory.times), stride, dtype=np.int64)
        if indices[-1] != len(trajectory.times) - 1:
            indices = np.append(indices, len(trajectory.times) - 1)
        self.times = trajectory.times[indices]
        self.positions = trajectory.positions[indices]
        self.drone_count = trajectory.drone_count
        self.cursor = 0
        self.debug_draw = _debug_draw.acquire_debug_draw_interface()
        self.debug_draw.clear_lines()

    def reset(self) -> None:
        self.debug_draw.clear_lines()
        self.cursor = 0

    def update(self, playback_time: float) -> None:
        visible_samples = int(
            np.searchsorted(self.times, playback_time, side="right")
        )
        finish = max(0, visible_samples - 1)
        if finish <= self.cursor:
            return
        starts, ends, colors, widths = [], [], [], []
        for drone_id in range(self.drone_count):
            starts.extend(
                self.positions[self.cursor:finish, drone_id].tolist()
            )
            ends.extend(
                self.positions[self.cursor + 1:finish + 1, drone_id].tolist()
            )
            count = finish - self.cursor
            colors.extend([COLORS[drone_id % len(COLORS)]] * count)
            widths.extend([2.0] * count)
        if starts:
            self.debug_draw.draw_lines(starts, ends, colors, widths)
        self.cursor = finish


def set_overview_camera() -> None:
    # Keep the camera inside the warehouse shell. The previous exterior eye
    # position was occluded by the rear wall and hid most of the growing map.
    set_camera_view(
        eye=np.asarray((0.0, -9.5, 7.8), dtype=float),
        target=np.asarray((0.0, 4.0, 3.0), dtype=float),
    )


def rotate_vector(quaternion, vector) -> np.ndarray:
    quaternion = normalized_quaternion(quaternion)
    vector = np.asarray(vector, dtype=np.float64)
    imaginary = quaternion[1:]
    return vector + 2.0 * np.cross(
        imaginary,
        np.cross(imaginary, vector) + quaternion[0] * vector,
    )


class ReplayCameraController:
    def __init__(self, drone_count: int, selected: int, mode: str) -> None:
        self.drone_count = int(drone_count)
        self.selected = int(selected)
        self.mode = str(mode)

    def overview(self) -> None:
        self.selected = -1
        set_overview_camera()

    def access_point(self) -> None:
        self.selected = -2
        set_camera_view(
            eye=BASE_STATION_POSITION
            + np.asarray((0.0, -1.55, -0.62), dtype=float),
            target=BASE_STATION_POSITION
            + np.asarray((0.0, 0.0, -0.16), dtype=float),
        )

    def select_drone(self, drone_id: int) -> None:
        self.selected = int(drone_id)

    def set_mode(self, mode: str) -> None:
        self.mode = str(mode)
        if self.selected < 0:
            self.selected = 0

    @property
    def description(self) -> str:
        if self.selected == -2:
            return "Industrial Wi-Fi AP"
        if self.selected < 0:
            return "Overview (mouse-controlled)"
        return f"D{self.selected} {self.mode}"

    def update(
        self,
        positions: np.ndarray,
        orientations: np.ndarray,
        velocities: np.ndarray,
    ) -> None:
        if self.selected < 0:
            return
        position = positions[self.selected]
        orientation = orientations[self.selected]
        if self.mode == "onboard":
            # The original forward depth camera looks along body +X. Place
            # the replay camera just above its optical origin and use body +Z
            # as the camera up vector so this is a genuine vehicle view.
            forward = rotate_vector(orientation, (1.0, 0.0, 0.0))
            up = rotate_vector(orientation, (0.0, 0.0, 1.0))
            eye = position + 0.10 * forward + 0.04 * up
            target = eye + 4.0 * forward
            set_camera_view(eye=eye, target=target)
            return

        horizontal = np.asarray(
            (velocities[self.selected, 0], velocities[self.selected, 1], 0.0),
            dtype=float,
        )
        norm = float(np.linalg.norm(horizontal))
        if norm < 0.15:
            forward = rotate_vector(orientation, (1.0, 0.0, 0.0))
            horizontal = np.asarray((forward[0], forward[1], 0.0), dtype=float)
            norm = float(np.linalg.norm(horizontal))
        if norm < 1.0e-9:
            horizontal = np.asarray((0.0, 1.0, 0.0), dtype=float)
        else:
            horizontal /= norm
        eye = position - 3.5 * horizontal + np.asarray((0.0, 0.0, 1.5))
        target = position + 1.5 * horizontal
        set_camera_view(eye=eye, target=target)


def create_overlay(
    trajectory: RecordedTrajectory,
    camera: ReplayCameraController,
    pointcloud: RecordedPointCloudMap | None,
):
    window = ui.Window(
        "Original RACER - Isaac 3-D Mapping Replay",
        width=560,
        height=(332 if pointcloud is not None else 296),
        position_x=16,
        position_y=42,
    )
    with window.frame:
        with ui.VStack(spacing=5, height=0):
            ui.Label("Original RACER / Warehouse Simple", height=24)
            status = ui.Label("Preparing replay...", height=22)
            clock = ui.Label("Recorded time: 0.000 s", height=22)
            speed = ui.Label("Drone speeds: 0.00 m/s", height=22)
            rpm = ui.Label("Motor RPM: 0", height=22)
            map_status = ui.Label(
                (
                    f"3-D map: 0 / {pointcloud.count:,} surface voxels"
                    if pointcloud is not None
                    else "3-D map: not loaded"
                ),
                height=22,
            )
            camera_status = ui.Label(
                "Camera: " + camera.description,
                height=22,
            )
            with ui.HStack(spacing=4, height=28):
                ui.Button("Overview", clicked_fn=camera.overview, width=78)
                if ARGS.base_station_usd is not None:
                    ui.Button(
                        "AP View",
                        clicked_fn=camera.access_point,
                        width=72,
                    )
                for drone_id in range(trajectory.drone_count):
                    ui.Button(
                        f"D{drone_id}",
                        clicked_fn=(
                            lambda selected=drone_id: camera.select_drone(selected)
                        ),
                        width=48,
                    )
            with ui.HStack(spacing=4, height=28):
                ui.Label("Selected-drone view:", width=140)
                ui.Button(
                    "Chase",
                    clicked_fn=lambda: camera.set_mode("chase"),
                    width=80,
                )
                ui.Button(
                    "Onboard",
                    clicked_fn=lambda: camera.set_mode("onboard"),
                    width=80,
                )
            ui.Label(
                "Recorded: PhysX rotor dynamics | Replay: display-only 1.00x",
                height=22,
            )
            if pointcloud is not None:
                ui.Label(
                    "Map: regenerated depth-hit voxels, revealed at first-observed time",
                    height=22,
                )
            if ARGS.base_station_usd is not None:
                ui.Label(
                    "AP: Wi-Fi 6 Ch.6 | 2.437 GHz | 20 MHz | 20 dBm | 4 dBi",
                    height=22,
                )
            ui.Label("Mouse camera input is independent from RACER", height=22)
    return window, status, clock, speed, rpm, map_status, camera_status


def main() -> None:
    trajectory = RecordedTrajectory(ARGS.trajectory)
    if trajectory.drone_count > len(COLORS):
        raise RuntimeError("the replay viewer supports at most five drones")
    if ARGS.follow_drone >= trajectory.drone_count:
        raise RuntimeError(
            f"--follow-drone {ARGS.follow_drone} is outside the recorded fleet"
        )
    stage, translate_ops, orient_ops = build_stage(trajectory)
    for _ in range(8):
        simulation_app.update()
    pointcloud = (
        RecordedPointCloudMap(ARGS.pointcloud_map)
        if ARGS.pointcloud_map is not None
        else None
    )
    propellers = PropellerAnimator(
        stage, trajectory.drone_count, ARGS.propeller_visual_hz
    )
    paths = RecordedPathRenderer(trajectory)
    camera = ReplayCameraController(
        trajectory.drone_count,
        ARGS.follow_drone,
        ARGS.camera_mode,
    )
    if ARGS.initial_view == "ap":
        if ARGS.base_station_usd is None:
            raise RuntimeError("--initial-view ap requires --base-station-usd")
        camera.access_point()
    else:
        set_overview_camera()
    simulation_app.update()
    (
        overlay,
        status_label,
        clock_label,
        speed_label,
        rpm_label,
        map_status_label,
        camera_status_label,
    ) = create_overlay(trajectory, camera, pointcloud)

    print(
        "RACER_TRAJECTORY_REPLAY_READY "
        + json.dumps(
            {
                "trajectory": str(ARGS.trajectory),
                "scene_usd": str(ARGS.scene_usd),
                "vehicle_usd": str(ARGS.vehicle_usd),
                "drone_count": trajectory.drone_count,
                "duration_s": trajectory.duration,
                "playback_speed": ARGS.speed,
                "target_fps": ARGS.fps,
                "propeller_count": len(propellers.entries),
                "camera_controls": "overview plus D0..D4 chase/onboard",
                "pointcloud_map": (
                    str(ARGS.pointcloud_map)
                    if ARGS.pointcloud_map is not None
                    else None
                ),
                "pointcloud_voxels": pointcloud.count if pointcloud else 0,
                "pointcloud_semantics": (
                    pointcloud.metadata.get("semantics") if pointcloud else None
                ),
                "base_station_usd": (
                    str(ARGS.base_station_usd)
                    if ARGS.base_station_usd is not None
                    else None
                ),
                "base_station_position_m": (
                    BASE_STATION_POSITION.tolist()
                    if ARGS.base_station_usd is not None
                    else None
                ),
                "base_station_radio": (
                    {
                        "standard": "IEEE 802.11ax",
                        "carrier_frequency_hz": 2.437e9,
                        "channel": 6,
                        "bandwidth_hz": 20.0e6,
                        "tx_power_dbm": 20.0,
                        "antenna_gain_dbi": 4.0,
                        "receiver_noise_figure_db": 7.0,
                    }
                    if ARGS.base_station_usd is not None
                    else None
                ),
                "loop": ARGS.loop,
                "recording_metadata": trajectory.metadata,
            },
            sort_keys=True,
        ),
        flush=True,
    )

    frame_period = 1.0 / ARGS.fps
    playback_started = time.monotonic()
    previous_playback_time = trajectory.start_time
    report_started = playback_started
    report_frames = 0
    completed = False
    try:
        while simulation_app.is_running():
            frame_started = time.monotonic()
            elapsed_wall = frame_started - playback_started
            playback_time = trajectory.start_time + elapsed_wall * ARGS.speed
            if playback_time >= trajectory.end_time:
                if ARGS.loop:
                    playback_started = frame_started
                    playback_time = trajectory.start_time
                    previous_playback_time = trajectory.start_time
                    propellers.reset()
                    paths.reset()
                    if pointcloud is not None:
                        pointcloud.reset()
                else:
                    playback_time = trajectory.end_time
                    completed = True

            positions, orientations, velocities, motor_rpm = trajectory.interpolate(
                playback_time
            )
            for drone_id in range(trajectory.drone_count):
                position = positions[drone_id]
                orientation = normalized_quaternion(orientations[drone_id])
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
            simulation_dt = max(0.0, playback_time - previous_playback_time)
            propellers.step(motor_rpm, simulation_dt)
            previous_playback_time = playback_time
            mapped_voxels = 0
            if pointcloud is not None:
                mapped_voxels = pointcloud.update(
                    playback_time - trajectory.start_time
                )
            paths.update(playback_time)
            camera.update(positions, orientations, velocities)
            camera_status_label.text = "Camera: " + camera.description

            status_label.text = (
                "Replay complete - close window when finished"
                if completed
                else f"Playing at {ARGS.speed:.2f}x (wall-clock synchronized)"
            )
            clock_label.text = (
                f"Recorded time: {playback_time:.3f} / "
                f"{trajectory.end_time:.3f} s"
            )
            speed_values = np.linalg.norm(velocities, axis=1)
            speed_label.text = "Drone speeds: " + ", ".join(
                f"D{index} {value:.2f} m/s"
                for index, value in enumerate(speed_values)
            )
            rpm_label.text = (
                f"Motor RPM range: {float(np.min(motor_rpm)):.0f} - "
                f"{float(np.max(motor_rpm)):.0f}"
            )
            map_status_label.text = (
                f"3-D map: {mapped_voxels:,} / {pointcloud.count:,} surface voxels"
                if pointcloud is not None
                else "3-D map: not loaded"
            )

            simulation_app.update()
            report_frames += 1
            report_now = time.monotonic()
            report_span = report_now - report_started
            if report_span >= 5.0:
                print(
                    "RACER_TRAJECTORY_REPLAY_FPS "
                    + json.dumps(
                        {
                            "measured_fps": report_frames / report_span,
                            "target_fps": ARGS.fps,
                            "playback_time_s": playback_time,
                            "speed": ARGS.speed,
                            "mapped_voxels": mapped_voxels,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                report_started = report_now
                report_frames = 0
            remaining = frame_period - (time.monotonic() - frame_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        overlay.visible = False
        simulation_app.close()


if __name__ == "__main__":
    main()
