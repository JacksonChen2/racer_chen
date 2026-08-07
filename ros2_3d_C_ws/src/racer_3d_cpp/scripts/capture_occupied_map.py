#!/usr/bin/env python3
"""Capture the latest C++ RACER occupied voxel maps and render them.

The C++ agents publish their current shared map as PointCloud2 on
``/drone_N/occupied_voxels``.  This utility keeps the latest map from every
agent, merges the occupied voxel centers, and writes both an ASCII PLY point
cloud and static overview figures.  It is intended to run alongside an Isaac
Sim acceptance test.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import struct
import time
from typing import Dict, Iterable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument("--resolution", type=float, default=0.20)
    parser.add_argument("--timeout", type=float, default=1000.0)
    parser.add_argument("--completion-grace", type=float, default=2.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--title", default="C++ RACER reconstructed occupancy map")
    return parser.parse_args()


def field_offset(message: PointCloud2, name: str) -> Optional[int]:
    for field in message.fields:
        if field.name == name:
            return int(field.offset)
    return None


def cloud_xyz(message: PointCloud2) -> np.ndarray:
    offsets = [field_offset(message, axis) for axis in ("x", "y", "z")]
    if any(offset is None for offset in offsets) or message.point_step <= 0:
        return np.empty((0, 3), dtype=np.float64)
    endian = ">" if message.is_bigendian else "<"
    unpack = struct.Struct(endian + "f").unpack_from
    points = []
    total = int(message.width) * int(message.height)
    ox, oy, oz = (int(value) for value in offsets)
    for index in range(total):
        base = index * int(message.point_step)
        try:
            x = unpack(message.data, base + ox)[0]
            y = unpack(message.data, base + oy)[0]
            z = unpack(message.data, base + oz)[0]
        except (struct.error, IndexError):
            break
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            points.append((x, y, z))
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def merged_voxels(
    latest: Iterable[np.ndarray], resolution: float
) -> np.ndarray:
    nonempty = [points for points in latest if points.size]
    if not nonempty:
        return np.empty((0, 3), dtype=np.float64)
    combined = np.vstack(nonempty)
    keys = np.rint(combined / resolution).astype(np.int64)
    unique = np.unique(keys, axis=0)
    return unique.astype(np.float64) * resolution


def write_ply(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write(f"element vertex {len(points)}\n")
        stream.write("property float x\n")
        stream.write("property float y\n")
        stream.write("property float z\n")
        stream.write("end_header\n")
        for x, y, z in points:
            stream.write(f"{x:.4f} {y:.4f} {z:.4f}\n")


def equal_3d_axes(axis, points: np.ndarray) -> None:
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = 0.5 * (minimum + maximum)
    radius = 0.5 * float(np.max(maximum - minimum))
    radius = max(radius, 0.5)
    axis.set_xlim(center[0] - radius, center[0] + radius)
    axis.set_ylim(center[1] - radius, center[1] + radius)
    axis.set_zlim(max(0.0, center[2] - radius), center[2] + radius)


def render_maps(
    output_dir: Path, points: np.ndarray, title: str, resolution: float
) -> None:
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg")
    # Debian's ``mpl_toolkits`` namespace can precede a newer user-installed
    # Matplotlib on sys.path.  Point the namespace at the toolkit shipped with
    # the selected Matplotlib so the 3-D projection uses a matching version.
    import mpl_toolkits

    matching_toolkits = (
        Path(matplotlib.__file__).resolve().parent.parent / "mpl_toolkits"
    )
    if (
        matching_toolkits.is_dir()
        and str(matching_toolkits) not in mpl_toolkits.__path__
    ):
        mpl_toolkits.__path__.insert(0, str(matching_toolkits))
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    import matplotlib.pyplot as plt

    z_low = 0.30
    z_high = 8.20
    obstacles = points[(points[:, 2] >= z_low) & (points[:, 2] <= z_high)]
    if obstacles.size == 0:
        obstacles = points

    figure = plt.figure(figsize=(16, 11), constrained_layout=True)
    figure.suptitle(
        f"{title}\n{len(points):,} occupied voxels, {resolution:.2f} m resolution",
        fontsize=15,
    )
    axis_3d = figure.add_subplot(2, 2, 1, projection="3d")
    scatter = axis_3d.scatter(
        obstacles[:, 0],
        obstacles[:, 1],
        obstacles[:, 2],
        c=obstacles[:, 2],
        cmap="viridis",
        s=2.0,
        alpha=0.62,
        linewidths=0,
    )
    axis_3d.set_title("3-D occupied voxels (floor/ceiling suppressed)")
    axis_3d.set_xlabel("x [m]")
    axis_3d.set_ylabel("y [m]")
    axis_3d.set_zlabel("z [m]")
    axis_3d.view_init(elev=28, azim=-58)
    equal_3d_axes(axis_3d, obstacles)
    figure.colorbar(scatter, ax=axis_3d, shrink=0.62, label="height z [m]")

    projections = (
        ((0, 1), "Top view (XY)", "x [m]", "y [m]"),
        ((0, 2), "Side view (XZ)", "x [m]", "z [m]"),
        ((1, 2), "Front view (YZ)", "y [m]", "z [m]"),
    )
    for plot_index, (axes, label, xlabel, ylabel) in enumerate(
        projections, start=2
    ):
        axis = figure.add_subplot(2, 2, plot_index)
        axis.scatter(
            obstacles[:, axes[0]],
            obstacles[:, axes[1]],
            c=obstacles[:, 2],
            cmap="viridis",
            s=1.5,
            alpha=0.50,
            linewidths=0,
        )
        axis.set_title(label)
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.18)
    figure.savefig(output_dir / "occupied_map_views.png", dpi=220)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(14, 8), constrained_layout=True)
    top = axis.scatter(
        obstacles[:, 0],
        obstacles[:, 1],
        c=obstacles[:, 2],
        cmap="turbo",
        s=3.0,
        alpha=0.68,
        linewidths=0,
    )
    axis.set_title(f"{title} — obstacle top view")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.18)
    figure.colorbar(top, ax=axis, label="height z [m]")
    figure.savefig(output_dir / "occupied_map_top.png", dpi=240)
    plt.close(figure)


class MapCapture(Node):
    def __init__(self, arguments: argparse.Namespace):
        super().__init__("racer_3d_cpp_map_capture")
        self.arguments = arguments
        self.latest: Dict[int, np.ndarray] = {}
        self.received_messages: Dict[int, int] = {}
        self.started = time.monotonic()
        self.completed_at: Optional[float] = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_subscriptions = []
        for drone_id in range(arguments.drone_count):
            self.map_subscriptions.append(
                self.create_subscription(
                    PointCloud2,
                    f"/drone_{drone_id}/occupied_voxels",
                    lambda message, identifier=drone_id: self.on_cloud(
                        identifier, message
                    ),
                    qos,
                )
            )
        self.complete_subscription = self.create_subscription(
            String,
            "/racer_3d/mission_complete",
            self.on_complete,
            qos,
        )

    def on_cloud(self, drone_id: int, message: PointCloud2) -> None:
        self.latest[drone_id] = cloud_xyz(message)
        self.received_messages[drone_id] = (
            self.received_messages.get(drone_id, 0) + 1
        )

    def on_complete(self, message: String) -> None:
        if message.data.strip().lower() == "true" and self.completed_at is None:
            self.completed_at = time.monotonic()

    def should_stop(self) -> bool:
        elapsed = time.monotonic() - self.started
        if elapsed >= self.arguments.timeout:
            return True
        return (
            self.completed_at is not None
            and time.monotonic() - self.completed_at
            >= self.arguments.completion_grace
        )

    def save(self) -> None:
        output_dir = self.arguments.output_dir.resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        points = merged_voxels(
            self.latest.values(), self.arguments.resolution
        )
        if points.size == 0:
            raise RuntimeError("no occupied voxel messages were captured")
        np.save(output_dir / "occupied_voxels.npy", points)
        write_ply(output_dir / "occupied_voxels.ply", points)
        summary = {
            "drone_count": self.arguments.drone_count,
            "map_resolution_m": self.arguments.resolution,
            "occupied_voxel_count": int(len(points)),
            "bounds_min": points.min(axis=0).tolist(),
            "bounds_max": points.max(axis=0).tolist(),
            "messages_per_drone": self.received_messages,
            "mission_complete_received": self.completed_at is not None,
            "capture_wall_time_s": time.monotonic() - self.started,
        }
        (output_dir / "map_capture_summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        render_maps(
            output_dir, points, self.arguments.title, self.arguments.resolution
        )
        print(json.dumps(summary, sort_keys=True), flush=True)


def main() -> None:
    arguments = parse_args()
    rclpy.init()
    node = MapCapture(arguments)
    exit_code = 0
    try:
        while rclpy.ok() and not node.should_stop():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.save()
    except Exception as error:  # noqa: BLE001 - command-line diagnostic
        print(f"map capture failed: {error}", flush=True)
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
