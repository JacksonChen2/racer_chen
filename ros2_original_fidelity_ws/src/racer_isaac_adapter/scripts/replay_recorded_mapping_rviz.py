#!/usr/bin/env python3
"""Replay a recorded RACER trajectory and exact map-coverage diagnostics in RViz.

The headless record/replay test stores vehicle states in an NPZ file and emits
``RACER_MAP_COVERAGE`` once every configured diagnostic period.  It does not
store occupancy/SDF voxel coordinates.  This node therefore publishes only
data that can be reproduced faithfully: recorded poses, traversed paths, the
configured planning box, sensor-range guides, and the recorded per-agent map
coverage history.  The RViz overlay explicitly reports that voxel geometry is
not part of this recording.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path as FilePath
import re
import time
from typing import Dict, List, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


COLORS: Tuple[Tuple[float, float, float, float], ...] = (
    (0.95, 0.20, 0.18, 1.0),
    (0.20, 0.85, 0.30, 1.0),
    (0.18, 0.48, 1.00, 1.0),
    (1.00, 0.78, 0.12, 1.0),
    (0.92, 0.25, 0.92, 1.0),
)

COVERAGE_RE = re.compile(
    r"\[racer_original_exploration_node-\d+\].*?"
    r"\[racer_original_exploration_(\d+)\]: "
    r"RACER_MAP_COVERAGE known=(\d+) total=(\d+) ratio=([0-9.]+)"
)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True, type=FilePath)
    parser.add_argument("--launch-log", required=True, type=FilePath)
    parser.add_argument("--result-json", required=True, type=FilePath)
    parser.add_argument("--vehicle-urdf", required=True, type=FilePath)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--coverage-period", type=float, default=2.0)
    parser.add_argument("--path-hz", type=float, default=2.0)
    parser.add_argument("--path-sample-hz", type=float, default=5.0)
    parser.add_argument("--loop", action="store_true")
    return parser.parse_args()


def load_trajectory(path: FilePath) -> Dict[str, np.ndarray]:
    with np.load(path, allow_pickle=False) as archive:
        data = {key: np.asarray(archive[key]) for key in archive.files}
    required = {"times", "positions", "orientations_wxyz", "metadata_json"}
    missing = required.difference(data)
    if missing:
        raise RuntimeError(f"trajectory is missing arrays: {sorted(missing)}")
    times = np.asarray(data["times"], dtype=np.float64)
    positions = np.asarray(data["positions"], dtype=np.float64)
    orientations = np.asarray(data["orientations_wxyz"], dtype=np.float64)
    if times.ndim != 1 or len(times) < 2 or not np.all(np.diff(times) > 0.0):
        raise RuntimeError("trajectory timestamps must be strictly increasing")
    if positions.shape[:2] != (len(times), orientations.shape[1]):
        raise RuntimeError("trajectory pose arrays have inconsistent shapes")
    data["times"] = times - times[0]
    data["positions"] = positions
    data["orientations_wxyz"] = orientations
    return data


def load_coverage(
    log_path: FilePath,
    drone_count: int,
    diagnostic_period: float,
) -> List[Dict[str, np.ndarray]]:
    samples: List[List[Tuple[int, int, float]]] = [[] for _ in range(drone_count)]
    with log_path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = COVERAGE_RE.search(line)
            if match is None:
                continue
            agent = int(match.group(1)) - 1
            if 0 <= agent < drone_count:
                samples[agent].append(
                    (int(match.group(2)), int(match.group(3)), float(match.group(4)))
                )
    if any(not agent_samples for agent_samples in samples):
        counts = [len(agent_samples) for agent_samples in samples]
        raise RuntimeError(f"incomplete RACER_MAP_COVERAGE history: {counts}")
    parsed: List[Dict[str, np.ndarray]] = []
    for agent_samples in samples:
        values = np.asarray(agent_samples, dtype=np.float64)
        parsed.append(
            {
                "times": diagnostic_period
                * np.arange(1, len(agent_samples) + 1, dtype=np.float64),
                "known": values[:, 0].astype(np.int64),
                "total": values[:, 1].astype(np.int64),
                "ratio": values[:, 2],
            }
        )
    return parsed


def set_color(marker: Marker, rgba: Sequence[float], alpha: float | None = None) -> None:
    marker.color.r = float(rgba[0])
    marker.color.g = float(rgba[1])
    marker.color.b = float(rgba[2])
    marker.color.a = float(rgba[3] if alpha is None else alpha)


def point(x: float, y: float, z: float) -> Point:
    value = Point()
    value.x = float(x)
    value.y = float(y)
    value.z = float(z)
    return value


class RecordedMappingReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("racer_recorded_mapping_replay")
        if args.speed <= 0.0 or args.fps <= 0.0:
            raise RuntimeError("speed and fps must be positive")
        self.args = args
        self.trajectory = load_trajectory(args.trajectory)
        self.times = self.trajectory["times"]
        self.positions = self.trajectory["positions"]
        self.orientations = self.trajectory["orientations_wxyz"]
        self.duration = float(self.times[-1])
        self.drone_count = int(self.positions.shape[1])
        self.metadata = json.loads(str(self.trajectory["metadata_json"].item()))
        self.result = json.loads(args.result_json.read_text(encoding="utf-8"))
        self.coverage = load_coverage(
            args.launch_log, self.drone_count, args.coverage_period
        )
        self.final_ratios = np.asarray(
            self.result["metrics"]["mapping_coverage_per_agent"], dtype=np.float64
        )
        self.final_joint = float(self.result["metrics"]["mapping_coverage_joint"])
        if len(self.final_ratios) != self.drone_count:
            raise RuntimeError("result JSON drone count does not match the trajectory")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_publishers = [
            self.create_publisher(Path, f"/racer_replay/drone_{idx}/path", qos)
            for idx in range(self.drone_count)
        ]
        self.description_publishers = [
            self.create_publisher(
                String, f"/racer_replay/drone_{idx}/robot_description", latched_qos
            )
            for idx in range(self.drone_count)
        ]
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/racer_replay/mapping_progress", qos
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        urdf = args.vehicle_urdf.read_text(encoding="utf-8")
        self.robot_descriptions: List[String] = []
        for idx in range(self.drone_count):
            description = String()
            description.data = urdf.replace(
                'robot name="racer_so3_quadrotor"',
                f'robot name="racer_so3_quadrotor_d{idx}"',
                1,
            ).replace(
                '<link name="base_link">', f'<link name="drone_{idx}_base_link">', 1
            )
            self.robot_descriptions.append(description)

        self.start_wall = time.monotonic()
        self.last_path_wall = -math.inf
        self.last_description_wall = -math.inf
        self.last_reported_second = -1
        self.path_stride = max(
            1,
            int(
                round(
                    float(self.metadata.get("sample_rate_hz", 60.0))
                    / args.path_sample_hz
                )
            ),
        )
        self.static_markers = self._make_static_markers()
        self.create_timer(1.0 / args.fps, self._tick)
        self.get_logger().info(
            "RACER_RVIZ_REPLAY_READY "
            + json.dumps(
                {
                    "trajectory": str(args.trajectory),
                    "duration_s": self.duration,
                    "drone_count": self.drone_count,
                    "coverage_samples_per_agent": [
                        len(agent["times"]) for agent in self.coverage
                    ],
                    "final_joint_coverage": self.final_joint,
                    "loop": bool(args.loop),
                    "speed": args.speed,
                    "voxel_geometry_recorded": False,
                },
                sort_keys=True,
            )
        )

    def _header(self, marker: Marker, namespace: str, marker_id: int) -> None:
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

    def _make_static_markers(self) -> List[Marker]:
        markers: List[Marker] = []
        bounds_min = (-10.0, -11.9, 0.4)
        bounds_max = (9.0, 17.6, 8.6)
        corners = [
            (x, y, z)
            for x in (bounds_min[0], bounds_max[0])
            for y in (bounds_min[1], bounds_max[1])
            for z in (bounds_min[2], bounds_max[2])
        ]
        edges = (
            (0, 1), (0, 2), (0, 4), (1, 3), (1, 5), (2, 3),
            (2, 6), (3, 7), (4, 5), (4, 6), (5, 7), (6, 7),
        )
        box = Marker()
        self._header(box, "planning_box", 0)
        box.type = Marker.LINE_LIST
        box.scale.x = 0.06
        set_color(box, (0.85, 0.88, 0.95, 1.0), 0.65)
        for start, end in edges:
            box.points.extend((point(*corners[start]), point(*corners[end])))
        markers.append(box)

        label = Marker()
        self._header(label, "planning_box", 1)
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position = point(-9.8, 17.4, 9.2)
        label.scale.z = 0.55
        label.text = "Original RACER planning box (0.1 m SDF resolution)"
        set_color(label, (0.90, 0.92, 1.0, 1.0))
        markers.append(label)

        chart_axes = Marker()
        self._header(chart_axes, "coverage_chart", 0)
        chart_axes.type = Marker.LINE_LIST
        chart_axes.scale.x = 0.045
        set_color(chart_axes, (0.90, 0.90, 0.90, 1.0), 0.8)
        chart_axes.points.extend(
            (
                point(-10.0, -13.6, 0.4),
                point(9.0, -13.6, 0.4),
                point(-10.0, -13.6, 0.4),
                point(-10.0, -13.6, 8.6),
            )
        )
        markers.append(chart_axes)
        return markers

    def _current_time(self, now_wall: float) -> float:
        elapsed = (now_wall - self.start_wall) * self.args.speed
        if self.args.loop:
            return elapsed % self.duration
        return min(elapsed, self.duration)

    def _trajectory_index(self, replay_time: float) -> int:
        return max(0, int(np.searchsorted(self.times, replay_time, side="right") - 1))

    def _coverage_index(self, drone: int, replay_time: float) -> int:
        return max(
            0,
            int(
                np.searchsorted(
                    self.coverage[drone]["times"], replay_time, side="right"
                )
                - 1
            ),
        )

    def _publish_descriptions(self, now_wall: float) -> None:
        if now_wall - self.last_description_wall < 1.0:
            return
        self.last_description_wall = now_wall
        for publisher, description in zip(
            self.description_publishers, self.robot_descriptions
        ):
            publisher.publish(description)

    def _publish_tf(self, index: int) -> None:
        transforms: List[TransformStamped] = []
        stamp = self.get_clock().now().to_msg()
        for drone in range(self.drone_count):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "world"
            transform.child_frame_id = f"drone_{drone}_base_link"
            position = self.positions[index, drone]
            quaternion = self.orientations[index, drone]
            transform.transform.translation.x = float(position[0])
            transform.transform.translation.y = float(position[1])
            transform.transform.translation.z = float(position[2])
            transform.transform.rotation.w = float(quaternion[0])
            transform.transform.rotation.x = float(quaternion[1])
            transform.transform.rotation.y = float(quaternion[2])
            transform.transform.rotation.z = float(quaternion[3])
            transforms.append(transform)
        self.tf_broadcaster.sendTransform(transforms)

    def _publish_paths(self, index: int, now_wall: float) -> None:
        if now_wall - self.last_path_wall < 1.0 / self.args.path_hz:
            return
        self.last_path_wall = now_wall
        indices = list(range(0, index + 1, self.path_stride))
        if not indices or indices[-1] != index:
            indices.append(index)
        stamp = self.get_clock().now().to_msg()
        for drone, publisher in enumerate(self.path_publishers):
            message = Path()
            message.header.frame_id = "world"
            message.header.stamp = stamp
            for sample in indices:
                pose = PoseStamped()
                pose.header = message.header
                position = self.positions[sample, drone]
                quaternion = self.orientations[sample, drone]
                pose.pose.position.x = float(position[0])
                pose.pose.position.y = float(position[1])
                pose.pose.position.z = float(position[2])
                pose.pose.orientation.w = float(quaternion[0])
                pose.pose.orientation.x = float(quaternion[1])
                pose.pose.orientation.y = float(quaternion[2])
                pose.pose.orientation.z = float(quaternion[3])
                message.poses.append(pose)
            publisher.publish(message)

    def _make_range_ring(self, drone: int, position: np.ndarray) -> Marker:
        ring = Marker()
        self._header(ring, "sensor_range", drone)
        ring.type = Marker.LINE_STRIP
        ring.scale.x = 0.025
        set_color(ring, COLORS[drone % len(COLORS)], 0.18)
        radius = 4.5
        for sample in range(65):
            angle = 2.0 * math.pi * sample / 64.0
            ring.points.append(
                point(
                    position[0] + radius * math.cos(angle),
                    position[1] + radius * math.sin(angle),
                    position[2],
                )
            )
        return ring

    def _make_dynamic_markers(self, index: int, replay_time: float) -> MarkerArray:
        message = MarkerArray()
        message.markers.extend(self.static_markers)

        ratios: List[float] = []
        known_counts: List[int] = []
        for drone in range(self.drone_count):
            coverage_index = self._coverage_index(drone, replay_time)
            data = self.coverage[drone]
            ratio = float(data["ratio"][coverage_index]) if replay_time >= 2.0 else 0.0
            known = int(data["known"][coverage_index]) if replay_time >= 2.0 else 0
            ratios.append(ratio)
            known_counts.append(known)

            graph = Marker()
            self._header(graph, "coverage_graph", drone)
            graph.type = Marker.LINE_STRIP
            graph.scale.x = 0.075
            set_color(graph, COLORS[drone % len(COLORS)])
            graph_indices = np.nonzero(data["times"] <= replay_time)[0]
            for sample in graph_indices:
                x = -10.0 + 19.0 * float(data["times"][sample]) / self.duration
                z = 0.4 + 8.2 * float(data["ratio"][sample])
                graph.points.append(point(x, -13.6, z))
            message.markers.append(graph)

            cursor = Marker()
            self._header(cursor, "coverage_cursor", drone)
            cursor.type = Marker.SPHERE
            cursor.pose.position = point(
                -10.0 + 19.0 * replay_time / self.duration,
                -13.6,
                0.4 + 8.2 * ratio,
            )
            cursor.scale.x = cursor.scale.y = cursor.scale.z = 0.18
            set_color(cursor, COLORS[drone % len(COLORS)])
            message.markers.append(cursor)
            message.markers.append(
                self._make_range_ring(drone, self.positions[index, drone])
            )

        joint = max(ratios) if ratios else 0.0
        status = Marker()
        self._header(status, "status", 0)
        status.type = Marker.TEXT_VIEW_FACING
        status.pose.position = point(-0.5, -14.3, 9.6)
        status.scale.z = 0.52
        lines = [
            f"Recorded RACER mapping progress  t={replay_time:6.1f}/{self.duration:.1f} s  1x",
            f"Joint known-SDF coverage: {100.0 * joint:5.2f}%  (final {100.0 * self.final_joint:5.2f}%)",
            "  ".join(
                f"D{drone}: {100.0 * ratios[drone]:5.2f}%"
                for drone in range(self.drone_count)
            ),
            "Exact poses + exact coverage counts; occupancy/SDF voxel geometry was not recorded",
        ]
        status.text = "\n".join(lines)
        set_color(status, (1.0, 1.0, 1.0, 1.0))
        message.markers.append(status)

        graph_label = Marker()
        self._header(graph_label, "status", 1)
        graph_label.type = Marker.TEXT_VIEW_FACING
        graph_label.pose.position = point(-0.5, -13.7, 9.0)
        graph_label.scale.z = 0.38
        graph_label.text = "Coverage history: horizontal=time, vertical=known SDF voxel ratio"
        set_color(graph_label, (0.92, 0.92, 0.92, 1.0))
        message.markers.append(graph_label)
        return message

    def _tick(self) -> None:
        now_wall = time.monotonic()
        replay_time = self._current_time(now_wall)
        index = self._trajectory_index(replay_time)
        self._publish_descriptions(now_wall)
        self._publish_tf(index)
        self._publish_paths(index, now_wall)
        self.marker_publisher.publish(self._make_dynamic_markers(index, replay_time))
        whole_second = int(replay_time)
        if whole_second % 10 == 0 and whole_second != self.last_reported_second:
            self.last_reported_second = whole_second
            ratios = [
                float(agent["ratio"][self._coverage_index(drone, replay_time)])
                if replay_time >= 2.0
                else 0.0
                for drone, agent in enumerate(self.coverage)
            ]
            self.get_logger().info(
                f"Replay {replay_time:.1f}/{self.duration:.1f}s, "
                f"joint coverage {100.0 * max(ratios):.2f}%"
            )


def main() -> None:
    args = parse_arguments()
    rclpy.init()
    node: RecordedMappingReplay | None = None
    try:
        node = RecordedMappingReplay(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
