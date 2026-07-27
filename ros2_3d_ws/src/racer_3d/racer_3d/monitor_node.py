"""Volume-coverage, map-quality, path-length, and collision acceptance monitor."""

import base64
import json
import math
from pathlib import Path
import zlib
from typing import Dict, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .scenario import (
    DEFAULT_SCENARIO,
    DRONE_RADIUS,
    get_scenario,
    obstacle_clearance,
)
from .voxel_map import FREE, OCCUPIED, UNKNOWN, VoxelMap


class Racer3DMonitor(Node):
    def __init__(self) -> None:
        super().__init__("racer_3d_monitor")
        self.declare_parameter("scenario_name", DEFAULT_SCENARIO.name)
        scenario = get_scenario(
            str(self.get_parameter("scenario_name").value)
        )
        self.scenario = scenario
        self.declare_parameter("drone_count", 3)
        self.declare_parameter("duration", 120.0)
        self.declare_parameter("result_file", "/tmp/racer_3d_result.json")
        self.declare_parameter("minimum_coverage", 0.90)
        self.declare_parameter("minimum_free_accuracy", 0.95)
        self.declare_parameter("minimum_occupied_precision", 0.75)
        self.declare_parameter("minimum_surface_recall", 0.35)
        self.declare_parameter("minimum_inter_drone", 0.35)
        self.declare_parameter("minimum_obstacle_clearance", 0.02)
        self.declare_parameter("require_physics_backend", False)
        self.declare_parameter("map_resolution", 0.20)
        self.declare_parameter("truth_mode", scenario.truth_mode)
        self.drone_count = int(self.get_parameter("drone_count").value)
        self.duration = float(self.get_parameter("duration").value)
        self.result_file = str(self.get_parameter("result_file").value)
        self.minimum_coverage = float(
            self.get_parameter("minimum_coverage").value
        )
        self.minimum_free_accuracy = float(
            self.get_parameter("minimum_free_accuracy").value
        )
        self.minimum_occupied_precision = float(
            self.get_parameter("minimum_occupied_precision").value
        )
        self.minimum_surface_recall = float(
            self.get_parameter("minimum_surface_recall").value
        )
        self.minimum_inter_drone = float(
            self.get_parameter("minimum_inter_drone").value
        )
        self.minimum_obstacle_clearance = float(
            self.get_parameter("minimum_obstacle_clearance").value
        )
        self.require_physics_backend = bool(
            self.get_parameter("require_physics_backend").value
        )
        self.truth_mode = str(self.get_parameter("truth_mode").value)
        self.map = VoxelMap(
            float(self.get_parameter("map_resolution").value),
            scenario.map_min,
            scenario.map_size,
        )
        self.truth_occupied = np.zeros(self.map.shape, dtype=bool)
        if self.truth_mode == "analytic_boxes":
            for z in range(self.map.nz):
                for y in range(self.map.ny):
                    for x in range(self.map.nx):
                        point = self.map.grid_to_world((x, y, z))
                        self.truth_occupied[z, y, x] = any(
                            # A voxel is occupied when its cube intersects a
                            # physical obstacle, not only when its centre is
                            # inside.
                            obstacle.contains(
                                point, 0.5 * self.map.resolution
                            )
                            for obstacle in scenario.obstacles
                        )
        elif self.truth_mode != "observed_volume":
            raise ValueError(f"unsupported truth mode: {self.truth_mode}")
        self.truth_free = ~self.truth_occupied
        self.truth_surface = self._surface_mask(self.truth_occupied)
        # Isaac Sim can take tens of seconds to initialize a referenced USD.
        # Start an Isaac acceptance window on its first metrics frame, rather
        # than charging application startup time against simulated flight.
        self.started: Optional[float] = (
            None if self.require_physics_backend else self._now()
        )
        self.completion_time: Optional[float] = None
        self.completion_wall_time: Optional[float] = None
        self.path_lengths = [0.0] * self.drone_count
        self.last_positions: Dict[int, np.ndarray] = {}
        self.positions: Dict[int, np.ndarray] = {}
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.sim_metrics: dict = {}
        self.backend_seen = False
        self.status: Dict[int, dict] = {}
        self.finished = False
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.completion_publisher = self.create_publisher(
            String, "/racer_3d/mission_complete", qos
        )
        self.create_subscription(
            String, "/racer_3d/map_share", self._map_callback, qos
        )
        self.create_subscription(
            String, "/racer_3d/sim_metrics", self._metrics_callback, qos
        )
        for drone_id in range(self.drone_count):
            namespace = f"/drone_{drone_id}"
            self.create_subscription(
                Odometry,
                namespace + "/odom",
                lambda message, index=drone_id: self._odom_callback(
                    index, message
                ),
                qos,
            )
            self.create_subscription(
                String,
                namespace + "/status",
                lambda message, index=drone_id: self._status_callback(
                    index, message
                ),
                qos,
            )
        self.create_timer(0.5, self._check)
        self.get_logger().info(
            f"3-D acceptance monitor: {self.duration:.1f}s, "
            f"target={self.minimum_coverage:.0%}, truth={self.truth_mode}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _surface_mask(occupied: np.ndarray) -> np.ndarray:
        surface = np.zeros_like(occupied)
        surface[1:, :, :] |= occupied[1:, :, :] & ~occupied[:-1, :, :]
        surface[:-1, :, :] |= occupied[:-1, :, :] & ~occupied[1:, :, :]
        surface[:, 1:, :] |= occupied[:, 1:, :] & ~occupied[:, :-1, :]
        surface[:, :-1, :] |= occupied[:, :-1, :] & ~occupied[:, 1:, :]
        surface[:, :, 1:] |= occupied[:, :, 1:] & ~occupied[:, :, :-1]
        surface[:, :, :-1] |= occupied[:, :, :-1] & ~occupied[:, :, 1:]
        return surface

    def _map_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            raw = zlib.decompress(base64.b64decode(payload["data"]))
            values = np.frombuffer(raw, dtype=np.int8).reshape(
                tuple(payload["shape"])
            )
        except (ValueError, KeyError, TypeError, zlib.error, json.JSONDecodeError):
            return
        self.map.merge(values)

    def _metrics_callback(self, message: String) -> None:
        try:
            metrics = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.sim_metrics = metrics
        self.backend_seen = True
        if self.started is None:
            self.started = self._now()

    def _status_callback(self, drone_id: int, message: String) -> None:
        try:
            self.status[drone_id] = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def _odom_callback(self, drone_id: int, message: Odometry) -> None:
        position = np.asarray(
            (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            )
        )
        previous = self.last_positions.get(drone_id)
        if previous is not None:
            distance = float(np.linalg.norm(position - previous))
            if distance < 0.25:
                self.path_lengths[drone_id] += distance
        self.last_positions[drone_id] = position
        self.positions[drone_id] = position
        self.min_obstacle_clearance = min(
            self.min_obstacle_clearance,
            obstacle_clearance(position, self.scenario.obstacles)
            - DRONE_RADIUS,
        )
        values = list(self.positions.values())
        for index, first in enumerate(values):
            for second in values[index + 1:]:
                self.min_inter_drone = min(
                    self.min_inter_drone,
                    float(np.linalg.norm(first - second)),
                )

    def _quality(self) -> dict:
        states = self.map.states()
        if self.truth_mode == "observed_volume":
            return {
                "volume_coverage": float(
                    np.count_nonzero(states != UNKNOWN)
                ) / max(1, states.size),
                "free_space_accuracy": None,
                "occupied_precision": None,
                "obstacle_surface_recall": None,
                "known_voxels": int(np.count_nonzero(states != UNKNOWN)),
                "total_voxels": int(states.size),
                "truth_free_voxels": None,
                "truth_surface_voxels": None,
                "map_quality_ground_truth_available": False,
            }
        known_free_truth = self.truth_free & (states != UNKNOWN)
        predicted_occupied = states == OCCUPIED
        free_coverage = float(np.count_nonzero(known_free_truth)) / max(
            1, np.count_nonzero(self.truth_free)
        )
        free_accuracy = float(
            np.count_nonzero((states == FREE) & known_free_truth)
        ) / max(1, np.count_nonzero(known_free_truth))
        occupied_precision = float(
            np.count_nonzero(predicted_occupied & self.truth_occupied)
        ) / max(1, np.count_nonzero(predicted_occupied))
        # Allow the one-voxel discretization offset around physical surfaces.
        expanded_prediction = predicted_occupied.copy()
        for axis in range(3):
            expanded_prediction |= np.roll(predicted_occupied, 1, axis=axis)
            expanded_prediction |= np.roll(predicted_occupied, -1, axis=axis)
        surface_recall = float(
            np.count_nonzero(expanded_prediction & self.truth_surface)
        ) / max(1, np.count_nonzero(self.truth_surface))
        return {
            "volume_coverage": free_coverage,
            "free_space_accuracy": free_accuracy,
            "occupied_precision": occupied_precision,
            "obstacle_surface_recall": surface_recall,
            "known_voxels": int(np.count_nonzero(states != UNKNOWN)),
            "total_voxels": int(states.size),
            "truth_free_voxels": int(np.count_nonzero(self.truth_free)),
            "truth_surface_voxels": int(np.count_nonzero(self.truth_surface)),
            "map_quality_ground_truth_available": True,
        }

    def _check(self) -> None:
        if self.finished:
            return
        if self.started is None:
            return
        elapsed = self._now() - self.started
        quality = self._quality()
        if (
            self.completion_time is None
            and quality["volume_coverage"] >= self.minimum_coverage
        ):
            self.completion_wall_time = elapsed
            self.completion_time = float(
                self.sim_metrics.get("elapsed", elapsed)
            )
            self.get_logger().info(
                "volume coverage reached at "
                f"{self.completion_time:.2f}s simulated "
                f"({elapsed:.2f}s wall)"
            )
            self.completion_publisher.publish(String(data="true"))
        deadline_elapsed = (
            float(self.sim_metrics.get("elapsed", 0.0))
            if self.require_physics_backend and self.backend_seen
            else elapsed
        )
        completion_settled = (
            self.completion_wall_time is not None
            and elapsed >= self.completion_wall_time + 2.0
        )
        if completion_settled or deadline_elapsed >= self.duration:
            self._finish(elapsed, quality)

    def _finish(self, elapsed: float, quality: dict) -> None:
        collisions = int(self.sim_metrics.get("collision_events", 0))
        physics_contacts = int(
            self.sim_metrics.get("physics_contact_events", collisions)
        )
        backend = str(self.sim_metrics.get("backend", "not_received"))
        physics_ok = (
            not self.require_physics_backend
            or backend == "isaac_sim_physx_3d"
        )
        min_inter = min(
            self.min_inter_drone,
            float(self.sim_metrics.get("min_inter_drone", math.inf)),
        )
        min_obstacle = min(
            self.min_obstacle_clearance,
            float(
                self.sim_metrics.get("min_obstacle_clearance", math.inf)
            ),
        )
        mission_elapsed = float(
            self.sim_metrics.get("elapsed", elapsed)
        )
        map_quality_ok = (
            self.truth_mode == "observed_volume"
            or (
                quality["free_space_accuracy"]
                >= self.minimum_free_accuracy
                and quality["occupied_precision"]
                >= self.minimum_occupied_precision
                and quality["obstacle_surface_recall"]
                >= self.minimum_surface_recall
            )
        )
        passed = bool(
            self.backend_seen
            and physics_ok
            and self.completion_time is not None
            and map_quality_ok
            and collisions == 0
            and physics_contacts == 0
            and min_inter >= self.minimum_inter_drone
            and min_obstacle >= self.minimum_obstacle_clearance
            and len(self.positions) == self.drone_count
        )
        result = {
            "passed": passed,
            "backend": backend,
            "scenario": self.scenario.name,
            "dimensions_m": list(self.scenario.map_size),
            "drone_count": self.drone_count,
            "elapsed_s": mission_elapsed,
            "wall_elapsed_s": elapsed,
            "completion_time_s": self.completion_time,
            "completion_wall_time_s": self.completion_wall_time,
            "flight_distance_m": self.path_lengths,
            "total_flight_distance_m": float(sum(self.path_lengths)),
            **quality,
            "collision_events": collisions,
            "physics_contact_events": physics_contacts,
            "safety_interventions": int(
                self.sim_metrics.get("safety_interventions", 0)
            ),
            "low_level_safety": self.sim_metrics.get(
                "low_level_safety", "not_reported"
            ),
            "minimum_inter_drone_m": min_inter,
            "minimum_obstacle_clearance_m": min_obstacle,
            "all_agents_reporting": len(self.status) == self.drone_count,
            "agent_status": self.status,
            "final_positions": {
                str(drone_id): position.tolist()
                for drone_id, position in self.positions.items()
            },
            "requirements": {
                "minimum_coverage": self.minimum_coverage,
                "minimum_free_accuracy": self.minimum_free_accuracy,
                "minimum_occupied_precision": self.minimum_occupied_precision,
                "minimum_surface_recall": self.minimum_surface_recall,
                "minimum_inter_drone": self.minimum_inter_drone,
                "minimum_obstacle_clearance": self.minimum_obstacle_clearance,
                "require_physics_backend": self.require_physics_backend,
                "ground_truth_quality_required": (
                    self.truth_mode == "analytic_boxes"
                ),
            },
        }
        target = Path(self.result_file)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print("RACER_3D_ACCEPTANCE " + json.dumps(result, sort_keys=True),
              flush=True)
        self.finished = True
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Racer3DMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
