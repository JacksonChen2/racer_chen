"""Acceptance monitor for mock and Isaac Sim exploration runs."""

import json
import math
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .scenario import get_scenario, point_box_clearance


class ExplorationMonitor(Node):
    def __init__(self) -> None:
        super().__init__("racer_monitor")
        self.declare_parameter("drone_count", 3)
        self.declare_parameter("duration", 45.0)
        self.declare_parameter("result_file", "/tmp/racer_result.json")
        self.declare_parameter("minimum_coverage", 0.55)
        self.declare_parameter("minimum_inter_drone", 0.70)
        self.declare_parameter("minimum_obstacle_clearance", 0.0)
        self.declare_parameter("require_physics_backend", False)
        self.declare_parameter("scenario", "small")
        self.drone_count = int(self.get_parameter("drone_count").value)
        self.duration = float(self.get_parameter("duration").value)
        self.result_file = str(self.get_parameter("result_file").value)
        self.minimum_coverage = float(
            self.get_parameter("minimum_coverage").value
        )
        self.inter_threshold = float(
            self.get_parameter("minimum_inter_drone").value
        )
        self.obstacle_threshold = float(
            self.get_parameter("minimum_obstacle_clearance").value
        )
        self.require_physics_backend = bool(
            self.get_parameter("require_physics_backend").value
        )
        self.scenario_name = str(self.get_parameter("scenario").value)
        # Isaac Sim may need several seconds to initialize its renderer and ROS
        # bridge. The acceptance window starts on the first plant message, not
        # when the ROS launch process was created.
        self.started = None
        self.coverage: Dict[int, float] = {}
        self.status: Dict[int, dict] = {}
        self.positions: Dict[int, tuple] = {}
        self.initial_positions: Dict[int, tuple] = {}
        self.max_displacements: Dict[int, float] = {}
        self.path_distances: Dict[int, float] = {}
        self.last_path_positions: Dict[int, tuple] = {}
        self.last_odom_stamps: Dict[int, int] = {}
        self.completion_time: Optional[float] = None
        self.completion_coverage: Optional[float] = None
        self.completion_path_distances: Dict[int, float] = {}
        self.maps: Dict[int, dict] = {}
        self.metrics = {
            "backend": "unknown",
            "collision_events": 0,
            "safety_interventions": 0,
            "min_inter_drone": math.inf,
            "min_obstacle_clearance": math.inf,
            "lidar_frames": 0,
            "physics_contact_events": 0,
        }
        self.pairwise_proposals = 0
        self.pairwise_acceptances = 0
        self.finished = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        for drone_id in range(self.drone_count):
            self.create_subscription(
                OccupancyGrid,
                f"/drone_{drone_id}/map",
                lambda message, index=drone_id: self._map(index, message),
                qos,
            )
            self.create_subscription(
                String,
                f"/drone_{drone_id}/status",
                lambda message, index=drone_id: self._status(index, message),
                qos,
            )
            self.create_subscription(
                Odometry,
                f"/drone_{drone_id}/odom",
                lambda message, index=drone_id: self._odom(index, message),
                qos,
            )
        self.create_subscription(
            String, "/racer/sim_metrics", self._metrics, qos
        )
        self.create_subscription(
            String, "/racer/pairwise", self._pairwise, qos
        )
        result_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.result_publisher = self.create_publisher(
            String, "/racer/test_result", result_qos
        )
        self.create_timer(0.25, self._timer)

    def _map(self, drone_id: int, message: OccupancyGrid) -> None:
        if not message.data:
            return
        known = sum(1 for item in message.data if item >= 0)
        self.coverage[drone_id] = known / len(message.data)
        self.maps[drone_id] = {
            "resolution": float(message.info.resolution),
            "width": int(message.info.width),
            "height": int(message.info.height),
            "origin": (
                float(message.info.origin.position.x),
                float(message.info.origin.position.y),
            ),
            "data": np.asarray(message.data, dtype=np.int16).reshape(
                (message.info.height, message.info.width)
            ),
        }
        self._capture_completion()

    def _status(self, drone_id: int, message: String) -> None:
        try:
            self.status[drone_id] = json.loads(message.data)
        except json.JSONDecodeError:
            pass

    def _odom(self, drone_id: int, message: Odometry) -> None:
        if self.started is None:
            self.started = self.get_clock().now()
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )
        position_3d = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        stamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        previous_stamp = self.last_odom_stamps.get(drone_id)
        previous_position = self.last_path_positions.get(drone_id)
        if (
            previous_position is not None
            and previous_stamp is not None
            and stamp > previous_stamp
        ):
            self.path_distances[drone_id] = self.path_distances.get(
                drone_id, 0.0
            ) + math.sqrt(
                sum(
                    (position_3d[axis] - previous_position[axis]) ** 2
                    for axis in range(3)
                )
            )
        else:
            self.path_distances.setdefault(drone_id, 0.0)
        if previous_stamp is None or stamp > previous_stamp:
            self.last_odom_stamps[drone_id] = stamp
            self.last_path_positions[drone_id] = position_3d
        self.positions[drone_id] = position
        if drone_id not in self.initial_positions:
            self.initial_positions[drone_id] = position
            self.max_displacements[drone_id] = 0.0
        initial = self.initial_positions[drone_id]
        self.max_displacements[drone_id] = max(
            self.max_displacements.get(drone_id, 0.0),
            math.hypot(position[0] - initial[0], position[1] - initial[1]),
        )

    def _metrics(self, message: String) -> None:
        try:
            incoming = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if self.started is None:
            self.started = self.get_clock().now()
        self.metrics.update(incoming)
        self._capture_completion()

    def _capture_completion(self) -> None:
        if self.completion_time is not None:
            return
        if len(self.coverage) < self.drone_count:
            return
        fleet_coverage = min(self.coverage.values())
        if fleet_coverage < self.minimum_coverage:
            return
        elapsed = self.metrics.get("elapsed")
        if not isinstance(elapsed, (int, float)) or elapsed <= 0.0:
            if self.started is None:
                return
            elapsed = (
                self.get_clock().now() - self.started
            ).nanoseconds * 1.0e-9
        self.completion_time = float(elapsed)
        self.completion_coverage = float(fleet_coverage)
        self.completion_path_distances = {
            drone_id: self.path_distances.get(drone_id, 0.0)
            for drone_id in range(self.drone_count)
        }

    def _pairwise(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if data.get("kind") == "proposal":
            self.pairwise_proposals += 1
        elif data.get("kind") == "response" and bool(
            data.get("accepted", False)
        ):
            self.pairwise_acceptances += 1

    def _timer(self) -> None:
        if self.started is None:
            return
        elapsed = (self.get_clock().now() - self.started).nanoseconds * 1.0e-9
        if elapsed >= self.duration and not self.finished:
            self._finish(elapsed)

    def _map_quality(self) -> dict:
        if not self.maps:
            return {
                "occupied_cells": 0,
                "known_free_cells": 0,
                "false_free_cells": 0,
                "false_free_rate": 1.0,
                "occupied_precision": 0.0,
            }
        selected_id = max(self.coverage, key=self.coverage.get)
        grid = self.maps[selected_id]
        values = grid["data"]
        resolution = grid["resolution"]
        origin_x, origin_y = grid["origin"]
        scenario = get_scenario(self.scenario_name)
        truth_occupied = np.zeros_like(values, dtype=bool)
        near_obstacle = np.zeros_like(values, dtype=bool)
        obstacle_clearance = np.full_like(values, math.inf, dtype=float)
        for y in range(grid["height"]):
            world_y = origin_y + (y + 0.5) * resolution
            for x in range(grid["width"]):
                world_x = origin_x + (x + 0.5) * resolution
                distances = [
                    point_box_clearance(world_x, world_y, obstacle)
                    for obstacle in scenario.obstacles
                ]
                clearance = min(distances)
                obstacle_clearance[y, x] = clearance
                # A cell center exactly on a box face is a discretization
                # boundary, not evidence that free space was carved through
                # the obstacle interior.
                truth_occupied[y, x] = clearance < -1.0e-9
                near_obstacle[y, x] = clearance <= 1.5 * resolution
        free = values == 0
        occupied = values >= 100
        false_free = int(np.count_nonzero(free & truth_occupied))
        truth_count = max(1, int(np.count_nonzero(truth_occupied)))
        occupied_count = int(np.count_nonzero(occupied))
        precise = int(np.count_nonzero(occupied & near_obstacle))
        false_details = []
        for y, x in np.argwhere(free & truth_occupied):
            false_details.append(
                {
                    "x": origin_x + (int(x) + 0.5) * resolution,
                    "y": origin_y + (int(y) + 0.5) * resolution,
                    "penetration": -float(obstacle_clearance[y, x]),
                }
            )
        return {
            "occupied_cells": occupied_count,
            "known_free_cells": int(np.count_nonzero(free)),
            "false_free_cells": false_free,
            "false_free_rate": false_free / truth_count,
            "occupied_precision": precise / max(1, occupied_count),
            "false_free_details": false_details[:20],
        }

    def _finish(self, elapsed: float) -> None:
        self.finished = True
        coverage = max(self.coverage.values(), default=0.0)
        failures = []
        if len(self.coverage) < self.drone_count:
            failures.append("not all UAV maps were received")
        if coverage < self.minimum_coverage:
            failures.append(
                f"coverage {coverage:.3f} < {self.minimum_coverage:.3f}"
            )
        if self.completion_time is None:
            failures.append(
                "not all UAV maps reached the completion coverage"
            )
        collisions = int(self.metrics.get("collision_events", 0))
        if collisions != 0:
            failures.append(f"{collisions} collision events")
        interventions = int(self.metrics.get("safety_interventions", 0))
        if interventions != 0:
            failures.append(
                f"{interventions} simulator safety-kernel interventions"
            )
        if self.pairwise_acceptances == 0:
            failures.append("no successful RACER pairwise allocation")
        if self.require_physics_backend:
            if self.metrics.get("backend") != "isaac_sim_physx":
                failures.append("backend is not the Isaac PhysX plant")
            if self.metrics.get("motion_source") != "Isaac PhysX rigid body":
                failures.append("odometry is not sourced from an Isaac rigid body")
            if self.metrics.get("sensor_source") != "Isaac RotatingLidarPhysX":
                failures.append("mapping scans are not sourced from Isaac PhysX lidar")
            if int(self.metrics.get("lidar_frames", 0)) < self.drone_count:
                failures.append("no PhysX lidar frame was received for every UAV")
            moving_uavs = sum(
                distance >= 0.50
                for distance in self.max_displacements.values()
            )
            if moving_uavs < min(2, self.drone_count):
                failures.append("fewer than two UAVs moved at least 0.5 m")
        map_quality = self._map_quality()
        if map_quality["occupied_cells"] < 8:
            failures.append("map contains too few occupied obstacle cells")
        if map_quality["false_free_rate"] > 0.05:
            failures.append(
                "map marks more than 5% of obstacle cells as free"
            )
        if map_quality["occupied_precision"] < 0.80:
            failures.append("mapped occupied cells do not match scene obstacles")
        min_inter = float(self.metrics.get("min_inter_drone", 0.0))
        if min_inter < self.inter_threshold:
            failures.append(
                f"minimum inter-UAV distance {min_inter:.3f} "
                f"< {self.inter_threshold:.3f}"
            )
        min_obstacle = float(
            self.metrics.get("min_obstacle_clearance", -math.inf)
        )
        if min_obstacle < self.obstacle_threshold:
            failures.append(
                f"minimum obstacle clearance {min_obstacle:.3f} "
                f"< {self.obstacle_threshold:.3f}"
            )
        result = {
            "passed": not failures,
            "elapsed": elapsed,
            "completion_target": self.minimum_coverage,
            "completion_time": self.completion_time,
            "completion_coverage": self.completion_coverage,
            "drone_count": self.drone_count,
            "coverage": coverage,
            "per_drone_coverage": self.coverage,
            "collision_events": collisions,
            "safety_interventions": interventions,
            "pairwise_proposals": self.pairwise_proposals,
            "pairwise_acceptances": self.pairwise_acceptances,
            "min_inter_drone": min_inter,
            "min_obstacle_clearance": min_obstacle,
            "backend": self.metrics.get("backend", "unknown"),
            "scenario": self.metrics.get("scenario", "unknown"),
            "vehicle_model": self.metrics.get("vehicle_model", "unknown"),
            "motion_source": self.metrics.get("motion_source", "unknown"),
            "sensor_source": self.metrics.get("sensor_source", "unknown"),
            "lidar_frames": int(self.metrics.get("lidar_frames", 0)),
            "physics_contact_events": int(
                self.metrics.get("physics_contact_events", 0)
            ),
            "max_contact_force": float(
                self.metrics.get("max_contact_force", 0.0)
            ),
            "failures": failures,
            "positions": self.positions,
            "max_displacements": self.max_displacements,
            "path_distances": self.path_distances,
            "completion_path_distances": self.completion_path_distances,
            "moving_uavs": sum(
                distance >= 0.50
                for distance in self.max_displacements.values()
            ),
            "map_quality": map_quality,
        }
        output = json.dumps(result, indent=2, sort_keys=True)
        result_path = Path(self.result_file)
        result_path.parent.mkdir(parents=True, exist_ok=True)
        result_path.write_text(output + "\n", encoding="utf-8")
        self.result_publisher.publish(String(data=output))
        if failures:
            self.get_logger().error("RACER acceptance FAILED: " + "; ".join(failures))
        else:
            self.get_logger().info(
                "RACER acceptance PASSED: "
                f"coverage={coverage:.3f}, min_inter={min_inter:.3f}, "
                f"min_obstacle={min_obstacle:.3f}"
            )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExplorationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.finished:
            elapsed = 0.0
            if node.started is not None:
                elapsed = (
                    node.get_clock().now() - node.started
                ).nanoseconds * 1.0e-9
            node._finish(
                elapsed
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
