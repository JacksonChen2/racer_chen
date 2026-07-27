"""Decentralized ROS 2 RACER agent using a shared-frame 3-D voxel map."""

import base64
import json
import math
import zlib
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .allocation import capacity_partition
from .hgrid import HierarchicalGrid3D
from .planning import ExplorationPlan3D, plan_exploration
from .pointcloud import create_xyzi_cloud, read_xyzi_cloud
from .safety import (
    cbf_swarm_filter,
    emergency_separation,
    esdf_obstacle_filter,
    limit_norm,
    predicted_path_conflict,
)
from .scenario import DEFAULT_SCENARIO, get_scenario
from .voxel_map import OCCUPIED, VoxelMap


Point3 = Tuple[float, float, float]


class Racer3DAgent(Node):
    """One independently runnable three-dimensional RACER fleet member."""

    def __init__(self) -> None:
        super().__init__("racer_3d_agent")
        self.declare_parameter("scenario_name", DEFAULT_SCENARIO.name)
        scenario = get_scenario(
            str(self.get_parameter("scenario_name").value)
        )
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("drone_count", 3)
        self.declare_parameter(
            "start_positions",
            [value for point in scenario.starts for value in point],
        )
        self.declare_parameter("map_resolution", 0.20)
        self.declare_parameter("map_origin", list(scenario.map_min))
        self.declare_parameter("map_size", list(scenario.map_size))
        self.declare_parameter("lidar_range", 7.0)
        self.declare_parameter("planning_clearance", 0.22)
        self.declare_parameter("control_clearance", 0.45)
        self.declare_parameter("swarm_safe_distance", 0.65)
        self.declare_parameter("emergency_distance", 0.80)
        self.declare_parameter("max_speed", 0.35)
        self.declare_parameter("max_acceleration", 1.4)
        self.declare_parameter("planning_period", 1.0)
        self.declare_parameter("pairwise_period", 3.0)
        self.declare_parameter("peer_timeout", 3.0)
        self.declare_parameter("completion_coverage", 0.90)
        self.declare_parameter(
            "coarse_grid_size", list(scenario.coarse_grid_size)
        )
        self.declare_parameter("hgrid_levels", 2)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.drone_count = int(self.get_parameter("drone_count").value)
        flattened = [
            float(value)
            for value in self.get_parameter("start_positions").value
        ]
        self.starts = [
            tuple(flattened[index:index + 3])
            for index in range(0, len(flattened) - 2, 3)
        ]
        self.map = VoxelMap(
            float(self.get_parameter("map_resolution").value),
            self.get_parameter("map_origin").value,
            self.get_parameter("map_size").value,
        )
        self.hgrid = HierarchicalGrid3D(
            self.map,
            self.get_parameter("coarse_grid_size").value,
            int(self.get_parameter("hgrid_levels").value),
        )
        self.lidar_range = float(self.get_parameter("lidar_range").value)
        self.clearance = float(
            self.get_parameter("planning_clearance").value
        )
        self.control_clearance = float(
            self.get_parameter("control_clearance").value
        )
        self.safe_distance = float(
            self.get_parameter("swarm_safe_distance").value
        )
        self.emergency_distance = float(
            self.get_parameter("emergency_distance").value
        )
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.max_acceleration = float(
            self.get_parameter("max_acceleration").value
        )
        self.pairwise_period = float(
            self.get_parameter("pairwise_period").value
        )
        self.peer_timeout = float(self.get_parameter("peer_timeout").value)
        self.completion_coverage = float(
            self.get_parameter("completion_coverage").value
        )
        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        namespace = f"/drone_{self.drone_id}"
        self.cmd_publisher = self.create_publisher(
            Twist, namespace + "/cmd_vel_3d", reliable
        )
        self.path_publisher = self.create_publisher(
            Path, namespace + "/planned_path_3d", reliable
        )
        self.occupied_publisher = self.create_publisher(
            PointCloud2, namespace + "/occupied_voxels", reliable
        )
        self.map_share_publisher = self.create_publisher(
            String, "/racer_3d/map_share", reliable
        )
        self.state_publisher = self.create_publisher(
            String, "/racer_3d/swarm_state", reliable
        )
        self.allocation_publisher = self.create_publisher(
            String, "/racer_3d/pairwise", reliable
        )
        self.status_publisher = self.create_publisher(
            String, namespace + "/status", reliable
        )
        self.create_subscription(
            Odometry, namespace + "/odom", self._odom_callback, sensor_qos
        )
        self.create_subscription(
            PointCloud2, namespace + "/points", self._cloud_callback, sensor_qos
        )
        self.create_subscription(
            String, "/racer_3d/map_share", self._map_callback, reliable
        )
        self.create_subscription(
            String, "/racer_3d/swarm_state", self._state_callback, reliable
        )
        self.create_subscription(
            String, "/racer_3d/pairwise", self._allocation_callback, reliable
        )
        self.position: Optional[Point3] = None
        self.velocity: Point3 = (0.0, 0.0, 0.0)
        self.yaw = 0.0
        self.sensor_ready = False
        self.peers: Dict[int, dict] = {}
        self.cell_owners: Dict[str, int] = {}
        self.distance_cache: Dict[Tuple[Point3, Point3], float] = {}
        self.owned_cells: List[str] = []
        self.coverage_route: List[str] = []
        self.current_plan: Optional[ExplorationPlan3D] = None
        self.plan_started = 0.0
        self.yield_until = 0.0
        self.map_sequence = 0
        self.last_pairwise = 0.0
        self.create_timer(0.05, self._control_timer)
        self.create_timer(
            float(self.get_parameter("planning_period").value),
            self._planning_timer,
        )
        self.create_timer(0.20, self._publish_state)
        self.create_timer(1.0, self._publish_map)
        self.create_timer(0.5, self._pairwise_timer)
        self.get_logger().info(
            f"RACER 3D agent {self.drone_id}/{self.drone_count} ready"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        velocity = message.twist.twist.linear
        self.position = (position.x, position.y, position.z)
        self.velocity = (velocity.x, velocity.y, velocity.z)
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )

    def _cloud_callback(self, message: PointCloud2) -> None:
        if self.position is None or message.header.frame_id != "map":
            return
        try:
            points, hit = read_xyzi_cloud(message)
        except (ValueError, TypeError):
            return
        self.map.update_point_cloud(
            self.position, points, self.lidar_range, hit, maximum_rays=1200
        )
        self.sensor_ready = True

    def _encode_map(self) -> str:
        states = self.map.states()
        payload = {
            "drone_id": self.drone_id,
            "sequence": self.map_sequence,
            "shape": list(states.shape),
            "resolution": self.map.resolution,
            "data": base64.b64encode(
                zlib.compress(states.tobytes(), level=3)
            ).decode("ascii"),
        }
        return json.dumps(payload, separators=(",", ":"))

    def _map_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if int(payload["drone_id"]) == self.drone_id:
                return
            raw = zlib.decompress(base64.b64decode(payload["data"]))
            values = np.frombuffer(raw, dtype=np.int8).reshape(
                tuple(payload["shape"])
            )
        except (ValueError, KeyError, TypeError, zlib.error, json.JSONDecodeError):
            return
        self.map.merge(values)

    def _state_callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            peer_id = int(state["drone_id"])
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            return
        if peer_id == self.drone_id:
            return
        state["received"] = self._now()
        self.peers[peer_id] = state

    def _allocation_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if int(data.get("to", -1)) != self.drone_id:
            return
        active = self.hgrid.cells
        sender_id = int(data.get("from", -1))
        receiver_cells = set(data.get("cells", []))
        sender_cells = set(data.get("sender_cells", []))
        union = set(data.get("union", receiver_cells | sender_cells))
        for item in union:
            if item not in active:
                continue
            self.cell_owners[item] = (
                self.drone_id if item in receiver_cells else sender_id
            )
        self.owned_cells = sorted(
            item
            for item, owner in self.cell_owners.items()
            if item in active and owner == self.drone_id
        )
        self.coverage_route = [
            item for item in data.get("route", []) if item in active
        ]

    def _active_peers(self) -> Dict[int, dict]:
        now = self._now()
        return {
            drone_id: state
            for drone_id, state in self.peers.items()
            if now - float(state.get("received", 0.0)) < self.peer_timeout
        }

    def _reconcile_ownership(self) -> None:
        active = self.hgrid.cells
        if not active:
            self.cell_owners = {}
            self.owned_cells, self.coverage_route = [], []
            return
        initial = self.hgrid.initial_owners(self.starts[:self.drone_count])
        previous = dict(self.cell_owners)
        updated: Dict[str, int] = {}
        for cell_id, cell in active.items():
            owner = previous.get(cell_id)
            if owner is None and cell.level > 1:
                parent_id = (
                    f"{cell.level - 1}:{cell.ix // 2}:"
                    f"{cell.iy // 2}:{cell.iz // 2}"
                )
                owner = previous.get(parent_id)
            updated[cell_id] = (
                int(owner) if owner is not None else initial[cell_id]
            )
        self.cell_owners = updated
        self.owned_cells = sorted(
            cell_id
            for cell_id, owner in self.cell_owners.items()
            if owner == self.drone_id
        )
        if self.position is not None:
            self.coverage_route = self.hgrid.coverage_route(
                self.owned_cells,
                self.position,
                self._coverage_distance,
            )

    def _coverage_distance(
        self, first: Sequence[float], second: Sequence[float]
    ) -> float:
        """Optimistic collision-free HGrid edge cost, as used by RACER CPs."""

        first_key = tuple(round(float(value), 3) for value in first)
        second_key = tuple(round(float(value), 3) for value in second)
        key = tuple(sorted((first_key, second_key)))
        cached = self.distance_cache.get(key)
        if cached is not None:
            return cached
        start = self.map.world_to_grid(first)
        goal = self.map.world_to_grid(second)
        direct = float(
            np.linalg.norm(np.asarray(second, dtype=float) - np.asarray(first))
        )
        if start is None or goal is None:
            self.distance_cache[key] = direct
            return direct
        search_clearance = (
            self.clearance
            + 0.5 * math.sqrt(3.0) * self.map.resolution
        )
        blocked = self.map.inflated_blocked(
            search_clearance, unknown_is_blocked=False
        )
        blocked[start[2], start[1], start[0]] = False
        blocked[goal[2], goal[1], goal[0]] = False
        line = self.map.ray_cells(first, second)
        if line and not any(
            blocked[cell[2], cell[1], cell[0]] for cell in line
        ):
            self.distance_cache[key] = direct
            return direct
        # RACER uses an incrementally maintained sparse HGrid graph instead
        # of launching a volumetric A* for every CVRP matrix entry.  A blocked
        # direct edge receives a conservative detour estimate here; the local
        # motion planner still computes and validates the full voxel A*.
        cost = 1.6 * direct
        self.distance_cache[key] = float(cost)
        return float(cost)

    def _pairwise_timer(self) -> None:
        now = self._now()
        peers = self._active_peers()
        if (
            self.position is None
            or not peers
            or now - self.last_pairwise < self.pairwise_period
        ):
            return
        # One globally deterministic unordered pair interacts per slot. This
        # preserves RACER's request/response exclusion without a central node.
        pairs = [
            (first, second)
            for first in range(self.drone_count)
            for second in range(first + 1, self.drone_count)
        ]
        if not pairs:
            return
        first_id, peer_id = pairs[
            int(now // self.pairwise_period) % len(pairs)
        ]
        if self.drone_id != first_id or peer_id not in peers:
            return
        peer = peers[peer_id]
        ids = set(self.owned_cells) | set(peer.get("owned_cells", []))
        cells = [
            self.hgrid.cells[item] for item in sorted(ids)
            if item in self.hgrid.cells
        ]
        if not cells:
            return
        peer_position = peer.get("position", self.starts[peer_id])
        self.distance_cache.clear()
        partition = capacity_partition(
            cells,
            self.position,
            peer_position,
            distance_function=self._coverage_distance,
        )
        for item in partition.first:
            self.cell_owners[item] = self.drone_id
        for item in partition.second:
            self.cell_owners[item] = peer_id
        self.owned_cells = partition.first
        self.coverage_route = partition.first_route
        self.allocation_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "from": self.drone_id,
                        "to": peer_id,
                        "cells": partition.second,
                        "sender_cells": partition.first,
                        "union": sorted(ids),
                        "route": partition.second_route,
                        "stamp": now,
                    },
                    separators=(",", ":"),
                )
            )
        )
        self.last_pairwise = now

    def _plan_reusable(self) -> bool:
        if self.position is None or self.current_plan is None:
            return False
        now = self._now()
        elapsed = now - self.plan_started
        if elapsed > self.current_plan.trajectory[-1][0] + 1.0:
            return False
        if np.linalg.norm(
            np.asarray(self.position) - np.asarray(self.current_plan.goal)
        ) < 0.35:
            return False
        for sample in self.current_plan.trajectory:
            if sample[0] + 0.25 < elapsed:
                continue
            cell = self.map.world_to_grid(sample[1:4])
            if (
                cell is None
                or self.map.states()[cell[2], cell[1], cell[0]] < 0
                or self.map.distance_at(sample[1:4], False) < self.clearance
            ):
                return False
        return True

    def _planning_timer(self) -> None:
        if self.position is None or not self.sensor_ready:
            return
        self.distance_cache.clear()
        self.hgrid.update()
        self._reconcile_ownership()
        if self.map.coverage() >= self.completion_coverage:
            self.current_plan = None
        elif not self._plan_reusable():
            plan = plan_exploration(
                self.map,
                self.hgrid,
                self.owned_cells,
                self.coverage_route,
                self.position,
                self.clearance,
                self.max_speed,
                self.max_acceleration,
            )
            if plan is not None:
                absolute = [
                    (self._now() + sample[0], *sample[1:4])
                    for sample in plan.trajectory
                ]
                for peer_id, peer in self._active_peers().items():
                    if (
                        self.drone_id > peer_id
                        and predicted_path_conflict(
                            absolute,
                            peer.get("trajectory", []),
                            self.safe_distance,
                        )
                    ):
                        self.yield_until = max(
                            self.yield_until, self._now() + 0.8
                        )
                self.current_plan = plan
                self.plan_started = self._now()
                self._publish_path(plan)
            else:
                # The previous plan is already expired, reached, outside the
                # map, or invalidated by new occupancy. Retaining it after a
                # failed replan can leave a vehicle commanding an unreachable
                # old endpoint indefinitely.
                self.current_plan = None
        self.status_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "drone_id": self.drone_id,
                        "coverage": self.map.coverage(),
                        "frontier_clusters": len(self.map.frontier_clusters()),
                        "owned_hgrid_cells": len(self.owned_cells),
                        "planning": self.current_plan is not None,
                        "completed": self.map.coverage()
                        >= self.completion_coverage,
                    },
                    separators=(",", ":"),
                )
            )
        )

    def _control_timer(self) -> None:
        message = Twist()
        if (
            self.position is None
            or self.current_plan is None
            or self._now() < self.yield_until
        ):
            message.angular.z = self.yaw
            self.cmd_publisher.publish(message)
            return
        elapsed = self._now() - self.plan_started
        target_time = elapsed + 0.25
        target = self.current_plan.trajectory[-1]
        for sample in self.current_plan.trajectory:
            if sample[0] >= target_time:
                target = sample
                break
        error = np.asarray(target[1:4]) - np.asarray(self.position)
        preferred = limit_norm(1.4 * error, self.max_speed)
        peers = self._active_peers()
        safe = cbf_swarm_filter(
            preferred,
            self.position,
            [
                (
                    peer_id,
                    state.get("position", (0.0, 0.0, 0.0)),
                    state.get("velocity", (0.0, 0.0, 0.0)),
                )
                for peer_id, state in peers.items()
            ],
            self.safe_distance,
            self.max_speed,
        )
        safe = esdf_obstacle_filter(
            safe,
            self.position,
            self.map,
            self.control_clearance,
            self.max_speed,
            current_velocity=self.velocity,
        )
        safe = emergency_separation(
            safe,
            self.position,
            [state.get("position", (0.0, 0.0, 0.0)) for state in peers.values()],
            self.emergency_distance,
            self.max_speed,
        )
        message.linear.x, message.linear.y, message.linear.z = safe
        message.angular.z = self.current_plan.yaw
        self.cmd_publisher.publish(message)

    def _publish_path(self, plan: ExplorationPlan3D) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        for _, x, y, z in plan.trajectory:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_map(self) -> None:
        self.map_sequence += 1
        self.map_share_publisher.publish(String(data=self._encode_map()))
        state = self.map.states()
        cells = np.argwhere(state == OCCUPIED)
        if cells.size:
            points = np.asarray(
                [
                    self.map.grid_to_world((int(x), int(y), int(z)))
                    for z, y, x in cells
                ],
                dtype=np.float32,
            )
            self.occupied_publisher.publish(
                create_xyzi_cloud(
                    self.get_clock().now().to_msg(), "map", points
                )
            )

    def _publish_state(self) -> None:
        if self.position is None:
            return
        trajectory = []
        if self.current_plan is not None:
            trajectory = [
                (self.plan_started + sample[0], *sample[1:4])
                for sample in self.current_plan.trajectory
            ]
        self.state_publisher.publish(
            String(
                data=json.dumps(
                    {
                        "drone_id": self.drone_id,
                        "stamp": self._now(),
                        "position": self.position,
                        "velocity": self.velocity,
                        "owned_cells": self.owned_cells,
                        "trajectory": trajectory,
                    },
                    separators=(",", ":"),
                )
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Racer3DAgent()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
