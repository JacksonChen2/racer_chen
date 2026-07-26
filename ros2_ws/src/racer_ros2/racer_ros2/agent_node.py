"""Decentralized ROS 2 RACER exploration agent."""

import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .allocation import capacity_partition
from .hgrid import HierarchicalGrid
from .mapping import OccupancyMap
from .planning import ExplorationPlan, plan_exploration
from .safety import (
    cbf_swarm_filter,
    emergency_separation,
    limit_norm,
    obstacle_brake,
    predicted_path_conflict,
)


def _yaw_from_quaternion(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


class RacerAgent(Node):
    """One independently runnable member of a RACER fleet."""

    def __init__(self) -> None:
        super().__init__("racer_agent")
        self.declare_parameter("drone_id", 0)
        self.declare_parameter("drone_count", 3)
        self.declare_parameter(
            "start_positions", [-8.2, -4.8, -8.2, 4.8, 7.8, 0.0]
        )
        self.declare_parameter("map_resolution", 0.35)
        self.declare_parameter("map_origin", [-10.0, -7.0])
        self.declare_parameter("map_size", [20.0, 14.0])
        self.declare_parameter("coarse_grid_size", 5.0)
        self.declare_parameter("hgrid_levels", 2)
        self.declare_parameter("hgrid_subdivide_ratio", 0.35)
        self.declare_parameter("hgrid_minimum_unknown", 4)
        self.declare_parameter("planning_clearance", 0.60)
        self.declare_parameter("swarm_safe_distance", 1.15)
        self.declare_parameter("robot_radius", 0.30)
        self.declare_parameter("flight_z", 1.0)
        self.declare_parameter("max_speed", 1.20)
        self.declare_parameter("max_acceleration", 1.50)
        self.declare_parameter("planning_period", 0.8)
        self.declare_parameter("pairwise_period", 3.0)
        self.declare_parameter("peer_timeout", 3.0)
        self.declare_parameter("completion_coverage", 0.95)

        self.drone_id = int(self.get_parameter("drone_id").value)
        self.drone_count = int(self.get_parameter("drone_count").value)
        flattened = [
            float(item) for item in self.get_parameter("start_positions").value
        ]
        self.starts = [
            (flattened[index], flattened[index + 1])
            for index in range(0, len(flattened) - 1, 2)
        ]
        while len(self.starts) < self.drone_count:
            self.starts.append((0.0, float(len(self.starts))))

        origin = tuple(float(item) for item in self.get_parameter("map_origin").value)
        size = tuple(float(item) for item in self.get_parameter("map_size").value)
        resolution = float(self.get_parameter("map_resolution").value)
        self.occupancy_map = OccupancyMap(resolution, origin, size)
        self.hgrid = HierarchicalGrid(
            self.occupancy_map,
            coarse_size=float(self.get_parameter("coarse_grid_size").value),
            levels=int(self.get_parameter("hgrid_levels").value),
            subdivide_known_ratio=float(
                self.get_parameter("hgrid_subdivide_ratio").value
            ),
            minimum_unknown=int(
                self.get_parameter("hgrid_minimum_unknown").value
            ),
        )
        self.clearance = float(self.get_parameter("planning_clearance").value)
        self.safe_distance = float(
            self.get_parameter("swarm_safe_distance").value
        )
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.flight_z = float(self.get_parameter("flight_z").value)
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
            Twist, namespace + "/cmd_vel", reliable
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid, namespace + "/map", reliable
        )
        self.map_share_publisher = self.create_publisher(
            OccupancyGrid, "/racer/map_share", reliable
        )
        self.state_publisher = self.create_publisher(
            String, "/racer/swarm_state", reliable
        )
        self.pairwise_publisher = self.create_publisher(
            String, "/racer/pairwise", reliable
        )
        self.path_publisher = self.create_publisher(
            Path, namespace + "/planned_path", reliable
        )
        self.status_publisher = self.create_publisher(
            String, namespace + "/status", reliable
        )

        self.create_subscription(
            Odometry, namespace + "/odom", self._odom_callback, sensor_qos
        )
        self.create_subscription(
            LaserScan, namespace + "/scan", self._scan_callback, sensor_qos
        )
        self.create_subscription(
            OccupancyGrid, "/racer/map_share", self._map_callback, reliable
        )
        self.create_subscription(
            String, "/racer/swarm_state", self._state_callback, reliable
        )
        self.create_subscription(
            String, "/racer/pairwise", self._pairwise_callback, reliable
        )

        self.position: Optional[Tuple[float, float]] = None
        self.velocity = (0.0, 0.0)
        self.yaw = 0.0
        self.latest_scan: Optional[LaserScan] = None
        self.odom_samples: Dict[
            int, Tuple[Tuple[float, float], float]
        ] = {}
        self.pending_scans: Dict[int, LaserScan] = {}
        self.peers: Dict[int, dict] = {}
        self.owned_cells: List[str] = []
        self.coverage_route: List[str] = []
        self.current_plan: Optional[ExplorationPlan] = None
        self.plan_started = 0.0
        self.map_sequence = 0
        self.pending_proposal: Optional[dict] = None
        self.last_attempt = 0.0
        self.last_interaction: Dict[int, float] = {
            drone: 0.0
            for drone in range(self.drone_count)
            if drone != self.drone_id
        }
        self.busy_until = 0.0
        self.yield_until = 0.0

        planning_period = float(
            self.get_parameter("planning_period").value
        )
        self.create_timer(0.05, self._control_timer)
        self.create_timer(planning_period, self._planning_timer)
        self.create_timer(0.20, self._publish_state)
        self.create_timer(1.0, self._publish_map)
        self.create_timer(0.5, self._pairwise_timer)
        self.get_logger().info(
            f"RACER agent {self.drone_id}/{self.drone_count} ready"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    @staticmethod
    def _stamp_key(message_stamp) -> int:
        return int(message_stamp.sec) * 1_000_000_000 + int(
            message_stamp.nanosec
        )

    def _odom_callback(self, message: Odometry) -> None:
        pose = message.pose.pose
        self.position = (pose.position.x, pose.position.y)
        self.velocity = (
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        )
        self.yaw = _yaw_from_quaternion(
            pose.orientation.z, pose.orientation.w
        )
        stamp = self._stamp_key(message.header.stamp)
        self.odom_samples[stamp] = (self.position, self.yaw)
        while len(self.odom_samples) > 32:
            self.odom_samples.pop(next(iter(self.odom_samples)))
        pending = self.pending_scans.pop(stamp, None)
        if pending is not None:
            self._integrate_scan(pending, self.position, self.yaw)

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message
        stamp = self._stamp_key(message.header.stamp)
        sample = self.odom_samples.get(stamp)
        if sample is None:
            self.pending_scans[stamp] = message
            while len(self.pending_scans) > 8:
                self.pending_scans.pop(next(iter(self.pending_scans)))
            return
        self._integrate_scan(message, sample[0], sample[1])

    def _integrate_scan(
        self,
        message: LaserScan,
        position: Tuple[float, float],
        yaw: float,
    ) -> None:
        self.occupancy_map.update_scan(
            position,
            yaw,
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_max,
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        if message.header.frame_id == f"map_drone_{self.drone_id}":
            return
        info = message.info
        if (
            info.width != self.occupancy_map.width
            or info.height != self.occupancy_map.height
        ):
            return
        values = np.asarray(message.data, dtype=np.int8).reshape(
            (info.height, info.width)
        )
        self.occupancy_map.merge(values)

    def _state_callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            peer_id = int(state["drone_id"])
        except (ValueError, KeyError, TypeError, json.JSONDecodeError):
            return
        if peer_id == self.drone_id or not 0 <= peer_id < self.drone_count:
            return
        state["received"] = self._now()
        self.peers[peer_id] = state

    def _pairwise_callback(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if int(data.get("to", -1)) != self.drone_id:
            return
        kind = data.get("kind")
        now = self._now()
        if kind == "proposal":
            proposal_id = data.get("proposal_id")
            response = {
                "kind": "response",
                "from": self.drone_id,
                "to": int(data.get("from", -1)),
                "proposal_id": proposal_id,
                "accepted": False,
            }
            if now >= self.busy_until:
                active = self.hgrid.cells
                allocated = [
                    cell_id
                    for cell_id in data.get("to_cells", [])
                    if cell_id in active
                ]
                self.owned_cells = allocated
                self.coverage_route = [
                    item
                    for item in data.get("to_route", allocated)
                    if item in active
                ]
                self.busy_until = now + 0.5 * self.pairwise_period
                self.last_interaction[int(data["from"])] = now
                response["accepted"] = True
            self.pairwise_publisher.publish(
                String(data=json.dumps(response, separators=(",", ":")))
            )
        elif (
            kind == "response"
            and self.pending_proposal is not None
            and data.get("proposal_id")
            == self.pending_proposal.get("proposal_id")
        ):
            if bool(data.get("accepted", False)):
                self.owned_cells = list(
                    self.pending_proposal.get("from_cells", [])
                )
                self.coverage_route = list(
                    self.pending_proposal.get("from_route", self.owned_cells)
                )
                peer_id = int(data["from"])
                self.last_interaction[peer_id] = now
            self.pending_proposal = None
            self.busy_until = now

    def _active_peer_states(self) -> Dict[int, dict]:
        now = self._now()
        return {
            peer_id: state
            for peer_id, state in self.peers.items()
            if now - float(state.get("stamp", 0.0)) <= self.peer_timeout
            and now - float(state.get("received", 0.0)) <= self.peer_timeout
        }

    def _reconcile_ownership(self) -> None:
        active = self.hgrid.cells
        if not active:
            self.owned_cells = []
            self.coverage_route = []
            return
        owners = self.hgrid.initial_owners(self.starts[: self.drone_count])
        claims: Dict[str, int] = {}
        for cell_id in self.owned_cells:
            if cell_id in active:
                claims[cell_id] = self.drone_id
        for peer_id, state in self._active_peer_states().items():
            for cell_id in state.get("owned_cells", []):
                if cell_id in active:
                    claims[cell_id] = min(peer_id, claims.get(cell_id, peer_id))
        for cell_id in active:
            if cell_id not in claims:
                claims[cell_id] = owners.get(cell_id, 0)
        self.owned_cells = sorted(
            cell_id
            for cell_id, owner in claims.items()
            if owner == self.drone_id
        )
        position = self.position or self.starts[self.drone_id]
        self.coverage_route = self.hgrid.coverage_route(
            self.owned_cells, position
        )

    def _planning_timer(self) -> None:
        if self.position is None or self.latest_scan is None:
            return
        self.hgrid.update()
        self._reconcile_ownership()
        if self.occupancy_map.coverage() >= self.completion_coverage:
            self.current_plan = None
            status = {
                "drone_id": self.drone_id,
                "coverage": round(self.occupancy_map.coverage(), 5),
                "frontiers": len(self.occupancy_map.frontier_clusters()),
                "owned_cells": len(self.owned_cells),
                "planning": False,
                "completed": True,
            }
            self.status_publisher.publish(
                String(data=json.dumps(status, separators=(",", ":")))
            )
            return
        plan = plan_exploration(
            self.occupancy_map,
            self.hgrid,
            self.owned_cells,
            self.coverage_route,
            self.position,
            self.yaw,
            clearance=self.clearance,
            max_speed=self.max_speed,
            max_acceleration=self.max_acceleration,
        )
        now = self._now()
        if plan is not None:
            absolute_trajectory = [
                (now + point[0], point[1], point[2])
                for point in plan.trajectory
            ]
            for peer_id, peer in self._active_peer_states().items():
                peer_trajectory = [
                    tuple(map(float, item))
                    for item in peer.get("trajectory", [])
                ]
                if (
                    self.drone_id > peer_id
                    and predicted_path_conflict(
                        absolute_trajectory,
                        peer_trajectory,
                        self.safe_distance,
                    )
                ):
                    self.yield_until = max(self.yield_until, now + 0.8)
            self.current_plan = plan
            self.plan_started = now
            self._publish_path(plan)
        elif self.occupancy_map.coverage() > 0.60:
            self.current_plan = None
        status = {
            "drone_id": self.drone_id,
            "coverage": round(self.occupancy_map.coverage(), 5),
            "frontiers": len(self.occupancy_map.frontier_clusters()),
            "owned_cells": len(self.owned_cells),
            "planning": plan is not None,
        }
        self.status_publisher.publish(
            String(data=json.dumps(status, separators=(",", ":")))
        )

    def _control_timer(self) -> None:
        command = Twist()
        if (
            self.position is None
            or self.current_plan is None
            or self._now() < self.yield_until
        ):
            self.cmd_publisher.publish(command)
            return
        elapsed = self._now() - self.plan_started
        lookahead = elapsed + 0.55
        trajectory = self.current_plan.trajectory
        target = trajectory[-1]
        for sample in trajectory:
            if sample[0] >= lookahead:
                target = sample
                break
        error = (target[1] - self.position[0], target[2] - self.position[1])
        preferred = limit_norm((1.8 * error[0], 1.8 * error[1]), self.max_speed)
        peer_constraints = []
        for peer_id, state in self._active_peer_states().items():
            peer_constraints.append(
                (
                    peer_id,
                    tuple(map(float, state.get("position", (0.0, 0.0)))),
                    tuple(map(float, state.get("velocity", (0.0, 0.0)))),
                )
            )
        safe = cbf_swarm_filter(
            preferred,
            self.position,
            peer_constraints,
            self.safe_distance,
            speed_limit=self.max_speed,
        )
        if self.latest_scan is not None:
            safe = obstacle_brake(
                safe,
                self.latest_scan.ranges,
                self.latest_scan.angle_min,
                self.latest_scan.angle_increment,
                self.yaw,
                self.robot_radius,
            )
        safe = emergency_separation(
            safe,
            self.position,
            peer_constraints,
            activation_distance=0.95,
            max_speed=self.max_speed,
        )
        safe = limit_norm(safe, self.max_speed)
        command.linear.x, command.linear.y = safe
        if math.hypot(*safe) > 0.05:
            desired_yaw = math.atan2(safe[1], safe[0])
            error_yaw = math.atan2(
                math.sin(desired_yaw - self.yaw),
                math.cos(desired_yaw - self.yaw),
            )
            command.angular.z = float(np.clip(2.0 * error_yaw, -1.5, 1.5))
        self.cmd_publisher.publish(command)

    def _pairwise_timer(self) -> None:
        now = self._now()
        if (
            self.pending_proposal is not None
            and now - float(self.pending_proposal.get("stamp", now))
            > self.pairwise_period
        ):
            self.pending_proposal = None
            self.busy_until = now
        if (
            self.position is None
            or not self.owned_cells
            or self.pending_proposal is not None
            or now < self.busy_until
            or now - self.last_attempt < self.pairwise_period
        ):
            return
        peers = self._active_peer_states()
        if not peers:
            return
        peer_id = min(
            peers,
            key=lambda item: (self.last_interaction.get(item, 0.0), item),
        )
        # One of a pair initiates, which prevents simultaneous conflicting
        # proposals without requiring a central scheduler.
        if self.drone_id > peer_id:
            return
        peer = peers[peer_id]
        union = set(self.owned_cells) | set(peer.get("owned_cells", []))
        cells = [
            self.hgrid.cells[cell_id]
            for cell_id in union
            if cell_id in self.hgrid.cells
        ]
        if not cells:
            return
        peer_position = tuple(
            map(float, peer.get("position", self.starts[peer_id]))
        )
        previous = {
            cell_id: 0 if cell_id in self.owned_cells else 1
            for cell_id in union
        }
        partition = capacity_partition(
            cells,
            self.position,
            peer_position,
            previous_owner=previous,
        )
        proposal_id = (
            f"{self.drone_id}-{peer_id}-{int(now * 1000000)}"
        )
        proposal = {
            "kind": "proposal",
            "proposal_id": proposal_id,
            "from": self.drone_id,
            "to": peer_id,
            "stamp": now,
            "from_cells": partition.first,
            "to_cells": partition.second,
            "from_route": partition.first_route,
            "to_route": partition.second_route,
        }
        self.pending_proposal = proposal
        self.last_attempt = now
        self.busy_until = now + self.pairwise_period
        self.pairwise_publisher.publish(
            String(data=json.dumps(proposal, separators=(",", ":")))
        )

    def _map_message(self) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = f"map_drone_{self.drone_id}"
        message.info.resolution = self.occupancy_map.resolution
        message.info.width = self.occupancy_map.width
        message.info.height = self.occupancy_map.height
        message.info.origin.position.x = self.occupancy_map.origin[0]
        message.info.origin.position.y = self.occupancy_map.origin[1]
        message.info.origin.orientation.w = 1.0
        message.data = self.occupancy_map.states().ravel().astype(int).tolist()
        return message

    def _publish_map(self) -> None:
        if self.latest_scan is None:
            return
        message = self._map_message()
        self.map_publisher.publish(message)
        self.map_share_publisher.publish(message)

    def _publish_state(self) -> None:
        if self.position is None:
            return
        now = self._now()
        trajectory: Sequence[Tuple[float, float, float]] = []
        if self.current_plan is not None:
            trajectory = [
                (self.plan_started + point[0], point[1], point[2])
                for point in self.current_plan.trajectory[::2]
            ]
        state = {
            "drone_id": self.drone_id,
            "stamp": now,
            "position": self.position,
            "velocity": self.velocity,
            "owned_cells": self.owned_cells,
            "coverage_route": self.coverage_route,
            "trajectory": trajectory,
            "attempt_stamp": self.last_attempt,
        }
        self.state_publisher.publish(
            String(data=json.dumps(state, separators=(",", ":")))
        )

    def _publish_path(self, plan: ExplorationPlan) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        for _, x, y in plan.trajectory:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.flight_z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_publisher.publish(message)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = RacerAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_publisher.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
