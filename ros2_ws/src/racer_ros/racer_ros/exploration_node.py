"""ROS 2 port of ``FastExplorationFSM`` and ``exploration_node``."""

from __future__ import annotations

from enum import IntEnum
import math
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

from racer_core import (
    EDTEnvironment,
    FrontierConfig,
    HeadingConfig,
    MapChunk,
    MultiMapManager,
    OptimizerConfig,
    PerceptionConfig,
    PlannerConfig,
    PlannerStatus,
    RacerPlanner,
    VehicleState,
    VoxelMap,
    VoxelMapConfig,
)
from racer_core.bspline import NonUniformBspline
from racer_core.math_utils import quaternion_to_yaw
from racer_interfaces.msg import Bspline, ChunkData, ChunkStamps, DroneState, IdxList

from .conversions import point_message, seconds_to_time


class FsmState(IntEnum):
    INIT = 0
    WAIT_TRIGGER = 1
    PLAN = 2
    EXECUTE = 3
    FINISH = 4


class ExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_node")
        self._declare_parameters()
        self.drone_id = self.get_parameter("drone_id").value
        map_config = VoxelMapConfig(
            resolution=self.get_parameter("map.resolution").value,
            map_size=tuple(self.get_parameter("map.size").value),
            ground_height=self.get_parameter("map.ground_height").value,
            obstacles_inflation=self.get_parameter("map.obstacles_inflation").value,
            optimistic=self.get_parameter("map.optimistic").value,
            p_hit=self.get_parameter("map.p_hit").value,
            p_miss=self.get_parameter("map.p_miss").value,
            p_min=self.get_parameter("map.p_min").value,
            p_max=self.get_parameter("map.p_max").value,
            p_occupied=self.get_parameter("map.p_occupied").value,
            max_ray_length=self.get_parameter("map.max_ray_length").value,
            box_min=tuple(self.get_parameter("map.box_min").value),
            box_max=tuple(self.get_parameter("map.box_max").value),
        )
        self.voxel_map = VoxelMap(map_config)
        environment = EDTEnvironment(self.voxel_map)
        max_velocity = self.get_parameter("manager.max_velocity").value
        max_acceleration = self.get_parameter("manager.max_acceleration").value
        self.planner = RacerPlanner(
            environment,
            PlannerConfig(
                drone_id=int(self.drone_id),
                drone_count=int(self.get_parameter("drone_count").value),
                max_velocity=max_velocity,
                max_acceleration=max_acceleration,
                max_yaw_rate=self.get_parameter("manager.max_yaw_rate").value,
                direction_weight=self.get_parameter("exploration.direction_weight").value,
                control_point_distance=self.get_parameter("manager.control_point_distance").value,
                clearance_threshold=self.get_parameter("manager.clearance_threshold").value,
                use_optimization=self.get_parameter("manager.use_optimization").value,
                use_active_perception=self.get_parameter("manager.use_active_perception").value,
            ),
            FrontierConfig(
                cluster_min=self.get_parameter("frontier.cluster_min").value,
                cluster_size_xy=self.get_parameter("frontier.cluster_size_xy").value,
                cluster_size_z=self.get_parameter("frontier.cluster_size_z").value,
                min_candidate_distance=self.get_parameter("frontier.min_candidate_distance").value,
                min_candidate_clearance=self.get_parameter("frontier.min_candidate_clearance").value,
                candidate_delta_yaw=self.get_parameter("frontier.candidate_delta_yaw").value,
                candidate_radius_count=self.get_parameter("frontier.candidate_radius_count").value,
                candidate_radius_min=self.get_parameter("frontier.candidate_radius_min").value,
                candidate_radius_max=self.get_parameter("frontier.candidate_radius_max").value,
                downsample=self.get_parameter("frontier.downsample").value,
                min_visible_count=self.get_parameter("frontier.min_visible_count").value,
                min_view_finish_fraction=self.get_parameter("frontier.finish_fraction").value,
            ),
            PerceptionConfig(
                top_angle=self.get_parameter("perception.top_angle").value,
                left_angle=self.get_parameter("perception.left_angle").value,
                right_angle=self.get_parameter("perception.right_angle").value,
                max_distance=self.get_parameter("perception.max_distance").value,
                visualization_distance=self.get_parameter("perception.visualization_distance").value,
            ),
            OptimizerConfig(
                lambda_smoothness=self.get_parameter("optimization.lambda_smoothness").value,
                lambda_distance=self.get_parameter("optimization.lambda_distance").value,
                lambda_feasibility=self.get_parameter("optimization.lambda_feasibility").value,
                safe_distance=self.get_parameter("optimization.safe_distance").value,
                max_velocity=max_velocity,
                max_acceleration=max_acceleration,
                max_iterations=self.get_parameter("optimization.max_iterations").value,
            ),
            HeadingConfig(
                yaw_diff=self.get_parameter("heading.yaw_diff").value,
                half_vertical_num=self.get_parameter("heading.half_vertical_num").value,
                max_yaw_rate=self.get_parameter("heading.max_yaw_rate").value,
                weight=self.get_parameter("heading.weight").value,
                info_lambda1=self.get_parameter("heading.lambda1").value,
                info_lambda2=self.get_parameter("heading.lambda2").value,
            ),
        )
        self.state = VehicleState()
        self.multi_map = MultiMapManager(
            int(self.drone_id), int(self.get_parameter("drone_count").value),
            int(self.get_parameter("multi_map.chunk_size").value),
        )
        self.fsm_state = FsmState.INIT
        self.have_odom = False
        self.have_map = False
        self.triggered = False
        self.busy = False
        self.trajectory_id = 0
        self.last_plan_time = -math.inf
        self.lock = threading.Lock()
        self.create_subscription(
            Odometry, "odometry", self._odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2, "pointcloud", self._cloud_callback, qos_profile_sensor_data
        )
        self.create_subscription(PoseStamped, "trigger", self._trigger_callback, 10)
        self.bspline_publisher = self.create_publisher(Bspline, "planning/bspline", 10)
        self.swarm_trajectory_publisher = self.create_publisher(
            Bspline, "/planning/swarm_traj", 100
        )
        self.new_publisher = self.create_publisher(Empty, "planning/new", 10)
        self.replan_publisher = self.create_publisher(Empty, "planning/replan", 10)
        self.state_publisher = self.create_publisher(
            DroneState, "/swarm_expl/drone_state", 10
        )
        self.create_subscription(
            DroneState, "/swarm_expl/drone_state", self._swarm_state_callback, 10
        )
        self.create_subscription(
            Bspline, "/planning/swarm_traj", self._swarm_trajectory_callback, 100
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "planning/markers", 10
        )
        self.chunk_publisher = self.create_publisher(
            ChunkData, "/multi_map_manager/chunk_data", 100
        )
        self.chunk_stamps_publisher = self.create_publisher(
            ChunkStamps, "/multi_map_manager/chunk_stamps", 10
        )
        self.create_subscription(
            ChunkData, "/multi_map_manager/chunk_data", self._chunk_callback, 100
        )
        self.create_subscription(
            ChunkStamps, "/multi_map_manager/chunk_stamps", self._chunk_stamps_callback, 10
        )
        self.create_timer(self.get_parameter("fsm.period").value, self._fsm_tick)
        self.create_timer(self.get_parameter("fsm.sync_interval").value, self._publish_state)
        self.create_timer(0.1, self._publish_chunk_stamps)

    def _declare_parameters(self) -> None:
        values = {
            "drone_id": 1, "drone_count": 1, "frame_id": "world",
            "multi_map.chunk_size": 200,
            "fsm.period": 0.05, "fsm.replan_time": 0.2, "fsm.sync_interval": 0.2,
            "map.resolution": 0.1, "map.size": [35.0, 35.0, 3.5],
            "map.ground_height": -1.0, "map.obstacles_inflation": 0.199,
            "map.optimistic": True, "map.p_hit": 0.65, "map.p_miss": 0.35,
            "map.p_min": 0.12, "map.p_max": 0.90, "map.p_occupied": 0.80,
            "map.max_ray_length": 4.5, "map.box_min": [-7.0, -15.0, 0.0],
            "map.box_max": [7.0, 15.0, 1.7],
            "manager.max_velocity": 2.0, "manager.max_acceleration": 2.0,
            "manager.max_yaw_rate": math.radians(120.0),
            "manager.control_point_distance": 0.5, "manager.clearance_threshold": 0.2,
            "manager.use_optimization": True, "manager.use_active_perception": True,
            "exploration.direction_weight": 1.5,
            "frontier.cluster_min": 100, "frontier.cluster_size_xy": 2.0,
            "frontier.cluster_size_z": 10.0, "frontier.min_candidate_distance": 0.5,
            "frontier.min_candidate_clearance": 0.21,
            "frontier.candidate_delta_yaw": math.radians(15.0),
            "frontier.candidate_radius_count": 3, "frontier.candidate_radius_min": 1.0,
            "frontier.candidate_radius_max": 1.5, "frontier.downsample": 3,
            "frontier.min_visible_count": 30, "frontier.finish_fraction": 0.2,
            "perception.top_angle": 0.56125, "perception.left_angle": 0.69222,
            "perception.right_angle": 0.68901, "perception.max_distance": 4.5,
            "perception.visualization_distance": 1.0,
            "optimization.lambda_smoothness": 5.0,
            "optimization.lambda_distance": 10.0,
            "optimization.lambda_feasibility": 2.0,
            "optimization.safe_distance": 0.7, "optimization.max_iterations": 200,
            "heading.yaw_diff": math.radians(30.0), "heading.half_vertical_num": 5,
            "heading.max_yaw_rate": math.radians(10.0), "heading.weight": 20000.0,
            "heading.lambda1": 2.0, "heading.lambda2": 1.0,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _odom_callback(self, message: Odometry) -> None:
        position, velocity = message.pose.pose.position, message.twist.twist.linear
        orientation = message.pose.pose.orientation
        self.state.position[:] = position.x, position.y, position.z
        self.state.velocity[:] = velocity.x, velocity.y, velocity.z
        self.state.yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.state.yaw_rate = message.twist.twist.angular.z
        self.state.stamp = self.get_clock().now().nanoseconds * 1.0e-9
        self.have_odom = True

    def _cloud_callback(self, message: PointCloud2) -> None:
        if not self.have_odom:
            return
        points = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"))
        if points.size == 0:
            return
        with self.lock:
            addresses = self.voxel_map.input_point_cloud(
                np.asarray(points[:, :3]), self.state.position
            )
            self.voxel_map.inflate_local_map()
            self.voxel_map.update_esdf()
            chunks = self.multi_map.append_addresses(addresses, self._address_occupancy)
        for chunk in chunks:
            self._publish_chunk(chunk, -1)
        self.have_map = True

    def _address_occupancy(self, address: int) -> int:
        return int(
            self.voxel_map.get_occupancy(self.voxel_map.address_to_index(address)) == 2
        )

    def _publish_chunk(self, chunk: MapChunk, target: int) -> None:
        message = ChunkData()
        message.from_drone_id = int(self.drone_id)
        message.to_drone_id = int(target)
        message.chunk_drone_id = int(chunk.owner)
        message.idx = int(chunk.index)
        message.voxel_adrs = chunk.addresses
        message.voxel_occ = chunk.occupancy
        message.latest_idx = len(self.multi_map.chunks.get(chunk.owner, {}))
        message.pos_x, message.pos_y, message.pos_z = map(float, self.state.position)
        self.chunk_publisher.publish(message)

    def _chunk_callback(self, message: ChunkData) -> None:
        if message.from_drone_id == self.drone_id:
            return
        if message.to_drone_id not in (-1, self.drone_id):
            return
        chunk = MapChunk(
            int(message.chunk_drone_id), int(message.idx),
            list(message.voxel_adrs), list(message.voxel_occ),
        )
        if not self.multi_map.insert(chunk):
            return
        with self.lock:
            for address, occupied in zip(chunk.addresses, chunk.occupancy):
                index = self.voxel_map.address_to_index(address)
                if self.voxel_map.is_in_map(index):
                    key = tuple(int(value) for value in index)
                    self.voxel_map.occupancy[key] = (
                        self.voxel_map.clamp_max_log
                        if occupied else self.voxel_map.clamp_min_log
                    )
            self.voxel_map.inflate_local_map()
            self.voxel_map.update_esdf()

    def _publish_chunk_stamps(self) -> None:
        message = ChunkStamps()
        message.from_drone_id = int(self.drone_id)
        message.time = self.get_clock().now().nanoseconds * 1.0e-9
        for owner in range(1, self.multi_map.drone_count + 1):
            item = IdxList()
            item.ids = self.multi_map.index_intervals(owner)
            message.idx_lists.append(item)
        self.chunk_stamps_publisher.publish(message)

    def _chunk_stamps_callback(self, message: ChunkStamps) -> None:
        if message.from_drone_id == self.drone_id:
            return
        for owner, remote in enumerate(message.idx_lists, start=1):
            for index in self.multi_map.missing(owner, list(remote.ids)):
                self._publish_chunk(
                    self.multi_map.chunks[owner][index], int(message.from_drone_id)
                )

    def _trigger_callback(self, _: PoseStamped) -> None:
        self.triggered = True

    def _swarm_state_callback(self, message: DroneState) -> None:
        if message.drone_id == self.drone_id or len(message.pos) < 3 or len(message.vel) < 3:
            return
        state = VehicleState(
            position=np.asarray(message.pos[:3], dtype=np.float64),
            velocity=np.asarray(message.vel[:3], dtype=np.float64),
            yaw=float(message.yaw),
            stamp=float(message.stamp),
        )
        self.planner.swarm.update_state(int(message.drone_id), state)

    def _swarm_trajectory_callback(self, message: Bspline) -> None:
        if message.drone_id == self.drone_id or len(message.pos_pts) <= message.order:
            return
        from racer_core.environment import PredictedBox

        control = np.asarray([(point.x, point.y, point.z) for point in message.pos_pts])
        spline = NonUniformBspline(control, int(message.order), 1.0)
        spline.set_knot(message.knots)
        elapsed = (
            self.get_clock().now().nanoseconds * 1.0e-9
            - (message.start_time.sec + message.start_time.nanosec * 1.0e-9)
        )
        elapsed = float(np.clip(elapsed, 0.0, spline.get_time_sum()))
        position = spline.evaluate(elapsed)
        velocity = spline.derivative().evaluate(
            min(elapsed, spline.derivative().get_time_sum())
        )
        boxes = [
            box for box in self.planner.environment.predicted_boxes
            if getattr(box, "drone_id", -1) != message.drone_id
        ]
        box = PredictedBox(position, velocity, np.ones(3), self.state.stamp, int(message.drone_id))
        boxes.append(box)
        self.planner.environment.predicted_boxes = boxes

    def _fsm_tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.fsm_state == FsmState.INIT:
            if self.have_odom and self.have_map:
                self.fsm_state = FsmState.WAIT_TRIGGER
        elif self.fsm_state == FsmState.WAIT_TRIGGER:
            if self.triggered:
                self.fsm_state = FsmState.PLAN
        elif self.fsm_state == FsmState.EXECUTE:
            if (
                now - self.last_plan_time >= self.get_parameter("fsm.replan_time").value
                or self.planner.frontier.is_frontier_covered()
            ):
                self.fsm_state = FsmState.PLAN
                self.replan_publisher.publish(Empty())
        if self.fsm_state == FsmState.PLAN and not self.busy:
            self.busy = True
            try:
                with self.lock:
                    result = self.planner.plan(self.state)
                if result.status == PlannerStatus.SUCCEED:
                    self._publish_trajectory(result)
                    self._publish_markers(result)
                    self.fsm_state = FsmState.EXECUTE
                    self.last_plan_time = now
                elif result.status == PlannerStatus.NO_FRONTIER:
                    self.fsm_state = FsmState.FINISH
                else:
                    self.fsm_state = FsmState.WAIT_TRIGGER
            finally:
                self.busy = False

    def _publish_trajectory(self, result) -> None:
        self.trajectory_id += 1
        message = Bspline()
        message.drone_id = int(self.drone_id)
        message.order = 3
        message.traj_id = self.trajectory_id
        start = self.get_clock().now().nanoseconds * 1.0e-9 + 0.1
        message.start_time = seconds_to_time(start)
        position = NonUniformBspline(result.position_control_points, 3, result.knot_span)
        message.knots = (
            result.position_knots.tolist()
            if result.position_knots is not None else position.get_knot().tolist()
        )
        message.pos_pts = [point_message(point) for point in result.position_control_points]
        message.yaw_pts = np.asarray(result.yaw_control_points).reshape(-1).tolist()
        message.yaw_dt = float(result.knot_span)
        self.bspline_publisher.publish(message)
        self.swarm_trajectory_publisher.publish(message)
        self.new_publisher.publish(Empty())

    def _publish_state(self) -> None:
        message = DroneState()
        message.drone_id = int(self.drone_id)
        message.trajectory_id = int(self.trajectory_id)
        message.stamp = self.get_clock().now().nanoseconds * 1.0e-9
        message.pos = self.state.position.astype(np.float32).tolist()
        message.vel = self.state.velocity.astype(np.float32).tolist()
        message.yaw = float(self.state.yaw)
        self.state_publisher.publish(message)

    def _publish_markers(self, result) -> None:
        markers = MarkerArray()
        path = Marker()
        path.header.frame_id = self.get_parameter("frame_id").value
        path.header.stamp = self.get_clock().now().to_msg()
        path.ns, path.id, path.type, path.action = "racer_path", 0, Marker.LINE_STRIP, Marker.ADD
        path.scale.x, path.color.r, path.color.g, path.color.b, path.color.a = 0.05, 0.1, 0.8, 1.0, 1.0
        path.pose.orientation.w = 1.0
        path.points = [point_message(point) for point in result.path]
        markers.markers.append(path)
        frontiers = Marker()
        frontiers.header, frontiers.ns, frontiers.id = path.header, "racer_frontiers", 1
        frontiers.type, frontiers.action, frontiers.pose.orientation.w = Marker.POINTS, Marker.ADD, 1.0
        frontiers.scale.x = frontiers.scale.y = 0.08
        frontiers.color.r, frontiers.color.g, frontiers.color.a = 1.0, 0.4, 1.0
        frontiers.points = [
            point_message(cell)
            for frontier in self.planner.frontier.frontiers
            for cell in frontier.filtered_cells
        ]
        markers.markers.append(frontiers)
        self.marker_publisher.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
