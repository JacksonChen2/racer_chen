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
    PartitionConfig,
    PerceptionConfig,
    PlannerConfig,
    PlannerStatus,
    RacerPlanner,
    VehicleState,
    VoxelMap,
    VoxelMapConfig,
)
from racer_core.bspline import NonUniformBspline
from racer_core.math_utils import quaternion_to_rotation, quaternion_to_yaw
from racer_interfaces.msg import Bspline, ChunkData, ChunkStamps, DroneState, IdxList

from .conversions import point_message, seconds_to_time


class FsmState(IntEnum):
    INIT = 0
    WAIT_TRIGGER = 1
    PLAN = 2
    EXECUTE = 3
    FINISH = 4
    IDLE = 5


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
                astar_resolution=self.get_parameter("search.astar_resolution").value,
                astar_max_search_time=self.get_parameter("search.astar_max_time").value,
                kino_max_search_time=self.get_parameter("search.kino_max_time").value,
                initial_plan_count=self.get_parameter("search.initial_plan_count").value,
                close_radius=self.get_parameter("search.close_radius").value,
                far_radius=self.get_parameter("search.far_radius").value,
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
                lambda_swarm=self.get_parameter("optimization.lambda_swarm").value,
                swarm_clearance=self.get_parameter("optimization.swarm_clearance").value,
            ),
            HeadingConfig(
                yaw_diff=self.get_parameter("heading.yaw_diff").value,
                half_vertical_num=self.get_parameter("heading.half_vertical_num").value,
                max_yaw_rate=self.get_parameter("heading.max_yaw_rate").value,
                weight=self.get_parameter("heading.weight").value,
                info_lambda1=self.get_parameter("heading.lambda1").value,
                info_lambda2=self.get_parameter("heading.lambda2").value,
            ),
            PartitionConfig(
                minimum_unknown=self.get_parameter("partition.minimum_unknown").value,
                minimum_frontier=self.get_parameter("partition.minimum_frontier").value,
                minimum_free=self.get_parameter("partition.minimum_free").value,
                consistent_cost=self.get_parameter("partition.consistent_cost").value,
                consistent_cost2=self.get_parameter("partition.consistent_cost2").value,
                unknown_weight=self.get_parameter("partition.unknown_weight").value,
                grid_size=self.get_parameter("partition.grid_size").value,
                first_weight=self.get_parameter("partition.first_weight").value,
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
        self.last_idle_check = -math.inf
        self.rotation_world_from_body = np.eye(3, dtype=np.float64)
        self.lidar_translation = np.asarray(
            self.get_parameter("pointcloud.translation").value, dtype=np.float64
        )
        self.pointcloud_is_world = bool(
            self.get_parameter("pointcloud.is_world").value
        )
        self.current_position_trajectory: NonUniformBspline | None = None
        self.current_yaw_trajectory: NonUniformBspline | None = None
        self.current_trajectory_start = 0.0
        self.current_trajectory_duration = 0.0
        self.swarm_trajectories: dict[
            int, tuple[NonUniformBspline, float]
        ] = {}
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
        self.create_timer(0.05, self._safety_tick)
        self.create_timer(self.get_parameter("fsm.sync_interval").value, self._publish_state)
        self.create_timer(0.1, self._publish_chunk_stamps)
        self.create_timer(0.1, self._publish_pending_chunk)

    def _declare_parameters(self) -> None:
        values = {
            "drone_id": 1, "drone_count": 1, "frame_id": "world",
            "multi_map.chunk_size": 200,
            "fsm.period": 0.05, "fsm.replan_time": 0.2, "fsm.sync_interval": 0.2,
            "fsm.replan_near_end": 0.5, "fsm.periodic_replan": 1.0,
            "fsm.idle_retry": 1.0,
            "pointcloud.is_world": False, "pointcloud.translation": [0.0, 0.0, 0.0],
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
            "search.astar_resolution": 0.3, "search.astar_max_time": 0.1,
            "search.kino_max_time": 0.25, "search.initial_plan_count": 2,
            "search.close_radius": 1.5, "search.far_radius": 7.0,
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
            "optimization.lambda_swarm": 10.0,
            "optimization.swarm_clearance": 0.7,
            "partition.minimum_unknown": 4000,
            "partition.minimum_frontier": 100,
            "partition.minimum_free": 3000,
            "partition.consistent_cost": -5.0,
            "partition.consistent_cost2": 8.0,
            "partition.unknown_weight": 0.0,
            "partition.grid_size": 5.0,
            "partition.first_weight": 1.0,
            "heading.yaw_diff": math.radians(30.0), "heading.half_vertical_num": 5,
            "heading.max_yaw_rate": math.radians(10.0), "heading.weight": 20000.0,
            "heading.lambda1": 2.0, "heading.lambda2": 1.0,
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    def _odom_callback(self, message: Odometry) -> None:
        position, velocity = message.pose.pose.position, message.twist.twist.linear
        orientation = message.pose.pose.orientation
        now = self.get_clock().now().nanoseconds * 1.0e-9
        previous_velocity = self.state.velocity.copy()
        previous_stamp = self.state.stamp
        self.state.position[:] = position.x, position.y, position.z
        self.state.velocity[:] = velocity.x, velocity.y, velocity.z
        self.state.yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.rotation_world_from_body = quaternion_to_rotation(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.state.yaw_rate = message.twist.twist.angular.z
        if previous_stamp > 0.0 and now > previous_stamp + 1.0e-4:
            measured = (self.state.velocity - previous_velocity) / (now - previous_stamp)
            self.state.acceleration[:] = np.clip(
                0.2 * measured + 0.8 * self.state.acceleration,
                -self.get_parameter("manager.max_acceleration").value,
                self.get_parameter("manager.max_acceleration").value,
            )
        self.state.stamp = now
        self.have_odom = True

    def _cloud_callback(self, message: PointCloud2) -> None:
        if not self.have_odom:
            return
        points = point_cloud2.read_points_numpy(message, field_names=("x", "y", "z"))
        if points.size == 0:
            return
        cloud = np.asarray(points[:, :3], dtype=np.float64)
        sensor_origin = (
            self.state.position
            + self.rotation_world_from_body @ self.lidar_translation
        )
        if not self.pointcloud_is_world:
            cloud = (
                cloud @ self.rotation_world_from_body.T
                + sensor_origin
            )
        with self.lock:
            addresses = self.voxel_map.input_point_cloud(
                cloud, sensor_origin
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

    def _publish_pending_chunk(self) -> None:
        chunk = self.multi_map.flush_pending(self._address_occupancy)
        if chunk is not None:
            self._publish_chunk(chunk, -1)

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
        self.planner.swarm.update_state(
            int(message.drone_id), state, list(message.grid_ids)
        )

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
        box = PredictedBox(position, velocity, np.ones(3), 0.0, int(message.drone_id))
        boxes.append(box)
        self.planner.environment.predicted_boxes = boxes
        previous = self.swarm_trajectories.get(int(message.drone_id))
        start_time = message.start_time.sec + message.start_time.nanosec * 1.0e-9
        if previous is None or start_time > previous[1] + 1.0e-3:
            self.swarm_trajectories[int(message.drone_id)] = (spline, start_time)

    def _planning_state(self, now: float) -> VehicleState:
        if (
            self.current_position_trajectory is None
            or self.fsm_state not in (FsmState.EXECUTE, FsmState.PLAN)
        ):
            return VehicleState(
                position=self.state.position.copy(),
                velocity=self.state.velocity.copy(),
                acceleration=self.state.acceleration.copy(),
                yaw=self.state.yaw,
                yaw_rate=self.state.yaw_rate,
                stamp=now,
            )
        replan_time = self.get_parameter("fsm.replan_time").value
        stamp = float(
            np.clip(
                now - self.current_trajectory_start + replan_time,
                0.0,
                self.current_trajectory_duration,
            )
        )
        derivatives = self.current_position_trajectory.derivatives(2)
        yaw = self.state.yaw
        yaw_rate = self.state.yaw_rate
        if self.current_yaw_trajectory is not None:
            yaw_stamp = min(stamp, self.current_yaw_trajectory.get_time_sum())
            yaw = float(self.current_yaw_trajectory.evaluate(yaw_stamp)[0])
            yaw_derivative = self.current_yaw_trajectory.derivative()
            yaw_rate = float(
                yaw_derivative.evaluate(
                    min(yaw_stamp, yaw_derivative.get_time_sum())
                )[0]
            )
        return VehicleState(
            position=self.current_position_trajectory.evaluate(stamp),
            velocity=derivatives[0].evaluate(
                min(stamp, derivatives[0].get_time_sum())
            ),
            acceleration=derivatives[1].evaluate(
                min(stamp, derivatives[1].get_time_sum())
            ),
            yaw=yaw,
            yaw_rate=yaw_rate,
            stamp=now + replan_time,
        )

    def _safety_tick(self) -> None:
        if self.fsm_state != FsmState.EXECUTE or self.current_position_trajectory is None:
            return
        now = self.get_clock().now().nanoseconds * 1.0e-9
        elapsed = max(0.0, now - self.current_trajectory_start)
        end = min(self.current_trajectory_duration, elapsed + 1.0)
        for stamp in np.arange(elapsed, end + 1.0e-6, 0.05):
            position = self.current_position_trajectory.evaluate(float(stamp))
            if (
                self.voxel_map.get_inflated_occupancy(position) == 1
                or self.voxel_map.get_distance(position)
                <= self.get_parameter("manager.clearance_threshold").value
            ):
                self.get_logger().warning("trajectory collision predicted; replanning")
                self.fsm_state = FsmState.PLAN
                return
            absolute_time = self.current_trajectory_start + float(stamp)
            for drone_id, (trajectory, start_time) in self.swarm_trajectories.items():
                other_stamp = absolute_time - start_time
                if not 0.0 <= other_stamp <= trajectory.get_time_sum():
                    continue
                other = trajectory.evaluate(other_stamp)
                if np.linalg.norm((position - other)[:2]) < 0.5:
                    self.get_logger().warning(
                        f"trajectory collision with drone {drone_id}; replanning"
                    )
                    self.fsm_state = FsmState.PLAN
                    return

    def _fsm_tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if self.fsm_state == FsmState.INIT:
            if self.have_odom and self.have_map:
                self.fsm_state = FsmState.WAIT_TRIGGER
        elif self.fsm_state == FsmState.WAIT_TRIGGER:
            if self.triggered:
                self.fsm_state = FsmState.PLAN
        elif self.fsm_state == FsmState.EXECUTE:
            elapsed = now - self.current_trajectory_start
            if (
                self.current_trajectory_duration - elapsed
                <= self.get_parameter("fsm.replan_near_end").value
                or now - self.last_plan_time
                >= self.get_parameter("fsm.periodic_replan").value
                or self.planner.frontier.is_frontier_covered()
            ):
                self.fsm_state = FsmState.PLAN
                self.replan_publisher.publish(Empty())
        elif self.fsm_state == FsmState.IDLE:
            if now - self.last_idle_check >= self.get_parameter("fsm.idle_retry").value:
                self.last_idle_check = now
                self.fsm_state = FsmState.PLAN
        if self.fsm_state == FsmState.PLAN and not self.busy:
            self.busy = True
            try:
                with self.lock:
                    result = self.planner.plan(self._planning_state(now))
                if result.status == PlannerStatus.SUCCEED:
                    self._publish_trajectory(result)
                    self._publish_markers(result)
                    self.fsm_state = FsmState.EXECUTE
                    self.last_plan_time = now
                elif result.status in (
                    PlannerStatus.NO_FRONTIER,
                    PlannerStatus.NO_GRID,
                ):
                    self.last_idle_check = now
                    self.fsm_state = FsmState.IDLE
                else:
                    self.last_idle_check = now
                    self.fsm_state = FsmState.IDLE
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
        message.yaw_dt = float(result.yaw_knot_span)
        self.bspline_publisher.publish(message)
        self.swarm_trajectory_publisher.publish(message)
        self.new_publisher.publish(Empty())
        self.current_position_trajectory = position
        self.current_yaw_trajectory = NonUniformBspline(
            result.yaw_control_points, 3, result.yaw_knot_span
        )
        self.current_trajectory_start = start
        self.current_trajectory_duration = position.get_time_sum()

    def _publish_state(self) -> None:
        message = DroneState()
        message.drone_id = int(self.drone_id)
        message.grid_ids = [int(value) for value in self.planner.grid_ids]
        message.recent_attempt_time = float(max(self.last_plan_time, 0.0))
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
