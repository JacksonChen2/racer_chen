"""Python composition of RACER's hierarchical exploration and trajectory pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .bspline import NonUniformBspline
from .environment import EDTEnvironment
from .frontier import FrontierConfig, FrontierFinder
from .graph import ViewGraph
from .heading import HeadingConfig, HeadingPlanner
from .kinodynamic import KinodynamicAStar, KinodynamicConfig, KinoResult
from .optimizer import BsplineOptimizer, OptimizerConfig
from .partition import HierarchicalGrid, PartitionConfig
from .perception import PerceptionConfig
from .polynomial import PolynomialTrajectory
from .search import AStar
from .swarm import SwarmCoordinator
from .tsp import LkhSolver
from .types import PlannerResult, PlannerStatus, VehicleState, Viewpoint


@dataclass(slots=True)
class PlannerConfig:
    drone_id: int = 1
    drone_count: int = 1
    max_velocity: float = 2.0
    max_acceleration: float = 2.0
    max_yaw_rate: float = math.radians(120.0)
    max_yaw_acceleration: float = math.radians(90.0)
    direction_weight: float = 1.5
    control_point_distance: float = 0.5
    clearance_threshold: float = 0.2
    use_optimization: bool = True
    use_active_perception: bool = True
    astar_resolution: float = 0.3
    astar_max_search_time: float = 0.1
    kino_max_search_time: float = 0.25
    initial_plan_count: int = 2
    close_radius: float = 1.5
    far_radius: float = 7.0
    refined_frontier_count: int = 3
    refined_radius: float = 5.0


class RacerPlanner:
    """Reproduce RACER's grid/frontier tour and local trajectory planning flow."""

    def __init__(
        self,
        environment: EDTEnvironment,
        config: PlannerConfig | None = None,
        frontier_config: FrontierConfig | None = None,
        perception_config: PerceptionConfig | None = None,
        optimizer_config: OptimizerConfig | None = None,
        heading_config: HeadingConfig | None = None,
        partition_config: PartitionConfig | None = None,
    ) -> None:
        self.environment = environment
        self.config = config or PlannerConfig()
        self.astar = AStar(
            environment,
            resolution=self.config.astar_resolution,
            max_search_time=self.config.astar_max_search_time,
        )
        self.view_graph = ViewGraph(
            environment.voxel_map,
            self.astar,
            self.config.max_velocity,
            self.config.max_acceleration,
            self.config.max_yaw_rate,
            self.config.max_yaw_acceleration,
            self.config.direction_weight,
        )
        self.frontier = FrontierFinder(
            environment,
            self.view_graph,
            frontier_config or FrontierConfig(),
            perception_config or PerceptionConfig(),
        )
        optimizer = optimizer_config or OptimizerConfig(
            max_velocity=self.config.max_velocity,
            max_acceleration=self.config.max_acceleration,
            safe_distance=0.7,
        )
        self.optimizer = BsplineOptimizer(environment, optimizer)
        self.heading = HeadingPlanner(self.frontier, heading_config or HeadingConfig())
        self.tsp = LkhSolver()
        self.swarm = SwarmCoordinator(self.config.drone_id, self.config.drone_count)
        self.hgrid = HierarchicalGrid(
            environment, self.astar, partition_config or PartitionConfig()
        )
        self.kino = KinodynamicAStar(
            environment,
            KinodynamicConfig(
                max_velocity=self.config.max_velocity,
                max_acceleration=self.config.max_acceleration,
                collision_clearance=self.config.clearance_threshold,
                search_time_limit=self.config.kino_max_search_time,
                horizon=self.config.far_radius,
            ),
        )
        self.grid_ids: list[int] = []
        self.last_grid_ids: list[int] = []
        self.plan_count = 0

    @staticmethod
    def _resample(path: list[np.ndarray], distance: float) -> np.ndarray:
        if len(path) < 2:
            return np.asarray(path, dtype=np.float64)
        result = [np.asarray(path[0], dtype=np.float64)]
        for start, end in zip(path[:-1], path[1:]):
            start, end = np.asarray(start), np.asarray(end)
            length = float(np.linalg.norm(end - start))
            count = max(1, int(math.ceil(length / max(distance, 1.0e-3))))
            result.extend(
                start + (end - start) * ratio / count
                for ratio in range(1, count + 1)
            )
        return np.asarray(result)

    def update_frontiers(self) -> None:
        self.frontier.search_frontiers()
        self.frontier.compute_frontiers_to_visit()
        self.frontier.update_cost_matrix()

    def _ordered_grids(self, state: VehicleState) -> list[int]:
        averages = [frontier.average for frontier in self.frontier.frontiers]
        self.hgrid.input_frontiers(averages)
        self.hgrid.update(self.config.drone_id, self.grid_ids)
        candidates = self.hgrid.active_grids()
        if not candidates:
            self.last_grid_ids, self.grid_ids = self.grid_ids, []
            return []

        grid_views = [
            Viewpoint(self.hgrid.get_grid(grid_id).center, 0.0)
            for grid_id in candidates
        ]
        if self.config.drone_count > 1:
            owned = self.swarm.allocate(
                state,
                grid_views,
                self.view_graph,
                item_ids=candidates,
                consistency_bonus=abs(self.hgrid.config.consistent_cost),
            )
            candidates = [candidates[index] for index in owned]
        if not candidates:
            self.last_grid_ids, self.grid_ids = self.grid_ids, []
            return []

        count = len(candidates)
        matrix = np.zeros((count + 1, count + 1), dtype=np.float64)
        for index, grid_id in enumerate(candidates):
            center = self.hgrid.get_grid(grid_id).center
            matrix[0, index + 1], _ = self.view_graph.compute_cost(
                state.position,
                center,
                state.yaw,
                0.0,
                state.velocity,
                state.yaw_rate,
            )
        for first in range(count):
            for second in range(first + 1, count):
                cost = self.hgrid.grid_to_grid_cost(
                    candidates[first], candidates[second], self.config.drone_count
                )
                matrix[first + 1, second + 1] = cost
                matrix[second + 1, first + 1] = cost
        route = self.tsp.solve_tsp(matrix)
        ordered = [candidates[item - 1] for item in route if 0 < item <= count]
        self.last_grid_ids, self.grid_ids = self.grid_ids, ordered
        return ordered

    def _ordered_frontiers(
        self, state: VehicleState, grid_ids: list[int]
    ) -> list[int]:
        frontier_ids = self.hgrid.frontiers_in_grids(grid_ids)
        if not frontier_ids and grid_ids and self.frontier.frontiers:
            center = self.hgrid.get_grid(grid_ids[0]).center
            frontier_ids = [
                min(
                    range(len(self.frontier.frontiers)),
                    key=lambda index: np.linalg.norm(
                        self.frontier.frontiers[index].average - center
                    ),
                )
            ]
        if not frontier_ids:
            return []

        full = self.frontier.full_cost_matrix(
            state.position, state.velocity, state.yaw
        )
        indices = [0] + [frontier_id + 1 for frontier_id in frontier_ids]
        matrix = full[np.ix_(indices, indices)]
        route = self.tsp.solve_tsp(matrix)
        return [
            frontier_ids[item - 1]
            for item in route
            if 0 < item <= len(frontier_ids)
        ]

    def _refine_local_view(
        self, state: VehicleState, frontier_ids: list[int]
    ) -> Viewpoint | None:
        if not frontier_ids:
            return None
        selected_ids: list[int] = []
        for frontier_id in frontier_ids[: self.config.refined_frontier_count]:
            selected_ids.append(frontier_id)
            if (
                len(selected_ids) >= 2
                and np.linalg.norm(
                    self.frontier.frontiers[frontier_id].average - state.position
                )
                > self.config.refined_radius
            ):
                break

        layers: list[list[Viewpoint]] = []
        for frontier_id in selected_ids:
            candidates = self.frontier.frontiers[frontier_id].viewpoints
            usable = [
                viewpoint
                for viewpoint in candidates
                if np.linalg.norm(viewpoint.position - state.position)
                >= self.frontier.config.min_candidate_distance
            ]
            layers.append((usable or candidates)[: max(1, min(8, len(candidates)))])
        if not layers or not layers[0]:
            return None

        costs = np.asarray(
            [
                self.view_graph.compute_cost(
                    state.position,
                    viewpoint.position,
                    state.yaw,
                    viewpoint.yaw,
                    state.velocity,
                    state.yaw_rate,
                )[0]
                for viewpoint in layers[0]
            ],
            dtype=np.float64,
        )
        parents: list[list[int]] = []
        for previous, current in zip(layers[:-1], layers[1:]):
            next_costs = np.full(len(current), math.inf, dtype=np.float64)
            next_parents = [-1] * len(current)
            for current_id, current_view in enumerate(current):
                for previous_id, previous_view in enumerate(previous):
                    edge, _ = self.view_graph.compute_cost(
                        previous_view.position,
                        current_view.position,
                        previous_view.yaw,
                        current_view.yaw,
                        np.zeros(3),
                        0.0,
                    )
                    value = costs[previous_id] + edge
                    if value < next_costs[current_id]:
                        next_costs[current_id] = value
                        next_parents[current_id] = previous_id
            parents.append(next_parents)
            costs = next_costs

        choice = int(np.argmin(costs))
        for parent_layer in reversed(parents):
            choice = parent_layer[choice]
        return layers[0][choice]

    def _shorten_path(self, path: list[np.ndarray]) -> list[np.ndarray]:
        if len(path) <= 2:
            return [np.asarray(point).copy() for point in path]
        shortened = [np.asarray(path[0]).copy()]
        for index in range(1, len(path) - 1):
            point = np.asarray(path[index])
            if (
                np.linalg.norm(point - shortened[-1]) > 3.0
                or not self.astar._segment_safe(
                    shortened[-1], np.asarray(path[index + 1]), False
                )
            ):
                shortened.append(point.copy())
        if np.linalg.norm(np.asarray(path[-1]) - shortened[-1]) > 1.0e-6:
            shortened.append(np.asarray(path[-1]).copy())
        return shortened

    def _truncate_path(
        self, path: list[np.ndarray], maximum_length: float
    ) -> list[np.ndarray]:
        result = [np.asarray(path[0]).copy()]
        length = 0.0
        for point in path[1:]:
            point = np.asarray(point)
            segment = float(np.linalg.norm(point - result[-1]))
            if length + segment >= maximum_length:
                ratio = (maximum_length - length) / max(segment, 1.0e-9)
                result.append(result[-1] + ratio * (point - result[-1]))
                break
            result.append(point.copy())
            length += segment
        return result

    @staticmethod
    def _sample_kino(result: KinoResult, step: float = 0.1) -> np.ndarray:
        if not result.states:
            return np.empty((0, 3), dtype=np.float64)
        points = [result.states[0][:3].copy()]
        for state, control, duration in zip(
            result.states, result.inputs, result.durations
        ):
            for stamp in np.arange(step, duration + 1.0e-6, step):
                points.append(
                    KinodynamicAStar.transition(
                        state, control, min(float(stamp), duration)
                    )[:3]
                )
            endpoint = KinodynamicAStar.transition(state, control, duration)[:3]
            if np.linalg.norm(endpoint - points[-1]) > 1.0e-6:
                points.append(endpoint)
        if result.shot_coefficients is not None:
            for stamp in np.arange(step, result.shot_time + 1.0e-6, step):
                time = min(float(stamp), result.shot_time)
                points.append(
                    sum(
                        result.shot_coefficients[:, power] * time**power
                        for power in range(4)
                    )
                )
        return np.asarray(points)

    def _sample_waypoint_trajectory(
        self, path: list[np.ndarray], state: VehicleState, interval: float
    ) -> np.ndarray:
        points = np.asarray(path, dtype=np.float64)
        if len(points) < 2:
            return points
        durations = np.asarray(
            [
                max(
                    float(np.linalg.norm(second - first))
                    / max(self.config.max_velocity * 0.5, 1.0e-3),
                    interval,
                )
                for first, second in zip(points[:-1], points[1:])
            ]
        )
        try:
            polynomial = PolynomialTrajectory.through_waypoints(
                points,
                state.velocity,
                np.zeros(3),
                state.acceleration,
                np.zeros(3),
                durations,
            )
            sample_times = np.arange(
                0.0, polynomial.get_total_time() + 1.0e-6, interval
            )
            samples = np.asarray(
                [polynomial.evaluate(float(stamp)) for stamp in sample_times]
            )
            if np.linalg.norm(samples[-1] - points[-1]) > 1.0e-6:
                samples = np.vstack((samples, points[-1]))
            return samples
        except (ValueError, np.linalg.LinAlgError):
            return self._resample(path, self.config.control_point_distance)

    def _build_trajectory(
        self,
        samples: np.ndarray,
        state: VehicleState,
        goal: np.ndarray,
        goal_yaw: float,
        path: list[np.ndarray],
        yaw_time_lower_bound: float,
    ) -> PlannerResult:
        if len(samples) < 2:
            return PlannerResult(PlannerStatus.NO_PATH, goal=goal, goal_yaw=goal_yaw)
        while len(samples) < 4:
            samples = np.insert(
                samples, -1, 0.5 * (samples[-2] + samples[-1]), axis=0
            )
        interval = max(
            self.config.control_point_distance
            / max(self.config.max_velocity, 1.0e-3),
            0.1,
        )
        derivatives = np.vstack(
            (state.velocity, np.zeros(3), state.acceleration, np.zeros(3))
        )
        control = NonUniformBspline.parameterize_to_bspline(
            interval, samples, derivatives, degree=3
        )
        if self.config.use_optimization:
            control = self.optimizer.optimize(control, interval)
        position = NonUniformBspline(control, 3, interval)
        position.set_physical_limits(
            self.config.max_velocity, self.config.max_acceleration
        )
        for _ in range(5):
            if position.check_feasibility():
                break
            position.reallocate_time()
        duration = position.get_time_sum()
        if duration < yaw_time_lower_bound and duration > 1.0e-6:
            position.lengthen_time(yaw_time_lower_bound / duration)
            duration = position.get_time_sum()

        sample_times = np.linspace(
            0.0, duration, max(4, int(math.ceil(duration / interval)) + 1)
        )
        positions = np.asarray(
            [position.evaluate(float(stamp)) for stamp in sample_times]
        )
        yaws = (
            self.heading.plan(
                positions,
                sample_times,
                state.yaw,
                goal_yaw,
                state.position,
            )
            if self.config.use_active_perception
            else np.linspace(state.yaw, goal_yaw, len(positions))
        )
        yaw_interval = max(
            duration / max(len(yaws) - 1, 1),
            0.05,
        )
        yaw_points = np.column_stack(
            (yaws, np.zeros_like(yaws), np.zeros_like(yaws))
        )
        yaw_control = NonUniformBspline.parameterize_to_bspline(
            yaw_interval, yaw_points, np.zeros((4, 3)), degree=3
        )[:, 0:1]
        return PlannerResult(
            PlannerStatus.SUCCEED,
            position_control_points=position.get_control_point(),
            yaw_control_points=yaw_control,
            knot_span=position.knot_span,
            yaw_knot_span=yaw_interval,
            position_knots=position.get_knot(),
            duration=duration,
            path=[np.asarray(point).copy() for point in path],
            goal=goal.copy(),
            goal_yaw=float(goal_yaw),
        )

    def plan(self, state: VehicleState) -> PlannerResult:
        self.update_frontiers()
        if not self.frontier.frontiers:
            return PlannerResult(PlannerStatus.NO_FRONTIER)

        grid_ids = self._ordered_grids(state)
        if not grid_ids:
            return PlannerResult(PlannerStatus.NO_GRID)
        frontier_ids = self._ordered_frontiers(state, grid_ids)
        selected = self._refine_local_view(state, frontier_ids)
        if selected is None:
            return PlannerResult(PlannerStatus.NO_FRONTIER)
        goal, goal_yaw = selected.position.copy(), float(selected.yaw)

        optimistic = self.plan_count < self.config.initial_plan_count
        self.astar.reset()
        self.astar.set_resolution(self.config.astar_resolution)
        if (
            self.astar.search(state.position, goal, optimistic=optimistic)
            != AStar.REACH_END
        ):
            return PlannerResult(
                PlannerStatus.NO_PATH, goal=goal, goal_yaw=goal_yaw
            )
        path = self._shorten_path(self.astar.path)
        length = AStar.path_length(path)
        yaw_difference = abs(goal_yaw - state.yaw) % (2.0 * math.pi)
        yaw_difference = min(yaw_difference, 2.0 * math.pi - yaw_difference)
        yaw_time_lower_bound = yaw_difference / max(
            self.config.max_yaw_rate, 1.0e-3
        )
        interval = max(
            self.config.control_point_distance
            / max(self.config.max_velocity, 1.0e-3),
            0.1,
        )

        if length < self.config.close_radius or optimistic:
            samples = self._sample_waypoint_trajectory(path, state, interval)
        elif length > self.config.far_radius:
            path = self._truncate_path(path, self.config.far_radius)
            goal = path[-1].copy()
            samples = self._sample_waypoint_trajectory(path, state, interval)
        else:
            kino = self.kino.search(
                state.position,
                state.velocity,
                state.acceleration,
                goal,
                np.zeros(3),
            )
            if kino.status == KinodynamicAStar.NO_PATH:
                return PlannerResult(
                    PlannerStatus.NO_PATH, goal=goal, goal_yaw=goal_yaw
                )
            samples = self._sample_kino(kino, min(interval, 0.1))
            path = [point.copy() for point in samples]

        result = self._build_trajectory(
            samples,
            state,
            goal,
            goal_yaw,
            path,
            yaw_time_lower_bound,
        )
        if result.status == PlannerStatus.SUCCEED:
            selected_id = min(
                range(len(self.frontier.frontiers)),
                key=lambda index: np.linalg.norm(
                    self.frontier.frontiers[index].viewpoints[0].position
                    - selected.position
                ),
            )
            self.frontier.set_next_frontier(selected_id)
            self.plan_count += 1
        return result
