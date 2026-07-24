"""Python-first composition of RACER's exploration and local planning pipeline."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .bspline import NonUniformBspline
from .environment import EDTEnvironment
from .frontier import FrontierConfig, FrontierFinder
from .graph import ViewGraph
from .heading import HeadingConfig, HeadingPlanner
from .optimizer import BsplineOptimizer, OptimizerConfig
from .perception import PerceptionConfig
from .search import AStar
from .tsp import LkhSolver
from .swarm import SwarmCoordinator
from .types import PlannerResult, PlannerStatus, VehicleState


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


class RacerPlanner:
    """The non-ROS state machine invokes this class for every replan cycle."""

    def __init__(
        self,
        environment: EDTEnvironment,
        config: PlannerConfig | None = None,
        frontier_config: FrontierConfig | None = None,
        perception_config: PerceptionConfig | None = None,
        optimizer_config: OptimizerConfig | None = None,
        heading_config: HeadingConfig | None = None,
    ) -> None:
        self.environment = environment
        self.config = config or PlannerConfig()
        self.astar = AStar(environment)
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
        opt = optimizer_config or OptimizerConfig(
            max_velocity=self.config.max_velocity,
            max_acceleration=self.config.max_acceleration,
            safe_distance=0.7,
        )
        self.optimizer = BsplineOptimizer(environment, opt)
        self.heading = HeadingPlanner(self.frontier, heading_config or HeadingConfig())
        self.tsp = LkhSolver()
        self.swarm = SwarmCoordinator(self.config.drone_id, self.config.drone_count)

    @staticmethod
    def _resample(path: list[np.ndarray], distance: float) -> np.ndarray:
        if len(path) < 2:
            return np.asarray(path, dtype=np.float64)
        result = [np.asarray(path[0], dtype=np.float64)]
        for start, end in zip(path[:-1], path[1:]):
            start, end = np.asarray(start), np.asarray(end)
            length = float(np.linalg.norm(end - start))
            count = max(1, int(math.ceil(length / distance)))
            result.extend(start + (end - start) * ratio / count for ratio in range(1, count + 1))
        return np.asarray(result)

    def update_frontiers(self) -> None:
        self.frontier.search_frontiers()
        self.frontier.compute_frontiers_to_visit()
        self.frontier.update_cost_matrix()

    def _select_goal(self, state: VehicleState) -> tuple[np.ndarray, float] | None:
        positions, yaws, _ = self.frontier.top_viewpoints(state.position)
        if not positions:
            return None
        allocated = self.swarm.allocate(
            state,
            [frontier.viewpoints[0] for frontier in self.frontier.frontiers],
            self.view_graph,
        )
        if self.config.drone_count > 1 and allocated:
            positions = [positions[index] for index in allocated]
            yaws = [yaws[index] for index in allocated]
        matrix = self.frontier.full_cost_matrix(state.position, state.velocity, state.yaw)
        if self.config.drone_count > 1 and allocated:
            indices = [0] + [index + 1 for index in allocated]
            matrix = matrix[np.ix_(indices, indices)]
        route = self.tsp.solve_tsp(matrix)
        candidate_index = next((item - 1 for item in route if item > 0), 0)
        return positions[candidate_index], yaws[candidate_index]

    def plan(self, state: VehicleState) -> PlannerResult:
        self.update_frontiers()
        selected = self._select_goal(state)
        if selected is None:
            return PlannerResult(PlannerStatus.NO_FRONTIER)
        goal, goal_yaw = selected
        self.astar.reset()
        if self.astar.search(state.position, goal, optimistic=True) != AStar.REACH_END:
            return PlannerResult(PlannerStatus.NO_PATH, goal=goal, goal_yaw=goal_yaw)
        path = self._resample(self.astar.path, self.config.control_point_distance)
        if len(path) == 1:
            path = np.vstack((path, path))
        while len(path) < 4:
            path = np.insert(path, -1, 0.5 * (path[-2] + path[-1]), axis=0)
        interval = max(
            self.config.control_point_distance / max(self.config.max_velocity, 1.0e-3),
            0.1,
        )
        derivatives = np.vstack((state.velocity, np.zeros(3), state.acceleration, np.zeros(3)))
        control = NonUniformBspline.parameterize_to_bspline(interval, path, derivatives, degree=3)
        if self.config.use_optimization:
            control = self.optimizer.optimize(control, interval)
        position = NonUniformBspline(control, 3, interval)
        position.set_physical_limits(self.config.max_velocity, self.config.max_acceleration)
        for _ in range(3):
            if position.check_feasibility():
                break
            position.reallocate_time()
        duration = position.get_time_sum()
        sample_times = np.linspace(0.0, duration, max(4, int(duration / interval) + 1))
        samples = np.asarray([position.evaluate(stamp) for stamp in sample_times])
        yaws = (
            self.heading.plan(samples, sample_times, state.yaw, goal_yaw, state.position)
            if self.config.use_active_perception
            else np.linspace(state.yaw, goal_yaw, len(samples))
        )
        yaw_points = np.column_stack((yaws, np.zeros_like(yaws), np.zeros_like(yaws)))
        yaw_control = NonUniformBspline.parameterize_to_bspline(
            max(duration / max(len(yaws) - 1, 1), 0.05),
            yaw_points,
            np.zeros((4, 3)),
            degree=3,
        )[:, 0:1]
        self.frontier.set_next_frontier(
            min(
                range(len(self.frontier.frontiers)),
                key=lambda index: np.linalg.norm(
                    self.frontier.frontiers[index].viewpoints[0].position - goal
                ),
            )
        )
        return PlannerResult(
            PlannerStatus.SUCCEED,
            position_control_points=position.get_control_point(),
            yaw_control_points=yaw_control,
            knot_span=position.knot_span,
            position_knots=position.get_knot(),
            path=[point.copy() for point in path],
            goal=goal.copy(),
            goal_yaw=float(goal_yaw),
        )
