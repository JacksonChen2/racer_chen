"""Frontier viewpoints, coverage-path guidance, A* and B-spline trajectories."""

from dataclasses import dataclass
import heapq
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .hgrid import HierarchicalGrid
from .mapping import FREE, UNKNOWN, GridIndex, OccupancyMap


WorldPoint = Tuple[float, float]


@dataclass
class ExplorationPlan:
    goal: WorldPoint
    yaw: float
    grid_path: List[GridIndex]
    geometric_path: List[WorldPoint]
    trajectory: List[Tuple[float, float, float]]
    information_gain: int


def astar(
    blocked: np.ndarray,
    start: GridIndex,
    goal: GridIndex,
) -> List[GridIndex]:
    """Eight-connected A* with diagonal corner-cut protection."""

    height, width = blocked.shape
    if not (0 <= start[0] < width and 0 <= start[1] < height):
        return []
    if not (0 <= goal[0] < width and 0 <= goal[1] < height):
        return []
    blocked = blocked.copy()
    blocked[start[1], start[0]] = False
    if blocked[goal[1], goal[0]]:
        return []
    queue: List[Tuple[float, float, GridIndex]] = []
    heapq.heappush(queue, (0.0, 0.0, start))
    came_from: Dict[GridIndex, GridIndex] = {}
    cost = {start: 0.0}
    moves = (
        (-1, -1, math.sqrt(2.0)),
        (0, -1, 1.0),
        (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)),
        (0, 1, 1.0),
        (1, 1, math.sqrt(2.0)),
    )
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current == goal:
            result = [current]
            while current in came_from:
                current = came_from[current]
                result.append(current)
            return list(reversed(result))
        if current_cost > cost.get(current, math.inf) + 1.0e-9:
            continue
        for dx, dy, step in moves:
            nx, ny = current[0] + dx, current[1] + dy
            if not (0 <= nx < width and 0 <= ny < height) or blocked[ny, nx]:
                continue
            if dx and dy:
                if blocked[current[1], nx] or blocked[ny, current[0]]:
                    continue
            neighbor = (nx, ny)
            next_cost = current_cost + step
            if next_cost + 1.0e-9 >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = next_cost
            came_from[neighbor] = current
            heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
            heapq.heappush(queue, (next_cost + heuristic, next_cost, neighbor))
    return []


def _line_free(
    first: GridIndex, second: GridIndex, blocked: np.ndarray
) -> bool:
    for x, y in OccupancyMap.bresenham(first, second):
        if blocked[y, x]:
            return False
    return True


def shorten_path(
    path: Sequence[GridIndex], blocked: np.ndarray
) -> List[GridIndex]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if _line_free(path[anchor], path[candidate], blocked):
                break
            candidate -= 1
        result.append(path[candidate])
        anchor = candidate
    return result


class UniformBSpline:
    """Clamped B-spline used to produce smooth executable samples."""

    def __init__(self, control_points: Sequence[WorldPoint], degree: int = 3):
        points = [tuple(map(float, point)) for point in control_points]
        while len(points) < degree + 1:
            points.insert(-1 if len(points) > 1 else len(points), points[-1])
        self.points = np.asarray(points, dtype=float)
        self.degree = min(degree, len(points) - 1)
        interior_count = len(points) - self.degree - 1
        self.knots = np.concatenate(
            (
                np.zeros(self.degree + 1),
                np.linspace(0.0, 1.0, interior_count + 2)[1:-1],
                np.ones(self.degree + 1),
            )
        )

    def evaluate(self, parameter: float) -> WorldPoint:
        u = float(np.clip(parameter, 0.0, 1.0))
        n = len(self.points) - 1
        if u >= 1.0:
            return tuple(self.points[-1])
        span = self.degree
        while span < n and not (
            self.knots[span] <= u < self.knots[span + 1]
        ):
            span += 1
        work = [
            self.points[span - self.degree + index].copy()
            for index in range(self.degree + 1)
        ]
        for level in range(1, self.degree + 1):
            for index in range(self.degree, level - 1, -1):
                knot_index = span - self.degree + index
                denominator = (
                    self.knots[knot_index + self.degree - level + 1]
                    - self.knots[knot_index]
                )
                alpha = 0.0 if denominator == 0.0 else (
                    (u - self.knots[knot_index]) / denominator
                )
                work[index] = (
                    (1.0 - alpha) * work[index - 1] + alpha * work[index]
                )
        return float(work[self.degree][0]), float(work[self.degree][1])


def _sample_polyline(
    points: Sequence[WorldPoint], speed: float, sample_dt: float
) -> List[Tuple[float, float, float]]:
    result = [(0.0, points[0][0], points[0][1])]
    timestamp = 0.0
    for first, second in zip(points, points[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        duration = max(sample_dt, distance / max(0.05, speed))
        count = max(1, int(math.ceil(duration / sample_dt)))
        for step in range(1, count + 1):
            ratio = step / count
            timestamp += duration / count
            result.append(
                (
                    timestamp,
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                )
            )
    return result


def minimum_time_trajectory(
    points: Sequence[WorldPoint],
    occupancy_map: OccupancyMap,
    blocked: np.ndarray,
    max_speed: float,
    max_acceleration: float,
    sample_dt: float = 0.2,
) -> List[Tuple[float, float, float]]:
    """Time-scale a cubic B-spline; fall back if it cuts a safe corridor."""

    if len(points) < 2:
        return [(0.0, points[0][0], points[0][1])] if points else []
    spline = UniformBSpline(points)
    dense = [spline.evaluate(index / 100.0) for index in range(101)]
    if any(
        (
            (cell := occupancy_map.world_to_grid(*point)) is None
            or blocked[cell[1], cell[0]]
        )
        for point in dense
    ):
        return _sample_polyline(points, max_speed, sample_dt)

    length = sum(
        math.hypot(second[0] - first[0], second[1] - first[1])
        for first, second in zip(dense, dense[1:])
    )
    duration = max(
        sample_dt,
        length / max(0.05, max_speed) + max_speed / max(0.05, max_acceleration),
    )
    count = max(2, int(math.ceil(duration / sample_dt)))
    samples = []
    for index in range(count + 1):
        timestamp = duration * index / count
        point = spline.evaluate(index / count)
        samples.append((timestamp, point[0], point[1]))
    return samples


def _cluster_candidates(
    cluster: Sequence[GridIndex],
    blocked: np.ndarray,
    maximum: int = 6,
) -> List[GridIndex]:
    centroid_x = sum(item[0] for item in cluster) / len(cluster)
    centroid_y = sum(item[1] for item in cluster) / len(cluster)
    ordered = sorted(
        cluster,
        key=lambda item: (
            math.hypot(item[0] - centroid_x, item[1] - centroid_y),
            item,
        ),
    )
    return [
        item for item in ordered if not blocked[item[1], item[0]]
    ][:maximum]


def plan_exploration(
    occupancy_map: OccupancyMap,
    hgrid: HierarchicalGrid,
    owned_cells: Sequence[str],
    coverage_route: Sequence[str],
    position: WorldPoint,
    yaw: float,
    clearance: float = 0.55,
    max_speed: float = 1.2,
    max_acceleration: float = 1.5,
) -> Optional[ExplorationPlan]:
    """Find a CP-guided frontier path and refine its local viewpoint."""

    start = occupancy_map.world_to_grid(*position)
    if start is None:
        return None
    state = occupancy_map.states()
    blocked = occupancy_map.inflated_blocked(
        clearance, unknown_is_blocked=False
    )
    blocked |= state == UNKNOWN
    blocked[start[1], start[0]] = False
    clusters = occupancy_map.frontier_clusters(minimum_size=2)
    if not clusters:
        return None

    owned = set(owned_cells)
    route_rank = {cell_id: index for index, cell_id in enumerate(coverage_route)}
    evaluated = []
    for cluster in clusters:
        world_centroid = occupancy_map.grid_to_world(
            (
                int(round(sum(cell[0] for cell in cluster) / len(cluster))),
                int(round(sum(cell[1] for cell in cluster) / len(cluster))),
            )
        )
        grid_cell = hgrid.containing(world_centroid)
        owner_match = grid_cell is not None and grid_cell.id in owned
        rank = route_rank.get(grid_cell.id, len(route_rank) + 5) if grid_cell else 999
        for candidate in _cluster_candidates(cluster, blocked):
            path = astar(blocked, start, candidate)
            if not path:
                continue
            gain = occupancy_map.information_gain(candidate)
            path_cost = sum(
                math.hypot(b[0] - a[0], b[1] - a[1])
                for a, b in zip(path, path[1:])
            )
            # Cells outside the allocated region are only a liveness fallback.
            allocation_cost = 0.0 if owner_match else 1000.0
            score = allocation_cost + 4.0 * rank + path_cost - 0.025 * gain
            evaluated.append((score, -gain, candidate, path))

    if not evaluated:
        return None
    # If no owned frontier is currently visible, remove the allocation penalty
    # and keep exploring toward the closest reachable frontier.
    if all(item[0] >= 999.0 for item in evaluated):
        evaluated = [
            (
                (item[0] - 1000.0),
                item[1],
                item[2],
                item[3],
            )
            for item in evaluated
        ]
    _, neg_gain, target, path = min(evaluated, key=lambda item: (item[0], item[1]))
    shortened = shorten_path(path, blocked)
    geometric = [occupancy_map.grid_to_world(cell) for cell in shortened]
    trajectory = minimum_time_trajectory(
        geometric,
        occupancy_map,
        blocked,
        max_speed=max_speed,
        max_acceleration=max_acceleration,
    )
    goal = occupancy_map.grid_to_world(target)
    if len(geometric) > 1:
        final_heading = math.atan2(
            geometric[-1][1] - geometric[-2][1],
            geometric[-1][0] - geometric[-2][0],
        )
    else:
        final_heading = yaw
    return ExplorationPlan(
        goal=goal,
        yaw=final_heading,
        grid_path=path,
        geometric_path=geometric,
        trajectory=trajectory,
        information_gain=-neg_gain,
    )
