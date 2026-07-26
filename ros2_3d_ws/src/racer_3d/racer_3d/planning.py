"""3-D frontier viewpoints, A*, and ESDF-constrained B-spline trajectories."""

from dataclasses import dataclass
import heapq
import itertools
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .hgrid import HierarchicalGrid3D
from .voxel_map import FREE, GridIndex3, VoxelMap


Point3 = Tuple[float, float, float]
TrajectorySample = Tuple[float, float, float, float]


@dataclass
class ExplorationPlan3D:
    goal: Point3
    yaw: float
    pitch: float
    frontier_centroid: Point3
    grid_path: List[GridIndex3]
    geometric_path: List[Point3]
    trajectory: List[TrajectorySample]
    information_gain: int
    minimum_clearance: float


MOVES_26 = tuple(
    (
        dx,
        dy,
        dz,
        math.sqrt(dx * dx + dy * dy + dz * dz),
    )
    for dz in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dx in (-1, 0, 1)
    if dx or dy or dz
)


def _diagonal_is_safe(
    current: GridIndex3,
    delta: Tuple[int, int, int],
    blocked: np.ndarray,
) -> bool:
    """Reject edge/corner cutting through occupied voxels."""

    changed = [axis for axis, value in enumerate(delta) if value]
    if len(changed) <= 1:
        return True
    # Every proper non-empty subset of a diagonal step must remain free.
    for count in range(1, len(changed)):
        for axes in itertools.combinations(changed, count):
            offset = [0, 0, 0]
            for axis in axes:
                offset[axis] = delta[axis]
            cell = tuple(current[index] + offset[index] for index in range(3))
            if blocked[cell[2], cell[1], cell[0]]:
                return False
    return True


def astar3d(
    blocked: np.ndarray,
    start: GridIndex3,
    goal: GridIndex3,
    resolution: float = 1.0,
) -> List[GridIndex3]:
    """26-connected Euclidean A* with strict three-dimensional corner checks."""

    nz, ny, nx = blocked.shape

    def valid(cell: GridIndex3) -> bool:
        return (
            0 <= cell[0] < nx
            and 0 <= cell[1] < ny
            and 0 <= cell[2] < nz
        )

    if not valid(start) or not valid(goal):
        return []
    effective = blocked.copy()
    effective[start[2], start[1], start[0]] = False
    if effective[goal[2], goal[1], goal[0]]:
        return []
    queue: List[Tuple[float, float, GridIndex3]] = [(0.0, 0.0, start)]
    came_from: Dict[GridIndex3, GridIndex3] = {}
    cost = {start: 0.0}
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
        for dx, dy, dz, step in MOVES_26:
            neighbor = current[0] + dx, current[1] + dy, current[2] + dz
            if (
                not valid(neighbor)
                or effective[neighbor[2], neighbor[1], neighbor[0]]
                or not _diagonal_is_safe(current, (dx, dy, dz), effective)
            ):
                continue
            candidate = current_cost + step * resolution
            if candidate + 1.0e-9 >= cost.get(neighbor, math.inf):
                continue
            cost[neighbor] = candidate
            came_from[neighbor] = current
            heuristic = resolution * math.sqrt(
                (goal[0] - neighbor[0]) ** 2
                + (goal[1] - neighbor[1]) ** 2
                + (goal[2] - neighbor[2]) ** 2
            )
            heapq.heappush(
                queue, (candidate + heuristic, candidate, neighbor)
            )
    return []


def _line_free(
    voxel_map: VoxelMap,
    first: GridIndex3,
    second: GridIndex3,
    blocked: np.ndarray,
) -> bool:
    for cell in voxel_map.ray_cells(
        voxel_map.grid_to_world(first), voxel_map.grid_to_world(second)
    ):
        if blocked[cell[2], cell[1], cell[0]]:
            return False
    return True


def shorten_path3d(
    voxel_map: VoxelMap,
    path: Sequence[GridIndex3],
    blocked: np.ndarray,
) -> List[GridIndex3]:
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            if _line_free(voxel_map, path[anchor], path[candidate], blocked):
                break
            candidate -= 1
        result.append(path[candidate])
        anchor = candidate
    return result


class UniformBSpline3D:
    """Clamped cubic B-spline evaluated with de Boor's algorithm."""

    def __init__(self, control_points: Sequence[Sequence[float]], degree: int = 3):
        values = [np.asarray(point, dtype=float) for point in control_points]
        if not values:
            raise ValueError("B-spline requires at least one control point")
        while len(values) < degree + 1:
            values.insert(-1 if len(values) > 1 else len(values), values[-1].copy())
        self.points = np.asarray(values)
        self.degree = min(degree, len(values) - 1)
        interior = len(values) - self.degree - 1
        self.knots = np.concatenate(
            (
                np.zeros(self.degree + 1),
                np.linspace(0.0, 1.0, interior + 2)[1:-1],
                np.ones(self.degree + 1),
            )
        )

    def evaluate(self, parameter: float) -> Point3:
        u = float(np.clip(parameter, 0.0, 1.0))
        final = len(self.points) - 1
        if u >= 1.0:
            return tuple(float(value) for value in self.points[-1])
        span = self.degree
        while span < final and not (
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
                alpha = (
                    0.0
                    if denominator == 0.0
                    else (u - self.knots[knot_index]) / denominator
                )
                work[index] = (
                    (1.0 - alpha) * work[index - 1] + alpha * work[index]
                )
        return tuple(float(value) for value in work[self.degree])


def optimize_bspline_control_points(
    points: Sequence[Sequence[float]],
    voxel_map: VoxelMap,
    clearance: float,
    iterations: int = 35,
) -> np.ndarray:
    """Optimize smoothness and ESDF clearance of the three-dimensional spline."""

    control = np.asarray(points, dtype=float).copy()
    if len(control) < 3:
        return control
    for iteration in range(iterations):
        updated = control.copy()
        rate = 0.20 * (1.0 - 0.6 * iteration / max(1, iterations - 1))
        for index in range(1, len(control) - 1):
            smooth_gradient = (
                control[index - 1] - 2.0 * control[index] + control[index + 1]
            )
            distance = voxel_map.distance_at(
                control[index], unknown_is_occupied=False
            )
            obstacle_gradient = np.zeros(3)
            if distance < 1.35 * clearance:
                direction = voxel_map.esdf_gradient(
                    control[index], unknown_is_occupied=False
                )
                norm = float(np.linalg.norm(direction))
                if norm > 1.0e-6:
                    obstacle_gradient = (
                        direction / norm * (1.35 * clearance - distance)
                    )
            updated[index] += rate * (
                0.30 * smooth_gradient + 1.8 * obstacle_gradient
            )
        control = updated
    return control


def _sample_polyline(
    points: Sequence[Sequence[float]],
    max_speed: float,
    sample_dt: float,
) -> List[TrajectorySample]:
    result = [(0.0, *tuple(float(value) for value in points[0]))]
    timestamp = 0.0
    for first_value, second_value in zip(points, points[1:]):
        first, second = np.asarray(first_value), np.asarray(second_value)
        distance = float(np.linalg.norm(second - first))
        duration = max(sample_dt, distance / max(0.05, max_speed))
        count = max(1, int(math.ceil(duration / sample_dt)))
        for index in range(1, count + 1):
            ratio = index / count
            timestamp += duration / count
            point = first + ratio * (second - first)
            result.append((timestamp, *tuple(float(value) for value in point)))
    return result


def minimum_time_bspline_trajectory(
    points: Sequence[Sequence[float]],
    voxel_map: VoxelMap,
    clearance: float,
    max_speed: float,
    max_acceleration: float,
    sample_dt: float = 0.15,
) -> Tuple[List[TrajectorySample], float]:
    """Optimize, collision-check, and dynamically time-scale a cubic spline."""

    if not points:
        return [], -math.inf
    if len(points) == 1:
        return [(0.0, *tuple(float(value) for value in points[0]))], math.inf
    optimized = optimize_bspline_control_points(points, voxel_map, clearance)
    spline = UniformBSpline3D(optimized)
    dense = np.asarray([spline.evaluate(index / 200.0) for index in range(201)])
    clearances = np.asarray(
        [
            voxel_map.distance_at(point, unknown_is_occupied=False)
            for point in dense
        ]
    )
    minimum_clearance = float(np.min(clearances))
    state = voxel_map.states()
    spline_known = all(
        (
            (cell := voxel_map.world_to_grid(point)) is not None
            and state[cell[2], cell[1], cell[0]] == FREE
        )
        for point in dense
    )
    if minimum_clearance < clearance or not spline_known:
        trajectory = _sample_polyline(points, max_speed, sample_dt)
        polyline_clearance = min(
            voxel_map.distance_at(sample[1:], unknown_is_occupied=False)
            for sample in trajectory
        )
        return trajectory, float(polyline_clearance)
    differences = np.diff(dense, axis=0)
    length = float(np.sum(np.linalg.norm(differences, axis=1)))
    # Trapezoidal/triangular minimum-time time scaling.
    acceleration_time = max_speed / max(0.05, max_acceleration)
    acceleration_distance = max_acceleration * acceleration_time**2
    if length <= acceleration_distance:
        duration = 2.0 * math.sqrt(length / max(0.05, max_acceleration))
    else:
        duration = acceleration_time + length / max_speed
    duration = max(sample_dt, duration)
    count = max(2, int(math.ceil(duration / sample_dt)))
    trajectory = [
        (duration * index / count, *spline.evaluate(index / count))
        for index in range(count + 1)
    ]
    return trajectory, minimum_clearance


def _viewpoint_candidates(
    voxel_map: VoxelMap,
    cluster: Sequence[GridIndex3],
    blocked: np.ndarray,
    clearance: float,
) -> List[Point3]:
    centroid = np.mean(
        np.asarray([voxel_map.grid_to_world(cell) for cell in cluster]), axis=0
    )
    candidates = []
    elevation_values = np.radians((-35.0, 0.0, 35.0))
    for radius in (0.9, 1.45):
        for elevation in elevation_values:
            horizontal = radius * math.cos(float(elevation))
            for azimuth in np.linspace(-math.pi, math.pi, 8, endpoint=False):
                point = centroid + np.asarray(
                    (
                        horizontal * math.cos(float(azimuth)),
                        horizontal * math.sin(float(azimuth)),
                        radius * math.sin(float(elevation)),
                    )
                )
                cell = voxel_map.world_to_grid(point)
                if (
                    cell is not None
                    and not blocked[cell[2], cell[1], cell[0]]
                    and voxel_map.distance_at(point, False) >= clearance
                ):
                    candidates.append(tuple(float(value) for value in point))
    return candidates


def plan_exploration(
    voxel_map: VoxelMap,
    hgrid: HierarchicalGrid3D,
    owned_cells: Sequence[str],
    coverage_route: Sequence[str],
    position: Sequence[float],
    clearance: float,
    max_speed: float,
    max_acceleration: float,
) -> Optional[ExplorationPlan3D]:
    """Choose a 3-D frontier viewpoint and optimize a safe trajectory to it."""

    start = voxel_map.world_to_grid(position)
    if start is None:
        return None
    # Centre-sampled ESDF needs a half-voxel body-diagonal allowance so the
    # continuous segment between two safe voxel centres is also safe.
    search_clearance = (
        clearance + 0.5 * math.sqrt(3.0) * voxel_map.resolution
    )
    state = voxel_map.states()
    blocked = voxel_map.inflated_blocked(
        search_clearance, unknown_is_blocked=False
    )
    # Unknown itself remains non-traversable, but unlike an occupied surface
    # it is not inflated. This is RACER's known-free frontier semantics.
    blocked |= state != FREE
    blocked[start[2], start[1], start[0]] = False
    clusters = voxel_map.frontier_clusters()
    if not clusters:
        return None
    route_rank = {cell_id: index for index, cell_id in enumerate(coverage_route)}
    owned = set(owned_cells)
    # A single connected frontier can span most of a large room.  Treating it
    # as one auction item makes every vehicle chase the same centroid and
    # reduces HGrid ownership to a weak score bonus.  Split each connected
    # frontier at active HGrid boundaries so that region allocation remains
    # effective in long three-dimensional environments.
    segments = []
    for cluster in clusters:
        grouped: Dict[Optional[str], List[GridIndex3]] = {}
        for cell in cluster:
            hcell = hgrid.containing(voxel_map.grid_to_world(cell))
            grouped.setdefault(hcell.id if hcell is not None else None, []).append(
                cell
            )
        retained = [values for values in grouped.values() if len(values) >= 4]
        segments.extend(retained if retained else [cluster])
    owned_segments = []
    for segment in segments:
        centroid = np.mean(
            [voxel_map.grid_to_world(cell) for cell in segment], axis=0
        )
        hcell = hgrid.containing(centroid)
        if hcell is not None and hcell.id in owned:
            owned_segments.append(segment)
    candidate_segments = owned_segments if owned_segments else segments
    best = None
    for cluster in sorted(candidate_segments, key=len, reverse=True)[:12]:
        centroid = tuple(
            float(value)
            for value in np.mean(
                [voxel_map.grid_to_world(cell) for cell in cluster], axis=0
            )
        )
        hcell = hgrid.containing(centroid)
        owner_bonus = (
            12.0 - route_rank.get(hcell.id, 8)
            if hcell is not None and hcell.id in owned
            else 0.0
        )
        candidates = _viewpoint_candidates(
            voxel_map, cluster, blocked, search_clearance
        )
        ranked_candidates = sorted(
            candidates,
            key=lambda point: (
                voxel_map.visible_unknown_gain(point, cluster)
                - 0.8 * float(
                    np.linalg.norm(np.asarray(point) - np.asarray(position))
                )
            ),
            reverse=True,
        )[:2]
        for viewpoint in ranked_candidates:
            goal = voxel_map.world_to_grid(viewpoint)
            if goal is None:
                continue
            path = astar3d(blocked, start, goal, voxel_map.resolution)
            if not path:
                continue
            path_distance = voxel_map.resolution * sum(
                math.sqrt(
                    (second[0] - first[0]) ** 2
                    + (second[1] - first[1]) ** 2
                    + (second[2] - first[2]) ** 2
                )
                for first, second in zip(path, path[1:])
            )
            gain = voxel_map.visible_unknown_gain(viewpoint, cluster)
            score = gain + 0.12 * len(cluster) + owner_bonus - 1.4 * path_distance
            candidate = (score, gain, path, viewpoint, centroid)
            if best is None or candidate[0] > best[0]:
                best = candidate
    if best is None:
        return None
    _, gain, path, viewpoint, centroid = best
    shortened = shorten_path3d(voxel_map, path, blocked)
    points = [tuple(float(value) for value in position)] + [
        voxel_map.grid_to_world(cell) for cell in shortened[1:-1]
    ] + [viewpoint]
    trajectory, minimum_clearance = minimum_time_bspline_trajectory(
        points,
        voxel_map,
        clearance,
        max_speed,
        max_acceleration,
    )
    direction = np.asarray(centroid) - np.asarray(viewpoint)
    yaw = math.atan2(float(direction[1]), float(direction[0]))
    pitch = math.atan2(
        float(direction[2]),
        max(1.0e-6, math.hypot(float(direction[0]), float(direction[1]))),
    )
    return ExplorationPlan3D(
        goal=viewpoint,
        yaw=yaw,
        pitch=pitch,
        frontier_centroid=centroid,
        grid_path=path,
        geometric_path=points,
        trajectory=trajectory,
        information_gain=gain,
        minimum_clearance=minimum_clearance,
    )
