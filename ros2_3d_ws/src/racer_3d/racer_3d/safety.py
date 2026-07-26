"""Three-dimensional ESDF and inter-UAV collision avoidance."""

import math
from typing import Iterable, Sequence, Tuple

import numpy as np

from .voxel_map import VoxelMap


Vector3 = Tuple[float, float, float]


def limit_norm(vector: Sequence[float], maximum: float) -> Vector3:
    value = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(value))
    if norm > maximum > 0.0:
        value *= maximum / norm
    return tuple(float(item) for item in value)


def cbf_swarm_filter(
    preferred: Sequence[float],
    position: Sequence[float],
    peers: Iterable[
        Tuple[int, Sequence[float], Sequence[float]]
    ],
    safe_distance: float,
    speed_limit: float,
    alpha: float = 1.8,
    iterations: int = 3,
) -> Vector3:
    """Project a velocity onto pairwise 3-D control-barrier constraints."""

    result = np.asarray(limit_norm(preferred, speed_limit), dtype=float)
    own = np.asarray(position, dtype=float)
    constraints = list(peers)
    for _ in range(iterations):
        for _, peer_position, peer_velocity in constraints:
            relative = own - np.asarray(peer_position, dtype=float)
            distance = float(np.linalg.norm(relative))
            if distance < 1.0e-6:
                relative = np.asarray((1.0, 0.0, 0.0))
                distance = 1.0e-6
            normal = relative / distance
            peer = np.asarray(peer_velocity, dtype=float)
            # h = ||p_i-p_j||^2-d_safe^2. Enforce h_dot >= -alpha h.
            h = distance * distance - safe_distance * safe_distance
            lower_bound = float(np.dot(normal, peer)) - alpha * h / (
                2.0 * distance
            )
            violation = lower_bound - float(np.dot(normal, result))
            if violation > 0.0:
                result += violation * normal
        result = np.asarray(limit_norm(result, speed_limit))
    return tuple(float(value) for value in result)


def esdf_obstacle_filter(
    preferred: Sequence[float],
    position: Sequence[float],
    voxel_map: VoxelMap,
    clearance: float,
    speed_limit: float,
    current_velocity: Sequence[float] = (0.0, 0.0, 0.0),
    alpha: float = 0.8,
    guaranteed_deceleration: float = 1.2,
    response_time: float = 0.12,
) -> Vector3:
    """Enforce an execution clearance using an ESDF velocity barrier.

    The relatively low barrier gain deliberately begins braking before the
    geometric limit.  A rigid-body Crazyflie cannot reverse velocity
    instantaneously, so a high-gain first-order barrier can be mathematically
    valid for a point mass while still allowing the real vehicle to overshoot
    into a wall.
    """

    result = np.asarray(limit_norm(preferred, speed_limit), dtype=float)
    distance = voxel_map.distance_at(position, unknown_is_occupied=False)
    if not math.isfinite(distance):
        return (0.0, 0.0, 0.0)
    gradient = voxel_map.esdf_gradient(position, unknown_is_occupied=False)
    norm = float(np.linalg.norm(gradient))
    if norm < 1.0e-8:
        return tuple(float(value) for value in result)
    normal = gradient / norm
    normal_speed = float(
        np.dot(normal, np.asarray(current_velocity, dtype=float))
    )
    approach_speed = max(0.0, -normal_speed)
    stopping_allowance = (
        approach_speed * approach_speed
        / (2.0 * max(0.1, guaranteed_deceleration))
        + response_time * approach_speed
    )
    dynamic_clearance = clearance + stopping_allowance
    lower_bound = -alpha * (distance - dynamic_clearance)
    violation = lower_bound - float(np.dot(normal, result))
    if violation > 0.0:
        result += violation * normal
    return limit_norm(result, speed_limit)


def aabb_obstacle_filter(
    preferred: Sequence[float],
    position: Sequence[float],
    obstacles: Iterable[object],
    clearance: float,
    speed_limit: float,
    current_velocity: Sequence[float] = (0.0, 0.0, 0.0),
    alpha: float = 1.2,
    guaranteed_deceleration: float = 0.7,
    response_time: float = 0.08,
    activation_distance: float = 1.0,
    iterations: int = 4,
) -> Vector3:
    """High-rate rigid-body velocity barrier for axis-aligned collision boxes.

    RACER's local trajectory planner runs at a much lower rate than a flight
    controller.  This projection is intended for that flight-controller layer:
    it adds stopping distance to the configured body clearance and enforces
    all nearby collision-shape half-space constraints every physics step.
    ``obstacles`` need only expose three-element ``minimum`` and ``maximum``
    properties, so this routine is independent of the acceptance scenario.
    """

    own = np.asarray(position, dtype=float)
    velocity = np.asarray(current_velocity, dtype=float)
    constraints = []
    for obstacle in obstacles:
        minimum = np.asarray(obstacle.minimum, dtype=float)
        maximum = np.asarray(obstacle.maximum, dtype=float)
        closest = np.clip(own, minimum, maximum)
        delta = own - closest
        distance = float(np.linalg.norm(delta))
        if distance > 1.0e-9:
            normal = delta / distance
            signed_distance = distance
        else:
            # Inside a box, choose the closest face and point out of it.
            face_distances = np.concatenate((own - minimum, maximum - own))
            face = int(np.argmin(face_distances))
            normal = np.zeros(3)
            normal[face % 3] = -1.0 if face < 3 else 1.0
            signed_distance = -float(face_distances[face])
        if signed_distance > activation_distance:
            continue
        approach_speed = max(0.0, -float(np.dot(normal, velocity)))
        stopping_allowance = (
            approach_speed * approach_speed
            / (2.0 * max(0.1, guaranteed_deceleration))
            + response_time * approach_speed
        )
        dynamic_clearance = clearance + stopping_allowance
        lower_bound = -alpha * (signed_distance - dynamic_clearance)
        constraints.append((signed_distance, normal, lower_bound))

    result = np.asarray(limit_norm(preferred, speed_limit), dtype=float)
    constraints.sort(key=lambda item: item[0])
    for _ in range(iterations):
        for _, normal, lower_bound in constraints:
            violation = lower_bound - float(np.dot(normal, result))
            if violation > 0.0:
                result += violation * normal
        result = np.asarray(limit_norm(result, speed_limit), dtype=float)
    return tuple(float(value) for value in result)


def emergency_separation(
    velocity: Sequence[float],
    position: Sequence[float],
    peer_positions: Iterable[Sequence[float]],
    emergency_distance: float,
    speed_limit: float,
) -> Vector3:
    result = np.asarray(velocity, dtype=float)
    own = np.asarray(position, dtype=float)
    for peer in peer_positions:
        delta = own - np.asarray(peer, dtype=float)
        distance = float(np.linalg.norm(delta))
        if distance < emergency_distance:
            if distance < 1.0e-6:
                delta, distance = np.asarray((1.0, 0.0, 0.0)), 1.0
            strength = speed_limit * (emergency_distance - distance) / max(
                emergency_distance, 1.0e-6
            )
            result += strength * delta / distance
    return limit_norm(result, speed_limit)


def predicted_path_conflict(
    first: Sequence[Sequence[float]],
    second: Sequence[Sequence[float]],
    safe_distance: float,
    time_tolerance: float = 0.35,
) -> bool:
    """Check time-stamped ``(t,x,y,z)`` trajectories for 3-D conflict."""

    for own in first:
        if len(own) < 4:
            continue
        for peer in second:
            if len(peer) < 4 or abs(float(own[0]) - float(peer[0])) > time_tolerance:
                continue
            if float(np.linalg.norm(np.asarray(own[1:4]) - np.asarray(peer[1:4]))) < safe_distance:
                return True
    return False
