"""Continuous obstacle and inter-UAV collision safety filters."""

import math
from typing import Iterable, Sequence, Tuple

import numpy as np


Vector2 = Tuple[float, float]


def limit_norm(vector: Vector2, limit: float) -> Vector2:
    norm = math.hypot(*vector)
    if norm <= limit or norm < 1.0e-12:
        return vector
    scale = limit / norm
    return vector[0] * scale, vector[1] * scale


def obstacle_brake(
    preferred: Vector2,
    scan_ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    yaw: float,
    robot_radius: float,
    braking_margin: float = 0.25,
) -> Vector2:
    """Remove velocity components that approach a close lidar return."""

    velocity = np.asarray(preferred, dtype=float)
    for index, measured in enumerate(scan_ranges):
        if not math.isfinite(measured):
            continue
        clearance = measured - robot_radius
        if clearance >= braking_margin + 0.7:
            continue
        angle = yaw + angle_min + index * angle_increment
        direction = np.asarray((math.cos(angle), math.sin(angle)))
        toward = float(np.dot(velocity, direction))
        if toward <= 0.0:
            continue
        allowed = max(0.0, (clearance - braking_margin) * 1.5)
        if toward > allowed:
            velocity -= (toward - allowed) * direction
    return float(velocity[0]), float(velocity[1])


def cbf_swarm_filter(
    preferred: Vector2,
    own_position: Vector2,
    peers: Iterable[Tuple[int, Vector2, Vector2]],
    safe_distance: float,
    gamma: float = 1.5,
    speed_limit: float | None = None,
) -> Vector2:
    """Project velocity onto pairwise control-barrier half planes.

    For h = ||p_i-p_j||²-d_safe², enforce h_dot >= -gamma*h.
    Repeated projections solve the small convex feasibility problem closely
    enough for the controller rate used here.
    """

    velocity = np.asarray(preferred, dtype=float)
    own = np.asarray(own_position, dtype=float)
    peer_list = list(peers)
    for _ in range(8):
        for _, peer_position, peer_velocity in peer_list:
            other = np.asarray(peer_position, dtype=float)
            other_velocity = np.asarray(peer_velocity, dtype=float)
            relative = own - other
            distance_sq = float(np.dot(relative, relative))
            if distance_sq < 1.0e-8:
                relative = np.asarray((1.0, 0.0))
                distance_sq = 1.0
            h_value = distance_sq - safe_distance**2
            # 2*r·v_i >= 2*r·v_j - gamma*h.
            lower = 2.0 * float(np.dot(relative, other_velocity)) - gamma * h_value
            actual = 2.0 * float(np.dot(relative, velocity))
            if actual + 1.0e-9 < lower:
                velocity += ((lower - actual) / (4.0 * distance_sq)) * (
                    2.0 * relative
                )
        if speed_limit is not None:
            velocity[:] = limit_norm(
                (float(velocity[0]), float(velocity[1])), speed_limit
            )
    return float(velocity[0]), float(velocity[1])


def emergency_separation(
    preferred: Vector2,
    own_position: Vector2,
    peers: Iterable[Tuple[int, Vector2, Vector2]],
    activation_distance: float,
    max_speed: float,
) -> Vector2:
    """Override task following when communication delay erodes separation."""

    own = np.asarray(own_position, dtype=float)
    correction = np.zeros(2, dtype=float)
    closest = math.inf
    for _, peer_position, _ in peers:
        difference = own - np.asarray(peer_position, dtype=float)
        distance = float(np.linalg.norm(difference))
        closest = min(closest, distance)
        if distance >= activation_distance:
            continue
        if distance < 1.0e-6:
            direction = np.asarray((1.0, 0.0))
        else:
            direction = difference / distance
        correction += (
            0.30 + 2.0 * (activation_distance - distance)
        ) * direction
    if closest >= activation_distance:
        return preferred
    # At close range separation has priority over exploration progress.
    return limit_norm(
        (float(correction[0]), float(correction[1])), max_speed
    )


def predicted_path_conflict(
    own_trajectory: Sequence[Tuple[float, float, float]],
    peer_trajectory: Sequence[Tuple[float, float, float]],
    safe_distance: float,
    time_tolerance: float = 0.35,
) -> bool:
    for own in own_trajectory:
        for peer in peer_trajectory:
            if abs(own[0] - peer[0]) > time_tolerance:
                continue
            if math.hypot(own[1] - peer[1], own[2] - peer[2]) < safe_distance:
                return True
    return False
