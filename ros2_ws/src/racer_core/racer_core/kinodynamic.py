"""Kinodynamic A* with RACER's constant-acceleration state transition."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import itertools
import math
import time

import numpy as np
from numpy.typing import NDArray

from .environment import EDTEnvironment


Array = NDArray[np.float64]


@dataclass(slots=True)
class KinodynamicConfig:
    max_tau: float = 0.6
    initial_max_tau: float = 0.8
    max_velocity: float = 2.0
    max_acceleration: float = 2.0
    horizon: float = 7.0
    resolution: float = 0.2
    lambda_heu: float = 5.0
    time_weight: float = 10.0
    collision_clearance: float = 0.2
    search_time_limit: float = 0.2


@dataclass(slots=True)
class KinoResult:
    status: int
    states: list[Array]
    inputs: list[Array]
    durations: list[float]
    shot_coefficients: Array | None = None
    shot_time: float = 0.0


@dataclass(slots=True)
class _Node:
    state: Array
    g: float
    f: float
    parent: "_Node | None"
    control: Array
    duration: float


class KinodynamicAStar:
    REACH_END, REACH_HORIZON, NO_PATH = 1, 2, 0

    def __init__(self, environment: EDTEnvironment, config: KinodynamicConfig | None = None) -> None:
        self.environment = environment
        self.config = config or KinodynamicConfig()

    @staticmethod
    def transition(state: Array, control: Array, duration: float) -> Array:
        result = np.empty(6, dtype=np.float64)
        result[:3] = state[:3] + state[3:] * duration + 0.5 * control * duration**2
        result[3:] = state[3:] + control * duration
        return result

    def _key(self, state: Array) -> tuple[int, ...]:
        return tuple(np.floor(state[:3] / self.config.resolution).astype(np.int64))

    def _safe_motion(self, state: Array, control: Array, duration: float) -> bool:
        for stamp in np.linspace(0.0, duration, max(3, int(math.ceil(duration / 0.05)) + 1))[1:]:
            point = self.transition(state, control, float(stamp))[:3]
            if (
                not self.environment.voxel_map.is_in_map(point)
                or self.environment.evaluate_coarse(point) <= self.config.collision_clearance
            ):
                return False
        return True

    def _heuristic(self, state: Array, goal: Array) -> tuple[float, float]:
        dp, v0, v1 = goal[:3] - state[:3], state[3:], goal[3:]
        lower = max(float(np.linalg.norm(dp)) / max(self.config.max_velocity, 1.0e-3), 0.05)
        best_cost, best_time = math.inf, lower
        for duration in np.geomspace(lower, max(lower, 10.0), 36):
            a = 6.0 * dp / duration**2 - (4.0 * v0 + 2.0 * v1) / duration
            b = -12.0 * dp / duration**3 + 6.0 * (v0 + v1) / duration**2
            energy = (
                float(np.dot(a, a)) * duration
                + float(np.dot(a, b)) * duration**2
                + float(np.dot(b, b)) * duration**3 / 3.0
            )
            cost = energy + self.config.time_weight * duration
            if cost < best_cost:
                best_cost, best_time = cost, float(duration)
        return best_cost, best_time

    @staticmethod
    def _shot(state: Array, goal: Array, duration: float) -> Array:
        p0, v0, p1, v1 = state[:3], state[3:], goal[:3], goal[3:]
        coefficients = np.zeros((3, 4), dtype=np.float64)
        coefficients[:, 0] = p0
        coefficients[:, 1] = v0
        coefficients[:, 2] = (3.0 * (p1 - p0) - (2.0 * v0 + v1) * duration) / duration**2
        coefficients[:, 3] = (-2.0 * (p1 - p0) + (v0 + v1) * duration) / duration**3
        return coefficients

    def _shot_safe(self, coefficients: Array, duration: float) -> bool:
        for stamp in np.linspace(0.0, duration, max(3, int(duration / 0.05))):
            point = sum(coefficients[:, power] * stamp**power for power in range(4))
            velocity = coefficients[:, 1] + 2.0 * coefficients[:, 2] * stamp + 3.0 * coefficients[:, 3] * stamp**2
            acceleration = 2.0 * coefficients[:, 2] + 6.0 * coefficients[:, 3] * stamp
            if (
                not self.environment.voxel_map.is_in_box(point)
                or np.max(np.abs(velocity)) > self.config.max_velocity
                or np.max(np.abs(acceleration)) > self.config.max_acceleration
                or self.environment.evaluate_coarse(point) <= self.config.collision_clearance
            ):
                return False
        return True

    @staticmethod
    def _backtrack(node: _Node) -> tuple[list[Array], list[Array], list[float]]:
        states, controls, durations = [], [], []
        while node is not None:
            states.append(node.state.copy())
            if node.parent is not None:
                controls.append(node.control.copy())
                durations.append(node.duration)
            node = node.parent
        return states[::-1], controls[::-1], durations[::-1]

    def search(
        self,
        start_position: Array,
        start_velocity: Array,
        start_acceleration: Array,
        goal_position: Array,
        goal_velocity: Array | None = None,
        initial_search: bool = True,
    ) -> KinoResult:
        start = np.r_[start_position, start_velocity].astype(np.float64)
        goal = np.r_[goal_position, np.zeros(3) if goal_velocity is None else goal_velocity].astype(np.float64)
        initial_h, initial_shot_time = self._heuristic(start, goal)
        if np.linalg.norm(start[:3] - goal[:3]) <= self.config.horizon:
            initial_shot = self._shot(start, goal, initial_shot_time)
            if self._shot_safe(initial_shot, initial_shot_time):
                return KinoResult(
                    self.REACH_END,
                    [start.copy()],
                    [],
                    [],
                    initial_shot,
                    initial_shot_time,
                )
        root = _Node(start, 0.0, self.config.lambda_heu * initial_h, None, np.zeros(3), 0.0)
        queue: list[tuple[float, int, _Node]] = [(root.f, 0, root)]
        best: dict[tuple[int, ...], float] = {self._key(start): 0.0}
        sequence = itertools.count(1)
        began = time.perf_counter()
        values = np.linspace(-self.config.max_acceleration, self.config.max_acceleration, 3)
        regular_controls = [np.asarray(value) for value in itertools.product(values, repeat=3)]
        first_controls = regular_controls
        if np.linalg.norm(start_acceleration) > 1.0e-6:
            first_controls = [
                np.asarray(start_acceleration, dtype=np.float64),
                *regular_controls,
            ]
        first_durations = (
            np.linspace(self.config.initial_max_tau / 5.0, self.config.initial_max_tau, 5)
            if initial_search else np.asarray((self.config.max_tau,))
        )
        expanded_root = False
        while queue and time.perf_counter() - began < self.config.search_time_limit:
            _, _, current = heapq.heappop(queue)
            if current.g > best.get(self._key(current.state), math.inf) + 1.0e-9:
                continue
            if np.linalg.norm(current.state[:3] - goal[:3]) <= self.config.resolution:
                states, used_controls, used_durations = self._backtrack(current)
                return KinoResult(self.REACH_END, states, used_controls, used_durations)
            if np.linalg.norm(current.state[:3] - start[:3]) >= self.config.horizon:
                states, used_controls, used_durations = self._backtrack(current)
                return KinoResult(self.REACH_HORIZON, states, used_controls, used_durations)
            controls = first_controls if not expanded_root else regular_controls
            durations = first_durations if not expanded_root else np.asarray((self.config.max_tau,))
            expanded_root = True
            for control in controls:
                for duration in durations:
                    successor = self.transition(current.state, control, float(duration))
                    if np.max(np.abs(successor[3:])) > self.config.max_velocity:
                        continue
                    if not self._safe_motion(current.state, control, float(duration)):
                        continue
                    tentative = current.g + (
                        float(np.dot(control, control)) + self.config.time_weight
                    ) * float(duration)
                    key = self._key(successor)
                    if tentative >= best.get(key, math.inf):
                        continue
                    heuristic, shot_time = self._heuristic(successor, goal)
                    node = _Node(
                        successor, tentative, tentative + self.config.lambda_heu * heuristic,
                        current, control.copy(), float(duration)
                    )
                    best[key] = tentative
                    if np.linalg.norm(successor[:3] - goal[:3]) < self.config.horizon:
                        shot = self._shot(successor, goal, shot_time)
                        if self._shot_safe(shot, shot_time):
                            states, used_controls, used_durations = self._backtrack(node)
                            return KinoResult(
                                self.REACH_END, states, used_controls, used_durations, shot, shot_time
                            )
                    heapq.heappush(queue, (node.f, next(sequence), node))
        return KinoResult(self.NO_PATH, [], [], [])
