"""Layered-graph yaw planner ported from RACER's ``heading_planner.cpp``."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from .frontier import FrontierFinder
from .math_utils import wrap_yaw, yaw_difference


Array = NDArray[np.float64]


@dataclass(slots=True)
class HeadingConfig:
    yaw_diff: float = 0.3
    half_vertical_num: int = 3
    max_yaw_rate: float = 1.0
    weight: float = 1.0
    info_lambda1: float = 0.0
    info_lambda2: float = 0.0


@dataclass(slots=True)
class _YawNode:
    yaw: float
    gain: float
    cost: float = math.inf
    parent: "_YawNode | None" = None


class HeadingPlanner:
    def __init__(self, frontier: FrontierFinder, config: HeadingConfig | None = None) -> None:
        self.frontier = frontier
        self.config = config or HeadingConfig()

    def _penalty(self, delta: float, duration: float) -> float:
        excess = abs(delta) / max(duration, 1.0e-3) - self.config.max_yaw_rate
        return max(excess, 0.0) ** 2

    def plan(
        self,
        positions: Array,
        times: Array,
        start_yaw: float,
        goal_yaw: float | None = None,
        current_position: Array | None = None,
    ) -> Array:
        points = np.asarray(positions, dtype=np.float64)
        stamps = np.asarray(times, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) != len(stamps):
            raise ValueError("positions must be (N,3) and times must contain N entries")
        if not len(points):
            return np.empty(0, dtype=np.float64)
        layers: list[list[_YawNode]] = [[_YawNode(wrap_yaw(start_yaw), 0.0, 0.0)]]
        center = wrap_yaw(start_yaw)
        for index in range(1, len(points)):
            final = goal_yaw is not None and index == len(points) - 1
            candidates = [wrap_yaw(goal_yaw)] if final else [
                wrap_yaw(center + offset * self.config.yaw_diff)
                for offset in range(-self.config.half_vertical_num, self.config.half_vertical_num + 1)
            ]
            layer: list[_YawNode] = []
            for yaw in candidates:
                gain = float(self.frontier.information_gain(points[index], yaw))
                if current_position is not None:
                    gain *= math.exp(
                        -self.config.info_lambda1 * float(np.linalg.norm(points[index] - points[index - 1]))
                        - self.config.info_lambda2 * float(np.linalg.norm(points[index] - current_position))
                    )
                node = _YawNode(yaw, gain)
                dt = max(float(stamps[index] - stamps[index - 1]), 1.0e-3)
                for previous in layers[-1]:
                    cost = (
                        previous.cost
                        - gain
                        + self.config.weight
                        * self._penalty(yaw_difference(yaw, previous.yaw), dt)
                    )
                    if cost < node.cost:
                        node.cost, node.parent = cost, previous
                layer.append(node)
            layers.append(layer)
            center = min(layer, key=lambda item: item.cost).yaw
        terminal = min(layers[-1], key=lambda item: item.cost)
        result = [terminal.yaw]
        while terminal.parent is not None:
            terminal = terminal.parent
            result.append(terminal.yaw)
        return np.unwrap(np.asarray(result[::-1], dtype=np.float64))

