"""LKH problem-file bridge used by RACER's TSP and multi-TSP services."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import shutil
import subprocess
import tempfile

import numpy as np
from numpy.typing import NDArray


Array = NDArray[np.float64]


@dataclass(slots=True)
class LkhSolver:
    executable: str = "LKH"
    runs: int = 1
    scale: int = 100

    def available(self) -> bool:
        return shutil.which(self.executable) is not None

    def solve_tsp(self, costs: Array, start_index: int = 0) -> list[int]:
        matrix = np.asarray(costs, dtype=np.float64)
        if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
            raise ValueError("cost matrix must be square")
        if not self.available():
            return self._fallback_tsp(matrix, start_index)
        with tempfile.TemporaryDirectory(prefix="racer_lkh_") as directory:
            root = Path(directory)
            problem, parameter, tour = root / "problem.atsp", root / "problem.par", root / "tour.txt"
            weights = np.rint(np.maximum(matrix, 0.0) * self.scale).astype(np.int64)
            problem.write_text(
                "\n".join([
                    "NAME: RACER", "TYPE: ATSP", f"DIMENSION: {len(matrix)}",
                    "EDGE_WEIGHT_TYPE: EXPLICIT", "EDGE_WEIGHT_FORMAT: FULL_MATRIX",
                    "EDGE_WEIGHT_SECTION", *[" ".join(map(str, row)) for row in weights], "EOF"
                ]), encoding="utf-8"
            )
            parameter.write_text(
                f"PROBLEM_FILE = {problem}\nOUTPUT_TOUR_FILE = {tour}\nRUNS = {self.runs}\n",
                encoding="utf-8",
            )
            subprocess.run([self.executable, str(parameter)], check=True, capture_output=True, text=True)
            ids, reading = [], False
            for line in tour.read_text(encoding="utf-8").splitlines():
                if line.strip() == "TOUR_SECTION":
                    reading = True
                elif reading:
                    value = int(line.strip())
                    if value == -1:
                        break
                    ids.append(value - 1)
            if start_index in ids:
                offset = ids.index(start_index)
                ids = ids[offset:] + ids[:offset]
            return ids

    @staticmethod
    def _fallback_tsp(matrix: Array, start_index: int) -> list[int]:
        """Use exact dynamic programming for normal RACER tour sizes.

        LKH remains the production solver.  This fallback avoids silently
        changing the planning objective to nearest-neighbour when LKH is not
        installed, which is especially important in a Python-only deployment.
        """
        nodes = [index for index in range(len(matrix)) if index != start_index]
        if len(nodes) <= 11:
            states: dict[tuple[int, int], tuple[float, list[int]]] = {}
            for offset, node in enumerate(nodes):
                states[1 << offset, offset] = (
                    float(matrix[start_index, node]),
                    [start_index, node],
                )
            for mask in range(1, 1 << len(nodes)):
                for last in range(len(nodes)):
                    item = states.get((mask, last))
                    if item is None:
                        continue
                    cost, path = item
                    for target in range(len(nodes)):
                        if mask & (1 << target):
                            continue
                        next_mask = mask | (1 << target)
                        next_cost = cost + float(matrix[nodes[last], nodes[target]])
                        previous = states.get((next_mask, target))
                        if previous is None or next_cost < previous[0]:
                            states[next_mask, target] = (
                                next_cost,
                                [*path, nodes[target]],
                            )
            full = (1 << len(nodes)) - 1
            return min(
                (
                    value
                    for (mask, _), value in states.items()
                    if mask == full
                ),
                key=lambda item: item[0],
                default=(0.0, [start_index]),
            )[1]

        remaining, route, current = set(nodes), [start_index], start_index
        while remaining:
            current = min(remaining, key=lambda index: matrix[current, index])
            remaining.remove(current)
            route.append(current)
        return route

    def solve_multiple(self, costs: Array, depot_count: int) -> list[list[int]]:
        remaining = set(range(depot_count, len(costs)))
        tours, current = [[] for _ in range(depot_count)], list(range(depot_count))
        matrix = np.asarray(costs)
        while remaining:
            drone = min(
                range(depot_count),
                key=lambda candidate: min(matrix[current[candidate], node] for node in remaining),
            )
            target = min(remaining, key=lambda node: matrix[current[drone], node])
            tours[drone].append(target)
            current[drone] = target
            remaining.remove(target)
        return tours
