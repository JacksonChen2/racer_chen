import math
import unittest

import numpy as np

from racer_ros2.allocation import capacity_partition
from racer_ros2.hgrid import HGridCell, HierarchicalGrid
from racer_ros2.mapping import FREE, OCCUPIED, OccupancyMap
from racer_ros2.planning import astar, plan_exploration


def make_cell(index, x, y, demand):
    return HGridCell(
        level=1,
        ix=index,
        iy=0,
        bounds=(index, 0, index + 1, 1),
        centroid=(x, y),
        unknown=demand,
        total=demand + 5,
    )


class AllocationPlanningTest(unittest.TestCase):
    def test_capacity_partition_is_complete_disjoint_and_balanced(self):
        cells = [
            make_cell(0, -4.0, -1.0, 10),
            make_cell(1, -2.0, 2.0, 9),
            make_cell(2, 1.0, -2.0, 8),
            make_cell(3, 4.0, 1.0, 11),
            make_cell(4, 0.0, 3.0, 7),
        ]
        result = capacity_partition(cells, (-5.0, 0.0), (5.0, 0.0))
        first, second = set(result.first), set(result.second)
        self.assertTrue(first.isdisjoint(second))
        self.assertEqual(first | second, {cell.id for cell in cells})
        self.assertLessEqual(
            max(result.first_demand, result.second_demand),
            math.ceil(0.75 * sum(cell.demand for cell in cells)),
        )

    def test_exact_open_cvrp_selects_short_routes(self):
        cells = [
            make_cell(0, -4.0, 0.0, 5),
            make_cell(1, -3.0, 0.0, 5),
            make_cell(2, 3.0, 0.0, 5),
            make_cell(3, 4.0, 0.0, 5),
        ]
        result = capacity_partition(
            cells,
            (-5.0, 0.0),
            (5.0, 0.0),
            consistency_penalty=0.0,
        )
        self.assertEqual(set(result.first), {"1:0:0", "1:1:0"})
        self.assertEqual(set(result.second), {"1:2:0", "1:3:0"})

    def test_astar_does_not_cut_obstacle_corners(self):
        blocked = np.zeros((8, 8), dtype=bool)
        blocked[2:7, 3] = True
        path = astar(blocked, (1, 4), (6, 4))
        self.assertTrue(path)
        self.assertTrue(all(not blocked[y, x] for x, y in path))
        self.assertGreater(len(path), 5)

    def test_cp_guided_frontier_plan_is_collision_free(self):
        occupancy = OccupancyMap(0.5, (-5.0, -5.0), (10.0, 10.0))
        values = np.full((20, 20), -1, dtype=np.int8)
        values[5:15, 2:11] = FREE
        values[7:13, 7] = OCCUPIED
        values[10, 7] = FREE
        occupancy.set_states(values)
        hgrid = HierarchicalGrid(
            occupancy,
            coarse_size=5.0,
            levels=2,
            subdivide_known_ratio=0.25,
            minimum_unknown=2,
        )
        hgrid.update()
        owners = hgrid.initial_owners([(-3.5, 0.0)])
        owned = [cell_id for cell_id, owner in owners.items() if owner == 0]
        route = hgrid.coverage_route(owned, (-3.5, 0.0))
        plan = plan_exploration(
            occupancy,
            hgrid,
            owned,
            route,
            (-3.5, 0.0),
            0.0,
            clearance=0.45,
        )
        self.assertIsNotNone(plan)
        blocked = occupancy.inflated_blocked(
            0.45, unknown_is_blocked=False
        )
        for _, x, y in plan.trajectory:
            cell = occupancy.world_to_grid(x, y)
            self.assertIsNotNone(cell)
            self.assertFalse(blocked[cell[1], cell[0]])
