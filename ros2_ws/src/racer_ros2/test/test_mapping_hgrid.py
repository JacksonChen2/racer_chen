import math
import unittest

import numpy as np

from racer_ros2.hgrid import HierarchicalGrid
from racer_ros2.mapping import FREE, OCCUPIED, OccupancyMap


class MappingHGridTest(unittest.TestCase):
    def test_scan_mapping_and_frontiers(self):
        occupancy = OccupancyMap(0.5, (-5.0, -5.0), (10.0, 10.0))
        ranges = [3.0] * 72
        occupancy.update_scan(
            (0.0, 0.0),
            0.0,
            ranges,
            -math.pi,
            2.0 * math.pi / 71,
            4.0,
        )
        state = occupancy.states()
        self.assertGreater(np.count_nonzero(state == FREE), 20)
        self.assertGreater(np.count_nonzero(state == OCCUPIED), 20)
        self.assertGreater(occupancy.coverage(), 0.15)
        self.assertTrue(occupancy.frontier_clusters(minimum_size=2))

    def test_hgrid_subdivides_partially_observed_cells(self):
        occupancy = OccupancyMap(0.5, (0.0, 0.0), (10.0, 10.0))
        values = np.full((20, 20), -1, dtype=np.int8)
        values[:8, :8] = FREE
        occupancy.set_states(values)
        hgrid = HierarchicalGrid(
            occupancy,
            coarse_size=5.0,
            levels=2,
            subdivide_known_ratio=0.25,
            minimum_unknown=2,
        )
        cells = hgrid.update()
        self.assertTrue(cells)
        self.assertTrue(any(cell.level == 2 for cell in cells.values()))
        self.assertTrue(any(cell.level == 1 for cell in cells.values()))
        self.assertEqual(
            len({cell.id for cell in cells.values()}), len(cells)
        )

    def test_peer_map_merge_preserves_obstacles(self):
        occupancy = OccupancyMap(1.0, (0.0, 0.0), (4.0, 4.0))
        free = np.full((4, 4), -1, dtype=np.int8)
        free[1, 1] = FREE
        occupancy.merge(free)
        occupied = np.full((4, 4), -1, dtype=np.int8)
        occupied[1, 1] = OCCUPIED
        occupancy.merge(occupied)
        self.assertEqual(occupancy.states()[1, 1], OCCUPIED)

    def test_non_finite_sensor_samples_are_ignored(self):
        occupancy = OccupancyMap(0.5, (-2.0, -2.0), (4.0, 4.0))
        occupancy.update_scan(
            (0.0, 0.0),
            0.0,
            [float("nan"), 1.0, float("inf")],
            float("nan"),
            0.1,
            4.0,
        )
        self.assertEqual(np.count_nonzero(occupancy.observations), 1)

    def test_clipped_miss_does_not_mark_map_edge_free(self):
        occupancy = OccupancyMap(0.5, (-2.0, -2.0), (4.0, 4.0))
        occupancy.update_scan(
            (0.0, 0.0),
            0.0,
            [10.0],
            0.0,
            1.0,
            10.0,
        )
        edge = occupancy.world_to_grid(1.99, 0.0)
        self.assertIsNotNone(edge)
        self.assertEqual(occupancy.observations[edge[1], edge[0]], 0)

    def test_hit_on_grid_boundary_marks_obstacle_side(self):
        occupancy = OccupancyMap(0.25, (0.0, 0.0), (2.0, 1.0))
        occupancy.update_scan(
            (0.5, 0.5),
            0.0,
            [0.5],
            0.0,
            1.0,
            2.0,
        )
        free_side = occupancy.world_to_grid(0.875, 0.5)
        obstacle_side = occupancy.world_to_grid(1.125, 0.5)
        self.assertNotEqual(
            occupancy.states()[free_side[1], free_side[0]], OCCUPIED
        )
        self.assertEqual(
            occupancy.states()[obstacle_side[1], obstacle_side[0]],
            OCCUPIED,
        )
