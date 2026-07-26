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
        self.assertGreater(np.count_nonzero(state == FREE), 40)
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
