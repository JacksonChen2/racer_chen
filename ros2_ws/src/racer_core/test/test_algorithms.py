from __future__ import annotations

import itertools
import unittest

import numpy as np

from racer_core import (
    AStar,
    EDTEnvironment,
    KinodynamicAStar,
    KinodynamicConfig,
    LkhSolver,
    MapChunk,
    MultiMapManager,
    NonUniformBspline,
    PartitionConfig,
    PlannerConfig,
    PolynomialTrajectory,
    RacerPlanner,
    VehicleState,
    VoxelMap,
    VoxelMapConfig,
)
from racer_core.environment import PredictedBox
from racer_core.raycast import RayCaster


def small_environment() -> EDTEnvironment:
    config = VoxelMapConfig(
        resolution=0.2,
        map_size=(12.0, 12.0, 2.0),
        ground_height=0.0,
        box_min=(-5.0, -5.0, 0.1),
        box_max=(5.0, 5.0, 1.8),
    )
    return EDTEnvironment(VoxelMap(config))


class AlgorithmTests(unittest.TestCase):
    def test_astar_python_budget_reaches_open_goal(self) -> None:
        search = AStar(
            small_environment(),
            resolution=0.3,
            max_search_time=0.1,
        )
        status = search.search(
            np.asarray((0.0, 0.0, 1.0)),
            np.asarray((5.0, 0.0, 1.0)),
            optimistic=True,
        )
        self.assertEqual(status, AStar.REACH_END)
        self.assertGreater(len(search.path), 2)

    def test_raycast_same_axis_voxel_terminates(self) -> None:
        caster = RayCaster(0.5, np.asarray((-5.0, -5.0, 0.0)))
        indices = list(
            caster.indices(
                np.asarray((2.5, 0.1, 1.0)),
                np.asarray((0.0, 0.0, 1.0)),
            )
        )
        self.assertLess(len(indices), 20)
        np.testing.assert_array_equal(
            indices[-1],
            np.floor(
                (np.asarray((0.0, 0.0, 1.0)) - caster.origin)
                / caster.resolution
            ).astype(np.int64),
        )

    def test_polynomial_waypoint_boundaries_are_continuous(self) -> None:
        points = np.asarray(
            ((0.0, 0.0, 1.0), (1.0, 0.0, 1.0), (2.0, 1.0, 1.0))
        )
        trajectory = PolynomialTrajectory.through_waypoints(
            points,
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            np.zeros(3),
            (1.0, 1.0),
        )
        np.testing.assert_allclose(trajectory.evaluate(0.0), points[0], atol=1.0e-8)
        np.testing.assert_allclose(trajectory.evaluate(1.0), points[1], atol=1.0e-8)
        np.testing.assert_allclose(trajectory.evaluate(2.0), points[2], atol=1.0e-8)
        np.testing.assert_allclose(
            trajectory.evaluate(1.0 - 1.0e-6, 1),
            trajectory.evaluate(1.0 + 1.0e-6, 1),
            atol=1.0e-4,
        )

    def test_kinodynamic_search_leaves_hover_state(self) -> None:
        search = KinodynamicAStar(
            small_environment(),
            KinodynamicConfig(search_time_limit=0.5, horizon=7.0),
        )
        result = search.search(
            np.asarray((0.0, 0.0, 1.0)),
            np.zeros(3),
            np.zeros(3),
            np.asarray((2.0, 0.0, 1.0)),
        )
        self.assertEqual(result.status, KinodynamicAStar.REACH_END)
        self.assertTrue(result.states)

    def test_dynamic_box_returns_escape_gradient(self) -> None:
        environment = small_environment()
        box = PredictedBox(
            np.asarray((1.0, 0.0, 1.0)),
            np.zeros(3),
            np.ones(3),
        )
        distance, gradient = environment.distance_to_box_with_gradient(
            box, np.asarray((1.0, 0.0, 1.0)), 0.0
        )
        self.assertLessEqual(distance, 0.0)
        self.assertAlmostEqual(float(np.linalg.norm(gradient)), 1.0)

    def test_lkh_fallback_optimizes_the_open_tour(self) -> None:
        matrix = np.asarray(
            (
                (0.0, 1.0, 2.0, 9.0),
                (9.0, 0.0, 8.0, 2.0),
                (9.0, 1.0, 0.0, 1.0),
                (9.0, 9.0, 1.0, 0.0),
            )
        )
        route = LkhSolver(executable="definitely-not-installed").solve_tsp(matrix)
        route_cost = sum(matrix[a, b] for a, b in zip(route, route[1:]))
        expected = min(
            sum(matrix[a, b] for a, b in zip((0, *order), (0, *order)[1:]))
            for order in itertools.permutations((1, 2, 3))
        )
        self.assertEqual(route[0], 0)
        self.assertAlmostEqual(float(route_cost), float(expected))

    def test_hierarchical_grid_is_in_active_planner(self) -> None:
        planner = RacerPlanner(
            small_environment(),
            PlannerConfig(astar_max_search_time=0.1),
            partition_config=PartitionConfig(
                minimum_unknown=1,
                minimum_frontier=1,
                minimum_free=1_000_000,
                grid_size=5.0,
            ),
        )
        grids = planner._ordered_grids(
            VehicleState(position=np.asarray((0.0, 0.0, 1.0)))
        )
        self.assertTrue(grids)
        self.assertEqual(grids, planner.grid_ids)

    def test_end_to_end_frontier_plan_produces_trajectory(self) -> None:
        environment = EDTEnvironment(
            VoxelMap(
                VoxelMapConfig(
                    resolution=0.5,
                    map_size=(10.0, 10.0, 2.0),
                    ground_height=0.0,
                    obstacles_inflation=0.0,
                    box_min=(-4.5, -4.5, 0.1),
                    box_max=(4.5, 4.5, 1.9),
                )
            )
        )
        voxel_map = environment.voxel_map
        for x in range(int(voxel_map.voxel_count[0])):
            for y in range(int(voxel_map.voxel_count[1])):
                position = voxel_map.index_to_position(
                    np.asarray((x, y, 2), dtype=np.int64)
                )
                if np.linalg.norm(position[:2]) < 3.0:
                    voxel_map.occupancy[x, y, :] = voxel_map.clamp_min_log
        voxel_map.update_min = np.asarray((-3.0, -3.0, 0.0))
        voxel_map.update_max = np.asarray((3.0, 3.0, 2.0))
        voxel_map.update_esdf()

        from racer_core import FrontierConfig

        planner = RacerPlanner(
            environment,
            PlannerConfig(
                use_optimization=False,
                use_active_perception=False,
                astar_max_search_time=0.2,
            ),
            FrontierConfig(
                cluster_min=2,
                cluster_size_xy=2.0,
                cluster_size_z=2.0,
                min_candidate_distance=0.2,
                min_candidate_clearance=0.0,
                candidate_delta_yaw=np.deg2rad(30.0),
                candidate_radius_count=2,
                candidate_radius_min=1.0,
                candidate_radius_max=2.0,
                downsample=1,
                min_visible_count=0,
            ),
            partition_config=PartitionConfig(
                minimum_unknown=1,
                minimum_frontier=1,
                minimum_free=1_000_000,
                grid_size=5.0,
            ),
        )
        result = planner.plan(
            VehicleState(position=np.asarray((0.0, 0.0, 1.0)))
        )
        self.assertEqual(result.status.name, "SUCCEED")
        self.assertIsNotNone(result.position_control_points)
        self.assertGreater(result.duration, 0.0)
        self.assertGreater(result.yaw_knot_span, 0.0)
        self.assertTrue(planner.grid_ids)

    def test_partial_map_chunk_is_flushed(self) -> None:
        manager = MultiMapManager(1, 1, chunk_size=4)
        self.assertEqual(manager.append_addresses([1, 2], lambda _: 1), [])
        chunk = manager.flush_pending(lambda _: 1)
        self.assertIsInstance(chunk, MapChunk)
        self.assertEqual(chunk.addresses, [1, 2])

    def test_bspline_preserves_independent_yaw_interval(self) -> None:
        points = np.asarray(
            ((0.0,), (0.1,), (0.2,), (0.3,), (0.4,)), dtype=np.float64
        )
        yaw = NonUniformBspline(points, 3, 0.2)
        self.assertAlmostEqual(yaw.knot_span, 0.2)
        self.assertGreater(yaw.get_time_sum(), 0.0)


if __name__ == "__main__":
    unittest.main()
