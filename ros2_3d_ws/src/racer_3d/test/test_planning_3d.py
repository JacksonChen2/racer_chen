from types import SimpleNamespace

import numpy as np

import racer_3d.planning as planning_module
from racer_3d.planning import (
    astar3d,
    minimum_time_bspline_trajectory,
    shorten_path3d,
)
from racer_3d.scenario import (
    DRONE_RADIUS,
    get_scenario,
    obstacle_clearance,
)
from racer_3d.voxel_map import FREE, OCCUPIED, VoxelMap


def test_astar_uses_z_to_cross_low_wall():
    voxel_map = VoxelMap(0.5, (0.0, 0.0, 0.0), (5.0, 3.0, 2.0))
    states = np.full(voxel_map.shape, FREE, dtype=np.int8)
    wall_x = 4
    states[0:2, :, wall_x] = OCCUPIED
    voxel_map.set_states(states)
    blocked = voxel_map.inflated_blocked(0.20, unknown_is_blocked=True)
    path = astar3d(blocked, (1, 3, 0), (8, 3, 0), voxel_map.resolution)
    assert path
    assert max(cell[2] for cell in path) >= 2
    assert all(not blocked[cell[2], cell[1], cell[0]] for cell in path)


def test_bspline_is_xyz_and_clear():
    voxel_map = VoxelMap(0.25, (0.0, 0.0, 0.0), (4.0, 3.0, 2.0))
    states = np.full(voxel_map.shape, FREE, dtype=np.int8)
    states[0:4, :, 7:9] = OCCUPIED
    voxel_map.set_states(states)
    blocked = voxel_map.inflated_blocked(
        0.25 + 0.5 * np.sqrt(3.0) * voxel_map.resolution, True
    )
    path = astar3d(blocked, (2, 6, 2), (13, 6, 2), voxel_map.resolution)
    assert path
    short = shorten_path3d(voxel_map, path, blocked)
    points = [voxel_map.grid_to_world(cell) for cell in short]
    trajectory, minimum = minimum_time_bspline_trajectory(
        points, voxel_map, 0.25, 1.0, 1.5
    )
    assert len(trajectory) > 2
    assert minimum >= 0.25 - 1.0e-6
    assert any(abs(sample[3] - trajectory[0][3]) > 0.1 for sample in trajectory)


def test_large_acceptance_scene_geometry_and_starts():
    scenario = get_scenario("acceptance_20x50x3")
    assert scenario.map_size == (20.0, 50.0, 3.0)
    assert scenario.coarse_grid_size == (5.0, 10.0, 3.0)
    assert len(scenario.starts) == 3
    assert all(
        obstacle_clearance(start, scenario.obstacles) > DRONE_RADIUS
        for start in scenario.starts
    )
    assert any(
        obstacle.name == "south_low_overflight_wall"
        and obstacle.minimum[0] <= -10.0
        and obstacle.maximum[0] >= 10.0
        and obstacle.maximum[2] < scenario.map_max[2]
        for obstacle in scenario.obstacles
    )
    assert any(
        obstacle.name == "south_high_underflight_wall"
        and obstacle.minimum[2] > scenario.map_min[2]
        and obstacle.minimum[0] <= -10.0
        and obstacle.maximum[0] >= 10.0
        for obstacle in scenario.obstacles
    )


def test_warehouse_external_usd_volume_and_starts():
    scenario = get_scenario("warehouse_simple")
    assert scenario.map_size == (19.4, 29.8, 9.0)
    assert scenario.truth_mode == "observed_volume"
    assert scenario.obstacles == ()
    assert len(scenario.starts) == 5
    assert scenario.starts[:3] == (
        (-6.0, -10.0, 0.60),
        (0.0, -10.0, 1.50),
        (6.0, -10.0, 2.40),
    )
    assert all(
        all(
            lower < value < upper
            for value, lower, upper in zip(
                start, scenario.map_min, scenario.map_max
            )
        )
        for start in scenario.starts
    )


def test_planner_falls_back_when_owned_frontier_is_unreachable(monkeypatch):
    voxel_map = VoxelMap(1.0, (0.0, 0.0, 0.0), (6.0, 4.0, 4.0))
    voxel_map.set_states(np.full(voxel_map.shape, FREE, dtype=np.int8))
    owned_cluster = [
        (1, 1, 1),
        (1, 2, 1),
        (1, 1, 2),
        (1, 2, 2),
    ]
    reachable_cluster = [
        (4, 1, 1),
        (4, 2, 1),
        (4, 1, 2),
        (4, 2, 2),
    ]
    monkeypatch.setattr(
        voxel_map,
        "frontier_clusters",
        lambda: [owned_cluster, reachable_cluster],
    )

    class FakeHGrid:
        @staticmethod
        def containing(point):
            return SimpleNamespace(
                id="owned" if float(point[0]) < 3.0 else "other"
            )

    def candidates(_, cluster, __, ___):
        if cluster == owned_cluster:
            return []
        return [voxel_map.grid_to_world(reachable_cluster[0])]

    monkeypatch.setattr(
        planning_module, "_viewpoint_candidates", candidates
    )
    monkeypatch.setattr(
        planning_module,
        "astar3d",
        lambda _, start, goal, __: [start, goal],
    )
    monkeypatch.setattr(
        planning_module,
        "minimum_time_bspline_trajectory",
        lambda points, *_: (
            [(0.0, *points[0]), (1.0, *points[-1])],
            1.0,
        ),
    )

    plan = planning_module.plan_exploration(
        voxel_map,
        FakeHGrid(),
        owned_cells=["owned"],
        coverage_route=["owned"],
        position=(0.5, 0.5, 0.5),
        clearance=0.2,
        max_speed=0.35,
        max_acceleration=1.4,
    )
    assert plan is not None
    assert plan.goal == voxel_map.grid_to_world(reachable_cluster[0])
