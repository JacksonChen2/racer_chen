import numpy as np

from racer_3d.planning import (
    astar3d,
    minimum_time_bspline_trajectory,
    shorten_path3d,
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
