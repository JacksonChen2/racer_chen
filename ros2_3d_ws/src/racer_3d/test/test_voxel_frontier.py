import numpy as np

from racer_3d.voxel_map import FREE, OCCUPIED, UNKNOWN, VoxelMap


def test_point_cloud_updates_xyz_and_esdf():
    voxel_map = VoxelMap(0.25, (-2.0, -2.0, 0.0), (4.0, 4.0, 2.0))
    origin = np.asarray((0.0, 0.0, 1.0))
    points = np.asarray(
        (
            (1.5, 0.0, 1.0),
            (0.0, 1.5, 1.0),
            (0.0, 0.0, 1.8),
        )
    )
    for _ in range(2):
        voxel_map.update_point_cloud(origin, points, 3.0)
    states = voxel_map.states()
    occupied = [voxel_map.world_to_grid(point) for point in points]
    assert all(states[cell[2], cell[1], cell[0]] == OCCUPIED for cell in occupied)
    midpoint = voxel_map.world_to_grid((0.75, 0.0, 1.0))
    assert states[midpoint[2], midpoint[1], midpoint[0]] == FREE
    occupied_center = voxel_map.grid_to_world(occupied[0])
    assert voxel_map.distance_at(occupied_center, False) < 0.0
    assert voxel_map.distance_at((0.75, 0.0, 1.0), False) > 0.0


def test_frontier_is_volumetric_and_merges():
    voxel_map = VoxelMap(0.5, (0.0, 0.0, 0.0), (3.0, 3.0, 2.0))
    states = np.full(voxel_map.shape, UNKNOWN, dtype=np.int8)
    states[1:3, 1:5, 1:5] = FREE
    voxel_map.set_states(states)
    clusters = voxel_map.frontier_clusters(minimum_size=1)
    assert clusters
    z_values = {cell[2] for cluster in clusters for cell in cluster}
    assert len(z_values) >= 2
    peer = np.full(voxel_map.shape, UNKNOWN, dtype=np.int8)
    peer[0, 0, 0] = OCCUPIED
    voxel_map.merge(peer)
    assert voxel_map.states()[0, 0, 0] == OCCUPIED


def test_boundary_surface_hit_is_not_discarded():
    voxel_map = VoxelMap(0.2, (-1.0, -1.0, 0.0), (2.0, 2.0, 2.0))
    origin = np.asarray((0.0, 0.0, 1.0))
    floor_hit = np.asarray(((0.0, 0.0, 0.0),))
    for _ in range(2):
        voxel_map.update_point_cloud(origin, floor_hit, 3.0)
    floor_cell = voxel_map.world_to_grid((0.0, 0.0, 0.1))
    assert voxel_map.states()[
        floor_cell[2], floor_cell[1], floor_cell[0]
    ] == OCCUPIED
