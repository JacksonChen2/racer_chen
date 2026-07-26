from racer_3d.allocation import capacity_partition
from racer_3d.hgrid import HierarchicalGrid3D
from racer_3d.voxel_map import FREE, VoxelMap


def test_hgrid_subdivides_xyz_and_allocates_every_cell():
    voxel_map = VoxelMap(0.5, (-3.0, -3.0, 0.0), (6.0, 6.0, 2.0))
    states = voxel_map.states()
    states[:, :4, :4] = FREE
    voxel_map.set_states(states)
    hgrid = HierarchicalGrid3D(
        voxel_map,
        coarse_size=(3.0, 3.0, 2.0),
        levels=2,
        subdivide_known_ratio=0.2,
        minimum_unknown=1,
    )
    cells = hgrid.update()
    assert cells
    assert any(cell.level == 2 for cell in cells.values())
    starts = ((-2.5, -2.5, 0.5), (2.5, 2.5, 1.5))
    owners = hgrid.initial_owners(starts)
    assert set(owners) == set(cells)
    partition = capacity_partition(list(cells.values()), *starts)
    assert set(partition.first).isdisjoint(partition.second)
    assert set(partition.first) | set(partition.second) == set(cells)
    assert partition.first_demand + partition.second_demand == sum(
        cell.demand for cell in cells.values()
    )
