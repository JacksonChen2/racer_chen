import numpy as np
import pytest

from agentic_crpo.action_mapping import ActionMapping


@pytest.mark.parametrize("n_uavs", [1, 3, 10, 15])
def test_action_mapping_round_trip_and_zero_diagonal(n_uavs):
    mapping = ActionMapping(n_uavs)
    action = np.arange(mapping.action_dim, dtype=np.int8) % 2
    matrix = mapping.decode(action)
    assert matrix.shape == (n_uavs + 1, n_uavs)
    assert np.all(np.diag(matrix[:n_uavs]) == 0)
    np.testing.assert_array_equal(mapping.encode(matrix), action)


def test_action_mapping_rejects_diagonal():
    mapping = ActionMapping(3)
    matrix = np.zeros((4, 3), dtype=np.int8)
    matrix[1, 1] = 1
    with pytest.raises(ValueError, match="diagonal"):
        mapping.encode(matrix)

