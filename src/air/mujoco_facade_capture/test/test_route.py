"""航线参数边界和蛇形遍历规则。"""

import numpy as np
import pytest

from facade.route import scan_route


def test_route_covers_rectangle_and_alternates_direction():
    route = scan_route()
    assert route.shape[1] == 3
    np.testing.assert_allclose(route[:, 1], -4)
    assert route[:, 0].min() == -3 and route[:, 0].max() == 3
    for i, z in enumerate(np.unique(route[:, 2])):
        row = route[route[:, 2] == z, 0]
        assert np.all(np.diff(row) > 0) if i % 2 == 0 else np.all(np.diff(row) < 0)


@pytest.mark.parametrize("kwargs", [{"overlap": 1}, {"distance": 0}, {"width": float("nan")}])
def test_invalid_route_parameters(kwargs):
    with pytest.raises(ValueError):
        scan_route(**kwargs)
