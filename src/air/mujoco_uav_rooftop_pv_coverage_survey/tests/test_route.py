"""Coverage-route tests / 覆盖航线测试。"""

import pytest

from rooftop_pv.route import RouteConfig, generate_lawnmower_route, route_length


def test_route_stays_inside_roof_and_covers_both_edges() -> None:
    config = RouteConfig(roof_width=20.0, roof_depth=14.0, strip_spacing=2.0, edge_margin=1.0)
    route = generate_lawnmower_route(config)
    assert min(point.x for point in route) == pytest.approx(-9.0)
    assert max(point.x for point in route) == pytest.approx(9.0)
    assert min(point.y for point in route) == pytest.approx(-6.0)
    assert max(point.y for point in route) == pytest.approx(6.0)
    assert route_length(route) > 0.0
    assert all(point.z == config.altitude for point in route)


def test_route_alternates_direction_and_respects_spacing() -> None:
    route = generate_lawnmower_route(RouteConfig(strip_spacing=2.0))
    starts = route[::2]
    assert all(abs(next_point.y - point.y) <= 2.0 + 1e-9 for point, next_point in zip(starts, starts[1:]))
    assert [point.x for point in starts[:4]] == [-9.0, 9.0, -9.0, 9.0]


@pytest.mark.parametrize("field,value", [("speed", 0.0), ("altitude", -1.0), ("edge_margin", 7.0)])
def test_invalid_route_parameters_are_rejected(field: str, value: float) -> None:
    values = {"speed": 2.0, "altitude": 8.0, "edge_margin": 1.0}
    values[field] = value
    with pytest.raises(ValueError):
        generate_lawnmower_route(RouteConfig(**values))
