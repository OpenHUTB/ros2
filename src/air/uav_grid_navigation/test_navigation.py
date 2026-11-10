import pytest

from navigation import plan_route, simulate_route


def test_astar_finds_shortest_route_around_obstacle():
    grid = [
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0],
    ]

    route = plan_route(grid, (0, 0), (3, 2))

    assert route[0] == (0, 0)
    assert route[-1] == (3, 2)
    assert len(route) - 1 == 5
    assert all(grid[y][x] == 0 for x, y in route)


def test_blocked_goal_returns_no_route():
    assert plan_route([[0, 1]], (0, 0), (1, 0)) == []


def test_invalid_grid_shape_is_rejected():
    with pytest.raises(ValueError, match="equal width"):
        plan_route([[0, 0], [0]], (0, 0), (1, 0))


def test_route_metrics_report_discrete_goal_reach():
    result = simulate_route([[0, 0], [0, 0]], (0, 0), (1, 1))

    assert result["goal_reached"] is True
    assert result["step_count"] == 2
    assert result["final_cell"] == [1, 1]
