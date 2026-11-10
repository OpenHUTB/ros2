import pytest

from navigation import plan_route, render_route, simulate_route
from simulation import run_mujoco_navigation


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


def test_route_renderer_writes_a_png(tmp_path):
    from PIL import Image

    output = tmp_path / "map.png"
    render_route([[0, 0], [0, 0]], [(0, 0), (1, 1)], (0, 0), (1, 1), output, cell_size=24)

    with Image.open(output) as image:
        assert image.format == "PNG"
        assert image.size == (84, 116)


def test_mujoco_navigation_maps_and_reaches_goal(tmp_path):
    result = run_mujoco_navigation(tmp_path)

    assert result["goal_reached"] is True
    assert result["navigation_steps"] > 0
    assert result["lidar_scans"] > 1
    assert result["replans"] > 0
    assert result["map_coverage"] == 1.0
    assert result["final_x_cell"] == 21
    assert result["final_y_cell"] == 13
    assert (tmp_path / "navigation_map.png").is_file()
    assert (tmp_path / "navigation_run.gif").is_file()
    assert (tmp_path / "trajectory.json").is_file()
    assert (tmp_path / "navigation_metrics.json").is_file()
