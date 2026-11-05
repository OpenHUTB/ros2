from mujoco_uav_moving_pad_landing.navigation import (
    GridWorld,
    OccupancyGrid,
    astar,
    run_slam_navigation,
)


def test_lidar_map_supports_safe_astar_route():
    world = GridWorld()
    grid = OccupancyGrid(world.width, world.height)
    grid.update_scan(world, world.start)
    route = astar(grid, world.start, world.goal)

    assert route[0] == world.start
    assert route[-1] == world.goal
    assert all(grid.data[y][x] != 100 for x, y in route[1:-1])


def test_navigation_updates_map_until_goal_and_saves_media(tmp_path):
    result = run_slam_navigation(tmp_path)

    assert result["goal_reached"] is True
    assert result["navigation_steps"] > 0
    assert 0.0 < result["map_coverage"] <= 1.0
    assert (tmp_path / "navigation_map.png").is_file()
    assert (tmp_path / "navigation_run.gif").is_file()
