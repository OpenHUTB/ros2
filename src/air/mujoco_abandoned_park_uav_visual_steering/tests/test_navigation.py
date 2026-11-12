from pathlib import Path
import sys

MODULE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(MODULE_ROOT))

from navigation import OccupancyGrid, astar, keyboard_command


def test_sensor_ray_updates_map_and_astar_avoids_hit():
    grid = OccupancyGrid(8, 5)
    grid.mark_sensor_ray((1, 2), [(3, 2)])
    path = astar(grid, (1, 2), (6, 2))
    assert (3, 2) in grid.occupied
    assert path and (3, 2) not in path


def test_keyboard_commands_are_deterministic():
    assert keyboard_command("w") == (0.8, 0.0)
    assert keyboard_command("A") == (0.4, -1.0)
    assert keyboard_command("x") == (0.0, 0.0)
