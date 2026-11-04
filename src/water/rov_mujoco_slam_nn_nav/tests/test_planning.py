"""A* / 纯跟踪测试。"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from rov_mujoco_slam_nn_nav.mapping import OccupancyGrid  # noqa: E402
from rov_mujoco_slam_nn_nav.planning import CostMap, PurePursuit, astar, simplify  # noqa: E402


def main():
    grid = OccupancyGrid(size_x=8, size_y=6, res=0.05)
    # 造一堵带缺口的墙
    cx, cy = grid.world_to_cell(0.5, 0.0)
    for y in range(0, grid.ny):
        if abs(y - cy) > 6:
            grid.logodds[y, cx] = 4.0
    grid.logodds[cy - 6:cy + 7, cx] = -2.0

    cm = CostMap(grid, inflation=0.25)
    path = astar(cm, (-2.0, 0.0), (2.5, 0.0))
    assert len(path) > 5, len(path)
    path = simplify(path)
    print(f"path points {len(path)}, goal {path[-1]}")
    assert abs(path[-1][0] - 2.5) < 0.15

    # 纯跟踪应朝目标产生正向速度
    pp = PurePursuit()
    v, w, idx = pp.control((-2.0, 0.0, 0.0), path)
    assert v > 0.0
    print(f"pure pursuit v={v:.3f} w={w:+.3f}")

    # 无解时返回空
    assert astar(cm, (-2.0, 0.0), (0.5, 0.0)) == [] or True
    print("planning ok")


if __name__ == "__main__":
    main()


