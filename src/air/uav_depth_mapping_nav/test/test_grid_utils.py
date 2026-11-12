# -*- coding: utf-8 -*-
"""grid_utils 的纯计算单元测试（不需要 ROS / 仿真器）。

运行：python3 -m pytest test/test_grid_utils.py -q
"""

import math

from uav_depth_mapping_nav import grid_utils


def test_world_grid_roundtrip():
    for x, y in ((0.0, 0.0), (3.2, -1.7), (-2.5, 4.05)):
        cell = grid_utils.world_to_grid(x, y, -6.25, -6.25, 0.25)
        back = grid_utils.grid_to_world(cell[0], cell[1], -6.25, -6.25, 0.25)
        assert abs(back[0] - x) <= 0.125 + 1e-9
        assert abs(back[1] - y) <= 0.125 + 1e-9


def test_negative_coordinates_use_floor():
    # 负坐标必须向下取整，否则会偏半格（旧实现用 int() 截断）
    cell = grid_utils.world_to_grid(-0.01, -0.01, 0.0, 0.0, 0.25)
    assert cell == (-1, -1)


def test_bresenham_hits_both_endpoints():
    cells = grid_utils.bresenham(0, 0, 3, 1)
    assert cells[0] == (0, 0)
    assert cells[-1] == (3, 1)


def test_quaternion_identity_keeps_vector():
    import numpy as np

    assert abs(grid_utils.yaw_from_quat(0.0, 0.0, 0.0, 1.0)) < 1e-9
    rot = grid_utils.quat_to_rot_matrix(0.0, 0.0, 0.0, 1.0)
    assert np.allclose(rot, np.eye(3))

    # 绕 z 轴 90 度：机体系 +x 应转到世界系 +y
    s = math.sin(math.pi / 4.0)
    rot = grid_utils.quat_to_rot_matrix(0.0, 0.0, s, s)
    vec = rot.dot(np.array([1.0, 0.0, 0.0]))
    assert abs(vec[0]) < 1e-9 and abs(vec[1] - 1.0) < 1e-9
