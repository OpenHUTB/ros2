"""A* 的离线单元测试：不依赖 ROS / 仿真器。

运行：
    python3 -m pytest src/air/uav_depth_mapping_nav/test/test_astar.py -q
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from uav_depth_mapping_nav import astar as astar_lib  # noqa: E402

SIZE = 120


def _empty():
    return np.zeros((SIZE, SIZE), dtype=bool)


def test_straight_line_on_empty_map():
    """空地上两点直连：路径是直线，格数等于曼哈顿距离 + 1。"""
    blocked = _empty()
    r = astar_lib.astar(blocked, (10, 20), (10, 70))
    assert r['success'] is True
    # 起点到终点差 50 格，路径含首尾共 51 个节点
    assert len(r['path']) == 51
    # 格坐标的第一个分量保持不变，只有第二个分量在变
    assert {cell[0] for cell in r['path']} == {10}


def test_detour_around_wall():
    """一堵留缺口的竖墙：必须绕行，路径明显长于直线。"""
    blocked = _empty()
    blocked[:90, 60:65] = True           # 墙挡 y<90，y>=90 处可绕
    detour = astar_lib.astar(blocked, (30, 50), (90, 50), allow_unknown=False)
    direct = astar_lib.astar(_empty(), (30, 50), (90, 50))
    assert detour['success'] is True
    # 绕行应比直连长至少 20%
    assert detour['length_cells'] > direct['length_cells'] * 1.2


def test_no_path_when_wall_spans_map():
    """墙贯穿整幅地图：无解，reason 必须是 no_path。"""
    blocked = _empty()
    blocked[:, 60:65] = True
    r = astar_lib.astar(blocked, (10, 50), (110, 50))
    assert r['success'] is False
    assert r['reason'] == 'no_path'


def test_max_expansions_guard():
    """扩展次数上限：把 max_expansions 压到很小，必须报 max_expansions 而不是死循环。"""
    blocked = _empty()
    blocked[::7, :] = True                # 稠密条纹，逼迫大量回溯
    r = astar_lib.astar(blocked, (2, 2), (117, 117), max_expansions=50)
    assert r['success'] is False
    assert r['reason'] == 'max_expansions'
    assert 50 <= r['expansions'] <= 51   # 计数在循环尾部自增，允许差 1


def test_inflate_shrinks_free_space():
    """膨胀：一个障碍点会把周围 radius_cells 圈都变成占据。"""
    occupied = _empty()
    occupied[60, 60] = True
    inflated = astar_lib.inflate(occupied, resolution=0.25, inflation_radius=0.5)
    assert inflated[60, 60]
    assert inflated[60 + 2, 60]           # 0.5 m / 0.25 m = 2 格
    assert not inflated[60 + 3, 60]
