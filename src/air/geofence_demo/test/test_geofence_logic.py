"""地理围栏多边形检测算法单元测试."""

import pytest

from geofence_demo.geofence_node import is_point_in_polygon


def test_normal_polygon():
    """矩形围栏内点返回 True，外点返回 False."""
    polygon = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
    assert is_point_in_polygon(0.0, 0.0, polygon) is True
    assert is_point_in_polygon(2.0, 2.0, polygon) is False


def test_edge_cases():
    """空多边形、单点、两点退化情况统一返回 False."""
    assert is_point_in_polygon(0.0, 0.0, []) is False
    assert is_point_in_polygon(0.0, 0.0, [(0.0, 0.0)]) is False
    assert is_point_in_polygon(0.0, 0.0, [(0.0, 0.0), (1.0, 1.0)]) is False


def test_point_on_boundary():
    """落在边界上的点被视为 False（不在围栏内部）."""
    polygon = [(0.0, 0.0), (0.0, 2.0), (2.0, 2.0), (2.0, 0.0)]
    assert is_point_in_polygon(0.0, 1.0, polygon) is False
    # 顶点也视为 False
    assert is_point_in_polygon(0.0, 0.0, polygon) is False


def test_inside_critical_rectangle():
    """PR 主用例：矩形 (2,2)-(4,4) 的内部 (3,3) 必须返回 True."""
    polygon = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    assert is_point_in_polygon(3.0, 3.0, polygon) is True
    assert is_point_in_polygon(2.5, 2.5, polygon) is True


def test_same_rectangle_same_answer():
    """同一条矩形围栏不能因遍历方向/起点不同而出现 Self-contradiction."""
    poly = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    # 内部点
    assert is_point_in_polygon(3.0, 3.0, poly) is True
    # 外部点 (2.5, 2.5) 也是内部
    assert is_point_in_polygon(2.5, 2.5, poly) is True
    # 明显外部
    assert is_point_in_polygon(1.0, 1.0, poly) is False
    assert is_point_in_polygon(5.0, 5.0, poly) is False


def test_clockwise_polygon():
    """顺时针顶点顺序的矩形应与逆时针结果一致."""
    poly_cw = [(2.0, 2.0), (2.0, 4.0), (4.0, 4.0), (4.0, 2.0)]
    assert is_point_in_polygon(3.0, 3.0, poly_cw) is True
    assert is_point_in_polygon(1.0, 1.0, poly_cw) is False
