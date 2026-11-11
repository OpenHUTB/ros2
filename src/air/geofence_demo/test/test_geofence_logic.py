# Copyright 2026 Toeting
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""地理围栏多边形检测算法单元测试."""

from geofence_demo.geofence_node import _point_on_segment
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
    """起点旋转或遍历方向反转后，同一点的判定结果必须不变."""
    ccw = [(2.0, 2.0), (4.0, 2.0), (4.0, 4.0), (2.0, 4.0)]
    poly_cw = [(2.0, 2.0), (2.0, 4.0), (4.0, 4.0), (4.0, 2.0)]
    variants = [
        ccw,
        ccw[1:] + ccw[:1],  # 起点后移一位
        ccw[2:] + ccw[:2],
        poly_cw,            # 顺时针
        poly_cw[3:] + poly_cw[:3],
    ]
    for polygon in variants:
        assert is_point_in_polygon(3.0, 3.0, polygon) is True
        assert is_point_in_polygon(2.5, 2.5, polygon) is True
        assert is_point_in_polygon(1.0, 1.0, polygon) is False
        assert is_point_in_polygon(5.0, 5.0, polygon) is False


def test_duplicate_vertices_do_not_break_detection():
    """重复顶点（零长度边）不能让整个围栏失效、恒返回 SAFE."""
    polygon = [
        (2.0, 2.0), (4.0, 2.0), (4.0, 2.0),
        (4.0, 4.0), (4.0, 4.0), (2.0, 4.0),
    ]
    assert is_point_in_polygon(3.0, 3.0, polygon) is True
    assert is_point_in_polygon(1.0, 1.0, polygon) is False


def test_degenerate_edge_is_not_boundary():
    """零长度边不构成有效边界，任意点都不应被判为落在其上."""
    assert _point_on_segment(3.0, 3.0, 4.0, 2.0, 4.0, 2.0) is False
    assert _point_on_segment(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) is False
