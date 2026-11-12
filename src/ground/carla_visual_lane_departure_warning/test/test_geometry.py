"""几何层离线测试：内参、投影、横向偏移口径。

不需要 CARLA，也不需要 numpy 之外的东西。
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from carla_visual_lane_departure_warning import geometry as geo

CASES = []


def case(func):
    CASES.append(func)
    return func


@case
def intrinsics_from_fov():
    """内参按水平视场角计算。"""
    k = geo.intrinsics(800, 600, 90.0)
    assert abs(k[0, 0] - 400.0) < 1e-9, 'fov 90 度、宽 800 时焦距应为 400'
    assert abs(k[0, 2] - 400.0) < 1e-9
    assert abs(k[1, 2] - 300.0) < 1e-9


@case
def narrow_fov_longer_focal():
    """视场角变小焦距变大。"""
    narrow = geo.intrinsics(800, 600, 60.0)
    wide = geo.intrinsics(800, 600, 120.0)
    assert narrow[0, 0] > wide[0, 0], '视场角越小焦距越大'


@case
def point_ahead_lands_in_centre():
    """正前方的点落在画面中心。"""
    k = geo.intrinsics(800, 600, 90.0)
    uv, depth, inside = geo.project(np.array([[10.0, 0.0, 0.0]]), np.eye(4), k)
    assert abs(uv[0, 0] - 400.0) < 1e-6
    assert abs(uv[0, 1] - 300.0) < 1e-6
    assert abs(depth[0] - 10.0) < 1e-9
    assert inside[0]


@case
def point_right_of_centre():
    """右侧的点落在中心线右边。"""
    k = geo.intrinsics(800, 600, 90.0)
    uv, _, inside = geo.project(np.array([[10.0, 1.8, 0.0]]), np.eye(4), k)
    assert inside[0]
    assert uv[0, 0] > 400.0, '右偏的点 u 应大于中心'


@case
def point_above_centre():
    """上方的点落在中心线上边。"""
    k = geo.intrinsics(800, 600, 90.0)
    uv, _, _ = geo.project(np.array([[10.0, 0.0, 1.0]]), np.eye(4), k)
    assert uv[0, 1] < 300.0, '图像 v 向下，上方点的 v 应更小'


@case
def point_behind_camera_invalid():
    """相机身后的点无效。"""
    k = geo.intrinsics(800, 600, 90.0)
    _, depth, inside = geo.project(np.array([[-5.0, 0.0, 0.0]]), np.eye(4), k)
    assert depth[0] < 0
    assert not inside[0]


@case
def camera_translation_counted():
    """相机位姿平移会被算进去。"""
    k = geo.intrinsics(800, 600, 90.0)
    pose = np.eye(4)
    pose[0, 3] = 10.0                     # 相机在 x=10
    uv, depth, inside = geo.project(np.array([[20.0, 0.0, 0.0]]), pose, k)
    assert abs(depth[0] - 10.0) < 1e-9, '距离应为 20-10'
    assert abs(uv[0, 0] - 400.0) < 1e-6


@case
def camera_rotated_90_degrees():
    """相机绕z轴旋转九十度。"""
    k = geo.intrinsics(800, 600, 90.0)
    pose = np.eye(4)
    angle = math.pi / 2
    pose[:3, :3] = np.array([[math.cos(angle), -math.sin(angle), 0],
                             [math.sin(angle), math.cos(angle), 0],
                             [0, 0, 1]])
    # 相机朝 +y，世界里的 (0, 10, 0) 应落在画面中心
    uv, _, inside = geo.project(np.array([[0.0, 10.0, 0.0]]), pose, k)
    assert inside[0]
    assert abs(uv[0, 0] - 400.0) < 1e-6


@case
def lateral_offset_sign():
    """横向偏移左正右负。"""
    right_of_centre = geo.lateral_offset((0.0, -0.5), 0.0, (0.0, 0.0), 0.0)
    left_of_centre = geo.lateral_offset((0.0, 0.5), 0.0, (0.0, 0.0), 0.0)
    assert right_of_centre < 0, '车在中心线右侧时偏移应为负'
    assert left_of_centre > 0, '车在中心线左侧时偏移应为正'
    assert abs(geo.lateral_offset((0, 0), 0.0, (0, 0), 0.0)) < 1e-12


@case
def lateral_offset_matches_road_yaw():
    """横向偏移与道路朝向无关。"""
    yaw = math.radians(37.0)
    centre = (3.0, 4.0)
    point = (3.0 - math.sin(yaw) * 0.8, 4.0 + math.cos(yaw) * 0.8)
    assert abs(geo.lateral_offset(point, 0.0, centre, yaw) - 0.8) < 1e-9


@case
def wrap_angle_to_pi():
    """角度折叠到正负pi。"""
    # 正负 pi 等价，只要求落在边界上
    assert abs(abs(geo.wrap_angle(3 * math.pi)) - math.pi) < 1e-9
    assert abs(abs(geo.wrap_angle(-3 * math.pi)) - math.pi) < 1e-9
    assert abs(geo.wrap_angle(0.5) - 0.5) < 1e-9


def main():
    failures = 0
    for func in CASES:
        try:
            func()
            print('[通过] %s' % func.__name__)
        except AssertionError as error:
            failures += 1
            print('[失败] %s：%s' % (func.__name__, error))
    print('=' * 60)
    if failures:
        print('[结果] %d 项失败，%d 项通过' % (failures, len(CASES) - failures))
        return 1
    print('[结果] %d 项全部通过' % len(CASES))
    return 0


if __name__ == '__main__':
    sys.exit(main())