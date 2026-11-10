# -*- coding: utf-8 -*-
"""单目测距的单元测试（纯数学，不需要 ROS 环境）。

运行方式（两种都行）：
    pytest src/carla_yolo_avoidance/test/test_distance_estimator.py
    python src/carla_yolo_avoidance/test/test_distance_estimator.py
"""

import math
import os
import sys

# 让脚本在未 colcon build 的情况下也能直接运行
_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from carla_yolo_avoidance import config, distance_estimator as de  # noqa: E402


def test_focal_length_default():
    """800 像素宽、90° 视场角时，焦距应为 400 像素。"""
    assert abs(de.focal_length_px(800, 90.0) - 400.0) < 1e-6
    # 默认参数应与 config 一致
    assert abs(de.default_focal_px() - 400.0) < 1e-6


def test_focal_length_narrow_fov():
    """视场角越小，焦距越长。"""
    assert de.focal_length_px(800, 60.0) > de.focal_length_px(800, 90.0)


def test_distance_from_bbox_width():
    """已知焦距时，距离 = f * 真实宽度 / 像素宽度。"""
    # car(1.8 m) 在图中占 100 像素、f=400 -> 7.2 m
    assert abs(de.estimate_distance_m(100.0, 2, 'car', focal_px=400.0) - 7.2) < 1e-6


def test_distance_shrinks_when_box_grows():
    """框越大 -> 距离越近（避障逻辑的基础）。"""
    near = de.estimate_distance_m(200.0, 2, 'car', focal_px=400.0)
    far = de.estimate_distance_m(50.0, 2, 'car', focal_px=400.0)
    assert near is not None and far is not None
    assert near < far


def test_distance_returns_none_for_tiny_box():
    """框太小时距离不可信，应返回 None。"""
    assert de.estimate_distance_m(2.0, 2, 'car', focal_px=400.0) is None
    assert de.estimate_distance_m(0.0, 2, 'car', focal_px=400.0) is None


def test_distance_returns_none_when_out_of_range():
    """超过最大有效距离时返回 None（框小到几像素，误差不可接受）。"""
    width_px = 4.0  # 恰好等于最小有效宽度
    assert de.estimate_distance_m(width_px, 2, 'car', focal_px=400.0) is None


def test_near_face_returns_min_not_none():
    """贴脸（距离 < MIN_VALID_DISTANCE_M）不应返回 None，应返回最小值触发急刹。"""
    # f=400, car 宽 1.8m, 框宽 800px -> 0.9m < 1.0m
    result = de.estimate_distance_m(800.0, 2, 'car', focal_px=400.0)
    assert result == config.MIN_VALID_DISTANCE_M


def test_distance_with_image_params_instead_of_focal():
    """不传 focal_px 时，应能用图像宽度 + 视场角算出同样的结果。"""
    a = de.estimate_distance_m(80.0, 2, 'car', focal_px=400.0)
    b = de.estimate_distance_m(80.0, 2, 'car', image_width_px=800, hfov_deg=90.0)
    assert a is not None and b is not None
    assert abs(a - b) < 1e-9


def test_real_width_of_known_and_unknown_class():
    """已知类别取配置值，未知类别回退到默认宽度。"""
    assert de.real_width_of_class(2, 'car') == config.CLASS_REAL_WIDTH_M[2]
    assert de.real_width_of_class(5, 'bus') == config.CLASS_REAL_WIDTH_M[5]
    assert de.real_width_of_class(999, 'unknown') == config.DEFAULT_REAL_WIDTH_M
    # 类别 id 不认识但名字里有 bus 时，也应识别为公交车宽度
    assert de.real_width_of_class(999, 'bus') == config.CLASS_REAL_WIDTH_M[5]


def test_lateral_offset_sign_and_value():
    """目标在画面右侧 -> 横向偏移为正。"""
    # 中心 500 像素、图像宽 800 -> 偏右 100 像素
    offset = de.lateral_offset_m(500.0, 7.2, 400.0, 800)
    assert abs(offset - 1.8) < 1e-6
    # 正中间 -> 0
    assert abs(de.lateral_offset_m(400.0, 10.0, 400.0, 800)) < 1e-9
    # 左侧 -> 负
    assert de.lateral_offset_m(300.0, 10.0, 400.0, 800) < 0


def test_bbox_width_px_is_non_negative():
    assert abs(de.bbox_width_px(10.0, 60.0) - 50.0) < 1e-9
    assert de.bbox_width_px(60.0, 10.0) == 0.0


def test_invalid_inputs_raise():
    """非法输入应明确报错，而不是静默算出错误距离。"""
    for bad in (0, -10):
        try:
            de.focal_length_px(bad, 90.0)
        except ValueError:
            pass
        else:
            raise AssertionError('image_width_px={} 应抛 ValueError'.format(bad))
    try:
        de.estimate_distance_m(10.0, 2, 'car')  # 既没给焦距也没给图像参数
    except ValueError:
        pass
    else:
        raise AssertionError('缺少焦距参数时应抛 ValueError')


def test_math_sanity_against_atan():
    """用反三角函数反向验证：由距离推出的视角应等于几何预期。"""
    # 1.8 m 宽的车在 30 m 处，在 f=400 px 的图里大约占 24 像素
    focal = de.focal_length_px(800, 90.0)
    estimated = de.estimate_distance_m(24.0, 2, 'car', focal_px=focal)
    assert estimated is not None
    assert abs(estimated - 30.0) < 1e-6

    # 两种算法算出的张角应当一致：atan2(半宽, 距离) 对像素与真实尺度都成立
    angle_from_real = math.degrees(math.atan2(1.8 / 2.0, estimated))
    angle_from_pixel = math.degrees(math.atan2(24.0 / 2.0, focal))
    assert abs(angle_from_real - angle_from_pixel) < 1e-6


if __name__ == '__main__':
    failures = 0
    for _name in sorted(list(globals())):
        if _name.startswith('test_') and callable(globals()[_name]):
            try:
                globals()[_name]()
                print('PASS {}'.format(_name))
            except AssertionError as exc:
                failures += 1
                print('FAIL {} -> {}'.format(_name, exc))
    print('---- 通过 {} 个，失败 {} 个 ----'.format(
        sum(1 for n in globals() if n.startswith('test_')) - failures, failures))
    sys.exit(1 if failures else 0)
