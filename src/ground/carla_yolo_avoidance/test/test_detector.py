# -*- coding: utf-8 -*-
"""检测结果整理的单元测试（不需要 ROS，也不需要真的跑 YOLO）。

运行方式（两种都行）：
    pytest src/carla_yolo_avoidance/test/test_detector.py
    python src/carla_yolo_avoidance/test/test_detector.py
"""

import os
import sys

_PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if _PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, _PACKAGE_ROOT)

from carla_yolo_avoidance import detector, distance_estimator as de  # noqa: E402

# COCO 类别名（只列出测试用到的）
NAMES = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}
FOCAL = de.default_focal_px()          # 800 px / 90° -> 400 px
IMAGE_WIDTH = 800


def test_is_vehicle_by_class_id():
    assert detector.is_vehicle(2, 'car') is True
    assert detector.is_vehicle(5, 'bus') is True
    assert detector.is_vehicle(0, 'person') is False
    assert detector.is_vehicle(1, 'bicycle') is False


def test_is_vehicle_by_name_fallback():
    """类别 id 对不上时用名字兜底（换权重也能工作）。"""
    assert detector.is_vehicle(999, 'truck') is True
    assert detector.is_vehicle(999, 'Motorcycle') is True
    assert detector.is_vehicle(999, 'traffic light') is False


def test_build_detections_filters_non_vehicle():
    """行人、自行车不应进入结果（作业只关心车辆）。"""
    boxes = [[100, 100, 200, 200], [300, 300, 340, 380]]
    confidences = [0.9, 0.8]
    class_ids = [0, 1]                       # person, bicycle
    result = detector.build_detections(boxes, confidences, class_ids, NAMES,
                                       IMAGE_WIDTH, FOCAL)
    assert result == []


def test_build_detections_fields_and_distance():
    """字段齐全，且距离用 框宽 * 焦距 / 真实宽度 算出。"""
    boxes = [[300, 250, 400, 330]]           # 宽 100 px
    result = detector.build_detections(boxes, [0.87], [2], NAMES, IMAGE_WIDTH, FOCAL)
    assert len(result) == 1
    det = result[0]
    assert det['class'] == 'car'
    assert det['class_id'] == 2
    assert det['confidence'] == 0.87
    assert det['bbox'] == [300.0, 250.0, 400.0, 330.0]
    assert det['width_px'] == 100.0
    # 400 * 1.8 / 100 = 7.2 m
    assert abs(det['distance_m'] - 7.2) < 1e-6
    # 框中心 x=350，比画面中心 400 靠左 -> 横向偏移为负
    assert det['lateral_m'] < 0


def test_build_detections_sorted_by_distance():
    """结果按距离从近到远排序，方便下游直接取第一个。"""
    boxes = [[300, 250, 340, 330],      # 宽 40 px -> 远
             [300, 250, 500, 400]]      # 宽 200 px -> 近
    result = detector.build_detections(boxes, [0.9, 0.9], [2, 7], NAMES,
                                       IMAGE_WIDTH, FOCAL)
    assert [d['class'] for d in result] == ['truck', 'car']
    assert result[0]['distance_m'] < result[1]['distance_m']


def test_build_detections_none_distance_sorted_last():
    """距离不可信的框排在最后，但仍然保留（可视化时能看到）。"""
    boxes = [[400, 400, 402, 402],      # 宽 2 px -> 距离 None
             [300, 250, 400, 330]]      # 宽 100 px -> 7.2 m
    result = detector.build_detections(boxes, [0.5, 0.9], [2, 2], NAMES,
                                       IMAGE_WIDTH, FOCAL)
    assert len(result) == 2
    assert result[0]['distance_m'] is not None
    assert result[1]['distance_m'] is None


def test_build_detections_max_detections():
    """max_detections 用于限制输出数量，避免一帧里几百个框拖慢下游。"""
    boxes = [[i * 10, 0, i * 10 + 50, 50] for i in range(1, 11)]
    result = detector.build_detections(
        boxes, [0.9] * 10, [2] * 10, NAMES, IMAGE_WIDTH, FOCAL, max_detections=3
    )
    assert len(result) == 3


def test_build_detections_empty_input():
    assert detector.build_detections([], [], [], NAMES, IMAGE_WIDTH, FOCAL) == []


def test_build_detections_names_without_get():
    """names 不是字典时也不能崩（退化用类别 id 当名字）。"""
    result = detector.build_detections([[300, 250, 400, 330]], [0.9], [2], None,
                                       IMAGE_WIDTH, FOCAL)
    assert len(result) == 1
    assert result[0]['class'] == '2'


def test_summarize():
    detections = [{'distance_m': 12.0}, {'distance_m': None}]
    nearest, count = detector.summarize(detections)
    assert nearest == 12.0
    assert count == 2
    assert detector.summarize([]) == (None, 0)


if __name__ == '__main__':
    failures = 0
    total = 0
    for _name in sorted(list(globals())):
        if _name.startswith('test_') and callable(globals()[_name]):
            total += 1
            try:
                globals()[_name]()
                print('PASS {}'.format(_name))
            except AssertionError as exc:
                failures += 1
                print('FAIL {} -> {}'.format(_name, exc))
    print('---- 通过 {} 个，失败 {} 个 ----'.format(total - failures, failures))
    sys.exit(1 if failures else 0)
