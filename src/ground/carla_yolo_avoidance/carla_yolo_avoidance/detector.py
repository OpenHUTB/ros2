# -*- coding: utf-8 -*-
"""把 YOLOv8 的原始输出整理成项目统一的检测结果结构。

抽成独立模块的目的：
    1. ROS 节点（yolo_detect_node.py）与无 ROS 的冒烟测试
       （tools/carla_yolo_smoke_test.py）共用同一份后处理逻辑，
       保证"测过的就是跑着的"；
    2. 本模块不导入 rclpy / ultralytics / numpy，可以被单元测试直接覆盖。

检测结果字典的字段（同时也是发布到 ROS 话题的 JSON 结构）：
    class_id / class / confidence / bbox[x1,y1,x2,y2] / width_px / distance_m / lateral_m
"""

from . import config, distance_estimator


def is_vehicle(class_id, class_name=''):
    """判断类别是否属于"车辆"。

    优先用 COCO 类别 id 判断；如果换了别的权重导致 id 对不上，
    再用类别名关键词兜底。
    """
    if int(class_id) in config.VEHICLE_CLASS_IDS:
        return True
    name = (class_name or '').lower()
    return any(keyword in name for keyword in config.VEHICLE_CLASS_KEYWORDS)


def build_detections(boxes_xyxy, confidences, class_ids, names,
                     image_width_px, focal_px, max_detections=None):
    """把 YOLO 的框整理成检测结果列表。

    参数:
        boxes_xyxy:   每行 [x_min, y_min, x_max, y_max]（可索引的序列）
        confidences:  每个框的置信度
        class_ids:    每个框的类别 id
        names:        类别 id -> 类别名 的映射（如 YOLO 的 model.names）
        image_width_px: 图像宽度（像素），用于算横向偏移
        focal_px:     焦距（像素），用于单目测距
        max_detections: 最多保留多少个目标，None 表示不限制
    返回:
        list[dict]，按距离由近到远排序（距离不可信的排在最后）
    """
    detections = []

    for index in range(len(class_ids)):
        class_id = int(class_ids[index])
        if hasattr(names, 'get'):
            class_name = str(names.get(class_id, str(class_id)))
        else:
            class_name = str(class_id)

        if not is_vehicle(class_id, class_name):
            continue

        x_min, y_min, x_max, y_max = (float(v) for v in boxes_xyxy[index][:4])
        width_px = distance_estimator.bbox_width_px(x_min, x_max)
        distance_m = distance_estimator.estimate_distance_m(
            width_px, class_id, class_name, focal_px=focal_px
        )

        lateral_m = None
        if distance_m is not None:
            lateral_m = distance_estimator.lateral_offset_m(
                (x_min + x_max) / 2.0, distance_m, focal_px, image_width_px
            )

        detections.append({
            'class_id': class_id,
            'class': class_name,
            'confidence': round(float(confidences[index]), 4),
            'bbox': [round(x_min, 2), round(y_min, 2), round(x_max, 2), round(y_max, 2)],
            'width_px': round(width_px, 2),
            'distance_m': None if distance_m is None else round(float(distance_m), 2),
            'lateral_m': None if lateral_m is None else round(float(lateral_m), 2),
        })

        if max_detections is not None and len(detections) >= int(max_detections):
            break

    detections.sort(
        key=lambda item: item['distance_m'] if item['distance_m'] is not None else 1e9
    )
    return detections


def summarize(detections):
    """返回 (最近距离, 车辆数量)，供日志与决策使用。"""
    distances = [d['distance_m'] for d in detections if d['distance_m'] is not None]
    return (min(distances) if distances else None), len(detections)
