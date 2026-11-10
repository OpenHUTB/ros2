# -*- coding: utf-8 -*-
"""单目测距：用 YOLO 检测框的大小估算目标距离。

原理（针孔相机模型）
--------------------
1) 由相机水平视场角 fov 和图像宽度 W 求焦距（像素单位）：
       f_px = (W / 2) / tan(fov / 2)
2) 由相似三角形，物体成像宽度 w_px 与其真实宽度 W_real 满足：
       w_px / f_px = W_real / distance
   因此：
       distance = f_px * W_real / w_px

也就是说：**同一个物体，框越大说明离得越近**，这正好满足作业
"利用 bounding box 的大小估算目标距离"的要求。

本模块是纯数学计算，不导入 rclpy / numpy，便于单元测试。
"""

import math

from . import config


def focal_length_px(image_width_px, hfov_deg):
    """由图像宽度和水平视场角计算等效焦距（单位：像素）。"""
    if image_width_px <= 0:
        raise ValueError('image_width_px 必须为正数，收到 {}'.format(image_width_px))
    if not 0.0 < hfov_deg < 180.0:
        raise ValueError('hfov_deg 必须在 (0, 180) 之间，收到 {}'.format(hfov_deg))
    return (float(image_width_px) / 2.0) / math.tan(math.radians(float(hfov_deg)) / 2.0)


def real_width_of_class(class_id, class_name=''):
    """取该类目标的真实宽度（米）。未知类别用默认值。"""
    if class_id in config.CLASS_REAL_WIDTH_M:
        return float(config.CLASS_REAL_WIDTH_M[class_id])
    # 类别 id 对不上时，用类别名兜底
    name = (class_name or '').lower()
    if 'bus' in name:
        return float(config.CLASS_REAL_WIDTH_M[5])
    if 'truck' in name:
        return float(config.CLASS_REAL_WIDTH_M[7])
    if 'motor' in name or 'bike' in name:
        return float(config.CLASS_REAL_WIDTH_M[3])
    return float(config.DEFAULT_REAL_WIDTH_M)


def estimate_distance_m(bbox_width_px, class_id, class_name='', focal_px=None,
                        image_width_px=None, hfov_deg=None):
    """估算目标距离（米）。

    参数:
        bbox_width_px: 检测框的像素宽度
        class_id:      COCO 类别 id
        class_name:    类别名（用于兜底判断真实宽度）
        focal_px:      焦距（像素）。为 None 时用 image_width_px + hfov_deg 计算
        image_width_px/hfov_deg: focal_px 为 None 时必须提供
    返回:
        float 距离（米）；框太小时返回 None 表示"距离不可信"
    """
    if bbox_width_px is None or float(bbox_width_px) < config.MIN_VALID_BBOX_WIDTH_PX:
        return None

    if focal_px is None:
        if image_width_px is None or hfov_deg is None:
            raise ValueError('focal_px 为 None 时必须提供 image_width_px 与 hfov_deg')
        focal_px = focal_length_px(image_width_px, hfov_deg)

    real_width = real_width_of_class(class_id, class_name)
    distance = float(focal_px) * real_width / float(bbox_width_px)

    if distance > config.MAX_VALID_DISTANCE_M:
        return None
    if distance < config.MIN_VALID_DISTANCE_M:
        return config.MIN_VALID_DISTANCE_M
    return distance


def lateral_offset_m(bbox_center_x_px, distance_m, focal_px, image_width_px):
    """估算目标的横向偏移（米）：正数在右，负数在左。

    用像平面上的横向像素偏差做同样的相似三角形换算。
    """
    offset_px = float(bbox_center_x_px) - float(image_width_px) / 2.0
    return offset_px * float(distance_m) / float(focal_px)


def bbox_width_px(x_min, x_max):
    """检测框像素宽度（保证非负）。"""
    return max(0.0, float(x_max) - float(x_min))


def default_focal_px():
    """按 config 里的相机参数计算默认焦距。"""
    return focal_length_px(config.CAMERA_IMAGE_WIDTH, config.CAMERA_FOV_DEG)
