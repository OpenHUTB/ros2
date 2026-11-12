# -*- coding: utf-8 -*-
"""限速牌的纯视觉定位与字符切分。

本模块只依赖 NumPy / OpenCV，不导入 ROS 或 PyTorch，便于在普通 Python
环境中对感知前处理做离线回归测试。
"""
import cv2
import numpy as np


def find_sign_roi(bgr):
    """HSV 红色分割定位圆形标志牌，返回 (cx, cy, r) 或 None。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, (0, 90, 60), (8, 255, 255))
    m2 = cv2.inRange(hsv, (172, 90, 60), (180, 255, 255))
    mask = cv2.morphologyEx(m1 | m2, cv2.MORPH_CLOSE,
                            np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 150:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity < 0.6:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        if not (15 <= radius <= 110):
            continue
        if best is None or area > best[3]:
            best = (int(cx), int(cy), int(radius), area)
    if best is None:
        return None
    return best[0], best[1], best[2]


def split_chars(bgr, cx, cy, r):
    """在圆形 ROI 内切分数字字符，返回按 x 排序的灰度图列表。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    height, width = gray.shape
    x0, y0 = max(0, cx - r), max(0, cy - r)
    x1, y1 = min(width, cx + r), min(height, cy + r)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return []
    h, w = roi.shape
    yy, xx = np.mgrid[0:h, 0:w]
    inside = (xx + x0 - cx) ** 2 + (yy + y0 - cy) ** 2 <= (r * 0.92) ** 2
    dark = ((roi < 110) & inside).astype(np.uint8) * 255
    count, _, stats, _ = cv2.connectedComponentsWithStats(dark, 8)
    boxes = []
    for i in range(1, count):
        x, y, box_w, box_h, area = stats[i]
        if area < 20 or not (0.18 * r <= box_h <= 0.95 * r):
            continue
        if box_w > 0.9 * r or box_w < 2 or box_h < 6:
            continue
        boxes.append((x + x0, y + y0, box_w, box_h))
    boxes.sort(key=lambda box: box[0])
    boxes = boxes[:2]
    chars = []
    for x, y, box_w, box_h in boxes:
        pad = max(2, int(0.12 * box_h))
        x0c, y0c = max(0, x - pad), max(0, y - pad)
        x1c, y1c = min(width, x + box_w + pad), min(height, y + box_h + pad)
        chars.append(gray[y0c:y1c, x0c:x1c])
    return chars
