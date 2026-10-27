#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""图像预处理：从相机帧中提取手写数字并归一化为 MNIST 风格的 28x28 输入。

处理流程：
    1. 灰度化；
    2. 自动判断前景极性（数字比背景暗还是亮），用 Otsu 自适应阈值二值化；
    3. 取最大连通域的外接矩形，四周扩边 10%；
    4. 反色为“白字黑底”，补成正方形，缩放到 20x20；
    5. 居中粘贴到 28x28 黑色画布上（与 MNIST 一致）。
"""
from typing import Optional, Tuple

import cv2
import numpy as np

TARGET = 28
INNER = 20


def extract_digit(frame_gray: np.ndarray, min_area: int = 60) -> Tuple[Optional[np.ndarray], Optional[Tuple[int, int, int, int]]]:
    """从灰度帧中提取数字的 28x28 归一化图。

    返回 (digit28, box)。若无有效前景则返回 (None, None)。
    box 为原图坐标 (x, y, w, h)。
    """
    h, w = frame_gray.shape
    # 用边缘一圈估计背景亮度，中心区域估计前景亮度，判断极性
    border = np.concatenate([
        frame_gray[0:5, :].ravel(), frame_gray[-5:, :].ravel(),
        frame_gray[:, 0:5].ravel(), frame_gray[:, -5:].ravel(),
    ])
    bg_mean = float(border.mean())

    flags = cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU if bg_mean > 127 else cv2.THRESH_BINARY + cv2.THRESH_OTSU
    _, binary = cv2.threshold(frame_gray, 0, 255, flags)

    # 轻微开运算去除噪点
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None

    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None, None

    x, y, bw, bh = cv2.boundingRect(largest)
    # 扩边 10%，同时夹在图像范围内
    mx, my = int(bw * 0.1) + 2, int(bh * 0.1) + 2
    x0, y0 = max(0, x - mx), max(0, y - my)
    x1, y1 = min(w, x + bw + mx), min(h, y + bh + my)
    crop = frame_gray[y0:y1, x0:x1]

    digit28 = _normalize_mnist(crop)
    return digit28, (x0, y0, x1 - x0, y1 - y0)


def _normalize_mnist(crop: np.ndarray) -> np.ndarray:
    """把数字裁剪块归一化为 28x28（20x20 居中，白字黑底）。"""
    # 灰度裁剪块统一为“白字黑底”：若亮色像素占多数说明数字比背景暗，需反转
    if (crop > 127).mean() > 0.5:
        crop = 255 - crop

    hh, ww = crop.shape
    side = max(hh, ww)
    square = np.zeros((side, side), dtype=np.uint8)
    square[(side - hh) // 2:(side - hh) // 2 + hh, (side - ww) // 2:(side - ww) // 2 + ww] = crop

    inner = cv2.resize(square, (INNER, INNER), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((TARGET, TARGET), dtype=np.uint8)
    off = (TARGET - INNER) // 2
    canvas[off:off + INNER, off:off + INNER] = inner
    return canvas
