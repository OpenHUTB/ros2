"""Largest-blob HSV target detection for the spiral search mission.

The AirSim bridge publishes raw ``bgr8`` frames; this module keeps the
detection logic free of ROS so it can be unit-tested with synthetic
images. Hue bounds use OpenCV units [0, 180); when ``hue_min`` exceeds
``hue_max`` the range wraps around zero, which is how the default red
range (170-10) covers both sides of the red hue sector.
"""
from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    cx: float              # 归一化横坐标 [-1, 1]，画面中心为 0，向右为正
    cy: float              # 归一化纵坐标 [-1, 1]，向下为正
    area_fraction: float   # 目标轮廓面积 / 画面总面积
    pixel_area: int        # 目标轮廓像素面积


@dataclass(frozen=True)
class Settings:
    hue_min: int = 170             # OpenCV 色相下界 [0, 180)
    hue_max: int = 10              # OpenCV 色相上界；大于下界时按环形区间处理
    sat_min: int = 80              # 饱和度下界 [0, 255]
    val_min: int = 80              # 明度下界 [0, 255]
    min_area_fraction: float = 0.0015   # 目标最小占画面面积比例
    morph_kernel: int = 5          # 形态学开运算核尺寸（像素）

    def __post_init__(self):
        for name in ('hue_min', 'hue_max', 'sat_min', 'val_min', 'morph_kernel'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f'{name} must be an integer')
        if not 0 <= self.hue_min < 180 or not 0 <= self.hue_max < 180:
            raise ValueError('hue bounds must be within [0, 180)')
        if not 0 <= self.sat_min <= 255 or not 0 <= self.val_min <= 255:
            raise ValueError('sat_min and val_min must be within [0, 255]')
        if not math.isfinite(self.min_area_fraction) \
                or not 0 < self.min_area_fraction < 0.5:
            raise ValueError('min_area_fraction must be within (0, 0.5)')
        if not 1 <= self.morph_kernel <= 31:
            raise ValueError('morph_kernel must be within [1, 31]')


def detect_target(frame_bgr, settings=None):
    """Return the largest in-range blob as a Detection, or None."""
    s = settings or Settings()
    if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] < 3:
        return None
    height, width = frame_bgr.shape[:2]
    if height <= 0 or width <= 0:
        return None

    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    hue = hsv[..., 0]
    if s.hue_min <= s.hue_max:
        hue_mask = (hue >= s.hue_min) & (hue <= s.hue_max)
    else:
        hue_mask = (hue >= s.hue_min) | (hue <= s.hue_max)
    mask = (hue_mask
            & (hsv[..., 1] >= s.sat_min)
            & (hsv[..., 2] >= s.val_min)).astype(np.uint8)

    if s.morph_kernel > 1:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (s.morph_kernel, s.morph_kernel))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area <= 0 or area / float(width * height) < s.min_area_fraction:
        return None
    moments = cv2.moments(contour)
    if moments['m00'] <= 0:
        return None
    u = moments['m10'] / moments['m00']
    v = moments['m01'] / moments['m00']
    return Detection(cx=2.0 * u / width - 1.0,
                     cy=2.0 * v / height - 1.0,
                     area_fraction=area / float(width * height),
                     pixel_area=int(area))
