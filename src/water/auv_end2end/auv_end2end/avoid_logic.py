"""
AUV 视觉避障的纯逻辑实现.

本模块不依赖 ROS，只依赖 numpy，便于单元测试：
- analyze()：把一帧灰度图换算成左右两侧的暗色区域占比
- AvoidStateMachine：滞环 + 超时 + 冷却 + 紧急重触发的状态机
"""

from typing import NamedTuple, Optional

import numpy as np

# 只取画面中上部条带，剔除海底地面与顶部的雾效
ROI_TOP_RATIO = 0.15
ROI_BOTTOM_RATIO = 0.70

# 自适应阈值的钳位区间：防止整体过亮/过暗时把噪声当成障碍
MIN_ADAPTIVE_THRESH = 10
MAX_ADAPTIVE_THRESH = 60

# Otsu 的双峰可分性下限（类间方差 / 总方差）.
# 实测：镜头对着墙体时约 0.997；开阔水域整幅画面几乎是同一个灰度，
# 根本没有可分割的分布。灰度渐变（雾）即使硬切也只能到 0.75 左右，
# 因此 0.8 能把「真有障碍」和「均匀背景 / 平滑渐变」分开.
MIN_SEPARABILITY = 0.8

DIR_RIGHT = -1
DIR_LEFT = 1
DIR_NONE = 0


class AvoidOutput(NamedTuple):
    """状态机一次更新的输出."""

    direction: int          # DIR_RIGHT / DIR_LEFT / DIR_NONE
    is_avoiding: bool
    is_cooldown: bool
    event: Optional[str]    # 事件名，供上层写日志


def split_roi(image):
    """裁剪画面中上部条带."""
    height = image.shape[0]
    top = int(height * ROI_TOP_RATIO)
    bottom = int(height * ROI_BOTTOM_RATIO)
    return image[top:bottom, :]


def otsu_split(gray):
    """
    用直方图计算 Otsu 阈值与双峰可分性.

    返回 (thresh, separability)：thresh 为 None 表示画面没有可分割的
    灰度分布；separability = 类间方差 / 总方差，取值 0~1，越大说明
    画面越接近「暗障碍 + 亮水体」两团.
    """
    if gray.size == 0:
        return None, 0.0
    hist = np.bincount(gray.ravel(), minlength=256).astype(np.float64)
    levels = np.arange(256)
    total = float(gray.size)

    weight_bg = np.cumsum(hist)
    weight_fg = total - weight_bg
    valid = (weight_bg > 0) & (weight_fg > 0)
    if not np.any(valid):
        return None, 0.0

    cum_sum = np.cumsum(levels * hist)
    sum_all = cum_sum[-1]
    mean_bg = cum_sum / np.maximum(weight_bg, 1.0)
    mean_fg = (sum_all - cum_sum) / np.maximum(weight_fg, 1.0)

    between = weight_bg * weight_fg * (mean_bg - mean_fg) ** 2
    between[~valid] = -1.0
    best = int(np.argmax(between))
    if between[best] <= 0:
        return None, 0.0

    mean_all = sum_all / total
    total_var = float((((levels - mean_all) ** 2) * hist).sum()) / total
    if total_var <= 0:
        return None, 0.0
    # between 是按像素个数累加的，除以 total² 才是概率化的类间方差
    separability = float(between[best]) / (total * total * total_var)
    return best, min(separability, 1.0)


def adaptive_threshold(gray, fixed_thresh=22):
    """
    Otsu 阈值并钳位到 MIN/MAX_ADAPTIVE_THRESH 之间.

    画面不够双峰（开阔水域、均匀雾背景）时退回调用方给的固定阈值，
    否则 Otsu 会把均匀背景从中间切开，整幅画面都被判成障碍.
    """
    thresh, separability = otsu_split(gray)
    if thresh is None or separability < MIN_SEPARABILITY:
        return int(fixed_thresh)
    return int(min(max(thresh, MIN_ADAPTIVE_THRESH), MAX_ADAPTIVE_THRESH))


def dark_ratios(gray, thresh):
    """
    按阈值统计左右两半的暗色像素占比.

    阈值的语义与 Otsu 一致：灰度 <= thresh 的像素归为障碍（暗类）。
    """
    width = gray.shape[1]
    mid = width // 2
    dark = gray <= thresh
    left_area = dark[:, :mid]
    right_area = dark[:, mid:]
    left = float(np.count_nonzero(left_area)) / left_area.size
    right = float(np.count_nonzero(right_area)) / right_area.size
    return left, right


def analyze(gray, fixed_thresh=22, adaptive=True):
    """返回 (left_ratio, right_ratio, 实际使用的阈值)."""
    if adaptive:
        thresh = adaptive_threshold(gray, fixed_thresh)
    else:
        thresh = int(fixed_thresh)
    left, right = dark_ratios(gray, thresh)
    return left, right, thresh


class AvoidStateMachine:
    """
    避障状态机.

    状态：直行 -> 避让 -> (冷却) -> 直行
    - 滞环：触发阈值 trigger_ratio，恢复阈值 recover_ratio
    - 超时：单次避让超过 max_avoid_duration 强制进入冷却
    - 冷却：cooldown 秒内强制直行；但画面显著恶化（超过触发阈值
      emergency_factor 倍）时立即中断冷却重新避让，避免盲区直穿障碍
    - 最小避让时长：进入避让后至少持续 min_avoid_duration 秒才允许
      因画面变亮而恢复，避免阈值附近抖动
    """

    EVENT_ENTER_RIGHT = 'enter_avoid_right'
    EVENT_ENTER_LEFT = 'enter_avoid_left'
    EVENT_CLEARED = 'cleared'
    EVENT_TIMEOUT = 'timeout'
    EVENT_COOLDOWN_END = 'cooldown_end'
    EVENT_EMERGENCY = 'emergency_retrigger'

    def __init__(self,
                 trigger_ratio=0.22,
                 recover_ratio=0.18,
                 max_avoid_duration=3.0,
                 cooldown=2.0,
                 min_avoid_duration=0.3,
                 emergency_factor=1.5):
        self.trigger_ratio = trigger_ratio
        self.recover_ratio = recover_ratio
        self.max_avoid_duration = max_avoid_duration
        self.cooldown = cooldown
        self.min_avoid_duration = min_avoid_duration
        self.emergency_factor = emergency_factor

        self.is_avoiding = False
        self.is_cooldown = False
        self.avoid_start = 0.0
        self.cooldown_start = 0.0
        self.direction = DIR_NONE

    def _direction_from(self, left_ratio, right_ratio):
        # 左侧障碍更多则向右避让，右侧更多则向左避让
        if left_ratio > right_ratio:
            return DIR_RIGHT
        return DIR_LEFT

    def _enter_avoid(self, now, left_ratio, right_ratio):
        self.is_avoiding = True
        self.is_cooldown = False
        self.avoid_start = now
        self.direction = self._direction_from(left_ratio, right_ratio)
        if self.direction == DIR_RIGHT:
            return self.EVENT_ENTER_RIGHT
        return self.EVENT_ENTER_LEFT

    def update(self, now, left_ratio, right_ratio):
        """输入当前时刻（秒）与左右占比，返回 AvoidOutput."""
        event = None
        peak = max(left_ratio, right_ratio)
        detected = peak > self.trigger_ratio
        cleared = peak < self.recover_ratio

        if self.is_cooldown:
            emergency = peak > self.trigger_ratio * self.emergency_factor
            if emergency:
                self._enter_avoid(now, left_ratio, right_ratio)
                event = self.EVENT_EMERGENCY
            elif now - self.cooldown_start >= self.cooldown:
                self.is_cooldown = False
                event = self.EVENT_COOLDOWN_END
        elif self.is_avoiding:
            elapsed = now - self.avoid_start
            if elapsed >= self.max_avoid_duration:
                self.is_avoiding = False
                self.is_cooldown = True
                self.cooldown_start = now
                self.direction = DIR_NONE
                event = self.EVENT_TIMEOUT
            elif cleared and elapsed >= self.min_avoid_duration:
                self.is_avoiding = False
                self.direction = DIR_NONE
                event = self.EVENT_CLEARED
        elif detected:
            event = self._enter_avoid(now, left_ratio, right_ratio)

        direction = self.direction if self.is_avoiding else DIR_NONE
        return AvoidOutput(direction, self.is_avoiding, self.is_cooldown, event)
