"""减速带几何：把路面高度表示成车轮纵向位置的函数。

模型把减速带当作刚性路面轮廓，车轮在通过时被"抬起" ``z_r(x)``。给定通过车速
``v``，把位置换成就足以得到时间激励 ``z_r(t) = z_r(v * t)``。这里只描述几何，
不含任何悬架动力学，便于单独用解析面积校验。
"""

from __future__ import annotations

import numpy as np

#: 支持的减速带截面形状。
KINDS = ("half_sine", "trapezoid")


class SpeedBump:
    """一段宽度为 ``width_m``、峰高为 ``height_m`` 的减速带。

    参数
    ----
    height_m:
        峰高（m），必须为正。
    width_m:
        沿行驶方向的宽度（m），必须为正。
    kind:
        ``half_sine`` 半正弦（圆弧减速带常用近似）或 ``trapezoid`` 梯形（含上下坡） 。
    ramp_ratio:
        仅梯形使用，上下坡各占宽度的比例，需满足 ``0 < ramp_ratio < 0.5``，
        保证中间留下平台段。
    """

    def __init__(
        self,
        height_m: float = 0.06,
        width_m: float = 1.2,
        kind: str = "half_sine",
        ramp_ratio: float = 0.25,
    ) -> None:
        if not height_m > 0.0:
            raise ValueError("height_m must be positive")
        if not width_m > 0.0:
            raise ValueError("width_m must be positive")
        if kind not in KINDS:
            raise ValueError(f"kind must be one of {KINDS}, got {kind!r}")
        if not 0.0 < ramp_ratio < 0.5:
            raise ValueError("ramp_ratio must be in (0, 0.5)")
        self.height_m = float(height_m)
        self.width_m = float(width_m)
        self.kind = str(kind)
        self.ramp_ratio = float(ramp_ratio)

    def __repr__(self) -> str:  # pragma: no cover - 仅便于调试输出
        return (
            f"SpeedBump(height_m={self.height_m}, width_m={self.width_m}, "
            f"kind={self.kind!r}, ramp_ratio={self.ramp_ratio})"
        )

    def duration_at(self, speed_mps: float) -> float:
        """车轮与减速带接触的时间（s）。"""
        if not speed_mps > 0.0:
            raise ValueError("speed_mps must be positive")
        return self.width_m / float(speed_mps)

    def profile(self, position_m) -> np.ndarray:
        """路面高度 ``z_r(x)``，单位 m；减速带之外返回 0。"""
        x = np.asarray(position_m, dtype=float)
        height = np.zeros_like(x)
        inside = (x > 0.0) & (x < self.width_m)
        if not np.any(inside):
            return height
        if self.kind == "half_sine":
            height[inside] = self.height_m * np.sin(np.pi * x[inside] / self.width_m)
            return height
        ramp = self.ramp_ratio * self.width_m
        rising = inside & (x <= ramp)
        plateau = inside & (x > ramp) & (x < self.width_m - ramp)
        falling = inside & (x >= self.width_m - ramp)
        height[rising] = self.height_m * x[rising] / ramp
        height[plateau] = self.height_m
        height[falling] = (
            self.height_m * (self.width_m - x[falling]) / ramp
        )
        return height

    def profile_time(self, time_s, speed_mps: float) -> np.ndarray:
        """以车轮到达减速带前沿为 ``t = 0`` 的时间激励 ``z_r(t)``。"""
        t = np.asarray(time_s, dtype=float)
        return self.profile(t * float(speed_mps))

    def reference_area_m2(self) -> float:
        """截面在行驶方向的积分面积（m²），用于几何校验。"""
        if self.kind == "half_sine":
            return self.height_m * 2.0 * self.width_m / np.pi
        return self.height_m * self.width_m * (1.0 - self.ramp_ratio)
