# -*- coding: utf-8 -*-
"""障碍物地图与线段-矩形碰撞检测。

纯几何运算，不依赖 pygame，便于离线单元测试。
碰撞检测采用 Liang-Barsky 线段裁剪算法判断线段是否与任一矩形相交。
"""
from __future__ import annotations
from typing import Iterable, Tuple

Rect = Tuple[float, float, float, float]  # (x, y, w, h)


class ObstacleMap:
    """矩形障碍物地图。

    坐标系：左上角原点，x 向右，y 向下（与 pygame 一致）。
    """

    def __init__(self, obstacles: Iterable[Rect], width: float, height: float):
        self.obstacles = [tuple(o) for o in obstacles]
        self.width = width
        self.height = height

    def point_in_obstacle(self, x: float, y: float) -> bool:
        """判断点是否落在任一矩形障碍内部（含边界）。"""
        for (ox, oy, ow, oh) in self.obstacles:
            if ox <= x <= ox + ow and oy <= y <= oy + oh:
                return True
        return False

    def segment_collides(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """判断线段 (x1,y1)-(x2,y2) 是否与任一矩形障碍相交（含穿过内部）。

        采用 Liang-Barsky 参数裁剪：线段写成 P = P1 + t*(P2-P1), t in [0,1]，
        对每个矩形求 t 的可行区间，若区间与 [0,1] 有交集则相交。
        """
        dx = x2 - x1
        dy = y2 - y1
        for (ox, oy, ow, oh) in self.obstacles:
            if self._liang_barsky_hit(x1, y1, dx, dy, ox, oy, ow, oh):
                return True
        return False

    @staticmethod
    def _liang_barsky_hit(x1, y1, dx, dy, ox, oy, ow, oh) -> bool:
        """Liang-Barsky 判断线段是否进入矩形 [ox,ox+ow]x[oy,oy+oh]。"""
        # 端点任一在矩形内即视为碰撞
        if (ox <= x1 <= ox + ow and oy <= y1 <= oy + oh) or \
           (ox <= x1 + dx <= ox + ow and oy <= y1 + dy <= oy + oh):
            return True
        t0, t1 = 0.0, 1.0
        for p, q in ((-dx, x1 - ox), (dx, ox + ow - x1),
                     (-dy, y1 - oy), (dy, oy + oh - y1)):
            if p == 0:
                # 平行于该边界：若端点在边界外侧则不可能进入
                if q < 0:
                    return False
            elif p < 0:
                t0 = max(t0, q / p)
            else:
                t1 = min(t1, q / p)
            if t0 > t1:
                break
        else:
            if t0 <= t1:
                return True
        return False

    def in_bounds(self, x: float, y: float, margin: float = 1.0) -> bool:
        """采样点是否在画布范围内。"""
        return (margin <= x <= self.width - margin) and (margin <= y <= self.height - margin)
