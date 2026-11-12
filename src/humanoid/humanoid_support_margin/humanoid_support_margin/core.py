"""人形机器人支撑多边形与倾倒裕度核算（只依赖标准库）.

站立稳定性的三笔账：

1. **支撑多边形**：两脚着地矩形区域的凸包（单调链凸包算法，纯标准库）；
2. **稳定裕度**：重心投影到多边形各边的最近距离——重心越靠边，越容易被推倒；
3. **倾倒判据**：
       F_crit = m * g * d / h        静态临界水平推力（力矩平衡于最近边）
       J_crit = m * sqrt(2*g*(sqrt(d^2+h^2) - h))   临界冲量（能量法，重心须抬过边棱）

d 为重心投影到最近边界的距离，h 为重心高度。冲量判据按"绕最近边棱翻转"的倒立摆能量
推导：给重心一个水平速度，动能必须足够把重心抬到边棱正上方的高度 sqrt(d^2+h^2)。

另给双足站立的载荷分配（等刚度弹簧模型，只沿前后轴 x 解）：
    f1 = W * (x2 - x) / (x2 - x1),   f2 = W * (x - x1) / (x2 - x1)
x 在两脚之间时成立；投影超出两脚范围时按单脚支撑处理（全部载荷在较近脚）。
"""

import math
from dataclasses import dataclass

G = 9.80665


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def convex_hull(points):
    """平面点集凸包（单调链，返回逆时针顶点列表）."""
    _require(len(points) >= 3, '凸包至少需要 3 个点')
    pts = sorted((float(x), float(y)) for x, y in points)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_in_polygon(point, polygon):
    """射线法判定点是否在多边形内（含边界）."""
    x, y = point
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) / (y2 - y1) * (x2 - x1)
            if x_cross > x:
                inside = not inside
    if inside:
        return True
    for i in range(n):
        if _distance_to_segment((x, y), polygon[i], polygon[(i + 1) % n]) < 1e-12:
            return True
    return False


def _distance_to_segment(p, a, b):
    """点到线段距离."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-24:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def support_margin(point, polygon):
    """重心投影到支撑多边形的最近边距离 m；在多边形外返回负值."""
    _require(_finite(point[0]) and _finite(point[1]), '投影点必须是有限数')
    _require(len(polygon) >= 3, '多边形至少需要 3 个点')
    n = len(polygon)
    distance = min(_distance_to_segment(point, polygon[i], polygon[(i + 1) % n])
                   for i in range(n))
    return distance if point_in_polygon(point, polygon) else -distance


def tipping_force_n(mass_kg, margin_m, com_height_m, gravity=G):
    """静态临界水平推力 N：力矩平衡于最近边棱."""
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(com_height_m) and com_height_m > 0.0, '重心高度必须为正数')
    _require(_finite(margin_m) and margin_m > 0.0, '稳定裕度必须为正数')
    return mass_kg * gravity * margin_m / com_height_m


def tipping_impulse_ns(mass_kg, margin_m, com_height_m, gravity=G):
    """临界冲量 N*s：能量法，重心须抬过边棱正上方."""
    _require(_finite(mass_kg) and mass_kg > 0.0, '质量必须为正数')
    _require(_finite(com_height_m) and com_height_m > 0.0, '重心高度必须为正数')
    _require(_finite(margin_m) and margin_m > 0.0, '稳定裕度必须为正数')
    lift = math.sqrt(margin_m ** 2 + com_height_m ** 2) - com_height_m
    return mass_kg * math.sqrt(2.0 * gravity * lift)


def foot_load_fractions(com_x_m, x1_m, x2_m):
    """双足站立载荷分配 (f1, f2)，各为重量 W 的系数；投影在两脚之外按单脚支撑."""
    _require(_finite(com_x_m) and _finite(x1_m) and _finite(x2_m), '坐标必须是有限数')
    _require(x2_m > x1_m, 'x2 必须大于 x1')
    if com_x_m <= x1_m:
        return 1.0, 0.0
    if com_x_m >= x2_m:
        return 0.0, 1.0
    return (x2_m - com_x_m) / (x2_m - x1_m), (com_x_m - x1_m) / (x2_m - x1_m)


@dataclass
class HumanoidSupport:
    """一次站立稳定性核算：两脚矩形着地区 + 重心投影."""

    mass_kg: float = 40.0
    foot1_x_m: float = -0.10
    foot2_x_m: float = 0.10
    foot_width_m: float = 0.10
    foot_length_m: float = 0.25
    com_x_m: float = 0.01
    com_y_m: float = 0.0
    com_height_m: float = 0.8

    def __post_init__(self):
        for name in ('mass_kg', 'foot1_x_m', 'foot2_x_m', 'foot_width_m',
                     'foot_length_m', 'com_x_m', 'com_y_m', 'com_height_m'):
            _require(_finite(getattr(self, name)), '%s 必须是有限数' % name)
        _require(self.mass_kg > 0.0, '质量必须为正数')
        _require(self.foot2_x_m > self.foot1_x_m, '右脚坐标必须大于左脚')
        _require(self.foot_width_m > 0.0 and self.foot_length_m > 0.0,
                 '脚掌尺寸必须为正数')
        _require(self.com_height_m > 0.0, '重心高度必须为正数')

    def support_polygon(self):
        half_w = self.foot_width_m / 2.0
        half_l = self.foot_length_m / 2.0
        corners = []
        for x in (self.foot1_x_m, self.foot2_x_m):
            corners.append((x - half_w, -half_l))
            corners.append((x - half_w, half_l))
            corners.append((x + half_w, half_l))
            corners.append((x + half_w, -half_l))
        return convex_hull(corners)

    def report(self):
        polygon = self.support_polygon()
        margin = support_margin((self.com_x_m, self.com_y_m), polygon)
        inside = margin >= 0.0
        weight = self.mass_kg * G
        f1, f2 = foot_load_fractions(self.com_x_m, self.foot1_x_m, self.foot2_x_m)
        warnings = []
        level = 'ok'
        if not inside:
            level = 'over'
            warnings.append('重心投影在支撑多边形之外 %.3f m，静立必倒' % (-margin))
        else:
            if margin < 0.02:
                level = 'marginal'
                warnings.append('稳定裕度只有 %.1f mm，轻微扰动就可能出界' % (margin * 1000.0))
            if margin >= 0.02:
                warnings.append('稳定裕度 %.1f mm，临界推力 %.1f N，临界冲量 %.1f N*s'
                                % (margin * 1000.0,
                                   tipping_force_n(self.mass_kg, margin, self.com_height_m),
                                   tipping_impulse_ns(self.mass_kg, margin,
                                                      self.com_height_m)))
        return {
            'parameters': {
                'mass_kg': self.mass_kg,
                'foot1_x_m': self.foot1_x_m,
                'foot2_x_m': self.foot2_x_m,
                'foot_width_m': self.foot_width_m,
                'foot_length_m': self.foot_length_m,
                'com_x_m': self.com_x_m,
                'com_y_m': self.com_y_m,
                'com_height_m': self.com_height_m,
            },
            'support_polygon': [list(p) for p in polygon],
            'com_projection': [self.com_x_m, self.com_y_m],
            'inside_polygon': inside,
            'margin_m': margin,
            'level': level,
            'tipping': {
                'static_force_n': tipping_force_n(self.mass_kg, margin, self.com_height_m)
                if margin > 0.0 else 0.0,
                'impulse_ns': tipping_impulse_ns(self.mass_kg, margin, self.com_height_m)
                if margin > 0.0 else 0.0,
            },
            'foot_loads': {'fraction_left': f1, 'fraction_right': f2,
                           'left_n': f1 * weight, 'right_n': f2 * weight},
            'warnings': warnings,
            'notes': [
                '两脚矩形着地区无偏航，凸包即支撑多边形',
                '载荷分配是等刚度弹簧模型，真实足底压力分布需要 CoP 传感器',
                '倾倒判据按绕最近边棱的刚体翻转推导，不含脚踝力矩与屈膝缓冲',
            ],
        }
