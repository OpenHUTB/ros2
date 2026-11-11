"""地形纵剖面模型（纯标准库）。

剖面是一条折线：`[[x0, z0], [x1, z1], ...]`，x 严格递增，单位米。

三条基本查询：
* `height(x)`  该处的路面高度（两端按水平延长，避免车开出剖面就报错）
* `slope(x)`   该处路面与水平面的夹角（弧度，上坡为正）
* `breaks()`   所有折点，并区分「下凹」与「上凸」
    - 下凹（凹角，路面往上折）：车头/车尾会先碰到坡，对应接近角/离去角
    - 上凸（凸角，路面往下折）：车底中部会先碰到坡顶，对应纵向通过角
"""

import math

# 折角小于该值不算折点（度）。剖面点写得密时会有大量 0.0x 度的伪折点，
# 用这个阈值滤掉，只留真正影响通过性的转折。
MIN_BREAK_DEG = 0.20


class TerrainError(ValueError):
    """剖面数据不合法。"""


def _as_point(item, index):
    if not isinstance(item, (list, tuple)) or len(item) != 2:
        raise TerrainError("第 %d 个剖面点应为 [x, z] 两元组：%r" % (index, item))
    try:
        x = float(item[0])
        z = float(item[1])
    except (TypeError, ValueError):
        raise TerrainError("第 %d 个剖面点不是数字：%r" % (index, item))
    for value, label in ((x, "x"), (z, "z")):
        if math.isnan(value) or math.isinf(value):
            raise TerrainError("第 %d 个剖面点的 %s 不是有限数" % (index, label))
    return x, z


class Terrain(object):
    """一条纵剖面折线。"""

    __slots__ = ("points", "name")

    def __init__(self, points, name="terrain"):
        if not isinstance(points, (list, tuple)):
            raise TerrainError("剖面必须是点列表：%r" % (points,))
        if len(points) < 2:
            raise TerrainError("剖面至少要 2 个点，实际 %d 个" % len(points))
        parsed = [_as_point(item, index) for index, item in enumerate(points)]
        for index in range(len(parsed) - 1):
            if parsed[index + 1][0] - parsed[index][0] <= 1e-9:
                raise TerrainError("剖面点的 x 必须严格递增（第 %d 与第 %d 个点）" % (index, index + 1))
        self.points = parsed
        self.name = name

    # ------------------------------------------------------------ 基本查询
    @property
    def min_x(self):
        return self.points[0][0]

    @property
    def max_x(self):
        return self.points[-1][0]

    @property
    def min_z(self):
        return min(point[1] for point in self.points)

    @property
    def max_z(self):
        return max(point[1] for point in self.points)

    def _segment_index(self, x):
        """返回 x 所在的折线段下标（两端按水平延长）。"""
        if x <= self.points[0][0]:
            return 0
        for index in range(len(self.points) - 1):
            if x < self.points[index + 1][0]:
                return index
        return len(self.points) - 2

    def height(self, x):
        """路面高度；剖面两端按水平延长。"""
        index = self._segment_index(x)
        x0, z0 = self.points[index]
        x1, z1 = self.points[index + 1]
        if x <= self.points[0][0]:
            return z0
        if x >= self.points[-1][0]:
            return self.points[-1][1]
        if abs(x1 - x0) < 1e-12:
            return z0
        ratio = (x - x0) / (x1 - x0)
        return z0 + (z1 - z0) * ratio

    def slope(self, x):
        """路面坡度（弧度，上坡为正）。"""
        index = self._segment_index(x)
        x0, z0 = self.points[index]
        x1, z1 = self.points[index + 1]
        return math.atan2(z1 - z0, x1 - x0)

    def slope_deg(self, x):
        return math.degrees(self.slope(x))

    # ------------------------------------------------------------ 折点
    def breaks(self, min_deg=MIN_BREAK_DEG):
        """所有折点，返回 [{x, before_deg, after_deg, delta_deg, kind}]。

        `kind`：
        * `"concave"` 下凹 —— 路面往上折，车头/车尾有风险
        * `"convex"`  上凸 —— 路面往下折，车底中部有风险
        """
        result = []
        for index in range(1, len(self.points) - 1):
            x = self.points[index][0]
            before = self.slope(x - 1e-9)
            after = self.slope(x + 1e-9)
            delta = math.degrees(after - before)
            if abs(delta) < min_deg:
                continue
            result.append({
                "x": x,
                "before_deg": math.degrees(before),
                "after_deg": math.degrees(after),
                "delta_deg": delta,
                "kind": "concave" if delta > 0 else "convex",
            })
        return result

    def max_concave_deg(self, min_deg=MIN_BREAK_DEG):
        """最陡的下凹折角（度）。没有折点时返回 0。"""
        values = [item["delta_deg"] for item in self.breaks(min_deg) if item["kind"] == "concave"]
        return max(values) if values else 0.0

    def max_convex_deg(self, min_deg=MIN_BREAK_DEG):
        """最陡的上凸折角（度）。没有折点时返回 0。"""
        values = [-item["delta_deg"] for item in self.breaks(min_deg) if item["kind"] == "convex"]
        return max(values) if values else 0.0

    def to_dict(self):
        """转回场景文件里的 `terrain` 字段格式。"""
        return {"name": self.name, "terrain": [[x, z] for x, z in self.points]}

    def max_step(self):
        """剖面里最大的单点高度突变（用于台阶/减速带这类近垂直折线）。"""
        step = 0.0
        for index in range(1, len(self.points)):
            step = max(step, abs(self.points[index][1] - self.points[index - 1][1]))
        return step

    def __repr__(self):
        return "Terrain(name=%r, points=%d, x=[%.2f, %.2f], z=[%.2f, %.2f])" % (
            self.name, len(self.points), self.min_x, self.max_x, self.min_z, self.max_z)


def ramp_profile(length_flat=3.0, ramp_length=6.0, ramp_deg=11.31,
                 top_flat=3.0, base_z=0.0):
    """生成一段「平地 → 上坡 → 平地」的剖面（地库坡道的典型形状）。"""
    if ramp_length <= 0.0:
        raise TerrainError("ramp_length 必须大于 0")
    rise = ramp_length * math.tan(math.radians(ramp_deg))
    return Terrain([
        [0.0, base_z],
        [length_flat, base_z],
        [length_flat + ramp_length, base_z + rise],
        [length_flat + ramp_length + top_flat, base_z + rise],
    ], name="ramp")


def crest_profile(up_length=5.0, up_deg=8.0, down_length=5.0, down_deg=4.0, base_z=0.0):
    """生成一段「上坡 → 坡顶 → 下坡」的剖面（山路过顶的典型形状）。"""
    rise = up_length * math.tan(math.radians(up_deg))
    drop = down_length * math.tan(math.radians(down_deg))
    return Terrain([
        [0.0, base_z],
        [up_length, base_z + rise],
        [up_length + down_length, base_z + rise - drop],
    ], name="crest")


def bump_profile(height=0.10, top_width=0.70, slope_width=0.35, flat=2.0, base_z=0.0):
    """生成一段减速带（梯形）剖面。"""
    if height <= 0.0:
        raise TerrainError("减速带高度必须大于 0")
    return Terrain([
        [0.0, base_z],
        [flat, base_z],
        [flat + slope_width, base_z + height],
        [flat + slope_width + top_width, base_z + height],
        [flat + 2.0 * slope_width + top_width, base_z],
        [flat + 2.0 * slope_width + top_width + flat, base_z],
    ], name="bump")
