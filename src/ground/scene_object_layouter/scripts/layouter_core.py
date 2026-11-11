#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""scene_object_layouter 的纯逻辑核心。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 不读写任何文件（除 load_scene 显式读入），不做 IO，纯粹是几何与文本

模块干的事：
给定一块场地和若干「物体组」的规格，按栅格把物体摆出来，然后回答四个问题：
边界内吗？互相重叠吗？间距够吗？数量对吗？
最后输出一份终端报告、一张字符俯视图、一份 CSV 与一份 JSON。
"""

from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import io
import math

__all__ = [
    "LayouterError",
    "Rect",
    "ObjectSpec",
    "PlacedObject",
    "Scene",
    "Field",
    "LayouterParams",
    "layout_group",
    "layout_scene",
    "check_layout",
    "format_report",
    "render_scene",
    "to_csv",
    "to_json",
    "DEMO_SCENE_TEXT",
    "parse_scene_text",
    "load_scene",
    "normalize_angle",
    "polygon_gap",
    "rectangles_overlap",
]


# ---------------------------------------------------------------- 默认参数

DEFAULT_FIELD_WIDTH = 30.0
DEFAULT_FIELD_LENGTH = 40.0
DEFAULT_MIN_GAP = 0.50
DEFAULT_GRID_WIDTH = 60
DEFAULT_GRID_HEIGHT = 24
DEFAULT_CSV_PATH = "scene_objects.csv"
DEFAULT_JSON_PATH = "scene_objects.json"
DEFAULT_PUBLISH_HZ = 1.0
DEFAULT_TOPIC_OBJECTS = "/scene/objects"
DEFAULT_TOPIC_SUMMARY = "/scene/summary"
DEFAULT_TOPIC_TRIGGER = "/scene/layout"

#: 判定两个矩形「贴边」时的浮点容差
_EPS = 1e-9

#: 点数小于这个值就当退化处理
_MIN_SIZE = 1e-6


DEMO_SCENE_TEXT = """\
name: demo_parking_row
field:
  width: 24.0
  length: 18.0
min_gap: 0.5
objects:
  - kind: slot
    width: 2.5
    length: 5.3
    rows: 2
    cols: 6
    gap_x: 0.6
    gap_y: 6.0
    origin_x: 3.0
    origin_y: 3.0
    blueprint: static.prop.parkingline
  - kind: pillar
    width: 1.0
    length: 1.0
    rows: 1
    cols: 3
    gap_x: 4.0
    origin_x: 6.0
    origin_y: 8.65
    blueprint: static.prop.pillar
"""


class LayouterError(ValueError):
    """场景描述或几何参数不合法。"""


# ---------------------------------------------------------------- 小工具


def _as_float(value, name, default=None):
    if value is None:
        if default is None:
            raise LayouterError("%s 缺少取值" % name)
        return float(default)
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise LayouterError("%s 不是数字：%r" % (name, value))
    if result != result or result in (float("inf"), float("-inf")):
        raise LayouterError("%s 不是有限数：%r" % (name, value))
    return result


def _as_int(value, name, default=None):
    number = _as_float(value, name, default)
    rounded = int(round(number))
    if abs(number - rounded) > 1e-6:
        raise LayouterError("%s 必须是整数：%r" % (name, value))
    return rounded


def _positive(value, name):
    number = _as_float(value, name)
    if number <= 0.0:
        raise LayouterError("%s 必须大于 0：%r" % (name, value))
    return number


def _non_negative(value, name):
    number = _as_float(value, name)
    if number < 0.0:
        raise LayouterError("%s 不能为负：%r" % (name, value))
    return number


def as_bool(value, default=False):
    """把 yaml / rosparam 里的各种写法统一成 bool（兼容 py2 的 str/unicode）。"""
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = "%s" % value
    text = text.strip().lower()
    if text in ("1", "true", "yes", "on", "y"):
        return True
    if text in ("0", "false", "no", "off", "n", ""):
        return False
    raise LayouterError("无法识别的布尔取值：%r" % (value,))


def to_message_text(text):
    """把文本转成 std_msgs/String 能安全携带的形态。

    Python 2 的 String.data 是 byte str，Python 3 是 str；Melodic 上如果直接塞
    unicode，rosgraph 的日志/序列化路径会抛 UnicodeEncodeError。
    """
    if text is None:
        return b"" if str is bytes else ""
    if str is bytes:  # Python 2
        if isinstance(text, unicode):  # noqa: F821
            return text.encode("utf-8")
        return text
    if isinstance(text, bytes):
        return text.decode("utf-8")
    return text


def normalize_angle(angle):
    """把角度折算到 (-pi, pi]。"""
    value = _as_float(angle, "angle")
    wrapped = math.fmod(value + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


# ---------------------------------------------------------------- 几何工具


def rotate_point(point, yaw):
    """把「物体坐标系」下的点转到世界坐标系（先旋转再平移由调用方做）。"""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (point[0] * cos_yaw - point[1] * sin_yaw,
            point[0] * sin_yaw + point[1] * cos_yaw)


def point_segment_distance(point, start, end):
    """点到线段的最短距离。"""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq < _MIN_SIZE:
        return distance(point, start)
    t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    projection = (start[0] + t * dx, start[1] + t * dy)
    return distance(point, projection)


def _cross(origin, a, b):
    return ((a[0] - origin[0]) * (b[1] - origin[1])
            - (a[1] - origin[1]) * (b[0] - origin[0]))


def segments_intersect(p1, p2, p3, p4):
    """两线段是否相交（含端点接触）。"""
    d1 = _cross(p3, p4, p1)
    d2 = _cross(p3, p4, p2)
    d3 = _cross(p1, p2, p3)
    d4 = _cross(p1, p2, p4)
    if ((d1 > _EPS and d2 < -_EPS) or (d1 < -_EPS and d2 > _EPS)) and \
       ((d3 > _EPS and d4 < -_EPS) or (d3 < -_EPS and d4 > _EPS)):
        return True
    # 共线/端点接触的情况，用「投影区间是否重叠」兜底
    for a, b, c in ((p1, p2, p3), (p1, p2, p4), (p3, p4, p1), (p3, p4, p2)):
        if abs(_cross(a, b, c)) <= _EPS:
            if (min(a[0], b[0]) - _EPS <= c[0] <= max(a[0], b[0]) + _EPS and
                    min(a[1], b[1]) - _EPS <= c[1] <= max(a[1], b[1]) + _EPS):
                return True
    return False


def _polygon_axes(points):
    """外法线方向（分离轴定理用），全部归一化。"""
    axes = []
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < _MIN_SIZE:
            continue
        axes.append((-dy / length, dx / length))
    return axes


def _project(points, axis):
    values = [points[0][0] * axis[0] + points[0][1] * axis[1]]
    for point in points[1:]:
        values.append(point[0] * axis[0] + point[1] * axis[1])
    return min(values), max(values)


def rectangles_overlap(points_a, points_b):
    """两个凸多边形是否重叠（分离轴定理）。贴边算「不重叠」。"""
    for axis in _polygon_axes(points_a) + _polygon_axes(points_b):
        min_a, max_a = _project(points_a, axis)
        min_b, max_b = _project(points_b, axis)
        if max_a <= min_b + _EPS or max_b <= min_a + _EPS:
            return False
    return True


def polygon_gap(points_a, points_b):
    """两个凸多边形之间的最小间距；重叠时返回 0.0。"""
    if rectangles_overlap(points_a, points_b):
        return 0.0
    best = None
    for point in points_a:
        for index in range(len(points_b)):
            gap = point_segment_distance(point, points_b[index],
                                         points_b[(index + 1) % len(points_b)])
            if best is None or gap < best:
                best = gap
    for point in points_b:
        for index in range(len(points_a)):
            gap = point_segment_distance(point, points_a[index],
                                         points_a[(index + 1) % len(points_a)])
            if best is None or gap < best:
                best = gap
    return 0.0 if best is None else best


# ---------------------------------------------------------------- 场地与物体


class Field(object):
    """矩形场地，坐标范围 x ∈ [0, width]，y ∈ [0, length]。"""

    __slots__ = ("width", "length")

    def __init__(self, width=DEFAULT_FIELD_WIDTH, length=DEFAULT_FIELD_LENGTH):
        self.width = _positive(width, "field.width")
        self.length = _positive(length, "field.length")

    def corners(self):
        return [(0.0, 0.0), (self.width, 0.0),
                (self.width, self.length), (0.0, self.length)]

    def contains(self, points):
        for x, y in points:
            if x < -1e-6 or x > self.width + 1e-6:
                return False
            if y < -1e-6 or y > self.length + 1e-6:
                return False
        return True

    def as_tuple(self):
        return (self.width, self.length)

    def __repr__(self):
        return "Field(width=%.3f, length=%.3f)" % (self.width, self.length)


class Rect(object):
    """带朝向的矩形（车辆/车位/障碍物都用它表示）。"""

    __slots__ = ("x", "y", "yaw", "width", "length")

    def __init__(self, x, y, width, length, yaw=0.0):
        self.x = _as_float(x, "rect.x")
        self.y = _as_float(y, "rect.y")
        self.width = _positive(width, "rect.width")
        self.length = _positive(length, "rect.length")
        self.yaw = normalize_angle(yaw)

    def corners(self):
        half_w = self.width / 2.0
        half_l = self.length / 2.0
        local = [(-half_w, -half_l), (half_w, -half_l),
                 (half_w, half_l), (-half_w, half_l)]
        result = []
        for point in local:
            dx, dy = rotate_point(point, self.yaw)
            result.append((self.x + dx, self.y + dy))
        return result

    def bounds(self):
        points = self.corners()
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return (min(xs), min(ys), max(xs), max(ys))

    def as_tuple(self):
        return (self.x, self.y, self.width, self.length, self.yaw)

    def __repr__(self):
        return ("Rect(x=%.3f, y=%.3f, w=%.3f, l=%.3f, yaw=%.3f)"
                % (self.x, self.y, self.width, self.length, self.yaw))


class ObjectSpec(object):
    """一组同规格物体的摆法。

    摆法是栅格：从 (origin_x, origin_y) 起，x 方向 rows 行、y 方向 cols 列，
    步长为「物体尺寸 + 间隙」。
    """

    __slots__ = ("kind", "count", "width", "length", "rows", "cols",
                 "gap_x", "gap_y", "origin_x", "origin_y", "yaw", "blueprint")

    def __init__(self, kind, width, length, rows=1, cols=1,
                 gap_x=0.0, gap_y=0.0, origin_x=0.0, origin_y=0.0,
                 yaw=0.0, blueprint="", count=None):
        if not kind:
            raise LayouterError("物体组缺少 kind")
        self.kind = "%s" % kind
        self.width = _positive(width, "object.width")
        self.length = _positive(length, "object.length")
        self.rows = _as_int(rows, "object.rows")
        self.cols = _as_int(cols, "object.cols")
        if self.rows <= 0 or self.cols <= 0:
            raise LayouterError("rows/cols 必须为正整数：%d x %d" % (self.rows, self.cols))
        self.gap_x = _non_negative(gap_x, "object.gap_x")
        self.gap_y = _non_negative(gap_y, "object.gap_y")
        self.origin_x = _as_float(origin_x, "object.origin_x")
        self.origin_y = _as_float(origin_y, "object.origin_y")
        self.yaw = normalize_angle(yaw)
        self.blueprint = "%s" % (blueprint or "")
        if count is None:
            self.count = self.rows * self.cols
        else:
            self.count = _as_int(count, "object.count")
            if self.count != self.rows * self.cols:
                raise LayouterError(
                    "count(%d) 与 rows*cols(%d) 不一致" % (self.count, self.rows * self.cols))

    def pitch_x(self):
        return self.width + self.gap_x

    def pitch_y(self):
        return self.length + self.gap_y

    def as_tuple(self):
        return (self.kind, self.count, self.width, self.length,
                self.rows, self.cols, self.gap_x, self.gap_y,
                self.origin_x, self.origin_y, self.yaw, self.blueprint)

    def __repr__(self):
        return ("ObjectSpec(%s, %dx%d, %.3fx%.3f, gap=%.3f/%.3f, origin=(%.3f, %.3f))"
                % (self.kind, self.rows, self.cols, self.width, self.length,
                   self.gap_x, self.gap_y, self.origin_x, self.origin_y))


class PlacedObject(object):
    """摆好之后的一个物体。"""

    __slots__ = ("index", "group", "kind", "rect", "blueprint", "row", "col")

    def __init__(self, index, group, kind, rect, blueprint="", row=0, col=0):
        self.index = int(index)
        self.group = "%s" % group
        self.kind = "%s" % kind
        self.rect = rect
        self.blueprint = "%s" % (blueprint or "")
        self.row = int(row)
        self.col = int(col)

    def as_row(self):
        return (self.index, self.group, self.kind, self.rect.x, self.rect.y,
                self.rect.yaw, self.rect.width, self.rect.length, self.blueprint)

    def __repr__(self):
        return ("PlacedObject(%d, %s, x=%.3f, y=%.3f, yaw=%.3f)"
                % (self.index, self.kind, self.rect.x, self.rect.y, self.rect.yaw))


class Scene(object):
    """一个完整的场景：场地 + 若干物体组。"""

    __slots__ = ("name", "field", "groups", "source", "min_gap")

    def __init__(self, name="scene", field=None, groups=None,
                 source="", min_gap=DEFAULT_MIN_GAP):
        self.name = "%s" % (name or "scene")
        self.field = field if field is not None else Field()
        self.groups = list(groups or [])
        self.source = "%s" % (source or "")
        self.min_gap = _non_negative(min_gap, "scene.min_gap")

    def total_objects(self):
        return sum(group.count for group in self.groups)

    def __repr__(self):
        return ("Scene(%s, field=%s, groups=%d, objects=%d)"
                % (self.name, self.field.as_tuple(), len(self.groups),
                   self.total_objects()))


# ---------------------------------------------------------------- 布局


def layout_group(spec, start_index=0):
    """把一组规格按栅格展开成具体的物体列表。"""
    placed = []
    index = start_index
    for row in range(spec.rows):
        for col in range(spec.cols):
            x = spec.origin_x + col * spec.pitch_x()
            y = spec.origin_y + row * spec.pitch_y()
            rect = Rect(x, y, spec.width, spec.length, spec.yaw)
            placed.append(PlacedObject(index, spec.kind, spec.kind, rect,
                                       spec.blueprint, row, col))
            index += 1
    return placed


def layout_scene(scene):
    """把整个场景展开。"""
    placed = []
    for spec in scene.groups:
        placed.extend(layout_group(spec, len(placed)))
    return placed


def bounds_of(placed):
    if not placed:
        return (0.0, 0.0, 0.0, 0.0)
    mins = [item.rect.bounds()[0] for item in placed]
    miny = [item.rect.bounds()[1] for item in placed]
    maxx = [item.rect.bounds()[2] for item in placed]
    maxy = [item.rect.bounds()[3] for item in placed]
    return (min(mins), min(miny), max(maxx), max(maxy))


def min_gap_of(placed):
    """所有物体两两之间的最小间距（没重叠的前提下）。"""
    if len(placed) < 2:
        return None
    best = None
    corners = [item.rect.corners() for item in placed]
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            gap = polygon_gap(corners[i], corners[j])
            if best is None or gap < best:
                best = gap
    return best


def overlapping_pairs(placed):
    """返回所有重叠的物体对（索引元组）。"""
    pairs = []
    corners = [item.rect.corners() for item in placed]
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if rectangles_overlap(corners[i], corners[j]):
                pairs.append((placed[i].index, placed[j].index))
    return pairs


# ---------------------------------------------------------------- 校验


def _result(kind, passed, detail):
    return {"type": kind, "passed": bool(passed), "detail": detail}


def check_layout(placed, field=None, min_gap=DEFAULT_MIN_GAP, expected_count=None):
    """四条断言：边界内 / 不重叠 / 间距够 / 数量对。"""
    field = field if field is not None else Field()
    results = []

    outside = [item for item in placed if not field.contains(item.rect.corners())]
    if outside:
        worst = outside[0]
        detail = ("%d/%d 个物体超出场地，首个 %s 位于 (%.2f, %.2f)"
                  % (len(outside), len(placed), worst.kind, worst.rect.x, worst.rect.y))
    else:
        detail = "全部 %d 个物体都在 %.1f x %.1f 的场地内" % (
            len(placed), field.width, field.length)
    results.append(_result("边界内", not outside, detail))

    pairs = overlapping_pairs(placed)
    if pairs:
        detail = "存在 %d 对重叠，例如 #%d 与 #%d" % (len(pairs), pairs[0][0], pairs[0][1])
    else:
        detail = "%d 个物体两两不重叠" % len(placed)
    results.append(_result("不重叠", not pairs, detail))

    actual_gap = min_gap_of(placed)
    if actual_gap is None:
        passed = True
        detail = "物体不足 2 个，无需检查间距"
    else:
        passed = actual_gap + 1e-6 >= min_gap
        detail = "最小间距 %.3f m，要求 >= %.3f m" % (actual_gap, min_gap)
    results.append(_result("最小间距", passed, detail))

    if expected_count is None:
        expected_count = len(placed)
    passed = expected_count == len(placed)
    detail = "摆放 %d 个，期望 %d 个" % (len(placed), expected_count)
    results.append(_result("物体数量", passed, detail))

    return results


def all_passed(results):
    for item in results:
        if not item["passed"]:
            return False
    return True


# ---------------------------------------------------------------- 输出


def _symbol_for(index, group_names):
    """不同物体组用不同字符，方便肉眼看图。"""
    symbols = "#@%&*+=~^o"
    try:
        slot = group_names.index(index)
    except ValueError:
        slot = 0
    return symbols[slot % len(symbols)]


def render_scene(placed, field=None, width=DEFAULT_GRID_WIDTH,
                 height=DEFAULT_GRID_HEIGHT):
    """把场景画成字符俯视图，返回每一行的字符串列表（y 轴向上，输出时翻转）。"""
    field = field if field is not None else Field()
    width = max(10, int(width))
    height = max(6, int(height))

    group_names = []
    for item in placed:
        if item.kind not in group_names:
            group_names.append(item.kind)

    cells = [[" "] * width for _ in range(height)]

    def to_cell(x, y):
        cx = int(round(x / field.width * (width - 1)))
        cy = int(round(y / field.length * (height - 1)))
        cx = min(width - 1, max(0, cx))
        cy = min(height - 1, max(0, cy))
        return cx, cy

    # 先画场地边框
    for cx in range(width):
        for cy in (0, height - 1):
            cells[cy][cx] = "-"
    for cy in range(height):
        for cx in (0, width - 1):
            cells[cy][cx] = "|"

    for item in placed:
        symbol = _symbol_for(item.group, group_names)
        corners = item.rect.corners()
        cell_corners = [to_cell(x, y) for x, y in corners]
        for index in range(len(cell_corners)):
            x1, y1 = cell_corners[index]
            x2, y2 = cell_corners[(index + 1) % len(cell_corners)]
            steps = max(abs(x2 - x1), abs(y2 - y1), 1)
            for step in range(steps + 1):
                t = step / float(steps)
                cx = int(round(x1 + (x2 - x1) * t))
                cy = int(round(y1 + (y2 - y1) * t))
                if 0 <= cy < height and 0 <= cx < width:
                    cells[cy][cx] = symbol

    lines = []
    for cy in range(height - 1, -1, -1):
        lines.append("".join(cells[cy]))
    return lines


def format_report(scene, placed, results, header=""):
    """终端报告：标题 + 汇总 + 四条断言 + 俯视图。"""
    lines = []
    if header:
        lines.append(header)
    lines.append("场景：%s%s" % (scene.name,
                                ("（来自 %s）" % scene.source) if scene.source else ""))
    lines.append("场地：%.2f m × %.2f m，物体组 %d 个，物体总数 %d 个"
                 % (scene.field.width, scene.field.length,
                    len(scene.groups), len(placed)))
    for spec in scene.groups:
        lines.append("  - %-10s %d×%d，单个 %.2f×%.2f m，间隙 %.2f/%.2f m，起点 (%.2f, %.2f)，朝向 %.2f rad"
                     % (spec.kind, spec.rows, spec.cols, spec.width, spec.length,
                        spec.gap_x, spec.gap_y, spec.origin_x, spec.origin_y, spec.yaw))

    if placed:
        bounds = bounds_of(placed)
        actual_gap = min_gap_of(placed)
        lines.append("包围盒：x ∈ [%.2f, %.2f]，y ∈ [%.2f, %.2f]"
                     % (bounds[0], bounds[2], bounds[1], bounds[3]))
        if actual_gap is not None:
            lines.append("最小间距：%.3f m" % actual_gap)

    passed = len([item for item in results if item["passed"]])
    lines.append("断言：%d 通过 / %d 失败（共 %d 条）"
                 % (passed, len(results) - passed, len(results)))
    for item in results:
        lines.append("  [%s] %-6s %s" % ("PASS" if item["passed"] else "FAIL",
                                         item["type"], item["detail"]))
    lines.append("结论：%s" % ("PASSED" if all_passed(results) else "FAILED"))
    lines.append("")
    lines.extend(render_scene(placed, scene.field))
    return "\n".join(lines)


CSV_HEADER = "index,group,kind,x,y,yaw,width,length,blueprint"


def to_csv(placed):
    """物体清单导出成 CSV 文本（不写文件，便于测试）。"""
    lines = [CSV_HEADER]
    for item in placed:
        row = item.as_row()
        lines.append("%d,%s,%s,%.4f,%.4f,%.4f,%.3f,%.3f,%s"
                     % (row[0], row[1], row[2], row[3], row[4],
                        row[5], row[6], row[7], row[8]))
    return "\n".join(lines) + "\n"


def _json_escape(text):
    out = []
    for char in "%s" % text:
        if char == "\\":
            out.append("\\\\")
        elif char == '"':
            out.append('\\"')
        elif char == "\n":
            out.append("\\n")
        elif ord(char) < 32:
            out.append("\\u%04x" % ord(char))
        else:
            out.append(char)
    return "".join(out)


def to_json(scene, placed, results):
    """汇总导出成 JSON 文本（自己拼，不依赖 json 模块的行为差异）。"""
    bounds = bounds_of(placed)
    actual_gap = min_gap_of(placed)
    parts = []
    parts.append('  "name": "%s"' % _json_escape(scene.name))
    parts.append('  "field": {"width": %.3f, "length": %.3f}'
                 % (scene.field.width, scene.field.length))
    parts.append('  "count": %d' % len(placed))
    parts.append('  "bounds": {"min_x": %.4f, "min_y": %.4f, "max_x": %.4f, "max_y": %.4f}'
                 % (bounds[0], bounds[1], bounds[2], bounds[3]))
    if actual_gap is None:
        parts.append('  "min_gap": null')
    else:
        parts.append('  "min_gap": %.4f' % actual_gap)
    parts.append('  "passed": %s' % ("true" if all_passed(results) else "false"))
    checks = []
    for item in results:
        checks.append('    {"type": "%s", "passed": %s, "detail": "%s"}'
                      % (_json_escape(item["type"]),
                         "true" if item["passed"] else "false",
                         _json_escape(item["detail"])))
    parts.append('  "checks": [\n%s\n  ]' % ",\n".join(checks))
    objects = []
    for item in placed:
        objects.append('    {"index": %d, "kind": "%s", "x": %.4f, "y": %.4f, "yaw": %.4f}'
                       % (item.index, _json_escape(item.kind),
                          item.rect.x, item.rect.y, item.rect.yaw))
    parts.append('  "objects": [\n%s\n  ]' % ",\n".join(objects))
    return "{\n" + ",\n".join(parts) + "\n}\n"


# ---------------------------------------------------------------- 场景文件解析
# 只支持本模块用到的那一小撮 YAML：缩进映射、以 "- " 开头的列表、行内 # 注释。
# 故意不引入 PyYAML —— Melodic 上装 PyYAML 要 pip，且仓库其它模块也没依赖它。


def _strip_comment(line):
    """去掉行内注释；引号里的 # 不当注释。"""
    out = []
    quote = None
    for char in line:
        if quote:
            out.append(char)
            if char == quote:
                quote = None
            continue
        if char in ("'", '"'):
            quote = char
            out.append(char)
            continue
        if char == "#":
            break
        out.append(char)
    return "".join(out).rstrip()


def _parse_scalar(text):
    text = text.strip()
    if not text:
        return ""
    if (text[0] == text[-1]) and text[0] in ("'", '"') and len(text) >= 2:
        return text[1:-1]
    lowered = text.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if lowered in ("null", "none", "~"):
        return None
    try:
        if text.isdigit() or (text.startswith("-") and text[1:].isdigit()):
            return int(text)
        return float(text)
    except (TypeError, ValueError):
        return text


def _tokenize(text, source=""):
    tokens = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = _strip_comment(raw)
        if not line.strip():
            continue
        indent = len(line) - len(line.lstrip(" "))
        body = line.strip()
        tokens.append((indent, body, number))
    if not tokens:
        raise LayouterError("%s 是空场景" % (source or "场景文件"))
    return tokens


def _parse_block(tokens, index, indent):
    if index < len(tokens) and tokens[index][1].startswith("- "):
        return _parse_list(tokens, index, indent)
    return _parse_map(tokens, index, indent)


def _parse_map(tokens, index, indent):
    result = {}
    while index < len(tokens):
        current_indent, body, number = tokens[index]
        if current_indent < indent:
            break
        if current_indent > indent:
            raise LayouterError("第 %d 行缩进异常：%r" % (number, body))
        if body.startswith("- "):
            break
        if ":" not in body:
            raise LayouterError("第 %d 行缺少冒号：%r" % (number, body))
        key, _, value = body.partition(":")
        key = key.strip()
        value = value.strip()
        index += 1
        if value:
            result[key] = _parse_scalar(value)
            continue
        if index < len(tokens) and tokens[index][0] > current_indent:
            child, index = _parse_block(tokens, index, tokens[index][0])
            result[key] = child
        elif index < len(tokens) and tokens[index][0] == current_indent and \
                tokens[index][1].startswith("- "):
            child, index = _parse_list(tokens, index, current_indent)
            result[key] = child
        else:
            result[key] = ""
    return result, index


def _parse_list(tokens, index, indent):
    result = []
    while index < len(tokens):
        current_indent, body, number = tokens[index]
        if current_indent < indent:
            break
        if current_indent > indent:
            raise LayouterError("第 %d 行缩进异常：%r" % (number, body))
        if not body.startswith("- "):
            break
        item_body = body[2:].strip()
        index += 1
        if not item_body:
            if index < len(tokens) and tokens[index][0] > current_indent:
                child, index = _parse_block(tokens, index, tokens[index][0])
                result.append(child)
            else:
                result.append("")
            continue
        if ":" in item_body and not item_body.startswith(("'", '"')):
            key, _, value = item_body.partition(":")
            entry = {}
            key = key.strip()
            value = value.strip()
            if value:
                entry[key] = _parse_scalar(value)
            elif index < len(tokens) and tokens[index][0] > current_indent:
                child, index = _parse_block(tokens, index, tokens[index][0])
                entry[key] = child
            else:
                entry[key] = ""
            if index < len(tokens) and tokens[index][0] > current_indent:
                rest, index = _parse_map(tokens, index, tokens[index][0])
                entry.update(rest)
            result.append(entry)
            continue
        result.append(_parse_scalar(item_body))
    return result, index


def build_scene(body, source=""):
    """把解析出来的 dict 组装成 Scene，并逐项校验。"""
    if not isinstance(body, dict):
        raise LayouterError("%s 的顶层必须是映射（key: value）" % (source or "场景"))
    name = body.get("name") or "scene"

    field_body = body.get("field")
    if isinstance(field_body, dict):
        width = field_body.get("width", DEFAULT_FIELD_WIDTH)
        length = field_body.get("length", DEFAULT_FIELD_LENGTH)
    else:
        width = DEFAULT_FIELD_WIDTH
        length = DEFAULT_FIELD_LENGTH
    field = Field(width, length)

    min_gap = body.get("min_gap", DEFAULT_MIN_GAP)
    groups_body = body.get("objects") or body.get("groups") or []
    if not isinstance(groups_body, list) or not groups_body:
        raise LayouterError("%s 里没有任何物体组（objects:）" % (source or "场景"))

    groups = []
    for position, entry in enumerate(groups_body):
        if not isinstance(entry, dict):
            raise LayouterError("%s 的第 %d 个物体组不是映射" % (source or "场景", position + 1))
        if "kind" not in entry:
            raise LayouterError("%s 的第 %d 个物体组缺少 kind"
                                % (source or "场景", position + 1))
        groups.append(ObjectSpec(
            kind=entry.get("kind"),
            width=entry.get("width"),
            length=entry.get("length"),
            rows=entry.get("rows", 1),
            cols=entry.get("cols", 1),
            gap_x=entry.get("gap_x", 0.0),
            gap_y=entry.get("gap_y", 0.0),
            origin_x=entry.get("origin_x", 0.0),
            origin_y=entry.get("origin_y", 0.0),
            yaw=entry.get("yaw", 0.0),
            blueprint=entry.get("blueprint", ""),
            count=entry.get("count"),
        ))

    return Scene(name=name, field=field, groups=groups,
                 source=source, min_gap=min_gap)


def parse_scene_text(text, source=""):
    """从字符串解析场景。"""
    tokens = _tokenize(text, source)
    body, index = _parse_block(tokens, 0, tokens[0][0])
    if index < len(tokens):
        raise LayouterError("%s 第 %d 行之后还有多余内容"
                            % (source or "场景", tokens[index][2]))
    return build_scene(body, source)


def load_scene(path):
    """从文件读取场景（UTF-8）。"""
    with io.open(path, "r", encoding="utf-8") as handle:
        return parse_scene_text(handle.read(), source=path)


# ---------------------------------------------------------------- 参数


_PARAM_DEFAULTS = {
    "scene_file": "",
    "field_width": DEFAULT_FIELD_WIDTH,
    "field_length": DEFAULT_FIELD_LENGTH,
    "min_gap": DEFAULT_MIN_GAP,
    "grid_width": DEFAULT_GRID_WIDTH,
    "grid_height": DEFAULT_GRID_HEIGHT,
    "csv_path": DEFAULT_CSV_PATH,
    "json_path": DEFAULT_JSON_PATH,
    "write_files": True,
    "publish_hz": DEFAULT_PUBLISH_HZ,
    "topic_objects": DEFAULT_TOPIC_OBJECTS,
    "topic_summary": DEFAULT_TOPIC_SUMMARY,
    "topic_trigger": DEFAULT_TOPIC_TRIGGER,
}

PARAM_KEYS = (
    "scene/file",
    "field/width",
    "field/length",
    "check/min_gap",
    "render/width",
    "render/height",
    "io/csv_path",
    "io/json_path",
    "io/write_files",
    "rate/publish_hz",
    "topic/objects",
    "topic/summary",
    "topic/trigger",
)


class LayouterParams(object):
    """节点参数（也是 CLI / launch 参数的一一映射）。"""

    __slots__ = tuple(_PARAM_DEFAULTS.keys())

    def __init__(self, **kwargs):
        for key, default in _PARAM_DEFAULTS.items():
            setattr(self, key, kwargs.get(key, default))

    @classmethod
    def from_params(cls, get_param):
        """按 13 个键从 rosparam 读出参数；get_param 需支持 (name, default)。"""
        return cls(
            scene_file="%s" % (get_param("scene/file", "") or ""),
            field_width=_positive(get_param("field/width", DEFAULT_FIELD_WIDTH), "field/width"),
            field_length=_positive(get_param("field/length", DEFAULT_FIELD_LENGTH), "field/length"),
            min_gap=_non_negative(get_param("check/min_gap", DEFAULT_MIN_GAP), "check/min_gap"),
            grid_width=_as_int(get_param("render/width", DEFAULT_GRID_WIDTH), "render/width"),
            grid_height=_as_int(get_param("render/height", DEFAULT_GRID_HEIGHT), "render/height"),
            csv_path="%s" % (get_param("io/csv_path", DEFAULT_CSV_PATH) or ""),
            json_path="%s" % (get_param("io/json_path", DEFAULT_JSON_PATH) or ""),
            write_files=as_bool(get_param("io/write_files", True), True),
            publish_hz=_positive(get_param("rate/publish_hz", DEFAULT_PUBLISH_HZ), "rate/publish_hz"),
            topic_objects="%s" % (get_param("topic/objects", DEFAULT_TOPIC_OBJECTS)
                                  or DEFAULT_TOPIC_OBJECTS),
            topic_summary="%s" % (get_param("topic/summary", DEFAULT_TOPIC_SUMMARY)
                                  or DEFAULT_TOPIC_SUMMARY),
            topic_trigger="%s" % (get_param("topic/trigger", DEFAULT_TOPIC_TRIGGER)
                                  or DEFAULT_TOPIC_TRIGGER),
        )

    def as_tuple(self):
        return tuple(getattr(self, key) for key in _PARAM_DEFAULTS)

    def __repr__(self):
        return "LayouterParams(%s)" % ", ".join(
            "%s=%r" % (key, getattr(self, key)) for key in _PARAM_DEFAULTS)
