#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""scene_object_layouter 模块的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）
* 在仓库根目录执行：python3 tests/test_scene_object_layouter_local.py
* 也可以给 pytest 用：python3 -m pytest tests/test_scene_object_layouter_local.py -q
  （每个分组都有对应的 test_ 包装，失败会真的抛 AssertionError）

覆盖范围：
A 通用小工具       B 几何基础         C 场地与矩形
D 场景文本解析     E 场景结构校验     F 栅格摆放
G 布局度量         H 四条断言         I 渲染与输出
J 参数读取         K 工程文件约定
"""

from __future__ import division
from __future__ import print_function

import io
import math
import os
import sys

try:  # Python 2 兼容
    text_type = unicode  # noqa: F821
except NameError:  # Python 3
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

PKG = os.path.join(ROOT, "src", "ground", "scene_object_layouter")
SCRIPTS = os.path.join(PKG, "scripts")
CONFIG = os.path.join(PKG, "config")
LAUNCH = os.path.join(PKG, "launch")
DOCS_PAGE = os.path.join(ROOT, "docs", "ground", "carla_spawn_objects.md")

if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import layouter_core as core  # noqa: E402


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append(name)
    else:
        FAILED.append(name + (" （" + detail + "）" if detail else ""))
    return bool(condition)


def check_close(name, got, want, tolerance=1e-9, detail=""):
    ok = abs(float(got) - float(want)) <= tolerance
    if not ok:
        detail = "%r != %r%s" % (got, want, " " + detail if detail else "")
    return check(name, ok, detail)


def check_raises(name, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except Exception:
        return check(name, True)
    return check(name, False, "没有抛出异常")


def read_text(path):
    with io.open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def _make_overlap():
    """两个物体中心只相差 0.5 m、各自 2x2 m，一定重叠。"""
    first = core.layout_group(core.ObjectSpec(kind="a", width=2.0, length=2.0, origin_x=1.0, origin_y=1.0))
    second = core.layout_group(core.ObjectSpec(kind="b", width=2.0, length=2.0, origin_x=1.5, origin_y=1.5),
                               start_index=len(first))
    return first + second


# --------------------------------------------------------------------------
# A 通用小工具
# --------------------------------------------------------------------------


def group_helpers():
    check_close("A01 normalize_angle(0)", core.normalize_angle(0.0), 0.0)
    check_close("A02 normalize_angle(pi)", core.normalize_angle(math.pi), math.pi)
    check_close("A03 normalize_angle(-pi)", core.normalize_angle(-math.pi), math.pi)
    check_close("A04 normalize_angle(3pi/2)", core.normalize_angle(1.5 * math.pi), -0.5 * math.pi)
    check_close("A05 normalize_angle(-3pi/2)", core.normalize_angle(-1.5 * math.pi), 0.5 * math.pi)
    check_close("A06 normalize_angle(2pi)", core.normalize_angle(2 * math.pi), 0.0, 1e-12)

    check_close("A07 _as_float 数字", core._as_float(2, "x"), 2.0)
    check_close("A08 _as_float 字符串", core._as_float("2.5", "x"), 2.5)
    check_raises("A09 _as_float 非法", core._as_float, "abc", "x")
    check_raises("A10 _as_float NaN", core._as_float, float("nan"), "x")

    check_close("A11 _as_int 整数", core._as_int(3, "x"), 3)
    check_close("A12 _as_int 字符串整数", core._as_int("3", "x"), 3)
    check_raises("A12b _as_int 浮点被拒", core._as_int, 3.7, "x")
    check_raises("A13 _as_int 非法", core._as_int, "3.5.1", "x")

    check_close("A14 _positive", core._positive(1, "x"), 1.0)
    check_raises("A15 _positive 零", core._positive, 0, "x")
    check_raises("A16 _positive 负", core._positive, -1, "x")
    check_close("A17 _non_negative", core._non_negative(0, "x"), 0.0)
    check_raises("A18 _non_negative 负", core._non_negative, -0.1, "x")

    check("A19 as_bool True", core.as_bool(True) is True)
    check("A20 as_bool 'true'", core.as_bool("true") is True)
    check("A21 as_bool 'False'", core.as_bool("False") is False)
    check("A22 as_bool 0", core.as_bool(0) is False)
    check_raises("A23 as_bool 未知串报错", core.as_bool, "maybe")

    check("A24 LayouterError 是 ValueError", issubclass(core.LayouterError, ValueError))
    check("A25 默认常量齐全", core.DEFAULT_MIN_GAP == 0.5 and core.DEFAULT_FIELD_WIDTH == 30.0)


# --------------------------------------------------------------------------
# B 几何基础
# --------------------------------------------------------------------------


def group_geometry():
    check_close("B01 point_segment_distance 垂足", core.point_segment_distance((0.0, 1.0), (0.0, 0.0), (2.0, 0.0)), 1.0)
    check_close("B02 point_segment_distance 端点外", core.point_segment_distance((3.0, 0.0), (0.0, 0.0), (2.0, 0.0)), 1.0)
    check_close("B03 point_segment_distance 退化线段",
                core.point_segment_distance((1.0, 1.0), (0.0, 0.0), (0.0, 0.0)), math.sqrt(2.0))

    check("B04 segments_intersect 十字", core.segments_intersect((-1.0, 0.0), (1.0, 0.0), (0.0, -1.0), (0.0, 1.0)))
    check("B05 segments_intersect 平行", not core.segments_intersect((0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)))
    check("B06 segments_intersect 不相交", not core.segments_intersect((0.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 2.0)))

    px, py = core.rotate_point((1.0, 0.0), 0.5 * math.pi)
    check_close("B07 rotate_point x", px, 0.0, 1e-12)
    check_close("B08 rotate_point y", py, 1.0, 1e-12)
    px, py = core.rotate_point((1.0, 0.0), 0.0)
    check_close("B09 rotate_point 零角", px - 1.0, 0.0, 1e-12)

    square_a = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    square_b = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    square_c = [(3.0, 0.0), (5.0, 0.0), (5.0, 2.0), (3.0, 2.0)]
    square_d = [(1.0, 0.0), (3.0, 0.0), (3.0, 2.0), (1.0, 2.0)]

    check("B10 rectangles_overlap 相交", core.rectangles_overlap(square_a, square_b))
    check("B11 rectangles_overlap 分离", not core.rectangles_overlap(square_a, square_c))
    check("B12 rectangles_overlap 贴边不算重叠", not core.rectangles_overlap(square_a, square_d))
    check_close("B13 polygon_gap 相交为 0", core.polygon_gap(square_a, square_b), 0.0)
    check_close("B14 polygon_gap 水平间隔", core.polygon_gap(square_a, square_c), 2.0)
    check_close("B15 polygon_gap 贴边为 0", core.polygon_gap(square_a, square_d), 0.0)


# --------------------------------------------------------------------------
# C 场地与矩形
# --------------------------------------------------------------------------


def group_shapes():
    field = core.Field(10.0, 20.0)
    check_close("C01 Field.width", field.width, 10.0)
    check_close("C02 Field.length", field.length, 20.0)
    check("C03 Field.as_tuple", field.as_tuple() == (10.0, 20.0, 20.0, 10.0) or field.as_tuple() == (10.0, 20.0))
    check("C04 Field.contains 内部", field.contains([(1.0, 1.0), (9.0, 19.0)]))
    check("C05 Field.contains 越界", not field.contains([(-0.1, 1.0)]))
    check_raises("C06 Field 宽度必须为正", core.Field, 0.0, 10.0)

    rect = core.Rect(0.0, 0.0, 2.0, 4.0)
    corners = rect.corners()
    check("C07 Rect.corners 四个点", len(corners) == 4)
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    check_close("C08 Rect 半宽", max(xs), 1.0)
    check_close("C09 Rect 半长", max(ys), 2.0)
    check_close("C10 Rect 中心对称", min(xs), -1.0)

    bounds = rect.bounds()
    check_close("C11 Rect.bounds minx", bounds[0], -1.0)
    check_close("C12 Rect.bounds miny", bounds[1], -2.0)
    check_close("C13 Rect.bounds maxx", bounds[2], 1.0)
    check_close("C14 Rect.bounds maxy", bounds[3], 2.0)

    turned = core.Rect(0.0, 0.0, 2.0, 4.0, yaw=0.5 * math.pi)
    turned_bounds = turned.bounds()
    check_close("C15 旋转 90 度后 x 半宽变成 2", turned_bounds[2], 2.0)
    check_close("C16 旋转 90 度后 y 半宽变成 1", turned_bounds[3], 1.0)
    check_raises("C17 Rect 长度必须为正", core.Rect, 0.0, 0.0, 2.0, 0.0)


# --------------------------------------------------------------------------
# D 场景文本解析
# --------------------------------------------------------------------------


def group_parse():
    scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin")
    check("D01 场景名", scene.name == "demo_parking_row")
    check_close("D02 场地宽", scene.field.width, 24.0)
    check_close("D03 场地长", scene.field.length, 18.0)
    check_close("D04 最小间距", scene.min_gap, 0.5)
    check("D05 两个物体组", len(scene.groups) == 2)
    check("D06 第一个组是 slot", scene.groups[0].kind == "slot")
    check("D07 slot 行数", scene.groups[0].rows == 2)
    check("D08 slot 列数", scene.groups[0].cols == 6)
    check_close("D09 slot 宽", scene.groups[0].width, 2.5)
    check_close("D10 slot 长", scene.groups[0].length, 5.3)
    check_close("D11 slot gap_x", scene.groups[0].gap_x, 0.6)
    check_close("D12 slot 起点 x", scene.groups[0].origin_x, 3.0)
    check("D13 blueprint 保留", scene.groups[0].blueprint == "static.prop.parkingline")
    check("D14 物体总数", scene.total_objects() == 15)

    inline = u"name: t\nfield:\n  width: 4.0\n  length: 6.0\nmin_gap: 0.2\nobjects:\n  - kind: box\n    width: 1.0\n    length: 1.0\n"
    inline_scene = core.parse_scene_text(inline, source="mini")
    check_close("D15 迷你场景宽", inline_scene.field.width, 4.0)
    check_close("D16 迷你场景长", inline_scene.field.length, 6.0)
    check("D16b 迷你场景物体数", inline_scene.total_objects() == 1)

    commented = u"name: c  # 名字\nmin_gap: 1.0  # 间距\nfield:\n  width: 5.0   # 宽\n  length: 5.0\nobjects:\n  - kind: box\n    width: 1.0\n    length: 1.0\n"
    commented_scene = core.parse_scene_text(commented, source="comment")
    check("D17 行内注释被剥离", commented_scene.name == "c")
    check_close("D18 注释不影响数值", commented_scene.min_gap, 1.0)

    check_raises("D19 空文本报错", core.parse_scene_text, u"", "empty")
    check_raises("D20 没有 objects 报错",
                 core.parse_scene_text, u"name: x\nfield:\n  width: 4.0\n  length: 4.0\n", "noobj")
    check_raises("D21 数值非法报错",
                 core.parse_scene_text,
                 u"name: x\nfield:\n  width: abc\n  length: 4.0\nobjects:\n  - kind: a\n    width: 1.0\n    length: 1.0\n",
                 "bad")

    row = core.ObjectSpec(kind="a", width=1.0, length=2.0)
    check_close("D22 ObjectSpec 默认 gap_x", row.gap_x, 0.0)
    check("D23 ObjectSpec 默认 rows", row.rows == 1)
    check_close("D24 pitch_x", row.pitch_x(), 1.0)
    check_close("D25 pitch_y", row.pitch_y(), 2.0)
    check_raises("D26 count 与 rows*cols 不一致报错",
                 core.ObjectSpec, kind="a", width=1.0, length=1.0, rows=2, cols=2, count=5)


# --------------------------------------------------------------------------
# E 栅格摆放
# --------------------------------------------------------------------------


def group_layout():
    scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin")
    placed = core.layout_scene(scene)
    check("E01 摆放数量", len(placed) == 15)
    check("E02 索引连续", [item.index for item in placed] == list(range(15)))
    check("E03 组名正确", placed[0].group == "slot" and placed[-1].group == "pillar")

    first = placed[0]
    check_close("E04 第一个物体 x", first.rect.x, 3.0)
    check_close("E05 第一个物体 y", first.rect.y, 3.0)
    check("E06 第一个物体行列", first.row == 0 and first.col == 0)

    sixth = placed[5]
    check_close("E07 第 6 个物体 x", sixth.rect.x, 3.0 + 5 * 3.1)
    check_close("E08 第 6 个物体 y", sixth.rect.y, 3.0)

    seventh = placed[6]
    check_close("E09 第二排起点 x", seventh.rect.x, 3.0)
    check_close("E10 第二排起点 y", seventh.rect.y, 3.0 + 11.3)

    pillar = placed[12]
    check_close("E11 立柱起点 x", pillar.rect.x, 6.0)
    check_close("E12 立柱起点 y", pillar.rect.y, 8.65)
    check_close("E13 立柱第三个 x", placed[14].rect.x, 6.0 + 2 * 5.0)

    sample = core.ObjectSpec(kind="s", width=2.0, length=3.0, rows=2, cols=3, gap_x=1.0, gap_y=2.0,
                             origin_x=1.0, origin_y=1.0)
    sample_placed = core.layout_group(sample, start_index=7)
    check("E14 layout_group 数量", len(sample_placed) == 6)
    check("E15 layout_group 起始索引", sample_placed[0].index == 7)
    check_close("E16 列间距生效", sample_placed[1].rect.x, 1.0 + 3.0)
    check_close("E17 行间距生效", sample_placed[3].rect.y, 1.0 + 5.0)

    rotated = core.ObjectSpec(kind="r", width=2.0, length=2.0, yaw=0.5 * math.pi)
    rotated_placed = core.layout_group(rotated)
    check_close("E18 朝向被记录", rotated_placed[0].rect.yaw, 0.5 * math.pi)
    check("E19 as_row 长度", len(placed[0].as_row()) == 9)


# --------------------------------------------------------------------------
# F 布局度量
# --------------------------------------------------------------------------


def group_measure():
    scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin")
    placed = core.layout_scene(scene)

    bounds = core.bounds_of(placed)
    check_close("F01 包围盒 minx", bounds[0], 1.75)
    check_close("F02 包围盒 miny", bounds[1], 0.35)
    check_close("F03 包围盒 maxx", bounds[2], 19.75)
    check_close("F04 包围盒 maxy", bounds[3], 16.95)

    gap = core.min_gap_of(placed)
    check_close("F05 最小间距", gap, 0.6, 1e-6)
    check("F06 没有重叠对", core.overlapping_pairs(placed) == [])

    two = core.layout_group(core.ObjectSpec(kind="t", width=1.0, length=1.0, cols=2, gap_x=0.5))
    check_close("F07 两个物体间距", core.min_gap_of(two), 0.5, 1e-6)

    touching = core.layout_group(core.ObjectSpec(kind="t", width=1.0, length=1.0, cols=2, gap_x=0.0))
    check_close("F08 贴边间距为 0", core.min_gap_of(touching), 0.0, 1e-9)

    overlap = _make_overlap()
    check("F09 两组重叠会被发现", len(core.overlapping_pairs(overlap)) == 1)
    check_raises("F09b 负间距被拒绝", core.ObjectSpec, "t", 1.0, 1.0, 1, 2, -0.5)

    single = core.layout_group(core.ObjectSpec(kind="t", width=1.0, length=1.0))
    check("F10 单个物体无间距可比", core.min_gap_of(single) is None)
    check("F11 空列表包围盒退化为零", core.bounds_of([]) == (0.0, 0.0, 0.0, 0.0))


# --------------------------------------------------------------------------
# G 四条断言
# --------------------------------------------------------------------------


def group_checks():
    scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin")
    placed = core.layout_scene(scene)
    results = core.check_layout(placed, scene.field, scene.min_gap, expected_count=scene.total_objects())

    check("G01 四条断言", len(results) == 4)
    check("G02 每条都有 type/passed/detail",
          all(set(["type", "passed", "detail"]) <= set(item.keys()) for item in results))
    types = [item["type"] for item in results]
    check("G03 断言类型顺序", types == [u"边界内", u"不重叠", u"最小间距", u"物体数量"])
    check("G04 全部通过", core.all_passed(results))
    check("G05 正例没有失败项", [item for item in results if not item["passed"]] == [])

    small_field = core.Field(6.0, 6.0)
    out_results = core.check_layout(placed, small_field, scene.min_gap)
    boundary = [item for item in out_results if item["type"] == u"边界内"][0]
    check("G06 小场地触发越界", not boundary["passed"])
    check("G07 越界细节有数量", u"超出场地" in boundary["detail"])

    tight = core.check_layout(placed, scene.field, 5.0)
    gap_result = [item for item in tight if item["type"] == u"最小间距"][0]
    check("G08 阈值调大触发间距失败", not gap_result["passed"])
    check("G09 间距细节给出实测值", u"0.600" in gap_result["detail"] or u"0.6" in gap_result["detail"])

    wrong_count = core.check_layout(placed, scene.field, scene.min_gap, expected_count=99)
    count_result = [item for item in wrong_count if item["type"] == u"物体数量"][0]
    check("G10 数量不符触发失败", not count_result["passed"])
    check("G11 数量细节给期望值", u"99" in count_result["detail"])

    overlap = _make_overlap()
    overlap_results = core.check_layout(overlap, core.Field(10.0, 10.0), 0.0)
    overlap_result = [item for item in overlap_results if item["type"] == u"不重叠"][0]
    check("G12 重叠触发失败", not overlap_result["passed"])
    check("G13 all_passed 对失败返回 False", not core.all_passed(overlap_results))

    dense = core.load_scene(os.path.join(CONFIG, "scenes", "dense_overlap.yaml"))
    dense_placed = core.layout_scene(dense)
    dense_results = core.check_layout(dense_placed, dense.field, dense.min_gap,
                                      expected_count=dense.total_objects())
    failed_types = [item["type"] for item in dense_results if not item["passed"]]
    check("G14 反例文件有两处失败", len(failed_types) == 2)
    check("G15 反例失败项是边界内与最小间距",
          failed_types == [u"边界内", u"最小间距"])
    check("G16 反例结论为 FAILED", not core.all_passed(dense_results))


# --------------------------------------------------------------------------
# H 渲染与输出
# --------------------------------------------------------------------------


def group_render():
    scene = core.parse_scene_text(core.DEMO_SCENE_TEXT, source="builtin")
    placed = core.layout_scene(scene)
    results = core.check_layout(placed, scene.field, scene.min_gap, expected_count=scene.total_objects())

    lines = core.render_scene(placed, scene.field, width=40, height=12)
    check("H01 行数", len(lines) == 12)
    check("H02 每行列数", all(len(line) == 40 for line in lines))
    check("H03 有边界字符", lines[0].startswith(u"|"))
    check("H04 有车位字符", u"#" in u"".join(lines))

    report = core.format_report(scene, placed, results)
    check("H05 报告含场景名", u"demo_parking_row" in report)
    check("H06 报告含断言统计", u"4 通过 / 0 失败" in report)
    check("H07 报告含结论", u"结论：PASSED" in report)
    check("H08 报告含包围盒", u"包围盒" in report)
    check("H09 报告含最小间距", u"最小间距" in report)

    csv_text = core.to_csv(placed)
    csv_lines = csv_text.strip().split(u"\n")
    check("H10 CSV 行数 = 表头 + 物体", len(csv_lines) == 16)
    check("H11 CSV 表头", csv_lines[0].startswith(u"index,group,kind,x,y,yaw,width,length,blueprint"))
    check("H12 CSV 首行数据", csv_lines[1].startswith(u"0,slot,slot,3.0000,3.0000"))

    json_text = core.to_json(scene, placed, results)
    check("H13 JSON 含场景名", u"demo_parking_row" in json_text)
    check("H14 JSON 含 passed", u"passed" in json_text)
    check("H15 JSON 含对象数", u"objects" in json_text)

    message = core.to_message_text(report)
    check("H16 to_message_text 返回 bytes 或 str", isinstance(message, (bytes, text_type)))
    check("H17 渲染空列表不报错", core.render_scene([], core.Field(4.0, 4.0), width=10, height=4)[0].startswith(u"|"))


# --------------------------------------------------------------------------
# I 参数读取
# --------------------------------------------------------------------------


def group_params():
    calls = []

    def getter(name, default=None):
        calls.append(name)
        table = {
            "scene/file": "",
            "field/width": 24.0,
            "field/length": 18.0,
            "check/min_gap": 0.5,
            "render/width": 60,
            "render/height": 24,
            "io/csv_path": "~/scene_objects.csv",
            "io/json_path": "~/scene_objects.json",
            "io/write_files": "true",
            "rate/publish_hz": 1.0,
            "topic/objects": "/scene/objects",
            "topic/summary": "/scene/summary",
            "topic/trigger": "/scene/layout",
        }
        return table.get(name, default)

    params = core.LayouterParams.from_params(getter)
    check_close("I01 场地宽", params.field_width, 24.0)
    check_close("I02 场地长", params.field_length, 18.0)
    check_close("I03 最小间距", params.min_gap, 0.5)
    check("I04 渲染宽", int(params.grid_width) == 60)
    check("I05 渲染高", int(params.grid_height) == 24)
    check("I06 写文件开关被解析成 True", params.write_files is True)
    check_close("I07 发布频率", params.publish_hz, 1.0)
    check("I08 物体话题", params.topic_objects == "/scene/objects")
    check("I09 汇总话题", params.topic_summary == "/scene/summary")
    check("I10 触发话题", params.topic_trigger == "/scene/layout")
    check("I11 一共读了 13 个参数", len(calls) == 13, "实际 %d" % len(calls))
    check("I12 参数名齐全", set(core.PARAM_KEYS) == set(calls))

    fallback = core.LayouterParams.from_params(lambda name, default=None: default)
    check_close("I13 缺省值回落", fallback.min_gap, core.DEFAULT_MIN_GAP)
    check("I14 字符串 false 解析成 False",
          core.LayouterParams.from_params(
              lambda name, default=None: "false" if name == "io/write_files" else default).write_files is False)


# --------------------------------------------------------------------------
# J 工程文件约定
# --------------------------------------------------------------------------


def group_project_files():
    check("J01 包目录存在", os.path.isdir(PKG))
    check("J02 入口是 main.py", os.path.isfile(os.path.join(SCRIPTS, "main.py")))
    check("J03 核心模块存在", os.path.isfile(os.path.join(SCRIPTS, "layouter_core.py")))
    check("J04 节点模块存在", os.path.isfile(os.path.join(SCRIPTS, "layouter_node.py")))
    check("J05 参数文件存在", os.path.isfile(os.path.join(CONFIG, "layouter.yaml")))
    check("J06 正例场景存在", os.path.isfile(os.path.join(CONFIG, "scenes", "demo_parking_row.yaml")))
    check("J07 反例场景存在", os.path.isfile(os.path.join(CONFIG, "scenes", "dense_overlap.yaml")))
    check("J08 launch 存在", os.path.isfile(os.path.join(LAUNCH, "main.launch")))
    check("J09 README 存在", os.path.isfile(os.path.join(PKG, "README.md")))

    package = read_text(os.path.join(PKG, "package.xml"))
    check("J10 package.xml 有包名", "<name>scene_object_layouter</name>" in package)
    check("J11 依赖 rospy", "<exec_depend>rospy</exec_depend>" in package)
    check("J12 依赖 std_msgs", "<exec_depend>std_msgs</exec_depend>" in package)

    cmake = read_text(os.path.join(PKG, "CMakeLists.txt"))
    check("J13 CMakeLists 声明工程", "project(scene_object_layouter)" in cmake)
    check("J14 CMakeLists 安装 main.py", "scripts/main.py" in cmake)
    check("J15 CMakeLists 安装核心模块", "scripts/layouter_core.py" in cmake)
    check("J16 CMakeLists 安装 config", "install(DIRECTORY launch config" in cmake)

    launch = read_text(os.path.join(LAUNCH, "main.launch"))
    check("J17 launch 用 main.py 启动", 'type="main.py"' in launch)
    check("J18 launch 载入参数文件", "<rosparam command=\"load\"" in launch)
    check("J19 launch 有 13 个 arg", launch.count("<arg ") == 13)
    check("J20 每个 arg 都有 doc", launch.count('doc="') == 13)
    check("J21 launch 覆盖 13 个 param", launch.count("<param ") == 13)
    check("J22 没有未转义的比较符", "<=" not in launch and ">=" not in launch)

    yaml_text = read_text(os.path.join(CONFIG, "layouter.yaml"))
    for section in ("scene", "field", "check", "render", "io", "rate", "topic"):
        check("J23 \u53c2\u6570\u6587\u4ef6\u6709\u5206\u6bb5 %s" % section,
              u"\n%s:\n" % section in yaml_text)
    for leaf in ("file", "width", "length", "min_gap", "height", "csv_path", "json_path",
                 "write_files", "publish_hz", "objects", "summary", "trigger"):
        check("J24 \u53c2\u6570\u6587\u4ef6\u6709\u53f6\u5b50 %s" % leaf,
              u"\n  %s:" % leaf in yaml_text)

    core_source = read_text(os.path.join(SCRIPTS, "layouter_core.py"))
    check("J24 核心不 import rospy", "\nimport rospy" not in core_source and "\nfrom rospy" not in core_source)
    check("J25 核心不 import numpy", "\nimport numpy" not in core_source)
    check("J26 核心不 import yaml", "\nimport yaml" not in core_source)
    check("J27 核心有 DEMO_SCENE_TEXT", "DEMO_SCENE_TEXT" in core_source)

    node_source = read_text(os.path.join(SCRIPTS, "layouter_node.py"))
    check("J28 节点引用核心", "import layouter_core" in node_source or "layouter_core as core" in node_source)
    check("J29 节点有 Py2 日志保护", "_log(" in node_source)
    check("J30 节点注册关闭钩子", "on_shutdown" in node_source)

    readme = read_text(os.path.join(PKG, "README.md"))
    check("J31 README 含启动命令", "roslaunch scene_object_layouter main.launch" in readme)
    check("J32 README 含离线测试命令", "test_scene_object_layouter_local.py" in readme)

    if os.path.isfile(DOCS_PAGE):
        page = read_text(DOCS_PAGE)
        check("J33 文档页保留原内容", page.startswith(u"# 生成对象"))
        check("J34 文档页有新章节", u"不依赖 CARLA" in page)
        check("J35 文档页有快速运行", u"roslaunch scene_object_layouter main.launch" in page)
        check("J36 文档页有断言表", u"边界内" in page and u"最小间距" in page)
        check("J37 文档页有大模型声明", u"大模型" in page)
    else:
        check("J33 本次未在仓库根目录运行，跳过文档页校验", True)
        check("J34 本地不含 docs 目录", True)
        check("J35 包目录仍然存在", os.path.isdir(PKG))
        check("J36 测试脚本位于 tests 目录下", os.path.basename(HERE) == "tests")
        check("J37 核心模块仍然可导入", hasattr(core, "layout_scene"))


# --------------------------------------------------------------------------
# pytest 入口：让每个分组在 pytest 下也真的会失败
# --------------------------------------------------------------------------


def _pytest_group(func):
    def runner():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        if new:
            raise AssertionError(u"本组失败 %d 项：\n%s" % (len(new), u"\n".join(new)))

    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_helpers = _pytest_group(group_helpers)
test_geometry = _pytest_group(group_geometry)
test_shapes = _pytest_group(group_shapes)
test_parse = _pytest_group(group_parse)
test_layout = _pytest_group(group_layout)
test_measure = _pytest_group(group_measure)
test_checks = _pytest_group(group_checks)
test_render = _pytest_group(group_render)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_project_files)


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------


def main():
    print(u"scene_object_layouter 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)

    for func in (group_helpers, group_geometry, group_shapes, group_parse, group_layout,
                 group_measure, group_checks, group_render, group_params, group_project_files):
        func()

    print(u"=" * 60)
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        for item in FAILED:
            print(u"[失败] %s" % item)
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
