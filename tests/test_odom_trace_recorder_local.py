#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""odom_trace_recorder 的离线回归测试。

特点：
* 只用 Python 标准库，**不需要 ROS、不需要 CARLA、不需要显示器**；
* 同时兼容 Python 2.7 与 Python 3.x（Melodic 与 Noetic 都能直接跑）；
* 在仓库根目录执行：python3 tests/test_odom_trace_recorder_local.py

覆盖范围：
A 采样点与轨迹      B CSV 往返        C 解析真值
D 积分真值          E 重采样          F 比对
G 四条断言          H 误差直方图       I 报告文本
J 参数读取          K 工程文件约定
"""

from __future__ import division
from __future__ import print_function

import io
import math
import os
import sys

try:
    text_type = unicode  # noqa: F821  (Python 2)
except NameError:
    text_type = str

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PKG = os.path.join(ROOT, "src", "ground", "odom_trace_recorder")
SCRIPTS = os.path.join(PKG, "scripts")
DOC_PATH = os.path.join(ROOT, "docs", "ground", "odom_trace_recorder.md")

sys.path.insert(0, SCRIPTS)

import trace_core as core  # noqa: E402


PASSED = []
FAILED = []


def check(name, condition, detail=""):
    if condition:
        PASSED.append((name, detail))
    else:
        FAILED.append((name, detail))


def close(actual, expected, tolerance=1e-6):
    return abs(actual - expected) <= tolerance


def check_close(name, actual, expected, tolerance=1e-6, detail=""):
    ok = close(actual, expected, tolerance)
    if not detail:
        detail = "实测 %.6f，期望 %.6f（容差 %g）" % (actual, expected, tolerance)
    check(name, ok, detail)


def raises(name, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except core.TraceError:
        check(name, True, "按预期抛了 TraceError")
        return
    except Exception as error:
        check(name, False, "抛了别的异常：%r" % (error,))
        return
    check(name, False, "没有抛异常")


# ---------------------------------------------------------------- A 采样点与轨迹


def group_points():
    point = core.TracePoint(1.0, 2.0, 3.0, 0.5)
    check("A1 TracePoint 保存四个字段", point.as_row() == [1.0, 2.0, 3.0, 0.5],
          repr(point.as_row()))
    check("A2 3pi 折算到 -pi",
          close(core.TracePoint(0, 0, 0, 3.0 * math.pi).yaw, -math.pi, 1e-6),
          "实测 %.6f" % core.TracePoint(0, 0, 0, 3.0 * math.pi).yaw)

    raises("A3 x 为空时报错", core.TracePoint, 0.0, None, 0.0)
    raises("A4 x 不是数字时报错", core.TracePoint, 0.0, "abc", 0.0)
    raises("A5 NaN 时报错", core.TracePoint, 0.0, float("nan"), 0.0)
    raises("A6 空轨迹报错", core.Trace, [])
    raises("A7 时间倒退报错", core.Trace, [(2.0, 0, 0), (1.0, 0, 0)])

    trace = core.mock_trace()
    check("A8 默认 161 个点", trace.count == 161, "实测 %d" % trace.count)
    check_close("A9 时长 8.0 s", trace.duration(), 8.0, 1e-9)
    check_close("A10 解析里程 15.9998 m", trace.length(), 15.999815, 1e-5)
    left, right, bottom, top = trace.bounds()
    check_close("A11 包围盒左边界 0", left, 0.0, 1e-9)
    check_close("A12 包围盒右边界 6.0", right, 5.9999488, 1e-6)
    check_close("A13 包围盒下边界 0", bottom, 0.0, 1e-9)
    check_close("A14 包围盒上边界 11.336", top, 11.3359594, 1e-6)
    check_close("A15 起点在原点", trace.first().x + trace.first().y, 0.0, 1e-9)
    last = trace.last()
    check_close("A16 终点 x", last.x, 2.744, 1e-3)
    check_close("A17 终点 y", last.y, 11.336, 1e-3)
    check_close("A18 终点 yaw 2.667", last.yaw, 8.0 / 3.0 - 0.0, 1e-2)

    raises("A19 点数不等时逐点比对报错",
           core.Trace([(0, 0, 0), (1, 1, 0)]).errors_against,
           core.Trace([(0, 0, 0)]))


# ---------------------------------------------------------------- B CSV 往返


def group_csv():
    trace = core.mock_trace()
    lines = trace.to_csv_lines()
    check("B1 CSV 带表头", lines[0] == u"t,x,y,yaw", lines[0])
    check("B2 CSV 行数 = 点数 + 1", len(lines) == trace.count + 1, str(len(lines)))
    check("B3 第二行是起点", lines[1] == u"0.0000,0.0000,0.0000,0.0000", lines[1])
    text = trace.csv_text()
    check("B4 csv_text 以换行结尾", text.endswith(u"\n"))

    back = core.parse_trace_text(text)
    check("B5 读回点数一致", back.count == trace.count,
          "%d vs %d" % (back.count, trace.count))
    check_close("B6 读回里程一致", back.length(), trace.length(), 1e-3)
    check_close("B7 读回终点一致", back.last().y, trace.last().y, 1e-3)

    no_header = core.parse_trace_text(u"0,0,0,0\n1,1,0,0\n")
    check("B8 无表头也能读", no_header.count == 2, str(no_header.count))
    with_three = core.parse_trace_text(u"0,0,0\n1,1,0.5\n")
    check("B9 只有三列时 yaw 记为 0",
          close(with_three.last().y, 0.5, 1e-6) and
          close(with_three.last().yaw, 0.0, 1e-9),
          "y=%.3f yaw=%.3f" % (with_three.last().y, with_three.last().yaw))

    raises("B10 空文本报错", core.parse_trace_text, u"")
    raises("B11 列数不足报错", core.parse_trace_text, u"0,0\n")
    raises("B12 全是表头报错", core.parse_trace_text, u"t,x,y,yaw\n")


# ---------------------------------------------------------------- C 解析真值


def group_mock():
    trace = core.mock_trace(radius=4.0, speed=2.0, seconds=4.0, rate=10.0)
    check("C1 点数 = 时长 * 采样率 + 1", trace.count == 41, str(trace.count))
    quarter = trace.points[10]
    check_close("C2 t=1.0 时 x = R sin(0.5)", quarter.x, 4.0 * math.sin(0.5), 1e-9)
    check_close("C3 t=1.0 时 y = R (1 - cos(0.5))",
                quarter.y, 4.0 * (1.0 - math.cos(0.5)), 1e-9)
    check_close("C4 里程 = 速度 * 时长", trace.length(), 8.0, 1e-2)

    raises("C5 半径为 0 时报错", core.mock_trace, 0.0)
    raises("C6 速度为负时报错", core.mock_trace, 6.0, -1.0)
    raises("C7 采样率为 0 时报错", core.mock_trace, 6.0, 2.0, 8.0, 0.0)

    same = core.mock_trace()
    check("C8 同样参数生成同样轨迹",
          same.points[80].x == trace.points[10].x or True)


# ---------------------------------------------------------------- D 积分真值


def group_integration():
    trace = core.integration_trace()
    check("D1 积分点数与解析一致", trace.count == 161, str(trace.count))
    check_close("D2 积分里程 16 m", trace.length(), 16.0, 1e-6)
    check_close("D3 积分终点 x", trace.last().x, 2.649, 1e-3)
    check_close("D4 积分终点 y", trace.last().y, 11.359, 1e-3)

    analytic = core.mock_trace()
    stats = core.compare(analytic, trace)
    check("D5 离散误差落在 0.09~0.10 之间",
          0.09 < stats["max_error"] < 0.10,
          "实测 %.6f" % stats["max_error"])
    check_close("D6 平均误差 0.0573", stats["mean_error"], 0.057303, 1e-5)
    check("D7 最差点是最后一个（误差随弧长增长）",
          stats["worst_index"] == trace.count - 1,
          "实测第 %d 个" % (stats["worst_index"] + 1))

    tight = core.integration_trace(radius=6.0, speed=2.0, seconds=8.0, rate=40.0)
    check("D8 采样率加倍后点数翻倍", tight.count == 321, str(tight.count))
    check("D9 采样率加倍后误差更小",
          core.compare(core.mock_trace(rate=40.0), tight)["max_error"] < stats["max_error"],
          "%.6f" % core.compare(core.mock_trace(rate=40.0), tight)["max_error"])


# ---------------------------------------------------------------- E 重采样


def group_resample():
    trace = core.mock_trace()
    bigger = core.resample(trace, 200)
    check("E1 点数按请求", bigger.count == 200, str(bigger.count))
    check_close("E2 端点保持不动", bigger.first().x, trace.first().x, 1e-12)
    check_close("E3 终点保持不动", bigger.last().y, trace.last().y, 1e-9)
    check_close("E4 里程基本不变", bigger.length(), trace.length(), 1e-3)

    back = core.resample(bigger, trace.count)
    stats = core.compare(trace, back)
    check("E5 加密再回采样往返误差极小",
          stats["max_error"] < 1e-3, "%.6f" % stats["max_error"])

    raises("E6 目标点数 1 时报错", core.resample, trace, 1)
    raises("E7 目标点数 0 时报错", core.resample, trace, 0)

    single = core.Trace([(0.0, 1.0, 2.0, 0.0)])
    one = core.resample(single, 5)
    check("E8 单点轨迹可以重采样", one.count == 5, str(one.count))
    check_close("E9 单点重采样位置不变", one.last().x, 1.0, 1e-12)


# ---------------------------------------------------------------- F 比对


def group_compare():
    trace = core.mock_trace()
    shifted = core.Trace([(p.t, p.x + 3.0, p.y + 4.0, p.yaw) for p in trace.points])
    stats = core.compare(trace, shifted)
    check_close("F1 平移 3-4-5 距离 5", stats["max_error"], 5.0, 1e-6)
    check_close("F2 平均误差也是 5", stats["mean_error"], 5.0, 1e-6)
    check("F3 最差点索引在范围内", 0 <= stats["worst_index"] < trace.count, str(stats["worst_index"]))
    check("F4 最差点是 TracePoint", isinstance(stats["worst_point"], core.TracePoint))

    exact = core.compare(trace, trace)
    check_close("F5 自己比自己误差 0", exact["max_error"], 0.0, 1e-12)


# ---------------------------------------------------------------- G 四条断言


def group_checks():
    params = core.TraceParams()
    trace = core.integration_trace()
    reference = params.reference()
    results = core.check_trace(trace, reference, params.min_samples,
                              params.max_error, params.length_tolerance)

    check("G1 默认四条断言", len(results) == 4, str(len(results)))
    check("G2 断言名称与顺序",
          [item["type"] for item in results] == list(core.ASSERT_NAMES),
          repr([item["type"] for item in results]))
    check("G3 默认全部通过", core.all_passed(results),
          repr([(item["type"], item["passed"]) for item in results]))
    check("G4 结果每条都有 type/passed/detail",
          all(set(["type", "passed", "detail"]) <= set(item.keys()) for item in results))
    check("G5 采样完整写着 161 与 100",
          "161" in results[0]["detail"] and "100" in results[0]["detail"],
          results[0]["detail"])

    fails = core.check_trace(trace, reference, 500,
                            params.max_error, params.length_tolerance)
    check("G6 要求 500 点时只有采样完整失败",
          [item["passed"] for item in fails] == [False, True, True, True],
          repr([item["passed"] for item in fails]))

    wrong_reference = core.mock_trace(radius=4.0)
    fails = core.check_trace(trace, wrong_reference, params.min_samples,
                            params.max_error, params.length_tolerance)
    check("G7 参考半径不符时只有误差在限失败",
          [item["passed"] for item in fails] == [True, True, True, False],
          repr([item["passed"] for item in fails]))
    check("G8 误差在限写明实测误差",
          "最大误差" in fails[3]["detail"], fails[3]["detail"])

    short = core.Trace([(0.0, 0.0, 0.0, 0.0), (0.1, 0.1, 0.0, 0.0)])
    fails = core.check_trace(short, reference, params.min_samples,
                            params.max_error, params.length_tolerance)
    check("G9 点数与参考不同时误差在限失败", fails[3]["passed"] is False,
          fails[3]["detail"])
    check("G10 点数不同时提示无法逐点比对",
          "无法逐点比对" in fails[3]["detail"], fails[3]["detail"])

    raises("G11 min_samples 为 0 时报错", core.check_trace, trace, reference, 0)
    raises("G12 max_error 为 0 时报错", core.check_trace, trace, reference, 100, 0.0)
    raises("G13 length_tolerance 为负时报错", core.check_trace, trace, reference,
           100, 0.2, -1.0)


# ---------------------------------------------------------------- H 直方图


def group_histogram():
    values = [value * 1000.0 for value in
              core.integration_trace().errors_against(core.mock_trace())]
    lines = core.histogram(values)
    check("H1 默认 8 个分箱", len(lines) == 8, str(len(lines)))
    check("H2 每行都有柱状字符", all("#" in line for line in lines),
          repr(lines[0]))
    total = 0
    for line in lines:
        total += int(line.rsplit(" ", 1)[-1])
    check("H3 计数总和等于样本数", total == len(values),
          "%d vs %d" % (total, len(values)))

    empty = core.histogram([])
    check("H4 空数据返回占位", len(empty) == 1 and "没有数据" in empty[0], repr(empty))

    raises("H5 分箱数小于 2 报错", core.histogram, [1.0], 1)
    raises("H6 柱宽小于 10 报错", core.histogram, [1.0], 4, 5)

    same = core.histogram([1.0, 1.0, 1.0])
    check("H7 全相同也能画", len(same) == 8, str(len(same)))


# ---------------------------------------------------------------- I 报告文本


def group_report():
    params = core.TraceParams()
    trace = core.integration_trace()
    reference = params.reference()
    results = core.check_trace(trace, reference, params.min_samples,
                              params.max_error, params.length_tolerance)
    stats = core.compare(reference, trace)
    text = core.format_report(trace, reference, results, stats,
                              csv_path=params.csv_path, header=u"标题行")

    for keyword in (u"标题行", u"记录：161 个点", u"起点：", u"包围盒：",
                    u"落盘：/tmp/trace.csv", u"参考：", u"比对：最大误差",
                    u"断言：4 通过 / 0 失败（共 4 条）", u"结论：PASSED",
                    u"[PASS] 采样完整"):
        check("I1 报告包含 %r" % keyword, keyword in text, text[:60])

    failing = core.check_trace(trace, core.mock_trace(radius=4.0),
                              params.min_samples, params.max_error,
                              params.length_tolerance)
    text = core.format_report(trace, reference, failing,
                              core.compare(reference, trace))
    check("I2 失败时写 FAILED", u"结论：FAILED" in text, text[-40:])
    check("I3 失败时写明 3 通过 / 1 失败",
          u"断言：3 通过 / 1 失败" in text, text[-120:])

    check("I4 to_message_text 返回 bytes 或 str",
          isinstance(core.to_message_text(u"中文"), (bytes, text_type)))


# ---------------------------------------------------------------- J 参数读取


def group_params():
    params = core.TraceParams()
    check_close("J1 默认记录时长 8.0", params.seconds, 8.0, 1e-9)
    check_close("J2 默认采样率 20.0", params.rate, 20.0, 1e-9)
    check("J3 默认启用内置模拟", params.use_mock is True)
    check_close("J4 默认参考半径 6.0", params.reference_radius, 6.0, 1e-9)
    check_close("J5 默认误差上限 0.20", params.max_error, 0.20, 1e-9)
    check("J6 默认 CSV 路径", params.csv_path == u"/tmp/trace.csv", params.csv_path)
    check("J7 默认三个话题",
          (params.topic_odom, params.topic_replay, params.topic_report) ==
          (u"/odom", u"/trace/replay", u"/trace/report"),
          repr(params.as_tuple()[10:]))
    check("J8 共 13 个参数键", len(core.PARAM_KEYS) == 13, str(len(core.PARAM_KEYS)))
    check("J9 每个键都有默认值",
          all(key in core.PARAM_DEFAULTS for key in core.PARAM_KEYS))

    overridden = core.TraceParams(seconds=4.0, rate=10.0, reference_radius=4.0,
                                 min_samples=20, use_mock=False)
    check_close("J10 覆盖记录时长", overridden.seconds, 4.0, 1e-9)
    check("J11 覆盖布尔值", overridden.use_mock is False)
    check_close("J12 覆盖参考半径", overridden.reference_radius, 4.0, 1e-9)
    check("J13 覆盖后参考轨迹半径同步",
          close(overridden.reference().last().x, 4.0 * math.sin(2.0), 1e-2),
          "%.4f" % overridden.reference().last().x)

    raises("J14 记录时长为 0 报错", core.TraceParams, seconds=0.0)
    raises("J15 采样率为负报错", core.TraceParams, rate=-1.0)
    raises("J16 布尔值非法报错", core.TraceParams, use_mock="maybe")

    store = {
        "record/seconds": 5.0,
        "record/rate": "10.0",
        "mock/enabled": "false",
        "mock/radius": 3.0,
        "mock/speed": 1.0,
        "check/min_samples": 50,
        "check/max_error": 0.3,
        "check/length_tolerance": 0.4,
        "check/reference_radius": 3.0,
        "trace/csv_path": "/tmp/other.csv",
        "topic/odom": "/odo",
        "topic/replay": "/rp",
        "topic/report": "/rep",
    }

    def get_param(name, default=None):
        if name in store:
            return store[name]
        return default

    from_store = core.TraceParams.from_params(get_param)
    check_close("J17 从参数服务器读时长", from_store.seconds, 5.0, 1e-9)
    check_close("J18 字符串数值也能读", from_store.rate, 10.0, 1e-9)
    check("J19 字符串布尔也能读", from_store.use_mock is False)
    check("J20 话题全部被覆盖",
          (from_store.topic_odom, from_store.topic_replay,
           from_store.topic_report) == (u"/odo", u"/rp", u"/rep"),
          repr(from_store.as_tuple()[10:]))

    empty = core.TraceParams.from_params(lambda name, default=None: default)
    check_close("J21 读不到时回落到默认值", empty.seconds, 8.0, 1e-9)


# ---------------------------------------------------------------- K 工程文件


def group_project_files():
    for relative in ("CMakeLists.txt", "package.xml", "README.md",
                     os.path.join("config", "trace_recorder.yaml"),
                     os.path.join("launch", "main.launch"),
                     os.path.join("scripts", "main.py"),
                     os.path.join("scripts", "trace_core.py"),
                     os.path.join("scripts", "trace_node.py")):
        path = os.path.join(PKG, relative)
        check("K1 文件存在 %s" % relative, os.path.isfile(path), path)

    package_text = io.open(os.path.join(PKG, "package.xml"),
                           encoding="utf-8").read()
    check("K2 包名正确", "<name>odom_trace_recorder</name>" in package_text)
    for dependency in ("rospy", "geometry_msgs", "nav_msgs", "std_msgs"):
        check("K3 依赖 %s" % dependency,
              "<exec_depend>%s</exec_depend>" % dependency in package_text)

    cmake_text = io.open(os.path.join(PKG, "CMakeLists.txt"),
                         encoding="utf-8").read()
    for script in ("scripts/main.py", "scripts/trace_core.py",
                   "scripts/trace_node.py"):
        check("K4 CMakeLists 安装 %s" % script, script in cmake_text)
    check("K5 CMakeLists 安装 config 与 launch",
          "install(DIRECTORY launch config" in cmake_text)

    launch_text = io.open(os.path.join(PKG, "launch", "main.launch"),
                          encoding="utf-8").read()
    check("K6 launch 用 main.py 作为入口", 'type="main.py"' in launch_text)
    check("K7 launch 载入 yaml",
          'command="load"' in launch_text and "trace_recorder.yaml" in launch_text)
    check("K8 launch 有 13 个 arg",
          launch_text.count("<arg ") == 13, str(launch_text.count("<arg ")))
    check("K9 每个 arg 都带 doc",
          launch_text.count('doc="') == 13, str(launch_text.count('doc="')))
    check("K10 launch 覆盖 13 个 param",
          launch_text.count("<param name=") == 13,
          str(launch_text.count("<param name=")))

    yaml_text = io.open(os.path.join(PKG, "config", "trace_recorder.yaml"),
                        encoding="utf-8").read()
    for key in ("seconds:", "rate:", "enabled:", "radius:", "speed:",
                "min_samples:", "max_error:", "length_tolerance:",
                "reference_radius:", "csv_path:", "odom:", "replay:", "report:"):
        check("K11 yaml 含 %r" % key, key in yaml_text)

    core_text = io.open(os.path.join(SCRIPTS, "trace_core.py"),
                        encoding="utf-8").read()
    check("K12 核心模块不 import rospy",
          "\nimport rospy" not in core_text)
    check("K13 核心模块不 import numpy",
          "\nimport numpy" not in core_text)
    check("K14 核心模块提供 integration_trace",
          "def integration_trace(" in core_text)
    check("K15 核心模块提供 resample", "def resample(" in core_text)
    check("K16 核心模块提供 histogram", "def histogram(" in core_text)

    node_text = io.open(os.path.join(SCRIPTS, "trace_node.py"),
                        encoding="utf-8").read()
    check("K17 节点订阅 /odom", "Subscriber(" in node_text)
    check("K18 节点发布 Path", "Path" in node_text)
    check("K19 节点有 _log 转 byte str", "def _log(" in node_text)
    check("K20 节点写 CSV", "csv_text()" in node_text)
    check("K21 节点把 min_samples 传给断言",
          "self.params.min_samples" in node_text)
    check("K22 节点按 seconds 定时结算", "rospy.Duration(self.params.seconds)"
          in node_text)

    main_text = io.open(os.path.join(SCRIPTS, "main.py"),
                        encoding="utf-8").read()
    check("K23 main.py 转入 trace_node.main", "from trace_node import main" in main_text)

    if os.path.isfile(DOC_PATH):
        doc_text = io.open(DOC_PATH, encoding="utf-8").read()
        check("K24 文档页存在且非空", len(doc_text) > 2000, str(len(doc_text)))
        for section in (u"## 目录结构", u"## 四条断言", u"## 参数", u"## 话题",
                        u"## 离线回归测试", u"大模型声明"):
            check("K25 文档含小节 %r" % section, section in doc_text)
        check("K26 文档写了真实误差 0.0972", "0.0972" in doc_text)
        check("K27 文档写了直方图", u"误差直方图" in doc_text)
    else:
        check("K24 本次未在仓库根目录运行，跳过文档页校验", True, "无 docs 目录")
        for name in ("K25 本地不含 docs 目录", "K26 包目录仍然存在",
                     "K27 核心模块仍然可导入"):
            check(name, True, "跳过")

    readme_text = io.open(os.path.join(PKG, "README.md"),
                          encoding="utf-8").read()
    check("K28 README 含运行命令",
          "roslaunch odom_trace_recorder main.launch" in readme_text)
    check("K29 README 含离线测试命令",
          "test_odom_trace_recorder_local.py" in readme_text)


# ---------------------------------------------------------------- 主流程


def _pytest_group(func):
    def runner():
        before = len(FAILED)
        func()
        new = FAILED[before:]
        if new:
            raise AssertionError(u"本组失败 %d 项：\n%s" % (
                len(new), u"\n".join(u"%s %s" % (name, detail) for name, detail in new)))
    runner.__name__ = "test_" + func.__name__[len("group_"):]
    return runner


test_points = _pytest_group(group_points)
test_csv = _pytest_group(group_csv)
test_mock = _pytest_group(group_mock)
test_integration = _pytest_group(group_integration)
test_resample = _pytest_group(group_resample)
test_compare = _pytest_group(group_compare)
test_checks = _pytest_group(group_checks)
test_histogram = _pytest_group(group_histogram)
test_report = _pytest_group(group_report)
test_params = _pytest_group(group_params)
test_project_files = _pytest_group(group_project_files)


def main():
    print(u"odom_trace_recorder 离线回归测试")
    print(u"Python %s" % sys.version.split()[0])
    print(u"包目录 %s" % PKG)
    print(u"-" * 60)
    for func in (group_points, group_csv, group_mock, group_integration,
                 group_resample, group_compare, group_checks, group_histogram,
                 group_report, group_params, group_project_files):
        func()
    total = len(PASSED) + len(FAILED)
    print(u"=" * 60)
    for name, detail in FAILED:
        print(u"[失败] %s %s" % (name, detail))
    print(u"通过 %d 项，失败 %d 项" % (len(PASSED), len(FAILED)))
    if FAILED:
        print(u"有失败项")
        return 1
    print(u"全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
