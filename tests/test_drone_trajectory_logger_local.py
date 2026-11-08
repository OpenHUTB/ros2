#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drone_trajectory_logger 的本地测试（不需要 ROS / 仿真器 / matplotlib）.

覆盖重点
--------
A. ``classify_state``：9 种状态、死区、参数化死区
B. ``PoseIntegrator``：固定步长积分、悬停不动、转向后前进方向正确
C. 指令序列：解析、默认值、未知指令报错、指令 -> Twist 数值
D. ``TrajectoryRecorder``：采样统计、真实时间戳时长、CSV / JSON 导出内容
E. ``publish_period``：频率 -> 周期，非法频率报错
F. 工程文件：launch 参数与文档参数表一致、节点用节点时钟打时间戳、纯逻辑不依赖 ROS

用法: python3 tests/test_drone_trajectory_logger_local.py
"""
from __future__ import annotations

import json
import math
import os
import re
import sys
import tempfile
import types

PASS, FAIL = [], []

# 本地控制台（尤其是 Windows 的 GBK 代码页）统一按 UTF-8 输出，避免中文乱码
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:  # pragma: no cover - Python 3.6 及以下
    pass


def check(cond, label):
    (PASS if cond else FAIL).append(label)
    print("  [%s] %s" % ("PASS" if cond else "FAIL", label))


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
PKG = os.path.join(ROOT, "src", "drone_trajectory_logger")
sys.path.insert(0, PKG)

from drone_trajectory_logger import flight_state as fs  # noqa: E402


# ============================================================= A. 状态识别
def test_classify_state():
    print("== A. classify_state 状态细分 ==")
    check(fs.classify_state(0.0, 0.0) == fs.STATE_HOVER, "静止 -> 悬停")
    check(fs.classify_state(1.0, 0.0) == fs.STATE_FORWARD, "只前进 -> 直线前进")
    check(fs.classify_state(-1.0, 0.0) == fs.STATE_BACKWARD, "只后退 -> 直线后退")
    check(fs.classify_state(0.0, 1.0) == fs.STATE_TURN_LEFT, "只左转 -> 原地左转")
    check(fs.classify_state(0.0, -1.0) == fs.STATE_TURN_RIGHT, "只右转 -> 原地右转")
    check(fs.classify_state(1.0, 1.0) == fs.STATE_FORWARD_LEFT, "前进 + 左转 -> 前进左转")
    check(fs.classify_state(1.0, -1.0) == fs.STATE_FORWARD_RIGHT, "前进 + 右转 -> 前进右转")
    check(fs.classify_state(-1.0, 1.0) == fs.STATE_BACKWARD_LEFT, "后退 + 左转 -> 后退左转")
    check(fs.classify_state(-1.0, -1.0) == fs.STATE_BACKWARD_RIGHT, "后退 + 右转 -> 后退右转")

    check(fs.classify_state(0.005, 0.005) == fs.STATE_HOVER, "死区内的小速度 -> 悬停")
    check(fs.classify_state(0.02, 0.0, linear_deadband=0.05) == fs.STATE_HOVER,
          "自定义死区生效（0.02 < 0.05）")
    check(len(set(fs.STATE_LABELS.values())) == 9, "共 9 个互不重复的状态名")
    check(fs.STATE_LABELS[(0, 0)] == fs.STATE_HOVER, "状态表把 (0,0) 映射为悬停")


# ============================================================= B. 位姿积分
def test_pose_integrator():
    print("== B. PoseIntegrator 位姿积分 ==")
    integrator = fs.PoseIntegrator(step_length=0.5, turn_step=0.2)
    for _ in range(4):
        integrator.step(1.0, 0.0)
    check(abs(integrator.x - 2.0) < 1e-9 and abs(integrator.y) < 1e-9,
          "前进 4 步、步长 0.5 -> x=2.0, y=0")

    integrator.reset()
    integrator.step(0.0, 0.0)
    check((integrator.x, integrator.y, integrator.theta) == (0.0, 0.0, 0.0),
          "悬停不改变位姿")

    integrator.reset()
    for _ in range(5):
        integrator.step(0.0, 1.0)
    check(abs(integrator.theta - 1.0) < 1e-9, "左转 5 步、每步 0.2 rad -> theta=1.0")

    integrator.step(1.0, 0.0)
    check(abs(integrator.x - 0.5 * math.cos(1.0)) < 1e-9
          and abs(integrator.y - 0.5 * math.sin(1.0)) < 1e-9,
          "转向后前进沿当前朝向分解")

    integrator.reset()
    for _ in range(20):
        integrator.step(0.0, -1.0)
    check(-math.pi <= integrator.theta <= math.pi, "连续旋转后 theta 归一化在 [-pi, pi]")

    integrator.reset()
    integrator.step(-1.0, 0.0)
    check(integrator.x < 0, "后退指令向 -x 方向移动")


# ============================================================= C. 指令解析
def test_commands():
    print("== C. 指令序列 ==")
    check(fs.parse_command_sequence(None) == list(fs.DEFAULT_SEQUENCE), "None -> 默认序列")
    check(fs.parse_command_sequence("  ") == list(fs.DEFAULT_SEQUENCE), "空白 -> 默认序列")
    check(fs.parse_command_sequence("forward, turn_left ,hover")
          == ["forward", "turn_left", "hover"], "逗号分隔 + 去空格 + 忽略空项")
    check(fs.parse_command_sequence("FORWARD") == ["forward"], "指令名大小写不敏感")

    try:
        fs.parse_command_sequence("forward,fly")
        check(False, "未知指令应抛 ValueError")
    except ValueError:
        check(True, "未知指令抛 ValueError")

    check(fs.command_to_twist_values("forward") == (1.0, 0.0), "forward -> (1.0, 0.0)")
    check(fs.command_to_twist_values("backward") == (-1.0, 0.0), "backward -> (-1.0, 0.0)")
    check(fs.command_to_twist_values("turn_left") == (0.0, 1.0), "turn_left -> (0, 1.0)")
    check(fs.command_to_twist_values("turn_right") == (0.0, -1.0), "turn_right -> (0, -1.0)")
    check(fs.command_to_twist_values("hover") == (0.0, 0.0), "hover -> (0, 0)")
    check(fs.command_to_twist_values("forward", linear_speed=2.5) == (2.5, 0.0),
          "线速度系数可配置")
    check(fs.command_to_twist_values("turn_left", angular_speed=0.4) == (0.0, 0.4),
          "角速度系数可配置")


# ============================================================= D. 记录导出
def test_recorder():
    print("== D. TrajectoryRecorder 记录与导出 ==")
    recorder = fs.TrajectoryRecorder(dt=1.0)
    recorder.add(fs.STATE_FORWARD, 0.0, 0.0, 0.0, 1.0, 0.0)
    recorder.add(fs.STATE_FORWARD, 0.5, 0.0, 0.0, 1.0, 0.0)
    recorder.add(fs.STATE_FORWARD, 1.0, 0.0, 0.0, 1.0, 0.0)
    recorder.add(fs.STATE_TURN_LEFT, 1.0, 0.0, 0.2, 0.0, 1.0)
    recorder.add(fs.STATE_HOVER, 1.0, 0.0, 0.2, 0.0, 0.0)

    check(len(recorder.samples) == 5, "记录了 5 个采样点")
    check(recorder.state_counts[fs.STATE_FORWARD] == 3, "直线前进计数为 3")
    check(abs(recorder.distance_m - 1.0) < 1e-9, "折线距离为 1.0 m（悬停/旋转不位移）")

    totals = recorder.state_totals()
    check(set(totals) == {fs.STATE_FORWARD, fs.STATE_TURN_LEFT, fs.STATE_HOVER}, "统计覆盖 3 种状态")
    check(abs(totals[fs.STATE_FORWARD]["duration_s"] - 3.0) < 1e-9, "前进时长 = 3 s")
    check(abs(totals[fs.STATE_TURN_LEFT]["duration_s"] - 1.0) < 1e-9, "左转时长 = 1 s")
    check(totals[fs.STATE_HOVER]["duration_s"] == 0.0, "最后一个采样点不计时长")
    check(abs(sum(item["duration_s"] for item in totals.values()) - 4.0) < 1e-9,
          "各状态时长之和 = 时间跨度")
    check(abs(sum(item["ratio"] for item in totals.values()) - 1.0) < 1e-6, "各状态占比之和为 1")

    summary = recorder.summary(params={"step_length": 0.5})
    check(summary["sample_count"] == 5 and summary["duration_s"] == 4.0,
          "汇总时长 = 首末时间戳差（4 s）")
    check(summary["params"] == {"step_length": 0.5}, "汇总里带上参数快照")
    check(summary["bounding_box"]["x_max"] == 1.0, "包围盒 x_max 正确")
    check(summary["final_pose"]["theta_rad"] == 0.2, "终点朝向正确")

    with tempfile.TemporaryDirectory() as tmp:
        paths = recorder.export(params={"step_length": 0.5}, output_dir=tmp)
        check(os.path.isfile(paths["csv"]) and os.path.isfile(paths["json"]),
              "CSV 与 JSON 均已生成")

        with open(paths["csv"], encoding="utf-8") as handle:
            lines = [line.rstrip("\n") for line in handle]
        check(lines[0].split(",") == fs.TrajectoryRecorder.CSV_HEADER, "CSV 表头正确")
        check(len(lines) == 6, "CSV 行数 = 表头 + 5 个采样点")
        check(lines[1].split(",")[4] == "0.000", "CSV 记录 theta（度）")
        check(lines[4].split(",")[5] == fs.STATE_TURN_LEFT, "CSV 记录中文状态名")

        with open(paths["json"], encoding="utf-8") as handle:
            data = json.load(handle)
        check(data["sample_count"] == 5 and data["distance_m"] == 1.0, "JSON 内容与记录一致")
        check(data["states"][fs.STATE_FORWARD]["count"] == 3, "JSON 状态统计正确")

    empty = fs.TrajectoryRecorder(dt=1.0)
    check(empty.summary()["sample_count"] == 0, "没有采样点时汇总不报错")
    check(empty.summary()["start_pose"] is None, "没有采样点时位姿为 None")


# ============================================================= D2. 真实时间戳
def test_timestamps():
    print("== D2. 真实时间戳（发布频率变化不影响时长）==")
    recorder = fs.TrajectoryRecorder(dt=1.0)
    recorder.add(fs.STATE_FORWARD, 0.0, 0.0, 0.0, 1.0, 0.0, timestamp=10.0)
    recorder.add(fs.STATE_FORWARD, 0.5, 0.0, 0.0, 1.0, 0.0, timestamp=10.1)
    recorder.add(fs.STATE_TURN_LEFT, 1.0, 0.0, 0.2, 0.0, 1.0, timestamp=10.2)
    recorder.add(fs.STATE_HOVER, 1.0, 0.0, 0.2, 0.0, 0.0, timestamp=10.3)

    totals = recorder.state_totals()
    check(abs(totals[fs.STATE_FORWARD]["duration_s"] - 0.2) < 1e-9,
          "10 Hz 采样：前进时长 0.2 s（不再按 1 s/点计算）")
    check(abs(totals[fs.STATE_TURN_LEFT]["duration_s"] - 0.1) < 1e-9, "左转时长 0.1 s")
    summary = recorder.summary()
    check(abs(summary["duration_s"] - 0.3) < 1e-9, "总时长 = 10.3 - 10.0 = 0.3 s")

    with tempfile.TemporaryDirectory() as tmp:
        paths = recorder.export(output_dir=tmp)
        with open(paths["csv"], encoding="utf-8") as handle:
            lines = [line.rstrip("\n") for line in handle]
        check(lines[1].split(",")[1] == "10.000", "CSV 记录真实时间戳 10.000")
        check(lines[4].split(",")[1] == "10.300", "CSV 记录真实时间戳 10.300")


# ============================================================= E. 发布周期
def test_publish_period():
    print("== E. publish_period 频率 -> 周期 ==")
    # main.py 顶层 import rclpy / geometry_msgs，这里装最小桩以便离线导入
    rclpy_mod = types.ModuleType("rclpy")
    node_mod = types.ModuleType("rclpy.node")

    class _Node(object):
        def __init__(self, *args, **kwargs):
            pass

    node_mod.Node = _Node
    rclpy_mod.node = node_mod
    geometry_mod = types.ModuleType("geometry_msgs")
    msg_mod = types.ModuleType("geometry_msgs.msg")

    class _Twist(object):
        pass

    msg_mod.Twist = _Twist
    geometry_mod.msg = msg_mod
    saved = {name: sys.modules.get(name) for name in
             ("rclpy", "rclpy.node", "geometry_msgs", "geometry_msgs.msg")}
    sys.modules.update({"rclpy": rclpy_mod, "rclpy.node": node_mod,
                        "geometry_msgs": geometry_mod, "geometry_msgs.msg": msg_mod})
    try:
        from drone_trajectory_logger import main as pub
        check(pub.publish_period(1.0) == 1.0, "1 Hz -> 1 s")
        check(abs(pub.publish_period(10.0) - 0.1) < 1e-12, "10 Hz -> 0.1 s")
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            try:
                pub.publish_period(bad)
                check(False, "非法 publish_rate 应报错: %r" % (bad,))
            except ValueError:
                check(True, "非法 publish_rate 报 ValueError: %r" % (bad,))
    finally:
        for name, module in saved.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module


# ============================================================= F. 工程文件
def test_project_files():
    print("== F. 工程文件与文档一致性 ==")
    with open(os.path.join(PKG, "package.xml"), encoding="utf-8") as handle:
        package_xml = handle.read()
    for dep in ("rclpy", "geometry_msgs", "std_msgs", "matplotlib"):
        check("<depend>%s</depend>" % dep in package_xml, "package.xml 依赖 %s" % dep)

    with open(os.path.join(PKG, "setup.py"), encoding="utf-8") as handle:
        setup_py = handle.read()
    check("'main = drone_trajectory_logger.main:main'" in setup_py, "setup.py 暴露 main 入口")
    check("'analyzer = drone_trajectory_logger.trajectory_analyzer:main'" in setup_py,
          "setup.py 暴露 analyzer 入口")

    with open(os.path.join(PKG, "launch", "main.launch.py"), encoding="utf-8") as handle:
        launch = handle.read()
    declared = set(re.findall(r"DeclareLaunchArgument\(\s*'([a-z0-9_]+)'", launch))
    check(len(declared) >= 14, "launch 声明了完整的参数集（%d 个）" % len(declared))

    docs_path = os.path.join(ROOT, "docs", "air", "drone_trajectory_logger.md")
    with open(docs_path, encoding="utf-8") as handle:
        docs = handle.read()
    table = docs.split("## 参数说明", 1)[1].split("## ", 1)[0]
    documented = set()
    for line in table.splitlines():
        match = re.match(r"\| `([a-z0-9_]+)`(?: / `([a-z0-9_]+)`)?", line)
        if match:
            documented.update(item for item in match.groups() if item)
    check(declared == documented, "launch 参数与文档参数表完全一致（差集：%s）"
          % sorted(declared ^ documented))
    check("sample_dt" not in declared and "sample_dt" not in documented,
          "不再暴露 sample_dt（记录改用真实时间戳）")

    with open(os.path.join(PKG, "drone_trajectory_logger", "trajectory_analyzer.py"),
              encoding="utf-8") as handle:
        analyzer_source = handle.read()
    check("get_clock().now()" in analyzer_source, "识别节点使用节点时钟")
    check("timestamp=self._now()" in analyzer_source, "采样点带真实时间戳")
    check("except Exception as exc" in analyzer_source, "绘图初始化失败会降级为无绘图")

    with open(os.path.join(PKG, "drone_trajectory_logger", "flight_state.py"),
              encoding="utf-8") as handle:
        pure_source = handle.read()
    check("import rclpy" not in pure_source, "flight_state 不依赖 rclpy（可在无 ROS 环境测试）")


def main():
    test_classify_state()
    test_pose_integrator()
    test_commands()
    test_recorder()
    test_timestamps()
    test_publish_period()
    test_project_files()
    print("\n==================== 结果 ====================")
    print("通过 %d 项，失败 %d 项" % (len(PASS), len(FAIL)))
    if FAIL:
        for item in FAIL:
            print("  FAIL: %s" % item)
        return 1
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
